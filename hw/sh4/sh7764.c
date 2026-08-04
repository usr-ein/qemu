/*
 * Renesas SH7764 (SH-4A) system on chip
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This models the subset of the SH7764 that is needed to bring a system up
 * from reset: the SH-4 cache/MMU control registers, the bus state controller,
 * the pin function controller, the watchdog and the DMA controller, plus the
 * two SCIF serial ports.
 *
 * The register layout was recovered empirically rather than from the Renesas
 * hardware manual, by executing a real SH7764 boot ROM under an instrumented
 * SH-4A interpreter and recording every peripheral access. Where a window's
 * identity is confirmed it is modelled properly; where it is not, it is a
 * plain register bank that reads back what was written. That is sufficient
 * because firmware bring-up code does not read values it did not write, with
 * the single exception of the DMA transfer-end bit, which is modelled for
 * real below.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/sh4/sh7764.h"
#include "hw/net/sh7764_eth.h"
#include "hw/sh4/sh.h"
#include "hw/misc/unimp.h"
#include "hw/usb/hcd-sh7764-usb.h"
#include "hw/sh4/sh_intc.h"
#include "hw/timer/tmu012.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/system.h"
#include "target/sh4/cpu.h"
#include "hw/core/irq.h"
#include "trace.h"

/*
 * Interrupt sources, named for sh_intc. Declared up here because peripherals
 * further down raise them by index.
 */
enum {
    UNUSED = 0,
    /* TMU */
    TUNI0, TUNI1, TUNI2, TICPI2,
    /* SCIF0 and SCIF2 */
    SCIF0_ERI, SCIF0_RXI, SCIF0_BRI, SCIF0_TXI,
    SCIF2_ERI, SCIF2_RXI, SCIF2_BRI, SCIF2_TXI,
    /* misc */
    WDT_ITI,
    ATAPI_ATAI,
    SSI_ADMA0, SSI_BDMA1,
    SSI_ACH0, SSI_BCH3,
    ETHERC,
    USBI,
    DMINT1,
    /* groups */
    SCIF0, SCIF2, SSI_B,
    NR_INTC_SOURCES,
};

/*
 * CHCR.TS selects the transfer unit. The boot ROM programs TS = 3 and
 * computes DMATCR as (length >> 4), which pins that encoding to 16 bytes;
 * the smaller encodings follow the usual SH-4 progression. Anything we have
 * not seen in the wild falls back to 16 bytes rather than transferring a
 * wrong length silently.
 */
static const unsigned sh7764_ts_unit[8] = { 1, 2, 4, 16, 32, 16, 16, 16 };

/*
 * Software hands the DMAC addresses in the P1/P2 windows (the boot ROM uses
 * 0xa0040000 for flash and 0xa7a00000 for DRAM). The engine itself works on
 * physical addresses, so strip the region bits the same way the CPU would.
 */
static hwaddr sh7764_dma_addr(uint32_t addr)
{
    if (addr >= 0x80000000 && addr < 0xe0000000) {
        return addr & 0x1fffffff;
    }
    return addr;
}

/* Step for one unit under an SM/DM address mode; 11 is prohibited. */
static int sh7764_dma_step(uint32_t chcr, int shift, unsigned unit)
{
    switch ((chcr >> shift) & SH7764_CHCR_AM_MASK) {
    case SH7764_CHCR_AM_INC:
        return unit;
    case SH7764_CHCR_AM_DEC:
        return -(int)unit;
    default:
        return 0;
    }
}

static void sh7764_dma_update_irq(SH7764State *s);
static bool sh7764_panel_channel(SH7764State *s, int ch, bool tx);
static void sh7764_panel_deliver(SH7764State *s);
static void sh7764_panel_send(SH7764State *s, int ch, uint32_t count);

static void sh7764_dma_run(SH7764State *s, int ch)
{
    uint32_t chcr = s->chcr[ch];
    unsigned unit, ts, rs;
    int sstep, dstep;
    uint32_t count;
    hwaddr src, dst;
    uint8_t buf[32];

    if (!(chcr & SH7764_CHCR_DE) || !(s->dmaor & SH7764_DMAOR_DME)) {
        return;
    }

    ts = (chcr >> SH7764_CHCR_TS_SHIFT) & SH7764_CHCR_TS_MASK;
    if (chcr & SH7764_CHCR_TS2) {
        ts |= 4;
    }
    unit = sh7764_ts_unit[ts];
    /* TCR counts units, and zero means the full 16,777,216. */
    count = s->tcr[ch] ? s->tcr[ch] : 0x1000000;

    sstep = sh7764_dma_step(chcr, SH7764_CHCR_SM_SHIFT, unit);
    dstep = sh7764_dma_step(chcr, SH7764_CHCR_DM_SHIFT, unit);
    rs = (chcr >> SH7764_CHCR_RS_SHIFT) & SH7764_CHCR_RS_MASK;

    /*
     * A channel in peripheral-request mode moves one unit per request from
     * the module named in DMARS, so it can only run as fast as that module.
     * Draining it here is right when the module is the destination - the
     * transmit side of a SCIF takes everything it is given - but wrong when
     * it is the source, because the data has not arrived yet and a fixed
     * source address would read the same empty receive register over and
     * over. Leave those armed and let nothing happen rather than invent
     * bytes; the receive direction needs the module to drive the channel.
     */
    if (rs == SH7764_CHCR_RS_DMARS && sstep == 0) {
        if (sh7764_panel_channel(s, ch, false)) {
            /*
             * The panel's answers come in over a chardev rather than off a
             * wire, so arm the channel and let them be delivered as they
             * arrive; a frame that beat the arming is still waiting.
             */
            s->panel_rx_chan = ch;
            sh7764_panel_deliver(s);
        }
        return;
    }

    if (sh7764_panel_channel(s, ch, true)) {
        sh7764_panel_send(s, ch, count);
        return;
    }

    src = sh7764_dma_addr(s->sar[ch]);
    dst = sh7764_dma_addr(s->dar[ch]);
    trace_sh7764_dma_run(ch, src, dst, (uint64_t)count * unit, unit);

    /*
     * Real hardware would run this in the background and raise TE when it
     * finishes. Firmware only ever polls TE, so completing synchronously is
     * indistinguishable and avoids modelling bus arbitration.
     */
    while (count--) {
        address_space_read(&address_space_memory, src,
                           MEMTXATTRS_UNSPECIFIED, buf, unit);
        address_space_write(&address_space_memory, dst,
                            MEMTXATTRS_UNSPECIFIED, buf, unit);
        src += sstep;
        dst += dstep;
    }

    /*
     * SAR and DAR follow the address modes, and TCR holds what is left of the
     * count, which is nothing once the transfer has run to the end.
     */
    s->sar[ch] = src;
    s->dar[ch] = dst;
    s->tcr[ch] = 0;
    s->chcr[ch] |= SH7764_CHCR_TE;
}

/*
 * The DMAC's channel 1 interrupt. It is a level: the controller holds it while
 * the channel's transfer-end flag stands and the service routine drops it by
 * clearing that flag, which is what INT2B3 reports to the routine as well.
 */
static void sh7764_dma_update_irq(SH7764State *s)
{
    bool pending = (s->chcr[1] & SH7764_CHCR_TE) &&
                   (s->chcr[1] & SH7764_CHCR_IE);

    qemu_set_irq(s->intc.irqs[DMINT1], pending);
}

/* Channel for a register offset, or -1 if the offset is not a channel's. */
static int sh7764_dma_chan(hwaddr offset)
{
    if (offset >= SH7764_DMAC_CH0 &&
        offset < SH7764_DMAC_CH0 + SH7764_DMAC_CH_STRIDE * 4) {
        return (offset - SH7764_DMAC_CH0) / SH7764_DMAC_CH_STRIDE;
    }
    if (offset >= SH7764_DMAC_CH4 &&
        offset < SH7764_DMAC_CH4 + SH7764_DMAC_CH_STRIDE * 2) {
        return 4 + (offset - SH7764_DMAC_CH4) / SH7764_DMAC_CH_STRIDE;
    }
    return -1;
}

/* Likewise for the B-side reload registers, which channels 0 to 3 have. */
static int sh7764_dma_chan_b(hwaddr offset)
{
    if (offset >= SH7764_DMAC_CHB0 &&
        offset < SH7764_DMAC_CHB0 +
                 SH7764_DMAC_CHB_STRIDE * SH7764_DMAC_NCHAN_B) {
        return (offset - SH7764_DMAC_CHB0) / SH7764_DMAC_CHB_STRIDE;
    }
    return -1;
}

static uint64_t sh7764_dmac_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764State *s = opaque;
    int ch, chb, rs;

    if (offset == SH7764_DMAC_DMAOR) {
        trace_sh7764_dmac_read(offset, s->dmaor);
        return s->dmaor;
    }

    rs = (offset - SH7764_DMAC_DMARS) / 4;
    if (offset >= SH7764_DMAC_DMARS && rs < SH7764_DMAC_NDMARS) {
        trace_sh7764_dmac_read(offset, s->dmars[rs]);
        return s->dmars[rs];
    }

    ch = sh7764_dma_chan(offset);
    if (ch >= 0) {
        switch (offset % SH7764_DMAC_CH_STRIDE) {
        case SH7764_DMAC_SAR:
            return s->sar[ch];
        case SH7764_DMAC_DAR:
            return s->dar[ch];
        case SH7764_DMAC_TCR:
            return s->tcr[ch];
        case SH7764_DMAC_CHCR:
            return s->chcr[ch];
        }
    }

    chb = sh7764_dma_chan_b(offset);
    if (chb >= 0) {
        switch (offset % SH7764_DMAC_CHB_STRIDE) {
        case SH7764_DMAC_SAR:
            return s->sarb[chb];
        case SH7764_DMAC_DAR:
            return s->darb[chb];
        case SH7764_DMAC_TCR:
            return s->tcrb[chb];
        }
    }

    qemu_log_mask(LOG_UNIMP, "sh7764: DMAC read of unhandled offset 0x%"
                  HWADDR_PRIx "\n", offset);
    return 0;
}

static void sh7764_dmac_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    SH7764State *s = opaque;
    int ch, chb, rs;

    trace_sh7764_dmac_write(offset, value);
    if (offset == SH7764_DMAC_DMAOR) {
        s->dmaor = value;
        return;
    }

    rs = (offset - SH7764_DMAC_DMARS) / 4;
    if (offset >= SH7764_DMAC_DMARS && rs < SH7764_DMAC_NDMARS) {
        s->dmars[rs] = value;
        return;
    }

    ch = sh7764_dma_chan(offset);
    if (ch >= 0) {
        switch (offset % SH7764_DMAC_CH_STRIDE) {
        case SH7764_DMAC_SAR:
            s->sar[ch] = value;
            break;
        case SH7764_DMAC_DAR:
            s->dar[ch] = value;
            break;
        case SH7764_DMAC_TCR:
            /* A write to TCR lands in TCRB too; section 12.3.6. */
            s->tcr[ch] = value;
            if (ch < SH7764_DMAC_NCHAN_B) {
                s->tcrb[ch] = value;
            }
            break;
        case SH7764_DMAC_CHCR:
            s->chcr[ch] = value;
            sh7764_dma_run(s, ch);
            sh7764_dma_update_irq(s);
            break;
        }
        return;
    }

    chb = sh7764_dma_chan_b(offset);
    if (chb >= 0) {
        switch (offset % SH7764_DMAC_CHB_STRIDE) {
        case SH7764_DMAC_SAR:
            s->sarb[chb] = value;
            break;
        case SH7764_DMAC_DAR:
            s->darb[chb] = value;
            break;
        case SH7764_DMAC_TCR:
            s->tcrb[chb] = value;
            break;
        }
        return;
    }

    qemu_log_mask(LOG_UNIMP, "sh7764: DMAC write of unhandled offset 0x%"
                  HWADDR_PRIx " = 0x%" PRIx64 "\n", offset, value);
}

static const MemoryRegionOps sh7764_dmac_ops = {
    .read = sh7764_dmac_read,
    .write = sh7764_dmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ------------------------------------------------------------------ CCN */

static uint64_t sh7764_ccn_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764State *s = opaque;
    CPUSH4State *env = &s->cpu->env;

    switch (offset) {
    case SH7764_CCN_PTEH:
        return env->pteh;
    case SH7764_CCN_PTEL:
        return env->ptel;
    case SH7764_CCN_TTB:
        return env->ttb;
    case SH7764_CCN_TEA:
        return env->tea;
    case SH7764_CCN_MMUCR:
        return env->mmucr;
    case SH7764_CCN_TRA:
        return env->tra;
    case SH7764_CCN_EXPEVT:
        return env->expevt;
    case SH7764_CCN_INTEVT:
        trace_sh7764_ccn_read(offset, env->intevt);
        return env->intevt;
    case SH7764_CCN_PVR:
        return SUPERH_CPU_GET_CLASS(s->cpu)->pvr;
    case SH7764_CCN_PRR:
        return SUPERH_CPU_GET_CLASS(s->cpu)->prr;
    case SH7764_CCN_CCR:
        return s->ccr;
    default:
        qemu_log_mask(LOG_UNIMP, "sh7764: CCN read of unhandled offset 0x%"
                      HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void sh7764_ccn_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    SH7764State *s = opaque;
    CPUSH4State *env = &s->cpu->env;

    trace_sh7764_ccn_write(offset, value);
    switch (offset) {
    case SH7764_CCN_PTEH:
        env->pteh = value;
        break;
    case SH7764_CCN_PTEL:
        env->ptel = value;
        break;
    case SH7764_CCN_TTB:
        env->ttb = value;
        break;
    case SH7764_CCN_TEA:
        env->tea = value;
        break;
    case SH7764_CCN_MMUCR:
        env->mmucr = value;
        break;
    case SH7764_CCN_TRA:
        env->tra = value & 0x7ff;
        break;
    case SH7764_CCN_EXPEVT:
        env->expevt = value & 0x7ff;
        break;
    case SH7764_CCN_INTEVT:
        env->intevt = value & 0x7ff;
        break;
    case SH7764_CCN_CCR:
        /* Caches are not modelled; store so reads stay consistent. */
        s->ccr = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sh7764: CCN write of unhandled offset 0x%"
                      HWADDR_PRIx " = 0x%" PRIx64 "\n", offset, value);
        break;
    }
}

static const MemoryRegionOps sh7764_ccn_ops = {
    .read = sh7764_ccn_read,
    .write = sh7764_ccn_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ------------------------------------------------------------------ WDT */

/*
 * WTCNT and WTCSR are write-protected: a write must carry 0x5a00 or 0xa500
 * respectively in the upper half. We do not model the count-up or the reset
 * it would eventually cause - firmware kicks the watchdog often enough that a
 * timer would only ever be a source of spurious resets in an emulator.
 */
static uint64_t sh7764_wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764State *s = opaque;

    switch (offset) {
    case SH7764_WDT_WTCNT:
        return s->wtcnt;
    case SH7764_WDT_WTCSR:
        return s->wtcsr;
    case SH7764_WDT_WRCSR:
        return s->wrcsr;
    default:
        return 0;
    }
}

static void sh7764_wdt_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    SH7764State *s = opaque;
    uint32_t key = (value >> 16) & 0xffff;
    uint32_t data = value & 0xff;

    trace_sh7764_wdt_write(offset, value);

    switch (offset) {
    case SH7764_WDT_WTCNT:
        if (key != SH7764_WTCNT_KEY) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "sh7764: WTCNT write without 0x5a00 key (0x%" PRIx64
                          ")\n", value);
            return;
        }
        s->wtcnt = data;
        break;
    case SH7764_WDT_WTCSR:
        if (key != SH7764_WTCSR_KEY) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "sh7764: WTCSR write without 0xa500 key (0x%" PRIx64
                          ")\n", value);
            return;
        }
        s->wtcsr = data;
        break;
    case SH7764_WDT_WRCSR:
        s->wrcsr = data;
        break;
    }
}

static const MemoryRegionOps sh7764_wdt_ops = {
    .read = sh7764_wdt_read,
    .write = sh7764_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---------------------------------------------------------------- ATAPI */

/*
 * ATAPI. No optical drive is modelled - emulating the CDJ's CD mechanism is
 * out of scope - so the point here is to let the firmware conclude that the
 * bus is empty and move on.
 *
 * An ATA bus with no device attached floats high, so the task file registers
 * below 0x80 read back all-ones. That is what the firmware's status wait is
 * looking for: it counts 2000 consecutive 0xff reads and then reports
 * failure, which is its no-device path. Returning zero instead looks like a
 * device that is permanently not ready, and it retries forever.
 *
 * The controller's own registers from 0x80 up (ATAPI_CONTROL, ATAPI_STATUS,
 * interrupt enable, the timing and DMA registers) are a plain register file.
 */
#define SH7764_ATAPI_TASKFILE_END  0x80

/*
 * Task file registers, table 17.3. They belong to the drive rather than to
 * the controller, sit four bytes apart, and the manual requires longword
 * access.
 */
#define SH7764_ATAPI_TF_DATA       0x00
#define SH7764_ATAPI_TF_ERROR      0x04   /* features on write            */
#define SH7764_ATAPI_TF_INTREASON  0x08
#define SH7764_ATAPI_TF_BCLOW      0x10
#define SH7764_ATAPI_TF_BCHIGH     0x14
#define SH7764_ATAPI_TF_DEVICE     0x18
#define SH7764_ATAPI_TF_STATUS     0x1c   /* command on write             */
#define SH7764_ATAPI_TF_ALTSTATUS  0x38   /* device control on write      */

/* Device control, written through the alternate status offset. */
#define SH7764_ATA_DC_NIEN         0x02

/* Controller registers, table 17.4. */
#define SH7764_ATAPI_CONTROL       0x80
#define SH7764_ATAPI_STATUS        0x84
#define SH7764_ATAPI_INT_ENABLE    0x88

/* ATAPI_CONTROL, section 17.3.1. Bit 7 drives the drive's reset line. */
#define SH7764_ATAPI_CTL_RESET     (1u << 7)

/* ATA status register bits. */
#define SH7764_ATA_ST_ERR          0x01
#define SH7764_ATA_ST_DRQ          0x08
#define SH7764_ATA_ST_DSC          0x10
#define SH7764_ATA_ST_DF           0x20
#define SH7764_ATA_ST_DRDY         0x40
#define SH7764_ATA_ST_BSY          0x80

/* ATAPI interrupt reason: which phase the packet protocol is in. */
#define SH7764_ATA_IR_CD           0x01   /* a command packet, not data   */
#define SH7764_ATA_IR_IO           0x02   /* towards the host             */

/* Commands the probe issues. */
#define SH7764_ATA_CMD_PACKET      0xa0
#define SH7764_ATA_CMD_IDENTIFY_PK 0xa1

/* Sense the drive reports with no disc loaded. */
#define SH7764_ATAPI_SK_NOT_READY  0x02
#define SH7764_ATAPI_ASC_NO_MEDIUM 0x3a

/*
 * Which instruction touched the controller. Bring-up needs this constantly:
 * a register trace says what the driver did and this says where to read the
 * code that did it.
 */
static uint64_t sh7764_guest_pc(void)
{
    return current_cpu ? SUPERH_CPU(current_cpu)->env.pc : 0;
}

/* ATAPI_STATUS and ATAPI_INT_ENABLE, figures in section 17.3. */
#define SH7764_ATAPI_ST_DEVINT     (1u << 4)

/*
 * The module reports the drive's IDEINT in ATAPI_STATUS as DEVINT and, when
 * the matching enable bit is set, drives the ATAI interrupt from it. DEVINT
 * is not latched - the manual is explicit that the bit follows the line - so
 * the whole path is recomputed from the drive's INTRQ each time either end
 * changes.
 */
static void sh7764_atapi_update_irq(SH7764State *s)
{
    unsigned idx = (SH7764_ATAPI_INT_ENABLE - SH7764_ATAPI_TASKFILE_END) / 4;
    bool devint = s->atapi_intrq && !s->atapi_nien;
    uint32_t enable = idx < ARRAY_SIZE(s->atapi_ctl) ? s->atapi_ctl[idx] : 0;

    idx = (SH7764_ATAPI_STATUS - SH7764_ATAPI_TASKFILE_END) / 4;
    if (idx < ARRAY_SIZE(s->atapi_ctl)) {
        s->atapi_ctl[idx] = (s->atapi_ctl[idx] & ~SH7764_ATAPI_ST_DEVINT) |
                            (devint ? SH7764_ATAPI_ST_DEVINT : 0);
    }

    qemu_set_irq(s->intc.irqs[ATAPI_ATAI],
                 devint && (enable & SH7764_ATAPI_ST_DEVINT));
}

/*
 * Reset the drive, which the host does by pulsing IDERST through bit 7 of
 * ATAPI_CONTROL. Everything in flight is abandoned and the task file is left
 * holding the signature that tells the host what kind of device answered:
 * 0x14EB in the byte count registers is what makes this a packet device
 * rather than a disk.
 */
static void sh7764_atapi_device_reset(SH7764State *s)
{
    s->atapi_status = SH7764_ATA_ST_DRDY | SH7764_ATA_ST_DSC;
    s->atapi_error = 0x01;              /* diagnostics passed               */
    s->atapi_intreason = 0x01;          /* sector count 1, per the standard */
    s->atapi_bytecount = 0xeb14;
    s->atapi_pos = 0;
    s->atapi_len = 0;
    s->atapi_packet_pos = 0;
    s->atapi_want_packet = false;
    s->atapi_intrq = false;
    sh7764_atapi_update_irq(s);
}

static void sh7764_atapi_raise(SH7764State *s)
{
    s->atapi_intrq = true;
    sh7764_atapi_update_irq(s);
}

static void sh7764_atapi_put_string(uint8_t *dst, const char *src, size_t len)
{
    size_t i;

    /*
     * A task file string is a sequence of 16-bit words, each holding two
     * characters with the first in the high half, and is padded with spaces
     * rather than terminated.
     */
    for (i = 0; i < len; i++) {
        dst[i ^ 1] = *src ? *src++ : ' ';
    }
}

static void sh7764_atapi_identify(SH7764State *s)
{
    uint8_t *b = s->atapi_buf;

    memset(b, 0, sizeof(s->atapi_buf));

    /*
     * Word 0 for a packet device: bits 15:14 are 10 to say so, the device
     * type in bits 12:8 is 5 for a CD-ROM, bit 7 marks the medium removable
     * and bits 1:0 are zero for a twelve byte command packet.
     */
    stw_le_p(b + 0 * 2, 0x85c0);
    sh7764_atapi_put_string(b + 10 * 2, "CDJ2KNXS0001", 20);      /* serial  */
    sh7764_atapi_put_string(b + 23 * 2, "1.00", 8);               /* firmware*/
    sh7764_atapi_put_string(b + 27 * 2, "PIONEER DVD-RW  DVR-105", 40);
    stw_le_p(b + 49 * 2, 0x0300);   /* LBA and DMA supported                */
    stw_le_p(b + 53 * 2, 0x0006);   /* words 64-70 and 88 are valid         */
    stw_le_p(b + 63 * 2, 0x0007);   /* multiword DMA modes 0 to 2           */
    stw_le_p(b + 64 * 2, 0x0003);   /* PIO modes 3 and 4                    */

    s->atapi_pos = 0;
    s->atapi_len = sizeof(s->atapi_buf);
    s->atapi_bytecount = s->atapi_len;
    s->atapi_intreason = SH7764_ATA_IR_IO;
    s->atapi_status = SH7764_ATA_ST_DRDY | SH7764_ATA_ST_DSC |
                      SH7764_ATA_ST_DRQ;
    sh7764_atapi_raise(s);
}

/*
 * Answer a command packet. Nothing here reads a disc, because there is no
 * disc: every command that needs one is refused with the sense a drive
 * reports when its tray is empty, and the few that do not are accepted so
 * that the probe completes rather than retrying.
 */
static void sh7764_atapi_do_packet(SH7764State *s)
{
    uint8_t op = s->atapi_packet[0];

    s->atapi_pos = 0;
    s->atapi_len = 0;

    switch (op) {
    case 0x00:  /* TEST UNIT READY   */
    case 0x1b:  /* START STOP UNIT   */
    case 0x1e:  /* PREVENT ALLOW     */
    case 0x25:  /* READ CAPACITY     */
    case 0x43:  /* READ TOC          */
        s->atapi_error = SH7764_ATAPI_SK_NOT_READY << 4;
        s->atapi_status = SH7764_ATA_ST_DRDY | SH7764_ATA_ST_DSC |
                          SH7764_ATA_ST_ERR;
        break;

    case 0x03:
        /*
         * REQUEST SENSE. This driver never reads a data-in phase for a packet
         * command: across a whole run it reads the data register exactly 256
         * times, which is the single IDENTIFY block, and it never reads the
         * interrupt reason register at all. Offering it one therefore leaves
         * DRQ asserted for data nobody collects, and the driver pulses the
         * drive's reset line to get out of it. Complete without a data phase
         * and let the error bit carry the answer.
         */
        s->atapi_status = SH7764_ATA_ST_DRDY | SH7764_ATA_ST_DSC;
        break;

    case 0xe0:
        /*
         * A Pioneer vendor command, issued as
         * E0 08 3C 00 00 00 00 00 00 04 00 00, with no documentation
         * available. Neither answer is known to be right: completing it
         * successfully makes the driver repeat it forever, which reads as a
         * poll waiting for the mechanism to reach some state, and refusing it
         * makes the driver stop after asking for sense. Refusing at least
         * tells the same story as every other command that needs a disc, and
         * a loop that never ends is certainly not what the hardware does.
         *
         * Either way the driver repeats it, and since it reads nothing but
         * the status registers afterwards there is no result it could be
         * waiting on - it looks like a periodic poll of the mechanism, which
         * is what a player with an empty slot would do anyway.
         */
        s->atapi_error = SH7764_ATAPI_SK_NOT_READY << 4;
        s->atapi_status = SH7764_ATA_ST_DRDY | SH7764_ATA_ST_DSC |
                          SH7764_ATA_ST_ERR;
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "sh7764-atapi: packet command 0x%02x\n", op);
        s->atapi_error = SH7764_ATAPI_SK_NOT_READY << 4;
        s->atapi_status = SH7764_ATA_ST_DRDY | SH7764_ATA_ST_DSC |
                          SH7764_ATA_ST_ERR;
        break;
    }

    s->atapi_intreason = SH7764_ATA_IR_CD | SH7764_ATA_IR_IO;
    sh7764_atapi_raise(s);
}

static uint64_t sh7764_atapi_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764State *s = opaque;
    unsigned idx;
    uint64_t val;

    if (offset < SH7764_ATAPI_TASKFILE_END) {
        switch (offset) {
        case SH7764_ATAPI_TF_DATA:
            val = 0xffff;
            if (s->atapi_pos + 1 < s->atapi_len) {
                val = lduw_le_p(s->atapi_buf + s->atapi_pos);
            }
            s->atapi_pos += 2;
            if (s->atapi_pos >= s->atapi_len) {
                s->atapi_status = SH7764_ATA_ST_DRDY | SH7764_ATA_ST_DSC;
                s->atapi_intreason = SH7764_ATA_IR_CD | SH7764_ATA_IR_IO;
                sh7764_atapi_raise(s);
            }
            break;
        case SH7764_ATAPI_TF_ERROR:
            val = s->atapi_error;
            break;
        case SH7764_ATAPI_TF_INTREASON:
            val = s->atapi_intreason;
            break;
        case SH7764_ATAPI_TF_BCLOW:
            val = s->atapi_bytecount & 0xff;
            break;
        case SH7764_ATAPI_TF_BCHIGH:
            val = s->atapi_bytecount >> 8;
            break;
        case SH7764_ATAPI_TF_DEVICE:
            val = s->atapi_device;
            break;
        case SH7764_ATAPI_TF_STATUS:
            /*
             * Reading the status register clears the pending interrupt; the
             * alternate status at 0x38 exists precisely so that a driver can
             * look without acknowledging.
             */
            val = s->atapi_status;
            if (s->atapi_intrq) {
                s->atapi_intrq = false;
                sh7764_atapi_update_irq(s);
            }
            break;
        case SH7764_ATAPI_TF_ALTSTATUS:
            val = s->atapi_status;
            break;
        default:
            val = 0xffffffffu >> ((4 - size) * 8);
            break;
        }
    } else {
        idx = (offset - SH7764_ATAPI_TASKFILE_END) / 4;
        val = idx < ARRAY_SIZE(s->atapi_ctl) ? s->atapi_ctl[idx] : 0;
    }

    trace_sh7764_atapi_read(offset, size, val, sh7764_guest_pc());
    return val;
}

static void sh7764_atapi_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    SH7764State *s = opaque;
    unsigned idx;

    trace_sh7764_atapi_write(offset, size, value, sh7764_guest_pc());

    if (offset < SH7764_ATAPI_TASKFILE_END) {
        switch (offset) {
        case SH7764_ATAPI_TF_DATA:
            if (s->atapi_want_packet) {
                if (s->atapi_packet_pos + 1 < sizeof(s->atapi_packet)) {
                    stw_le_p(s->atapi_packet + s->atapi_packet_pos, value);
                }
                s->atapi_packet_pos += 2;
                if (s->atapi_packet_pos >= sizeof(s->atapi_packet)) {
                    s->atapi_want_packet = false;
                    sh7764_atapi_do_packet(s);
                }
            }
            return;
        case SH7764_ATAPI_TF_ERROR:
            s->atapi_features = value;
            return;
        case SH7764_ATAPI_TF_BCLOW:
            s->atapi_bytecount = (s->atapi_bytecount & 0xff00) | (value & 0xff);
            return;
        case SH7764_ATAPI_TF_BCHIGH:
            s->atapi_bytecount = (s->atapi_bytecount & 0x00ff) |
                                 ((value & 0xff) << 8);
            return;
        case SH7764_ATAPI_TF_DEVICE:
            s->atapi_device = value;
            return;
        case SH7764_ATAPI_TF_STATUS:
            s->atapi_error = 0;
            switch (value & 0xff) {
            case SH7764_ATA_CMD_IDENTIFY_PK:
                sh7764_atapi_identify(s);
                break;
            case SH7764_ATA_CMD_PACKET:
                /* Ask for the twelve byte command packet. */
                s->atapi_want_packet = true;
                s->atapi_packet_pos = 0;
                s->atapi_intreason = SH7764_ATA_IR_CD;
                s->atapi_status = SH7764_ATA_ST_DRDY | SH7764_ATA_ST_DSC |
                                  SH7764_ATA_ST_DRQ;
                break;
            default:
                qemu_log_mask(LOG_UNIMP, "sh7764-atapi: command 0x%02x\n",
                              (unsigned)(value & 0xff));
                s->atapi_error = 0x04;    /* aborted */
                s->atapi_status = SH7764_ATA_ST_DRDY | SH7764_ATA_ST_DSC |
                                  SH7764_ATA_ST_ERR;
                break;
            }
            return;
        }
        if (offset == SH7764_ATAPI_TF_ALTSTATUS) {
            s->atapi_nien = value & SH7764_ATA_DC_NIEN;
            sh7764_atapi_update_irq(s);
        }
        return; /* device control and the reserved holes */
    }
    idx = (offset - SH7764_ATAPI_TASKFILE_END) / 4;
    if (idx < ARRAY_SIZE(s->atapi_ctl)) {
        s->atapi_ctl[idx] = value;
    }
    if (offset == SH7764_ATAPI_INT_ENABLE) {
        sh7764_atapi_update_irq(s);
    }
    if (offset == SH7764_ATAPI_CONTROL) {
        bool asserted = value & SH7764_ATAPI_CTL_RESET;

        if (s->atapi_reset && !asserted) {
            sh7764_atapi_device_reset(s);
        }
        s->atapi_reset = asserted;
    }
}

static const MemoryRegionOps sh7764_atapi_ops = {
    .read = sh7764_atapi_read,
    .write = sh7764_atapi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ----------------------------------------------------------------- SDHI */

/*
 * SD host interface. The SH7764 manual lists 0xffe40000 as reserved, but the
 * CDJ firmware drives a TMIO/Renesas style SD controller there: the offsets it
 * touches - block count 0x0a, response 0x0c, SD_INFO1 0x1c, SD_INFO2 0x1e and
 * card options 0x28 - are exactly that layout, and the firmware's own symbol
 * for the block is "SDINFO1_INI".
 *
 * This is a stub, not a card. Its only job is to stop the firmware spinning:
 * its driver waits for status bits with "read, mask, compare equal" loops, so
 * the status words read back all-ones and every wait completes immediately.
 * The command response registers stay zero, so the driver finds no usable card
 * and moves on.
 */
#define SH7764_SDHI_INFO1       0x1c
#define SH7764_SDHI_INFO2       0x1e

static uint64_t sh7764_sdhi_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset) {
    case SH7764_SDHI_INFO1:
    case SH7764_SDHI_INFO2:
        return 0xffff;
    default:
        return 0;
    }
}

static void sh7764_sdhi_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    /* Nothing to do: there is no card behind this. */
}

/* ------------------------------------------------------------------ IIC */

/*
 * The I2C bus interface, section 16.3. Registers are one byte each, four
 * bytes apart.
 *
 * Two bits of ICMCR are not storage at all: FSCL and FSDA read back the level
 * on the two bus lines, and an idle I2C bus is held high by its pull-ups.
 * Unmodelled the register reads zero, which says both lines are pulled down -
 * a bus stuck busy - and the driver waits for it to go free before it will
 * start a transfer. That wait is where the main firmware sat: a thousand reads
 * of ICMCR and no way out of them.
 *
 * Nothing is attached to the bus here, so a transfer is answered the way real
 * hardware answers one addressed to nobody. The address goes out and comes
 * back unacknowledged, the driver sees MNR and gives up on that device rather
 * than waiting on it forever.
 */
#define SH7764_IIC_ICMCR        0x04
#define SH7764_IIC_ICMSR        0x0c

#define SH7764_ICMCR_ESG        (1u << 0)   /* generate a start condition */
#define SH7764_ICMCR_FSB        (1u << 1)   /* generate a stop condition  */
#define SH7764_ICMCR_OBPC       (1u << 4)   /* software drives the pins   */
#define SH7764_ICMCR_LINES      0x60        /* FSDA and FSCL, read as bus */

#define SH7764_ICMSR_MAT        (1u << 0)   /* address transmitted        */
#define SH7764_ICMSR_MST        (1u << 4)   /* stop transmitted           */
#define SH7764_ICMSR_MNR        (1u << 6)   /* no acknowledge came back   */

static uint64_t sh7764_iic_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764State *s = opaque;
    unsigned idx = offset / 4;
    uint8_t val;

    if (idx >= ARRAY_SIZE(s->iic_regs)) {
        return 0;
    }
    val = s->iic_regs[idx];

    /*
     * Unless software has taken the pins over, the two line bits show the bus
     * itself, and an idle bus reads high.
     */
    if (offset == SH7764_IIC_ICMCR && !(val & SH7764_ICMCR_OBPC)) {
        val |= SH7764_ICMCR_LINES;
    }
    trace_sh7764_bank_read("sh7764.iic", offset, val, size, sh7764_guest_pc());
    return val;
}

static void sh7764_iic_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    SH7764State *s = opaque;
    unsigned idx = offset / 4;

    if (idx >= ARRAY_SIZE(s->iic_regs)) {
        return;
    }
    trace_sh7764_bank_write("sh7764.iic", offset, value, size,
                            sh7764_guest_pc());

    if (offset == SH7764_IIC_ICMSR) {
        /* Table 16.2 note 3: a flag is cleared by writing zero over it. */
        s->iic_regs[idx] &= value;
        return;
    }

    s->iic_regs[idx] = value;

    if (offset == SH7764_IIC_ICMCR) {
        unsigned sr = SH7764_IIC_ICMSR / 4;

        if (value & SH7764_ICMCR_ESG) {
            /* The address goes out and nobody answers it. */
            s->iic_regs[sr] |= SH7764_ICMSR_MAT | SH7764_ICMSR_MNR;
            s->iic_regs[idx] &= ~SH7764_ICMCR_ESG;
        }
        if (value & SH7764_ICMCR_FSB) {
            s->iic_regs[sr] |= SH7764_ICMSR_MST;
            s->iic_regs[idx] &= ~SH7764_ICMCR_FSB;
        }
    }
}

static const MemoryRegionOps sh7764_iic_ops = {
    .read = sh7764_iic_read,
    .write = sh7764_iic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps sh7764_sdhi_ops = {
    .read = sh7764_sdhi_read,
    .write = sh7764_sdhi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* --------------------------------------------------------------- CPUOPM */

/*
 * CPU operation mode register (appendix A of the hardware manual). The only
 * bit that matters to us is INTMU: with it set, an accepted interrupt loads
 * its priority into SR.IMASK, which is what stops a handler being re-entered
 * by a source it has not acknowledged yet. It lives in the CPU rather than
 * this device, so the register just forwards to it.
 */
static uint64_t sh7764_cpuopm_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764State *s = opaque;

    return offset == 0 ? s->cpu->env.cpuopm : 0;
}

static void sh7764_cpuopm_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    SH7764State *s = opaque;

    if (offset == 0) {
        s->cpu->env.cpuopm = value;
    }
}

static const MemoryRegionOps sh7764_cpuopm_ops = {
    .read = sh7764_cpuopm_read,
    .write = sh7764_cpuopm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ------------------------------------------------------------ write watch */

/*
 * A write watch over ordinary memory, enabled with
 *
 *   -global sh7764.watch-base=ADDR -global sh7764.watch-size=N
 *
 * Firmware archaeology keeps reducing to "which instruction wrote this word",
 * and there is no way to ask that of a plain RAM region. This shadows the
 * range at higher priority, logs each write with the instruction that made
 * it, and writes through, so the guest sees no difference.
 */
static uint64_t sh7764_watch_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764State *s = opaque;

    return ldn_le_p((uint8_t *)s->watch_ram + offset, size);
}

static void sh7764_watch_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    SH7764State *s = opaque;

    qemu_log_mask(LOG_UNIMP,
                  "WATCH %08" PRIx64 " <- %0*" PRIx64 " size %u pc %08" PRIx64
                  "\n", (uint64_t)(s->watch_base + offset), size * 2, value,
                  size, sh7764_guest_pc());
    stn_le_p((uint8_t *)s->watch_ram + offset, size, value);
}

static const MemoryRegionOps sh7764_watch_ops = {
    .read = sh7764_watch_read,
    .write = sh7764_watch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* --------------------------------------------------------------- SSI link */

/*
 * The serial sound interface, which on this board is not sound: it is the
 * link to the GUI processor on the TFT assembly.
 *
 * Each SSI has its own small DMA controller, and the firmware uses one
 * direction of each of the two modules. SSI-A writes what arrives into
 * memory at 0xA4500000 - the address GuiCom_RcvTASK carries in its literal
 * pool - and SSI-B reads what is to be sent from 0xA4500800. Registers are
 * from the SSI_DMAC list in section 18: RDMA reads out of memory, so it is
 * the transmit direction, and WDMA writes into memory, so it is receive.
 *
 * The far end is the Blackfin's SPORT1, whose DMA channel 3 receives and 4
 * transmits. Both ends move whole buffers, so that is the unit here too.
 */
#define SH7764_SSI_RDMADR    0x1008   /* transmit source in memory          */
#define SH7764_SSI_RDMCNTR   0x1010   /* transmit length, in words          */
#define SH7764_SSI_WDMADR    0x1018   /* receive destination in memory      */
#define SH7764_SSI_WDMCNTR   0x1020   /* receive length, in words           */
#define SH7764_SSI_DMCOR     0x1028   /* bit 0 starts the transfer          */
#define SH7764_SSI_BLCNTSR   0x1040   /* block size, in bytes               */
#define SH7764_SSI_BLNCNTSR  0x1050   /* how many blocks                    */

#define SH7764_SSI_DMINTSR   0x1188   /* interrupt status                   */
#define SH7764_SSI_DMINTMR   0x1190   /* interrupt mask, 1 = masked         */

#define SH7764_SSI_DMCOR_EN     (1u << 0)   /* DMEN                         */
#define SH7764_SSI_DMCOR_RPTMD  (1u << 2)   /* repeat mode                  */

/* Channel 0's sources sit in the low five bits; table 18.3.14. */
#define SH7764_SSI_DMINT_RXFIFOEMP0 (1u << 0)
#define SH7764_SSI_DMINT_TXFIFOFUL0 (1u << 1)
#define SH7764_SSI_DMINT_DMEND0     (1u << 2)
#define SH7764_SSI_DMINT_BLKNEND0   (1u << 3)
#define SH7764_SSI_DMINT_BLKEND0    (1u << 4)
#define SH7764_SSI_DMINT_CH0        0x1f

/*
 * The audio channel itself, one register pair per channel: SSICR at 0x2000
 * and SSISR at 0x2004 within each module (sections 18.3.16 and 18.3.17).
 * These sit above the DMA controller's registers and are a separate interrupt
 * from it - SSICH0 at vector H'A20 and SSICH3 at H'AC0, against SSIDMA0 at
 * H'A00 and SSIDMA1 at H'AA0 - and the firmware installs a handler for each
 * of the four.
 *
 * The enable bits in SSICR line up with the status bits in SSISR at the same
 * positions, so an interrupt is pending exactly while the two agree.
 *
 * IIRQ, the idle flag, is the one that matters here. It comes out of reset
 * set, is read only - "writing 0 in this bit will not clear the interrupt" -
 * and simply reports that the channel has nothing in flight. Transfers in
 * this model complete within the store that starts them, so there is no
 * observable moment when the channel is busy and the flag stays set.
 *
 * That is the whole of what the GUI link needs, and what it was missing. The
 * firmware sets SendFlg when it arms a transmit, and only the idle interrupt
 * clears it again: the DMA-end interrupt enables IIEN, the channel interrupt
 * then fires and clears the flag. GuiCom_RcvTASK refuses to look at anything
 * that arrived while SendFlg stands, printing
 *
 *     GU受:受信動作条件フラグが不一致(SendFlg !=0)!!!
 *
 * and returning without calling the message processor. With no channel
 * interrupt the flag was set once and never cleared, so every reply the GUI
 * processor sent was discarded - which is what the panel reports as E-8709
 * COMMUNICATION ERROR. The receive direction is the same shape: its idle
 * interrupt is what re-arms the receive DMA for the next message.
 */
#define SH7764_SSI_CR        0x2000
#define SH7764_SSI_SR        0x2004

#define SH7764_SSI_SR_RESET  0x0210a003u   /* IIRQ set, reserved bits fixed */
#define SH7764_SSI_SR_W0C    0x0c000000u   /* UIRQ and OIRQ, write 0 to clear */
#define SH7764_SSI_IRQ_MASK  0x0f000000u   /* UIRQ, OIRQ, IIRQ, DIRQ        */

/*
 * Reset values from table 18.2. DMINTMR comes up with every source masked,
 * which matters because the firmware deliberately leaves RXFIFOEMP and
 * TXFIFOFUL masked and reads them out of the status register by hand.
 */
#define SH7764_SSI_DMINTSR_RESET 0x01010101u
#define SH7764_SSI_DMINTMR_RESET 0x1f1f1f1fu

/*
 * The transfer count registers are named for words and measured in bytes -
 * sections 18.3.3 and 18.3.5 say so outright, and their low three bits are
 * read-only because the count has to be a whole number of bursts. The block
 * registers agree: the GUI link is programmed with sixteen bytes a block and
 * three blocks to receive, which is the forty-eight it asks WDMCNT for, and
 * four blocks to send, which is the sixty-four in RDMCNT.
 *
 * Reading them as sixteen-bit words doubled every message. The panel takes
 * sixty-four bytes at a time, so a doubled send arrived as two messages and a
 * doubled receive swallowed two answers as one.
 */
#define SH7764_SSI_MAX_XFER  65536

static uint32_t sh7764_ssi_reg(SH7764RegBank *b, hwaddr off)
{
    unsigned idx = off / 4;

    return idx < b->nregs ? b->regs[idx] : 0;
}

static void sh7764_ssi_set(SH7764RegBank *b, hwaddr off, uint32_t val)
{
    unsigned idx = off / 4;

    if (idx < b->nregs) {
        b->regs[idx] = val;
    }
}

/*
 * Raise the channel's interrupt if it has a source the mask lets through.
 * Section 18.3.15: a mask bit set means masked, and every one of them comes
 * out of reset that way.
 */
static bool sh7764_ssi_pending(SH7764RegBank *b)
{
    uint32_t status = sh7764_ssi_reg(b, SH7764_SSI_DMINTSR);
    uint32_t mask = sh7764_ssi_reg(b, SH7764_SSI_DMINTMR);

    return (status & ~mask & SH7764_SSI_DMINT_CH0) != 0;
}

static void sh7764_ssi_update_irq(SH7764State *s, SH7764RegBank *b)
{
    int src = (b == &s->ssi_a) ? SSI_ADMA0 : SSI_BDMA1;

    qemu_set_irq(s->intc.irqs[src], sh7764_ssi_pending(b));
}

/* The channel's own interrupt: a status flag whose enable bit is also set. */
static bool sh7764_ssi_chan_pending(SH7764RegBank *b)
{
    uint32_t sr = sh7764_ssi_reg(b, SH7764_SSI_SR);
    uint32_t cr = sh7764_ssi_reg(b, SH7764_SSI_CR);

    return (sr & cr & SH7764_SSI_IRQ_MASK) != 0;
}

static void sh7764_ssi_chan_update_irq(SH7764State *s, SH7764RegBank *b)
{
    int src = (b == &s->ssi_a) ? SSI_ACH0 : SSI_BCH3;

    qemu_set_irq(s->intc.irqs[src], sh7764_ssi_chan_pending(b));
}

/* Reset state for one SSI module, shared by realize and device reset. */
static void sh7764_ssi_bank_reset(SH7764RegBank *b)
{
    sh7764_ssi_set(b, SH7764_SSI_DMINTSR, SH7764_SSI_DMINTSR_RESET);
    sh7764_ssi_set(b, SH7764_SSI_DMINTMR, SH7764_SSI_DMINTMR_RESET);
    sh7764_ssi_set(b, SH7764_SSI_SR, SH7764_SSI_SR_RESET);
    sh7764_ssi_set(b, SH7764_SSI_CR, 0);
    b->blocks_left = 0;
}

/*
 * INT2B4, the individual module interrupt register. Several peripherals share
 * one priority level, and this names which of them is actually asserting.
 *
 * The SSI driver's service routine reads it before anything else and returns
 * at once if its own bit is clear, so with the register absent - reading zero
 * - the routine never recognised the interrupt it had just been entered for.
 * The transfer went unserviced, nothing cleared the source, and the handler
 * was simply entered again: four hundred thousand times in half a minute,
 * while the receive channel it was supposed to re-arm stayed idle and the
 * conversation with the panel stopped after a single exchange.
 *
 * Four of the nine bits are filled in, one per handler the firmware installs:
 * the two DMA controllers and the two audio channels they feed. Bits 2, 3, 7
 * and 8 belong to SSI channels this board does not wire up.
 */
#define SH7764_INT2B4_SSIDMA0   (1u << 0)
#define SH7764_INT2B4_SSICH0    (1u << 1)
#define SH7764_INT2B4_SSIDMA1   (1u << 5)
#define SH7764_INT2B4_SSICH3    (1u << 6)

static uint64_t sh7764_int2b4_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764State *s = opaque;
    uint32_t val = 0;

    if (sh7764_ssi_pending(&s->ssi_a)) {
        val |= SH7764_INT2B4_SSIDMA0;
    }
    if (sh7764_ssi_pending(&s->ssi_b)) {
        val |= SH7764_INT2B4_SSIDMA1;
    }
    if (sh7764_ssi_chan_pending(&s->ssi_a)) {
        val |= SH7764_INT2B4_SSICH0;
    }
    if (sh7764_ssi_chan_pending(&s->ssi_b)) {
        val |= SH7764_INT2B4_SSICH3;
    }
    return val;
}

static void sh7764_int2b4_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    /* Read only. */
}

/*
 * INT2B3, the DMAC's detailed source register. One bit per channel, set while
 * that channel has finished and its transfer-end flag is still standing. The
 * firmware's DMA service routine at 0x0429abe0 reads this first and leaves
 * immediately unless the bit for the channel it cares about is set, so
 * without it the routine bails before it ever looks at the channel.
 */
static uint64_t sh7764_int2b3_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764State *s = opaque;
    uint32_t val = 0;
    int ch;

    for (ch = 0; ch < SH7764_DMAC_NCHAN; ch++) {
        if (s->chcr[ch] & SH7764_CHCR_TE) {
            val |= 1u << ch;
        }
    }
    return val;
}

static const MemoryRegionOps sh7764_int2b3_ops = {
    .read = sh7764_int2b3_read,
    .write = sh7764_int2b4_write,       /* read only, same as INT2B4 */
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps sh7764_int2b4_ops = {
    .read = sh7764_int2b4_read,
    .write = sh7764_int2b4_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/*
 * Finish a transfer the way section 18.3.6 says the hardware does: DMEND is
 * raised once the word count has been satisfied, and DMEN clears itself when
 * the count reaches zero outside repeat mode. Nothing polls any of this - the
 * firmware unmasks DMEND, BLKEND and BLKNEND and then waits - so without the
 * interrupt it arms the channel once, receives one message and never speaks
 * again, which leaves the panel holding surface descriptors whose buffers the
 * next command would have supplied.
 */
static void sh7764_ssi_done(SH7764State *s, SH7764RegBank *b)
{
    uint32_t dmcor = sh7764_ssi_reg(b, SH7764_SSI_DMCOR);

    uint32_t blksz = sh7764_ssi_reg(b, SH7764_SSI_BLCNTSR);
    uint32_t nblk = sh7764_ssi_reg(b, SH7764_SSI_BLNCNTSR);

    if (!(dmcor & SH7764_SSI_DMCOR_RPTMD)) {
        sh7764_ssi_set(b, SH7764_SSI_DMCOR, dmcor & ~SH7764_SSI_DMCOR_EN);
    }

    /*
     * The three completions do not all mean the same thing and the driver
     * counts on that. BLKEND is raised once per block - the byte count in
     * SSIBLCNTSR - BLKNEND once the block count in SSIBLNCNTSR is exhausted,
     * and DMEND once the whole word count is done. The service routine at
     * 0x04310d20 counts BLKENDs and, when DMEND arrives, checks the count
     * against the block count before deciding the transfer was clean; raising
     * all three together leaves that count at one, and the routine takes its
     * "short transfer" path every time.
     *
     * So hold the remaining blocks and hand them over one interrupt at a
     * time. The routine acknowledges by writing the status bit back clear,
     * which is where the next one is raised.
     */
    b->blocks_left = (blksz && nblk) ? nblk : 1;
    sh7764_ssi_set(b, SH7764_SSI_DMINTSR,
                   sh7764_ssi_reg(b, SH7764_SSI_DMINTSR) |
                   SH7764_SSI_DMINT_BLKEND0);
    if (b->blocks_left <= 1) {
        sh7764_ssi_set(b, SH7764_SSI_DMINTSR,
                       sh7764_ssi_reg(b, SH7764_SSI_DMINTSR) |
                       SH7764_SSI_DMINT_DMEND0 |
                       SH7764_SSI_DMINT_BLKNEND0);
    }
    sh7764_ssi_update_irq(s, b);
}

/*
 * Called when the driver acknowledges a completion by writing the status
 * register. If blocks are still outstanding the next BLKEND goes up, and the
 * last one brings BLKNEND and DMEND with it.
 */
static void sh7764_ssi_acked(SH7764State *s, SH7764RegBank *b)
{
    uint32_t sts = sh7764_ssi_reg(b, SH7764_SSI_DMINTSR);

    if (!b->blocks_left || (sts & SH7764_SSI_DMINT_BLKEND0)) {
        return;
    }
    if (--b->blocks_left == 0) {
        return;
    }

    sts |= SH7764_SSI_DMINT_BLKEND0;
    if (b->blocks_left == 1) {
        sts |= SH7764_SSI_DMINT_DMEND0 | SH7764_SSI_DMINT_BLKNEND0;
    }
    sh7764_ssi_set(b, SH7764_SSI_DMINTSR, sts);
    sh7764_ssi_update_irq(s, b);
}

/* Hand the transmit buffer to whatever is on the other end of the chardev. */
static void sh7764_ssi_tx(SH7764State *s, SH7764RegBank *b)
{
    uint32_t addr = sh7764_ssi_reg(b, SH7764_SSI_RDMADR);
    uint32_t words = sh7764_ssi_reg(b, SH7764_SSI_RDMCNTR);
    uint32_t len = words;
    g_autofree uint8_t *buf = NULL;

    if (!addr || !len || len > SH7764_SSI_MAX_XFER) {
        return;
    }

    buf = g_malloc(len);
    address_space_read(&address_space_memory, sh7764_dma_addr(addr),
                       MEMTXATTRS_UNSPECIFIED, buf, len);
    qemu_log_mask(LOG_UNIMP, "sh7764-ssi: sending %u bytes from 0x%08x\n",
                  len, addr);
    qemu_chr_fe_write_all(&s->gui_chr, buf, len);
    sh7764_ssi_done(s, b);
}

/*
 * Deliver what has arrived, once a receive transfer has been armed to take
 * it. Anything beyond one transfer's worth stays queued: the far end sends
 * whole messages and this end asks for them one at a time.
 */
/*
 * RXFIFOEMP0 is a level, not an event. Section 18.3.14 gives it an initial
 * value of one and says the hardware clears it by itself the moment the
 * buffer stops being empty, so it reports what is there rather than latching
 * what happened.
 *
 * The firmware reads it in the DMA-end handler, off the same snapshot it took
 * of the status register, to decide whether the message it has just been
 * given was the last one waiting. If it was, it stops the receive DMA, arms
 * the channel's idle interrupt and lets that re-arm the transfer; the loop
 * only continues because of this bit. Its interrupt is left masked - only the
 * value is wanted.
 */
static void sh7764_ssi_rx_level(SH7764State *s)
{
    SH7764RegBank *b = &s->ssi_a;
    uint32_t sts = sh7764_ssi_reg(b, SH7764_SSI_DMINTSR);

    if (s->ssi_rx_len) {
        sts &= ~SH7764_SSI_DMINT_RXFIFOEMP0;
    } else {
        sts |= SH7764_SSI_DMINT_RXFIFOEMP0;
    }
    sh7764_ssi_set(b, SH7764_SSI_DMINTSR, sts);
}

static void sh7764_ssi_rx_deliver(SH7764State *s)
{
    SH7764RegBank *b = &s->ssi_a;
    uint32_t addr = sh7764_ssi_reg(b, SH7764_SSI_WDMADR);
    uint32_t len = sh7764_ssi_reg(b, SH7764_SSI_WDMCNTR);

    if (!s->ssi_rx_armed || !addr || !len || len > sizeof(s->ssi_rx_buf)) {
        return;
    }
    if (s->ssi_rx_len < len) {
        return;
    }

    address_space_write(&address_space_memory, sh7764_dma_addr(addr),
                        MEMTXATTRS_UNSPECIFIED, s->ssi_rx_buf, len);
    s->ssi_rx_len -= len;
    memmove(s->ssi_rx_buf, s->ssi_rx_buf + len, s->ssi_rx_len);
    s->ssi_rx_armed = false;
    qemu_log_mask(LOG_UNIMP, "sh7764-ssi: delivered %u bytes to 0x%08x\n",
                  len, addr);
    /* Before the interrupt: the handler reads both in the same breath. */
    sh7764_ssi_rx_level(s);
    sh7764_ssi_done(s, b);
}

static int sh7764_ssi_can_receive(void *opaque)
{
    SH7764State *s = opaque;

    return sizeof(s->ssi_rx_buf) - s->ssi_rx_len;
}

static void sh7764_ssi_receive(void *opaque, const uint8_t *buf, int size)
{
    SH7764State *s = opaque;
    int room = sizeof(s->ssi_rx_buf) - s->ssi_rx_len;

    if (size > room) {
        size = room;
    }
    memcpy(s->ssi_rx_buf + s->ssi_rx_len, buf, size);
    s->ssi_rx_len += size;
    sh7764_ssi_rx_level(s);
    sh7764_ssi_rx_deliver(s);
}

/* Called from the bank write path when either SSI's DMA control is set. */
static void sh7764_ssi_start(SH7764State *s, SH7764RegBank *b, uint32_t value)
{
    if (!(value & SH7764_SSI_DMCOR_EN)) {
        return;
    }
    if (sh7764_ssi_reg(b, SH7764_SSI_RDMCNTR)) {
        sh7764_ssi_tx(s, b);
    }
    if (sh7764_ssi_reg(b, SH7764_SSI_WDMCNTR)) {
        s->ssi_rx_armed = true;
        sh7764_ssi_rx_deliver(s);
    }
}


/* ----------------------------------------------------------- panel link */

/*
 * The front panel's M16C.
 *
 * Every button, the jog wheel and the rotary encoder come in over SCIF2 in
 * fixed 24-byte frames, moved by a DMA channel each way: channel 0 out to
 * SCFTDR2 and channel 1 in from SCFRDR2, both selected through DMARS0. The
 * frame is 22 bytes of payload, a checksum byte and 0x8F; the firmware's
 * decoder at 0x042f3e76 rejects anything whose last byte is not 0x8F or whose
 * checksum - an eight-bit sum of the payload with end-around carry - does not
 * match, so a frame has to be well formed to be seen at all.
 *
 * The transfers are carried here rather than through the UART model because
 * the DMA is what moves them: a peripheral-request channel is paced by the
 * module it names, and nothing in the SCIF model can drive one. Bytes still
 * arrive and leave through a chardev, so the other end is whatever the user
 * connects - tools/cdjpanel.py, or anything else that speaks the frame.
 */

static int sh7764_panel_can_receive(void *opaque)
{
    SH7764State *s = opaque;

    return sizeof(s->panel_rx_buf) - s->panel_rx_len;
}

/*
 * The frame a panel with nothing touched on it sends.
 *
 * A button pulls its line down, so a released one reads as 1 and an idle
 * panel is all ones; the checksum of twenty-two 0xFF bytes folds back to 0xFF.
 * The M16C starts sending this the moment it is out of reset, long before the
 * main processor asks anything of it, so the firmware never sees a blank
 * panel - and it does read the panel early, to decide whether the operator is
 * holding the combination that asks for a firmware update. This model has to
 * do the same or that decision is made against a buffer of zeros, which is
 * every button at once.
 */
static void sh7764_panel_idle_frame(uint8_t *frame)
{
    unsigned i;
    uint8_t sum = 0;

    /*
     * Only the bytes the firmware reads as buttons are all ones. The unpacker
     * at 0x042f5834 takes bytes 4 to 11 as four sixteen-bit words of button
     * state and byte 15 as individual switches; the rest are counts and
     * fields, where 0xFF is a value rather than "not pressed", and bytes 0 and
     * 1 it does not read at all.
     */
    memset(frame, 0, SH7764_PANEL_FRAME);
    memset(frame + 4, 0xff, 8);
    frame[15] = 0xff;

    for (i = 0; i < SH7764_PANEL_FRAME - 2; i++) {
        unsigned t = sum + frame[i];

        sum = t > 0xff ? (t & 0xff) + 1 : t;    /* end-around carry */
    }
    frame[SH7764_PANEL_FRAME - 2] = sum;
    frame[SH7764_PANEL_FRAME - 1] = 0x8f;
}

/* Hand a complete frame to the armed channel, if both are ready. */
static void sh7764_panel_deliver(SH7764State *s)
{
    int ch = s->panel_rx_chan;
    uint32_t len;

    if (ch < 0 || !(s->chcr[ch] & SH7764_CHCR_DE)) {
        return;                         /* nothing armed; hold the frame */
    }

    len = s->tcr[ch] ? s->tcr[ch] : SH7764_PANEL_FRAME;
    if (s->panel_rx_len < len) {
        /*
         * Nothing whole has arrived from whatever is playing the panel, so
         * answer as the real one would - it is always sending - rather than
         * leave the buffer as it was found. Only when the buffer is empty,
         * though: padding a half-received frame out with made-up bytes would
         * lose the rest of the real one and leave the stream a frame out of
         * step from then on.
         *
         * What the real one would send is the state it is in, which is the
         * last frame it sent, not "nothing pressed". The firmware arms this
         * channel far faster than any panel produces frames - eight arms
         * inside ten milliseconds, against a panel sending at a hundred hertz
         * - so most arms find the buffer empty. Minting an idle frame for
         * each of them buried the real ones about four times in five, and a
         * button that is only seen in one frame out of five never survives
         * the firmware's debounce: a press reached memory and still nothing
         * happened. Repeating the last frame holds the state, which is what
         * the hardware does. Only before anything has ever arrived is there
         * nothing to repeat.
         */
        if (s->panel_rx_len != 0 || len > SH7764_PANEL_FRAME) {
            return;
        }
        if (s->panel_have_last) {
            memcpy(s->panel_rx_buf, s->panel_last, len);
        } else {
            sh7764_panel_idle_frame(s->panel_rx_buf);
        }
        s->panel_rx_len = len;
    }

    /*
     * The panel reports a state, not a stream: it sends the same frame over
     * and over and only the latest is worth having. If several piled up while
     * nothing was armed, take the newest and drop the rest, so the firmware
     * never works through a backlog of stale button positions.
     */
    if (s->panel_rx_len >= 2 * len) {
        uint32_t skip = (s->panel_rx_len / len - 1) * len;

        memmove(s->panel_rx_buf, s->panel_rx_buf + skip,
                s->panel_rx_len - skip);
        s->panel_rx_len -= skip;
    }

    if (len <= SH7764_PANEL_FRAME) {
        memcpy(s->panel_last, s->panel_rx_buf, len);
        s->panel_have_last = true;
    }
    address_space_write(&address_space_memory, sh7764_dma_addr(s->dar[ch]),
                        MEMTXATTRS_UNSPECIFIED, s->panel_rx_buf, len);
    trace_sh7764_dma_run(ch, 0, sh7764_dma_addr(s->dar[ch]), len, 1);

    s->panel_rx_len -= len;
    if (s->panel_rx_len) {
        memmove(s->panel_rx_buf, s->panel_rx_buf + len, s->panel_rx_len);
    }

    s->tcr[ch] = 0;
    s->dar[ch] += len;
    s->chcr[ch] = (s->chcr[ch] | SH7764_CHCR_TE) & ~SH7764_CHCR_DE;
    s->panel_rx_chan = -1;
    /*
     * The buffer is only a few frames deep, so it fills while nothing is
     * armed and the front end stops offering bytes. Draining it is not
     * enough on its own; the flow has to be restarted by hand.
     */
    qemu_chr_fe_accept_input(&s->panel_chr);
    sh7764_dma_update_irq(s);
}

static void sh7764_panel_receive(void *opaque, const uint8_t *buf, int size)
{
    SH7764State *s = opaque;

    if (size <= 0) {
        return;
    }
    if (s->panel_rx_len + size > sizeof(s->panel_rx_buf)) {
        size = sizeof(s->panel_rx_buf) - s->panel_rx_len;
    }
    memcpy(s->panel_rx_buf + s->panel_rx_len, buf, size);
    s->panel_rx_len += size;
    sh7764_panel_deliver(s);
}

/* Send a frame the firmware has handed to the transmit channel. */
static void sh7764_panel_send(SH7764State *s, int ch, uint32_t count)
{
    uint8_t buf[SH7764_PANEL_FRAME];
    uint32_t len = MIN(count, sizeof(buf));

    address_space_read(&address_space_memory, sh7764_dma_addr(s->sar[ch]),
                       MEMTXATTRS_UNSPECIFIED, buf, len);
    trace_sh7764_dma_run(ch, sh7764_dma_addr(s->sar[ch]), 0, len, 1);
    qemu_chr_fe_write_all(&s->panel_chr, buf, len);

    s->sar[ch] += len;
    s->tcr[ch] = 0;
    s->chcr[ch] = (s->chcr[ch] | SH7764_CHCR_TE) & ~SH7764_CHCR_DE;
    sh7764_dma_update_irq(s);
}

/* True if this channel is the panel link, in the given direction. */
static bool sh7764_panel_channel(SH7764State *s, int ch, bool tx)
{
    return tx ? s->dar[ch] == SH7764_SCIF2_SCFTDR
              : s->sar[ch] == SH7764_SCIF2_SCFRDR;
}

/* ------------------------------------------------- generic register bank */

/*
 * Port C bit 2 is an output, not an input.
 *
 * The bank reads back whatever software wrote, which is what section 27.2.13
 * specifies for a general output port, and the firmware drives this bit as
 * one: it clears it with a read-modify-write at 0x0429ab14 and sets it again
 * at 0x0429ad06. GuiCom_SndTASK reads it at 0x04259e2e and declares the GUI
 * processor usable only while it is *clear* - the sense is the opposite of
 * what the earlier note here assumed. Holding it high therefore contradicted
 * the firmware's own writes and stalled the link: the task spun for its full
 * 10,000-tick timeout, took the failure path at 0x04259ecc, and left the
 * ready flag at 0x049853fc zero, which is the flag the SSI interrupt handler
 * tests before it wakes GuiCom_RcvTASK. PTDAT_C resets to H'0000, so the
 * plain bank gives the right answer on its own.
 */

static uint64_t sh7764_bank_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764RegBank *b = opaque;
    unsigned idx = offset / 4;
    uint32_t val;

    if (idx >= b->nregs) {
        return 0;
    }
    val = b->regs[idx];
    trace_sh7764_bank_read(b->name, offset, val, size, sh7764_guest_pc());
    return val;
}

static void sh7764_bank_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    SH7764RegBank *b = opaque;
    unsigned idx = offset / 4;

    if (idx >= b->nregs) {
        return;
    }
    trace_sh7764_bank_write(b->name, offset, value, size, sh7764_guest_pc());

    if (b == &b->soc->ssi_a || b == &b->soc->ssi_b) {
        switch (offset) {
        case SH7764_SSI_DMINTSR:
            /* Writing one clears a source; writing zero is ignored. */
            b->regs[idx] &= ~(uint32_t)value;
            if (b == &b->soc->ssi_a) {
                sh7764_ssi_rx_level(b->soc);
            }
            sh7764_ssi_update_irq(b->soc, b);
            sh7764_ssi_acked(b->soc, b);
            return;
        case SH7764_SSI_CR:
            b->regs[idx] = value;
            sh7764_ssi_chan_update_irq(b->soc, b);
            return;
        case SH7764_SSI_SR:
            /*
             * UIRQ and OIRQ clear on a written zero. IIRQ and DIRQ are read
             * only, and the reserved bits read back fixed, so both survive
             * whatever is written - the firmware clears the whole register in
             * one store and depends on the idle flag still standing when it
             * looks again.
             */
            b->regs[idx] = (b->regs[idx] & ~SH7764_SSI_SR_W0C) |
                           (b->regs[idx] & (uint32_t)value & SH7764_SSI_SR_W0C);
            sh7764_ssi_chan_update_irq(b->soc, b);
            return;
        case SH7764_SSI_DMINTMR:
            b->regs[idx] = value;
            sh7764_ssi_update_irq(b->soc, b);
            return;
        case SH7764_SSI_DMCOR:
            b->regs[idx] = value;
            sh7764_ssi_start(b->soc, b, value);
            return;
        }
    }
    b->regs[idx] = value;
}

static const MemoryRegionOps sh7764_bank_ops = {
    .read = sh7764_bank_read,
    .write = sh7764_bank_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sh7764_bank_init(SH7764State *s, SH7764RegBank *b,
                             const char *name, hwaddr base, uint64_t size)
{
    b->soc = s;
    b->name = name;
    b->nregs = size / 4;
    b->regs = g_new0(uint32_t, b->nregs);
    memory_region_init_io(&b->iomem, OBJECT(s), &sh7764_bank_ops, b,
                          name, size);
    memory_region_add_subregion(get_system_memory(), base, &b->iomem);
}

/* ---------------------------------------------------------------- SCIF */

static void sh7764_scif_init(SH7764State *s, hwaddr base, int chan_index,
                             const char *id, qemu_irq eri, qemu_irq rxi,
                             qemu_irq bri, qemu_irq txi)
{
    DeviceState *dev = qdev_new(TYPE_SH_SERIAL);
    SysBusDevice *sb = SYS_BUS_DEVICE(dev);

    dev->id = g_strdup(id);
    qdev_prop_set_chr(dev, "chardev", serial_hd(chan_index));
    qdev_prop_set_uint8(dev, "features", SH_SERIAL_FEAT_SCIF);
    sysbus_realize_and_unref(sb, &error_fatal);
    sysbus_mmio_map(sb, 0, base);
    qdev_connect_gpio_out_named(dev, "eri", 0, eri);
    qdev_connect_gpio_out_named(dev, "rxi", 0, rxi);
    qdev_connect_gpio_out_named(dev, "bri", 0, bri);
    qdev_connect_gpio_out_named(dev, "txi", 0, txi);
}

/* ---------------------------------------------------------------- INTC */

/*
 * On-chip module interrupt controller ("INT2"). Vectors are from table 13.1
 * of the SH7764 hardware manual (R01UH0360EJ0300), priority register field
 * assignments from table 13.5, and mask register bit assignments from
 * sections 13.3.14 and 13.3.15.
 *
 * Only the sources this machine can currently drive are modelled. Adding more
 * is a matter of extending these tables; the register layout is already right.
 */

static struct intc_vect sh7764_vectors[] = {
    INTC_VECT(TUNI0, 0x580), INTC_VECT(TUNI1, 0x5a0),
    INTC_VECT(TUNI2, 0x5c0), INTC_VECT(TICPI2, 0x5e0),
    INTC_VECT(WDT_ITI, 0x560),
    INTC_VECT(ATAPI_ATAI, 0xc00),
    INTC_VECT(SSI_ADMA0, 0xa00), INTC_VECT(SSI_BDMA1, 0xaa0),
    INTC_VECT(SSI_ACH0, 0xa20), INTC_VECT(SSI_BCH3, 0xac0),
    INTC_VECT(ETHERC, 0x920),
    INTC_VECT(USBI, 0xc60),
    INTC_VECT(DMINT1, 0x660),
    INTC_VECT(SCIF0_ERI, 0x700), INTC_VECT(SCIF0_RXI, 0x720),
    INTC_VECT(SCIF0_BRI, 0x740), INTC_VECT(SCIF0_TXI, 0x760),
    INTC_VECT(SCIF2_ERI, 0xf00), INTC_VECT(SCIF2_RXI, 0xf20),
    INTC_VECT(SCIF2_BRI, 0xf40), INTC_VECT(SCIF2_TXI, 0xf60),
};

static struct intc_group sh7764_groups[] = {
    INTC_GROUP(SCIF0, SCIF0_ERI, SCIF0_RXI, SCIF0_BRI, SCIF0_TXI),
    INTC_GROUP(SCIF2, SCIF2_ERI, SCIF2_RXI, SCIF2_BRI, SCIF2_TXI),
    /*
     * Table 13.1 gives SSIDMA1 and SSICH3 separate vectors but a single
     * priority field, INT2PRI5[4:0], and a single mask bit, INT2MSKR[19].
     * A group is how sh_intc expresses one gate feeding several sources.
     * SSI_A is not like this: SSIDMA0 and SSICH0 have a field and a mask bit
     * each, so they stay independent.
     */
    INTC_GROUP(SSI_B, SSI_BDMA1, SSI_BCH3),
};

/*
 * INT2PRI0 to INT2PRI12, 32 bits holding four 5-bit priority fields at bits
 * 28-24, 20-16, 12-8 and 4-0. sh_intc wants evenly spaced fields, so they are
 * declared as 8 bits wide; the top three bits of each byte are reserved and
 * read as zero, which does not affect priority comparison. Entries are listed
 * most significant field first.
 */
static struct intc_prio_reg sh7764_prio_registers[] = {
    { 0xffd40000, 0, 32, 8, /* INT2PRI0 */ { TUNI0, TUNI1, TUNI2, TICPI2 } },
    { 0xffd40008, 0, 32, 8, /* INT2PRI2 */ { SCIF0, UNUSED, WDT_ITI, UNUSED } },
    { 0xffd4000c, 0, 32, 8, /* INT2PRI3 */
      { UNUSED, DMINT1, UNUSED, UNUSED } },
    { 0xffd40010, 0, 32, 8, /* INT2PRI4 */
      { UNUSED, UNUSED, SSI_ADMA0, SSI_ACH0 } },
    { 0xffd40014, 0, 32, 8, /* INT2PRI5 */
      { UNUSED, UNUSED, UNUSED, SSI_B } },
    { 0xffd40018, 0, 32, 8, /* INT2PRI6 */
      { ATAPI_ATAI, UNUSED, UNUSED, UNUSED } },
    { 0xffd4001c, 0, 32, 8, /* INT2PRI7 */ { SCIF2, UNUSED, UNUSED, UNUSED } },
    { 0xffd400b0, 0, 32, 8, /* INT2PRI12 */
      { UNUSED, UNUSED, USBI, ETHERC } },
};

/*
 * INT2MSKR / INT2MSKR1 use 1 = masked, and the paired INT2MSKCR / INT2MSKCR1
 * clear a mask bit. sh_intc's dual-register model is the other way round: a
 * set bit means enabled. The clear register is therefore given as sh_intc's
 * "set" register and vice versa, which makes enable and disable behave
 * correctly. The consequence is that a guest read of INT2MSKR returns
 * sh_intc's enable bits rather than the hardware's inverted mask bits; no
 * firmware we have examined reads them back, but it is a known deviation.
 *
 * Entries are most significant bit first, so index i is bit (31 - i).
 */
static struct intc_mask_reg sh7764_mask_registers[] = {
    { 0xffd4003c, 0xffd40038, 32, /* INT2MSKCR / INT2MSKR */
      { 0, 0, 0, 0, 0, 0, 0, 0,                 /* 31..24 */
        0, 0, 0, ATAPI_ATAI, SSI_B, 0, 0, 0,            /* 23..16 */
        SSI_ACH0, SSI_ADMA0, 0, 0, 0, 0, 0, DMINT1,     /* 15..8  */
        0, 0, WDT_ITI, SCIF0, 0, 0, TUNI1, TUNI0 },     /* 7..0 */
      0, true },
    { 0xffd400d4, 0xffd400d0, 32, /* INT2MSKCR1 / INT2MSKR1 */
      { 0, 0, 0, 0, 0, 0, SCIF2, 0,             /* 31..24 */
        0, 0, 0, 0, 0, 0, USBI, ETHERC,         /* 23..16 */
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0 }, 0, true },
};

/* ----------------------------------------------------------------- QOM */

static void sh7764_realize(DeviceState *dev, Error **errp)
{
    SH7764State *s = SH7764(dev);
    MemoryRegion *sysmem = get_system_memory();

    if (!s->cpu) {
        error_setg(errp, "sh7764: 'cpu' link property must be set");
        return;
    }
    if (!s->periph_freq) {
        s->periph_freq = 50000000;
    }

    /*
     * Catch-all beneath everything else (priority -1000). Peripheral windows
     * this model does not implement then log under -d unimp instead of
     * silently reading back zero, which makes it obvious when firmware is
     * waiting on something that was never wired up.
     */
    create_unimplemented_device("sh7764.p4", 0xfe000000, 0x02000000);

    memory_region_init_io(&s->ccn, OBJECT(s), &sh7764_ccn_ops, s,
                          "sh7764.ccn", SH7764_CCN_SIZE);
    memory_region_add_subregion(sysmem, SH7764_CCN_BASE, &s->ccn);

    memory_region_init_io(&s->dmac, OBJECT(s), &sh7764_dmac_ops, s,
                          "sh7764.dmac", SH7764_DMAC_SIZE);
    memory_region_add_subregion(sysmem, SH7764_DMAC_BASE, &s->dmac);

    memory_region_init_io(&s->wdt, OBJECT(s), &sh7764_wdt_ops, s,
                          "sh7764.wdt", SH7764_WDT_SIZE);
    memory_region_add_subregion(sysmem, SH7764_WDT_BASE, &s->wdt);

    sh7764_bank_init(s, &s->bsc, "sh7764.bsc",
                     SH7764_BSC_BASE, SH7764_BSC_SIZE);
    sh7764_bank_init(s, &s->gpio, "sh7764.gpio",
                     SH7764_GPIO_BASE, SH7764_GPIO_SIZE);

    qemu_chr_fe_set_handlers(&s->gui_chr, sh7764_ssi_can_receive,
                             sh7764_ssi_receive, NULL, NULL, s, NULL, true);

    s->panel_rx_chan = -1;
    qemu_chr_fe_set_handlers(&s->panel_chr, sh7764_panel_can_receive,
                             sh7764_panel_receive, NULL, NULL, s, NULL, true);

    if (s->watch_size) {
        MemoryRegionSection sec;

        sec = memory_region_find(get_system_memory(), s->watch_base, 1);
        if (sec.mr && memory_region_is_ram(sec.mr)) {
            s->watch_ram = memory_region_get_ram_ptr(sec.mr) +
                           sec.offset_within_region;
            memory_region_init_io(&s->watch, OBJECT(s), &sh7764_watch_ops, s,
                                  "sh7764.watch", s->watch_size);
            memory_region_add_subregion_overlap(get_system_memory(),
                                                s->watch_base, &s->watch, 1);
        }
        memory_region_unref(sec.mr);
    }
    sh7764_bank_init(s, &s->ssi_a, "sh7764.ssi-a",
                     SH7764_SSI_A_BASE, SH7764_SSI_SIZE);
    sh7764_bank_init(s, &s->ssi_b, "sh7764.ssi-b",
                     SH7764_SSI_B_BASE, SH7764_SSI_SIZE);
    sh7764_ssi_bank_reset(&s->ssi_a);
    sh7764_ssi_bank_reset(&s->ssi_b);
    memory_region_init_io(&s->atapi, OBJECT(s), &sh7764_atapi_ops, s,
                          "sh7764.atapi", SH7764_ATAPI_SIZE);
    memory_region_add_subregion(sysmem, SH7764_ATAPI_BASE, &s->atapi);

    memory_region_init_io(&s->sdhi, OBJECT(s), &sh7764_sdhi_ops, s,
                          "sh7764.sdhi", SH7764_SDHI_SIZE);
    memory_region_add_subregion(sysmem, SH7764_SDHI_BASE, &s->sdhi);
    memory_region_init_io(&s->iic, OBJECT(s), &sh7764_iic_ops, s,
                          "sh7764.iic", SH7764_IIC_SIZE);
    memory_region_add_subregion(sysmem, SH7764_IIC_BASE, &s->iic);
    sh7764_bank_init(s, &s->misc_a, "sh7764.misc-ffa0",
                     SH7764_MISC_A_BASE, SH7764_MISC_A_SIZE);
    memory_region_init_io(&s->cpuopm, OBJECT(s), &sh7764_cpuopm_ops, s,
                          "sh7764.cpuopm", SH7764_CPUOPM_SIZE);
    memory_region_add_subregion(sysmem, SH7764_CPUOPM_BASE, &s->cpuopm);

    sh_intc_init(sysmem, &s->intc, NR_INTC_SOURCES,
                 _INTC_ARRAY(sh7764_mask_registers),
                 _INTC_ARRAY(sh7764_prio_registers));

    /*
     * INT2B4 lives with the rest of the interrupt controller, which sh_intc
     * places at the area 7 addresses and then aliases into P4 one register at
     * a time - the processor does not fold P4 onto area 7 by itself. Software
     * reaches this one through P4, so it needs both.
     */
    memory_region_init_io(&s->int2b4, OBJECT(s), &sh7764_int2b4_ops, s,
                          "sh7764.int2b4", 4);
    memory_region_add_subregion(sysmem, 0x1fd40050, &s->int2b4);
    memory_region_init_alias(&s->int2b4_p4, OBJECT(s), "sh7764.int2b4-p4",
                             &s->int2b4, 0, 4);
    memory_region_add_subregion(sysmem, 0xffd40050, &s->int2b4_p4);

    memory_region_init_io(&s->int2b3, OBJECT(s), &sh7764_int2b3_ops, s,
                          "sh7764.int2b3", 4);
    memory_region_add_subregion(sysmem, 0x1fd4004c, &s->int2b3);
    memory_region_init_alias(&s->int2b3_p4, OBJECT(s), "sh7764.int2b3-p4",
                             &s->int2b3, 0, 4);
    memory_region_add_subregion(sysmem, 0xffd4004c, &s->int2b3_p4);
    sh_intc_register_sources(&s->intc,
                             _INTC_ARRAY(sh7764_vectors),
                             _INTC_ARRAY(sh7764_groups));
    s->cpu->env.intc_handle = &s->intc;

    sh7764_scif_init(s, SH7764_SCIF0_BASE, 0, "scif0",
                     s->intc.irqs[SCIF0_ERI], s->intc.irqs[SCIF0_RXI],
                     s->intc.irqs[SCIF0_BRI], s->intc.irqs[SCIF0_TXI]);
    sh7764_scif_init(s, SH7764_SCIF2_BASE, 1, "scif2",
                     s->intc.irqs[SCIF2_ERI], s->intc.irqs[SCIF2_RXI],
                     s->intc.irqs[SCIF2_BRI], s->intc.irqs[SCIF2_TXI]);

    /*
     * EtherC and its DMA engine. The firmware polls this block hard - roughly
     * 11,700 accesses in a 400-second run with nothing behind it - because it
     * is looking for a link before it will start Pro DJ Link.
     */
    s->eth = qdev_new(TYPE_SH7764_ETH);
    qemu_configure_nic_device(s->eth, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->eth), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->eth), 0, SH7764_ETH_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->eth), 0, s->intc.irqs[ETHERC]);

    /*
     * The USB 2.0 host/function module. The firmware selects the host half
     * and drives the Type A socket with it, so anything plugged into the
     * emulated bus - a usb-storage image or a passed-through stick - appears
     * to the player as a device in that socket.
     */
    s->usb = qdev_new(TYPE_SH7764_USB);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->usb), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->usb), 0, SH7764_USB_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->usb), 0, s->intc.irqs[USBI]);

    /*
     * TMU channels 0 to 2. The register block is laid out exactly as the
     * SH7750's (TOCR, TSTR0, then TCOR/TCNT/TCR per channel), so QEMU's
     * existing tmu012 model fits without modification.
     */
    tmu012_init(sysmem, SH7764_TMU_BASE,
                TMU012_FEAT_TOCR | TMU012_FEAT_3CHAN,
                s->periph_freq,
                s->intc.irqs[TUNI0], s->intc.irqs[TUNI1],
                s->intc.irqs[TUNI2], s->intc.irqs[TICPI2]);
}

static void sh7764_reset(DeviceState *dev)
{
    SH7764State *s = SH7764(dev);
    int i;

    s->dmaor = 0;
    for (i = 0; i < SH7764_DMAC_NCHAN; i++) {
        s->sar[i] = 0;
        s->dar[i] = 0;
        s->tcr[i] = 0;
        s->chcr[i] = 0;
    }
    s->ccr = 0;
    s->wtcnt = 0;
    s->wtcsr = 0;
    s->wrcsr = 0;

    if (s->ssi_a.regs) {
        s->ssi_rx_len = 0;
        s->ssi_rx_armed = false;
        s->panel_rx_len = 0;
        s->panel_have_last = false;
        sh7764_ssi_bank_reset(&s->ssi_a);
        sh7764_ssi_bank_reset(&s->ssi_b);
    }

    /* The drive comes up idle, ready and with nothing to hand over. */
    s->atapi_status = SH7764_ATA_ST_DRDY | SH7764_ATA_ST_DSC;
    s->atapi_error = 0;
    s->atapi_intreason = SH7764_ATA_IR_CD | SH7764_ATA_IR_IO;
    s->atapi_bytecount = 0;
    s->atapi_pos = 0;
    s->atapi_len = 0;
    s->atapi_packet_pos = 0;
    s->atapi_want_packet = false;

    /*
     * EXPEVT reads 0x000 after a power-on reset and 0x020 after a manual
     * reset. Boot ROMs branch on this, so it has to be right.
     */
    if (s->cpu) {
        s->cpu->env.expevt = 0x000;
    }
}

static const Property sh7764_properties[] = {
    DEFINE_PROP_LINK("cpu", SH7764State, cpu, TYPE_SUPERH_CPU, SuperHCPU *),
    /*
     * Peripheral clock feeding the TMU. The exact Pck for a given board
     * depends on the crystal and the clock mode pins, neither of which we can
     * read out of the firmware, so it is a property with a plausible default
     * rather than a hardcoded guess.
     */
    DEFINE_PROP_UINT32("periph-clock-hz", SH7764State, periph_freq, 50000000),
    DEFINE_PROP_CHR("gui-chardev", SH7764State, gui_chr),
    DEFINE_PROP_CHR("panel-chardev", SH7764State, panel_chr),
    DEFINE_PROP_UINT32("watch-base", SH7764State, watch_base, 0),
    DEFINE_PROP_UINT32("watch-size", SH7764State, watch_size, 0),
};

static void sh7764_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sh7764_realize;
    device_class_set_legacy_reset(dc, sh7764_reset);
    device_class_set_props(dc, sh7764_properties);
    dc->desc = "Renesas SH7764 SoC";
    /* This SoC is only meaningful as part of a board. */
    dc->user_creatable = false;
}

static const TypeInfo sh7764_types[] = {
    {
        .name           = TYPE_SH7764,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(SH7764State),
        .class_init     = sh7764_class_init,
    },
};

DEFINE_TYPES(sh7764_types)
