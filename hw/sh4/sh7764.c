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
#include "hw/sh4/sh.h"
#include "hw/misc/unimp.h"
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
    /* groups */
    SCIF0, SCIF2,
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

static void sh7764_dma_run(SH7764State *s, int ch)
{
    uint32_t chcr = s->chcr[ch];
    unsigned unit, ts;
    uint64_t len;
    hwaddr src, dst;
    uint8_t buf[4096];

    if (!(chcr & SH7764_CHCR_DE) || !(s->dmaor & SH7764_DMAOR_DME)) {
        return;
    }

    ts = (chcr >> SH7764_CHCR_TS_SHIFT) & SH7764_CHCR_TS_MASK;
    unit = sh7764_ts_unit[ts];
    len = (uint64_t)s->tcr[ch] * unit;
    if (len == 0) {
        return;
    }

    src = sh7764_dma_addr(s->sar[ch]);
    dst = sh7764_dma_addr(s->dar[ch]);
    trace_sh7764_dma_run(ch, src, dst, len, unit);

    /*
     * Real hardware would run this in the background and raise TE when it
     * finishes. Firmware only ever polls TE, so completing synchronously is
     * indistinguishable and avoids modelling bus arbitration.
     */
    while (len) {
        size_t n = MIN(len, sizeof(buf));

        address_space_read(&address_space_memory, src,
                           MEMTXATTRS_UNSPECIFIED, buf, n);
        address_space_write(&address_space_memory, dst,
                            MEMTXATTRS_UNSPECIFIED, buf, n);
        src += n;
        dst += n;
        len -= n;
    }

    /*
     * Deliberately leave SAR/DAR alone. Whether the engine post-increments
     * them depends on CHCR's SM/DM address-mode bits, and we cannot verify a
     * reading of those against real silicon. Leaving them as programmed
     * matches the reference interpreter this model was validated against, and
     * firmware reprograms them before every transfer anyway.
     */
    s->tcr[ch] = 0;
    s->chcr[ch] |= SH7764_CHCR_TE;
}

static int sh7764_dma_chan(hwaddr offset)
{
    if (offset < SH7764_DMAC_CH0) {
        return -1;
    }
    offset -= SH7764_DMAC_CH0;
    if (offset >= SH7764_DMAC_CH_STRIDE * SH7764_DMAC_NCHAN) {
        return -1;
    }
    return offset / SH7764_DMAC_CH_STRIDE;
}

static uint64_t sh7764_dmac_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764State *s = opaque;
    int ch = sh7764_dma_chan(offset);

    if (offset == SH7764_DMAC_DMAOR) {
        trace_sh7764_dmac_read(offset, s->dmaor);
        return s->dmaor;
    }
    if (ch < 0) {
        qemu_log_mask(LOG_UNIMP, "sh7764: DMAC read of unhandled offset 0x%"
                      HWADDR_PRIx "\n", offset);
        return 0;
    }

    trace_sh7764_dmac_read(offset, 0);
    switch (offset % SH7764_DMAC_CH_STRIDE) {
    case SH7764_DMAC_SAR:
        return s->sar[ch];
    case SH7764_DMAC_DAR:
        return s->dar[ch];
    case SH7764_DMAC_TCR:
        return s->tcr[ch];
    case SH7764_DMAC_CHCR:
        return s->chcr[ch];
    default:
        return 0;
    }
}

static void sh7764_dmac_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    SH7764State *s = opaque;
    int ch = sh7764_dma_chan(offset);

    trace_sh7764_dmac_write(offset, value);
    if (offset == SH7764_DMAC_DMAOR) {
        s->dmaor = value;
        return;
    }
    if (ch < 0) {
        qemu_log_mask(LOG_UNIMP, "sh7764: DMAC write of unhandled offset 0x%"
                      HWADDR_PRIx " = 0x%" PRIx64 "\n", offset, value);
        return;
    }

    switch (offset % SH7764_DMAC_CH_STRIDE) {
    case SH7764_DMAC_SAR:
        s->sar[ch] = value;
        break;
    case SH7764_DMAC_DAR:
        s->dar[ch] = value;
        break;
    case SH7764_DMAC_TCR:
        s->tcr[ch] = value;
        break;
    case SH7764_DMAC_CHCR:
        s->chcr[ch] = value;
        sh7764_dma_run(s, ch);
        break;
    }
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

/* ------------------------------------------------- generic register bank */

/*
 * Port pins driven from outside the chip.
 *
 * A data register bank reads back whatever software wrote, which is right for
 * an output and wrong for an input: an input reads what the other end of the
 * wire is doing. The GUI processor drives one of these. GuiCom_SndTASK sits
 * at 0x04259e2e polling port C bit 2 and only sends once it is set, so with
 * the bank alone the main processor never says anything to the panel at all.
 *
 * Until the two machines are wired together, the answer is the one a working
 * player gives: the GUI processor is fitted and ready.
 */
#define SH7764_PTDAT_C             0x48
#define SH7764_PTDAT_C_GUI_READY   (1u << 2)

static uint64_t sh7764_bank_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764RegBank *b = opaque;
    unsigned idx = offset / 4;
    uint32_t val;

    if (idx >= b->nregs) {
        return 0;
    }
    val = b->regs[idx];
    if (b->input_mask && offset == SH7764_PTDAT_C) {
        val |= b->input_mask;
    }
    trace_sh7764_bank_read(b->name, offset, val, size);
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
    trace_sh7764_bank_write(b->name, offset, value, size);
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
    INTC_VECT(SCIF0_ERI, 0x700), INTC_VECT(SCIF0_RXI, 0x720),
    INTC_VECT(SCIF0_BRI, 0x740), INTC_VECT(SCIF0_TXI, 0x760),
    INTC_VECT(SCIF2_ERI, 0xf00), INTC_VECT(SCIF2_RXI, 0xf20),
    INTC_VECT(SCIF2_BRI, 0xf40), INTC_VECT(SCIF2_TXI, 0xf60),
};

static struct intc_group sh7764_groups[] = {
    INTC_GROUP(SCIF0, SCIF0_ERI, SCIF0_RXI, SCIF0_BRI, SCIF0_TXI),
    INTC_GROUP(SCIF2, SCIF2_ERI, SCIF2_RXI, SCIF2_BRI, SCIF2_TXI),
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
    { 0xffd40018, 0, 32, 8, /* INT2PRI6 */
      { ATAPI_ATAI, UNUSED, UNUSED, UNUSED } },
    { 0xffd4001c, 0, 32, 8, /* INT2PRI7 */ { SCIF2, UNUSED, UNUSED, UNUSED } },
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
        0, 0, 0, 0, 0, 0, 0, 0,                 /* 23..16 */
        0, ATAPI_ATAI, 0, 0, 0, 0, 0, 0,        /* 15..8  */
        0, 0, WDT_ITI, SCIF0, 0, 0, TUNI1, TUNI0 },     /* 7..0 */
      0, true },
    { 0xffd400d4, 0xffd400d0, 32, /* INT2MSKCR1 / INT2MSKR1 */
      { 0, 0, 0, 0, 0, 0, SCIF2, 0,             /* 31..24 */
        0, 0, 0, 0, 0, 0, 0, 0,                 /* 23..16 */
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
    s->gpio.input_mask = SH7764_PTDAT_C_GUI_READY;
    sh7764_bank_init(s, &s->ssi_a, "sh7764.ssi-a",
                     SH7764_SSI_A_BASE, SH7764_SSI_SIZE);
    sh7764_bank_init(s, &s->ssi_b, "sh7764.ssi-b",
                     SH7764_SSI_B_BASE, SH7764_SSI_SIZE);
    memory_region_init_io(&s->atapi, OBJECT(s), &sh7764_atapi_ops, s,
                          "sh7764.atapi", SH7764_ATAPI_SIZE);
    memory_region_add_subregion(sysmem, SH7764_ATAPI_BASE, &s->atapi);

    memory_region_init_io(&s->sdhi, OBJECT(s), &sh7764_sdhi_ops, s,
                          "sh7764.sdhi", SH7764_SDHI_SIZE);
    memory_region_add_subregion(sysmem, SH7764_SDHI_BASE, &s->sdhi);
    sh7764_bank_init(s, &s->misc_a, "sh7764.misc-ffa0",
                     SH7764_MISC_A_BASE, SH7764_MISC_A_SIZE);
    memory_region_init_io(&s->cpuopm, OBJECT(s), &sh7764_cpuopm_ops, s,
                          "sh7764.cpuopm", SH7764_CPUOPM_SIZE);
    memory_region_add_subregion(sysmem, SH7764_CPUOPM_BASE, &s->cpuopm);

    sh_intc_init(sysmem, &s->intc, NR_INTC_SOURCES,
                 _INTC_ARRAY(sh7764_mask_registers),
                 _INTC_ARRAY(sh7764_prio_registers));
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
