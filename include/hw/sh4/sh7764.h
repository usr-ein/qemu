/*
 * Renesas SH7764 (SH-4A) system on chip
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SH4_SH7764_H
#define HW_SH4_SH7764_H

#include "hw/core/sysbus.h"
#include "system/memory.h"
#include "target/sh4/cpu-qom.h"
#include "hw/sh4/sh_intc.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_SH7764 "sh7764"
OBJECT_DECLARE_SIMPLE_TYPE(SH7764State, SH7764)

/*
 * Peripheral windows in the P4 control-register area.
 *
 * These are the windows the CDJ-2000NXS firmware actually drives; the address
 * assignments were recovered by executing the boot ROM under an instrumented
 * SH-4A interpreter (see docs/boot-sequence.md in the parent project) and by
 * static analysis of the application image. Where a window's identity is not
 * yet confirmed against the Renesas hardware manual it is modelled as a plain
 * register bank and named accordingly.
 */
#define SH7764_CCN_BASE         0xff000000  /* cache / MMU control            */
#define SH7764_CCN_SIZE         0x00001000

#define SH7764_DMAC_BASE        0xff608000  /* direct memory access controller*/
#define SH7764_DMAC_SIZE        0x00002000  /* reaches the DMARS at +0x9000   */

#define SH7764_BSC_BASE         0xff800000  /* bus state controller / DDR     */
#define SH7764_BSC_SIZE         0x00003000  /* covers +0x1000 and +0x2000     */

#define SH7764_WDT_BASE         0xffcc0000  /* watchdog timer                 */
#define SH7764_WDT_SIZE         0x00001000

#define SH7764_SCIF0_BASE       0xffe00000
#define SH7764_SCIF2_BASE       0xffe20000  /* the firmware's debug console   */

#define SH7764_GPIO_BASE        0xfff10000  /* GPIO / pin function            */
#define SH7764_GPIO_SIZE        0x00001000
#define SH7764_PTDAT_C          0x0048
#define SH7764_PTDAT_C_PTC2     (1u << 2)

#define SH7764_INTC_BASE        0xffd00000  /* IRQ-side INTC registers        */
#define SH7764_INTC_SIZE        0x00001000
#define SH7764_TMU_BASE         0xffd80000  /* TMU channels 0-2               */
#define SH7764_SSI_A_BASE       0xff400000  /* serial sound interface A       */
#define SH7764_SSI_B_BASE       0xff500000  /* serial sound interface B       */
#define SH7764_SSI_SIZE         0x00008000
#define SH7764_SDHI_BASE        0xffe40000  /* SD host interface (undoc.)     */
#define SH7764_IIC_BASE         0xffe70000  /* I2C bus interface              */
#define SH7764_IIC_SIZE         0x00000028
#define SH7764_SDHI_SIZE        0x00010000
#define SH7764_ATAPI_BASE       0xfff00000  /* ATAPI (the CD/DVD mechanism)   */
#define SH7764_ATAPI_SIZE       0x00001000

/*
 * Windows the firmware touches that the manual's register list does not
 * account for. Left as plain banks until they can be identified.
 */
#define SH7764_MISC_A_BASE      0xffa00000
#define SH7764_MISC_A_SIZE      0x00001000
/*
 * The front panel link. SCIF2 carries it, a DMA channel each way, in fixed
 * 24-byte frames: 22 bytes of payload, a checksum byte and a 0x8F terminator.
 */
#define SH7764_PANEL_FRAME      24
#define SH7764_SCIF2_SCFTDR     0xffe2000c  /* transmit FIFO data           */
#define SH7764_SCIF2_SCFRDR     0xffe20014  /* receive FIFO data            */

/* USB 2.0 host/function module, section 21. */
#define SH7764_USB_BASE         0xfe400000

/*
 * CPU operation mode register, appendix A of the hardware manual. INTMU
 * (bit 3) makes an accepted interrupt load its priority into SR.IMASK.
 */
#define SH7764_CPUOPM_BASE      0xff2f0000
#define SH7764_CPUOPM_SIZE      0x00001000

/* CCN register offsets (SH-4 architectural). */
#define SH7764_CCN_PTEH         0x00
#define SH7764_CCN_PTEL         0x04
#define SH7764_CCN_TTB          0x08
#define SH7764_CCN_TEA          0x0c
#define SH7764_CCN_MMUCR        0x10
#define SH7764_CCN_CCR          0x1c
#define SH7764_CCN_TRA          0x20
#define SH7764_CCN_EXPEVT       0x24
#define SH7764_CCN_INTEVT       0x28
#define SH7764_CCN_PVR          0x30
#define SH7764_CCN_PRR          0x44

/*
 * DMAC, section 12.3. Channels 0 to 3 sit in one run from 0x20, DMAOR splits
 * them from channels 4 and 5, and the B-side reload registers follow at 0x120.
 * The DMA extended resource selectors are a kilobyte further on, which is why
 * the window has to reach past 0x9008.
 */
#define SH7764_DMAC_CH0         0x0020      /* SAR0; channels 0 to 3          */
#define SH7764_DMAC_DMAOR       0x0060      /* 16-bit, bit 0 = DME            */
#define SH7764_DMAC_CH4         0x0070      /* SAR4; channels 4 and 5         */
#define SH7764_DMAC_CHB0        0x0120      /* SARB0; channels 0 to 3         */
#define SH7764_DMAC_CH_STRIDE   0x0010
#define SH7764_DMAC_CHB_STRIDE  0x0010
#define SH7764_DMAC_NCHAN       6
#define SH7764_DMAC_NCHAN_B     4
#define SH7764_DMAC_DMARS       0x9000      /* 16-bit, one per channel pair   */
#define SH7764_DMAC_NDMARS      3

#define SH7764_DMAC_SAR         0x00
#define SH7764_DMAC_DAR         0x04
#define SH7764_DMAC_TCR         0x08
#define SH7764_DMAC_CHCR        0x0c

#define SH7764_CHCR_DE          (1u << 0)   /* enable                         */
#define SH7764_CHCR_TE          (1u << 1)   /* transfer end                   */
#define SH7764_CHCR_IE          (1u << 2)   /* interrupt on transfer end      */
/* Transfer size is TS[1:0] at bits 4 and 3 with TS[2] stranded at bit 20. */
#define SH7764_CHCR_TS_SHIFT    3
#define SH7764_CHCR_TS_MASK     0x3
#define SH7764_CHCR_TS2         (1u << 20)
#define SH7764_CHCR_SM_SHIFT    12          /* source address mode            */
#define SH7764_CHCR_DM_SHIFT    14          /* destination address mode       */
#define SH7764_CHCR_AM_MASK     0x3
#define SH7764_CHCR_AM_FIXED    0
#define SH7764_CHCR_AM_INC      1
#define SH7764_CHCR_AM_DEC      2
#define SH7764_CHCR_RS_SHIFT    8           /* resource select                */
#define SH7764_CHCR_RS_MASK     0xf
#define SH7764_CHCR_RS_AUTO     0x4
#define SH7764_CHCR_RS_DMARS    0x8

#define SH7764_DMAOR_DME        (1u << 0)

/* Watchdog key writes: WTCNT takes 0x5a00xxxx, WTCSR takes 0xa500xxxx. */
#define SH7764_WDT_WTCNT        0x00
#define SH7764_WDT_WTCSR        0x04
#define SH7764_WDT_WRCSR        0x08
#define SH7764_WTCNT_KEY        0x5a00
#define SH7764_WTCSR_KEY        0xa500

/* A plain read/write register bank with no side effects. */
typedef struct SH7764RegBank {
    MemoryRegion iomem;
    SH7764State *soc;
    const char *name;
    uint32_t *regs;
    unsigned nregs;
    /* Blocks an SSI transfer still owes the driver; see sh7764_ssi_done. */
    uint32_t blocks_left;
} SH7764RegBank;

struct SH7764State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    SuperHCPU *cpu;
    MemoryRegion *dram_mr;

    MemoryRegion ccn;
    MemoryRegion dmac;
    MemoryRegion wdt;
    MemoryRegion cpuopm;

    struct intc_desc intc;
    uint32_t periph_freq;

    SH7764RegBank bsc;
    SH7764RegBank gpio;
    SH7764RegBank ssi_a;
    SH7764RegBank ssi_b;
    MemoryRegion sdhi;
    MemoryRegion iic;
    uint8_t iic_regs[SH7764_IIC_SIZE / 4];
    MemoryRegion int2b3;
    MemoryRegion int2b3_p4;
    MemoryRegion int2b4;
    MemoryRegion int2b4_p4;
    DeviceState *eth;
    DeviceState *usb;
    MemoryRegion atapi;
    uint32_t atapi_ctl[0x40];
    /*
     * The drive behind the ATAPI controller. Only enough of it is modelled
     * for the firmware to identify the mechanism and be told there is no
     * disc in it: the task file, the two commands the probe issues, and a
     * fixed sense reply.
     */
    uint8_t atapi_status;
    uint8_t atapi_error;
    uint8_t atapi_intreason;
    uint8_t atapi_device;
    uint8_t atapi_features;
    uint16_t atapi_bytecount;
    uint8_t atapi_buf[512];
    uint32_t atapi_pos;
    uint32_t atapi_len;
    uint8_t atapi_packet[12];
    uint32_t atapi_packet_pos;
    bool atapi_want_packet;
    /*
     * The drive's INTRQ line, and nIEN from the device control register.
     * INTRQ is level held: the drive asserts it when a command ends or data
     * becomes available and drops it when the host reads the status register.
     */
    bool atapi_intrq;
    bool atapi_nien;
    bool atapi_reset;
    SH7764RegBank misc_a;

    /*
     * The link to the front panel's M16C, which carries every button, the
     * jog wheel and the rotary encoder. See the panel section.
     */
    CharFrontend panel_chr;
    uint8_t panel_rx_buf[SH7764_PANEL_FRAME * 4];
    uint32_t panel_rx_len;
    uint8_t panel_last[SH7764_PANEL_FRAME];  /* the last frame really sent  */
    bool panel_have_last;
    int32_t panel_rx_chan;              /* DMA channel armed on SCIF2 RX    */

    /* The link to the GUI processor; see the SSI section. */
    CharFrontend gui_chr;
    uint8_t ssi_rx_buf[4096];
    uint32_t ssi_rx_len;
    bool ssi_rx_armed;
    bool ssi_gui_answered;

    /* Optional write watch; see sh7764_watch_write. */
    MemoryRegion watch;
    void *watch_ram;
    uint32_t watch_base;
    uint32_t watch_size;

    /* DMAC state */
    uint32_t dmaor;
    uint32_t sar[SH7764_DMAC_NCHAN];
    uint32_t dar[SH7764_DMAC_NCHAN];
    uint32_t tcr[SH7764_DMAC_NCHAN];
    uint32_t chcr[SH7764_DMAC_NCHAN];
    uint32_t sarb[SH7764_DMAC_NCHAN_B];
    uint32_t darb[SH7764_DMAC_NCHAN_B];
    uint32_t tcrb[SH7764_DMAC_NCHAN_B];
    uint16_t dmars[SH7764_DMAC_NDMARS];

    uint32_t ccr;

    /* WDT state */
    uint32_t wtcnt;
    uint32_t wtcsr;
    uint32_t wrcsr;
};

#endif /* HW_SH4_SH7764_H */
