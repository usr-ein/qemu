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
#define SH7764_DMAC_SIZE        0x00001000

#define SH7764_BSC_BASE         0xff800000  /* bus state controller / DDR     */
#define SH7764_BSC_SIZE         0x00003000  /* covers +0x1000 and +0x2000     */

#define SH7764_WDT_BASE         0xffcc0000  /* watchdog timer                 */
#define SH7764_WDT_SIZE         0x00001000

#define SH7764_SCIF0_BASE       0xffe00000
#define SH7764_SCIF2_BASE       0xffe20000  /* the firmware's debug console   */

#define SH7764_GPIO_BASE        0xfff10000  /* GPIO / pin function            */
#define SH7764_GPIO_SIZE        0x00001000

#define SH7764_INTC_BASE        0xffd00000  /* IRQ-side INTC registers        */
#define SH7764_INTC_SIZE        0x00001000
#define SH7764_TMU_BASE         0xffd80000  /* TMU channels 0-2               */
#define SH7764_SSI_A_BASE       0xff400000  /* serial sound interface A       */
#define SH7764_SSI_B_BASE       0xff500000  /* serial sound interface B       */
#define SH7764_SSI_SIZE         0x00008000
#define SH7764_SDHI_BASE        0xffe40000  /* SD host interface (undoc.)     */
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
 * DMAC. Offsets proven from the boot ROM: it loads r1 = 0xff608060 and then
 * writes SAR/DAR/DMATCR/CHCR at +0x20/+0x24/+0x28/+0x2c.
 */
#define SH7764_DMAC_DMAOR       0x0060      /* 16-bit, bit 0 = DME            */
#define SH7764_DMAC_CH0         0x0080      /* channel register block         */
#define SH7764_DMAC_CH_STRIDE   0x0010
#define SH7764_DMAC_NCHAN       8

#define SH7764_DMAC_SAR         0x00
#define SH7764_DMAC_DAR         0x04
#define SH7764_DMAC_TCR         0x08
#define SH7764_DMAC_CHCR        0x0c

#define SH7764_CHCR_DE          (1u << 0)   /* enable                         */
#define SH7764_CHCR_TE          (1u << 1)   /* transfer end                   */
#define SH7764_CHCR_TS_SHIFT    4
#define SH7764_CHCR_TS_MASK     0x7

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
    SH7764RegBank misc_a;

    /* DMAC state */
    uint32_t dmaor;
    uint32_t sar[SH7764_DMAC_NCHAN];
    uint32_t dar[SH7764_DMAC_NCHAN];
    uint32_t tcr[SH7764_DMAC_NCHAN];
    uint32_t chcr[SH7764_DMAC_NCHAN];

    uint32_t ccr;

    /* WDT state */
    uint32_t wtcnt;
    uint32_t wtcsr;
    uint32_t wrcsr;
};

#endif /* HW_SH4_SH7764_H */
