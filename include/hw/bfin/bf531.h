/*
 * Analog Devices ADSP-BF531 system on chip
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_BFIN_BF531_H
#define HW_BFIN_BF531_H

#include "hw/core/sysbus.h"
#include "system/memory.h"
/* The SoC embeds the CPU object, so the full type is needed, not just QOM. */
#include "target/bfin/cpu.h"
#include "hw/dma/bfin_dma.h"
#include "hw/display/bfin_ppi.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_BF531 "bf531"
OBJECT_DECLARE_SIMPLE_TYPE(BF531State, BF531)

/*
 * Memory map, from figure 3 of the ADSP-BF531/BF532/BF533 datasheet rev I.
 * This is the BF531 map specifically: its L1 instruction memory starts at
 * 0xFFA08000, where the BF533 has 64 KB starting at 0xFFA00000.
 */
#define BF531_SDRAM_BASE        0x00000000
#define BF531_SDRAM_MAX         0x08000000  /* 16 to 128 MB                 */

#define BF531_ASYNC_BASE        0x20000000
#define BF531_ASYNC_BANK_SIZE   0x00100000
#define BF531_ASYNC_BANKS       4

#define BF531_BOOTROM_BASE      0xef000000
#define BF531_BOOTROM_SIZE      0x00001000

#define BF531_L1_DATA_A_BASE    0xff804000  /* SRAM/cache, 16 KB            */
#define BF531_L1_DATA_A_SIZE    0x00004000
#define BF531_L1_INST_BASE      0xffa08000  /* SRAM, 16 KB                  */
#define BF531_L1_INST_SIZE      0x00004000
#define BF531_L1_INST_C_BASE    0xffa10000  /* SRAM/cache, 16 KB            */
#define BF531_L1_INST_C_SIZE    0x00004000
#define BF531_L1_SCRATCH_BASE   0xffb00000  /* 4 KB                         */
#define BF531_L1_SCRATCH_SIZE   0x00001000

#define BF531_SYS_MMR_BASE      0xffc00000
#define BF531_SYS_MMR_SIZE      0x00200000
#define BF531_CORE_MMR_BASE     0xffe00000
#define BF531_CORE_MMR_SIZE     0x00200000

/*
 * System peripherals the GUI firmware demonstrably drives, recovered by
 * tracking pointer construction through the decoded instruction stream (see
 * docs/blackfin-gui.md in the parent project). Register layouts are from the
 * ADSP-BF533 Blackfin Processor Hardware Reference rev 3.6, which covers the
 * BF531 as the same peripheral set minus the parts it does not carry.
 */
#define BF531_SIC_BASE          0xffc00100
#define BF531_WDOG_BASE         0xffc00200
#define BF531_RTC_BASE          0xffc00300
#define BF531_UART_BASE         0xffc00400
#define BF531_SPI_BASE          0xffc00500
#define BF531_TIMER_BASE        0xffc00600
#define BF531_GPIO_BASE         0xffc00700
#define BF531_SPORT0_BASE       0xffc00800
#define BF531_SPORT1_BASE       0xffc00900
#define BF531_EBIU_BASE         0xffc00a00
#define BF531_DMA_TC_BASE       0xffc00b00
#define BF531_DMA_BASE          0xffc00c00
#define BF531_PPI_BASE          0xffc01000
#define BF531_PERIPH_PAGE       0x00000100

/* Core MMRs, chapter 6 and appendix B of the hardware reference. */
#define BF531_DMEM_CONTROL      0xffe00004
#define BF531_DTEST_COMMAND     0xffe00300
#define BF531_IMEM_CONTROL      0xffe01004
#define BF531_ITEST_COMMAND     0xffe01300
#define BF531_EVT0              0xffe02000
#define BF531_IMASK             0xffe02104
#define BF531_IPEND             0xffe02108
#define BF531_ILAT              0xffe0210c
#define BF531_TCNTL             0xffe03000
#define BF531_TPERIOD           0xffe03004
#define BF531_TSCALE            0xffe03008
#define BF531_TCOUNT            0xffe0300c

struct BF531State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    ArchCPU cpu;
    BfinDMAState dma;
    BfinPPIState ppi;

    MemoryRegion *sysmem;
    MemoryRegion sdram;
    MemoryRegion l1_data_a;
    MemoryRegion l1_inst;
    MemoryRegion l1_inst_c;
    MemoryRegion l1_scratch;
    MemoryRegion core_mmr;
    MemoryRegion spi;
    MemoryRegion async;
    MemoryRegion gpio;
    MemoryRegion trace;

    QEMUTimer *core_timer;
    uint64_t core_timer_next;
    uint32_t cclk_hz;

    uint32_t trace_base;
    uint32_t trace_size;

    uint16_t gpio_dir;
    uint16_t gpio_inen;
    uint16_t gpio_out;
    uint16_t gpio_in;

    uint32_t spi_ctl;
    uint32_t spi_flg;
    uint32_t spi_tdbr;
    uint32_t spi_baud;

    uint64_t sdram_size;
};

#endif /* HW_BFIN_BF531_H */
