/*
 * Analog Devices Blackfin parallel peripheral interface, display output
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_BFIN_PPI_H
#define HW_DISPLAY_BFIN_PPI_H

#include "hw/core/sysbus.h"
#include "system/memory.h"
#include "ui/console.h"
#include "hw/dma/bfin_dma.h"
#include "qom/object.h"

#define TYPE_BFIN_PPI "bfin-ppi"
OBJECT_DECLARE_SIMPLE_TYPE(BfinPPIState, BFIN_PPI)

/* Chapter 11 and table B-1 of the ADSP-BF533 hardware reference rev 3.6. */
#define BFIN_PPI_CONTROL 0x00
#define BFIN_PPI_STATUS  0x04
#define BFIN_PPI_COUNT   0x08
#define BFIN_PPI_DELAY   0x0c
#define BFIN_PPI_FRAME   0x10

/* PPI_CONTROL, figure 11-1. */
#define PPI_CTL_PORT_EN     (1u << 0)
#define PPI_CTL_PORT_DIR    (1u << 1)   /* 1 = transmit, i.e. driving a panel */
#define PPI_CTL_XFR_TYPE_SH 2
#define PPI_CTL_PORT_CFG_SH 4
#define PPI_CTL_FLD_SEL     (1u << 6)
#define PPI_CTL_PACK_EN     (1u << 7)
#define PPI_CTL_SKIP_EN     (1u << 8)
#define PPI_CTL_SKIP_EO     (1u << 9)
#define PPI_CTL_DLEN_SH     10
#define PPI_CTL_DLEN_MSK    0x7
#define PPI_CTL_POLC        (1u << 13)
#define PPI_CTL_POLS        (1u << 14)

struct BfinPPIState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    QemuConsole *con;
    BfinDMAState *dma;
    qemu_irq irq;

    uint16_t control;
    uint16_t status;
    uint16_t count;
    uint16_t delay;
    uint16_t frame;

    /* Which DMA channel streams the frame buffer. */
    uint32_t dma_channel;

    /* Geometry currently presented, so a change can resize the window. */
    uint32_t width;
    uint32_t height;
    int bpp;
    bool invalid;
    bool announced;
};

#endif /* HW_DISPLAY_BFIN_PPI_H */
