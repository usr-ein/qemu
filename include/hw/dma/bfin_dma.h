/*
 * Analog Devices Blackfin DMA controller
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DMA_BFIN_DMA_H
#define HW_DMA_BFIN_DMA_H

#include "hw/core/sysbus.h"
#include "system/memory.h"
#include "qom/object.h"

#define TYPE_BFIN_DMA "bfin-dma"
OBJECT_DECLARE_SIMPLE_TYPE(BfinDMAState, BFIN_DMA)

/*
 * Eight peripheral DMA channels at 0x40 apart, then the memory DMA streams.
 * Base addresses are from table B-13 of the ADSP-BF533 Blackfin Processor
 * Hardware Reference rev 3.6, register offsets from table B-14.
 */
#define BFIN_DMA_CHANNELS       8
#define BFIN_DMA_CHAN_STRIDE    0x40

#define BFIN_DMA_NEXT_DESC_PTR  0x00
#define BFIN_DMA_START_ADDR     0x04
#define BFIN_DMA_CONFIG         0x08
#define BFIN_DMA_X_COUNT        0x10
#define BFIN_DMA_X_MODIFY       0x14
#define BFIN_DMA_Y_COUNT        0x18
#define BFIN_DMA_Y_MODIFY       0x1c
#define BFIN_DMA_CURR_DESC_PTR  0x20
#define BFIN_DMA_CURR_ADDR      0x24
#define BFIN_DMA_IRQ_STATUS     0x28
#define BFIN_DMA_PERIPHERAL_MAP 0x2c
#define BFIN_DMA_CURR_X_COUNT   0x30
#define BFIN_DMA_CURR_Y_COUNT   0x38

/* DMAx_CONFIG bits. */
#define BFIN_DMA_CFG_DMAEN      (1u << 0)
#define BFIN_DMA_CFG_WNR        (1u << 1)   /* 0 = memory read (transmit)   */
#define BFIN_DMA_CFG_WDSIZE_SH  2
#define BFIN_DMA_CFG_WDSIZE_MSK 0x3
#define BFIN_DMA_CFG_DMA2D      (1u << 4)

/* DMAx_IRQ_STATUS bits. */
#define BFIN_DMA_IRQ_DONE       (1u << 0)
#define BFIN_DMA_IRQ_ERR        (1u << 1)
#define BFIN_DMA_IRQ_DFETCH     (1u << 2)
#define BFIN_DMA_IRQ_RUN        (1u << 3)

typedef struct BfinDMAChan {
    uint32_t next_desc_ptr;
    uint32_t start_addr;
    uint16_t config;
    uint16_t x_count;
    uint16_t x_modify;
    uint16_t y_count;
    uint16_t y_modify;
    uint32_t curr_desc_ptr;
    uint32_t curr_addr;
    uint16_t irq_status;
    uint16_t peripheral_map;
    uint16_t curr_x_count;
    uint16_t curr_y_count;
} BfinDMAChan;

struct BfinDMAState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    BfinDMAChan chan[BFIN_DMA_CHANNELS];
    qemu_irq irq[BFIN_DMA_CHANNELS];

    /*
     * Set when a channel's configuration changes, so a consumer such as the
     * PPI can pick up a new frame buffer address or geometry.
     */
    void (*chan_changed)(void *opaque, unsigned chan);
    void *chan_changed_opaque;
};

/*
 * Describe a channel to a peripheral that is fed by it. Returns false if the
 * channel is not enabled.
 */
bool bfin_dma_channel_active(BfinDMAState *s, unsigned chan);

#endif /* HW_DMA_BFIN_DMA_H */
