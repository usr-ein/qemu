/*
 * Analog Devices Blackfin synchronous serial port
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_BFIN_SPORT_H
#define HW_CHAR_BFIN_SPORT_H

#include "hw/core/sysbus.h"
#include "system/memory.h"
#include "chardev/char-fe.h"
#include "hw/dma/bfin_dma.h"
#include "qom/object.h"

#define TYPE_BFIN_SPORT "bfin-sport"
OBJECT_DECLARE_SIMPLE_TYPE(BfinSPORTState, BFIN_SPORT)

/*
 * A ceiling on one transfer, so a channel programmed with a wild count
 * cannot ask for an unbounded allocation. The GUI link's messages are a few
 * hundred bytes.
 */
#define BFIN_SPORT_MAX_XFER 65536

struct BfinSPORTState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    CharFrontend chr;
    BfinDMAState *dma;

    uint16_t tcr1;
    uint16_t tcr2;
    uint16_t rcr1;
    uint16_t rcr2;

    /* Which DMA channels carry the two directions. */
    uint32_t rx_channel;
    uint32_t tx_channel;

    /* Bytes arrived but not yet claimed by an armed receive channel. */
    uint8_t rx_buf[4096];
    uint32_t rx_len;
};

#endif /* HW_CHAR_BFIN_SPORT_H */
