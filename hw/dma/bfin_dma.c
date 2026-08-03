/*
 * Analog Devices Blackfin DMA controller
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Chapter 9 of the ADSP-BF533 Blackfin Processor Hardware Reference rev 3.6.
 *
 * Only the register file and the descriptor-less ("autobuffer" and "stop")
 * flow modes are modelled. That is enough for the peripheral this controller
 * exists to feed here - the PPI, which streams a frame buffer continuously
 * and never actually needs the transfer to be simulated cycle by cycle,
 * because the display consumer reads guest memory directly.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "hw/dma/bfin_dma.h"

static BfinDMAChan *chan_of(BfinDMAState *s, hwaddr offset, hwaddr *reg)
{
    unsigned n = offset / BFIN_DMA_CHAN_STRIDE;

    if (n >= BFIN_DMA_CHANNELS) {
        return NULL;
    }
    *reg = offset % BFIN_DMA_CHAN_STRIDE;
    return &s->chan[n];
}

bool bfin_dma_channel_active(BfinDMAState *s, unsigned chan)
{
    return chan < BFIN_DMA_CHANNELS &&
           (s->chan[chan].config & BFIN_DMA_CFG_DMAEN);
}

static uint64_t bfin_dma_read(void *opaque, hwaddr offset, unsigned size)
{
    BfinDMAState *s = opaque;
    hwaddr reg;
    BfinDMAChan *c = chan_of(s, offset, &reg);

    if (!c) {
        qemu_log_mask(LOG_UNIMP, "bfin-dma: read from 0x%" HWADDR_PRIx
                      " (memory DMA is not modelled)\n", offset);
        return 0;
    }

    switch (reg) {
    case BFIN_DMA_NEXT_DESC_PTR:
        return c->next_desc_ptr;
    case BFIN_DMA_START_ADDR:
        return c->start_addr;
    case BFIN_DMA_CONFIG:
        return c->config;
    case BFIN_DMA_X_COUNT:
        return c->x_count;
    case BFIN_DMA_X_MODIFY:
        return c->x_modify;
    case BFIN_DMA_Y_COUNT:
        return c->y_count;
    case BFIN_DMA_Y_MODIFY:
        return c->y_modify;
    case BFIN_DMA_CURR_DESC_PTR:
        return c->curr_desc_ptr;
    case BFIN_DMA_CURR_ADDR:
        return c->curr_addr;
    case BFIN_DMA_IRQ_STATUS:
        return c->irq_status;
    case BFIN_DMA_PERIPHERAL_MAP:
        return c->peripheral_map;
    case BFIN_DMA_CURR_X_COUNT:
        return c->curr_x_count;
    case BFIN_DMA_CURR_Y_COUNT:
        return c->curr_y_count;
    }

    qemu_log_mask(LOG_UNIMP, "bfin-dma: read from reserved offset 0x%"
                  HWADDR_PRIx "\n", reg);
    return 0;
}

static void bfin_dma_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    BfinDMAState *s = opaque;
    hwaddr reg;
    BfinDMAChan *c = chan_of(s, offset, &reg);
    unsigned n = offset / BFIN_DMA_CHAN_STRIDE;

    if (!c) {
        qemu_log_mask(LOG_UNIMP, "bfin-dma: write to 0x%" HWADDR_PRIx
                      " (memory DMA is not modelled)\n", offset);
        return;
    }

    switch (reg) {
    case BFIN_DMA_NEXT_DESC_PTR:
        c->next_desc_ptr = value;
        return;
    case BFIN_DMA_START_ADDR:
        c->start_addr = value;
        break;
    case BFIN_DMA_CONFIG:
        c->config = value;
        if (value & BFIN_DMA_CFG_DMAEN) {
            /*
             * Starting a channel latches the working copies. Nothing is
             * transferred here: a peripheral that consumes the stream reads
             * guest memory itself, and one that produces it has no source.
             */
            c->curr_addr = c->start_addr;
            c->curr_x_count = c->x_count;
            c->curr_y_count = c->y_count;
            c->irq_status |= BFIN_DMA_IRQ_RUN;
        } else {
            c->irq_status &= ~BFIN_DMA_IRQ_RUN;
        }
        break;
    case BFIN_DMA_X_COUNT:
        c->x_count = value;
        break;
    case BFIN_DMA_X_MODIFY:
        c->x_modify = value;
        break;
    case BFIN_DMA_Y_COUNT:
        c->y_count = value;
        break;
    case BFIN_DMA_Y_MODIFY:
        c->y_modify = value;
        break;
    case BFIN_DMA_CURR_DESC_PTR:
        c->curr_desc_ptr = value;
        return;
    case BFIN_DMA_CURR_ADDR:
        c->curr_addr = value;
        return;
    case BFIN_DMA_IRQ_STATUS:
        /* Writing a one clears the sticky completion and error bits. */
        c->irq_status &= ~(value & (BFIN_DMA_IRQ_DONE | BFIN_DMA_IRQ_ERR));
        if (!(c->irq_status & (BFIN_DMA_IRQ_DONE | BFIN_DMA_IRQ_ERR))) {
            qemu_set_irq(s->irq[n], 0);
        }
        return;
    case BFIN_DMA_PERIPHERAL_MAP:
        c->peripheral_map = value;
        break;
    case BFIN_DMA_CURR_X_COUNT:
        c->curr_x_count = value;
        return;
    case BFIN_DMA_CURR_Y_COUNT:
        c->curr_y_count = value;
        return;
    default:
        qemu_log_mask(LOG_UNIMP, "bfin-dma: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", reg);
        return;
    }

    if (s->chan_changed) {
        s->chan_changed(s->chan_changed_opaque, n);
    }
}

static const MemoryRegionOps bfin_dma_ops = {
    .read = bfin_dma_read,
    .write = bfin_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

static void bfin_dma_reset_hold(Object *obj, ResetType type)
{
    BfinDMAState *s = BFIN_DMA(obj);

    memset(s->chan, 0, sizeof(s->chan));
}

static void bfin_dma_realize(DeviceState *dev, Error **errp)
{
    BfinDMAState *s = BFIN_DMA(dev);
    unsigned i;

    memory_region_init_io(&s->iomem, OBJECT(dev), &bfin_dma_ops, s,
                          "bfin-dma",
                          BFIN_DMA_CHANNELS * BFIN_DMA_CHAN_STRIDE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    for (i = 0; i < BFIN_DMA_CHANNELS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[i]);
    }
}

static const VMStateDescription vmstate_bfin_dma_chan = {
    .name = "bfin-dma-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(next_desc_ptr, BfinDMAChan),
        VMSTATE_UINT32(start_addr, BfinDMAChan),
        VMSTATE_UINT16(config, BfinDMAChan),
        VMSTATE_UINT16(x_count, BfinDMAChan),
        VMSTATE_UINT16(x_modify, BfinDMAChan),
        VMSTATE_UINT16(y_count, BfinDMAChan),
        VMSTATE_UINT16(y_modify, BfinDMAChan),
        VMSTATE_UINT32(curr_desc_ptr, BfinDMAChan),
        VMSTATE_UINT32(curr_addr, BfinDMAChan),
        VMSTATE_UINT16(irq_status, BfinDMAChan),
        VMSTATE_UINT16(peripheral_map, BfinDMAChan),
        VMSTATE_UINT16(curr_x_count, BfinDMAChan),
        VMSTATE_UINT16(curr_y_count, BfinDMAChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_bfin_dma = {
    .name = "bfin-dma",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(chan, BfinDMAState, BFIN_DMA_CHANNELS, 1,
                             vmstate_bfin_dma_chan, BfinDMAChan),
        VMSTATE_END_OF_LIST()
    }
};

static void bfin_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = bfin_dma_realize;
    dc->vmsd = &vmstate_bfin_dma;
    rc->phases.hold = bfin_dma_reset_hold;
}

static const TypeInfo bfin_dma_types[] = {
    {
        .name          = TYPE_BFIN_DMA,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(BfinDMAState),
        .class_init    = bfin_dma_class_init,
    },
};

DEFINE_TYPES(bfin_dma_types)
