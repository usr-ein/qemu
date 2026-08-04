/*
 * Analog Devices Blackfin DMA controller
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Chapter 9 of the ADSP-BF533 Blackfin Processor Hardware Reference rev 3.6.
 *
 * The transfer itself is not simulated cycle by cycle: the peripheral this
 * controller exists to feed here is the PPI, and the display consumer reads
 * guest memory directly. What does have to be modelled is where the transfer
 * starts, because a channel in a descriptor flow mode never has its start
 * address written by the processor at all - the controller fetches it.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "exec/cpu-common.h"
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

/*
 * Load the descriptor a channel was started on.
 *
 * The elements are a fixed sequence of 16-bit words and NDSIZE says how many
 * of them the controller reads, so a descriptor carries only the fields that
 * differ from the register file. The two flow modes that name a next
 * descriptor start the sequence with that pointer: the large model with both
 * halves, the small model with the low half only, taking the upper half from
 * the pointer it was reached by. A descriptor array has no pointer at all and
 * starts at the start address.
 *
 * The GUI firmware programs the frame buffer this way and never writes
 * DMA0_START_ADDR, so without this the PPI has nothing to display.
 */
static void bfin_dma_fetch_descriptor(BfinDMAChan *c)
{
    unsigned flow = (c->config >> BFIN_DMA_CFG_FLOW_SH) & BFIN_DMA_CFG_FLOW_MSK;
    unsigned ndsize = (c->config >> BFIN_DMA_CFG_NDSIZE_SH) &
                      BFIN_DMA_CFG_NDSIZE_MSK;
    uint32_t addr = c->next_desc_ptr;
    uint16_t elem[9];
    unsigned i, first;

    switch (flow) {
    case BFIN_DMA_FLOW_LARGE:
        first = 0;
        break;
    case BFIN_DMA_FLOW_SMALL:
        first = 1;
        break;
    case BFIN_DMA_FLOW_ARRAY:
        first = 2;
        break;
    default:
        return;
    }

    if (ndsize == 0 || first + ndsize > ARRAY_SIZE(elem)) {
        return;
    }

    c->curr_desc_ptr = addr;
    for (i = 0; i < ndsize; i++) {
        uint8_t buf[2];

        cpu_physical_memory_read(addr + i * 2, buf, sizeof(buf));
        elem[first + i] = lduw_le_p(buf);
    }

    /*
     * Elements past the ones fetched keep the value already in the register
     * file, which is how a short descriptor works.
     */
    for (i = first + ndsize; i < ARRAY_SIZE(elem); i++) {
        elem[i] = 0;
    }

    if (first == 0 && ndsize >= 2) {
        c->next_desc_ptr = elem[0] | ((uint32_t)elem[1] << 16);
    } else if (first == 1) {
        c->next_desc_ptr = (addr & 0xffff0000) | elem[1];
    }
    if (first + ndsize > 3) {
        c->start_addr = elem[2] | ((uint32_t)elem[3] << 16);
    }
    if (first + ndsize > 4) {
        c->config = elem[4];
    }
    if (first + ndsize > 5) {
        c->x_count = elem[5];
    }
    if (first + ndsize > 6) {
        c->x_modify = elem[6];
    }
    if (first + ndsize > 7) {
        c->y_count = elem[7];
    }
    if (first + ndsize > 8) {
        c->y_modify = elem[8];
    }
}

void bfin_dma_complete(BfinDMAState *s, unsigned chan)
{
    BfinDMAChan *c;

    if (!bfin_dma_channel_active(s, chan)) {
        return;
    }

    c = &s->chan[chan];
    c->irq_status |= BFIN_DMA_IRQ_DONE;
    if (c->config & BFIN_DMA_CFG_DI_EN) {
        qemu_set_irq(s->irq[chan], 1);
    }
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
            bfin_dma_fetch_descriptor(c);
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
