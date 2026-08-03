/*
 * Analog Devices Blackfin parallel peripheral interface, display output
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Chapter 11 of the ADSP-BF533 Blackfin Processor Hardware Reference rev 3.6.
 *
 * The PPI is a generic parallel port; when configured for output with frame
 * syncs and fed by DMA it is how a Blackfin drives an LCD panel. Rather than
 * simulate the pixel clock, this model treats the DMA channel's source as a
 * frame buffer and reads it from guest memory on each display refresh, which
 * is what every other framebuffer device in QEMU does.
 *
 * Geometry is taken from the hardware rather than assumed: PPI_COUNT holds
 * samples per line minus one and PPI_FRAME holds lines per frame, so a panel
 * of any size the firmware happens to program will come out right.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "exec/cpu-common.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/display/bfin_ppi.h"

static unsigned ppi_bits_per_sample(BfinPPIState *s)
{
    /* DLEN[2:0]: 000 is 8 bits, then 10, 11, 12, 13, 14, 15, 16. */
    unsigned dlen = (s->control >> PPI_CTL_DLEN_SH) & PPI_CTL_DLEN_MSK;

    return dlen == 0 ? 8 : dlen + 9;
}

static bool ppi_enabled(BfinPPIState *s)
{
    return (s->control & PPI_CTL_PORT_EN) &&
           (s->control & PPI_CTL_PORT_DIR);
}

static void bfin_ppi_geometry(BfinPPIState *s, uint32_t *w, uint32_t *h)
{
    /* PPI_COUNT is samples per line minus one; PPI_FRAME is lines. */
    *w = s->count + 1;
    *h = s->frame;

    /*
     * Some panels are driven with the line count left in the DMA descriptor
     * rather than PPI_FRAME. Fall back to the DMA 2D shape when PPI_FRAME
     * has not been programmed.
     */
    if (*h == 0 && s->dma && s->dma_channel < BFIN_DMA_CHANNELS) {
        *h = s->dma->chan[s->dma_channel].y_count;
    }
}

static void bfin_ppi_invalidate(void *opaque)
{
    BfinPPIState *s = opaque;

    s->invalid = true;
}

static void bfin_ppi_update_display(void *opaque)
{
    BfinPPIState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    BfinDMAChan *c;
    uint32_t w, h, y, x;
    unsigned bits;
    hwaddr src;
    g_autofree uint8_t *line = NULL;

    if (!ppi_enabled(s) || !s->dma ||
        !bfin_dma_channel_active(s->dma, s->dma_channel)) {
        return;
    }

    bfin_ppi_geometry(s, &w, &h);
    bits = ppi_bits_per_sample(s);
    if (!w || !h || w > 4096 || h > 4096) {
        return;
    }

    if (!s->announced) {
        qemu_log_mask(LOG_UNIMP,
                      "bfin-ppi: panel is %ux%u, %u bits per sample, "
                      "frame buffer at 0x%08x\n", w, h, bits,
                      s->dma->chan[s->dma_channel].start_addr);
        s->announced = true;
    }

    if (w != s->width || h != s->height || bits != (unsigned)s->bpp) {
        s->width = w;
        s->height = h;
        s->bpp = bits;
        qemu_console_resize(s->con, w, h);
        surface = qemu_console_surface(s->con);
        s->invalid = true;
    }

    c = &s->dma->chan[s->dma_channel];
    src = c->start_addr;
    line = g_malloc(w * 2);

    for (y = 0; y < h; y++) {
        uint32_t *dst = (uint32_t *)(surface_data(surface) +
                                     y * surface_stride(surface));

        if (bits <= 8) {
            cpu_physical_memory_read(src, line, w);
            for (x = 0; x < w; x++) {
                uint8_t v = line[x];

                dst[x] = rgb_to_pixel32(v, v, v);
            }
            src += w;
        } else {
            cpu_physical_memory_read(src, line, w * 2);
            for (x = 0; x < w; x++) {
                uint16_t v = lduw_le_p(line + x * 2);

                dst[x] = rgb_to_pixel32(((v >> 11) & 0x1f) << 3,
                                        ((v >> 5) & 0x3f) << 2,
                                        (v & 0x1f) << 3);
            }
            src += w * 2;
        }
    }

    dpy_gfx_update(s->con, 0, 0, w, h);
    s->invalid = false;
}

static const GraphicHwOps bfin_ppi_gfx_ops = {
    .invalidate = bfin_ppi_invalidate,
    .gfx_update = bfin_ppi_update_display,
};

static uint64_t bfin_ppi_read(void *opaque, hwaddr offset, unsigned size)
{
    BfinPPIState *s = opaque;

    switch (offset) {
    case BFIN_PPI_CONTROL:
        return s->control;
    case BFIN_PPI_STATUS:
        return s->status;
    case BFIN_PPI_COUNT:
        return s->count;
    case BFIN_PPI_DELAY:
        return s->delay;
    case BFIN_PPI_FRAME:
        return s->frame;
    }

    qemu_log_mask(LOG_UNIMP, "bfin-ppi: read from reserved offset 0x%"
                  HWADDR_PRIx "\n", offset);
    return 0;
}

static void bfin_ppi_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    BfinPPIState *s = opaque;

    switch (offset) {
    case BFIN_PPI_CONTROL:
        s->control = value;
        s->announced = false;
        s->invalid = true;
        return;
    case BFIN_PPI_STATUS:
        /* The error bits are sticky and write-one-to-clear. */
        s->status &= ~(uint16_t)value;
        return;
    case BFIN_PPI_COUNT:
        s->count = value;
        s->announced = false;
        return;
    case BFIN_PPI_DELAY:
        s->delay = value;
        return;
    case BFIN_PPI_FRAME:
        s->frame = value;
        s->announced = false;
        return;
    }

    qemu_log_mask(LOG_UNIMP, "bfin-ppi: write to reserved offset 0x%"
                  HWADDR_PRIx "\n", offset);
}

static const MemoryRegionOps bfin_ppi_ops = {
    .read = bfin_ppi_read,
    .write = bfin_ppi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

static void bfin_ppi_dma_changed(void *opaque, unsigned chan)
{
    BfinPPIState *s = opaque;

    if (chan == s->dma_channel) {
        s->invalid = true;
        s->announced = false;
    }
}

static void bfin_ppi_reset_hold(Object *obj, ResetType type)
{
    BfinPPIState *s = BFIN_PPI(obj);

    s->control = 0;
    s->status = 0;
    s->count = 0;
    s->delay = 0;
    s->frame = 0;
    s->width = 0;
    s->height = 0;
    s->bpp = 0;
    s->invalid = true;
    s->announced = false;
}

static void bfin_ppi_realize(DeviceState *dev, Error **errp)
{
    BfinPPIState *s = BFIN_PPI(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bfin_ppi_ops, s,
                          "bfin-ppi", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    if (s->dma) {
        s->dma->chan_changed = bfin_ppi_dma_changed;
        s->dma->chan_changed_opaque = s;
    }

    s->con = graphic_console_init(dev, 0, &bfin_ppi_gfx_ops, s);
}

static const VMStateDescription vmstate_bfin_ppi = {
    .name = "bfin-ppi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(control, BfinPPIState),
        VMSTATE_UINT16(status, BfinPPIState),
        VMSTATE_UINT16(count, BfinPPIState),
        VMSTATE_UINT16(delay, BfinPPIState),
        VMSTATE_UINT16(frame, BfinPPIState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property bfin_ppi_properties[] = {
    DEFINE_PROP_UINT32("dma-channel", BfinPPIState, dma_channel, 0),
    DEFINE_PROP_LINK("dma", BfinPPIState, dma, TYPE_BFIN_DMA, BfinDMAState *),
};

static void bfin_ppi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = bfin_ppi_realize;
    dc->vmsd = &vmstate_bfin_ppi;
    device_class_set_props(dc, bfin_ppi_properties);
    rc->phases.hold = bfin_ppi_reset_hold;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo bfin_ppi_types[] = {
    {
        .name          = TYPE_BFIN_PPI,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(BfinPPIState),
        .class_init    = bfin_ppi_class_init,
    },
};

DEFINE_TYPES(bfin_ppi_types)
