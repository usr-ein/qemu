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

/*
 * Work out the frame the controller sends and the part of it the panel shows.
 *
 * Everything here comes from the hardware: PPI_COUNT is samples per line
 * minus one, and the number of lines is PPI_FRAME or, when the frame syncs
 * come from elsewhere and PPI_FRAME is left at zero, the DMA channel's 2D
 * shape. The one thing the controller cannot tell us is how many of those
 * lines the panel lights, because the blanking lines are ordinary DMA traffic
 * as far as it is concerned; the board supplies that, and the blanking is
 * taken to lead the frame, which is where a vertical sync pulse puts it.
 */
static void bfin_ppi_geometry(BfinPPIState *s, uint32_t *w, uint32_t *h,
                              uint32_t *skip)
{
    uint32_t lines = s->frame;

    *w = s->count + 1;

    if (lines == 0 && s->dma && s->dma_channel < BFIN_DMA_CHANNELS) {
        lines = s->dma->chan[s->dma_channel].y_count;
    }

    if (s->panel_lines && s->panel_lines <= lines) {
        *skip = lines - s->panel_lines;
        *h = s->panel_lines;
    } else {
        *skip = 0;
        *h = lines;
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
    uint32_t w, h, y, x, skip;
    unsigned bits;
    hwaddr src;
    g_autofree uint8_t *line = NULL;

    if (!ppi_enabled(s) || !s->dma ||
        !bfin_dma_channel_active(s->dma, s->dma_channel)) {
        return;
    }

    bfin_ppi_geometry(s, &w, &h, &skip);
    bits = ppi_bits_per_sample(s);
    if (!w || !h || w > 4096 || h > 4096) {
        return;
    }

    if (!s->announced) {
        qemu_log_mask(LOG_UNIMP,
                      "bfin-ppi: panel is %ux%u, %u bits per sample, "
                      "%u blanking lines, frame buffer at 0x%08x\n",
                      w, h, bits, skip,
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
    src = c->start_addr + (hwaddr)skip * w * (bits <= 8 ? 1 : 2);
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

/*
 * A frame's worth of pixels has been clocked out.
 *
 * Real timing comes from PPI_CLK, an external clock this model has no view
 * of, so the frame period is the panel's refresh rate rather than anything
 * derived from the registers. What the firmware sees is what matters: the
 * DMA channel finishes its pass and raises the completion interrupt it was
 * configured to raise, once per frame, which is the beat the drawing code
 * runs on.
 */
static void bfin_ppi_frame_done(void *opaque)
{
    BfinPPIState *s = opaque;

    if (!ppi_enabled(s) || !s->dma || !s->refresh_hz) {
        return;
    }

    bfin_dma_complete(s->dma, s->dma_channel);
    timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / s->refresh_hz);
}

static void bfin_ppi_update_frame_timer(BfinPPIState *s)
{
    if (!s->frame_timer) {
        return;
    }
    if (ppi_enabled(s) && s->refresh_hz) {
        timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / s->refresh_hz);
    } else {
        timer_del(s->frame_timer);
    }
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
        bfin_ppi_update_frame_timer(s);
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
    if (s->frame_timer) {
        timer_del(s->frame_timer);
    }
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

    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bfin_ppi_frame_done, s);

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
    DEFINE_PROP_UINT32("panel-lines", BfinPPIState, panel_lines, 0),
    DEFINE_PROP_UINT32("refresh-hz", BfinPPIState, refresh_hz, 60),
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
