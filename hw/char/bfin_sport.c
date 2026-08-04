/*
 * Analog Devices Blackfin synchronous serial port
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Chapter 12 of the ADSP-BF533 Blackfin Processor Hardware Reference rev 3.6.
 *
 * On the CDJ-2000NXS this is the link between the GUI processor and the main
 * processor. The GUI firmware sets it up at 0x00D0C576: SPORT1_TCR1 = 0x2602
 * and SPORT1_RCR1 = 0x6400 with SLEN = 15 in both control words, so sixteen
 * bit words in each direction, and it never touches the data registers - the
 * traffic is carried by DMA. DMA3 receives and DMA4 transmits, with SIC bits
 * 11 and 12 for their completions.
 *
 * The other end is the SH7764's SSI, whose own DMA reads from 0xA4500000 and
 * writes to 0xA4500800. Wiring the two together means carrying whole DMA
 * buffers rather than individual samples, which is what this does: a chardev
 * takes what the transmit channel was pointed at and hands what arrives to
 * the receive channel.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "exec/cpu-common.h"
#include "chardev/char-fe.h"
#include "hw/char/bfin_sport.h"

/* Table 12-1: control words at the page base, data at 0x10 and 0x18. */
#define BFIN_SPORT_TCR1     0x00
#define BFIN_SPORT_TCR2     0x04
#define BFIN_SPORT_TCLKDIV  0x08
#define BFIN_SPORT_TFSDIV   0x0c
#define BFIN_SPORT_TX       0x10
#define BFIN_SPORT_RX       0x18
#define BFIN_SPORT_RCR1     0x20
#define BFIN_SPORT_RCR2     0x24
#define BFIN_SPORT_RCLKDIV  0x28
#define BFIN_SPORT_RFSDIV   0x2c
#define BFIN_SPORT_STAT     0x30

#define BFIN_SPORT_TCR1_TSPEN  (1u << 0)
#define BFIN_SPORT_RCR1_RSPEN  (1u << 0)

/* SPORT_STAT, figure 12-13. */
#define BFIN_SPORT_STAT_RXNE   (1u << 0)   /* receive buffer not empty     */
#define BFIN_SPORT_STAT_TXHRE  (1u << 3)   /* transmit holding empty       */
#define BFIN_SPORT_STAT_TXF    (1u << 4)

/*
 * Push whatever the transmit channel was pointed at down the wire. The DMA
 * model does not move data itself - the peripheral at the end of a channel is
 * what knows when a transfer means something - so the buffer is read here and
 * the channel reports completion once it has gone.
 */
static void bfin_sport_tx_run(BfinSPORTState *s)
{
    BfinDMAChan *c;
    g_autofree uint8_t *buf = NULL;
    uint32_t len;

    if (!s->dma || !(s->tcr1 & BFIN_SPORT_TCR1_TSPEN) ||
        !bfin_dma_channel_active(s->dma, s->tx_channel)) {
        return;
    }

    c = &s->dma->chan[s->tx_channel];
    len = (uint32_t)c->x_count * 2;
    if (!len || len > BFIN_SPORT_MAX_XFER) {
        return;
    }

    buf = g_malloc(len);
    cpu_physical_memory_read(c->start_addr, buf, len);
    qemu_log_mask(LOG_UNIMP, "bfin-sport: sending %u bytes from 0x%08x\n",
                  len, c->start_addr);
    qemu_chr_fe_write_all(&s->chr, buf, len);
    bfin_dma_complete(s->dma, s->tx_channel);
}

/*
 * The receive channel is armed long before anything arrives, so bytes are
 * held until there are enough of them to fill it. Anything left over stays
 * for the next transfer rather than being dropped, because a message split
 * across two reads is still one message.
 */
static void bfin_sport_rx_deliver(BfinSPORTState *s)
{
    BfinDMAChan *c;
    uint32_t len;

    if (!s->dma || !(s->rcr1 & BFIN_SPORT_RCR1_RSPEN) ||
        !bfin_dma_channel_active(s->dma, s->rx_channel)) {
        return;
    }

    c = &s->dma->chan[s->rx_channel];
    len = (uint32_t)c->x_count * 2;
    if (!len || s->rx_len < len) {
        return;
    }

    qemu_log_mask(LOG_UNIMP, "bfin-sport: delivering %u bytes to 0x%08x\n",
                  len, c->start_addr);
    cpu_physical_memory_write(c->start_addr, s->rx_buf, len);
    s->rx_len -= len;
    memmove(s->rx_buf, s->rx_buf + len, s->rx_len);
    bfin_dma_complete(s->dma, s->rx_channel);
}

static int bfin_sport_can_receive(void *opaque)
{
    BfinSPORTState *s = opaque;

    return sizeof(s->rx_buf) - s->rx_len;
}

static void bfin_sport_receive(void *opaque, const uint8_t *buf, int size)
{
    BfinSPORTState *s = opaque;
    int room = sizeof(s->rx_buf) - s->rx_len;

    if (size > room) {
        size = room;
    }
    memcpy(s->rx_buf + s->rx_len, buf, size);
    s->rx_len += size;
    qemu_log_mask(LOG_UNIMP, "bfin-sport: %d bytes in, %u queued\n",
                  size, s->rx_len);
    bfin_sport_rx_deliver(s);
}

static void bfin_sport_dma_changed(void *opaque, unsigned chan)
{
    BfinSPORTState *s = opaque;

    if (chan == s->tx_channel) {
        bfin_sport_tx_run(s);
    } else if (chan == s->rx_channel) {
        bfin_sport_rx_deliver(s);
    }
}

static uint64_t bfin_sport_read(void *opaque, hwaddr offset, unsigned size)
{
    BfinSPORTState *s = opaque;

    switch (offset) {
    case BFIN_SPORT_TCR1:
        return s->tcr1;
    case BFIN_SPORT_TCR2:
        return s->tcr2;
    case BFIN_SPORT_RCR1:
        return s->rcr1;
    case BFIN_SPORT_RCR2:
        return s->rcr2;
    case BFIN_SPORT_STAT:
        /*
         * Nothing here holds a word in the data registers: the port is fed by
         * DMA, so the transmitter is always able to take another word and the
         * receiver never has one waiting.
         */
        return BFIN_SPORT_STAT_TXHRE;
    case BFIN_SPORT_RX:
        return 0;
    }
    return 0;
}

static void bfin_sport_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    BfinSPORTState *s = opaque;

    switch (offset) {
    case BFIN_SPORT_TCR1:
        s->tcr1 = value;
        bfin_sport_tx_run(s);
        return;
    case BFIN_SPORT_TCR2:
        s->tcr2 = value;
        return;
    case BFIN_SPORT_RCR1:
        s->rcr1 = value;
        bfin_sport_rx_deliver(s);
        return;
    case BFIN_SPORT_RCR2:
        s->rcr2 = value;
        return;
    case BFIN_SPORT_TX: {
        uint8_t b[2];

        /* A word written by hand still goes out, even though nothing does. */
        stw_le_p(b, value);
        qemu_chr_fe_write_all(&s->chr, b, sizeof(b));
        return;
    }
    }
}

static const MemoryRegionOps bfin_sport_ops = {
    .read = bfin_sport_read,
    .write = bfin_sport_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

static void bfin_sport_reset_hold(Object *obj, ResetType type)
{
    BfinSPORTState *s = BFIN_SPORT(obj);

    s->tcr1 = 0;
    s->tcr2 = 0;
    s->rcr1 = 0;
    s->rcr2 = 0;
    s->rx_len = 0;
}

static void bfin_sport_realize(DeviceState *dev, Error **errp)
{
    BfinSPORTState *s = BFIN_SPORT(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bfin_sport_ops, s,
                          "bfin-sport", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    if (s->dma) {
        s->dma->chan_changed = bfin_sport_dma_changed;
        s->dma->chan_changed_opaque = s;
    }

    qemu_chr_fe_set_handlers(&s->chr, bfin_sport_can_receive,
                             bfin_sport_receive, NULL, NULL, s, NULL, true);
}

static const VMStateDescription vmstate_bfin_sport = {
    .name = "bfin-sport",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(tcr1, BfinSPORTState),
        VMSTATE_UINT16(tcr2, BfinSPORTState),
        VMSTATE_UINT16(rcr1, BfinSPORTState),
        VMSTATE_UINT16(rcr2, BfinSPORTState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property bfin_sport_properties[] = {
    DEFINE_PROP_CHR("chardev", BfinSPORTState, chr),
    DEFINE_PROP_UINT32("rx-channel", BfinSPORTState, rx_channel, 3),
    DEFINE_PROP_UINT32("tx-channel", BfinSPORTState, tx_channel, 4),
    DEFINE_PROP_LINK("dma", BfinSPORTState, dma, TYPE_BFIN_DMA, BfinDMAState *),
};

static void bfin_sport_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = bfin_sport_realize;
    dc->vmsd = &vmstate_bfin_sport;
    device_class_set_props(dc, bfin_sport_properties);
    rc->phases.hold = bfin_sport_reset_hold;
}

static const TypeInfo bfin_sport_types[] = {
    {
        .name          = TYPE_BFIN_SPORT,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(BfinSPORTState),
        .class_init    = bfin_sport_class_init,
    },
};

DEFINE_TYPES(bfin_sport_types)
