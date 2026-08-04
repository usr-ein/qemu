/*
 * Renesas SH7764 Ethernet controller (EtherC) and its DMA engine (E-DMAC)
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Register addresses and widths are from sections 19 and 20 of the SH7764
 * hardware manual. The descriptor semantics are from the manual too, but were
 * cross-checked against the CDJ-2000NXS firmware's own driver, which builds
 * both rings explicitly: sixteen 16-byte entries each, status word 0x30000000
 * on a normal entry and 0x70000000 on the last one to wrap, ownership handed
 * over by setting bit 31, and 1600-byte receive buffers.
 *
 * Only what a driver can observe is modelled. Transmission completes
 * synchronously inside the EDTRR write, because a driver that polls the
 * ownership bit or waits for the interrupt cannot tell the difference, and
 * modelling bus arbitration would buy nothing.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/net/sh7764_eth.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "trace.h"

/*
 * The driver hands the engine addresses it has already masked down out of the
 * P1/P2 windows, but mask again rather than trusting that: a stray region bit
 * would otherwise turn into a wild DMA.
 */
static inline hwaddr sh7764_eth_addr(uint32_t addr)
{
    if (addr >= 0x80000000 && addr < 0xe0000000) {
        return addr & 0x1fffffff;
    }
    return addr;
}

static void sh7764_eth_update_irq(SH7764EthState *s)
{
    qemu_set_irq(s->irq, (s->eesr & s->eesipr) != 0);
}

static uint32_t sh7764_eth_desc_read(hwaddr desc, unsigned word)
{
    uint32_t v;

    address_space_read(&address_space_memory, desc + word * 4,
                       MEMTXATTRS_UNSPECIFIED, &v, 4);
    return le32_to_cpu(v);
}

static void sh7764_eth_desc_write(hwaddr desc, unsigned word, uint32_t val)
{
    uint32_t v = cpu_to_le32(val);

    address_space_write(&address_space_memory, desc + word * 4,
                        MEMTXATTRS_UNSPECIFIED, &v, 4);
}

/*
 * Walk the transmit ring from wherever we stopped last time, sending every
 * descriptor the driver has handed us, and stop at the first one it still
 * owns. EDTRR.TR is cleared on the way out: the hardware drops it when the
 * list runs dry, and the driver's kick routine tests exactly that before
 * writing it again.
 */
static void sh7764_eth_transmit(SH7764EthState *s)
{
    uint8_t buf[SH7764_ETH_MAX_FRAME];
    unsigned guard;

    if (!(s->edtrr & SH7764_EDTRR_TR) || !s->tdlar) {
        return;
    }

    for (guard = 0; guard < 64; guard++) {
        hwaddr desc = sh7764_eth_addr(s->tdlar) + s->tx_cursor;
        uint32_t status = sh7764_eth_desc_read(desc, 0);
        uint32_t len, addr;

        if (!(status & SH7764_DESC_ACT)) {
            break;
        }

        len = sh7764_eth_desc_read(desc, 1) >> 16;
        addr = sh7764_eth_desc_read(desc, 2);

        if (len > sizeof(buf)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "sh7764-eth: transmit length %u capped to %zu\n",
                          len, sizeof(buf));
            len = sizeof(buf);
        }
        if (len) {
            address_space_read(&address_space_memory, sh7764_eth_addr(addr),
                               MEMTXATTRS_UNSPECIFIED, buf, len);
            trace_sh7764_eth_tx((uint32_t)desc, len);
            qemu_send_packet(qemu_get_queue(s->nic), buf, len);
        }

        /* Hand the descriptor back and step on, wrapping at the DLE entry. */
        sh7764_eth_desc_write(desc, 0, status & ~SH7764_DESC_ACT);
        if (status & SH7764_DESC_DLE) {
            s->tx_cursor = 0;
        } else {
            s->tx_cursor += SH7764_ETH_DESC_SIZE;
        }

        s->eesr |= SH7764_EESR_TC;
    }

    s->edtrr &= ~SH7764_EDTRR_TR;
    sh7764_eth_update_irq(s);
}

static bool sh7764_eth_can_receive(NetClientState *nc)
{
    SH7764EthState *s = qemu_get_nic_opaque(nc);

    return (s->edrrr & SH7764_EDRRR_RR) && s->rdlar;
}

static ssize_t sh7764_eth_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    SH7764EthState *s = qemu_get_nic_opaque(nc);
    hwaddr desc;
    uint32_t status, capacity, addr;

    if (!sh7764_eth_can_receive(nc)) {
        return -1;
    }
    if (size > SH7764_ETH_MAX_FRAME) {
        return size;    /* drop, but do not stall the queue */
    }

    desc = sh7764_eth_addr(s->rdlar) + s->rx_cursor;
    status = sh7764_eth_desc_read(desc, 0);
    if (!(status & SH7764_DESC_ACT)) {
        /* No buffer free. Tell the net layer to hold off. */
        return 0;
    }

    capacity = sh7764_eth_desc_read(desc, 1) >> 16;
    addr = sh7764_eth_desc_read(desc, 2);
    if (size > capacity) {
        return size;
    }

    address_space_write(&address_space_memory, sh7764_eth_addr(addr),
                        MEMTXATTRS_UNSPECIFIED, buf, size);

    /* Frame length goes in the low half; the buffer length stays in the high. */
    sh7764_eth_desc_write(desc, 1, (capacity << 16) | (uint32_t)size);
    sh7764_eth_desc_write(desc, 0,
                          (status & ~SH7764_DESC_ACT) | SH7764_DESC_FRAME);

    if (status & SH7764_DESC_DLE) {
        s->rx_cursor = 0;
    } else {
        s->rx_cursor += SH7764_ETH_DESC_SIZE;
    }

    trace_sh7764_eth_rx((uint32_t)desc, (uint32_t)size);
    s->eesr |= SH7764_EESR_FR;
    sh7764_eth_update_irq(s);
    return size;
}

/*
 * MII management, driven a bit at a time through PIR: bit 0 is the clock,
 * bit 1 turns the data line around, bit 2 is the bit the guest drives and
 * bit 3 the bit it reads back.
 *
 * A driver that finds no PHY here concludes the cable is unplugged and never
 * brings the interface up, so enough of one is modelled to answer the
 * identity and status reads: link up, autonegotiation complete.
 */
#define PIR_MDC     (1u << 0)
#define PIR_MMD     (1u << 1)
#define PIR_MDO     (1u << 2)
#define PIR_MDI     (1u << 3)

/*
 * The board's PHY is a Realtek RTL8201FL-VB-CG (IC704 in the service manual,
 * fed by a 25 MHz clock). Its registers are paged: register 31 selects the
 * page, registers 0 to 15 are the IEEE set and always visible, and 16 upward
 * belong to whichever page is selected. The driver here selects page 7 and
 * programs RMSR, the LED registers and the link-change interrupt, so those
 * have to read back what was written or the read-modify-writes corrupt them.
 */
#define PHY_PAGE_SEL    31

static uint16_t sh7764_eth_phy_read(SH7764EthState *s, unsigned reg)
{
    if (reg == PHY_PAGE_SEL) {
        return s->phy_page;
    }

    /* Registers 0 to 15 are the IEEE set and do not page. */
    if (reg < 16) {
        switch (reg) {
        case 0:     /* BMCR */
            return s->phy_bmcr;
        case 1:
            /*
             * BMSR. Link up (bit 2), autonegotiation able (3) and complete
             * (5), plus the 10/100 capability bits. The link bit latches low
             * on the real part, but there is nothing here that can drop the
             * link, so reporting it up on every read is accurate.
             */
            return 0x782d | (1 << 2) | (1 << 5);
        case 2:     /* PHY identifier, RTL8201F */
            return 0x001c;
        case 3:
            return 0xc816;
        case 4:     /* our advertisement: 100/10, full and half duplex   */
            return 0x01e1;
        case 5:     /* link partner ability, same plus the ack bit       */
            return 0x45e1;
        case 6:     /* ANER: link partner is autonegotiation able        */
            return 0x0001;
        default:
            return s->phy_page0[reg];
        }
    }

    /* Paged registers. */
    switch (s->phy_page) {
    case 7:
        return s->phy_page7[reg - 16];
    default:
        return s->phy_page0[reg];
    }
}

static void sh7764_eth_phy_write(SH7764EthState *s, unsigned reg, uint16_t val)
{
    if (reg == PHY_PAGE_SEL) {
        s->phy_page = val;
        return;
    }
    if (reg == 0) {
        /* A reset request completes immediately; the bit reads back clear. */
        s->phy_bmcr = val & ~0x8000;
        return;
    }
    if (reg < 16) {
        s->phy_page0[reg] = val;
        return;
    }
    switch (s->phy_page) {
    case 7:
        s->phy_page7[reg - 16] = val;
        break;
    default:
        s->phy_page0[reg] = val;
        break;
    }
}

/*
 * One MDC cycle. Frame layout is Table 40 of the PHY datasheet:
 *
 *   read   preamble ST=01 OP=10 PHYAD(5) REGAD(5) TA=Z0 DATA(16)
 *   write  preamble ST=01 OP=01 PHYAD(5) REGAD(5) TA=10 DATA(16)
 *
 * so REGAD is the low five bits of the thirteen that follow the leading zero
 * of ST, not shifted by a turnaround that has not happened yet. Getting that
 * wrong makes the guest appear to read a scatter of nonsense registers, which
 * is exactly how this was found.
 *
 * PHYAD is ignored: this board has one PHY and the driver is written for it.
 */
enum {
    MDIO_IDLE = 0,      /* preamble; waiting for the leading zero of ST */
    MDIO_CMD,           /* 13 bits: ST0, OP, PHYAD, REGAD              */
    MDIO_READ_OUT,      /* turnaround then 16 bits back to the guest   */
    MDIO_WRITE_IN,      /* turnaround then 16 bits from the guest      */
};

static void sh7764_eth_mdio_clock(SH7764EthState *s, uint32_t val)
{
    bool bit = val & PIR_MDO;

    switch (s->mdio_state) {
    case MDIO_IDLE:
        if (!bit) {         /* ST[1] == 0 ends the preamble */
            s->mdio_state = MDIO_CMD;
            s->mdio_shift = 0;
            s->mdio_count = 0;
        }
        break;

    case MDIO_CMD:
        s->mdio_shift = (s->mdio_shift << 1) | (bit ? 1 : 0);
        if (++s->mdio_count < 13) {
            break;
        }
        {
            unsigned op = (s->mdio_shift >> 10) & 0x3;
            unsigned reg = s->mdio_shift & 0x1f;

            s->mdio_reg = reg;
            s->mdio_count = 0;
            if (op == 0x2) {
                s->mdio_data = sh7764_eth_phy_read(s, reg);
                trace_sh7764_eth_mdio(reg, s->mdio_data);
                s->mdio_state = MDIO_READ_OUT;
            } else if (op == 0x1) {
                s->mdio_shift = 0;
                s->mdio_state = MDIO_WRITE_IN;
            } else {
                s->mdio_state = MDIO_IDLE;
            }
        }
        break;

    case MDIO_READ_OUT:
        /* One turnaround bit, then the data most significant bit first. */
        if (s->mdio_count == 0) {
            s->pir &= ~PIR_MDI;
        } else {
            s->pir = (s->pir & ~PIR_MDI) |
                     ((s->mdio_data & 0x8000) ? PIR_MDI : 0);
            s->mdio_data <<= 1;
        }
        if (++s->mdio_count > 16) {
            s->mdio_state = MDIO_IDLE;
        }
        break;

    case MDIO_WRITE_IN:
        /* Two turnaround bits then sixteen of data. */
        s->mdio_shift = (s->mdio_shift << 1) | (bit ? 1 : 0);
        if (++s->mdio_count >= 18) {
            sh7764_eth_phy_write(s, s->mdio_reg,
                                 (uint16_t)(s->mdio_shift & 0xffff));
            s->mdio_state = MDIO_IDLE;
        }
        break;
    }
}

static uint64_t sh7764_eth_read_reg(SH7764EthState *s, hwaddr offset)
{
    switch (offset) {
    case SH7764_ETH_EDMR:       return s->edmr;
    case SH7764_ETH_EDTRR:      return s->edtrr;
    case SH7764_ETH_EDRRR:      return s->edrrr;
    case SH7764_ETH_TDLAR:      return s->tdlar;
    case SH7764_ETH_RDLAR:      return s->rdlar;
    case SH7764_ETH_EESR:       return s->eesr;
    case SH7764_ETH_EESIPR:     return s->eesipr;
    case SH7764_ETH_TRSCER:     return s->trscer;
    case SH7764_ETH_ECMR:       return s->ecmr;
    case SH7764_ETH_RFLR:       return s->rflr;
    case SH7764_ETH_ECSR:       return s->ecsr;
    case SH7764_ETH_ECSIPR:     return s->ecsipr;
    case SH7764_ETH_PIR:        return s->pir;
    case SH7764_ETH_MAHR:       return s->mahr;
    case SH7764_ETH_MALR:       return s->malr;
    case SH7764_ETH_PSR:
        /* LMON: the link is always up, there is nothing to unplug. */
        return 1;
    default:
        /*
         * The statistics counters and anything else unmodelled read as zero,
         * which for a counter is the honest answer.
         */
        return 0;
    }
}

static uint64_t sh7764_eth_read(void *opaque, hwaddr offset, unsigned size)
{
    SH7764EthState *s = opaque;
    uint64_t v = sh7764_eth_read_reg(s, offset);

    trace_sh7764_eth_read((uint32_t)offset, (uint32_t)v);
    return v;
}

static void sh7764_eth_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    SH7764EthState *s = opaque;

    trace_sh7764_eth_write((uint32_t)offset, (uint32_t)val);
    switch (offset) {
    case SH7764_ETH_EDMR:
        if (val & SH7764_EDMR_SWR) {
            /* Software reset clears the engine but leaves the MAC address. */
            s->edtrr = s->edrrr = 0;
            s->tdlar = s->rdlar = 0;
            s->eesr = s->eesipr = 0;
            s->tx_cursor = s->rx_cursor = 0;
            val &= ~SH7764_EDMR_SWR;
        }
        s->edmr = val;
        break;

    case SH7764_ETH_EDTRR:
        s->edtrr = val;
        sh7764_eth_transmit(s);
        break;

    case SH7764_ETH_EDRRR:
        s->edrrr = val;
        if (val & SH7764_EDRRR_RR) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;

    case SH7764_ETH_TDLAR:
        s->tdlar = val;
        s->tx_cursor = 0;
        break;

    case SH7764_ETH_RDLAR:
        s->rdlar = val;
        s->rx_cursor = 0;
        break;

    case SH7764_ETH_EESR:
        /* Write one to clear. */
        s->eesr &= ~(uint32_t)val;
        sh7764_eth_update_irq(s);
        break;

    case SH7764_ETH_EESIPR:
        s->eesipr = val;
        sh7764_eth_update_irq(s);
        break;

    case SH7764_ETH_TRSCER:     s->trscer = val; break;
    case SH7764_ETH_ECMR:       s->ecmr = val;   break;
    case SH7764_ETH_RFLR:       s->rflr = val;   break;
    case SH7764_ETH_ECSR:       s->ecsr &= ~(uint32_t)val; break;
    case SH7764_ETH_ECSIPR:     s->ecsipr = val; break;
    case SH7764_ETH_MAHR:       s->mahr = val;   break;
    case SH7764_ETH_MALR:       s->malr = val;   break;

    case SH7764_ETH_PIR:
        /* Everything happens on the rising edge of MDC. */
        if ((val & PIR_MDC) && !(s->pir & PIR_MDC)) {
            sh7764_eth_mdio_clock(s, val);
        }
        s->pir = (s->pir & PIR_MDI) | (val & ~PIR_MDI);
        break;

    default:
        break;
    }
}

static const MemoryRegionOps sh7764_eth_ops = {
    .read = sh7764_eth_read,
    .write = sh7764_eth_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static NetClientInfo net_sh7764_eth_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = sh7764_eth_can_receive,
    .receive = sh7764_eth_receive,
};

static void sh7764_eth_reset(DeviceState *dev)
{
    SH7764EthState *s = SH7764_ETH(dev);

    s->edmr = s->edtrr = s->edrrr = 0;
    s->tdlar = s->rdlar = 0;
    s->eesr = s->eesipr = s->trscer = 0;
    s->ecmr = s->rflr = s->ecsr = s->ecsipr = 0;
    s->pir = 0;
    s->tx_cursor = s->rx_cursor = 0;
    s->mdio_shift = s->mdio_count = s->mdio_data = 0;
    s->mdio_state = 0;
}

static void sh7764_eth_realize(DeviceState *dev, Error **errp)
{
    SH7764EthState *s = SH7764_ETH(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &sh7764_eth_ops, s,
                          "sh7764.eth", SH7764_ETH_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_sh7764_eth_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static const VMStateDescription vmstate_sh7764_eth = {
    .name = "sh7764-eth",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(edmr, SH7764EthState),
        VMSTATE_UINT32(edtrr, SH7764EthState),
        VMSTATE_UINT32(edrrr, SH7764EthState),
        VMSTATE_UINT32(tdlar, SH7764EthState),
        VMSTATE_UINT32(rdlar, SH7764EthState),
        VMSTATE_UINT32(eesr, SH7764EthState),
        VMSTATE_UINT32(eesipr, SH7764EthState),
        VMSTATE_UINT32(trscer, SH7764EthState),
        VMSTATE_UINT32(ecmr, SH7764EthState),
        VMSTATE_UINT32(rflr, SH7764EthState),
        VMSTATE_UINT32(ecsr, SH7764EthState),
        VMSTATE_UINT32(ecsipr, SH7764EthState),
        VMSTATE_UINT32(mahr, SH7764EthState),
        VMSTATE_UINT32(malr, SH7764EthState),
        VMSTATE_UINT32(tx_cursor, SH7764EthState),
        VMSTATE_UINT32(rx_cursor, SH7764EthState),
        VMSTATE_UINT32(pir, SH7764EthState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property sh7764_eth_properties[] = {
    DEFINE_NIC_PROPERTIES(SH7764EthState, conf),
};

static void sh7764_eth_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sh7764_eth_realize;
    device_class_set_legacy_reset(dc, sh7764_eth_reset);
    dc->vmsd = &vmstate_sh7764_eth;
    device_class_set_props(dc, sh7764_eth_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo sh7764_eth_info = {
    .name = TYPE_SH7764_ETH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SH7764EthState),
    .class_init = sh7764_eth_class_init,
};

static void sh7764_eth_register_types(void)
{
    type_register_static(&sh7764_eth_info);
}

type_init(sh7764_eth_register_types)
