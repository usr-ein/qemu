/*
 * Renesas SH7764 Ethernet controller (EtherC) and its DMA engine (E-DMAC)
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_SH7764_ETH_H
#define HW_NET_SH7764_ETH_H

#include "hw/core/sysbus.h"
#include "net/net.h"
#include "system/memory.h"
#include "qom/object.h"

#define TYPE_SH7764_ETH "sh7764-eth"
OBJECT_DECLARE_SIMPLE_TYPE(SH7764EthState, SH7764_ETH)

/* Register block base and size. Section 19 and 20 of the hardware manual. */
#define SH7764_ETH_BASE         0xfef00000
#define SH7764_ETH_SIZE         0x1000

/* E-DMAC, section 20.2 */
#define SH7764_ETH_EDMR         0x000   /* mode, bit 0 = SWR software reset  */
#define SH7764_ETH_EDTRR        0x008   /* transmit request, bit 0 = TR      */
#define SH7764_ETH_EDRRR        0x010   /* receive request, bit 0 = RR       */
#define SH7764_ETH_TDLAR        0x018   /* transmit descriptor list base     */
#define SH7764_ETH_RDLAR        0x020   /* receive descriptor list base      */
#define SH7764_ETH_EESR         0x028   /* status                            */
#define SH7764_ETH_EESIPR       0x030   /* status interrupt permission       */
#define SH7764_ETH_TRSCER       0x038   /* status copy enable                */

/* EtherC, section 19.3 */
#define SH7764_ETH_ECMR         0x100   /* mode                              */
#define SH7764_ETH_RFLR         0x108   /* receive frame length limit        */
#define SH7764_ETH_ECSR         0x110   /* status                            */
#define SH7764_ETH_ECSIPR       0x118   /* status interrupt permission       */
#define SH7764_ETH_PIR          0x120   /* PHY (MII management) interface    */
#define SH7764_ETH_PSR          0x128   /* PHY status, bit 0 = LMON          */
#define SH7764_ETH_MAHR         0x1c0   /* MAC address, high four bytes      */
#define SH7764_ETH_MALR         0x1c8   /* MAC address, low two bytes        */

#define SH7764_EDMR_SWR         (1u << 0)
#define SH7764_EDTRR_TR         (1u << 0)
#define SH7764_EDRRR_RR         (1u << 0)

/*
 * Status bits. Only the two the driver actually waits on are modelled: a
 * frame was received, and a frame finished transmitting.
 */
#define SH7764_EESR_FR          (1u << 18)
#define SH7764_EESR_RDE         (1u << 17)   /* receive descriptor empty */
/* ECI reflects EtherC's own status register, section 20.2.6 bit 22. */
#define SH7764_EESR_ECI         (1u << 22)

/* EtherC status: bit 2 is LCHNG, the link signal change, write 1 to clear. */
#define SH7764_ECSR_LCHNG       (1u << 2)
#define SH7764_ECSIPR_LCHNGIP   (1u << 2)

/* ECMR, section 19.3.1. */
#define SH7764_ECMR_DM          (1u << 1)
#define SH7764_ECMR_TE          (1u << 5)
#define SH7764_ECMR_RE          (1u << 6)
#define SH7764_EESR_TC          (1u << 21)

/*
 * Descriptor layout, 16 bytes, confirmed against the firmware's own ring
 * setup in ether_open:
 *
 *   +0x00  status   bit 31 ACT (owned by the engine), bit 30 DLE (last
 *                   entry, wrap), bits 29-28 FP (frame position)
 *   +0x04  length   buffer length in the upper 16 bits; on receive the
 *                   engine writes the frame length into the lower 16
 *   +0x08  buffer   physical address of the data
 *   +0x0c  unused
 */
#define SH7764_ETH_DESC_SIZE    0x10
#define SH7764_DESC_ACT         (1u << 31)
#define SH7764_DESC_DLE         (1u << 30)
#define SH7764_DESC_FP1         (1u << 29)
#define SH7764_DESC_FP0         (1u << 28)
#define SH7764_DESC_FRAME       (SH7764_DESC_FP1 | SH7764_DESC_FP0)

/* The firmware programs 1600-byte receive buffers; guard against silly ones. */
#define SH7764_ETH_MAX_FRAME    2048

struct SH7764EthState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;

    uint32_t edmr;
    uint32_t edtrr;
    uint32_t edrrr;
    uint32_t tdlar;
    uint32_t rdlar;
    uint32_t eesr;
    uint32_t eesipr;
    uint32_t trscer;

    uint32_t ecmr;
    uint32_t rflr;
    uint32_t ecsr;
    uint32_t ecsipr;
    uint32_t psr;
    uint32_t mahr;
    uint32_t malr;

    /* Where the engine will look next, as an offset from TDLAR / RDLAR. */
    uint32_t tx_cursor;
    uint32_t rx_cursor;

    /* MII management (PIR) bit-bang state; see sh7764_eth_mdio_clock(). */
    uint32_t pir;
    uint32_t mdio_shift;        /* bits shifted in from the guest          */
    uint32_t mdio_count;        /* how many have arrived                   */
    uint32_t mdio_data;         /* reply being shifted back out            */
    int mdio_state;
    unsigned mdio_reg;

    /* RTL8201FL PHY: paged register file, page selected by register 31. */
    uint16_t phy_page;
    uint16_t phy_bmcr;
    uint16_t phy_page0[32];
    uint16_t phy_page7[16];
};

#endif /* HW_NET_SH7764_ETH_H */
