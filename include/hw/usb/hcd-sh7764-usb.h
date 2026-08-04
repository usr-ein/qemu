/*
 * Renesas SH7764 USB 2.0 host/function module
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_USB_HCD_SH7764_USB_H
#define HW_USB_HCD_SH7764_USB_H

#include "hw/core/sysbus.h"
#include "hw/usb/usb.h"
#include "system/memory.h"
#include "qom/object.h"

#define TYPE_SH7764_USB "sh7764-usb"
OBJECT_DECLARE_SIMPLE_TYPE(SH7764USBState, SH7764_USB)

/*
 * Register offsets from table 21.2. The window has to reach 0x1c0, where the
 * second D1FIFO alias lives.
 */
#define SH7764_USB_SIZE       0x200

#define SH7764_USB_SYSCFG     0x000
#define SH7764_USB_BUSWAIT    0x002
#define SH7764_USB_SYSSTS     0x004
#define SH7764_USB_DVSTCTR    0x008
#define SH7764_USB_TESTMODE   0x00c
#define SH7764_USB_D0FBCFG    0x010
#define SH7764_USB_D1FBCFG    0x012
#define SH7764_USB_CFIFO      0x014
#define SH7764_USB_D0FIFO     0x018
#define SH7764_USB_D1FIFO     0x01c
#define SH7764_USB_CFIFOSEL   0x020
#define SH7764_USB_CFIFOCTR   0x022
#define SH7764_USB_D0FIFOSEL  0x028
#define SH7764_USB_D0FIFOCTR  0x02a
#define SH7764_USB_D1FIFOSEL  0x02c
#define SH7764_USB_D1FIFOCTR  0x02e
#define SH7764_USB_INTENB0    0x030
#define SH7764_USB_INTENB1    0x032
#define SH7764_USB_BRDYENB    0x036
#define SH7764_USB_NRDYENB    0x038
#define SH7764_USB_BEMPENB    0x03a
#define SH7764_USB_SOFCFG     0x03c
#define SH7764_USB_INTSTS0    0x040
#define SH7764_USB_INTSTS1    0x042
#define SH7764_USB_BRDYSTS    0x046
#define SH7764_USB_NRDYSTS    0x048
#define SH7764_USB_BEMPSTS    0x04a
#define SH7764_USB_FRMNUM     0x04c
#define SH7764_USB_UFRMNUM    0x04e
#define SH7764_USB_USBADDR    0x050
#define SH7764_USB_USBREQ     0x054
#define SH7764_USB_USBVAL     0x056
#define SH7764_USB_USBINDX    0x058
#define SH7764_USB_USBLENG    0x05a
#define SH7764_USB_DCPCFG     0x05c
#define SH7764_USB_DCPMAXP    0x05e
#define SH7764_USB_DCPCTR     0x060
#define SH7764_USB_PIPESEL    0x064
#define SH7764_USB_PIPECFG    0x068
#define SH7764_USB_PIPEBUF    0x06a
#define SH7764_USB_PIPEMAXP   0x06c
#define SH7764_USB_PIPEPERI   0x06e
#define SH7764_USB_PIPE1CTR   0x070
#define SH7764_USB_PIPE9CTR   0x080
#define SH7764_USB_PIPE1TRE   0x090
#define SH7764_USB_PIPE5TRN   0x0a2
#define SH7764_USB_DEVADD0    0x0d0
#define SH7764_USB_DEVADDA    0x0e4

/* Pipe 0 is the default control pipe, then the nine general pipes. */
#define SH7764_USB_NPIPES     10

/*
 * Per-pipe buffer. The chip carves a shared 8 KB RAM up with PIPEBUF; this
 * gives each pipe its own, which no register can tell apart and which is
 * comfortably larger than any packet the pipes are configured for.
 */
#define SH7764_USB_PIPE_BUF   2048

typedef struct SH7764USBPipe {
    uint16_t cfg;                       /* PIPECFG                          */
    uint16_t buf;                       /* PIPEBUF                          */
    uint16_t maxp;                      /* PIPEMAXP, or DCPMAXP for pipe 0  */
    uint16_t peri;                      /* PIPEPERI                         */
    uint16_t ctr;                       /* PIPEnCTR                         */
    uint16_t tre;
    uint16_t trn;
    bool in;                            /* last started direction           */
    bool ready;                         /* BVAL: buffer is worth sending    */
    uint32_t len;                       /* bytes held in the buffer         */
    uint32_t pos;                       /* how far the FIFO port has read   */
    uint8_t data[SH7764_USB_PIPE_BUF];
} SH7764USBPipe;

struct SH7764USBState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;

    USBBus bus;
    USBPort port;
    USBPacket packet;

    uint16_t syscfg;
    uint16_t buswait;
    uint16_t syssts;
    uint16_t dvstctr;
    uint16_t testmode;
    uint16_t d0fbcfg;
    uint16_t d1fbcfg;
    uint16_t fifosel[3];                /* CFIFO, D0FIFO, D1FIFO            */
    uint16_t fifoctr[3];
    uint16_t intenb0;
    uint16_t intenb1;
    uint16_t brdyenb;
    uint16_t nrdyenb;
    uint16_t bempenb;
    uint16_t sofcfg;
    uint16_t intsts0;
    uint16_t intsts1;
    uint16_t brdysts;
    uint16_t nrdysts;
    uint16_t bempsts;
    uint16_t frmnum;
    uint16_t ufrmnum;
    uint16_t usbaddr;
    uint16_t usbreq;
    uint16_t usbval;
    uint16_t usbindx;
    uint16_t usbleng;
    uint16_t dcpcfg;
    uint16_t dcpmaxp;
    uint16_t dcpctr;
    uint16_t pipesel;
    uint16_t devadd[11];

    uint8_t setup[8];
    int32_t async_pipe;                 /* pipe awaiting completion, or -1  */

    SH7764USBPipe pipe[SH7764_USB_NPIPES];
};

#endif /* HW_USB_HCD_SH7764_USB_H */
