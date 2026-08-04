/*
 * Renesas SH7764 USB 2.0 host/function module, host side
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Section 21 of the SH7764 hardware manual. The register set is the one
 * Renesas use across this family - the M66592 and R8A66597 have the same
 * SYSCFG/CFIFO/PIPESEL/PIPEnCTR layout - so a driver written for any of them
 * drives this too.
 *
 * Only the host controller half is modelled, which is what the CDJ-2000NXS
 * main processor selects: it sets DCFM and DRPD in SYSCFG and then waits for
 * something to be plugged into the Type A socket. The function half, which
 * would make the player look like a USB audio device, is a separate chip on
 * that board (an M66291 at IC701) and is not this.
 *
 * How much of the hardware is here, and what is left out:
 *
 *   - Transfers run to completion when the pipe is started rather than being
 *     paced by the bus. A device that answers asynchronously is handled
 *     properly through the port's complete callback, so a SCSI-backed disk
 *     behaves; what is missing is the timing, not the sequencing.
 *   - Each pipe gets its own buffer instead of carving a shared 8 KB RAM up
 *     the way PIPEBUF describes. PIPEBUF is stored and read back, but the
 *     partitioning it expresses cannot be observed through the FIFO ports.
 *   - Double buffering (DBLB) is accepted and ignored; one buffer is always
 *     ready, which is what BSTS and BRDY end up reporting either way.
 *   - There is one root port, as on the chip: DP and DM are pins of the SoC.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/usb/hcd-sh7764-usb.h"
#include "migration/vmstate.h"
#include "trace.h"

/* SYSCFG */
#define SYSCFG_USBE     (1u << 0)
#define SYSCFG_DPRPU    (1u << 4)
#define SYSCFG_DRPD     (1u << 5)
#define SYSCFG_DCFM     (1u << 6)
#define SYSCFG_HSE      (1u << 7)
#define SYSCFG_SCKE     (1u << 10)

/* SYSSTS: table 21.6. */
#define SYSSTS_LNST_SE0 0
#define SYSSTS_LNST_J   1
#define SYSSTS_LNST_K   2

/* DVSTCTR */
#define DVSTCTR_RHST    0x0007
#define DVSTCTR_UACT    (1u << 4)
#define DVSTCTR_RESUME  (1u << 5)
#define DVSTCTR_USBRST  (1u << 6)
#define DVSTCTR_RWUPE   (1u << 7)
#define DVSTCTR_WKUP    (1u << 8)

#define RHST_NONE       0
#define RHST_FULL       2
#define RHST_HIGH       3
#define RHST_RESETTING  4

/* xFIFOSEL */
#define FIFOSEL_CURPIPE 0x000f
#define FIFOSEL_ISEL    (1u << 5)
#define FIFOSEL_BIGEND  (1u << 8)
#define FIFOSEL_MBW     0x0c00
#define FIFOSEL_MBW_SH  10
#define FIFOSEL_REW     (1u << 14)
#define FIFOSEL_RCNT    (1u << 15)

/* xFIFOCTR */
#define FIFOCTR_DTLN    0x0fff
#define FIFOCTR_FRDY    (1u << 13)
#define FIFOCTR_BCLR    (1u << 14)
#define FIFOCTR_BVAL    (1u << 15)

/* INTENB0 and INTSTS0 share their upper bits. */
#define INTSTS0_BRDY    (1u << 8)
#define INTSTS0_NRDY    (1u << 9)
#define INTSTS0_BEMP    (1u << 10)
#define INTSTS0_CTRT    (1u << 11)
#define INTSTS0_DVST    (1u << 12)
#define INTSTS0_SOFR    (1u << 13)
#define INTSTS0_RESM    (1u << 14)
#define INTSTS0_VBINT   (1u << 15)
#define INTSTS0_VBSTS   (1u << 7)

/* INTENB1 and INTSTS1 */
#define INTSTS1_SACK    (1u << 4)
#define INTSTS1_SIGN    (1u << 5)
#define INTSTS1_EOFERR  (1u << 6)
#define INTSTS1_ATTCH   (1u << 11)
#define INTSTS1_DTCH    (1u << 12)
#define INTSTS1_BCHG    (1u << 14)
#define INTSTS1_OVRCR   (1u << 15)

/* DCPCTR and PIPEnCTR */
#define PIPECTR_PID     0x0003
#define PIPECTR_CCPL    (1u << 2)       /* DCP only                         */
#define PIPECTR_PBUSY   (1u << 5)
#define PIPECTR_SQMON   (1u << 6)
#define PIPECTR_SQSET   (1u << 7)
#define PIPECTR_SQCLR   (1u << 8)
#define PIPECTR_ACLRM   (1u << 9)
#define PIPECTR_SUREQCLR (1u << 11)     /* DCP only                         */
#define PIPECTR_CSSTS   (1u << 12)
#define PIPECTR_CSCLR   (1u << 13)
#define PIPECTR_SUREQ   (1u << 14)      /* DCP only                         */
#define PIPECTR_INBUFM  (1u << 14)      /* PIPEn only                       */
#define PIPECTR_BSTS    (1u << 15)

#define PID_NAK         0
#define PID_BUF         1
#define PID_STALL       2

/* PIPECFG */
#define PIPECFG_EPNUM   0x000f
#define PIPECFG_DIR     (1u << 4)       /* 1 = out (host to device)         */
#define PIPECFG_TYPE    0xc000
#define PIPECFG_TYPE_SH 14

/* PIPEMAXP and DCPMAXP */
#define MAXP_MXPS       0x07ff
#define MAXP_DEVSEL     0xf000
#define MAXP_DEVSEL_SH  12

/* DEVADDn */
#define DEVADD_USBSPD   0x00c0
#define DEVADD_USBSPD_SH 6
#define USBSPD_LOW      1
#define USBSPD_FULL     2
#define USBSPD_HIGH     3

static void sh7764_usb_update_irq(SH7764USBState *s)
{
    bool level;

    if (s->intsts0 & (s->intenb0 & 0xff00)) {
        level = true;
    } else if (s->intsts1 & s->intenb1) {
        level = true;
    } else {
        level = false;
    }

    qemu_set_irq(s->irq, level);
}

static void sh7764_usb_raise0(SH7764USBState *s, uint16_t bits)
{
    s->intsts0 |= bits;
    sh7764_usb_update_irq(s);
}

static void sh7764_usb_raise1(SH7764USBState *s, uint16_t bits)
{
    s->intsts1 |= bits;
    sh7764_usb_update_irq(s);
}

/*
 * The device a pipe talks to.
 *
 * DEVSEL in the pipe's maximum-packet-size register names one of the eleven
 * DEVADD registers, and that register carries the speed. The address itself
 * is the DEVSEL index: the module uses register n for the device it has given
 * address n, which is the convention the manual's transfer sequences assume.
 */
static USBDevice *sh7764_usb_pipe_dev(SH7764USBState *s, unsigned n)
{
    uint16_t maxp = n == 0 ? s->dcpmaxp : s->pipe[n].maxp;
    unsigned addr = (maxp & MAXP_DEVSEL) >> MAXP_DEVSEL_SH;

    if (!s->port.dev || !s->port.dev->attached) {
        return NULL;
    }
    return usb_find_device(&s->port, addr);
}

static unsigned sh7764_usb_pipe_epnum(SH7764USBState *s, unsigned n)
{
    return n == 0 ? 0 : (s->pipe[n].cfg & PIPECFG_EPNUM);
}

/*
 * True when the pipe carries data from the device to the host.
 *
 * The control pipe's direction is DCPCFG's DIR bit and nothing else. It is not
 * the direction implied by the request: a control transfer turns round for its
 * status stage, and software says so by flipping DIR, which is exactly what
 * the firmware does between reading a descriptor and acknowledging it.
 */
static bool sh7764_usb_pipe_in(SH7764USBState *s, unsigned n)
{
    if (n == 0) {
        return !(s->dcpcfg & PIPECFG_DIR);
    }
    return !(s->pipe[n].cfg & PIPECFG_DIR);
}

static unsigned sh7764_usb_pipe_maxp(SH7764USBState *s, unsigned n)
{
    uint16_t maxp = n == 0 ? s->dcpmaxp : s->pipe[n].maxp;
    unsigned mxps = maxp & MAXP_MXPS;

    return mxps ? mxps : 64;
}

static void sh7764_usb_pipe_done(SH7764USBState *s, unsigned n, int status,
                                 unsigned actual);

static void sh7764_usb_transfer(SH7764USBState *s, unsigned n, int pid,
                                uint8_t *buf, unsigned len)
{
    USBDevice *dev = sh7764_usb_pipe_dev(s, n);
    USBEndpoint *ep;

    if (!dev) {
        sh7764_usb_pipe_done(s, n, USB_RET_NODEV, 0);
        return;
    }

    ep = usb_ep_get(dev, pid, sh7764_usb_pipe_epnum(s, n));
    trace_usb_sh7764_pipe_start(n, pid == USB_TOKEN_IN, dev->addr,
                                sh7764_usb_pipe_epnum(s, n), len);
    usb_packet_setup(&s->packet, pid, ep, 0, 0, pid != USB_TOKEN_IN, true);
    usb_packet_addbuf(&s->packet, buf, len);
    s->async_pipe = n;
    usb_handle_packet(dev, &s->packet);

    if (s->packet.status == USB_RET_ASYNC) {
        /* The port's complete callback finishes it. */
        return;
    }
    sh7764_usb_pipe_done(s, n, s->packet.status, s->packet.actual_length);
}

/*
 * Finish whatever the pipe was doing and tell software about it the way the
 * hardware does: a buffer that filled or emptied raises BRDY, a device that
 * had nothing to say raises NRDY, and a stall latches PID to STALL.
 */
static void sh7764_usb_pipe_done(SH7764USBState *s, unsigned n, int status,
                                 unsigned actual)
{
    SH7764USBPipe *p = &s->pipe[n];
    uint16_t *ctr = n == 0 ? &s->dcpctr : &p->ctr;

    trace_usb_sh7764_pipe_done(n, status, actual);
    s->async_pipe = -1;
    *ctr &= ~PIPECTR_PBUSY;

    switch (status) {
    case USB_RET_SUCCESS:
        if (p->in) {
            p->len = actual;
            p->pos = 0;
            /*
             * A short packet ends the transfer; the driver sees that from
             * DTLN being less than the maximum packet size.
             */
        } else {
            p->len = 0;
            p->pos = 0;
            s->bempsts |= 1u << n;
            if (s->bempenb & (1u << n)) {
                sh7764_usb_raise0(s, INTSTS0_BEMP);
            }
        }
        s->brdysts |= 1u << n;
        if (s->brdyenb & (1u << n)) {
            sh7764_usb_raise0(s, INTSTS0_BRDY);
        }
        break;

    case USB_RET_NAK:
        s->nrdysts |= 1u << n;
        if (s->nrdyenb & (1u << n)) {
            sh7764_usb_raise0(s, INTSTS0_NRDY);
        }
        break;

    case USB_RET_STALL:
        *ctr = (*ctr & ~PIPECTR_PID) | PID_STALL;
        s->nrdysts |= 1u << n;
        if (s->nrdyenb & (1u << n)) {
            sh7764_usb_raise0(s, INTSTS0_NRDY);
        }
        break;

    default:
        /* No device, babble, IO error: report it as a failed handshake. */
        s->nrdysts |= 1u << n;
        if (s->nrdyenb & (1u << n)) {
            sh7764_usb_raise0(s, INTSTS0_NRDY);
        }
        break;
    }
}

static void sh7764_usb_async_complete(USBPort *port, USBPacket *packet)
{
    SH7764USBState *s = port->opaque;
    int n = s->async_pipe;

    if (n < 0) {
        return;
    }
    sh7764_usb_pipe_done(s, n, packet->status, packet->actual_length);
}

/*
 * Start whatever the pipe is set up to do.
 *
 * An IN pipe fetches a packet into the pipe buffer so the FIFO port has
 * something to hand out; an OUT pipe sends what the FIFO port was given.
 */
static void sh7764_usb_pipe_start(SH7764USBState *s, unsigned n)
{
    SH7764USBPipe *p = &s->pipe[n];
    uint16_t ctr = n == 0 ? s->dcpctr : p->ctr;

    if ((ctr & PIPECTR_PID) != PID_BUF || s->async_pipe >= 0) {
        return;
    }

    p->in = sh7764_usb_pipe_in(s, n);
    if (p->in) {
        unsigned maxp = MIN(sh7764_usb_pipe_maxp(s, n), sizeof(p->data));

        sh7764_usb_transfer(s, n, USB_TOKEN_IN, p->data, maxp);
    } else if (p->ready) {
        /*
         * BVAL is what marks an outgoing buffer as worth sending, and it says
         * nothing about how full it is: the zero-length packet that closes a
         * control transfer arrives here with nothing in the buffer at all.
         */
        p->ready = false;
        sh7764_usb_transfer(s, n, USB_TOKEN_OUT, p->data, p->len);
    }
}

/* The setup stage, driven by SUREQ in DCPCTR. */
static void sh7764_usb_send_setup(SH7764USBState *s)
{
    USBDevice *dev = sh7764_usb_pipe_dev(s, 0);
    USBEndpoint *ep;

    s->setup[0] = s->usbreq & 0xff;
    s->setup[1] = s->usbreq >> 8;
    s->setup[2] = s->usbval & 0xff;
    s->setup[3] = s->usbval >> 8;
    s->setup[4] = s->usbindx & 0xff;
    s->setup[5] = s->usbindx >> 8;
    s->setup[6] = s->usbleng & 0xff;
    s->setup[7] = s->usbleng >> 8;

    trace_usb_sh7764_setup(s->setup[0], s->setup[1], s->usbval, s->usbindx,
                           s->usbleng);
    s->dcpctr &= ~PIPECTR_SUREQ;

    if (!dev) {
        sh7764_usb_raise1(s, INTSTS1_SIGN);
        return;
    }

    ep = usb_ep_get(dev, USB_TOKEN_SETUP, 0);
    usb_packet_setup(&s->packet, USB_TOKEN_SETUP, ep, 0, 0, false, true);
    usb_packet_addbuf(&s->packet, s->setup, sizeof(s->setup));
    usb_handle_packet(dev, &s->packet);

    if (s->packet.status == USB_RET_SUCCESS) {
        s->pipe[0].len = 0;
        s->pipe[0].pos = 0;
        sh7764_usb_raise1(s, INTSTS1_SACK);
    } else {
        sh7764_usb_raise1(s, INTSTS1_SIGN);
    }
}

/*
 * The status stage. Writing CCPL with the pipe enabled sends the handshake
 * that closes the control transfer, in whichever direction is left over.
 */
static void sh7764_usb_control_complete(SH7764USBState *s)
{
    int pid = (s->setup[0] & USB_DIR_IN) ? USB_TOKEN_OUT : USB_TOKEN_IN;

    s->dcpctr &= ~PIPECTR_CCPL;
    s->pipe[0].in = pid == USB_TOKEN_IN;
    sh7764_usb_transfer(s, 0, pid, s->pipe[0].data, 0);
    sh7764_usb_raise0(s, INTSTS0_CTRT);
}

/* ---------------------------------------------------------------- FIFOs */

/*
 * The three FIFO ports are windows onto whichever pipe their select register
 * names. CFIFO is the one software uses by hand; D0FIFO and D1FIFO are meant
 * for the DMA controller but behave identically when read or written directly.
 */
static SH7764USBPipe *sh7764_usb_fifo_pipe(SH7764USBState *s, unsigned f)
{
    unsigned n = s->fifosel[f] & FIFOSEL_CURPIPE;

    return n < SH7764_USB_NPIPES ? &s->pipe[n] : NULL;
}

static uint64_t sh7764_usb_fifo_read(SH7764USBState *s, unsigned f,
                                     unsigned size)
{
    SH7764USBPipe *p = sh7764_usb_fifo_pipe(s, f);
    uint64_t val = 0;
    unsigned i;

    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        uint8_t byte = p->pos < p->len ? p->data[p->pos] : 0;

        if (p->pos < p->len) {
            p->pos++;
        }
        /*
         * MBW selects the access width, and the module packs the bytes in
         * little-endian order within it unless BIGEND says otherwise.
         */
        if (s->fifosel[f] & FIFOSEL_BIGEND) {
            val = (val << 8) | byte;
        } else {
            val |= (uint64_t)byte << (8 * i);
        }
    }
    return val;
}

static void sh7764_usb_fifo_write(SH7764USBState *s, unsigned f,
                                  uint64_t val, unsigned size)
{
    SH7764USBPipe *p = sh7764_usb_fifo_pipe(s, f);
    unsigned i;

    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        uint8_t byte;

        if (s->fifosel[f] & FIFOSEL_BIGEND) {
            byte = val >> (8 * (size - 1 - i));
        } else {
            byte = val >> (8 * i);
        }
        if (p->len < sizeof(p->data)) {
            p->data[p->len++] = byte;
        }
    }
}

static uint16_t sh7764_usb_fifoctr(SH7764USBState *s, unsigned f)
{
    SH7764USBPipe *p = sh7764_usb_fifo_pipe(s, f);
    uint16_t val = FIFOCTR_FRDY;

    if (p && p->in) {
        val |= (p->len - p->pos) & FIFOCTR_DTLN;
    }
    return val;
}

static void sh7764_usb_fifoctr_write(SH7764USBState *s, unsigned f,
                                     uint16_t val)
{
    SH7764USBPipe *p = sh7764_usb_fifo_pipe(s, f);
    unsigned n = s->fifosel[f] & FIFOSEL_CURPIPE;

    if (!p) {
        return;
    }
    if (val & FIFOCTR_BCLR) {
        p->len = 0;
        p->pos = 0;
    }
    if (val & FIFOCTR_BVAL) {
        /* Software has finished filling the buffer: send it. */
        p->in = false;
        p->ready = true;
        sh7764_usb_pipe_start(s, n);
    }
}

/* ------------------------------------------------------------ registers */

static uint16_t sh7764_usb_pipectr(SH7764USBState *s, unsigned n)
{
    SH7764USBPipe *p = &s->pipe[n];
    uint16_t val = n == 0 ? s->dcpctr : p->ctr;

    /*
     * BSTS says the buffer is accessible from the CPU side. For a pipe
     * bringing data in that means a packet has landed; for one sending data
     * out it means there is room, which here is always.
     */
    if (!p->in || p->pos < p->len) {
        val |= PIPECTR_BSTS;
    }
    return val;
}

static void sh7764_usb_pipectr_write(SH7764USBState *s, unsigned n,
                                     uint16_t val)
{
    SH7764USBPipe *p = &s->pipe[n];
    uint16_t *ctr = n == 0 ? &s->dcpctr : &p->ctr;
    uint16_t old = *ctr;

    if (val & PIPECTR_ACLRM) {
        p->len = 0;
        p->pos = 0;
    }
    if (val & PIPECTR_CSCLR) {
        val &= ~PIPECTR_CSSTS;
    }

    *ctr = val & ~(PIPECTR_ACLRM | PIPECTR_CSCLR | PIPECTR_SQCLR |
                   PIPECTR_SQSET | PIPECTR_BSTS);

    if (n == 0) {
        if (val & PIPECTR_SUREQ) {
            sh7764_usb_send_setup(s);
            return;
        }
        if (val & PIPECTR_SUREQCLR) {
            *ctr &= ~PIPECTR_SUREQ;
        }
        if (val & PIPECTR_CCPL && (val & PIPECTR_PID) == PID_BUF) {
            sh7764_usb_control_complete(s);
            return;
        }
    }

    /* Enabling the pipe is what starts a transfer. */
    if ((val & PIPECTR_PID) == PID_BUF && (old & PIPECTR_PID) != PID_BUF) {
        sh7764_usb_pipe_start(s, n);
    }
}

static uint64_t sh7764_usb_read_reg(SH7764USBState *s, hwaddr addr,
                                    unsigned size)
{
    switch (addr) {
    case SH7764_USB_SYSCFG:
        return s->syscfg;
    case SH7764_USB_BUSWAIT:
        return s->buswait;
    case SH7764_USB_SYSSTS:
        return s->syssts;
    case SH7764_USB_DVSTCTR:
        return s->dvstctr;
    case SH7764_USB_TESTMODE:
        return s->testmode;
    case SH7764_USB_D0FBCFG:
        return s->d0fbcfg;
    case SH7764_USB_D1FBCFG:
        return s->d1fbcfg;
    case SH7764_USB_CFIFO ... SH7764_USB_CFIFO + 3:
        return sh7764_usb_fifo_read(s, 0, size);
    case SH7764_USB_D0FIFO ... SH7764_USB_D0FIFO + 3:
        return sh7764_usb_fifo_read(s, 1, size);
    case SH7764_USB_D1FIFO ... SH7764_USB_D1FIFO + 3:
        return sh7764_usb_fifo_read(s, 2, size);
    case SH7764_USB_CFIFOSEL:
        return s->fifosel[0];
    case SH7764_USB_D0FIFOSEL:
        return s->fifosel[1];
    case SH7764_USB_D1FIFOSEL:
        return s->fifosel[2];
    case SH7764_USB_CFIFOCTR:
        return sh7764_usb_fifoctr(s, 0);
    case SH7764_USB_D0FIFOCTR:
        return sh7764_usb_fifoctr(s, 1);
    case SH7764_USB_D1FIFOCTR:
        return sh7764_usb_fifoctr(s, 2);
    case SH7764_USB_INTENB0:
        return s->intenb0;
    case SH7764_USB_INTENB1:
        return s->intenb1;
    case SH7764_USB_BRDYENB:
        return s->brdyenb;
    case SH7764_USB_NRDYENB:
        return s->nrdyenb;
    case SH7764_USB_BEMPENB:
        return s->bempenb;
    case SH7764_USB_SOFCFG:
        return s->sofcfg;
    case SH7764_USB_INTSTS0:
        return s->intsts0 | INTSTS0_VBSTS;
    case SH7764_USB_INTSTS1:
        return s->intsts1;
    case SH7764_USB_BRDYSTS:
        return s->brdysts;
    case SH7764_USB_NRDYSTS:
        return s->nrdysts;
    case SH7764_USB_BEMPSTS:
        return s->bempsts;
    case SH7764_USB_FRMNUM:
        return s->frmnum;
    case SH7764_USB_UFRMNUM:
        return s->ufrmnum;
    case SH7764_USB_USBADDR:
        return s->usbaddr;
    case SH7764_USB_USBREQ:
        return s->usbreq;
    case SH7764_USB_USBVAL:
        return s->usbval;
    case SH7764_USB_USBINDX:
        return s->usbindx;
    case SH7764_USB_USBLENG:
        return s->usbleng;
    case SH7764_USB_DCPCFG:
        return s->dcpcfg;
    case SH7764_USB_DCPMAXP:
        return s->dcpmaxp;
    case SH7764_USB_DCPCTR:
        return sh7764_usb_pipectr(s, 0);
    case SH7764_USB_PIPESEL:
        return s->pipesel;
    case SH7764_USB_PIPECFG:
        return s->pipesel ? s->pipe[s->pipesel].cfg : 0;
    case SH7764_USB_PIPEBUF:
        return s->pipesel ? s->pipe[s->pipesel].buf : 0;
    case SH7764_USB_PIPEMAXP:
        return s->pipesel ? s->pipe[s->pipesel].maxp : 0;
    case SH7764_USB_PIPEPERI:
        return s->pipesel ? s->pipe[s->pipesel].peri : 0;
    case SH7764_USB_PIPE1CTR ... SH7764_USB_PIPE9CTR:
        return sh7764_usb_pipectr(s, 1 + (addr - SH7764_USB_PIPE1CTR) / 2);
    case SH7764_USB_PIPE1TRE ... SH7764_USB_PIPE5TRN:
        if ((addr - SH7764_USB_PIPE1TRE) & 2) {
            return s->pipe[1 + (addr - SH7764_USB_PIPE1TRE) / 4].trn;
        }
        return s->pipe[1 + (addr - SH7764_USB_PIPE1TRE) / 4].tre;
    case SH7764_USB_DEVADD0 ... SH7764_USB_DEVADDA:
        return s->devadd[(addr - SH7764_USB_DEVADD0) / 2];
    default:
        qemu_log_mask(LOG_UNIMP, "sh7764-usb: read of unhandled register 0x%"
                      HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static uint64_t sh7764_usb_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7764USBState *s = opaque;
    uint64_t val = sh7764_usb_read_reg(s, addr, size);

    trace_usb_sh7764_reg_read(addr, val);
    return val;
}

/*
 * A bus reset drives the port and settles the speed. RHST reads back as the
 * reset running while USBRST is set and as the negotiated speed once it is
 * cleared, which is the sequence the manual describes and the one a driver
 * waits on.
 */
static void sh7764_usb_dvstctr_write(SH7764USBState *s, uint16_t val)
{
    uint16_t old = s->dvstctr;

    s->dvstctr = (s->dvstctr & DVSTCTR_RHST) |
                 (val & ~(uint16_t)DVSTCTR_RHST);

    if ((val & DVSTCTR_USBRST) && !(old & DVSTCTR_USBRST)) {
        s->dvstctr = (s->dvstctr & ~DVSTCTR_RHST) | RHST_RESETTING;
        if (s->port.dev && s->port.dev->attached) {
            usb_device_reset(s->port.dev);
        }
    } else if (!(val & DVSTCTR_USBRST) && (old & DVSTCTR_USBRST)) {
        unsigned rhst = RHST_NONE;

        if (s->port.dev && s->port.dev->attached) {
            rhst = (s->port.dev->speed == USB_SPEED_HIGH &&
                    (s->syscfg & SYSCFG_HSE)) ? RHST_HIGH : RHST_FULL;
        }
        s->dvstctr = (s->dvstctr & ~DVSTCTR_RHST) | rhst;
        trace_usb_sh7764_reset(rhst);
    }
}

static void sh7764_usb_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    SH7764USBState *s = opaque;
    unsigned n;

    trace_usb_sh7764_reg_write(addr, val);
    switch (addr) {
    case SH7764_USB_SYSCFG:
        s->syscfg = val;
        break;
    case SH7764_USB_BUSWAIT:
        s->buswait = val;
        break;
    case SH7764_USB_DVSTCTR:
        sh7764_usb_dvstctr_write(s, val);
        break;
    case SH7764_USB_TESTMODE:
        s->testmode = val;
        break;
    case SH7764_USB_D0FBCFG:
        s->d0fbcfg = val;
        break;
    case SH7764_USB_D1FBCFG:
        s->d1fbcfg = val;
        break;
    case SH7764_USB_CFIFO ... SH7764_USB_CFIFO + 3:
        sh7764_usb_fifo_write(s, 0, val, size);
        break;
    case SH7764_USB_D0FIFO ... SH7764_USB_D0FIFO + 3:
        sh7764_usb_fifo_write(s, 1, val, size);
        break;
    case SH7764_USB_D1FIFO ... SH7764_USB_D1FIFO + 3:
        sh7764_usb_fifo_write(s, 2, val, size);
        break;
    case SH7764_USB_CFIFOSEL:
    case SH7764_USB_D0FIFOSEL:
    case SH7764_USB_D1FIFOSEL:
        n = addr == SH7764_USB_CFIFOSEL ? 0 :
            addr == SH7764_USB_D0FIFOSEL ? 1 : 2;
        /*
         * Pointing the window at a pipe rewinds it, and so does REW. ISEL
         * says software means to write, which for the control pipe is the
         * only statement of direction it makes before the buffer is filled.
         */
        if ((val & FIFOSEL_CURPIPE) != (s->fifosel[n] & FIFOSEL_CURPIPE) ||
            (val & FIFOSEL_REW)) {
            SH7764USBPipe *p = &s->pipe[val & FIFOSEL_CURPIPE];

            p->pos = 0;
            if (val & FIFOSEL_ISEL) {
                p->len = 0;
            }
        }
        s->fifosel[n] = val & ~FIFOSEL_REW;
        break;
    case SH7764_USB_CFIFOCTR:
        sh7764_usb_fifoctr_write(s, 0, val);
        break;
    case SH7764_USB_D0FIFOCTR:
        sh7764_usb_fifoctr_write(s, 1, val);
        break;
    case SH7764_USB_D1FIFOCTR:
        sh7764_usb_fifoctr_write(s, 2, val);
        break;
    case SH7764_USB_INTENB0:
        s->intenb0 = val;
        break;
    case SH7764_USB_INTENB1:
        s->intenb1 = val;
        break;
    case SH7764_USB_BRDYENB:
        s->brdyenb = val;
        break;
    case SH7764_USB_NRDYENB:
        s->nrdyenb = val;
        break;
    case SH7764_USB_BEMPENB:
        s->bempenb = val;
        break;
    case SH7764_USB_SOFCFG:
        s->sofcfg = val;
        break;
    /* The status registers are cleared by writing zero to a bit. */
    case SH7764_USB_INTSTS0:
        s->intsts0 &= val;
        break;
    case SH7764_USB_INTSTS1:
        s->intsts1 &= val;
        break;
    case SH7764_USB_BRDYSTS:
        s->brdysts &= val;
        break;
    case SH7764_USB_NRDYSTS:
        s->nrdysts &= val;
        break;
    case SH7764_USB_BEMPSTS:
        s->bempsts &= val;
        break;
    case SH7764_USB_FRMNUM:
        s->frmnum = val;
        break;
    case SH7764_USB_USBREQ:
        s->usbreq = val;
        break;
    case SH7764_USB_USBVAL:
        s->usbval = val;
        break;
    case SH7764_USB_USBINDX:
        s->usbindx = val;
        break;
    case SH7764_USB_USBLENG:
        s->usbleng = val;
        break;
    case SH7764_USB_DCPCFG:
        s->dcpcfg = val;
        break;
    case SH7764_USB_DCPMAXP:
        s->dcpmaxp = val;
        break;
    case SH7764_USB_DCPCTR:
        sh7764_usb_pipectr_write(s, 0, val);
        break;
    case SH7764_USB_PIPESEL:
        s->pipesel = val & 0xf;
        break;
    case SH7764_USB_PIPECFG:
        if (s->pipesel) {
            s->pipe[s->pipesel].cfg = val;
        }
        break;
    case SH7764_USB_PIPEBUF:
        if (s->pipesel) {
            s->pipe[s->pipesel].buf = val;
        }
        break;
    case SH7764_USB_PIPEMAXP:
        if (s->pipesel) {
            s->pipe[s->pipesel].maxp = val;
        }
        break;
    case SH7764_USB_PIPEPERI:
        if (s->pipesel) {
            s->pipe[s->pipesel].peri = val;
        }
        break;
    case SH7764_USB_PIPE1CTR ... SH7764_USB_PIPE9CTR:
        sh7764_usb_pipectr_write(s, 1 + (addr - SH7764_USB_PIPE1CTR) / 2, val);
        break;
    case SH7764_USB_PIPE1TRE ... SH7764_USB_PIPE5TRN:
        n = 1 + (addr - SH7764_USB_PIPE1TRE) / 4;
        if ((addr - SH7764_USB_PIPE1TRE) & 2) {
            s->pipe[n].trn = val;
        } else {
            s->pipe[n].tre = val;
        }
        break;
    case SH7764_USB_DEVADD0 ... SH7764_USB_DEVADDA:
        s->devadd[(addr - SH7764_USB_DEVADD0) / 2] = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sh7764-usb: write of unhandled register 0x%"
                      HWADDR_PRIx " = 0x%" PRIx64 "\n", addr, val);
        break;
    }

    sh7764_usb_update_irq(s);
}

static const MemoryRegionOps sh7764_usb_ops = {
    .read = sh7764_usb_read,
    .write = sh7764_usb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---------------------------------------------------------------- port */

static void sh7764_usb_attach(USBPort *port)
{
    SH7764USBState *s = port->opaque;

    if (!port->dev || !port->dev->attached) {
        return;
    }

    /*
     * With the pull-downs enabled a device announces itself by pulling one
     * line up, which is the J state for anything that is not low speed.
     */
    trace_usb_sh7764_attach(port->dev->speed);
    s->syssts = port->dev->speed == USB_SPEED_LOW ? SYSSTS_LNST_K
                                                  : SYSSTS_LNST_J;
    sh7764_usb_raise1(s, INTSTS1_ATTCH | INTSTS1_BCHG);
}

static void sh7764_usb_detach(USBPort *port)
{
    SH7764USBState *s = port->opaque;

    s->syssts = SYSSTS_LNST_SE0;
    s->dvstctr &= ~DVSTCTR_RHST;
    sh7764_usb_raise1(s, INTSTS1_DTCH | INTSTS1_BCHG);
}

static void sh7764_usb_child_detach(USBPort *port, USBDevice *child)
{
    sh7764_usb_detach(port);
}

static void sh7764_usb_wakeup(USBPort *port)
{
    SH7764USBState *s = port->opaque;

    sh7764_usb_raise0(s, INTSTS0_RESM);
}

static USBPortOps sh7764_usb_port_ops = {
    .attach = sh7764_usb_attach,
    .detach = sh7764_usb_detach,
    .child_detach = sh7764_usb_child_detach,
    .wakeup = sh7764_usb_wakeup,
    .complete = sh7764_usb_async_complete,
};

static USBBusOps sh7764_usb_bus_ops = { };

static void sh7764_usb_reset(DeviceState *dev)
{
    SH7764USBState *s = SH7764_USB(dev);
    unsigned i;

    s->syscfg = 0;
    s->buswait = 0xf;
    s->syssts = SYSSTS_LNST_SE0;
    s->dvstctr = 0;
    s->testmode = 0;
    s->d0fbcfg = s->d1fbcfg = 0;
    s->intenb0 = s->intenb1 = 0;
    s->brdyenb = s->nrdyenb = s->bempenb = s->sofcfg = 0;
    s->intsts0 = s->intsts1 = 0;
    s->brdysts = s->nrdysts = s->bempsts = 0;
    s->frmnum = s->ufrmnum = 0;
    s->usbaddr = s->usbreq = s->usbval = s->usbindx = s->usbleng = 0;
    s->dcpcfg = s->dcpmaxp = s->dcpctr = 0;
    s->pipesel = 0;
    s->async_pipe = -1;

    for (i = 0; i < ARRAY_SIZE(s->fifosel); i++) {
        s->fifosel[i] = 0;
        s->fifoctr[i] = 0;
    }
    for (i = 0; i < SH7764_USB_NPIPES; i++) {
        memset(&s->pipe[i], 0, sizeof(s->pipe[i]));
    }
    memset(s->devadd, 0, sizeof(s->devadd));

    /* A device already on the port is still there after a controller reset. */
    if (s->port.dev && s->port.dev->attached) {
        sh7764_usb_attach(&s->port);
    }
    sh7764_usb_update_irq(s);
}

static void sh7764_usb_realize(DeviceState *dev, Error **errp)
{
    SH7764USBState *s = SH7764_USB(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &sh7764_usb_ops, s,
                          "sh7764-usb", SH7764_USB_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    usb_bus_new(&s->bus, sizeof(s->bus), &sh7764_usb_bus_ops, dev);
    usb_register_port(&s->bus, &s->port, s, 0, &sh7764_usb_port_ops,
                      USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL |
                      USB_SPEED_MASK_HIGH);
    usb_packet_init(&s->packet);
    s->async_pipe = -1;
}

static const VMStateDescription vmstate_sh7764_usb_pipe = {
    .name = "sh7764-usb-pipe",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(cfg, SH7764USBPipe),
        VMSTATE_UINT16(buf, SH7764USBPipe),
        VMSTATE_UINT16(maxp, SH7764USBPipe),
        VMSTATE_UINT16(peri, SH7764USBPipe),
        VMSTATE_UINT16(ctr, SH7764USBPipe),
        VMSTATE_UINT16(tre, SH7764USBPipe),
        VMSTATE_UINT16(trn, SH7764USBPipe),
        VMSTATE_BOOL(in, SH7764USBPipe),
        VMSTATE_BOOL(ready, SH7764USBPipe),
        VMSTATE_UINT32(len, SH7764USBPipe),
        VMSTATE_UINT32(pos, SH7764USBPipe),
        VMSTATE_UINT8_ARRAY(data, SH7764USBPipe, SH7764_USB_PIPE_BUF),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_sh7764_usb = {
    .name = "sh7764-usb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(syscfg, SH7764USBState),
        VMSTATE_UINT16(buswait, SH7764USBState),
        VMSTATE_UINT16(syssts, SH7764USBState),
        VMSTATE_UINT16(dvstctr, SH7764USBState),
        VMSTATE_UINT16(testmode, SH7764USBState),
        VMSTATE_UINT16(d0fbcfg, SH7764USBState),
        VMSTATE_UINT16(d1fbcfg, SH7764USBState),
        VMSTATE_UINT16_ARRAY(fifosel, SH7764USBState, 3),
        VMSTATE_UINT16_ARRAY(fifoctr, SH7764USBState, 3),
        VMSTATE_UINT16(intenb0, SH7764USBState),
        VMSTATE_UINT16(intenb1, SH7764USBState),
        VMSTATE_UINT16(brdyenb, SH7764USBState),
        VMSTATE_UINT16(nrdyenb, SH7764USBState),
        VMSTATE_UINT16(bempenb, SH7764USBState),
        VMSTATE_UINT16(sofcfg, SH7764USBState),
        VMSTATE_UINT16(intsts0, SH7764USBState),
        VMSTATE_UINT16(intsts1, SH7764USBState),
        VMSTATE_UINT16(brdysts, SH7764USBState),
        VMSTATE_UINT16(nrdysts, SH7764USBState),
        VMSTATE_UINT16(bempsts, SH7764USBState),
        VMSTATE_UINT16(frmnum, SH7764USBState),
        VMSTATE_UINT16(ufrmnum, SH7764USBState),
        VMSTATE_UINT16(usbaddr, SH7764USBState),
        VMSTATE_UINT16(usbreq, SH7764USBState),
        VMSTATE_UINT16(usbval, SH7764USBState),
        VMSTATE_UINT16(usbindx, SH7764USBState),
        VMSTATE_UINT16(usbleng, SH7764USBState),
        VMSTATE_UINT16(dcpcfg, SH7764USBState),
        VMSTATE_UINT16(dcpmaxp, SH7764USBState),
        VMSTATE_UINT16(dcpctr, SH7764USBState),
        VMSTATE_UINT16(pipesel, SH7764USBState),
        VMSTATE_UINT16_ARRAY(devadd, SH7764USBState, 11),
        VMSTATE_UINT8_ARRAY(setup, SH7764USBState, 8),
        VMSTATE_INT32(async_pipe, SH7764USBState),
        VMSTATE_STRUCT_ARRAY(pipe, SH7764USBState, SH7764_USB_NPIPES, 1,
                             vmstate_sh7764_usb_pipe, SH7764USBPipe),
        VMSTATE_END_OF_LIST()
    }
};

static void sh7764_usb_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sh7764_usb_realize;
    device_class_set_legacy_reset(dc, sh7764_usb_reset);
    dc->vmsd = &vmstate_sh7764_usb;
    dc->desc = "SH7764 USB 2.0 host controller";
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static const TypeInfo sh7764_usb_info = {
    .name          = TYPE_SH7764_USB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SH7764USBState),
    .class_init    = sh7764_usb_class_init,
};

static void sh7764_usb_register_types(void)
{
    type_register_static(&sh7764_usb_info);
}

type_init(sh7764_usb_register_types)
