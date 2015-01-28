/*
 * Ethernet port.
 *
 * Copyright (C) 2014-2015 Serge Vakulenko
 *
 * Permission to use, copy, modify, and distribute this software
 * and its documentation for any purpose and without fee is hereby
 * granted, provided that the above copyright notice appear in all
 * copies and that both that the copyright notice and this
 * permission notice and warranty disclaimer appear in supporting
 * documentation, and that the name of the author not be used in
 * advertising or publicity pertaining to distribution of the
 * software without specific, written prior permission.
 *
 * The author disclaim all warranties with regard to this
 * software, including all implied warranties of merchantability
 * and fitness.  In no event shall the author be liable for any
 * special, indirect or consequential damages or any damages
 * whatsoever resulting from loss of use, data or profits, whether
 * in an action of contract, negligence or other tortious action,
 * arising out of or in connection with the use or performance of
 * this software.
 */
#include "hw/hw.h"
#include "exec/address-spaces.h"
#include "pic32_peripherals.h"

#include "pic32mz.h"

#define TRACE       printf
#ifndef TRACE
#define TRACE(...)  /*empty*/
#endif

/*
 * DMA buffer descriptor.
 */
typedef struct {
    uint32_t hdr;               /* Flags */
    uint32_t paddr;             /* Phys address of data buffer */
    uint32_t ctl;               /* TX options / RX filter status */
    uint32_t status;            /* Status */
} eth_desc_t;

/* Start of packet */
#define DESC_SOP(d)         ((d)->hdr & 0x80000000)
#define DESC_SET_SOP(d)     (d)->hdr |= 0x80000000

/* End of packet */
#define DESC_EOP(d)         ((d)->hdr & 0x40000000)
#define DESC_SET_EOP(d)     (d)->hdr |= 0x40000000

/* Number of data bytes */
#define DESC_BYTECNT(d)     ((d)->hdr >> 16 & 0x7ff)
#define DESC_SET_BYTECNT(d,n) ((d)->hdr |= (n) << 16)

/* Next descriptor pointer valid */
#define DESC_SET_NPV(d)     (d)->hdr |= 0x00000100
#define DESC_CLEAR_NPV(d)   (d)->hdr &= ~0x00000100

/* Eth controller owns this desc */
#define DESC_EOWN(d)        ((d)->hdr & 0x00000080)
#define DESC_SET_EOWN(d)    (d)->hdr |= 0x00000080
#define DESC_CLEAR_EOWN(d)  (d)->hdr &= ~0x00000080

/* Size of received packet */
#define DESC_FRAMESZ(d)     ((d)->status & 0xffff)

/*
 * Reset the Ethernet controller.
 */
static void eth_reset(pic32_t *s)
{
    uint8_t *macaddr = s->eth_conf.macaddr.a;

    s->irq_clear(s, PIC32_IRQ_ETH);
    VALUE(ETHSTAT) = 0;
    VALUE(EMAC1SA2) = macaddr[0] | (macaddr[1] << 8);
    VALUE(EMAC1SA1) = macaddr[2] | (macaddr[3] << 8);
    VALUE(EMAC1SA0) = macaddr[4] | (macaddr[5] << 8);
}

/*
 * Write to ETHCON1 register.
 */
void pic32_eth_control(pic32_t *s)
{
    if (! (VALUE(ETHCON1) & PIC32_ETHCON1_ON)) {
        /* Ethernet controller is disabled. */
        eth_reset(s);
        return;
    }

    if (VALUE(ETHCON1) & PIC32_ETHCON1_TXRTS) {
        /* Transmit request. */
        eth_desc_t d = { 0 };
        unsigned paddr = VALUE(ETHTXST);
        unsigned nbytes, i;
        unsigned char buf[2048];

        //TODO
        d.hdr   = ldl_le_phys(&address_space_memory, paddr);
        d.paddr = ldl_le_phys(&address_space_memory, paddr + 4);
        TRACE("--- eth transmit request: desc [%08x] = %08x %08x\n", paddr, d.hdr, d.paddr);

        nbytes = DESC_BYTECNT(&d);
        if (nbytes > 0 && nbytes <= sizeof(buf)) {
            for (i=0; i<nbytes; i+=4)
                *(uint32_t*) (buf + i) = ldl_le_phys(&address_space_memory,
                                                     d.paddr + i);

            TRACE("--- %u bytes of data: %02x", nbytes, buf[0]);
            for (i=1; i<nbytes; i++)
                TRACE("-%02x", buf[i]);
            TRACE("\n");

            qemu_send_packet(qemu_get_queue(s->eth_nic), buf, nbytes);
        }

        /* Activate TXDONE interrupt. */
        VALUE(ETHCON1) &= ~PIC32_ETHCON1_TXRTS;
        VALUE(ETHIRQ) |= PIC32_ETHIRQ_TXDONE;
        s->irq_raise(s, PIC32_IRQ_ETH);
    }
}

/*
 * Return true when we aready to receive a packet.
 */
static int eth_can_receive(NetClientState *nc)
{
    pic32_t *s = qemu_get_nic_opaque(nc);
    eth_desc_t d = { 0 };

    TRACE("--- %s()\n", __func__);
    if (! (VALUE(ETHCON1) & PIC32_ETHCON1_ON)) {
        /* While interface is down,
         * we can receive anything (and discard). */
        return 1;
    }

    /* Check whether we have at least one receive buffer available. */
    d.hdr = ldl_le_phys(&address_space_memory, VALUE(ETHRXST));
    return DESC_EOWN(&d);
}


/*
 * Receive a packet.
 */
static ssize_t eth_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    pic32_t *s = qemu_get_nic_opaque(nc);

    TRACE("--- %s: %d bytes\n", __func__, (int) size);
    if (! (VALUE(ETHCON1) & PIC32_ETHCON1_ON)) {
        /* Ethernet controller is down. */
        return -1;
    }
#if 0
    int nbytes;
    uint8_t *p;
    unsigned int total_len, next, avail, len, index, mcast_idx;
    uint8_t buf1[60];
    static const uint8_t broadcast_macaddr[6] =
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
#define MIN_BUF_SIZE 60

    if (eth_buffer_full(s))
        return -1;

    /* XXX: check this */
    if (s->rxcr & 0x10) {
        /* promiscuous: receive all */
    } else {
        if (memcmp(buf, broadcast_macaddr, 6) == 0) {
            /* broadcast address */
            if (!(s->rxcr & 0x04))
                return size;
        } else if (buf[0] & 0x01) {
            /* multicast */
            if (!(s->rxcr & 0x08))
                return size;
            mcast_idx = compute_mcast_idx(buf);
            if (!(s->mult[mcast_idx >> 3] & (1 << (mcast_idx & 7))))
                return size;
        } else if (s->mem[0] == buf[0] &&
                   s->mem[2] == buf[1] &&
                   s->mem[4] == buf[2] &&
                   s->mem[6] == buf[3] &&
                   s->mem[8] == buf[4] &&
                   s->mem[10] == buf[5]) {
            /* match */
        } else {
            return size;
        }
    }

    /* if too small buffer, then expand it */
    nbytes = size;
    if (nbytes < MIN_BUF_SIZE) {
        memcpy(buf1, buf, nbytes);
        memset(buf1 + nbytes, 0, MIN_BUF_SIZE - nbytes);
        buf = buf1;
        nbytes = MIN_BUF_SIZE;
    }

    index = s->curpag << 8;
    /* 4 bytes for header */
    total_len = nbytes + 4;
    /* address for next packet (4 bytes for CRC) */
    next = index + ((total_len + 4 + 255) & ~0xff);
    if (next >= s->stop)
        next -= (s->stop - s->start);
    /* prepare packet header */
    p = s->mem + index;
    s->rsr = ENRSR_RXOK; /* receive status */
    /* XXX: check this */
    if (buf[0] & 0x01)
        s->rsr |= ENRSR_PHY;
    p[0] = s->rsr;
    p[1] = next >> 8;
    p[2] = total_len;
    p[3] = total_len >> 8;
    index += 4;

    /* write packet data */
    while (nbytes > 0) {
        if (index <= s->stop)
            avail = s->stop - index;
        else
            avail = 0;
        len = nbytes;
        if (len > avail)
            len = avail;
        memcpy(s->mem + index, buf, len);
        buf += len;
        index += len;
        if (index == s->stop)
            index = s->start;
        nbytes -= len;
    }
    s->curpag = next >> 8;

    /* now we can signal we have received something */
    s->isr |= ENISR_RX;
    eth_update_irq(s);
#endif
    return size;
}

/*
 * Deallocate the QEMU network interface.
 */
static void eth_cleanup(NetClientState *nc)
{
    pic32_t *s = qemu_get_nic_opaque(nc);

    s->eth_nic = 0;
}

/*
 * Table of methods for QEMU network interface.
 */
static NetClientInfo eth_info = {
    .type           = NET_CLIENT_OPTIONS_KIND_NIC,
    .size           = sizeof(NICState),
    .can_receive    = eth_can_receive,
    .receive        = eth_receive,
    .cleanup        = eth_cleanup,
};

/*
 * Initialize Ethernet data.
 */
void pic32_eth_init(pic32_t *s)
{
    /* Initialize MAC address. */
    qemu_macaddr_default_if_unset(&s->eth_conf.macaddr);
    eth_reset(s);

    /* Create a QEMU network interface. */
    s->eth_nic = qemu_new_nic(&eth_info, &s->eth_conf, "pic32", "eth", s);
    qemu_format_nic_info_str(qemu_get_queue(s->eth_nic), s->eth_conf.macaddr.a);
}
