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

#define NAME_PIC32_ETH "pic32-eth"

/*
 * PIC32 Ethernet device.
 */
typedef struct _eth_t {
    SysBusDevice parent_obj;

    pic32_t     *pic32;
    NICState    *eth_nic;       /* virtual network interface */
    NICConf     eth_conf;       /* network configuration */
} eth_t;

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
static void eth_reset(eth_t *e)
{
    uint8_t *macaddr = e->eth_conf.macaddr.a;
    pic32_t *s = e->pic32;

    TRACE("--- %s()\n", __func__);
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
    eth_t *e = s->eth;

    TRACE("--- %s()\n", __func__);
    if (! (VALUE(ETHCON1) & PIC32_ETHCON1_ON)) {
        /* Ethernet controller is disabled. */
        eth_reset(e);
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

            qemu_send_packet(qemu_get_queue(e->eth_nic), buf, nbytes);
        }

        /* Activate TXDONE interrupt. */
        VALUE(ETHCON1) &= ~PIC32_ETHCON1_TXRTS;
        VALUE(ETHIRQ) |= PIC32_ETHIRQ_TXDONE;
        s->irq_raise(s, PIC32_IRQ_ETH);
    }
}

/*
 * Return true when we are ready to receive a packet.
 */
static int nic_can_receive(NetClientState *nc)
{
    eth_t *e = qemu_get_nic_opaque(nc);
    pic32_t *s = e->pic32;
    eth_desc_t d = { 0 };

    //TRACE("--- %s()\n", __func__);
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
static ssize_t nic_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    eth_t *e = qemu_get_nic_opaque(nc);
    pic32_t *s = e->pic32;

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
    if (e->rxcr & 0x10) {
        /* promiscuous: receive all */
    } else {
        if (memcmp(buf, broadcast_macaddr, 6) == 0) {
            /* broadcast address */
            if (!(e->rxcr & 0x04))
                return size;
        } else if (buf[0] & 0x01) {
            /* multicast */
            if (!(e->rxcr & 0x08))
                return size;
            mcast_idx = compute_mcast_idx(buf);
            if (!(e->mult[mcast_idx >> 3] & (1 << (mcast_idx & 7))))
                return size;
        } else if (e->mem[0] == buf[0] &&
                   e->mem[2] == buf[1] &&
                   e->mem[4] == buf[2] &&
                   e->mem[6] == buf[3] &&
                   e->mem[8] == buf[4] &&
                   e->mem[10] == buf[5]) {
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

    index = e->curpag << 8;
    /* 4 bytes for header */
    total_len = nbytes + 4;
    /* address for next packet (4 bytes for CRC) */
    next = index + ((total_len + 4 + 255) & ~0xff);
    if (next >= e->stop)
        next -= (e->stop - e->start);
    /* prepare packet header */
    p = e->mem + index;
    e->rsr = ENRSR_RXOK; /* receive status */
    /* XXX: check this */
    if (buf[0] & 0x01)
        e->rsr |= ENRSR_PHY;
    p[0] = e->rsr;
    p[1] = next >> 8;
    p[2] = total_len;
    p[3] = total_len >> 8;
    index += 4;

    /* write packet data */
    while (nbytes > 0) {
        if (index <= e->stop)
            avail = e->stop - index;
        else
            avail = 0;
        len = nbytes;
        if (len > avail)
            len = avail;
        memcpy(e->mem + index, buf, len);
        buf += len;
        index += len;
        if (index == e->stop)
            index = e->start;
        nbytes -= len;
    }
    e->curpag = next >> 8;

    /* now we can signal we have received something */
    e->isr |= ENISR_RX;
    eth_update_irq(s);
#endif
    return size;
}

/*
 * Deallocate the QEMU network interface.
 */
static void nic_cleanup(NetClientState *nc)
{
    eth_t *e = qemu_get_nic_opaque(nc);

    TRACE("--- %s()\n", __func__);
    e->eth_nic = 0;
}

/*
 * Initialize Ethernet device.
 */
void pic32_eth_init(pic32_t *s, NICInfo *nd)
{
    TRACE("--- %s()\n", __func__);

    /* Create Ethernet device. */
    s->eth_dev = qdev_create(NULL, NAME_PIC32_ETH);
    qdev_set_nic_properties(s->eth_dev, nd);
    qdev_init_nofail(s->eth_dev);

    /* Setup back pointer from Ethernet device to pic32 object. */
    s->eth = OBJECT_CHECK(eth_t, s->eth_dev, NAME_PIC32_ETH);
    s->eth->pic32 = s;
}

/*
 * Initialize pic32-eth object.
 */
static int eth_object_init(SysBusDevice *sbd)
{
    /* Table of methods for QEMU network interface. */
    static NetClientInfo nic_info = {
        .type           = NET_CLIENT_OPTIONS_KIND_NIC,
        .size           = sizeof(NICState),
        .can_receive    = nic_can_receive,
        .receive        = nic_receive,
        .cleanup        = nic_cleanup,
    };
    DeviceState *dev = DEVICE(sbd);
    eth_t *e = OBJECT_CHECK(eth_t, dev, NAME_PIC32_ETH);

    TRACE("--- %s()\n", __func__);

    /* Initialize MAC address. */
    qemu_macaddr_default_if_unset(&e->eth_conf.macaddr);

    /* Create a QEMU network interface. */
    e->eth_nic = qemu_new_nic(&nic_info, &e->eth_conf,
        object_get_typename(OBJECT(dev)), dev->id, e);

    qemu_format_nic_info_str(qemu_get_queue(e->eth_nic),
        e->eth_conf.macaddr.a);

    return 0;
}

/*
 * Reset pic32-eth object.
 */
static void eth_object_reset(DeviceState *dev)
{
    eth_t *e = OBJECT_CHECK(eth_t, dev, NAME_PIC32_ETH);

    TRACE("--- %s()\n", __func__);
    eth_reset(e);
}

/*
 * Descriptor of eth_t data structure.
 */
static const VMStateDescription vmstate_info = {
    .name               = NAME_PIC32_ETH,
    .version_id         = 0,
    .minimum_version_id = 0,
    .fields             = (VMStateField[]) {
#if 0
        //TODO
        VMSTATE_UINT32(busy, MIPSnetState),
        VMSTATE_UINT32(rx_count, MIPSnetState),
        VMSTATE_UINT32(rx_read, MIPSnetState),
        VMSTATE_UINT32(tx_count, MIPSnetState),
        VMSTATE_UINT32(tx_written, MIPSnetState),
        VMSTATE_UINT32(intctl, MIPSnetState),
        VMSTATE_BUFFER(rx_buffer, MIPSnetState),
        VMSTATE_BUFFER(tx_buffer, MIPSnetState),
#endif
        VMSTATE_END_OF_LIST()
    }
};

/*
 * Build pic32-eth object.
 */
static void eth_object_constructor(ObjectClass *klass, void *data)
{
    static Property eth_properties[] = {
        DEFINE_NIC_PROPERTIES(eth_t, eth_conf),
        DEFINE_PROP_END_OF_LIST(),
    };
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = eth_object_init;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "PIC32 Ethernet device";
    dc->reset = eth_object_reset;
    dc->vmsd = &vmstate_info;
    dc->props = eth_properties;
}

/*
 * Instantiate pic32-eth class.
 */
static void eth_register_types(void)
{
    static const TypeInfo eth_class_info = {
        .name          = NAME_PIC32_ETH,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(eth_t),
        .class_init    = eth_object_constructor,
    };

    type_register_static(&eth_class_info);
}

type_init(eth_register_types)
