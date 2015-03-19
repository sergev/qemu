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
#include "sysemu/dma.h"
#include "pic32_peripherals.h"

#include "pic32mz.h"

//#define TRACE       printf
#ifndef TRACE
#define TRACE(...)  /*empty*/
#endif

#define NAME_PIC32_ETH "pic32-eth"

#define MIN_RX_SIZE 60

/*
 * PIC32 Ethernet device.
 */
struct _eth_t {
    SysBusDevice parent_obj;

    pic32_t     *pic32;
    NICState    *eth_nic;       /* virtual network interface */
    NICConf     eth_conf;       /* network configuration */

    uint16_t    phy_reg[32];    /* PHY registers */

};

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
#define DESC_SOP(d)             ((d)->hdr & 0x80000000)
#define DESC_SET_SOP(d)         ((d)->hdr |= 0x80000000)
#define DESC_CLEAR_SOP(d)       ((d)->hdr &= ~0x80000000)

/* End of packet */
#define DESC_EOP(d)             ((d)->hdr & 0x40000000)
#define DESC_SET_EOP(d)         ((d)->hdr |= 0x40000000)
#define DESC_CLEAR_EOP(d)       ((d)->hdr &= ~0x40000000)

/* Number of data bytes */
#define DESC_BYTECNT(d)         ((d)->hdr >> 16 & 0x7ff)
#define DESC_SET_BYTECNT(d,n)   ((d)->hdr = ((d)->hdr & ~0x7ff0000) | (n) << 16)

/* Next descriptor pointer valid */
#define DESC_NPV(d)             ((d)->hdr & 0x00000100)
#define DESC_SET_NPV(d)         ((d)->hdr |= 0x00000100)
#define DESC_CLEAR_NPV(d)       ((d)->hdr &= ~0x00000100)

/* Eth controller owns this desc */
#define DESC_EOWN(d)            ((d)->hdr & 0x00000080)
#define DESC_SET_EOWN(d)        ((d)->hdr |= 0x00000080)
#define DESC_CLEAR_EOWN(d)      ((d)->hdr &= ~0x00000080)

/* Size of received packet */
#define DESC_FRAMESZ(d)         ((d)->status & 0xffff)
#define DESC_SET_FRAMESZ(d,n)   ((d)->status = ((d)->status & ~0xffff) | (n))

/* Receive filter status */
#define DESC_RXF(d)             ((d)->ctl >> 24)
#define DESC_SET_RXF(d,n)       ((d)->ctl = ((d)->ctl & 0xffffff) | (n) << 24)

/*-------------------------------------------------------------
 * PHY declarations for SMSC LAN8720A/8740A chip.
 */
#define PHY_CONTROL             0       /* Basic Control Register */
#define PHY_STATUS              1       /* Basic Status Register */
#define PHY_ID1                 2       /* PHY identifier 1 */
#define PHY_ID2                 3       /* PHY identifier 2 */
#define PHY_MODE                18      /* Special Modes */
#define PHY_SPECIAL             31      /* Special Control/Status Register */

#define PHY_CONTROL_RESET       0x8000  /* Soft reset, bit self cleared */

#define PHY_STATUS_CAP_100_FDX  0x4000  /* Can do 100Base-TX full duplex */
#define PHY_STATUS_CAP_100_HDX  0x2000  /* Can do 100Base-TX half duplex */
#define PHY_STATUS_CAP_10_FDX   0x1000  /* Can do 10Base-TX full duplex */
#define PHY_STATUS_CAP_10_HDX   0x0800  /* Can do 10Base-TX half duplex */
#define PHY_STATUS_ANEG_ACK     0x0020  /* Auto-negotiation acknowledge */
#define PHY_STATUS_CAP_ANEG     0x0008  /* Auto-negotiation available */
#define PHY_STATUS_LINK         0x0004  /* Link valid */

#define PHY_MODE_PHYAD          0x000f  /* PHY address mask */

#define PHY_SPECIAL_AUTODONE    0x1000  /* Auto-negotiation is done */
#define PHY_SPECIAL_FDX         0x0010  /* Full duplex */
#define PHY_SPECIAL_100         0x0008  /* Speed 100 Mbps */

/*
 * Reset the PHY chip.
 */
static void phy_reset(eth_t *e)
{
    TRACE("---     PHY reset\n");
    e->phy_reg[PHY_ID1]     = 0x0007;               /* Vendor: SMSC */
    e->phy_reg[PHY_ID2]     = 0xc111;               /* Device: LAN8740A */
    e->phy_reg[PHY_CONTROL] = 0x1000;
    e->phy_reg[PHY_MODE]    = 0x4000;               /* RMII interface */
    e->phy_reg[PHY_STATUS]  = PHY_STATUS_ANEG_ACK |
                              PHY_STATUS_CAP_ANEG |
                              PHY_STATUS_LINK |
                              PHY_STATUS_CAP_100_FDX |
                              PHY_STATUS_CAP_100_HDX |
                              PHY_STATUS_CAP_10_FDX |
                              PHY_STATUS_CAP_10_HDX;
    e->phy_reg[PHY_SPECIAL] = PHY_SPECIAL_AUTODONE |
                              PHY_SPECIAL_FDX |
                              PHY_SPECIAL_100;
}

/*
 * Reset the Ethernet controller.
 */
static void eth_reset(eth_t *e)
{
    uint8_t *macaddr = e->eth_conf.macaddr.a;
    pic32_t *s = e->pic32;

    TRACE("--- %s()\n", __func__);
    if (qemu_loglevel_mask(CPU_LOG_INSTR))
        fprintf(qemu_logfile, "--- Ethernet reset\n");
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

    if (! (VALUE(ETHCON1) & PIC32_ETHCON1_ON)) {
        /* Ethernet controller is disabled. */
        TRACE("--- %s() ETH disabled\n", __func__);
        eth_reset(e);
        return;
    }

    if (VALUE(ETHCON1) & PIC32_ETHCON1_TXRTS) {
        /* Transmit request. */
        eth_desc_t d = { 0 };
        unsigned paddr = VALUE(ETHTXST);
        unsigned nbytes, i;
        unsigned char buf[2048];

        d.hdr   = ldl_le_phys(&address_space_memory, paddr);
        d.paddr = ldl_le_phys(&address_space_memory, paddr + 4);

        nbytes = DESC_BYTECNT(&d);
        TRACE("--- %s() trasmit request %u bytes\n", __func__, nbytes);
        TRACE("--- TX desc [%08x] = %08x %08x\n", paddr, d.hdr, d.paddr);
        if (qemu_loglevel_mask(CPU_LOG_INSTR))
            fprintf(qemu_logfile, "--- Ethernet transmit request, %u bytes\n", nbytes);
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
 * Write to EMAC1MCMD register.
 */
void pic32_mii_command(pic32_t *s)
{
    eth_t *e = s->eth;
    unsigned cmd = VALUE(EMAC1MCMD);
    unsigned addr = VALUE(EMAC1MADR);
    unsigned data;

    if (cmd & (PIC32_EMAC1MCMD_READ | PIC32_EMAC1MCMD_SCAN)) {
        data = e->phy_reg[addr & 0x1f];
        //TRACE("--- %s() cmd = %04x, addr = %04x\n", __func__, cmd, addr);
        TRACE("--- %s() read register [%u] = %04x\n", __func__, addr & 0x1f, data);
        if (qemu_loglevel_mask(CPU_LOG_INSTR))
            fprintf(qemu_logfile, "--- Ethernet MII read register [%u] = %04x\n",
                addr & 0x1f, data);
        VALUE(EMAC1MRDD) = data;
    }
}

/*
 * Write to EMAC1MWTD register.
 */
void pic32_mii_write(pic32_t *s)
{
    eth_t *e = s->eth;
    unsigned addr = VALUE(EMAC1MADR);
    unsigned data = VALUE(EMAC1MWTD);

    TRACE("--- %s() write register [%u] = %04x\n", __func__, addr & 0x1f, data);
    if (qemu_loglevel_mask(CPU_LOG_INSTR))
        fprintf(qemu_logfile, "--- Ethernet MII write register [%u] = %04x\n",
            addr & 0x1f, data);
    switch (addr & 0x1f) {
    case PHY_CONTROL:                   /* Basic Control Register */
        e->phy_reg[PHY_CONTROL] = data;
        if (data & PHY_CONTROL_RESET) {
            phy_reset(e);
        }
        break;
    }
}

/*
 * Return true when no receive buffers available.
 */
static int eth_buffer_full(pic32_t *s)
{
    eth_desc_t d = { 0 };

    d.hdr = ldl_le_phys(&address_space_memory, VALUE(ETHRXST));
    return !DESC_EOWN(&d);
}

/*
 * Return true when we are ready to receive a packet.
 */
static int nic_can_receive(NetClientState *nc)
{
    eth_t *e = qemu_get_nic_opaque(nc);
    pic32_t *s = e->pic32;

    //TRACE("--- %s()\n", __func__);
    if (! (VALUE(ETHCON1) & PIC32_ETHCON1_ON)) {
        /* While interface is down,
         * we can receive anything (and discard). */
        return 1;
    }

    /* Check whether we have at least one receive buffer available. */
    return !eth_buffer_full(s);
}

/*
 * Receive a packet.
 */
static ssize_t nic_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    eth_t *e = qemu_get_nic_opaque(nc);
    pic32_t *s = e->pic32;
    unsigned rxfc, rxf, rxst, rxst0, rxbufsz, nbytes, len;
    int start_of_packet;
    uint8_t buf1[60];
    eth_desc_t *d, desc0, desc1;

    TRACE("--- %s: %d bytes\n", __func__, (int) size);
    if (qemu_loglevel_mask(CPU_LOG_INSTR))
        fprintf(qemu_logfile, "--- Ethernet receive, %u bytes\n", (unsigned)size);
    if (! (VALUE(ETHCON1) & PIC32_ETHCON1_ON)) {
        /* Ethernet controller is down. */
        return -1;
    }

    if (eth_buffer_full(s))
        return -1;

    /* Cast acceptance filter. */
    rxfc = VALUE(ETHRXFC);
    if (buf[0] == 0xff && buf[1] == 0xff && buf[2] == 0xff &&
        buf[3] == 0xff && buf[4] == 0xff && buf[5] == 0xff) {
        /* Broadcast address. */
        if (! (rxfc & PIC32_ETHRXFC_BCEN))
            return size;
        rxf = 0x40;
    } else if (buf[0] & 0x01) {
        /* Multicast. */
        if (! (rxfc & PIC32_ETHRXFC_MCEN))
            return size;
        rxf = 0x80;
    } else if (VALUE(EMAC1SA2) == (buf[0] | (buf[1] << 8)) ||
               VALUE(EMAC1SA2) == (buf[2] | (buf[3] << 8)) ||
               VALUE(EMAC1SA2) == (buf[4] | (buf[5] << 8))) {
        /* Unicast for our MAC address. */
        if (! (rxfc & PIC32_ETHRXFC_UCEN))
            return size;
        rxf = 0x20;
    } else {
        /* Not-Me Unicast. */
        if (! (rxfc & PIC32_ETHRXFC_NOTMEEN))
            return size;
        rxf = 0x00;
    }

    /* If packet too small, then expand it */
    nbytes = size;
    if (nbytes < MIN_RX_SIZE) {
        memcpy(buf1, buf, nbytes);
        memset(buf1 + nbytes, 0, MIN_RX_SIZE - nbytes);
        buf = buf1;
        nbytes = MIN_RX_SIZE;
    }

    /* Copy data to descriptor chain. */
    rxst0 = VALUE(ETHRXST);
    rxbufsz = VALUE(ETHCON2) & 0x7f0;
    start_of_packet = 1;
    d = &desc0;
    rxst = rxst0;
    for (;;) {
        /* Get descriptor. */
        dma_memory_read(&address_space_memory, rxst, d, 16);
        if (! DESC_EOWN(d) || (! start_of_packet && rxst == rxst0)) {
            TRACE("--- No more descriptors available\n");
            VALUE(ETHIRQ) |= PIC32_ETHIRQ_RXBUFNA;
            break;
        }

        /* Copy data to buffer. */
        len = nbytes;
        if (len > rxbufsz)
            len = rxbufsz;
        TRACE("--- Copy %d bytes to %08x\n", len, d->paddr);
        dma_memory_write(&address_space_memory, d->paddr, buf, len);
        buf += len;

        /* Update the descriptor. */
        DESC_CLEAR_EOWN(d);
        DESC_SET_BYTECNT(d, len);
        if (start_of_packet) {
            DESC_SET_SOP(d);
            DESC_SET_FRAMESZ(d, nbytes);
            DESC_SET_RXF(d, rxf);
            start_of_packet = 0;
        } else
            DESC_CLEAR_SOP(d);
        if (nbytes == len)
            DESC_SET_EOP(d);
        else
            DESC_CLEAR_EOP(d);

        /* Write the descriptor back memory (all but the first). */
        if (d == &desc1) {
            TRACE("--- RX desc [%08x] = %08x ... ... %08x\n", rxst, desc1.hdr, desc1.status);
            dma_memory_write(&address_space_memory, rxst, &desc1, 16);
        }

        /* Switch to next descriptor. */
        rxst += 16;
        if (DESC_NPV(d))
            rxst = ldl_le_phys(&address_space_memory, rxst);
        VALUE(ETHRXST) = rxst;
        d = &desc1;

        nbytes -= len;
        if (nbytes == 0) {
            /* Update the first descriptor. */
            TRACE("--- RX desc [%08x] = %08x ... ... %08x\n", rxst0, desc0.hdr, desc0.status);
            dma_memory_write(&address_space_memory, rxst0, &desc0, 16);

            /* Packet successfully received. */
            VALUE(ETHIRQ) |= PIC32_ETHIRQ_RXDONE;
            break;
        }
    }

    /* Activate interrupt. */
    s->irq_raise(s, PIC32_IRQ_ETH);
    return size;
}

/*
 * Deallocate the QEMU network interface.
 */
static void nic_cleanup(NetClientState *nc)
{
    eth_t *e = qemu_get_nic_opaque(nc);

    //TRACE("--- %s()\n", __func__);
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

    //TRACE("--- %s()\n", __func__);

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

    //TRACE("--- %s()\n", __func__);
    eth_reset(e);
    phy_reset(e);
}

/*
 * Descriptor of eth_t data structure.
 */
static const VMStateDescription vmstate_info = {
    .name               = NAME_PIC32_ETH,
    .version_id         = 0,
    .minimum_version_id = 0,
    .fields             = (VMStateField[]) {
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
