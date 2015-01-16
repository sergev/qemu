/*
 * SPI ports.
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
#include "pic32_peripherals.h"

#include "pic32mz.h"

#define SPI_IRQ_FAULT   0               // error irq offset
#define SPI_IRQ_TX      1               // transmitter irq offset
#define SPI_IRQ_RX      2               // receiver irq offset

unsigned pic32_spi_readbuf (pic32_t *s, int unit)
{
    spi_t *p = &s->spi[unit];
    unsigned result = p->buf[p->rfifo];

    if (VALUE(p->con) & PIC32_SPICON_ENHBUF) {
        p->rfifo++;
        p->rfifo &= 3;
    }
    if (VALUE(p->stat) & PIC32_SPISTAT_SPIRBF) {
        VALUE(p->stat) &= ~PIC32_SPISTAT_SPIRBF;
        //s->irq_clear (s, p->irq + SPI_IRQ_RX);
    }
    return result;
}

void pic32_spi_writebuf (pic32_t *s, int unit, unsigned val)
{
    spi_t *p = &s->spi[unit];

    /* Perform SD card i/o on configured SPI port. */
    if (unit == s->sdcard_spi_port) {
        unsigned result = 0;

        if (VALUE(p->con) & PIC32_SPICON_MODE32) {
            /* 32-bit data width */
            result  = (unsigned char) pic32_sdcard_io (s, val >> 24) << 24;
            result |= (unsigned char) pic32_sdcard_io (s, val >> 16) << 16;
            result |= (unsigned char) pic32_sdcard_io (s, val >> 8) << 8;
            result |= (unsigned char) pic32_sdcard_io (s, val);

        } else if (VALUE(p->con) & PIC32_SPICON_MODE16) {
            /* 16-bit data width */
            result = (unsigned char) pic32_sdcard_io (s, val >> 8) << 8;
            result |= (unsigned char) pic32_sdcard_io (s, val);

        } else {
            /* 8-bit data width */
            result = (unsigned char) pic32_sdcard_io (s, val);
        }
        p->buf[p->wfifo] = result;
    } else {
        /* No device */
        p->buf[p->wfifo] = ~0;
    }
    if (VALUE(p->stat) & PIC32_SPISTAT_SPIRBF) {
        VALUE(p->stat) |= PIC32_SPISTAT_SPIROV;
        //s->irq_raise (s, p->irq + SPI_IRQ_FAULT);
    } else if (VALUE(p->con) & PIC32_SPICON_ENHBUF) {
        p->wfifo++;
        p->wfifo &= 3;
        if (p->wfifo == p->rfifo) {
            VALUE(p->stat) |= PIC32_SPISTAT_SPIRBF;
            //s->irq_raise (s, p->irq + SPI_IRQ_RX);
        }
    } else {
        VALUE(p->stat) |= PIC32_SPISTAT_SPIRBF;
        //s->irq_raise (s, p->irq + SPI_IRQ_RX);
    }
}

void pic32_spi_control (pic32_t *s, int unit)
{
    spi_t *p = &s->spi[unit];

    if (! (VALUE(p->con) & PIC32_SPICON_ON)) {
        s->irq_clear (s, p->irq + SPI_IRQ_FAULT);
        s->irq_clear (s, p->irq + SPI_IRQ_RX);
        s->irq_clear (s, p->irq + SPI_IRQ_TX);
        VALUE(p->stat) = PIC32_SPISTAT_SPITBE;
    } else if (! (VALUE(p->con) & PIC32_SPICON_ENHBUF)) {
        p->rfifo = 0;
        p->wfifo = 0;
    }
}

/*
 * Initialize the SPI data structure.
 */
void pic32_spi_init(pic32_t *s, int unit, int irq, int con, int stat)
{
    spi_t *p = &s->spi[unit];

    p->irq = irq;
    p->con = con;
    p->stat = stat;
    p->rfifo = 0;
    p->wfifo = 0;
}
