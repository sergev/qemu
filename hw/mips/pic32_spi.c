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
    unsigned result = s->spi_buf[unit][s->spi_rfifo[unit]];

    if (VALUE(s->spi_con[unit]) & PIC32_SPICON_ENHBUF) {
        s->spi_rfifo[unit]++;
        s->spi_rfifo[unit] &= 3;
    }
    if (VALUE(s->spi_stat[unit]) & PIC32_SPISTAT_SPIRBF) {
        VALUE(s->spi_stat[unit]) &= ~PIC32_SPISTAT_SPIRBF;
        //s->irq_clear (s, s->spi_irq[unit] + SPI_IRQ_RX);
    }
    return result;
}

void pic32_spi_writebuf (pic32_t *s, int unit, unsigned val)
{
    /* Perform SD card i/o on configured SPI port. */
    if (unit == s->sdcard_spi_port) {
        unsigned result = 0;

        if (VALUE(s->spi_con[unit]) & PIC32_SPICON_MODE32) {
            /* 32-bit data width */
            result  = (unsigned char) pic32_sdcard_io (s, val >> 24) << 24;
            result |= (unsigned char) pic32_sdcard_io (s, val >> 16) << 16;
            result |= (unsigned char) pic32_sdcard_io (s, val >> 8) << 8;
            result |= (unsigned char) pic32_sdcard_io (s, val);

        } else if (VALUE(s->spi_con[unit]) & PIC32_SPICON_MODE16) {
            /* 16-bit data width */
            result = (unsigned char) pic32_sdcard_io (s, val >> 8) << 8;
            result |= (unsigned char) pic32_sdcard_io (s, val);

        } else {
            /* 8-bit data width */
            result = (unsigned char) pic32_sdcard_io (s, val);
        }
        s->spi_buf[unit][s->spi_wfifo[unit]] = result;
    } else {
        /* No device */
        s->spi_buf[unit][s->spi_wfifo[unit]] = ~0;
    }
    if (VALUE(s->spi_stat[unit]) & PIC32_SPISTAT_SPIRBF) {
        VALUE(s->spi_stat[unit]) |= PIC32_SPISTAT_SPIROV;
        //s->irq_raise (s, s->spi_irq[unit] + SPI_IRQ_FAULT);
    } else if (VALUE(s->spi_con[unit]) & PIC32_SPICON_ENHBUF) {
        s->spi_wfifo[unit]++;
        s->spi_wfifo[unit] &= 3;
        if (s->spi_wfifo[unit] == s->spi_rfifo[unit]) {
            VALUE(s->spi_stat[unit]) |= PIC32_SPISTAT_SPIRBF;
            //s->irq_raise (s, s->spi_irq[unit] + SPI_IRQ_RX);
        }
    } else {
        VALUE(s->spi_stat[unit]) |= PIC32_SPISTAT_SPIRBF;
        //s->irq_raise (s, s->spi_irq[unit] + SPI_IRQ_RX);
    }
}

void pic32_spi_control (pic32_t *s, int unit)
{
    if (! (VALUE(s->spi_con[unit]) & PIC32_SPICON_ON)) {
        s->irq_clear (s, s->spi_irq[unit] + SPI_IRQ_FAULT);
        s->irq_clear (s, s->spi_irq[unit] + SPI_IRQ_RX);
        s->irq_clear (s, s->spi_irq[unit] + SPI_IRQ_TX);
        VALUE(s->spi_stat[unit]) = PIC32_SPISTAT_SPITBE;
    } else if (! (VALUE(s->spi_con[unit]) & PIC32_SPICON_ENHBUF)) {
        s->spi_rfifo[unit] = 0;
        s->spi_wfifo[unit] = 0;
    }
}
