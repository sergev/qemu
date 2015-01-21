/*
 * UART ports.
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
#include "hw/char/serial.h"
#include "sysemu/char.h"
#include "pic32_peripherals.h"

#include "pic32mz.h"

#define UART_IRQ_ERR    0               // error irq offset
#define UART_IRQ_RX     1               // receiver irq offset
#define UART_IRQ_TX     2               // transmitter irq offset

/*
 * Read of UxRXREG register.
 */
unsigned pic32_uart_get_char (pic32_t *s, int unit)
{
    uart_t *u = &s->uart[unit];
    unsigned value;

    /* Read a byte from input queue. */
    value = u->rxbyte;
    VALUE(u->sta) &= ~PIC32_USTA_URXDA;

    s->irq_clear (s, u->irq + UART_IRQ_RX);
    return value;
}

/*
 * Write to UxTXREG register.
 */
void pic32_uart_put_char (pic32_t *s, int unit, unsigned char byte)
{
    uart_t *u = &s->uart[unit];

    if (! u->chr) {
        printf("--- %s(unit = %u) serial port not configured\n",
            __func__, unit);
        return;
    }

    /* Send the byte. */
    if (qemu_chr_fe_write(u->chr, &byte, 1) != 1) {
        //TODO: suspend simulation until serial port ready
        printf("--- %s(unit = %u) failed\n", __func__, unit);
        return;
    }

    if ((VALUE(u->mode) & PIC32_UMODE_ON) &&
        (VALUE(u->sta) & PIC32_USTA_UTXEN))
    {
        VALUE(u->sta) |= PIC32_USTA_UTXBF;

        /* Generate TX interrupt with some delay. */
        timer_mod(u->transmit_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
            get_ticks_per_sec() / 5000 + 10);
        u->oactive = 1;
    }
}

/*
 * Called before reading a value of UxBRG, UxMODE or UxSTA registers.
 */
void pic32_uart_poll_status (pic32_t *s, int unit)
{
    uart_t *u = &s->uart[unit];

    // Keep receiver idle, transmit shift register always empty
    VALUE(u->sta) |= PIC32_USTA_RIDLE | PIC32_USTA_TRMT;

    //printf ("<%x>", VALUE(u->sta)); fflush (stdout);
}

/*
 * Write to UxMODE register.
 */
void pic32_uart_update_mode (pic32_t *s, int unit)
{
    uart_t *u = &s->uart[unit];

    if (! (VALUE(u->mode) & PIC32_UMODE_ON)) {
        s->irq_clear (s, u->irq + UART_IRQ_RX);
        s->irq_clear (s, u->irq + UART_IRQ_TX);
        VALUE(u->sta) &= ~(PIC32_USTA_URXDA | PIC32_USTA_FERR |
                           PIC32_USTA_PERR | PIC32_USTA_UTXBF);
        VALUE(u->sta) |= PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    }
}

/*
 * Write to UxSTA register.
 */
void pic32_uart_update_status (pic32_t *s, int unit)
{
    uart_t *u = &s->uart[unit];

    if (! (VALUE(u->sta) & PIC32_USTA_URXEN)) {
        s->irq_clear (s, u->irq + UART_IRQ_RX);
        VALUE(u->sta) &= ~(PIC32_USTA_URXDA | PIC32_USTA_FERR |
                           PIC32_USTA_PERR);
    }
    if (! (VALUE(u->sta) & PIC32_USTA_UTXEN)) {
        s->irq_clear (s, u->irq + UART_IRQ_TX);
        VALUE(u->sta) &= ~PIC32_USTA_UTXBF;
        VALUE(u->sta) |= PIC32_USTA_TRMT;
    }
}

/*
 * Return a number of free bytes in the receive FIFO.
 */
static int uart_can_receive(void *opaque)
{
    uart_t *u = opaque;
    pic32_t *s = u->mcu;        /* used in VALUE() */

//printf("--- %s(%p) called\n", __func__, u);

    if (! (VALUE(u->mode) & PIC32_UMODE_ON) ||
        ! (VALUE(u->sta) & PIC32_USTA_URXEN)) {
        /* UART disabled. */
        return 0;
    }

    if (VALUE(u->sta) & PIC32_USTA_URXDA) {
        /* Receive buffer full. */
        return 0;
    }
    return 1;
}

/*
 * Process the received data.
 */
static void uart_receive(void *opaque, const uint8_t *buf, int size)
{
    uart_t *u = opaque;
    pic32_t *s = u->mcu;        /* used in VALUE() */

//printf("--- %s(%p) called\n", __func__, u);
    if (! (VALUE(u->mode) & PIC32_UMODE_ON) ||
        ! (VALUE(u->sta) & PIC32_USTA_URXEN)) {
        /* UART disabled. */
        return;
    }

    if (VALUE(u->sta) & PIC32_USTA_URXDA) {
        /* Receive buffer full. */
        return;
    }

    u->rxbyte = buf[0];
    VALUE(u->sta) |= PIC32_USTA_URXDA;

    /* Activate receive interrupt. */
    s->irq_raise (s, u->irq + UART_IRQ_RX);
}

/*
 * Activate the transmit interrupt.
 */
static void uart_timeout(void *opaque)
{
    uart_t *u = opaque;
    pic32_t *s = u->mcu;        /* used in VALUE() */

//printf("--- %s() called\n", __func__);
    if (u->oactive) {
        /* Activate transmit interrupt. */
//printf("uart%u: raise tx irq %u\n", unit, u->irq + UART_IRQ_TX);
        if ((VALUE(u->mode) & PIC32_UMODE_ON) &&
            (VALUE(u->sta) & PIC32_USTA_UTXEN))
            s->irq_raise (s, u->irq + UART_IRQ_TX);
        VALUE(u->sta) &= ~PIC32_USTA_UTXBF;
        u->oactive = 0;
    }
}

/*
 * Initialize the UART data structure.
 */
void pic32_uart_init(pic32_t *s, int unit, int irq, int sta, int mode)
{
    uart_t *u = &s->uart[unit];

    u->mcu = s;
    u->irq = irq;
    u->sta = sta;
    u->mode = mode;
    u->transmit_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_timeout, u);

    if (unit >= MAX_SERIAL_PORTS) {
        /* Cannot instantiate so many serial ports. */
        u->chr = 0;
        return;
    }
    u->chr = serial_hds[unit];

    /* Setup callback functions. */
    if (u->chr)
        qemu_chr_add_handlers(u->chr, uart_can_receive, uart_receive, NULL, u);
}
