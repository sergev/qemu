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

#define OUTPUT_DELAY    3

//TODO
static int vtty_get_char (unsigned unit) {return 0;}
static void vtty_put_char (unsigned unit, char ch) { putchar(ch); fflush(stdout); }
static int vtty_is_char_avail (unsigned unit) {return 0;}

/*
 * Read of UxRXREG register.
 */
unsigned pic32_uart_get_char (pic32_t *s, int unit)
{
    uart_t *u = &s->uart[unit];
    unsigned value;

    // Read a byte from input queue
    value = vtty_get_char (unit);
    VALUE(u->sta) &= ~PIC32_USTA_URXDA;

    if (vtty_is_char_avail (unit)) {
        // One more byte available
        VALUE(u->sta) |= PIC32_USTA_URXDA;
    } else {
        s->irq_clear (s, u->irq + UART_IRQ_RX);
    }
    return value;
}

/*
 * Called before reading a value of UxBRG, UxMODE or UxSTA registers.
 */
void pic32_uart_poll_status (pic32_t *s, int unit)
{
    uart_t *u = &s->uart[unit];

    // Keep receiver idle, transmit shift register always empty
    VALUE(u->sta) |= PIC32_USTA_RIDLE | PIC32_USTA_TRMT;

    if (vtty_is_char_avail (unit)) {
        // Receive data available
        VALUE(u->sta) |= PIC32_USTA_URXDA;
    }
    //printf ("<%x>", VALUE(u->sta)); fflush (stdout);
}

/*
 * Write to UxTXREG register.
 */
void pic32_uart_put_char (pic32_t *s, int unit, unsigned data)
{
    uart_t *u = &s->uart[unit];

    vtty_put_char (unit, data);
    if ((VALUE(u->mode) & PIC32_UMODE_ON) &&
        (VALUE(u->sta) & PIC32_USTA_UTXEN) &&
        ! u->oactive)
    {
        u->oactive = 1;
        u->odelay = 0;
        VALUE(u->sta) |= PIC32_USTA_UTXBF;
    }
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

void pic32_uart_poll(pic32_t *s)
{
    int unit;

    for (unit=0; unit<NUM_UART; unit++) {
        uart_t *u = &s->uart[unit];

        if (! (VALUE(u->mode) & PIC32_UMODE_ON)) {
            /* UART disabled. */
            u->oactive = 0;
            VALUE(u->sta) &= ~PIC32_USTA_UTXBF;
            continue;
        }

        /* UART enabled. */
        if ((VALUE(u->sta) & PIC32_USTA_URXEN) && vtty_is_char_avail (unit)) {
            /* Receive data available. */
            VALUE(u->sta) |= PIC32_USTA_URXDA;

            /* Activate receive interrupt. */
            s->irq_raise (s, u->irq + UART_IRQ_RX);
            continue;
        }

        if (! (VALUE(u->sta) & PIC32_USTA_UTXEN)) {
            /* Transmitter disabled. */
            u->oactive = 0;
            continue;
        }
        if (u->oactive) {
            /* Activate transmit interrupt. */
            if (++u->odelay > OUTPUT_DELAY) {
//printf("uart%u: raise tx irq %u\n", unit, u->irq + UART_IRQ_TX);
                s->irq_raise (s, u->irq + UART_IRQ_TX);
                VALUE(u->sta) &= ~PIC32_USTA_UTXBF;
                u->oactive = 0;
            }
        }
    }
}

/*
 * Return true when any I/O is active.
 * Check uart output and pending input.
 */
int pic32_uart_active(pic32_t *s)
{
    int unit;

    for (unit=0; unit<NUM_UART; unit++) {
        uart_t *u = &s->uart[unit];

        if (u->oactive)
            return 1;
        if (vtty_is_char_avail (unit))
            return 1;
    }
    return 0;
}

static int uart_can_receive(void *opaque)
{
    uart_t *u = opaque;

    //TODO:
    printf("--- %s(%p) called\n", __func__, u);
    return 1;
}

static void uart_receive(void *opaque, const uint8_t *buf, int size)
{
    uart_t *u = opaque;

    //TODO:
    printf("--- %s(%p) called\n", __func__, u);
}

static void uart_timeout(void *opaque)
{
    pic32_t *s = opaque;

    //TODO:
    printf("--- %s(%p) called\n", __func__, s);
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

    if (unit >= MAX_SERIAL_PORTS) {
        /* Cannot instantiate so many serial ports. */
        u->chr = 0;
        return;
    }
    if (! serial_hds[unit]) {
        char buf [16];
        sprintf(buf, "serial%d", unit);
        serial_hds[unit] = qemu_chr_new(buf, "null", NULL);
    }
    u->chr = serial_hds[unit];

    /* Setup callback functions. */
    qemu_chr_add_handlers(u->chr, uart_can_receive, uart_receive, NULL, u);

    /* Common timeout timer for all UARTs. */
    if (! s->uart_timer)
        s->uart_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_timeout, s);
}
