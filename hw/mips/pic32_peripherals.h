/*
 * Define memory map for PIC32 microcontroller.
 *
 * Copyright (C) 2015 Serge Vakulenko
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
#include "hw/sysbus.h"             /* SysBusDevice */

#define IO_MEM_SIZE     (1024*1024)     /* 1 Mbyte */

/* SD card private data */
struct sdcard {
    const char *name;                   /* Device name */
    unsigned kbytes;                    /* Disk size */
    int unit;                           /* Index (sd0 or sd1) */
    int fd;                             /* Image file */
    int select;                         /* Selected */
    int read_multiple;                  /* Read-multiple mode */
    unsigned blen;                      /* Block length */
    unsigned wbecnt;                    /* Write block erase count */
    unsigned offset;                    /* Read/write offset */
    unsigned count;                     /* Byte count */
    unsigned limit;                     /* Reply length */
    unsigned char buf [1024 + 16];
};
typedef struct sdcard sdcard_t;

/*
 * PIC32 data structure.
 */
typedef struct _pic32_t pic32_t;

struct _pic32_t {
    SysBusDevice    parent_obj;
    MIPSCPU         *cpu;                   /* back pointer to cpu object */
    //SerialState     *uart;

    int             stop_on_reset;          /* halt simulation on soft reset */
    unsigned        syskey_unlock;          /* syskey state */

#define NUM_UART    6                       /* number of UART ports */
    unsigned        uart_irq [NUM_UART];    /* interrupt numbers */
    int             uart_oactive[NUM_UART]; /* output active */
    int             uart_odelay [NUM_UART]; /* output delay count */
    unsigned        uart_sta [NUM_UART];    /* UxSTA address */
    unsigned        uart_mode [NUM_UART];   /* UxMODE address */

#define NUM_SPI     6                       /* max number of SPI ports */
    unsigned        spi_buf [NUM_SPI][4];   /* transmit and receive buffer */
    unsigned        spi_rfifo [NUM_SPI];    /* read fifo counter */
    unsigned        spi_wfifo [NUM_SPI];    /* write fifo counter */
    unsigned        spi_irq [NUM_SPI];      /* interrupt numbers */
    unsigned        spi_con [NUM_SPI];      /* SPIxCON address */
    unsigned        spi_stat [NUM_SPI];     /* SPIxSTAT address */

    unsigned        sdcard_spi_port;        /* SPI port number of SD card */
    unsigned        sdcard_gpio_port0;      /* GPIO port number of CS0 signal */
    unsigned        sdcard_gpio_port1;      /* GPIO port number of CS1 signal */
    unsigned        sdcard_gpio_cs0;        /* GPIO pin mask of CS0 signal */
    unsigned        sdcard_gpio_cs1;        /* GPIO pin mask of CS1 signal */
    sdcard_t        sdcard [2];             /* SD card data */

    uint32_t        iomem [IO_MEM_SIZE/4];  /* backing storage for I/O area */

    void (*irq_raise) (pic32_t *s, int irq); /* set interrupt request */
    void (*irq_clear) (pic32_t *s, int irq); /* clear interrupt request */
};

/*
 * UART routines.
 */
unsigned pic32_uart_get_char (pic32_t *s, int unit);
void pic32_uart_poll_status (pic32_t *s, int unit);
void pic32_uart_put_char (pic32_t *s, int unit, unsigned data);
void pic32_uart_update_mode (pic32_t *s, int unit);
void pic32_uart_update_status (pic32_t *s, int unit);
void pic32_uart_poll (pic32_t *s);
int pic32_uart_active (pic32_t *s);

/*
 * SPI routines.
 */
void pic32_spi_control (pic32_t *s, int unit);
unsigned pic32_spi_readbuf (pic32_t *s, int unit);
void pic32_spi_writebuf (pic32_t *s, int unit, unsigned val);

/*
 * SD card routines.
 */
void pic32_sdcard_init (pic32_t *s, int unit, const char *name,
    const char *filename, int cs_port, int cs_pin);
void pic32_sdcard_reset (pic32_t *s);
void pic32_sdcard_select (pic32_t *s, int unit, int on);
unsigned pic32_sdcard_io (pic32_t *s, unsigned data);

/*
 * Helper defines for i/o switch.
 */
#define VALUE(name)     s->iomem[(name & 0xfffff) >> 2]
#define STORAGE(name)   case name: *namep = #name;
#define READONLY(name)  case name: *namep = #name; goto readonly
#define WRITEOP(name)   case name: *namep = #name; goto op_##name;\
                        case name+4: *namep = #name"CLR"; goto op_##name;\
                        case name+8: *namep = #name"SET"; goto op_##name;\
                        case name+12: *namep = #name"INV"; op_##name: \
                        VALUE(name) = write_op (VALUE(name), data, offset)
#define WRITEOPX(name,label) \
                        case name: *namep = #name; goto op_##label;\
                        case name+4: *namep = #name"CLR"; goto op_##label;\
                        case name+8: *namep = #name"SET"; goto op_##label;\
                        case name+12: *namep = #name"INV"; goto op_##label
#define WRITEOPR(name,romask) \
                        case name: *namep = #name; goto op_##name;\
                        case name+4: *namep = #name"CLR"; goto op_##name;\
                        case name+8: *namep = #name"SET"; goto op_##name;\
                        case name+12: *namep = #name"INV"; op_##name: \
                        VALUE(name) &= romask; \
                        VALUE(name) |= write_op (VALUE(name), data, offset) & ~(romask)
