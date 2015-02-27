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
#include "hw/sysbus.h"                  /* SysBusDevice */
#include "net/net.h"

#define IO_MEM_SIZE     (1024*1024)     /* 1 Mbyte */

typedef struct _uart_t uart_t;
typedef struct _spi_t spi_t;
typedef struct _sdcard_t sdcard_t;
typedef struct _pic32_t pic32_t;
typedef struct _eth_t eth_t;

/*
 * UART private data.
 */
struct _uart_t {
    pic32_t     *mcu;                   /* back pointer to pic32 object */
    unsigned    irq;                    /* interrupt number */
    int         oactive;                /* output active */
    unsigned    sta;                    /* UxSTA address */
    unsigned    mode;                   /* UxMODE address */
    unsigned    rxbyte;                 /* received byte */
    CharDriverState *chr;               /* pointer to serial_hds[i] */
    QEMUTimer   *transmit_timer;        /* needed to delay TX interrupt */
};

/*
 * SPI private data.
 */
struct _spi_t {
    unsigned    buf[4];                 /* transmit and receive buffer */
    unsigned    rfifo;                  /* read fifo counter */
    unsigned    wfifo;                  /* write fifo counter */
    unsigned    irq;                    /* interrupt numbers */
    unsigned    con;                    /* SPIxCON address */
    unsigned    stat;                   /* SPIxSTAT address */
};

/*
 * SD card private data.
 */
struct _sdcard_t {
    const char  *name;                  /* Device name */
    unsigned    gpio_port;              /* GPIO port number of CS0 signal */
    unsigned    gpio_cs;                /* GPIO pin mask of CS0 signal */
    unsigned    kbytes;                 /* Disk size */
    int         unit;                   /* Index (sd0 or sd1) */
    int         fd;                     /* Image file */
    int         select;                 /* Selected */
    int         read_multiple;          /* Read-multiple mode */
    unsigned    blen;                   /* Block length */
    unsigned    wbecnt;                 /* Write block erase count */
    unsigned    offset;                 /* Read/write offset */
    unsigned    count;                  /* Byte count */
    unsigned    limit;                  /* Reply length */
    unsigned    char buf [1024 + 16];
};

/*
 * PIC32 data structure.
 */
struct _pic32_t {
    SysBusDevice parent_obj;
    MIPSCPU     *cpu;                   /* back pointer to cpu object */
    uint32_t    *iomem;                 /* backing storage for I/O area */

    int         board_type;             /* board variant */
    int         stop_on_reset;          /* halt simulation on soft reset */
    unsigned    syskey_unlock;          /* syskey state */

#define NUM_UART 6                      /* number of UART ports */
    uart_t      uart [NUM_UART];        /* UART data */

#define NUM_SPI 6                       /* max number of SPI ports */
    spi_t       spi [NUM_SPI];          /* SPI data */

    unsigned    sdcard_spi_port;        /* SPI port number of SD card */
    sdcard_t    sdcard [2];             /* SD card data */

    DeviceState *eth_dev;               /* Ethernet device */
    eth_t       *eth;                   /* Ethernet driver data */

    void (*irq_raise)(pic32_t *s, int irq); /* set interrupt request */
    void (*irq_clear)(pic32_t *s, int irq); /* clear interrupt request */
};

/*
 * GPIO routines.
 */
void pic32_gpio_write(pic32_t *s, int unit, unsigned val);

/*
 * UART routines.
 */
void pic32_uart_init(pic32_t *s, int unit, int irq, int sta, int mode);
unsigned pic32_uart_get_char(pic32_t *s, int unit);
void pic32_uart_put_char(pic32_t *s, int unit, unsigned char data);
void pic32_uart_poll_status(pic32_t *s, int unit);
void pic32_uart_update_mode(pic32_t *s, int unit);
void pic32_uart_update_status(pic32_t *s, int unit);

/*
 * SPI routines.
 */
void pic32_spi_init(pic32_t *s, int unit, int irq, int con, int stat);
void pic32_spi_control(pic32_t *s, int unit);
unsigned pic32_spi_readbuf(pic32_t *s, int unit);
void pic32_spi_writebuf(pic32_t *s, int unit, unsigned val);

/*
 * SD card routines.
 */
void pic32_sdcard_init(pic32_t *s, int unit, const char *name,
    const char *filename, int cs_port, int cs_pin);
void pic32_sdcard_reset(pic32_t *s);
void pic32_sdcard_select(pic32_t *s, int unit, int on);
unsigned pic32_sdcard_io(pic32_t *s, unsigned data);

/*
 * Ethernet routines.
 */
void pic32_eth_init(pic32_t *s, NICInfo *nd);
void pic32_eth_control(pic32_t *s);
void pic32_mii_command(pic32_t *s);
void pic32_mii_write(pic32_t *s);

/*
 * Load a binary file in hex or srec format.
 */
int pic32_load_hex_file(const char *filename,
    void (*store_byte)(unsigned address, unsigned char byte));

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
                        VALUE(name) = write_op(VALUE(name), data, offset)
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
                        VALUE(name) |= write_op(VALUE(name), data, offset) & ~(romask)
