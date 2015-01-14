/*
 * Define memory map for PIC32 microcontroller.
 *
 * Copyright (C) 2014 Serge Vakulenko, <serge@vak.ru>
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

#define IO_MEM_SIZE         (1024*1024)     /* 1 Mbyte */

typedef struct {
    SysBusDevice    parent_obj;
    SerialState     *uart;

    uint32_t        iomem [IO_MEM_SIZE/4];  /* backing storage for I/O area */
    int             stop_on_reset;          /* halt simulation on soft reset */
    unsigned        syskey_unlock;          /* syskey state */

    unsigned        sdcard_spi_port;        /* SPI port number of SD card */
    unsigned        sdcard_gpio_port0;      /* GPIO port number of CS0 signal */
    unsigned        sdcard_gpio_port1;      /* GPIO port number of CS1 signal */
    unsigned        sdcard_gpio_cs0;        /* GPIO pin mask of CS0 signal */
    unsigned        sdcard_gpio_cs1;        /* GPIO pin mask of CS1 signal */
} PIC32State;

static void sdcard_init (PIC32State *s, int unit, const char *name, const char *filename, int cs_port, int cs_pin) { }
static void sdcard_reset (PIC32State *s) { }
static void sdcard_select (PIC32State *s, int unit, int on) { }
//static unsigned sdcard_io (unsigned data) { return 0; }

static void uart_reset (void) { }
static unsigned uart_get_char (int unit) { return 0; }
static void uart_poll_status (int unit) { }
static void uart_put_char (int unit, unsigned data) { }
static void uart_update_mode (int unit) { }
static void uart_update_status (int unit) { }
//static void uart_poll (void) { }
//static int uart_active (void) { return 0; }

static void spi_reset (void) { }
static void spi_control (int unit) { }
static unsigned spi_readbuf (int unit) { return 0; }
static void spi_writebuf (int unit, unsigned val) { }

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
