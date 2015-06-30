/*
 * GPIO ports.
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
#include "hw/hw.h"
#include "pic32_peripherals.h"

#include "pic32mz.h"

void pic32_gpio_write(pic32_t *s, int gpio_port, unsigned lat_value)
{
    /* Control SD card 0 */
    if (gpio_port == s->sdcard[0].gpio_port && s->sdcard[0].gpio_cs) {
        pic32_sdcard_select(s, 0, ! (lat_value & s->sdcard[0].gpio_cs));
    }
    /* Control SD card 1 */
    if (gpio_port == s->sdcard[1].gpio_port && s->sdcard[0].gpio_cs) {
        pic32_sdcard_select(s, 1, ! (lat_value & s->sdcard[1].gpio_cs));
    }
}
