/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2012 Karl Palsson <karlp@tweak.net.au>
 * Copyright (C) 2015 Kuldeep Singh Dhaka <kuldeepdhaka9@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/gpio.h>

// LEDs are on PF6 and PF7.

#define PORT_LED0 GPIOF
#define PIN_LED0  GPIO6
#define PORT_LED1 GPIOF
#define PIN_LED1  GPIO7

static void gpio_setup(void)
{
    /* Enable GPIOB clock. */
    /* Using API functions: */
    cmu_periph_clock_enable(CMU_GPIO);

    /* Set GPIO6 (in GPIO port A) to 'output push-pull'. */
    /* Using API functions: */
    gpio_mode_setup(PORT_LED0, GPIO_MODE_PUSH_PULL, PIN_LED0);
    gpio_mode_setup(PORT_LED1, GPIO_MODE_PUSH_PULL, PIN_LED1);
}

union {
    uint32_t u32;
    uint8_t u8[4];
} u;

int main(void)
{
    unsigned i;
    int j = 0;
    // u.u32 = 0x000000FF;
    u.u32 = 0xFF000000;

    gpio_setup();

    /* Toggle the LED pin */
    while (1) {
        switch (j++) {
        case 1:
            if (u.u8[0])
                j = 0;
        case 0:
            gpio_toggle(PORT_LED0, PIN_LED0);
            break;

            break;

        case 3:
            j = 0;
        case 2:
            gpio_toggle(PORT_LED1, PIN_LED1);
            break;
        }

        /* Wait a bit. */
        for (i = 0; i < 1000000; i++) {
            __asm__("nop");
        }
    }

    return 0;
}
