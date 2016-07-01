/*
 * This file is part of the libopencm3 project.
 *
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

#include <assert.h>
#include <stdio.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/timer.h>

#include "spi.h"
#include "systick.h"
#include "vcom.h"

// EXP pins 15 and 16 are on P201 pins 15, 16 are on radio PD7 and PD6.

#define PORT_SENS0     GPIOD
#define PIN_SENS0      GPIO7
#define PORT_SENS1     GPIOD
#define PIN_SENS1      GPIO6

// Button PB0 is PE3
// Button PB1 is PE2

#define PORT_BUT0      GPIOE
#define PIN_BUT0       GPIO3
#define PORT_BUT1      GPIOE
#define PIN_BUT1       GPIO2

// LEDs are on PF6 and PF7.  They are tied to TIMER0 CC0 and CC1 when
// TIMER0's location is set to 2.

#define PORT_LED0      GPIOF
#define PIN_LED0       GPIO6
#define PORT_LED1      GPIOF
#define PIN_LED1       GPIO7

#define TIMER_LEDS     TIMER0
#define CMU_TIMER_LEDS CMU_TIMER0
#define TIMER_LEDS_LOC TIMER_ROUTE_LOCATION_LOC2
#define CC_LED0        0
#define CC_LED1        1

#define SPI_INTERVAL_MSEC 1000

typedef enum SPI_role {
    SPI_ROLE_MASTER,
    SPI_ROLE_SLAVE,
} SPI_role;

static const char *SPI_role_name(SPI_role role)
{
    switch (role) {
        case SPI_ROLE_MASTER:
            return "master";
            break;


        case SPI_ROLE_SLAVE:
            return "slave";
            break;

        default:
            assert(false);
    }
}

static void gpio_setup(void)
{
    /* Enable GPIO clock. */
    /* Using API functions: */
    cmu_periph_clock_enable(CMU_GPIO);

    // Use LED pins as outputs
    gpio_mode_setup(PORT_LED0, GPIO_MODE_PUSH_PULL, PIN_LED0);
    gpio_mode_setup(PORT_LED1, GPIO_MODE_PUSH_PULL, PIN_LED1);

    // Use button pins as input.
    gpio_mode_setup(PORT_BUT0, GPIO_MODE_INPUT, PIN_BUT0);
    gpio_mode_setup(PORT_BUT1, GPIO_MODE_INPUT, PIN_BUT1);
}

static void timer_setup(void)
{
    // enable timer clock;
    cmu_periph_clock_enable(CMU_TIMER_LEDS);

    timer_stop(TIMER_LEDS);
    TIMER_ROUTE(TIMER_LEDS) = TIMER_ROUTE_LOCATION(TIMER_LEDS_LOC) |
                              TIMER_ROUTE_CCxPEN(CC_LED0)          |
                              TIMER_ROUTE_CCxPEN(CC_LED1);

    TIMER_CCx_CTRL(TIMER_LEDS, CC_LED0) =
        TIMER_CC_CTRL_COFOA(TIMER_CC_CTRL_COFOA_CLEAR) |
        TIMER_CC_CTRL_CMOA(TIMER_CC_CTRL_CMOA_SET)     |
        TIMER_CC_CTRL_MODE(TIMER_CC_CTRL_MODE_PWM);
    TIMER_CCx_CTRL(TIMER_LEDS, CC_LED1) =
        TIMER_CC_CTRL_COFOA(TIMER_CC_CTRL_COFOA_CLEAR) |
        TIMER_CC_CTRL_CMOA(TIMER_CC_CTRL_CMOA_SET)     |
        TIMER_CC_CTRL_MODE(TIMER_CC_CTRL_MODE_PWM);

    TIMER_CCx_CCV(TIMER_LEDS, CC_LED0) = 0;
    TIMER_CCx_CCV(TIMER_LEDS, CC_LED1) = 0;

    timer_start(TIMER_LEDS);
}

static const char *printable(uint8_t c)
{
    static char buf[8];

    if (c > ' ' && c < '\177')
        snprintf(buf, sizeof buf, "%c", c);
    else
        snprintf(buf, sizeof buf, "\\%03o", c);
    return buf;
}

2nnnnn nnnnn3
snprintf("\2%d %d\3", 10000, 10000)

int main(void)
{
    uint32_t next_time = SPI_INTERVAL_MSEC;

    init_systick();
    gpio_setup();
    timer_setup();
    VCOM_init();
    VCOM_init_stdio();

    // Hold down Button 0 during reset to be master.
    SPI_role my_role = gpio_get(PORT_BUT0, PIN_BUT0)
                       ? SPI_ROLE_SLAVE
                       : SPI_ROLE_MASTER;

    if (my_role == SPI_ROLE_MASTER) {
        init_spi_master();
        gpio_set(PORT_LED0, PIN_LED0);
        gpio_clear(PORT_LED1, PIN_LED1);
        TIMER_CCx_CCV(TIMER_LEDS, CC_LED0) = 65535;
    } else
        init_spi_slave();

    printf("\n\nI am %s.\n\n", SPI_role_name(my_role));
    uint8_t counter = 0;
    while (true) {
        if (my_role == SPI_ROLE_MASTER) {
            while ((int32_t)(millisecond_time - next_time) < 0)
                continue;
            next_time += SPI_INTERVAL_MSEC;
        }
        uint8_t tx_byte = 'A' + (counter = (counter + 1) & 0x3);
        uint8_t rx_byte;
        printf("%s: sending %c\n", SPI_role_name(my_role), tx_byte);
        bool ret = spi_transfer_byte(tx_byte, &rx_byte, 1000);
        if (ret)
            printf("%s: received %s\n",
                   SPI_role_name(my_role), printable(rx_byte));
        else
            printf("%s: timed out\n", SPI_role_name(my_role));
    }
    return 0;
}
