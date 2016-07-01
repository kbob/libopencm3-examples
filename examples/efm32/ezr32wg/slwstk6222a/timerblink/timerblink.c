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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/timer.h>

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

static volatile uint32_t millisecond_time;

static void gpio_setup(void)
{
    /* Enable GPIO clock. */
    /* Using API functions: */
    cmu_periph_clock_enable(CMU_GPIO);

    /* Set GPIOs 6 and 7 (in GPIO port F) to 'output push-pull'. */
    /* Using API functions: */
    gpio_mode_setup(PORT_LED0, GPIO_MODE_PUSH_PULL, PIN_LED0);
    gpio_mode_setup(PORT_LED1, GPIO_MODE_PUSH_PULL, PIN_LED1);
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
    TIMER_CCx_CCV(TIMER_LEDS, CC_LED0) = 0;

    timer_start(TIMER_LEDS);
}

static void systick_setup(int freq)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    STK_CVR = 0;
    systick_set_reload((14000000 / freq) - 1);
    systick_counter_enable();
    systick_interrupt_enable();
}

void sys_tick_handler(void)
{
    millisecond_time++;
    
    uint32_t t = millisecond_time / 3;
    uint32_t b0 = t & 0xFF, b1 = t & 0xFF;

    switch (t >> 8 & 3) {

    case 0:
        // LED 0 rising, LED 1 off
        b1 = 0;
        break;

    case 1:
        // LED 0 falling, LED 1 off
        b0 = 255 - b0; b1 = 0;
        break;

    case 2:
        // LED 0 off, LED 1 rising
        b0 = 0;
        break;

    case 3:
        // LED 0 off, LED 1 falling
        b0 = 0; b1 = 255 - b1;
        break;
    }

    TIMER_CCx_CCVB(TIMER_LEDS, CC_LED0) = b0 * (b0 + 1);
    TIMER_CCx_CCVB(TIMER_LEDS, CC_LED1) = b1 * (b1 + 1);
}

int main(void)
{
    systick_setup(1000);
    gpio_setup();
    timer_setup();

    while (true)
        continue;

    return 0;
}
