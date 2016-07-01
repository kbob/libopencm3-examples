/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2013 Chuck McManis <cmcmanis@mcmanis.com>
 * Copyright (C) 2013 Onno Kortmann <onno@gmx.net>
 * Copyright (C) 2013 Frantisek Burian <BuFran@seznam.cz> (merge)
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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

// LEDs are on PF6 and PF7.

#define PORT_LED0 GPIOF
#define PIN_LED0  GPIO6
#define PORT_LED1 GPIOF
#define PIN_LED1  GPIO7

/* Called when systick fires */
void sys_tick_handler(void)
{
	static volatile uint32_t millis;
	if (++millis % 1000)
		return;
	gpio_toggle(PORT_LED0, PIN_LED0); /* LED on/off */
	gpio_toggle(PORT_LED1, PIN_LED1); /* LED off/on */
}

/* Set up timer to fire freq times per second */
static void systick_setup(int freq)
{
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);

	/* clear counter so it starts right away */
	STK_CVR = 0;

	/* systick is running from cpu clock. (so 48Mhz) */
	systick_set_reload((48000000 / freq / 4) - 1);
	systick_counter_enable();
	systick_interrupt_enable();
}

/* set EFM32 to clock by 48MHz from HFXO */
static void clock_setup(void)
{
	// Enable HFPER clock.
	CMU_HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN;

	/* Enable clocks to the GPIO subsystems */
	cmu_periph_clock_enable(CMU_GPIO);

}

static void gpio_setup(void)
{
	/* Set GPIO{2,3} (in GPIO port E) to 'output push-pull'. */
	gpio_mode_setup(PORT_LED0, GPIO_MODE_PUSH_PULL, PIN_LED0);
	gpio_mode_setup(PORT_LED1, GPIO_MODE_PUSH_PULL, PIN_LED1);

	/* Preconfigure the LEDs. */
	gpio_set(PORT_LED0, PIN_LED0);	/* Switch off LED. */
	gpio_clear(PORT_LED1, PIN_LED1); /* Switch on LED. */
}

int main(void)
{
	clock_setup();
	gpio_setup();

	systick_setup(1000);

	/* Do nothing in main loop */
	while (1);
}
