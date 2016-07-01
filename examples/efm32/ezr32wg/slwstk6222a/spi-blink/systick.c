#include "systick.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

volatile uint32_t millisecond_time;

void sys_tick_handler(void)
{
    millisecond_time++;
}

void init_systick(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    STK_CVR = 0;
    systick_set_reload((14000000 / 1000) - 1);
    systick_counter_enable();
    systick_interrupt_enable();
}
