#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/gpio.h>

#include "vcom.h"

// Button PB0 is PE3
// Button PB1 is PE2

#define STABLE_COUNT 10000
#define HIST_MAX 1000

typedef uint32_t histogram[HIST_MAX];

static histogram b0_up_hist, b0_dn_hist, b1_up_hist, b1_dn_hist;

static void buttons_init(void)
{
    cmu_periph_clock_enable(CMU_GPIO);
    gpio_mode_setup(GPIOE, GPIO_MODE_INPUT, GPIO3 | GPIO2);
}

static inline bool read_button_0(void)
{
    return (bool)BBIO_PERIPH(&GPIO_PE_DIN, 3);
}

static inline bool read_button_1(void)
{
    return (bool)BBIO_PERIPH(&GPIO_PE_DIN, 2);
}

static inline void count(uint32_t counter, histogram *histp)
{
    if (counter < HIST_MAX)
        (*histp)[counter]++;
    else
        (*histp)[0]++;
}

static void print_and_clear(const char *button_name,
                            bool        is_up,
                            histogram   dn_hist,
                            histogram   up_hist)
{
    size_t i;

    printf("%s %-4s\n", button_name, is_up ? "UP" : "DOWN");
    for (i = 1; i < HIST_MAX; i++) {
        if (dn_hist[i] || up_hist[i]) {
            printf("%3zu: %3lu %3lu\n", i, dn_hist[i], up_hist[i]);
            dn_hist[i] = up_hist[i] = 0;
        }
    }
    printf("\n");
}

int main(void)
{
    VCOM_init();
    VCOM_init_stdio();

    buttons_init();
    
    printf("\n\n");
    printf("Hello, Buttons and LEDs!\n\n");

    uint16_t buttons_were_up = GPIO3 | GPIO2;
    uint32_t b0_counter = 0, b1_counter = 0;
    bool b0_stable = true, b1_stable = true;
    while (1) {
        uint16_t buttons_are_up = gpio_get(GPIOE, GPIO3 | GPIO2);
        bool b0_was_up = (bool)(buttons_were_up & GPIO3);
        bool b1_was_up = (bool)(buttons_were_up & GPIO2);
        bool b0_is_up  = (bool)(buttons_are_up  & GPIO3);
        bool b1_is_up  = (bool)(buttons_are_up  & GPIO2);
        if (b0_is_up == b0_was_up) {
            b0_counter++;
            if (b0_counter == STABLE_COUNT && !b0_stable) {
                b0_stable = true;
                print_and_clear("B0", b0_is_up, b0_dn_hist, b0_up_hist);
            }
        } else {
            if (b0_was_up)
                count(b0_counter, &b0_up_hist);
            else
                count(b0_counter, &b0_dn_hist);
            b0_counter = 1;
            b0_stable = false;
        }
        if (b1_is_up == b1_was_up) {
            b1_counter++;
            if (b1_counter == STABLE_COUNT && !b1_stable) {
                b1_stable = true;
                print_and_clear("B1", b1_is_up, b1_dn_hist, b1_up_hist);
            }
        } else {
            if (b1_was_up)
                count(b1_counter, &b1_up_hist);
            else
                count(b1_counter, &b1_dn_hist);
            b1_counter = 1;
            b1_stable = false;
        }
        buttons_were_up = buttons_are_up;
    }
}
