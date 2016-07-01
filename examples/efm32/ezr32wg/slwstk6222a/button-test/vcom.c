#include "vcom.h"

#include <assert.h>

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/usart.h>

// 115200 baud, 16x oversampling, 8 data bits, no parity, 1 stop bit

#define VCOM_BAUD          115200
#define VCOM_OVS           16
#define VCOM_OVS_bits      USART_CTRL_OVS_X16
#define VCOM_DATABITS_bits USART_FRAME_DATABITS_EIGHT
#define VCOM_STOPBITS_bits USART_FRAME_STOPBITS_ONE

#define RX_BUF_SIZE        16

static uint8_t rx_buf[RX_BUF_SIZE];
static uint8_t *volatile rx_head = rx_buf;
static uint8_t *volatile rx_tail = rx_buf;

// Calculate the USART clock divider.  The USART clock is derived from
// HFPERCLK, which is derived from HFCLK.  HFCLK comes from one of
// four sources.  This code only works when HFCLK comes from the
// default source, HFRCO.

static uint32_t calculate_USART_clock_divider(uint32_t baud, uint8_t ovs)
{
    // check HFCLK source.
    assert(CMU_STATUS & CMU_STATUS_HFRCOSEL);
    // N.B., the USART is not initialized.  If this assertion fails,
    // it can't print anything.

    // calculate HFCLK frequency.
    uint32_t hfclk_band_mask = (CMU_HFRCOCTRL & CMU_HFRCOCTRL_BAND_MASK) >>
                               CMU_HFRCOCTRL_BAND_SHIFT;
    uint32_t hfclkdiv = (CMU_CTRL & CMU_CTRL_HFCLKDIV_MASK) >>
                        CMU_CTRL_HFCLKDIV_SHIFT;
    uint32_t hfclk_base_freq;
    switch (hfclk_band_mask) {
    case CMU_HFRCOCTRL_BAND_1MHZ:
        hfclk_base_freq = 1000000;
        break;

    case CMU_HFRCOCTRL_BAND_7MHZ:
        hfclk_base_freq = 7000000;
        break;

    case CMU_HFRCOCTRL_BAND_11MHZ:
        hfclk_base_freq = 11000000;
        break;

    case CMU_HFRCOCTRL_BAND_14MHZ:
        hfclk_base_freq = 14000000;
        break;

    case CMU_HFRCOCTRL_BAND_21MHZ:
        hfclk_base_freq = 21000000;
        break;

    case CMU_HFRCOCTRL_BAND_28MHZ:
        hfclk_base_freq = 28000000;
        break;

    default:
        assert(false);
    }
    uint32_t hfclk_freq = hfclk_base_freq / (1 + hfclkdiv);

    // calculate HFPERCLK frequency
    uint32_t hfperclkdiv = (CMU_HFPERCLKDIV & CMU_HFPERCLKDIV_HFPERCLKDIV_MASK)
                           >> CMU_HFPERCLKDIV_HFPERCLKDIV_SHIFT;
    uint32_t hfperclk_freq = hfclk_freq >> hfperclkdiv;

    // check that baud is in range
    assert(baud <= hfperclk_freq / ovs);

    // calculate USART clock divider.  Use 64 bit math.
    uint32_t divisor = ovs * baud;
    uint32_t usart_clkdiv = (256ULL * hfperclk_freq) / divisor - 256;

    return usart_clkdiv;
}

void VCOM_init(void)
{
    // enable HFPER clock
    CMU_HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN;

    // enable GPIO clock
    cmu_periph_clock_enable(CMU_GPIO);

    // set PB3 (USART2 TX) to push-pull, initially high.
    gpio_set(GPIOB, GPIO3);
    gpio_mode_setup(GPIOB, GPIO_MODE_PUSH_PULL, GPIO3);

    // set PB4 (USART2 RX) to input, no filter.
    gpio_clear(GPIOB, GPIO4);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO4);

    // set PA12 (VCOM Enable) high.
    gpio_set(GPIOA, GPIO12);
    gpio_mode_setup(GPIOA, GPIO_MODE_PUSH_PULL, GPIO12);

    // enable USART clock
    cmu_periph_clock_enable(CMU_USART2);

    // calculate USART clock divider.
    uint32_t usart_clkdiv = calculate_USART_clock_divider(VCOM_BAUD, VCOM_OVS);

    // reset USART
    USART2_CMD      = USART_CMD_RXDIS      |
                      USART_CMD_TXDIS      |
                      USART_CMD_MASTERDIS  |
                      USART_CMD_RXBLOCKDIS |
                      USART_CMD_TXTRIDIS   |
                      USART_CMD_CLEARTX    |
                      USART_CMD_CLEARRX;
    USART2_TRIGCTRL = 0;
    USART2_IEN      = 0;
    USART2_IFC      = USART_IFC_CCF    |
                      USART_IFC_SSM    |
                      USART_IFC_MPAF   |
                      USART_IFC_FERR   |
                      USART_IFC_PERR   |
                      USART_IFC_TXUF   |
                      USART_IFC_TXOF   |
                      USART_IFC_RXUF   |
                      USART_IFC_RXOF   |
                      USART_IFC_RXFULL |
                      USART_IFC_TXC;
    USART2_INPUT    = 0;
    USART2_I2SCTRL  = 0;

    // configure USART
    USART2_FRAME    = USART_FRAME_STOPBITS(VCOM_STOPBITS_bits) |
                      USART_FRAME_DATABITS(VCOM_DATABITS_bits);
    USART2_CTRL     = USART_CTRL_OVS(VCOM_OVS_bits);
    USART2_CLKDIV   = usart_clkdiv;
    USART2_ROUTE    = USART_ROUTE_LOCATION(USART_ROUTE_LOCATION_LOC1) |
                      USART_ROUTE_TXPEN                               |
                      USART_ROUTE_RXPEN;

    // enable interrupt
    USART2_IFC = USART_IFC_RXDATAV;
    nvic_clear_pending_irq(NVIC_USART2_RX_IRQ);
    nvic_enable_irq(NVIC_USART2_RX_IRQ);
    USART2_IEN = USART_IEN_RXDATAV;

    // enable USART
    USART2_CMD = USART_CMD_RXEN | USART_CMD_TXEN;
}

void usart2_rx_isr(void)
{
    if (USART2_STATUS & USART_STATUS_RXDATAV) {
        uint8_t c = USART2_RXDATA;
        uint8_t *next_tail = rx_tail + 1;
        if (next_tail == rx_buf + RX_BUF_SIZE)
            next_tail = rx_buf;
        if (next_tail != rx_head) {
            *rx_tail = c;
            rx_tail = next_tail;
        }            
    }
}

void VCOM_print_string(const char *p)
{
    while (*p)
        VCOM_print_char(*p++);
}

void VCOM_print_char(char c)
{
    if (c == '\n')
        VCOM_print_char('\r');
    while (!(USART2_STATUS & USART_STATUS_TXBL))
        continue;
    USART2_TXDATA = (uint32_t)(c & 0xFF);
}

int VCOM_read_char(void)
{
    bool interrupts_masked = cm_is_masked_interrupts();
    cm_disable_interrupts();
    int c = -1;
    uint8_t *head = rx_head;
    if (head != rx_tail) {
        c = *head++;
        if (head > rx_buf + RX_BUF_SIZE)
            head = rx_buf;
        rx_head = head;
    }
    if (interrupts_masked)
        cm_enable_interrupts();
    return c;
}

#include <stdio.h>

static ssize_t VCOM_readfn(void *cookie, char *buf, size_t size)
{
    cookie = cookie;
    size_t i;

    for (i = 0; i < size; i++) {
        int c = VCOM_read_char();
        if (c == -1)
            return -1;
        buf[i++] = c;
        break;
        // if (c == '\r' || c == '\n')
        //     break;
    }
    return i;
}

static ssize_t VCOM_writefn(void *cookie, const char *buf, size_t size)
{
    cookie = cookie;
    size_t i;

    for (i = 0; i < size; i++)
        VCOM_print_char(buf[i]);
    return (ssize_t)i;
}

void VCOM_init_stdio(void)
{
    cookie_io_functions_t input_fns = {
        .read  = VCOM_readfn,
        .write = NULL,
        .seek  = NULL,
        .close = NULL,
    };
    cookie_io_functions_t output_fns = {
        .read  = NULL,
        .write = VCOM_writefn,
        .seek  = NULL,
        .close = NULL,
    };

    stdin  = fopencookie(NULL, "r", input_fns);
    stdout = fopencookie(NULL, "w", output_fns);
    stderr = fopencookie(NULL, "w", output_fns);
    setlinebuf(stdout);
    setbuf(stderr, NULL);
}
