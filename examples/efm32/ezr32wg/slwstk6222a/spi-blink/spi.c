#include "spi.h"

#include <assert.h>

#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/usart.h>

#include "systick.h"

// SPI pins
//   MOSI  PD0
//   MISO  PD1
//   CLK   PD2
//   CS    PD3
// Thats USART1 route location 1.

#define MASTER_BAUD 1000000

// Calculate the USART clock divider.  The USART clock is derived from
// HFPERCLK, which is derived from HFCLK.  HFCLK comes from one of
// four sources.  This code only works when HFCLK comes from the
// default source, HFRCO.

static uint32_t calculate_sync_clock_divider(uint32_t baud)
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
    assert(baud <= hfperclk_freq / 2);

    // calculate USART clock divider.  Use 64 bit math.
    uint32_t divisor = 2 * baud - 1;
    uint32_t usart_clkdiv = 256ULL * hfperclk_freq / divisor - 256;

    return usart_clkdiv;
    
}

static void init_spi(bool is_master, uint32_t freq)
{
    // enable HFPER clock
    CMU_HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN;

    // enable GPIO clock
    cmu_periph_clock_enable(CMU_GPIO);

    if (is_master) {
        // master: set PD0 (USART1 MOSI) to push-pull, initially low.
        gpio_mode_setup(GPIOD, GPIO_MODE_PUSH_PULL, GPIO0);

        // set PD1 (USART1 MISO) to input.
        gpio_mode_setup(GPIOD, GPIO_MODE_INPUT_PULL, GPIO1);

        // set PD2 (CLK) to push-pull, initially low.
        gpio_mode_setup(GPIOD, GPIO_MODE_PUSH_PULL, GPIO2);

        // master: set PD3 (CS) to push-pull, initially high.
        gpio_set(GPIOD, GPIO3);
        gpio_mode_setup(GPIOD, GPIO_MODE_PUSH_PULL, GPIO3);
    } else {
        // slave: set PD0 (USART1 MOSI) to input.
        gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO0);

        // slave: set PD1 (USART1 MISO) to push-pull, initially low.
        gpio_mode_setup(GPIOD, GPIO_MODE_PUSH_PULL, GPIO1);

        // slave: set PD2 (CLK)  to input.
        gpio_mode_setup(GPIOD, GPIO_MODE_INPUT_PULL, GPIO2);
        
        // slave: set PD3 (CS) to input, pull-up.
        gpio_set(GPIOD, GPIO3);
        gpio_mode_setup(GPIOD, GPIO_MODE_INPUT_PULL, GPIO3);
    }

    // enable USART clock
    cmu_periph_clock_enable(CMU_USART1);

    // calculate USART clock divider.
    uint32_t usart_clkdiv =
        calculate_sync_clock_divider(freq);


    // reset USART
    USART1_CMD      = USART_CMD_RXDIS      |
                      USART_CMD_TXDIS      |
                      USART_CMD_MASTERDIS  |
                      USART_CMD_RXBLOCKDIS |
                      USART_CMD_TXTRIDIS   |
                      USART_CMD_CLEARTX;
    USART1_TRIGCTRL = 0;
    USART1_IEN      = 0;
    USART1_IFC      = USART_IFC_CCF    |
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
    USART1_INPUT    = 0;
    USART1_I2SCTRL  = 0;

    // configure USART
    USART1_CTRL     = USART_CTRL_AUTOCS | USART_CTRL_MSBF | USART_CTRL_SYNC;
    USART1_FRAME    = USART_FRAME_DATABITS(USART_FRAME_DATABITS_EIGHT);
    USART1_CLKDIV   = usart_clkdiv;
    USART1_ROUTE    = USART_ROUTE_LOCATION(USART_ROUTE_LOCATION_LOC1) |
                      USART_ROUTE_CLKPEN                              |
                      USART_ROUTE_CSPEN                               |
                      USART_ROUTE_TXPEN                               |
                      USART_ROUTE_RXPEN;

    // // enable interrupt
    // USART1_IFC = USART_IFC_RXDATAV;
    // nvic_clear_pending_irq(NVIC_USART1_RX_IRQ);
    // nvic_enable_irq(NVIC_USART1_RX_IRQ);
    // USART1_IEN = USART_IEN_RXDATAV;

    // enable USART
    uint32_t cmd = USART_CMD_RXEN | USART_CMD_TXEN;
    if (is_master)
        cmd |= USART_CMD_MASTEREN;
    USART1_CMD = cmd;
}

void init_spi_master()
{
    init_spi(true, MASTER_BAUD);
}

void init_spi_slave(void)
{
    init_spi(false, 4 * MASTER_BAUD);
}

bool spi_transfer_byte(uint8_t tx, uint8_t *rxp, uint32_t timeout_millis)
{
    uint32_t t0 = millisecond_time;
    while (!(USART1_STATUS & USART_STATUS_TXBL))
        if (!timeout_millis || millisecond_time - t0 > timeout_millis)
            return false;

    USART1_TXDATA = tx;

    while (!(USART1_STATUS & USART_STATUS_RXDATAV))
        if (!timeout_millis || millisecond_time - t0 > timeout_millis)
            return false;

    uint8_t rx = USART1_RXDATA;
    if (rxp)
        *rxp = rx;
    return true;                // success
}
