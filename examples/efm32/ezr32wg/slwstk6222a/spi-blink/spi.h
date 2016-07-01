#ifndef SPI_included
#define SPI_included

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern void init_spi_master     (void);
extern void init_spi_slave      (void);

// Pass timeout = 0 to wait indefinitely.
extern bool spi_transfer_byte   (uint8_t        tx,
                                 uint8_t       *rxp,
                                 uint32_t       timeout_millis);

extern bool spi_transfer_buffer (uint8_t const *tx_buf,
                                 uint8_t       *rx_buf,
                                 size_t         count,
                                 uint32_t       timeout_millis);

#endif /* !SPI_included */
