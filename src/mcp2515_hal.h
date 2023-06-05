#ifndef MCP2515_HAL_H
#define MCP2515_HAL_H

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
//#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SPI_INSTANCE    0 /**< SPI instance index. */
#define SPI_SCK_PIN     5
#define SPI_MOSI_PIN    1
#define SPI_MISO_PIN    0
#define SPI_SS_PIN      13
#define MCP_INT_PIN     24

#define DEBUG(...)        NRF_LOG_INFO(__VA_ARGS__)

void spi_init(void);
void spi_uninit(void);

int8_t spi_transfer(uint8_t *tx_data, uint8_t *rx_data, uint32_t len);

void delay_ms(uint32_t ms);

#endif
