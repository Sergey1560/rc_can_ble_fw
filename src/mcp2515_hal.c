#include "mcp2515_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "pins_config.h"

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

void spi_uninit(void){
    nrf_drv_spi_uninit(&spi);
	nrf_gpio_cfg_output(SPI_SS_PIN);
	nrf_gpio_pin_write(SPI_SS_PIN, 1);
    NRF_LOG_INFO("SPI stoped.");
}

void spi_init(void){
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    spi_config.frequency = SPI_FREQUENCY_FREQUENCY_M2;

    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
    NRF_LOG_INFO("SPI started.");
}

int8_t spi_transfer(uint8_t *tx_data, uint8_t *rx_data, uint32_t len){

	nrf_drv_spi_transfer(&spi, tx_data, len, rx_data, len);

	return 0;
}

void delay_ms(uint32_t ms){
	
	vTaskDelay(ms);
}

