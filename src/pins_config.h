#ifndef PINS_CFG_H
#define PINS_CFG_H

#include "nrf.h"

/*
MCP2515 CAN SPI
*/
#define SPI_INSTANCE        0 /**< SPI instance index. */
#define SPI_SCK_PIN         NRF_GPIO_PIN_MAP(0,5)
#define SPI_MOSI_PIN        NRF_GPIO_PIN_MAP(0,1)
#define SPI_MISO_PIN        NRF_GPIO_PIN_MAP(0,0)
#define SPI_SS_PIN          NRF_GPIO_PIN_MAP(0,26)
#define MCP_INT_PIN         NRF_GPIO_PIN_MAP(1,9)
#define MCP_RST_PIN         NRF_GPIO_PIN_MAP(0,30)

/*
LED Pins
*/
#define LEDS_ACTIVE_STATE  1
#define LEDS_INCTIVE_STATE 0
#define LED_CONNECTION     NRF_GPIO_PIN_MAP(0,29)
#define LED_ALIVE          NRF_GPIO_PIN_MAP(1,11)


/*
UART Pins
*/
#define RX_PIN              NRF_GPIO_PIN_MAP(0,7)
#define TX_PIN              NRF_GPIO_PIN_MAP(0,15)


#endif