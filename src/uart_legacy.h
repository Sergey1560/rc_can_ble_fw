#ifdef UART_LEGACY
#ifndef UART_H
#define UART_H
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_drv_common.h"
#include "nrf_gpio.h"
#include "nrf_drv_uart.h"

#include "ublox.h"

#define USE_NATIVE_UART

void uart_config(uint32_t speed);
void uart_send_data(uint8_t *data, uint32_t len, uint8_t wait);
//void uart_init(void);

#endif

#endif