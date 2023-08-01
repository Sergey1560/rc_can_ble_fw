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

void uart_init(uint32_t speed, uint32_t timeout);
void uart_send_data(uint8_t *data, uint32_t len, uint8_t wait);
//void uart_init(void);


static const uint32_t uart_speed_list[]={UART_BAUDRATE_BAUDRATE_Baud9600,UART_BAUDRATE_BAUDRATE_Baud19200,UART_BAUDRATE_BAUDRATE_Baud38400,UART_BAUDRATE_BAUDRATE_Baud57600};
static const uint32_t uart_speed_val[]={9600,19200,38400,57600};


#endif

#endif