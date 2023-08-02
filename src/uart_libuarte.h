#ifdef UART_LIBUARTE

#ifndef UART_LIBUARTE_H
#define UART_LIBUARTE_H

#include "nrf.h"
#include "nrf_drv_common.h"
#include "nrf_gpio.h"
#include "nrf_libuarte_async.h"

#include "ublox.h"

//void uart_start(void);
void uart_init(nrf_uarte_baudrate_t speed, uint32_t timeout);
ret_code_t uart_send_data(uint8_t *data, uint32_t len, uint8_t wait);

static const uint32_t uart_speed_list[]={NRF_UARTE_BAUDRATE_9600,NRF_UARTE_BAUDRATE_19200,NRF_UARTE_BAUDRATE_38400,NRF_UARTE_BAUDRATE_57600,NRF_UARTE_BAUDRATE_115200};
static const uint32_t uart_speed_val[]={9600,19200,38400,57600,115200};

#endif

#endif