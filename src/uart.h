#ifndef UART_H
#define UART_H
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_drv_common.h"
#include "nrf_drv_uart.h"


#define RX_PIN_NUMBER       15
#define TX_PIN_NUMBER       26

void uart_init(void);

#endif