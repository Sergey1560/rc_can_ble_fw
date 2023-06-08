#ifndef UART_H
#define UART_H
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_drv_common.h"
#include "nrf_gpio.h"
#include "nrf_drv_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ublox.h"

//#define USE_APP_UART
#define USE_NATIVE_UART

//#define UART_PIN 5

#define RX_PIN 15
#define TX_PIN 26

extern volatile TaskHandle_t xGpsParse;
extern volatile TaskHandle_t xGpsTask;

void uart_init(void);

#endif