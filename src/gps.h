#ifndef GPS_H
#define GPS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ublox.h"
#define GPS_UPDATE_STAT()   ublox_update_stat()

#ifdef UART_LEGACY
#include "uart_legacy.h"
#endif

#ifdef UART_LIBUARTE
#include "uart_libuarte.h"
#endif


extern volatile TaskHandle_t xGpsParse;
extern volatile TaskHandle_t xGpsTask;

extern volatile uint8_t uart_br_found;

void gps_init(void);

#endif