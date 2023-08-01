#ifndef GPS_H
#define GPS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ublox.h"
#ifdef UART_LEGACY
#include "uart_legacy.h"
#endif


extern volatile TaskHandle_t xGpsParse;
extern volatile TaskHandle_t xGpsTask;


void gps_init(void);

#endif