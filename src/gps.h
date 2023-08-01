#ifndef GPS_H
#define GPS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#include "uart.h"

extern volatile TaskHandle_t xGpsParse;
extern volatile TaskHandle_t xGpsTask;


void gps_init(void);

#endif