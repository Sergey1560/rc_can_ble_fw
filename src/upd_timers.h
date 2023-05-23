#ifndef TIMERS_H
#define TIMERS_H

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_timer.h"

#include "ble_service.h"

void timers_init(void);
void timer_control(uint8_t state);

#endif