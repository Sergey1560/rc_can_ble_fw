#ifndef RC_DIY_LEDS
#define RC_DIY_LEDS

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "nordic_common.h"
#include "nrf_drv_common.h"


#define LEDS_ACTIVE_STATE  0
#define LEDS_INCTIVE_STATE 1


#define LED_CONNECTION     17
#define LED_CAN_DATA       18
#define LED_ALIVE          19

void rc_leds_init(void);

void rc_led_connection(uint8_t state);
void rc_led_candata(uint8_t state);

void rc_led_all(uint8_t state);

void rc_led_alive_invert(void);
void rc_led_candata_invert(void);

#endif