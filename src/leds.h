#ifndef RC_DIY_LEDS
#define RC_DIY_LEDS

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "nordic_common.h"
#include "nrf_drv_common.h"

void rc_leds_init(void);

void rc_led_connection(uint8_t state);
void rc_led_alive(uint8_t state);

void rc_led_all(uint8_t state);
void rc_led_alive_invert(void);

#endif