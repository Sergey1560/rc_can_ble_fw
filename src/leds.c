#include "leds.h"
#include "pins_config.h"

void rc_leds_init(void){
    nrf_gpio_cfg_output(LED_CONNECTION);
    nrf_gpio_cfg_output(LED_ALIVE);
}

void rc_led_connection(uint8_t state){
    nrf_gpio_pin_write(LED_CONNECTION, state ? LEDS_ACTIVE_STATE : LEDS_INCTIVE_STATE);
}

void rc_led_alive(uint8_t state){
    nrf_gpio_pin_write(LED_ALIVE, state ? LEDS_ACTIVE_STATE : LEDS_INCTIVE_STATE);
}

void rc_led_all(uint8_t state){
    nrf_gpio_pin_write(LED_CONNECTION, state ? LEDS_ACTIVE_STATE : LEDS_INCTIVE_STATE);
    nrf_gpio_pin_write(LED_ALIVE, state ? LEDS_ACTIVE_STATE : LEDS_INCTIVE_STATE);
}

void rc_led_alive_invert(void){
    nrf_gpio_pin_toggle(LED_ALIVE);
}
