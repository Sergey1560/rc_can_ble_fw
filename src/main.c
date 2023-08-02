#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_common.h"
#include "nrf_drv_clock.h"

#define NRF_LOG_MODULE_NAME MAIN
#define NRF_LOG_LEVEL   4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "ble_common.h"
#include "leds.h"
#include "gps.h"
#include "can.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    ret_code_t err_code;
    
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("Start");

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_clock_hfclk_request(NULL);
    while (!nrf_drv_clock_hfclk_is_running()){}

    nrf_drv_clock_lfclk_request(NULL);
    while (!nrf_drv_clock_lfclk_is_running()){}

    rc_leds_init();
    rc_led_all(0);

    for(uint32_t i=0; i<10; i++){
        rc_led_all(1);
        for(uint32_t k=0; k<0x100000; k++){__NOP();};
        rc_led_all(0);
        for(uint32_t k=0; k<0x100000; k++){__NOP();};
    }

    power_management_init();
    gps_init();
    can_init();
    bluetooth_start(0);

    vTaskStartScheduler();

    while(1);
}



void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    {
		NRF_LOG_ERROR("Task %s stack ovewflow",pcTaskName);
	    while (1)
        {
            __NOP();
        }
    }
}
