#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_clock.h"

#include "ble_common.h"
#include "ble_service.h"
#include "upd_timers.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

void blink_task(void *p){
    
    while(1){
        NRF_LOG_INFO("Blink");
        vTaskDelay(1000);
    }
};


int main(void)
{

    // Initialize.
    log_init();
    clock_init();
    //timers_init();
    power_management_init();
    bluetooth_start(0);
    
    BaseType_t xReturned = xTaskCreate(blink_task, "Blink", 256, NULL, 2, NULL);
    if (xReturned != pdPASS)
    {
        NRF_LOG_ERROR("Task failed");
    }else{
        vTaskStartScheduler();
    }

    while(1);
}

