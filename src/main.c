#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_common.h"
#include "nrf_drv_clock.h"

#include "ble_common.h"
#include "ble_service.h"
#include "upd_timers.h"
#include "leds.h"

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

void blink_task(void *p){
    
    while(1){
        rc_led_alive_invert();
        vTaskDelay(500);
    }
};

void vOneSecTimer( TimerHandle_t xTimer ){
	uint32_t ulCount;

	configASSERT( xTimer );
	
	ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
	ulCount++;

    vTimerSetTimerID( xTimer, ( void * ) ulCount );
    NRF_LOG_INFO("Timer %d",ulCount);

}

volatile TimerHandle_t xOneSec_Timer;

int main(void)
{
    ret_code_t err_code;
    
    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Start");

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    // /* Configure LED-pins as outputs */
    // bsp_board_init(BSP_INIT_LEDS);

    rc_leds_init();
    rc_led_all(0);

    for(uint32_t i=0; i<10; i++){
        //bsp_board_led_invert(BSP_BOARD_LED_0);
        rc_led_all(1);
        for(uint32_t k=0; k<0x100000; k++){__NOP();};
        rc_led_all(0);
        for(uint32_t k=0; k<0x100000; k++){__NOP();};
    }

    timers_init();
    power_management_init();
    bluetooth_start(0);
    xOneSec_Timer = xTimerCreate( "1STimer",pdMS_TO_TICKS(1000),pdTRUE,( void * ) 0, vOneSecTimer);
    xTimerStart(xOneSec_Timer,0);
    xTaskCreate(blink_task, "Blink", 256, NULL, 2, NULL);
    vTaskStartScheduler();

    while(1);
}

