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

void blink_task(void *p){
    
    while(1){
        //NRF_LOG_INFO("Blink");
        bsp_board_led_invert(BSP_BOARD_LED_1);
        vTaskDelay(200);
    }
};

void vOneSecTimer( TimerHandle_t xTimer ){
	uint32_t ulCount;

	configASSERT( xTimer );
	
	ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
	ulCount++;

    vTimerSetTimerID( xTimer, ( void * ) ulCount );
    bsp_board_led_invert(BSP_BOARD_LED_0);
    //NRF_LOG_INFO("Timer");

}

volatile TimerHandle_t xOneSec_Timer;

int main(void)
{

    // Initialize.
    log_init();

    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    /* Configure LED-pins as outputs */
    bsp_board_init(BSP_INIT_LEDS);

    for(uint32_t i=0; i<20; i++){
        bsp_board_led_invert(BSP_BOARD_LED_0);
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

