#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_common.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"

#include "ble_common.h"
#include "ble_service.h"
#include "upd_timers.h"
#include "leds.h"

#include "mcp2515.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

volatile TimerHandle_t xOneSec_Timer;
volatile TaskHandle_t xCanTask;

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
//    NRF_LOG_INFO("Timer %d",ulCount);
}


static void int_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xCanTask, &xHigherPriorityTaskWoken);
    //NRF_LOG_INFO("CAN IRQ");
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void can_task(void *p){
    ret_code_t err_code;
    uint8_t data;
    struct can_message_t can_msg;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(MCP_INT_PIN, &in_config, int_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MCP_INT_PIN, true);

    mcp2515_init();

    while(1){
        ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(100));
        data = mcp2515_read_status();

        if(data & 3){
            if((data & 1)){
                mcp2515_get_msg(0, &can_msg);
                mcp2515_push_msg(&can_msg);
                NRF_LOG_INFO("RX0 ID: 0x%0X", can_msg.id);
            }

            if((data & (1<<1))){
                mcp2515_get_msg(1, &can_msg);
                mcp2515_push_msg(&can_msg);
                NRF_LOG_INFO("RX1 ID: 0x%0X", can_msg.id);
            }

            //NRF_LOG_INFO("Send notify");
            if(xNotifyTask != NULL){
                xTaskNotifyGive(xNotifyTask);
            };

        }else{
            //NRF_LOG_INFO("Data not present");
        }

    }
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
    rc_leds_init();
    rc_led_all(0);

    for(uint32_t i=0; i<10; i++){
        rc_led_all(1);
        for(uint32_t k=0; k<0x100000; k++){__NOP();};
        rc_led_all(0);
        for(uint32_t k=0; k<0x100000; k++){__NOP();};
    }

    //timers_init();
    power_management_init();
    bluetooth_start(0);
    xOneSec_Timer = xTimerCreate( "1STimer",pdMS_TO_TICKS(1000),pdTRUE,( void * ) 0, vOneSecTimer);
    xTimerStart(xOneSec_Timer,0);

    xTaskCreate(blink_task, "Blink", 256, NULL, 2, NULL);
    xTaskCreate(can_task, "Can", 1024, NULL, 2, (TaskHandle_t *)&xCanTask);
    
    vTaskStartScheduler();

    while(1);
}

