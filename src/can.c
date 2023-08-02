#include "can.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "nrf_drv_gpiote.h"

#include "ble_common.h"
#include "pins_config.h"


#define NRF_LOG_MODULE_NAME CAN
#define NRF_LOG_LEVEL   4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

volatile TaskHandle_t xCanTask;

static void int_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xCanTask, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}



void can_task(void *p){
    BaseType_t xResult;
    ret_code_t err_code;
    uint8_t data;
    struct can_message_t can_msg;

    NRF_LOG_INFO("Start CAN task");
    
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(MCP_INT_PIN, &in_config, int_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MCP_INT_PIN, true);

    mcp2515_init();

    while(1){
        xResult = xTaskNotifyWait(pdFALSE,     /* Не очищать биты на входе. */
                                0xffffffff,        /* На выходе очищаются все биты. */
                                NULL, /* Здесь хранится значение оповещения. */
                                pdMS_TO_TICKS(100));  /* Время таймаута на блокировке. */

        if(xResult != pdTRUE){
            NRF_LOG_DEBUG("CAN Reply timout");
            CAN_NOMSG;
        }

        data = mcp2515_read_status();
        
        if(data & 3){
            if((data & 1)){
                mcp2515_get_msg(0, &can_msg);
                CAN_PARSE_MSG(&can_msg);
                //NRF_LOG_INFO("[%d]GET RX0 0x%0X",xTaskGetTickCount(),can_msg.id);
                //NRF_LOG_HEXDUMP_INFO(can_msg.data,8);
            }

            if((data & (1<<1))){
                mcp2515_get_msg(1, &can_msg);
                CAN_PARSE_MSG(&can_msg);
                //NRF_LOG_INFO("[%d]GET RX1 0x%0X",xTaskGetTickCount(),can_msg.id);
                //NRF_LOG_HEXDUMP_INFO(can_msg.data,8);
            }
        }
    }
}

void can_send_data(void){
    //xTaskNotifyGive(xNotifyCanTask);
}


void can_init(void){
    
    xTaskCreate(can_task, "Can", 1024, NULL, 2, (TaskHandle_t *)&xCanTask);
};