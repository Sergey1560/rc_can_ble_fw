#include "uart.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"

#include "pins_config.h"

volatile TaskHandle_t xGpsTask;
volatile TaskHandle_t xGpsParse;
volatile QueueHandle_t GpsCmdQ_Handle;

const uint32_t uart_speed_list[]={UART_BAUDRATE_BAUDRATE_Baud9600,UART_BAUDRATE_BAUDRATE_Baud19200,UART_BAUDRATE_BAUDRATE_Baud38400};
const uint32_t uart_speed_val[]={9600,19200,38400};

static nrf_drv_uart_t m_uart = NRF_DRV_UART_INSTANCE(0);
uint8_t __attribute__ ((aligned (4))) uart_tx_data[2048];

static void uart_config(uint32_t speed);

/*
static void uart_error_desk(uint32_t mask){
    //nrf_uart_error_mask_t

    if(mask & NRF_UART_ERROR_OVERRUN_MASK){
        NRF_LOG_ERROR("->OVERRUN");
    }
    if(mask & NRF_UART_ERROR_PARITY_MASK){
        NRF_LOG_ERROR("->PARITY");
    }
    if(mask & NRF_UART_ERROR_FRAMING_MASK){
        NRF_LOG_ERROR("->FRAMING");
    }
    if(mask & NRF_UART_ERROR_BREAK_MASK){
        NRF_LOG_ERROR("->BREAK");
    }

    mask &= ~(NRF_UART_ERROR_OVERRUN_MASK|NRF_UART_ERROR_PARITY_MASK|NRF_UART_ERROR_FRAMING_MASK|NRF_UART_ERROR_BREAK_MASK);


}
*/

static void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context){
    uint8_t uart_data;
    if (p_event->type == NRF_DRV_UART_EVT_RX_DONE){
        if (p_event->data.rxtx.bytes){
            nrf_drv_uart_rx(&m_uart, &uart_data, 1);
            //NRF_LOG_INFO("Get byte");
            ublox_input((uint8_t)uart_data);
        }
    }
    else if (p_event->type == NRF_DRV_UART_EVT_ERROR){
       //NRF_LOG_ERROR("UART ERROR %d:",p_event->data.error.error_mask);
       nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_RXTO);
       nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_RXDRDY);
       nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_ERROR);

        
        if(p_event->data.error.error_mask & NRF_UART_ERROR_OVERRUN_MASK){
            NRF_LOG_ERROR("UART OVR");
            for(uint32_t i = 0; i < 6; i++)
            {
                nrf_uart_rxd_get(m_uart.uart.p_reg);
            }        
        }
        
        nrf_drv_uart_rx(&m_uart, &uart_data, 1);
       
        //uart_error_desk(p_event->data.error.error_mask);
             
       
    }else if (p_event->type == NRF_DRV_UART_EVT_TX_DONE){
       //NRF_LOG_ERROR("TX DONE");
    }
}


void vTaskGpsParse(void *arg){
    uint16_t crc, calc_crc;
    void (*ubx_parse) (uint8_t *msg, uint8_t len);

    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        crc = (new_msg.payload[new_msg.size] << 8) | (new_msg.payload[new_msg.size+1]);
        calc_crc = ublox_crc((struct ubx_packet*)&new_msg);

        if(crc ==  calc_crc){  
            //NRF_LOG_INFO("Pkt %04X",new_msg.msgid);
            ubx_parse = ublox_select_func(new_msg.msgid);
            if(ubx_parse != NULL){
                ubx_parse((uint8_t *)new_msg.payload,new_msg.size);
            }
        }else{
            //NRF_LOG_ERROR("Calc FAIL ID: %04X Len: %d CRC: %0X Packet CRC: %0X",new_msg.msgid,new_msg.size,calc_crc,crc);
        }
    }
}

static void uart_send_data(uint8_t *data, uint32_t len, uint8_t wait){

    for(uint32_t i=0; i<len; i++){
        uart_tx_data[i] = data[i];
    };
    
    //NRF_LOG_INFO("Send pkt %04X",(data[2] << 8)|data[3]);
    nrf_drv_uart_tx(&m_uart, (uint8_t *)uart_tx_data,len);

    if(wait){
        while(nrf_drv_uart_tx_in_progress(&m_uart)){__NOP();};
    }
}


void vTaskGps(void *arg){
	struct ubx_cmd cmd;
	uint16_t msg_class;
	uint32_t ulNotifiedValue;
    BaseType_t xResult;
	uint8_t uart_br_found=0;

	/* Очередь команд к GPS приемнику */
	GpsCmdQ_Handle = xQueueCreate( 30, sizeof(struct ubx_cmd));
    vTaskDelay(1000); //Задержка, для включения модуля GPS

    while(!uart_br_found){
        for(uint32_t i=0; i<(sizeof(uart_speed_list)/sizeof(uart_speed_list[0])); i++){
            NRF_LOG_INFO("Trying GPS at %d",uart_speed_val[i]);
            
            uart_config(uart_speed_list[i]);
            vTaskDelay(50);

            uart_send_data((uint8_t *)ubx_cfg_prt_poll,sizeof(ubx_cfg_prt_poll),1);

            msg_class = ((ubx_cfg_prt_poll[2] << 8)|ubx_cfg_prt_poll[3]);
            
            xResult = xTaskNotifyWait(pdFALSE,     /* Не очищать биты на входе. */
                                0xffffffff,        /* На выходе очищаются все биты. */
                                &ulNotifiedValue, /* Здесь хранится значение оповещения. */
                                pdMS_TO_TICKS(1000));  /* Время таймаута на блокировке. */

            if(xResult == pdTRUE){
                if(ulNotifiedValue == msg_class){ 
                    NRF_LOG_INFO("GPS baudrate found, %d 0x%04X 0x%04X",uart_speed_val[i],ulNotifiedValue,msg_class);
                    uart_br_found=1;
                    break;
                }else{
                    NRF_LOG_ERROR("GPS Notify val 0x%0X msg 0x%0X",ulNotifiedValue,msg_class);
                };
            }else{
                NRF_LOG_INFO("Timeout gps answer");
            }
            
        }
        vTaskDelay(2000);
    }


	NRF_LOG_INFO("Setup new baudrate msg");
	uart_send_data((uint8_t *)ubx_cfg_prt_ubx_only_38400,sizeof(ubx_cfg_prt_ubx_only_38400),1);
    vTaskDelay(100);
    
    NRF_LOG_INFO("Setup new baudrate cfg");
    uart_config(UART_BAUDRATE_BAUDRATE_Baud38400);
    
    vTaskDelay(100);

    //Clean all notifycations
    while(xTaskNotifyWait(pdFALSE, 0xffffffff, &ulNotifiedValue, pdMS_TO_TICKS(0)) == pdTRUE){__NOP();}
    
    
    for(uint32_t i=0; i<10; i++){
        NRF_LOG_INFO("Waiting ACK %d",i);
        uart_send_data((uint8_t *)ubx_cfg_prt_poll,sizeof(ubx_cfg_prt_poll),1);
        
        xResult = xTaskNotifyWait(pdFALSE, 0xffffffff, &ulNotifiedValue, pdMS_TO_TICKS(300));
        msg_class = ((ubx_cfg_prt_poll[2] << 8)|ubx_cfg_prt_poll[3]);
        
        if(xResult == pdTRUE){
            if(ulNotifiedValue == msg_class){
                NRF_LOG_INFO("Get ACK, uart ready");
                break;
            }
        }
    }


	//Отключить сообщение одометра
	cmd.cmd=(uint8_t *)ubx_cfg_msg_odo_disable;
	cmd.size=sizeof(ubx_cfg_msg_odo_disable);
	xQueueSend(GpsCmdQ_Handle,&cmd,portTICK_PERIOD_MS);

	//Остановка одометра
	cmd.cmd=(uint8_t *)ubx_cfg_odo_stop;
	cmd.size=sizeof(ubx_cfg_odo_stop);
	xQueueSend(GpsCmdQ_Handle,&cmd,portTICK_PERIOD_MS);
 
	//Сброс одометра
	cmd.cmd=(uint8_t *)ubx_msg_odo_reset;
	cmd.size=sizeof(ubx_msg_odo_reset);
	xQueueSend(GpsCmdQ_Handle,&cmd,portTICK_PERIOD_MS);
 
	//Сообщения NAVPVT
	cmd.cmd=(uint8_t *)ubx_msg_navpvt_enable;
	cmd.size=sizeof(ubx_msg_navpvt_enable);
	xQueueSend(GpsCmdQ_Handle,&cmd,portTICK_PERIOD_MS);
 
    //Сообщение NAVSAT
	cmd.cmd=(uint8_t *)ubx_msg_navsat_disable;
	cmd.size=sizeof(ubx_msg_navsat_disable);
	xQueueSend(GpsCmdQ_Handle,&cmd,portTICK_PERIOD_MS);
 
	//Сообщение CFG-NAV5
	cmd.cmd=(uint8_t *)ubx_cfg_nav5_auto;
	cmd.size=sizeof(ubx_cfg_nav5_auto);
	xQueueSend(GpsCmdQ_Handle,&cmd,portTICK_PERIOD_MS);
 
	//Сообщение 10Hz
	cmd.cmd=(uint8_t *)ubx_cfg_rate_25Hz;
	cmd.size=sizeof(ubx_cfg_rate_25Hz);
	xQueueSend(GpsCmdQ_Handle,&cmd,portTICK_PERIOD_MS);
 
	//Запрос номера версии
	cmd.cmd=(uint8_t *)ubx_poll_ver;
	cmd.size=sizeof(ubx_poll_ver);
	xQueueSend(GpsCmdQ_Handle,&cmd,portTICK_PERIOD_MS);
	

	/* Основной цикл обработки команд из очереди */

	while(1){
		if(xQueuePeek(GpsCmdQ_Handle,&cmd,portMAX_DELAY)){  //Получить команду, но не убирать из очереди
			msg_class = ((cmd.cmd[2] << 8)|cmd.cmd[3]);
			uart_send_data(cmd.cmd,cmd.size,1);

			//Ждать 2 секунды выставления флагов. Флаг выставляется
			//при приеме ACK пакета в значение MSG_ID
            xResult = xTaskNotifyWait(pdFALSE,     /* Не очищать биты на входе. */
                                0xffffffff,        /* На выходе очищаются все биты. */
                                &ulNotifiedValue, /* Здесь хранится значение оповещения. */
                                pdMS_TO_TICKS(2000));  /* Время таймаута на блокировке. */

            if(xResult == pdTRUE){
                if(ulNotifiedValue == msg_class){ 
                    xQueueReceive(GpsCmdQ_Handle,&cmd,0);
                }else{
                    NRF_LOG_ERROR("GPS Notify val 0x%0X msg 0x%0X",ulNotifiedValue,msg_class);
                };
            }else{
                NRF_LOG_INFO("GPS cmd timeout %0X",msg_class);
            }
		};
	}	

}


static void uart_config(uint32_t speed){
    uint32_t err_code;
    uint8_t uart_data;

    nrf_drv_uart_uninit(&m_uart);
    vTaskDelay(100);

    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.baudrate = speed;
    uart_config.hwfc = NRF_UART_HWFC_DISABLED; 
    uart_config.parity = NRF_UART_PARITY_EXCLUDED;
    uart_config.pselrxd = RX_PIN; 
    uart_config.pseltxd = TX_PIN; 
    //uart_config.interrupt_priority = 2;

    err_code = nrf_drv_uart_init(&m_uart, &uart_config, uart_event_handler);
  
    if(err_code != NRF_SUCCESS){
        NRF_LOG_ERROR("UART INIT ERROR: %d",err_code);
    }else{
        NRF_LOG_INFO("UART Init done");
    }

    nrf_drv_uart_rx(&m_uart, &uart_data, 1);

    for(uint32_t i = 0; i < 6; i++){
        nrf_uart_rxd_get(m_uart.uart.p_reg);
    }        
        
}

void uart_init(void){

    xTaskCreate(vTaskGps, "GPS", 1024, NULL, 2, (TaskHandle_t *)&xGpsTask);
    xTaskCreate(vTaskGpsParse, "GPP", 1024, NULL, 2, (TaskHandle_t *)&xGpsParse);

};

