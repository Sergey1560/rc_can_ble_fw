#include "gps.h"

#define NRF_LOG_MODULE_NAME GPS
#define NRF_LOG_LEVEL   4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


volatile TaskHandle_t xGpsTask;
volatile TaskHandle_t xGpsParse;
volatile QueueHandle_t GpsCmdQ_Handle;

volatile uint8_t uart_br_found=0;

void vTaskGpsParse(void *arg){
    uint16_t crc, calc_crc;
    void (*ubx_parse) (uint8_t *msg, uint8_t len);

    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        crc = (new_msg.payload[new_msg.size] << 8) | (new_msg.payload[new_msg.size+1]);
        calc_crc = ublox_crc((struct ubx_packet*)&new_msg);

        if(crc ==  calc_crc){  
            ubx_parse = ublox_select_func(new_msg.msgid);
            if(ubx_parse != NULL){
                ubx_parse((uint8_t *)new_msg.payload,new_msg.size);
            }
        }else{
            NRF_LOG_ERROR("Calc FAIL ID: %04X Len: %d CRC: %0X Packet CRC: %0X",new_msg.msgid,new_msg.size,calc_crc,crc);
        }
    }
}


void vTaskGps(void *arg){
	struct ubx_cmd cmd;
	uint16_t msg_class;
	uint32_t ulNotifiedValue;
    BaseType_t xResult;
	

	/* Очередь команд к GPS приемнику */
	GpsCmdQ_Handle = xQueueCreate( 30, sizeof(struct ubx_cmd));
    vTaskDelay(pdMS_TO_TICKS(1000)); //Задержка, для включения модуля GPS

    while(!uart_br_found){
        for(uint32_t i=0; i<(sizeof(uart_speed_list)/sizeof(uart_speed_list[0])); i++){
            NRF_LOG_INFO("Trying GPS at %d",uart_speed_val[i]);
            
            uart_init(uart_speed_list[i],1000);
            vTaskDelay(pdMS_TO_TICKS(50));

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
        vTaskDelay(pdMS_TO_TICKS(2000));
    }


	NRF_LOG_INFO("Setup new baudrate msg");
	uart_send_data((uint8_t *)ubx_cfg_prt_ubx_only_115200,sizeof(ubx_cfg_prt_ubx_only_115200),1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    NRF_LOG_INFO("Setup new baudrate cfg");
    uart_init(NRF_UARTE_BAUDRATE_115200,1000);
    
    vTaskDelay(pdMS_TO_TICKS(100));

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
 
	//Сообщение 25Hz
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
                NRF_LOG_INFO("GPS cmd %0X timeout",msg_class);
            }
		};
	}	

}


void gps_init(void){

    xTaskCreate(vTaskGps, "GPS", 1024, NULL, 2, (TaskHandle_t *)&xGpsTask);
    xTaskCreate(vTaskGpsParse, "GPP", 1024, NULL, 2, (TaskHandle_t *)&xGpsParse);

};
