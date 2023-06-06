#include "uart.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"

#ifdef USE_NATIVE_UART

volatile TaskHandle_t xGpsTask;
volatile TaskHandle_t xGpsParse;
volatile QueueHandle_t GpsCmdQ_Handle;

const uint32_t uart_speed_list[]={UART_BAUDRATE_BAUDRATE_Baud115200,UART_BAUDRATE_BAUDRATE_Baud921600,UART_BAUDRATE_BAUDRATE_Baud38400,UART_BAUDRATE_BAUDRATE_Baud9600,UART_BAUDRATE_BAUDRATE_Baud57600,UART_BAUDRATE_BAUDRATE_Baud460800};
const uint32_t uart_speed_val[]={115200,921600,38400,9600,57600,460800};

static nrf_drv_uart_t m_uart = NRF_DRV_UART_INSTANCE(0);
uint8_t __attribute__ ((aligned (4))) uart_data[256];


static void uart_config(uint32_t speed);


static void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context){
    if (p_event->type == NRF_DRV_UART_EVT_RX_DONE){
        if (p_event->data.rxtx.bytes){
            nrf_drv_uart_rx(&m_uart, uart_data,p_event->data.rxtx.bytes);
            for(uint32_t i=0; i<p_event->data.rxtx.bytes; i++){
                ublox_input((uint8_t)uart_data[i]);
            }
        }
    }
    else if (p_event->type == NRF_DRV_UART_EVT_ERROR){
       NRF_LOG_ERROR("UART ERROR %d",p_event->data.error.error_mask);
       nrf_uart_error_mask_t
    }else if (p_event->type == NRF_DRV_UART_EVT_TX_DONE){
       NRF_LOG_ERROR("TX DONE");
    }
}


void vTaskGpsParse(void *arg){
    uint16_t crc, calc_crc;
    void (*ubx_parse) (uint8_t *msg, uint8_t len);

    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        crc = (new_msg.payload[new_msg.size-2] << 8) | (new_msg.payload[new_msg.size-1]);
        calc_crc = ublox_crc((struct ubx_packet*)&new_msg);

        if(crc ==  calc_crc){  //Packet Ok, Parse here
            ubx_parse = ublox_select_func(new_msg.msgid);
            if(ubx_parse != NULL){
                ubx_parse((uint8_t *)new_msg.payload,new_msg.size);
            }
        }else{
            NRF_LOG_ERROR("UBX crc fail!");
            NRF_LOG_ERROR("Calc CRC: %0X Packet CRC: %0X",calc_crc,crc);
            NRF_LOG_ERROR("ID: %0X Len: %d",new_msg.msgid,new_msg.size);
        }
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
	vTaskDelay(1700); //Задержка, для включения модуля GPS

	for(uint32_t i=0; i<(sizeof(uart_speed_list)/sizeof(uart_speed_list[0])); i++){
		NRF_LOG_INFO("Trying GPS at %d",uart_speed_val[i]);

		uart_config(uart_speed_list[i]);
		nrf_drv_uart_tx(&m_uart, (uint8_t *)ubx_poll_ver,sizeof(ubx_poll_ver));

        msg_class = ((ubx_poll_ver[2] << 8)|ubx_poll_ver[3]);
		
        xResult = xTaskNotifyWait(pdFALSE,     /* Не очищать биты на входе. */
                            0xffffffff,        /* На выходе очищаются все биты. */
                            &ulNotifiedValue, /* Здесь хранится значение оповещения. */
                            pdMS_TO_TICKS(1000));  /* Время таймаута на блокировке. */

        if(xResult == pdTRUE){
            if(ulNotifiedValue == msg_class){ 
                NRF_LOG_INFO("GPS baudrate found, %d",uart_speed_val[i]);
                uart_br_found=1;
                break;
            };
        }else{
            NRF_LOG_INFO("Timeout gps answer at %d",uart_speed_val[i]);
        }
		
	}

	if(!uart_br_found){
		NRF_LOG_ERROR("Can't setup uart baudrate");
		vTaskSuspend(NULL);
	}

	NRF_LOG_INFO("Setup new baudrate");
	nrf_drv_uart_tx(&m_uart, (uint8_t *)ubx_cfg_prt_ubx_115200,sizeof(ubx_cfg_prt_ubx_115200));
    vTaskDelay(100);
	
    uart_config(UART_BAUDRATE_BAUDRATE_Baud115200);

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
	cmd.cmd=(uint8_t *)ubx_cfg_rate_10Hz;
	cmd.size=sizeof(ubx_cfg_rate_10Hz);
	xQueueSend(GpsCmdQ_Handle,&cmd,portTICK_PERIOD_MS);

	//Запрос номера версии
	cmd.cmd=(uint8_t *)ubx_poll_ver;
	cmd.size=sizeof(ubx_poll_ver);
	xQueueSend(GpsCmdQ_Handle,&cmd,portTICK_PERIOD_MS);
	

	/* Основной цикл обработки команд из очереди */

	while(1){
		if(xQueuePeek(GpsCmdQ_Handle,&cmd,portMAX_DELAY)){  //Получить команду, но не убирать из очереди
			msg_class = ((cmd.cmd[2] << 8)|cmd.cmd[3]);
			nrf_drv_uart_tx(&m_uart, cmd.cmd,cmd.size);

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
                NRF_LOG_INFO("GPS cmd timeout %d",msg_class);
            }
		};

	}	

}


static void uart_config(uint32_t speed){
    uint32_t err_code;
    uint8_t data;

    nrf_drv_uart_uninit(&m_uart);

    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.baudrate = speed;
    uart_config.hwfc = NRF_UART_HWFC_DISABLED; 
    uart_config.parity = NRF_UART_PARITY_EXCLUDED;
    uart_config.pselrxd = RX_PIN; 
    uart_config.pseltxd = TX_PIN; 

    err_code = nrf_drv_uart_init(&m_uart, &uart_config, uart_event_handler);
  
    if(err_code != NRF_SUCCESS){
        NRF_LOG_ERROR("UART INIT ERROR: %d",err_code);
    }else{
        NRF_LOG_INFO("UART Init done");
    }

    err_code = nrf_drv_uart_rx(&m_uart, &data,1);
    if(err_code != NRF_SUCCESS){
        NRF_LOG_ERROR("UART RX ERROR: %d",err_code);
    }else{
        NRF_LOG_INFO("UART RX done");
    }
}

void uart_init(void){

    xTaskCreate(vTaskGps, "GPS", 1024, NULL, 2, (TaskHandle_t *)&xGpsTask);
    xTaskCreate(vTaskGpsParse, "GPP", 1024, NULL, 2, (TaskHandle_t *)&xGpsParse);

};

#endif

#ifdef USE_APP_UART
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED


void uart_error_handle(app_uart_evt_t * p_event)
{
    uint8_t cr;
    
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }else if(p_event->evt_type == APP_UART_DATA_READY)
        app_uart_get(&cr);
        NRF_LOG_INFO("UART DATA %c",cr);
    {
    }

}

void uart_init(void){
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN,
          TX_PIN,
          NRF_UART_PSEL_DISCONNECTED,
          NRF_UART_PSEL_DISCONNECTED,
          UART_HWFC,
          false,
          NRF_UART_BAUDRATE_115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
};

#endif