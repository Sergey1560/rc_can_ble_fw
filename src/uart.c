#include "uart.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"

//#define USE_APP_UART
#define USE_NATIVE_UART

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#define RX_PIN 15
#define TX_PIN 26

#ifdef USE_NATIVE_UART
static nrf_drv_uart_t m_uart = NRF_DRV_UART_INSTANCE(0);

uint8_t __attribute__ ((aligned (4))) uart_data[256];

static void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context){

    if (p_event->type == NRF_DRV_UART_EVT_RX_DONE)
    {
        if (p_event->data.rxtx.bytes)
        {
            nrf_drv_uart_rx(&m_uart, uart_data,p_event->data.rxtx.bytes);
//          uart_data[p_event->data.rxtx.bytes] = 0;
//          NRF_LOG_INFO("RX %d bytes \n %s",p_event->data.rxtx.bytes,uart_data);
            // if(data == 'a'){
            //     err_code = nrf_drv_uart_tx(&m_uart, my_text, 5);
            //     NRF_LOG_INFO("error code: %s.", NRF_LOG_ERROR_STRING_GET(err_code));
            // }
        }
    }
    else if (p_event->type == NRF_DRV_UART_EVT_ERROR)
    {
            NRF_LOG_ERROR("UART ERROR");
    }
    else if (p_event->type == NRF_DRV_UART_EVT_TX_DONE)
    {
       NRF_LOG_ERROR("TX DONE");
    }
}

void uart_init(void){
    uint32_t err_code;
    uint8_t data;
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    
    uart_config.baudrate = UART_BAUDRATE_BAUDRATE_Baud9600;
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

};

#endif

#ifdef USE_APP_UART
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */



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