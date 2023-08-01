#ifdef UART_LEGACY

#include "uart_legacy.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"

#include "pins_config.h"

static nrf_drv_uart_t m_uart = NRF_DRV_UART_INSTANCE(0);
uint8_t __attribute__ ((aligned (4))) uart_tx_data[2048];

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
       NRF_LOG_ERROR("UART ERROR %d:",p_event->data.error.error_mask);
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



void uart_send_data(uint8_t *data, uint32_t len, uint8_t wait){

    for(uint32_t i=0; i<len; i++){
        uart_tx_data[i] = data[i];
    };
    
    nrf_drv_uart_tx(&m_uart, (uint8_t *)uart_tx_data,len);

    if(wait){
        while(nrf_drv_uart_tx_in_progress(&m_uart)){__NOP();};
    }
}



void uart_init(uint32_t speed, uint32_t timeout){
    uint32_t err_code;
    uint8_t uart_data;

    nrf_drv_uart_uninit(&m_uart);
    
    for(uint32_t i=0; i<0x10000; i++){__NOP();};

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


#endif