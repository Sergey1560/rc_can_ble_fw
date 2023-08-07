#ifdef UART_LIBUARTE

#include "uart_libuarte.h"
#include "pins_config.h"
#include "nrf_delay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define NRF_LOG_MODULE_NAME UARTE
#define NRF_LOG_LEVEL   4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "gps.h"

uint8_t __attribute__ ((aligned (4))) uart_tx_data[2048];

//SoftDevice TIMER0 RTC0
NRF_LIBUARTE_ASYNC_DEFINE(libuarte,  /* name        */
0,                                   /* _uarte_idx  */
1,                                   /* _timer0_idx */
NRF_LIBUARTE_PERIPHERAL_NOT_USED,    /* _rtc1_idx   */
2,                                   /* _timer1_idx */
32,                                  /* _rx_buf_size*/
3                                    /* _rx_buf_cnt */
);

ret_code_t uart_send_data(uint8_t *data, uint32_t len, uint8_t wait){
    ret_code_t err_code;
    uint8_t *data_ptr = data;

    //Data not in RAM, or not aligned
    if(((uint32_t)data < 0x20000000) || (((uint32_t)data % 4) != 0) ){
        for(uint32_t i=0; i<len;i++){
            uart_tx_data[i] = data[i];
        }
        data_ptr = uart_tx_data;
    }

    do{
        err_code = nrf_libuarte_async_tx(&libuarte, data_ptr, len);
        if(err_code == NRF_ERROR_BUSY){
            NRF_LOG_INFO("TX Busy");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }while(err_code == NRF_ERROR_BUSY);

    if (err_code != NRF_SUCCESS){
        char const * p_desc = nrf_strerror_find(err_code);
        if (p_desc == NULL){
            NRF_LOG_INFO("TX return code: UNKNOWN (%x)", err_code);
        }else{
            NRF_LOG_INFO("TX return code: %s", p_desc);
        }

    }
    
    return err_code;
}


void uart_event_handler(void * context, nrf_libuarte_async_evt_t * p_evt){
    nrf_libuarte_async_t * p_libuarte = (nrf_libuarte_async_t *)context;

    switch (p_evt->type){
        case NRF_LIBUARTE_ASYNC_EVT_ERROR:
            if(uart_br_found){
                NRF_LOG_ERROR("UART Error");
            }
            break;
        case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
            //NRF_LOG_INFO("Get %d bytes",p_evt->data.rxtx.length);

            for(uint32_t i=0; i<p_evt->data.rxtx.length; i++){
                ublox_input(p_evt->data.rxtx.p_data[i]);
            }

            nrf_libuarte_async_rx_free(p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);

            break;
        case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
            NRF_LOG_DEBUG("TX done");
            break;
        default:
            break;
    }
}


void uart_init(nrf_uarte_baudrate_t speed, uint32_t timeout){
    ret_code_t err_code;

    nrf_libuarte_async_uninit(&libuarte);

    nrf_libuarte_async_config_t nrf_libuarte_async_config = {
            .tx_pin     = TX_PIN,
            .rx_pin     = RX_PIN,
            .baudrate   = speed,
            .parity     = NRF_UARTE_PARITY_EXCLUDED,
            .hwfc       = NRF_UARTE_HWFC_DISABLED,
            .timeout_us = timeout,
            .int_prio   = APP_IRQ_PRIORITY_LOW
    };


    err_code = nrf_libuarte_async_init(&libuarte, &nrf_libuarte_async_config, uart_event_handler, (void *)&libuarte);

    if (err_code != NRF_SUCCESS){
        char const * p_desc = nrf_strerror_find(err_code);
        if (p_desc == NULL){
            NRF_LOG_INFO("Function return code: UNKNOWN (%x)", err_code);
        }else{
            NRF_LOG_INFO("Function return code: %s", p_desc);
        }
    } 


    nrf_libuarte_async_enable(&libuarte);

}

#endif