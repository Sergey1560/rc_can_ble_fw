#include "upd_timers.h"

#include "FreeRTOS.h"
#include "timers.h"


#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */
#if 0

//APP_TIMER_DEF(m_notification_timer_id);
static TimerHandle_t candata_timer;                               /**< Definition of battery timer. */


void timers_init(void){
    candata_timer = xTimerCreate("CAN",
                                   CAN_MAIN_DATA_INTERVAL,
                                   pdTRUE,
                                   NULL,
                                   notification_timeout_handler);
};


void timer_control(uint8_t state){
    if(candata_timer != NULL){

        if(state == 0){
                NRF_LOG_INFO("Stop timer");
                if (pdPASS != xTimerStop(candata_timer, OSTIMER_WAIT_FOR_QUEUE))
                {
                    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
                }
         }

         if(state == 1){
                NRF_LOG_INFO("Start timer");
                if (pdPASS != xTimerStart(candata_timer, OSTIMER_WAIT_FOR_QUEUE))
                {
                    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
                }
         }
     }

}

#endif