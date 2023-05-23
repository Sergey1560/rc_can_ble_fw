#include "upd_timers.h"

#include "FreeRTOS.h"
#include "timers.h"


#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

//APP_TIMER_DEF(m_notification_timer_id);
static TimerHandle_t candata_timer;                               /**< Definition of battery timer. */


void timers_init(void){
    // Initialize timer module.
    // ret_code_t err_code = app_timer_init();
    // APP_ERROR_CHECK(err_code);

    // Create timers.

    // err_code = app_timer_create(&m_notification_timer_id, APP_TIMER_MODE_REPEATED, notification_timeout_handler);
    // APP_ERROR_CHECK(err_code);

    candata_timer = xTimerCreate("CAN",
                                   CAN_MAIN_DATA_INTERVAL,
                                   pdTRUE,
                                   NULL,
                                   notification_timeout_handler);


};


void timer_control(uint8_t state){
    //ret_code_t err_code;

    if(candata_timer != NULL){

        if(state == 0){
                NRF_LOG_INFO("Stop timer");
                if (pdPASS != xTimerStop(candata_timer, OSTIMER_WAIT_FOR_QUEUE))
                {
                    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
                }

             //err_code = app_timer_stop(m_notification_timer_id);
             //APP_ERROR_CHECK(err_code);
         }

         if(state == 1){
                NRF_LOG_INFO("Start timer");
                if (pdPASS != xTimerStart(candata_timer, OSTIMER_WAIT_FOR_QUEUE))
                {
                    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
                }

            //  err_code = app_timer_start(m_notification_timer_id, RCDIY_NOTIFICATION_INTERVAL, NULL);
            //  APP_ERROR_CHECK(err_code);
         }
     }

}