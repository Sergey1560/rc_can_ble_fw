#include "upd_timers.h"

APP_TIMER_DEF(m_notification_timer_id);

void timers_init(void){
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

    err_code = app_timer_create(&m_notification_timer_id, APP_TIMER_MODE_REPEATED, notification_timeout_handler);
    APP_ERROR_CHECK(err_code);

};


void timer_control(uint8_t state){
    ret_code_t err_code;

    if(m_notification_timer_id != NULL){

        if(state == 0){
            err_code = app_timer_stop(m_notification_timer_id);
            APP_ERROR_CHECK(err_code);
        }

        if(state == 1){
            err_code = app_timer_start(m_notification_timer_id, RCDIY_NOTIFICATION_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
        }
    }

}