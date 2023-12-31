#include "ble_service.h"
#include "sdk_common.h"
#include "ble_srv_common.h"
#include <string.h>
#include "nrf_gpio.h"

#include "mcp2515.h"
#include "can_abit.h"
#include "ublox.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "can.h"

#define NRF_LOG_MODULE_NAME BLE_S
#define NRF_LOG_LEVEL   4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


TimerHandle_t xTimers;

volatile uint8_t __attribute__ ((aligned (4))) can_main_data[CAN_MAIN_UUID_LEN];
volatile uint8_t __attribute__ ((aligned (4))) can_filter_data[CAN_FILTER_UUID_LEN];
volatile uint8_t __attribute__ ((aligned (4))) can_data[CAN_MAIN_UUID_LEN] = {0};
volatile uint8_t __attribute__ ((aligned (4))) gps_main_data[GPS_MAIN_UUID_LEN] = {0};
volatile uint8_t __attribute__ ((aligned (4))) gps_time_data[GPS_TIME_UUID_LEN] = {0};

volatile struct notification_enabled_t notification_enabled = {.can_main=0,.gps_main=0,.gps_time=0};

void notify_set(uint8_t flag, enum CHAR_ID_t char_id){
    NRF_LOG_INFO("Set notify %d for %d",flag,char_id);
    if(flag > 0){
        flag = 1;
    }

    switch (char_id)
    {
    case 1:
        notification_enabled.can_main = flag;
        control_notify_task(flag);
        break;

    case 2:
        notification_enabled.gps_main = flag;
        break;

    case 3:
        notification_enabled.gps_time = flag;
        break;

    default:
        NRF_LOG_INFO("Default value, disable all");
        notification_enabled.can_main = 0;
        control_notify_task(0);
        notification_enabled.gps_main = 0;
        notification_enabled.gps_time = 0;
        break;
    }

}

uint32_t notify_get(enum CHAR_ID_t char_id){
    uint32_t ret_val;

    switch (char_id)
    {
    case CAN_MAIN_ID:
        ret_val = notification_enabled.can_main;
        break;

    case GPS_MAIN_ID:
        ret_val = notification_enabled.gps_main;
        break;

    case GPS_TIME_ID:
        ret_val = notification_enabled.gps_time;
        break;

    default:
        ret_val =notification_enabled.can_main | notification_enabled.gps_main | notification_enabled.gps_time;
        break;
    }

    return ret_val;
}

int8_t handle_to_id(ble_cus_t * p_cus, ble_cus_evt_t *evt){

    if(evt->char_handle == p_cus->can_main_handles.cccd_handle){
        return CAN_MAIN_ID;
    }
    
    if(evt->char_handle == p_cus->gps_main_handles.cccd_handle){
        return GPS_MAIN_ID;
    }

    if(evt->char_handle == p_cus->gps_time_handles.cccd_handle){
        return GPS_TIME_ID;
    }


    return -1;
}

void control_notify_task(uint8_t flag){
    if(xTimers != NULL){
        if(flag){
            xTimerStart(xTimers, 0 );
        }else{
            xTimerStop(xTimers, 0 );
        }
    }
}
 

 void vTimerCallback( TimerHandle_t xTimer ){
    if(xNotifyCanTask != NULL){
        xTaskNotifyGive(xNotifyCanTask);
    };
 }

void ble_notify_can_task(void *p){
    NRF_LOG_DEBUG("CAN notify task started");

    if(CAN_NOTIFY_BY_TIMER == 1){
        xTimers= xTimerCreate("Timer", NOTIFY_DATA_INTERVAL , pdTRUE, ( void * ) 0, vTimerCallback);
    }else{
        xTimers=NULL;
    }

    while(1){
        xTaskNotifyWait(pdFALSE, 0xffffffff, NULL, portMAX_DELAY); 

        if((notification_enabled.can_main == 0)){
            NRF_LOG_INFO("Stop can notify timer");
            control_notify_task(0);
        }else{
            if(notification_enabled.can_main){
                CAN_PACK_DATA((uint8_t *)can_data);
                update_data((uint8_t *)can_data, CAN_MAIN_ID, CAN_MAIN_UUID_LEN);
            }
        }
    }
}

void ble_notify_gps_task(void *p){
    while(1){
        xTaskNotifyWait(pdFALSE, 0xffffffff, NULL, portMAX_DELAY); 
        
        if(notification_enabled.gps_main){
            rc_led_alive_invert();
            ublox_pack_data((uint8_t *)gps_main_data,(uint8_t *)gps_time_data);
            update_data((uint8_t *)gps_main_data,GPS_MAIN_ID, GPS_MAIN_UUID_LEN);
        }else{
            rc_led_alive(0);
        };
    }
}


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_CONNECTED;
    evt.char_handle = 0; 

    ublox_pack_data((uint8_t *)gps_main_data,(uint8_t *)gps_time_data);
    p_cus->evt_handler(p_cus, &evt);
}

static void on_par_update(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;

    gap_conn_params.min_conn_interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval;
    gap_conn_params.max_conn_interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval;
    gap_conn_params.slave_latency     = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency;
    gap_conn_params.conn_sup_timeout  = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

}

static void on_data_len_upd(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{

}

static void on_data_len_upd_req(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{

}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = (ble_gatts_evt_write_t *)&p_ble_evt->evt.gatts_evt.params.write;

    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_cus->can_main_handles.cccd_handle) && (p_evt_write->len == 2)){
        if (p_cus->evt_handler != NULL){
            ble_cus_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data)){
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
            }else{
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
            }
            evt.char_handle = p_evt_write->handle;
            p_cus->evt_handler(p_cus, &evt);
        }
    }

    if ((p_evt_write->handle == p_cus->gps_main_handles.cccd_handle) && (p_evt_write->len == 2)){
        if (p_cus->evt_handler != NULL){
            ble_cus_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data)){
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
            }else{
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
            }
            evt.char_handle = p_evt_write->handle;
            p_cus->evt_handler(p_cus, &evt);
        }
    }

    if ((p_evt_write->handle == p_cus->gps_time_handles.cccd_handle) && (p_evt_write->len == 2)){
        if (p_cus->evt_handler != NULL){
            ble_cus_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data)){
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
            }else{
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
            }
            evt.char_handle = p_evt_write->handle;
            p_cus->evt_handler(p_cus, &evt);
        }
    }


}

static uint32_t can_main_char_add(ble_cus_t * p_cus)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&cccd_md, 0, sizeof(cccd_md));

    //Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CAN_MAIN_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = CAN_MAIN_UUID_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = CAN_MAIN_UUID_LEN;
    attr_char_value.p_value   = (uint8_t *)can_main_data;

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md, &attr_char_value, &p_cus->can_main_handles);
    return err_code;
}

static uint32_t can_filter_char_add(ble_cus_t * p_cus)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CAN_FILTER_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = CAN_FILTER_UUID_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = CAN_FILTER_UUID_LEN;
    attr_char_value.p_value   = (uint8_t *)can_filter_data;

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md, &attr_char_value,  &p_cus->can_filter_handles);
    return err_code;
}



static uint32_t gps_main_char_add(ble_cus_t * p_cus)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = GPS_MAIN_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = GPS_MAIN_UUID_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = GPS_MAIN_UUID_LEN;
    attr_char_value.p_value   = (uint8_t *)gps_main_data;

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md, &attr_char_value,  &p_cus->gps_main_handles);
    return err_code;
}

static uint32_t gps_time_char_add(ble_cus_t * p_cus)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_USER;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = GPS_TIME_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = GPS_TIME_UUID_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = GPS_TIME_UUID_LEN;
    attr_char_value.p_value   = (uint8_t *)gps_time_data;
    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md, &attr_char_value,  &p_cus->gps_time_handles);
    return err_code;
}


uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_cus->evt_handler = p_cus_init->evt_handler;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {RCDIY_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = RCDIY_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Custom Value characteristic
    err_code = can_main_char_add(p_cus);
    if (err_code != NRF_SUCCESS){
        return err_code;
    }
    
    err_code = can_filter_char_add(p_cus);
    if (err_code != NRF_SUCCESS){
        return err_code;
    }

    err_code = gps_main_char_add(p_cus);
    if (err_code != NRF_SUCCESS){
        return err_code;
    }

    err_code = gps_time_char_add(p_cus);

    return err_code;
}

void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cus_t * p_cus = (ble_cus_t *) p_context;
    
    if (p_cus == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //NRF_LOG_INFO("BLE_GAP_EVT_CONNECTED");
            on_connect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //NRF_LOG_INFO("BLE_GAP_EVT_DISCONNECTED");
            on_disconnect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            //NRF_LOG_INFO("BLE_GAP_EVT_CONN_PARAM_UPDATE");
            on_par_update(p_cus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
             //NRF_LOG_INFO("BLE_GATTS_EVT_WRITE");
             on_write(p_cus, p_ble_evt);
             break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
             //NRF_LOG_INFO("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST");
             on_data_len_upd_req(p_cus, p_ble_evt);
             break;


        case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
             //NRF_LOG_INFO("BLE_GAP_EVT_DATA_LENGTH_UPDATE");
             on_data_len_upd(p_cus, p_ble_evt);
             break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
             //NRF_LOG_INFO("BLE_GATTS_EVT_HVN_TX_COMPLETE");
             break;

        default:
            NRF_LOG_INFO("Unknown BLE_GAP_EVT %d",p_ble_evt->header.evt_id);
            // No implementation needed.
            break;
    }
}

uint32_t ble_data_update(ble_cus_t * p_cus, uint8_t char_id, uint8_t *data, uint32_t len){

    if (p_cus == NULL){
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
    uint16_t value_handle;

    switch (char_id)
    {
    case CAN_MAIN_ID:
        value_handle = p_cus->can_main_handles.value_handle;
        break;

    case GPS_MAIN_ID:
        value_handle = p_cus->gps_main_handles.value_handle;
        break;

    case GPS_TIME_ID:
        value_handle = p_cus->gps_time_handles.value_handle;
        break;

    default:
        return NRF_ERROR_NULL;
        break;
    }

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = len;
    gatts_value.offset  = 0;
    gatts_value.p_value = data;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle, value_handle, &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
     }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }


    return err_code;
}