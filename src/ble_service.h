#ifndef BLE_CUSTOM_SERVICE_H
#define BLE_CUSTOM_SERVICE_H

/* This code belongs in ble_cus.h*/
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "ble_common.h"

//f364adc9-b000-4042-ba50-05ca45bf8abc
//00001ff8-0000-1000-8000-00805f9b34fb

//#define CUSTOM_SERVICE_UUID_BASE         {0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, 0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, 0xF3}

//RaceChrono UUID 00 00 1f f8-00 00-10 00-80 00-00 80 5f 9b 34 fb 
#define RCDIY_SERVICE_UUID_BASE         {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xf8, 0x1f, 0x00, 0x00}
#define RCDIY_SERVICE_UUID              0x1ff8

#define CAN_MAIN_UUID                   0x0001
#define CAN_MAIN_UUID_LEN               12 // 4 extID + 8 data

#define CAN_FILTER_UUID                 0x0002
#define CAN_FILTER_UUID_LEN             8 

#define GPS_MAIN_UUID                   0x0003
#define GPS_MAIN_UUID_LEN               20

#define GPS_TIME_UUID                   0x0004
#define GPS_TIME_UUID_LEN               3 

#define NOTIFY_DATA_INTERVAL            pdMS_TO_TICKS(50)


struct notification_enabled_t{
    uint8_t can_main;
    uint8_t gps_main;
    uint8_t gps_time;
};

enum CHAR_ID_t{
    ALL_CHARS_ID,
    CAN_MAIN_ID,
    GPS_MAIN_ID,
    GPS_TIME_ID
};


typedef enum
{
    BLE_CUS_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_CUS_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
    BLE_CUS_EVT_DISCONNECTED,
    BLE_CUS_EVT_CONNECTED
} ble_cus_evt_type_t;


/**@brief Custom Service event. */
typedef struct
{
    uint16_t           char_handle;
    ble_cus_evt_type_t evt_type;                                  /**< Type of event. */
} ble_cus_evt_t;

// Forward declaration of the ble_cus_t type.
typedef struct ble_cus_s ble_cus_t;

/**@brief Custom Service event handler type. */
typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_cus, ble_cus_evt_t * p_evt);


/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint8_t                       initial_custom_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_cus_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_cus_s
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      can_main_handles;               /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      can_filter_handles;             /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      gps_main_handles;             /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      gps_time_handles;             /**< Handles related to the Custom Value characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};


/**@brief   Macro for defining a ble_cus instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
//#define BLE_CUS_DEF(_name)      static ble_cus_t _name;     

#define BLE_CUS_DEF(_name)                                                                          \
static ble_cus_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_cus_on_ble_evt, &_name)

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init);
void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

uint32_t ble_data_update(ble_cus_t * p_cus, uint8_t char_id, uint8_t *data, uint32_t len);

void ble_notify_can_task(void *p);
void ble_notify_gps_task(void *p);
void control_notify_task(uint8_t flag);
void notify_set(uint8_t flag, enum CHAR_ID_t char_id);
uint32_t notify_get(enum CHAR_ID_t char_id);
int8_t handle_to_id(ble_cus_t * p_cus, ble_cus_evt_t *evt);

#endif