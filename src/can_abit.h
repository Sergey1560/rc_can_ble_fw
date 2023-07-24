#ifndef CAN_ABIT_H
#define CAN_ABIT_H

#ifdef CAN_ABIT

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_drv_common.h"

#include "mcp2515.h"

#define TO16B(X)    *((int16_t *)(&X))
#define CAN_RC_ID   (uint16_t)0x281

struct can_data_struct
{    
  uint16_t 	rpm;
  int16_t 	coolant_temp;
  int16_t 	intake_air_temp;
  uint8_t 	speed;
  uint8_t 	dros;
  uint16_t  fuel_press;
  uint16_t  oil_press;
  uint16_t  oil_temp;
  uint16_t   u_batt;
  int16_t   acc_x;
  int16_t   acc_y;
  uint16_t   lamda;
  uint16_t  brake_press;
  char      version[10];
  uint8_t   logger_enabled;
  uint8_t   check_engine;
  uint8_t   gear_num;
  uint32_t  iqr_call_count;
  uint32_t  iqr_call_rate;
  uint32_t  iqr_overall_call_count;
  uint32_t  iqr_overall_call_rate;
};


void adlm_parse_msg(struct can_message_t *msg);
void adlm_pack_data(uint8_t *data);

#endif


#endif