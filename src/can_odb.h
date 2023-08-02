#ifndef CAN_ODB_H
#define CAN_ODB_H

#ifdef CAN_ODB

#include "mcp2515.h"
#include "can.h"

#define OBD_REQ             (uint16_t)0x7DF
#define OBD_ANS             (uint16_t)0x7E8
#define SWA_REQ             (uint16_t)0x797
#define SWA_ANS	            (uint16_t)0x79F
#define ABS_REQ	            (uint16_t)0x760
#define ABS_ANS	            (uint16_t)0x768
#define TEST_ANS	          (uint16_t)0x78

#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define SPEED               0x0D
#define DROS                0x09

#define CAN_RC_ID           (uint16_t)0x281

struct can_data_struct
{    
  uint16_t 	rpm;
  int16_t 	coolant_temp;
  uint8_t 	speed;
  uint8_t 	dros;
  uint32_t  iqr_call_count;
  uint32_t  iqr_call_rate;
  uint32_t  iqr_overall_call_count;
  uint32_t  iqr_overall_call_rate;
};


void odb_send_next(void);
void odb_parse_msg(struct can_message_t *msg);
void odb_pack_data(uint8_t *data);
void odb_update_stat(void);

#endif

#endif
