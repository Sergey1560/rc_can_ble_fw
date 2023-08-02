#ifndef CAN_H
#define CAN_H

#ifdef CAN_ODB
#include "can_odb.h"
#define CAN_ENABLE_FILTERS   1
#define CAN_NOMSG       	 odb_send_next()
#define CAN_PARSE_MSG(x)     odb_parse_msg(x)
#define CAN_PACK_DATA(x)     odb_pack_data(x)
#define CAN_UPD_STAT()       odb_update_stat()
#endif

#ifdef CAN_ABIT
#include "can_abit.h"
#define CAN_NOMSG       	    
#define CAN_PARSE_MSG(x)      adlm_parse_msg(x)
#define CAN_PACK_DATA(x)      adlm_pack_data(x)
#define CAN_UPD_STAT()        adlm_update_stat()
#endif



void can_init(void);
void can_send_data(void);

#endif