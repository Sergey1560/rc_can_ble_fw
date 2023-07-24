#include "can_odb.h"
#ifdef CAN_ODB
static struct can_data_struct can_data;

const struct can_message_t canreq_rpm     = {OBD_REQ,8,{0x02,0x01,ENGINE_RPM,00,00,00,00,00} };
const struct can_message_t canreq_coolant = {OBD_REQ,8,{0x02,0x01,ENGINE_COOLANT_TEMP,00,00,00,00,00} };
const struct can_message_t canreq_speed   = {OBD_REQ,8,{0x02,0x01,SPEED,00,00,00,00,00} };
const struct can_message_t canreq_drossel = {OBD_REQ,8,{0x03,0x22,DROS,0xd4,00,00,00,00} };
const struct can_message_t* can_msg[]= {&canreq_rpm,&canreq_coolant,&canreq_speed,&canreq_drossel};
#define MAX_MSG_INDEX ((sizeof(can_msg)/sizeof(can_msg[0])) - 1)


void odb_send_next(void){
	static uint8_t can_queque_index=0;
	if(can_queque_index > MAX_MSG_INDEX) can_queque_index=0;
	mcp2515_send_msg((struct can_message_t*)can_msg[can_queque_index]);
	can_queque_index++; 
};


void odb_parse_msg(struct can_message_t *msg){
	switch (msg->id){
		case OBD_ANS:
					switch (msg->data[2]){
						case ENGINE_RPM:
										can_data.rpm=(((uint16_t)msg->data[3] << 8)+msg->data[4])/4;
										break;
						case ENGINE_COOLANT_TEMP:
										can_data.coolant_temp=msg->data[3]-40;
										break;
						case SPEED:
										can_data.speed=msg->data[3];
										break;
						case DROS:
										can_data.dros=msg->data[4]/2;
										break;
						default :
										break;  
					}
					can_data.iqr_call_count++;
					odb_send_next();
				break;  

		default :
				break;  
	};
	can_data.iqr_overall_call_count++;
}

/*
8 * 8 = 64 бита

0   rpm         14b     0 - 16383
14  tps         10b     0 - 1023
24  coolant     08b     0 - 255
32  lamda       08b      0 - 255   0
40  ce          01b                8
41  intake      08b     0 - 255    9
49  oil press   06b     0 - 127    16
55  fuel press  06b     0 - 127    23
61  gear        03b     0 - 7
*/


void odb_pack_data(uint8_t *data){
        uint32_t tmp = 0;
        uint8_t *ptr = data+4;

        data[0] = CAN_RC_ID & 0xFF;
        data[1] = (CAN_RC_ID >> 8) & 0xFF;
        data[2] = 0;
        data[3] = 0;

        tmp = (can_data.rpm & 0x3FFF);
        tmp |= (can_data.dros & 0x3FF) << 14;
        tmp |= ((can_data.coolant_temp+40) & 0xFF) << 24;

        ptr[0] = tmp & 0xFF ;
        ptr[1] = (tmp >> 8)  & 0xFF ;
        ptr[2] = (tmp >> 16) & 0xFF ;
        ptr[3] = (tmp >> 24) & 0xFF ;

        ptr[4] = 0;
        ptr[5] = 0;
        ptr[6] = 0;
        ptr[7] = 0;
}

#endif