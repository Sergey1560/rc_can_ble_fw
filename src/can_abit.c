#include "can_abit.h"
#ifdef CAN_ABIT
#include "abit_dtc.h"
#include "can.h"

static struct can_data_struct can_data;

void adlm_parse_msg(struct can_message_t *msg){
    int16_t i;
    static uint8_t dtc_present = 0;

    switch (msg->id){
        case 0x281:
                can_data.rpm = (uint16_t)TO16B(msg->data[0]);
                can_data.iqr_call_rate++;
                can_send_data();
                break;  

        case 0x283:
                can_data.dros = (uint8_t)(TO16B(msg->data[0]) / 10);
                can_send_data();
                break;  
                
        case 0x284:
                can_data.acc_x = TO16B(msg->data[0]);
                can_data.acc_y = TO16B(msg->data[2]);
                can_send_data();
                break;  

        case 0x381:
                can_data.lamda = (uint16_t)TO16B(msg->data[0]);
                can_send_data();
                break;  

        case 0x382:
                can_data.speed = (uint8_t)(TO16B(msg->data[0]) / 16);
                can_data.gear_num = msg->data[3];
                if(msg->data[4] > 0 && msg->data[4] < ABIT_DTC_COUNT){
                        dtc_present = 1;
                }else if((msg->data[4] == 0) && (dtc_present == 1)){
                        dtc_present = 0;
                }
                can_send_data();
                break;


        case 0x482: 
                i=TO16B(msg->data[0]);
                if(i < 0 ) i=0;
                can_data.fuel_press = i;
                
                i=TO16B(msg->data[2]);
                if(i < 0 ) i=0;
                can_data.oil_press = i;
                
                i=TO16B(msg->data[4]);
                if(i < 0 ) i=0;
                can_data.brake_press = i;

                can_send_data();
                break;

        case 0x581: 
                can_data.oil_temp = TO16B(msg->data[4]);
                can_send_data();
                break;


        case 0x582: 
                can_data.u_batt = TO16B(msg->data[6]);
                can_send_data();
                break;

        case 0x583: 
                can_data.coolant_temp = msg->data[0]-40;
                can_data.intake_air_temp = msg->data[1]-40;
                can_send_data();
                break;

        case 0x584:
                can_data.logger_enabled = msg->data[0] & 1;
                can_data.check_engine = ((msg->data[0] & 4) > 0) ? 1 : 0;
                can_send_data();
                break;

        case 0x781:
                can_data.version[0]=msg->data[0];
                can_data.version[1]=msg->data[1];
                can_data.version[3]=msg->data[2];
                can_data.version[4]=msg->data[3];
                can_data.version[6]=msg->data[4];
                can_data.version[7]=msg->data[5];
                can_data.version[8]=msg->data[6];
                can_data.version[9]=0;
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


void adlm_pack_data(uint8_t *data){
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

        tmp = 0;
        tmp |= ((can_data.lamda/10) & 0xFF) << (32 - 32);

        if(can_data.check_engine > 0){
        tmp |= (1 << (40 - 32));
        }

        tmp |= ((can_data.intake_air_temp+40) & 0xFF) << (41 -32);
        tmp |= ((can_data.oil_press/10) & 0x3F) << (49 - 32);
        tmp |= ((can_data.fuel_press/10) & 0x3F) << (55 - 32);
        tmp |= ((can_data.gear_num) & 0x7) << (61 - 32);

        ptr[4] = (tmp >> 0) & 0xFF ;
        ptr[5] = (tmp >> 8) & 0xFF ;
        ptr[6] = (tmp >> 16) & 0xFF ;
        ptr[7] = (tmp >> 24) & 0xFF ;

}


void adlm_update_stat(void){
	can_data.iqr_call_rate = can_data.iqr_call_count;
	can_data.iqr_call_count = 0;

	can_data.iqr_overall_call_rate = can_data.iqr_overall_call_count;
	can_data.iqr_overall_call_count = 0;
}


#endif