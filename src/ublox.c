#include "ublox.h"


volatile struct ublox_gps_data_t ublox_data;
volatile struct ubx_packet ALGN32 new_msg;
volatile uint8_t ALGN32 ubx_msg[UBX_MSG_MAX_LEN];

void* ublox_select_func(uint16_t msg_id){
	void (*ubx_parse) (uint8_t *msg, uint8_t len);

	switch (msg_id) {
		case UBX_NAV_ODO: {
			ubx_parse=ublox_parse_odo;
			break;
		};
		case UBX_NAV_PVT: {
			ubx_parse=ublox_parse_pvt;
			break;
		};
		case UBX_NAV_SAT: {
			ubx_parse=ublox_parse_navsat;
			break;
		};
		case UBX_MON_HW: {
			ubx_parse=ublox_parse_hwmon;
			break;
		};
		case UBX_MON_VER: {
			ubx_parse=ublox_parse_monver;
			break;
		};
		case UBX_ACK_ACK: {
			NRF_LOG_INFO("Ack packet");
			ubx_parse=ublox_parse_ack;
			break;
		};
		case UBX_NAV_HPPOSECEF: {
			ubx_parse=ublox_parse_hpposecef;
			break;
		};
	default:
		ubx_parse=NULL;
		break;
	};

	return ubx_parse;
}


void ublox_input(uint8_t Data){
	static uint16_t ubx_msg_index=0;
	static uint16_t ubx_msg_start=0;
	static uint16_t payload_size=0;
	
	if((Data == 0xB5) && (ubx_msg_start == 0)) {
		ubx_msg_start=1;
	}else if((Data == 0x62) && (ubx_msg_start == 1)){
			ubx_msg_start=2;
			ubx_msg_index=0;
			payload_size=(UBX_MSG_MAX_LEN-1);
	}else if(ubx_msg_start == 1){
		ubx_msg_start=0;
	}else if(ubx_msg_start >=2){
		if(ubx_msg_index < (payload_size+6)){ 
			ubx_msg[ubx_msg_index]=(uint8_t)Data;

			if(ubx_msg_index == 3) {
				payload_size = (ubx_msg[ubx_msg_index] << 8) | ubx_msg[ubx_msg_index-1];
				if((payload_size+6) > UBX_MSG_MAX_LEN){
					ubx_msg_start=0;
				};
			};
			ubx_msg_start++;
			ubx_msg_index++;
		}else{
			
			new_msg.msgid = (ubx_msg[0] << 8) | ubx_msg[1];
			new_msg.size = payload_size;
			
			for(uint32_t i=0; i<payload_size+2; i++){
				new_msg.payload[i] = ubx_msg[i+4];
			}

			NRF_LOG_INFO("Get msgid 0x%04X size %d",new_msg.msgid,new_msg.size);

			if(((uint8_t)Data == 0xB5)) {  //Next packet comming 
				ubx_msg_start=1;
			}else{
				ubx_msg_start=0;
			};
			ubx_msg_index=0;

			if(xGpsParse != NULL){
				vTaskNotifyGiveFromISR(xGpsParse, NULL);
			}

		};
	}else{
		//NRF_LOG_INFO("Byte not in order");
	};
}

void ublox_parse_navsat(uint8_t *msg, uint8_t len){
	uint8_t gnssId;
	uint32_t flags;
	uint8_t  valid;
	
	ublox_data.nav_sat.gpstime = (msg[3] << 24) | (msg[2] << 16) | (msg[1] << 8) | msg[0];
	ublox_data.nav_sat.numSvs = msg[5];
	
	ublox_data.nav_sat.beidou_sv=0;
	ublox_data.nav_sat.beidou_sv_used=0;
	ublox_data.nav_sat.galileo_sv=0;
	ublox_data.nav_sat.galileo_sv_used=0;
	ublox_data.nav_sat.glonass_sv=0;
	ublox_data.nav_sat.glonass_sv_used=0;
	ublox_data.nav_sat.gps_sv=0;
	ublox_data.nav_sat.gps_sv_used=0;
	ublox_data.nav_sat.imes_sv=0;
	ublox_data.nav_sat.imes_sv_used=0;
	ublox_data.nav_sat.qzss_sv=0;
	ublox_data.nav_sat.qzss_sv_used=0;
	ublox_data.nav_sat.sbas_sv=0;
	ublox_data.nav_sat.sbas_sv_used=0;
	
	for(uint8_t i=0; i<ublox_data.nav_sat.numSvs; i++){
			gnssId = msg[8+12*i];
			//svId = msg[9+12*i];
			flags = (msg[19] << 24) | (msg[18] << 16) | (msg[17+12*i] << 8) | msg[16+12*i];
			
		  valid = flags & (1<<3);
			
			 switch (gnssId) {
				 case NAV_SAT_GPS: {
						if(valid){
							ublox_data.nav_sat.gps_sv_used++;
						}else{
							ublox_data.nav_sat.gps_sv++;
						};
					 break;
				 };

				 case NAV_SAT_BeiDou: {
						if(valid){
							ublox_data.nav_sat.beidou_sv_used++;
						}else{
							ublox_data.nav_sat.beidou_sv++;
						};
					 break;
				 };
			 
 				 case NAV_SAT_Galileo: {
						if(valid){
							ublox_data.nav_sat.galileo_sv_used++;
						}else{
							ublox_data.nav_sat.galileo_sv++;
						};
					 break;
				 };

 				 case NAV_SAT_GLONASS: {
						if(valid){
							ublox_data.nav_sat.glonass_sv_used++;
						}else{
							ublox_data.nav_sat.glonass_sv++;
						};
					 break;
				 };

 				 case NAV_SAT_IMES: {
						if(valid){
							ublox_data.nav_sat.imes_sv_used++;
						}else{
							ublox_data.nav_sat.imes_sv++;
						};
					 break;
				 };

 				 case NAV_SAT_QZSS: {
						if(valid){
							ublox_data.nav_sat.qzss_sv_used++;
						}else{
							ublox_data.nav_sat.qzss_sv++;
						};
					 break;
				 };

 				 case NAV_SAT_SBAS: {
						if(valid){
							ublox_data.nav_sat.sbas_sv_used++;
						}else{
							ublox_data.nav_sat.sbas_sv++;
						};
					 break;
				 };
			 };
	};

};

void ublox_parse_pvt(uint8_t *msg, uint8_t len){

	struct ubx_nav_pvt_t *ubx_navpvt = (struct ubx_nav_pvt_t *)msg;

	ublox_data.nav_pvt.gpstime = ubx_navpvt->gpstime;
	ublox_data.nav_pvt.year = ubx_navpvt->year;
	ublox_data.nav_pvt.month = ubx_navpvt->month;
	ublox_data.nav_pvt.day = ubx_navpvt->day;
	ublox_data.nav_pvt.hour = ubx_navpvt->hour;
	ublox_data.nav_pvt.minute = ubx_navpvt->minute;
	ublox_data.nav_pvt.second = ubx_navpvt->second;
	
	ublox_data.nav_pvt.validDate = ubx_navpvt->valid_flag & 1; 
	ublox_data.nav_pvt.validTime = (ubx_navpvt->valid_flag >> 1) & 1;
	ublox_data.nav_pvt.fullyResolved = (ubx_navpvt->valid_flag >> 2) & 1;
	ublox_data.nav_pvt.validMag = (ubx_navpvt->valid_flag >> 3) & 1;
	
	ublox_data.nav_pvt.validFlag = ubx_navpvt->valid_flag;
	
	ublox_data.nav_pvt.nano = ubx_navpvt->nano;
	
	ublox_data.nav_pvt.fixtype = ubx_navpvt->fixtype;
	
	ublox_data.nav_pvt.gnssFixOk = ubx_navpvt->flags & 1;
	ublox_data.nav_pvt.diffSoln = (ubx_navpvt->flags >> 1 ) & 1;
	ublox_data.nav_pvt.carrSoln = (ubx_navpvt->flags >> 6) & 3;


	ublox_data.nav_pvt.numSV = ubx_navpvt->numsv;
	ublox_data.nav_pvt.longitude = ubx_navpvt->longitude;
	ublox_data.nav_pvt.latitude = ubx_navpvt->latitude;
	
	ublox_data.nav_pvt.height = ubx_navpvt->height;
	ublox_data.nav_pvt.hMSL = ubx_navpvt->hMSL;
	
	ublox_data.nav_pvt.speed = (ubx_navpvt->speed*36)/1000; // mm/s => km/h
	ublox_data.nav_pvt.head = ubx_navpvt->headMot;
	ublox_data.nav_pvt.hAcc = ubx_navpvt->hAcc;
	ublox_data.nav_pvt.magDec = ubx_navpvt->madDec;

	ublox_data.gps_update_count++;

};



void  ublox_parse_odo(uint8_t *msg, uint8_t len){
	ublox_data.odometr.version = msg[0];
	ublox_data.odometr.gpstime = (msg[7] << 24) | (msg[6] << 16) | (msg[5] << 8) | msg[4];
	ublox_data.odometr.distance = (msg[11] << 24) | (msg[10] << 16) | (msg[9] << 8) | msg[8];
	ublox_data.odometr.totaldistance = (msg[15] << 24) | (msg[14] << 16) | (msg[13] << 8) | msg[12];
	ublox_data.odometr.ground_accurancy = (msg[19] << 24) | (msg[18] << 16) | (msg[17] << 8) | msg[16];
};


void  ublox_parse_hwmon(uint8_t *msg, uint8_t len){
	ublox_data.hwmon.noise = 	(msg[17] << 8) | msg[16];
	ublox_data.hwmon.agcCnt = (msg[19] << 8) | msg[18];
	ublox_data.hwmon.antStatus = msg[20];
	ublox_data.hwmon.antPower = msg[21];
	ublox_data.hwmon.rtcCalib = msg[22] & 1;
	ublox_data.hwmon.safeboot = (msg[22] >> 1) & 1;
    ublox_data.hwmon.jammState = (msg[22] >> 2) & 3;
	ublox_data.hwmon.cwjamming = msg[45];
	//percent
	ublox_data.hwmon.cwjamming = ublox_data.hwmon.cwjamming*100/255;
	ublox_data.hwmon.agcCnt = (uint16_t)(((uint32_t)ublox_data.hwmon.agcCnt*100)/8191);
};


void  ublox_parse_hpposecef(uint8_t *msg, uint8_t len){
	ublox_data.nav_hpposecef.gpstime = (msg[7] << 24) | (msg[6] << 16) | (msg[5] << 8) | msg[4];

	ublox_data.nav_hpposecef.ecefX = (msg[11] << 24) | (msg[10] << 16) | (msg[9] << 8) | msg[8];
	ublox_data.nav_hpposecef.ecefY = (msg[15] << 24) | (msg[14] << 16) | (msg[13] << 8) | msg[12];
	ublox_data.nav_hpposecef.ecefZ = (msg[19] << 24) | (msg[18] << 16) | (msg[17] << 8) | msg[16];
	
	ublox_data.nav_hpposecef.ecefXHp = msg[20];
	ublox_data.nav_hpposecef.ecefYHp = msg[21];
	ublox_data.nav_hpposecef.ecefZHp = msg[22];

	ublox_data.nav_hpposecef.pAcc = (msg[27] << 24) | (msg[26] << 16) | (msg[25] << 8) | msg[24];
};


uint16_t  ublox_crc(struct ubx_packet* pkt){
	uint8_t cka;
	uint8_t ckb;
	uint16_t crc=0;

	cka=(uint8_t)((pkt->msgid >> 8) & 0xFF);;
	ckb=cka;

	cka+=(uint8_t)(pkt->msgid & 0xFF);
	ckb+=cka;

	cka+=(uint8_t)((pkt->size) & 0xFF);
	ckb+=cka;

	cka+=(uint8_t)(((pkt->size) >> 8) & 0xFF);
	ckb+=cka;

	for(uint32_t i=0; i<(pkt->size); i++){
		cka+=pkt->payload[i];
		ckb+=cka;
		crc=(uint16_t)(cka << 8)|(ckb);
	};

	return crc;
};


void  ublox_parse_monver(uint8_t *msg, uint8_t len){
	char* s;
		
	memset((struct ublox_version*)&ublox_data.version,0,sizeof(struct ublox_version));
	
	memcpy((char *)ublox_data.version.swVersion,msg,30);
	msg+=30;
	memcpy((char *)ublox_data.version.hwVersion,msg,10);
	msg+=10;
	memcpy((char *)ublox_data.version.extension,msg,len-40);
	
	ublox_data.version.ext_count = (len-40)/30;

	for(uint8_t i=0; i<ublox_data.version.ext_count; i++){
		s=(char *)&ublox_data.version.extension + i*30;
		if(!strncmp(s,"FWVER=",6)){
			strcpy((char *)&ublox_data.version.FW,s+6);
		}
		if(!strncmp(s,"MOD=",4)){
			strcpy((char *)&ublox_data.version.MOD,s+4);
		}
	}

	//xEventGroupSetBits(xGpsEventGroup,UBX_MON_VER);
	xTaskNotifyFromISR(xGpsTask, UBX_MON_VER, eSetBits, NULL);
};

void  ublox_parse_ack(uint8_t *msg, uint8_t len){
	uint16_t ubx_ack=0;
	
	ubx_ack = (msg[0] << 8) | msg[1];
	(void)ubx_ack;

	NRF_LOG_INFO("Get ASK 0x%04X",ubx_ack);

	//xEventGroupSetBits(xGpsEventGroup,ubx_ack);
	xTaskNotifyFromISR(xGpsTask, ubx_ack, eSetBits, NULL);
};
