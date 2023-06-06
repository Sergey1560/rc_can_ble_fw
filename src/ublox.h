#ifndef UBLOX_H
#define UBLOX_H
#include <string.h>
#include "uart.h"

#define GMT_TIME  3
#define UBX_MSG_MAX_LEN		2048

#define  UBX_NAV_HPPOSECEF   (uint16_t)0x0113
#define  UBX_NAV_ODO   (uint16_t)0x0109
#define  UBX_NAV_PVT   (uint16_t)0x0107
#define  UBX_NAV_SAT   (uint16_t)0x0135
#define  UBX_MON_HW    (uint16_t)0x0A09
#define  UBX_MON_VER    (uint16_t)0x0A04
#define  UBX_ACK_ACK    (uint16_t)0x0501
#define  UBX_ACK_NAK    (uint16_t)0x0500


#define NAV_SAT_GPS  		 0
#define NAV_SAT_SBAS 		 1 
#define NAV_SAT_Galileo  2
#define NAV_SAT_BeiDou   3
#define NAV_SAT_IMES     4
#define NAV_SAT_QZSS	   5
#define NAV_SAT_GLONASS  6


#define NAV_PVT_VALID_DATE  1
#define NAV_PVT_VALID_TIME  (1 <<1)


#ifndef ALGN32
#define ALGN32 __attribute__ ((aligned (4)))
#endif


struct ubx_cmd{
	uint8_t *cmd;
	uint32_t size;
};

struct ubx_packet
{
	 uint16_t msgid;
	 uint16_t size;
	 uint8_t payload[UBX_MSG_MAX_LEN];
};


struct ublox_odometr
{
	 uint8_t version;
	 uint32_t gpstime;
	 uint32_t distance;
	 uint32_t totaldistance;
	 uint32_t ground_accurancy;
	
};

struct ublox_nav_sat{
	uint8_t numSvs;
	uint32_t gpstime;
	uint8_t glonass_sv;
	uint8_t glonass_sv_used;
	uint8_t gps_sv;
	uint8_t gps_sv_used;
	uint8_t sbas_sv;
	uint8_t sbas_sv_used;
	uint8_t galileo_sv;
	uint8_t galileo_sv_used;
	uint8_t beidou_sv;
	uint8_t beidou_sv_used;
	uint8_t imes_sv;
	uint8_t imes_sv_used;
	uint8_t qzss_sv;
	uint8_t qzss_sv_used;
};

struct ublox_nav_pvt
{
	 uint32_t gpstime;
	 uint16_t year;
	 uint8_t month;
	 uint8_t day;
	 uint8_t hour;
	 uint8_t minute;
	 uint8_t second;
     uint8_t validTime;
	 uint8_t validDate;
	 uint8_t fullyResolved;
	 uint8_t validFlag;
     uint8_t validMag;
	 int32_t nano;
	 uint8_t fixtype;
	 uint8_t gnssFixOk;
	 uint8_t diffSoln;
	 uint8_t carrSoln;
	 uint8_t numSV;
	 int32_t longitude;
	 int32_t latitude;
	 int32_t height;
	 int32_t hMSL;
	 uint32_t hAcc;
	 int32_t speed;
	 int32_t head;
	 uint32_t speed_acc;
	 uint16_t magDec;
};

struct ubx_nav_pvt_t
{
	 uint32_t gpstime;
	 uint16_t year;
	 uint8_t month;
	 uint8_t day;
	 uint8_t hour;
	 uint8_t minute;
	 uint8_t second;
     uint8_t valid_flag;
	 uint32_t tacc;
	 int32_t  nano;
	 uint8_t fixtype;
	 uint8_t flags;
	 uint8_t flags2;
	 uint8_t numsv;
	 int32_t longitude;
	 int32_t latitude;
	 int32_t height;
	 int32_t hMSL;
	 uint32_t hAcc;
	 uint32_t vAcc;
	 int32_t velN;
	 int32_t velE;
	 int32_t velD;
	 int32_t speed;
	 int32_t headMot;
	 uint32_t sAcc;
	 uint32_t headAcc;
	 uint16_t pDOP;
	 uint8_t reserved[6];
	 int32_t headVeh;
	 int16_t madDec;
	 uint16_t magAcc;
};



struct ublox_hwmon
{
	 uint16_t agcCnt;
	 uint8_t antStatus;
	 uint8_t antPower;
	 uint16_t noise;
	 uint8_t jammState;
	 uint8_t cwjamming;
	 uint8_t rtcCalib;
	 uint8_t safeboot;
};

struct ublox_nav_posllh
{
	 uint32_t gpstime;
	 int32_t longitude;
	 int32_t latitude;
	 int32_t height;
	 int32_t hMSL;
	 uint32_t hAcc;
	 uint32_t vAcc;
};

struct ublox_nav_posecef
{
	 uint32_t gpstime;
	 int32_t ecefX;
	 int32_t ecefY;
	 int32_t ecefZ;
	 uint32_t pAcc;
};

struct ublox_nav_hpposecef
{
	uint32_t gpstime;
	int32_t ecefX;
	int32_t ecefY;
	int32_t ecefZ;
	int8_t ecefXHp;
	int8_t ecefYHp;
	int8_t ecefZHp;
	uint32_t pAcc;
};

struct ublox_version
{
	char swVersion[30];
	char hwVersion[10];
	char extension[30*10];
	char FW[30];
	char MOD[30];
	uint8_t ext_count;
};

struct ublox_gps_data_t
{
	struct ublox_odometr odometr;
	struct ublox_nav_sat nav_sat;
	struct ublox_nav_pvt nav_pvt;
	struct ublox_hwmon	 hwmon;
	struct ublox_nav_posllh nav_posllh;
	struct ublox_nav_posecef nav_posecef;
	struct ublox_nav_hpposecef nav_hpposecef;
	struct ublox_version	version;
	uint32_t gps_update_count;
};



extern volatile struct ubx_packet new_msg;
//extern volatile void (*ubx_parse) (uint8_t *msg, uint8_t len);
// extern volatile struct ublox_nav_pvt ublox_navpvt;
// extern volatile struct ublox_odometr ublox_odo;
// extern volatile struct ublox_hwmon ublox_hw_mon;
// extern volatile struct ublox_nav_sat ublox_navsat;
// extern volatile uint32_t gps_update_count;
// extern volatile struct ublox_nav_hpposecef ublox_navhpposecef;
// extern volatile struct ublox_version ublox_ver;
//extern volatile uint8_t UART_GET_ACK;

extern volatile struct ublox_gps_data_t ublox_data;

void ublox_parse_navsat(uint8_t *msg, uint8_t len);
void ublox_parse_pvt(uint8_t *msg, uint8_t len);
void ublox_parse_odo(uint8_t *msg, uint8_t len);
void ublox_parse_hwmon(uint8_t *msg, uint8_t len);
void ublox_parse_hpposecef(uint8_t *msg, uint8_t len);
void ublox_parse_monver(uint8_t *msg, uint8_t len);
void ublox_parse_ack(uint8_t *msg, uint8_t len);

uint16_t  ublox_crc(struct ubx_packet* pkt);

void ublox_input(uint8_t Data);
void* ublox_select_func(uint16_t msg_id);

//static uint8_t ublox_cmd_ubx_only [] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xE1,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xDC,0xBD};

static const uint8_t ubx_cfg_save [] = {0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x1D,0xAB};

static const uint8_t ubx_cfg_prt_ubx_nmea_57600  [] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xE1,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xDE,0xC9};
static const uint8_t ubx_cfg_prt_ubx_nmea_115200 [] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0xE7};


static const uint8_t ubx_cfg_prt_ubx_921600 [] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x10,0x0E,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x13,0x1E};
static const uint8_t ubx_cfg_prt_ubx_460800 [] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x08,0x07,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x0A,0xB0};
static const uint8_t ubx_cfg_prt_ubx_230400 [] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x84,0x03,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x82,0xDC};
static const uint8_t ubx_cfg_prt_ubx_115200 [] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xBE,0x72};
static const uint8_t ubx_cfg_prt_ubx_57600  [] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xE1,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xDC,0xBD};
static const uint8_t ubx_cfg_prt_ubx_38400  [] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x8D,0x64};

static const uint8_t ubx_cfg_prt_ubx_only_115200 [] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xB8,0x42};

static const uint8_t ubx_cfg_odo_stop          [] = {0xB5,0x62,0x06,0x1E,0x14,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x32,0x00,0x00,0x99,0x4C,0x00,0x00,0x5C,0x1D};
static const uint8_t ubx_cfg_odo_start_car     [] = {0xB5,0x62,0x06,0x1E,0x14,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x32,0x00,0x00,0x99,0x4C,0x00,0x00,0x5D,0x2D};
static const uint8_t ubx_cfg_odo_start_run     [] = {0xB5,0x62,0x06,0x1E,0x14,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x32,0x00,0x00,0x99,0x4C,0x00,0x00,0x5A,0x00};
static const uint8_t ubx_cfg_odo_start_ciclyng [] = {0xB5,0x62,0x06,0x1E,0x14,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x32,0x00,0x00,0x99,0x4C,0x00,0x00,0x5B,0x0F};

static const uint8_t ubx_msg_odo_reset     [] = {0xB5,0x62,0x01,0x10,0x00,0x00,0x11,0x34};																						
																			

static const uint8_t ubx_cfg_msg_odo_enable  [] = {0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x09,0x00,0x01,0x00,0x00,0x00,0x00,0x1A,0xEF};
static const uint8_t ubx_cfg_msg_odo_disable [] = {0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x19,0xEA};
																						
static const uint8_t ubx_cfg_rate_10Hz[] ={0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12};
static const uint8_t ubx_cfg_rate_20Hz[] ={0xB5,0x62,0x06,0x08,0x06,0x00,0x32,0x00,0x01,0x00,0x01,0x00,0x48,0xE6};
static const uint8_t ubx_cfg_rate_25Hz[] ={0xB5,0x62,0x06,0x08,0x06,0x00,0x28,0x00,0x01,0x00,0x01,0x00,0x3E,0xAA};

static const uint8_t ubx_cfg_nav5_auto[] = {0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x04,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,
																				0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x3C,0x00,0x00,0x00,0x00,0xC8,0x00,
																				0x00,0x00,0x00,0x00,0x00,0x00,0x18,0xE4};

static const uint8_t ubx_cfg_nav5_ped[] = {0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x03,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,
																				0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x3C,0x00,0x00,0x00,0x00,0xC8,0x00,
																				0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xC2};
																				
static const uint8_t ubx_cfg_nav5_bike[] = {0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x0A,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,
																				0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x3C,0x00,0x00,0x00,0x00,0xC8,0x00,
																				0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0xB0};

static const uint8_t ubx_msg_navpvt_disable [] = {0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC};
static const uint8_t ubx_msg_navpvt_enable []  = {0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1};

static const uint8_t ubx_msg_navsat_disable  [] = {0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x35,0x00,0x00,0x00,0x00,0x00,0x00,0x45,0x1E};
static const uint8_t ubx_msg_navsat_enable   []  = {0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x35,0x00,0x01,0x00,0x00,0x00,0x00,0x46,0x23};
static const uint8_t ubx_msg_navsat_enable2  []  = {0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x35,0x00,0x02,0x00,0x00,0x00,0x00,0x47,0x28};
static const uint8_t ubx_msg_navsat_enable4  []  = {0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x35,0x00,0x04,0x00,0x00,0x00,0x00,0x49,0x32};
static const uint8_t ubx_msg_navsat_enable10 []  = {0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x35,0x00,0x0A,0x00,0x00,0x00,0x00,0x4F,0x50};

static const uint8_t ubx_msg_hwmon_enable [] =  {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x0A, 0x09, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x23, 0x37};
static const uint8_t ubx_msg_hwmon_enable10 []= {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x0A, 0x09, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x64};
static const uint8_t ubx_msg_hwmon_disable [] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x0A, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x32};

static const uint8_t ubx_msg_hpposecef_enable [] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x13, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x27, 0x44}; 
static const uint8_t ubx_msg_hpposecef_disable[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x30};

static const uint8_t ubx_msg_itfm_enable [] = {0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xF3, 0xAC, 0x62, 0xAD, 0x1E, 0x63, 0x00, 0x00, 0x76, 0xA5};
static const uint8_t ubx_msg_itfm_disable[] = {0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xF3, 0xAC, 0x62, 0x2D, 0x1E, 0x23, 0x00, 0x00, 0xB6, 0x65};

static const uint8_t ubx_cfg_gnss_gps_glonass [] = {
0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08,
0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00,
0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x2F, 0x5D };

static const uint8_t ubx_cfg_gnss_gps [] = {
0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x20, 0x00, 0x01, 0x00, 0x01, 0x01,
0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08,
0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00,
0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x3E, 0xB9 };


static const uint8_t ubx_cfg9_gps_glo [] = {
0xB5, 0x62, 0x06, 0x3E, 0x2C, 0x00, 0x00, 0x00, 0x3C, 0x05, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x11, 0x01, 
0x02, 0x0A, 0x12, 0x00, 0x00, 0x00, 0x21, 0x01, 0x03, 0x04, 0x05, 0x00, 0x00, 0x00, 0x11, 0x01, 0x05, 0x00, 
0x03, 0x00, 0x00, 0x00, 0x11, 0x01, 0x06, 0x08, 0x0C, 0x00, 0x01, 0x00, 0x11, 0x01, 0x81, 0x8C };

static const uint8_t ubx_cfg9_gps_gal_glo [] = {
0xB5, 0x62, 0x06, 0x8A, 0x4F, 0x00, 0x01, 0x03, 0x00, 0x00, 0x01, 0x00, 0x31, 0x10, 0x01, 0x03, 0x00, 0x31, 0x10, 0x01, 0x07, 0x00, 0x31, 0x10, 0x01, 
0x0A, 0x00, 0x31, 0x10, 0x01, 0x0D, 0x00, 0x31, 0x10, 0x01, 0x0E, 0x00, 0x31, 0x10, 0x00, 0x12, 0x00, 0x31, 0x10, 0x01, 0x15, 0x00, 0x31, 0x10, 0x01, 
0x18, 0x00, 0x31, 0x10, 0x01, 0x1A, 0x00, 0x31, 0x10, 0x01, 0x1F, 0x00, 0x31, 0x10, 0x01, 0x21, 0x00, 0x31, 0x10, 0x01, 0x22, 0x00, 0x31, 0x10, 0x00, 
0x24, 0x00, 0x31, 0x10, 0x00, 0x25, 0x00, 0x31, 0x10, 0x01, 0xF2, 0x48};


static const uint8_t ubx_poll_ver [] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};

/* Пример пакетов NAV-PVT */
static const uint8_t ubx_data1[]={0x01, 0x07, 0x5C, 0x00, 0x5C, 0x9C, 0x02, 0x00, 0xDF, 0x07, 0x0A, 0x12, 0x00, 0x02, 0x33, 0xF0,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0xF5, 0x05, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x98, 0xBD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0x00, 0x7B, 0x84, 0xDF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x4E, 0x00, 0x00, 0x80, 0xA8, 0x12, 0x01,
0x0F, 0x27, 0x00, 0x00, 0xF8, 0x4A, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xCF, 0x0F};

static const uint8_t ubx_data2[]={0x01, 0x07, 0x5C, 0x00, 0x04, 0x05, 0x06, 0x00, 0xDF, 0x07, 0x0A, 0x12, 0x00, 0x06, 0x22, 0xF0,
0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x65, 0xCD, 0x1D, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x98, 0xBD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0x00, 0x96, 0x84, 0xDF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x4E, 0x00, 0x00, 0x80, 0xA8, 0x12, 0x01,
0x0F, 0x27, 0x00, 0x00, 0xF8, 0x4A, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x68, 0x84};

#endif


