#ifndef __COMMON_DEFINE_H
#define __COMMON_DEFINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

#define DEVICE_ID_SIZE 10
#define TIME_SIZE 25

#define BODY_TEMP_MIN 34
#define BODY_TEMP_MAX 48

enum LAVIE_CASE{
	BLANK_CASE = 255,
	ERROR_CASE = 250,
	WEAK_CASE = 245,
	PREVIOUS_CASE = 240,
	BODY_TEMP_BLANK_CASE = 127,	
};

typedef enum{
	RESP_NORMAL = 0,
	RESP_VIBRATE,
	RESP_EMERGENCY,
}resp_status_t;

/*General Data sensor*/
typedef struct {
	int16_t adc1_x;
	int16_t adc1_y;
	int16_t adc1_z;
	int16_t adc2_x;
	int16_t adc2_y;
	int16_t adc2_z;
	int16_t adc3_x;
	int16_t adc3_y;
	int16_t adc3_z;
	int8_t bodyTemp;
	bool occupancy : 1;
	bool vibrate   : 1;	
}__attribute__((packed))data_master_node;

/*Data heart rate*/
typedef struct
{
	uint8_t instant_hr;
	uint8_t actual_hr;
	uint8_t rr;
	uint8_t apnea;
	uint8_t motions;
	uint8_t numMov5s;
	uint8_t seizure;
	uint8_t snoring;
	uint8_t snoring_noisy;
	float movEnergy;
	float maxPeak;
	float pnp_bg;
	float pnp_rt;
	uint32_t Th_rt;
	uint32_t Th_bg;
	float hrv;
	resp_status_t resp_status;
}__attribute__((packed))hr_rr_t;

/*Packet data heart rate*/
typedef struct
{
	hr_rr_t hr_rr;
	int8_t room_temp;
	uint8_t room_hum;
	int8_t body_temp_var;
	int8_t body_temp;
	bool bed_occupied;
}__attribute__((packed))lavie_data_t;

typedef enum{
	HEART_RATE_DATA,
	RESP_RATE_DATA,
	INSTANT_DATA,
	RESP_LIFE_DATA,
	SNORING_DATA,
	EMPTY_DATA,
	HAVE_MOV_DATA,
}hrRrDataType;

/*Packet data heart rate*/
typedef struct
{
	hrRrDataType type;
	lavie_data_t lavie_data;
}__attribute__((packed))hrRr2Display_t;

typedef struct{
	int8_t temp;
	uint8_t hum;
}RoomTH_t;

#ifdef __cplusplus
}
#endif

#endif
