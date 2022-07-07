#ifndef __MASTER_PROCESS_H
#define __MASTER_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "log.h"
#include "arm_math.h"
#include "define.h"

#define ACTUAL_DATA_LEN (uint16_t)10000
#define HEART_WINDOW (uint16_t)1250 // 5 seconds
#define RESP_RAW_DATA_LEN (uint16_t)8192
#define RESP_LIFE_RAW_DATA_LEN (uint16_t)2500 //10s



void MasterProcess(void);
void MasterBoardManager(void);
void EmptyProcess(bool resetAllFlag);
void OccupiedProcess(bool startFlag);
void EmptyBedUpdate(bool val);

void enable_receive_master(void);
void disable_receive_master(void);

typedef struct{
	float32_t x1[ACTUAL_DATA_LEN];
	float32_t y1[ACTUAL_DATA_LEN];
	float32_t z1[ACTUAL_DATA_LEN];
#ifdef PAD_3_SENSOR
	float32_t x2[ACTUAL_DATA_LEN];
	float32_t y2[ACTUAL_DATA_LEN];
	float32_t z2[ACTUAL_DATA_LEN];
	float32_t x3[ACTUAL_DATA_LEN];
	float32_t y3[ACTUAL_DATA_LEN];
	float32_t z3[ACTUAL_DATA_LEN];
#endif
}actual_data_t;

typedef struct{
	float32_t x1[HEART_WINDOW];
	float32_t y1[HEART_WINDOW];
	float32_t z1[HEART_WINDOW];
#ifdef PAD_3_SENSOR
	float32_t x2[HEART_WINDOW];
	float32_t y2[HEART_WINDOW];
	float32_t z2[HEART_WINDOW];
	float32_t x3[HEART_WINDOW];
	float32_t y3[HEART_WINDOW];
	float32_t z3[HEART_WINDOW];
#endif
	bool occupied[HEART_WINDOW];
}instant_data_t;

typedef struct{
	float32_t x1[RESP_RAW_DATA_LEN];
	float32_t y1[RESP_RAW_DATA_LEN];
#ifdef PAD_3_SENSOR
	float32_t x2[RESP_RAW_DATA_LEN];
	float32_t y2[RESP_RAW_DATA_LEN];
	float32_t x3[RESP_RAW_DATA_LEN];
	float32_t y3[RESP_RAW_DATA_LEN];
#endif
}resp_data_t;

typedef struct{
	float32_t x1[RESP_LIFE_RAW_DATA_LEN];
	float32_t y1[RESP_LIFE_RAW_DATA_LEN];
#ifdef PAD_3_SENSOR
	float32_t x2[RESP_LIFE_RAW_DATA_LEN];
	float32_t y2[RESP_LIFE_RAW_DATA_LEN];
	float32_t x3[RESP_LIFE_RAW_DATA_LEN];
	float32_t y3[RESP_LIFE_RAW_DATA_LEN];
#endif
}resplife_data_t;
	
typedef struct{
	float avgsq_noise_x;
	float avgsq_noise_y;
	float avgsq_noise_z;
	float avgsq_noise_xyz;
	float eb_th_resp_freq;
	float eb_th_resp_time;
	float avg_energy_xy;
	float avg_energy_xyz;
	float avg_energy_resp;
}calib_params_t;


#ifdef __cplusplus
}
#endif

#endif
