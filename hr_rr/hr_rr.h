#ifndef __HR_RR_H
#define __HR_RR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"
#include "stdbool.h"
#include "master_process.h"
#include "define.h"

#define WINDOW_SIZE_MOV  (uint16_t)250 // movement window size 250 (1 second)
#define MAX_MOV_40s_NUM    (uint16_t)40 //ACTUAL_DATA_LEN/WINDOW_SIZE_MOV
#define MAX_MOV_5s_NUM    (uint16_t)5 //HEART_WINDOW/WINDOW_SIZE_MOV
//#define MOV_MAX_ENERGY 		2.0e+8
//#define MOV_RESP_MAX_ENERGY 		3.0e+10
#define MOV_MAX_XYZ_ENERGY 		2.5e+8
#define MOV_MAX_XY_ENERGY 		1.5e+8
#define MOV_MAX_RESP_ENERGY 	1e+11

typedef struct{
	float32_t	amp; // amplitude
	uint16_t loc; //location
}Peak_;

typedef struct{
	uint16_t	loc_start; // location start
	uint16_t loc_end; //location end
}movements_t;

uint8_t search_movements(const float32_t *heart_data_bpf,const uint16_t data_size, float32_t avg_energy, movements_t* movements, float32_t* avg_energy_out, float* avg_energy_all_out, float *maxPeakOut);
uint8_t search_movements_resp(const float *resp, const uint16_t resp_size, float avg_energy, float resp_time, movements_t* movements, float* avg_energy_out);

void hr_detect(float32_t * square_data, uint16_t data_size, float32_t th_adapt, movements_t* movements, uint8_t num_moves, bool xz_add, uint16_t timegap_in, float32_t avgsq_noise,
	bool *heart_lowsig, uint16_t *timegap_out, uint16_t *heart_beats, uint8_t *HR, float *HRV);
void hr_rr(actual_data_t* actual_data ,lavie_data_t *lavie_data, uint8_t preHR);

#ifdef __cplusplus
}
#endif

#endif
