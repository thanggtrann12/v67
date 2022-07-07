#ifndef __RESP_DETECT_H
#define __RESP_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif
#include "../../DSP/Include/arm_math.h"
#include "hr_rr.h"

void resp_detect(resp_data_t *resp_data, const uint16_t data_size, uint8_t *RR, uint8_t preRR);
resp_status_t lifeline_resp(resplife_data_t* resplife_data, const uint16_t data_size);
void reset_lifeline_resp(void);
void resetApnea(void);
uint16_t getCountApnea(void);
uint16_t getCountApnea40s(void);
void find_resp_noise(actual_data_t *resp_data, const uint16_t data_size, float *eb_th_resp_freq, float *eb_th_resp_time);
void find_thresh_mov_resp(actual_data_t* resp_data, const uint16_t data_size, float* energy_mov_resp);
uint8_t check_valid_respRate(uint8_t rr, bool clean);
void thien_testResp(void);
typedef enum{
	NOBREATH_DISABLE = 0,
	NOBREATH_DEMO,
	NOBREATH_HIGH,
	NOBREATH_MEDIUM_2,
	NOBREATH_MEDIUM_1,
	NOBREATH_LOW,
}nobreathing_mode_t;

void set_time_nobreathing(nobreathing_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
