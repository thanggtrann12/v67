#ifndef __LIFE_SOS_H
#define __LIFE_SOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "log.h"
#include "arm_math.h"
#include "hr_rr.h"
#include "master_process.h"

#define HEART_WINDOW (uint16_t)1250 // 5 seconds

#define DECISION_TIME_SEIZURE (uint8_t)25 //25 seconds consecutive

float32_t signoise_heart(float32_t *signal_data, uint32_t size_of_data);
void powernoise_heart(float32_t* emptybed_sq_data, uint16_t emptydata_size, uint16_t heart_window, float32_t* avgdft_eb);
void snr_heart(float32_t *heart_square_4s, uint16_t data_size, float32_t noise_dft, float32_t *snr_dft);
bool lifeline_heart(float32_t *heart_data, uint16_t heart_data_size, movements_t *movements, uint8_t num_movs, float32_t th_adapt, float32_t noise_dft, float32_t avgsq_noise, bool *occupied, lavie_data_t *lavie_data, bool good_found);
void linesos(instant_data_t* instant_data, lavie_data_t *lavie_data);
void SeizureReset();
#ifdef __cplusplus
}
#endif

#endif
