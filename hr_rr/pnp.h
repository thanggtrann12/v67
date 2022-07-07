/*
 * pnp.h
 *
 *  Created on: Nov 4, 2021
 *      Author: Thien Phan
 */

#ifndef HR_RR_PNP_H_
#define HR_RR_PNP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"
#include "stdbool.h"

typedef struct{
	uint16_t index;
	float32_t th_adapt;
	float32_t avg_pks;
	float32_t avg_p;
	float32_t avg_np;
	uint16_t count_p;
	uint16_t count_np;
	float32_t pnp_value;
	float32_t avg_diff_np;
	float32_t np_ratio;
	bool flag_lowsig;
}pnp_data_table_t;

typedef struct{
	float X;
	float Y;
	float Z;
}channel_accel_t;
typedef struct{
	channel_accel_t AvgPksOpti1;
	channel_accel_t AvgPksOpti2;
	channel_accel_t AvgPksOpti3;
	channel_accel_t PnpBgRatio1;
	channel_accel_t PnpBgRatio2;
	channel_accel_t PnpBgRatio3;
	float PnpBgRatioSum;
}dev_params_t;

float get_pnp_bg(void);
void reset_pnp_bg(void);

void pnp(float32_t * square_data, uint16_t data_size, float32_t avgsq_noise, bool xz_add, pnp_data_table_t * pnp_opti,
		uint16_t *opti_loc, bool *good_found,  bool *flag_lowsig);
dev_params_t* get_dev_params();

#ifdef __cplusplus
}
#endif

#endif /* HR_RR_PNP_H_ */
