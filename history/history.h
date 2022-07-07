/*
 * history.h
 *
 *  Created on: Jun 2, 2021
 *      Author: Thien Phan
 */

#ifndef HISTORY_HISTORY_H_
#define HISTORY_HISTORY_H_

#include "stdint.h"
#include "stdbool.h"
#include "define.h"

typedef struct{
	uint8_t min;
	uint8_t avg;
	uint8_t max;
}__attribute__((packed))Off_u8;

typedef struct{
	uint8_t min;
	uint8_t avg;
	uint8_t max;
}__attribute__((packed))Off_u32;

typedef struct{
	int8_t min;
	int8_t avg;
	int8_t max;
}__attribute__((packed))Off_i8;

typedef struct{
	float min;
	float avg;
	float max;
}__attribute__((packed))Off_f32;



/*Packet data heart rate*/
typedef struct
{
	Off_u8 instant_hr;
	Off_u8 actual_hr;
	Off_u8 rr;
	uint8_t totalApnea;
	uint8_t totalMotions;
	uint8_t totalSnoring;
	Off_f32 pnp_bg;
	Off_f32 pnp_rt;
	Off_u32 Th_rt;
	Off_u32 Th_bg;
	Off_i8 room_temp;
	Off_u8 room_hum;
	Off_u8 body_temp;
	bool bed_occupied;
	char time[TIME_SIZE];
}__attribute__((packed))lavie_offline_data_t;


#endif /* HISTORY_HISTORY_H_ */
