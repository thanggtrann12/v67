#ifndef __DSP_H
#define __DSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"
#include "math_helper.h"
#include "log.h"
#include "cmsis_os.h" 
#include "arm_math.h"
#include "arm_const_structs.h"
#include "hr_rr.h"

#define Fs  250
#define BLOCK_SIZE            (uint32_t)50
#define NUM_TAPS              (uint16_t)129	

typedef enum {BPF_0P7_20, BPF_2P0_10, BPF_0P2_2P0, BPF_0P05_1P0, LPF_1P0}filter_type_;

void filter_data(filter_type_ filter_type,  float32_t* input, float32_t* output, uint32_t size_data);
void fft_f32(float32_t* input_data, float32_t* output_data, uint16_t data_size);

float32_t find_agv_k_largest(Peak_ peak[], int n, uint16_t k);
uint16_t find_all_peaks(float32_t arr[], int n, Peak_ peaks[]);
uint16_t find_all_troughs(float32_t arr[], int n, Peak_ troughs[]);

uint16_t find_size_peaks(float32_t arr[], int n);
uint16_t find_size_troughs(float32_t arr[], int n);
float32_t largest(float32_t arr[],uint8_t* loc_max, int n) ;
float32_t findMaxPeak(Peak_ *arr,int size);
float32_t findMaxF(float *arr,int size);
uint16_t findMax(uint16_t *arr, int size);

int8_t check_valid_zcp(float32_t arr[], uint16_t x, uint16_t y, float32_t max_th);
int8_t check_valid_zcn(float32_t arr[], uint16_t x, uint16_t y, float32_t min_th);

#ifdef __cplusplus
}
#endif

#endif
