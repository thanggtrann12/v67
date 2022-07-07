	/****************************************************************************
	%OnSky Inc. Confidential and Proprietary
	%April 15, 2020
	%Algorithm written by Hung Nguyen
	%C code written by Thien Phan
	%
	% Part 1: Receive 6 sensor ADC data. Filter data using FIR LPF. Then Square
	% them. Add all data together.
	%****************************************************************************/

#include "main.h"
#include "hr_rr.h"
#include "dsp.h"
#include "mqtt_interface.h"
#include "Utils.h"
#include "pnp.h"
#include "settings.h"
#include "BedManager.h"

extern float avg_energy_xyz;
extern float32_t avgsq_noise_xyz;
extern float32_t alpha_x1, alpha_y1, alpha_z1, alpha_x2, alpha_y2, alpha_z2, alpha_x3, alpha_y3, alpha_z3;

static uint8_t checking_motions(movements_t *movements, uint8_t num_movs);

//actual_data_t* actual_data_test;
//void test_hr_rr(void)
//{
//	LOG_DEBUGF("+++++++++++test_hr_rr+++++++++++");
//	uint32_t tui = HAL_GetTick();
//	
//	lavie_data_t lavie_data;
//	actual_data_test = (actual_data_t*)raw_datax1;
//	hr_rr(actual_data_test, &lavie_data );
//	LOG_DEBUGF("time to run %dms", HAL_GetTick()-tui);
//}

void hr_rr(actual_data_t* actual_data ,lavie_data_t *lavie_data, uint8_t preHR)
{	
	
	actual_data_t *sq_data = NULL;  // for filter results and square results
 	sq_data = (actual_data_t *)pvPortMalloc(sizeof(actual_data_t)); 
	if(sq_data == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sq_data");
		Error_Handler();
	}
	
	float32_t *sum_data_bpf = NULL;  // for sum bfp results
 	sum_data_bpf = (float32_t *)pvPortMalloc(ACTUAL_DATA_LEN*sizeof(float32_t)); 
	if(sum_data_bpf == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sum_data_bpf");
		Error_Handler();
	}	
	
	filter_data(BPF_0P7_20, actual_data->x1, sq_data->x1, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->y1, sq_data->y1, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->z1, sq_data->z1, ACTUAL_DATA_LEN);
#ifdef PAD_3_SENSOR
	filter_data(BPF_0P7_20, actual_data->x2, sq_data->x2, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->y2, sq_data->y2, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->z2, sq_data->z2, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->x3, sq_data->x3, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->y3, sq_data->y3, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->z3, sq_data->z3, ACTUAL_DATA_LEN);
#endif

	for(uint16_t i = 0; i < ACTUAL_DATA_LEN; i++)
	{
#ifdef PAD_3_SENSOR
		sum_data_bpf[i] = sq_data->x1[i] + sq_data->y1[i] + sq_data->z1[i] + sq_data->x2[i] + sq_data->y2[i] + sq_data->z2[i] + sq_data->x3[i] + sq_data->y3[i] + sq_data->z3[i];
#else
		sum_data_bpf[i] = sq_data->x1[i] + sq_data->y1[i] + sq_data->z1[i];
#endif
		sq_data->x1[i] = sq_data->x1[i]*sq_data->x1[i];
		sq_data->y1[i] = sq_data->y1[i]*sq_data->y1[i];
		sq_data->z1[i] = sq_data->z1[i]*sq_data->z1[i];
#ifdef PAD_3_SENSOR
		sq_data->x2[i] = sq_data->x2[i]*sq_data->x2[i];
		sq_data->y2[i] = sq_data->y2[i]*sq_data->y2[i];
		sq_data->z2[i] = sq_data->z2[i]*sq_data->z2[i];
		sq_data->x3[i] = sq_data->x3[i]*sq_data->x3[i];
		sq_data->y3[i] = sq_data->y3[i]*sq_data->y3[i];
		sq_data->z3[i] = sq_data->z3[i]*sq_data->z3[i];
#endif
	}
	
	movements_t movements[MAX_MOV_40s_NUM] = {0};
	float32_t avg_energy_out = 0, avg_energy_all_out = 0, maxPeak = 0;   //this value will need to be adjusted later
	uint8_t num_moves = search_movements(sum_data_bpf, ACTUAL_DATA_LEN, avg_energy_xyz, movements, &avg_energy_out, &avg_energy_all_out, &maxPeak);
	lavie_data->hr_rr.motions = checking_motions(movements, num_moves);
	lavie_data->hr_rr.movEnergy = avg_energy_all_out;
	lavie_data->hr_rr.maxPeak = maxPeak;
	
	pnp_data_table_t pnp_opti_xyz;
	bool good_found_xyz = false;
	bool flag_lowsig_xyz = false;
	uint16_t opti_loc_xyz = 0;

	if(alpha_x1 == 0 && alpha_y1 == 0 && alpha_z1 == 0 && alpha_y2 == 0 && alpha_x2 == 0 && alpha_z2 == 0 && alpha_x3 == 0 && alpha_y3 == 0 && alpha_z3 == 0)
	{
		alpha_x1 = 1;
		alpha_y1 = 1;
		alpha_z1 = 1;
		alpha_x2 = 1;
		alpha_y2 = 1;
		alpha_z2 = 1;
		alpha_x3 = 1;
		alpha_y3 = 1;
		alpha_z3 = 1;
	}
	LOG_DEBUGF("alpha_x1 %f, alpha_y1 %f, alpha_z1 %f, alpha_x2 %f, alpha_y2 %f, alpha_z2 %f, alpha_x3 %f, alpha_y3 %f, alpha_z3 %f",
		alpha_x1, alpha_y1, alpha_z1, alpha_x2, alpha_y2, alpha_z2, alpha_x3, alpha_y3, alpha_z3);
	// use sum_data_bpf to save memory
	for(uint16_t k = 0; k < ACTUAL_DATA_LEN; k++)
	{
#ifdef PAD_3_SENSOR
		sum_data_bpf[k] = sq_data->x1[k]*alpha_x1 + sq_data->y1[k]*alpha_y1 + sq_data->z1[k]*alpha_z1 + sq_data->x2[k]*alpha_x2 + sq_data->y2[k]*alpha_y2
		+ sq_data->z2[k]*alpha_z2 + sq_data->x3[k]*alpha_x3 + sq_data->y3[k]*alpha_y3 + sq_data->z3[k]*alpha_z3;
#else
		sum_data_bpf[k] = sq_data->x1[k]*alpha_x1 + sq_data->y1[k]*alpha_y1 + sq_data->z1[k]*alpha_z1;
#endif
	}
	
	pnp(sum_data_bpf, ACTUAL_DATA_LEN, avgsq_noise_xyz, true, &pnp_opti_xyz, &opti_loc_xyz, &good_found_xyz,  &flag_lowsig_xyz);
	LOG_DEBUGF("sq_xyz- good: %d, lowsig : %d, location: %d, np_ratio: %f\r\n",
	good_found_xyz, flag_lowsig_xyz, opti_loc_xyz, pnp_opti_xyz.np_ratio);
	
	
    float32_t th_adapt_xyz = pnp_opti_xyz.th_adapt;   //optimum th_adapt
	LOG_DEBUGF("th_adapt_xyz %f", th_adapt_xyz);
	//combined waveform with multiple sensors
    static uint16_t  timegap_in = 100;
	uint16_t timegap_out = 0;    //default value for timegap (= max_timegap), assuming 
                         //avg R-R time (np_window) is about 250 samples (60BPM).
                         //each new data block will use timegap from
                         //previous data block.	
	uint16_t heart_beats = 0;
	bool heart_lowsig = false;

	
//	LOG_DEBUGF("th_adapt_xyz : %f, timegap_in : %d",th_adapt_xyz, timegap_in);
	uint8_t Heart_Rate = 0;
	float HRV = 0;
	hr_detect(sum_data_bpf, ACTUAL_DATA_LEN, th_adapt_xyz, movements, num_moves, true, timegap_in, avgsq_noise_xyz, &heart_lowsig, &timegap_out, &heart_beats, &Heart_Rate, &HRV);
	LOG_DEBUGF("heart_beats : %d, Actual_hr : %d, HRV : %f, lowsig: %d, timegap_out: %d",heart_beats, Heart_Rate, HRV, heart_lowsig, timegap_out);
	if(Heart_Rate < 40 || Heart_Rate > 200 || heart_lowsig == true || flag_lowsig_xyz == true)
	{
		LOG_DEBUGF("HR Error case");
		lavie_data->hr_rr.actual_hr = ERROR_CASE;
		lavie_data->hr_rr.hrv = 0;		
	}
	else
	{			
		if(num_moves <= 20)
		{
			if(good_found_xyz == true || ( (preHR < 40 || preHR > 120) && BedManagerPredictUserNormalModeGet() ) )
			{
				LOG_DEBUGF("good_found_xz = 1");
				lavie_data->hr_rr.actual_hr = Heart_Rate;
				lavie_data->hr_rr.hrv = HRV;
				if(timegap_out > 40 && timegap_out < 150)
				{
					LOG_DEBUGF("timegap_out %d", timegap_out);
					timegap_in = timegap_out;
				}
			}
			else
			{
				LOG_DEBUGF("good_found_xz = 0");
				lavie_data->hr_rr.actual_hr = WEAK_CASE; // data weak
				lavie_data->hr_rr.hrv = 0;			
			}
		}
		else
		{
			LOG_DEBUGF("num movs %d", num_moves);
			lavie_data->hr_rr.actual_hr = PREVIOUS_CASE;
			lavie_data->hr_rr.hrv = 0;
		}			
	}
	lavie_data->hr_rr.pnp_rt = roundf(pnp_opti_xyz.np_ratio);
	 
	// free memory
	vPortFree(sq_data); // free pointer sq_data
	vPortFree(sum_data_bpf); // free pointer sum_data_bpf		
	
}

static uint8_t checking_motions(movements_t *movements, uint8_t num_movs)
{
	if(num_movs == 0){
		return 0;
	} 

	uint8_t motions = 0;
	uint8_t sum_mov = 1;
	
	for(uint16_t i = 1; i < (num_movs); i++)
	{
		if(movements[i].loc_start == movements[i-1].loc_end)
		{
			sum_mov++;
		}
		else
		{
			if(sum_mov > 0 && sum_mov <= 4)
			{
				motions++;
			}
			else if(sum_mov > 4)
			{
				motions += (uint8_t)roundf((float)sum_mov/4);
			}				
			
			sum_mov = 1;
		}
	}
	if(sum_mov >= 1)
	{
		if(sum_mov > 0 && sum_mov <= 5)
		{
			motions++;
		}
		else if(sum_mov > 4)
		{
			motions += (uint8_t)roundf((float)sum_mov/4);
		}		
	}

	return motions;
}
