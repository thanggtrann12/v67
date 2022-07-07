	/***************************************************************************
	%OnSky Inc. Confidential and Proprietary
	%Algorithm written by Hung Nguyen
	%C code written by Thien Phan
	%September 1, 2020
	%
	%PnP Algorithm:
	%Algorithm to find quality of the ADC signals
	%Pulses vs. No-Pulses (PnP Value: signal quality for heart beats)
	%(invention of Mr. Hung Nguyen - September 2020)
	%***************************************************************************

	%****************************************************************************
	%These constants are based on 40 seconds of ADC data and for adults
	%with heart beat rate from 40BPM to 120BPM while lying on bed. ADC sampling
	%rate is 250Hz (0.004s), we use 10K samples (40s) for this algorithm.
	%These constants may change for other applications (e.g. newborn babies have
	%faster heart beat rates).
	%****************************************************************************/
#include <stdlib.h>
#include "main.h"
#include "hr_rr.h"
#include "dsp.h"
#include "Utils.h"
#include "pnp.h"
#include "Rtos.h"
#include "LcdCtrl.h"
#include "LcdUpdate.h"
#include "settings.h"
#include "BedManager.h"

dev_params_t dev_params = {0};

static float th_adapt_xyz = 0;
static float Pnp_bg = 0;
float alpha_x1 = 0,  alpha_y1 = 0, alpha_z1 = 0, alpha_x2 = 0, alpha_y2 = 0, alpha_z2 = 0, alpha_x3 = 0, alpha_y3 = 0, alpha_z3 = 0;
float pnp_bg_index = 0;
extern actual_data_t  pnp_backgound_data;
extern float avgsq_noise_x, avgsq_noise_y, avgsq_noise_z, avgsq_noise_xyz;
extern float avg_energy_xyz;

static const uint16_t th_timegap = 100;   //unit: samples
static const uint16_t one_pulse = 20;     //duration of one pulse in term of samples
static const uint16_t min_np = 100;       //Min number of samples within no-pulse window (np), based
										//on 120BPM
static const uint16_t max_np = 385;       //Max number of samples within no-pulse window (np), based
										//on 40BPM
//static const uint16_t step_back = 100;      //After find optimum pnp, step back 5 times if possible
										//this is to have guard band for short pulses
//static const float32_t stepback_pnp_limit = 0.5;   //stepback limit for pnp value
static const float32_t min_pnp_rt = 2.5;      //Optimum pnp value (result) must be = or > min_pnp
static const float32_t min_pnp_bg = 2.8;
//const float32_t th_noise = 50;      //do not consider peaks below this threshold, it is noise
//static float32_t adapt_step = 100; 	//Increment th_adapt to create table pnp_data

void pnp(float32_t * square_data, uint16_t data_size, float32_t avgsq_noise, bool xz_add, pnp_data_table_t * pnp_opti,
	uint16_t *opti_loc, bool *good_found,  bool *flag_lowsig)
{
	uint16_t min_count_p = 20; //for 40s of data, at 40BPM, there are 27 beats, so we use 20 as min number of beats per 40s.
	if(data_size == ACTUAL_DATA_LEN)
	{
		min_count_p = 20;
	}
	else
	{
		min_count_p = 3;
	}
	/*Find all peaks of signal*/
	uint16_t size_all_peaks = find_size_peaks(square_data, data_size);	
//	LOG_DEBUGF("size_all_peaks: %d\r\n", size_all_peaks);

	Peak_ *all_peaks = NULL;
	LcdCtrlLock();
	if(size_all_peaks > 0){
		all_peaks = (Peak_*)pvPortMalloc(size_all_peaks * sizeof(Peak_));
	}
	LcdCtrlUnlock();
	if(all_peaks == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :all_peaks");
		return;
	}		
	
	find_all_peaks(square_data, data_size, all_peaks);
	
	//Find average values of all peaks that above th_noise
	float32_t sum_pks = 0;
	uint16_t count_pks = 0;
	float32_t max_pks = 0;

	uint16_t size_peaks_over_noise = 0;
	avgsq_noise *= 2;
	for(uint16_t i = 0; i < size_all_peaks; i++)
	{
		if(all_peaks[i].amp > avgsq_noise)
		{
			size_peaks_over_noise++;				
		}
	}
	
	Peak_ *peaks_over_noise = NULL;
	LcdCtrlLock();
	if(size_peaks_over_noise > 0){
		peaks_over_noise = (Peak_*)pvPortMalloc(size_peaks_over_noise * sizeof(Peak_));
	}
	if(peaks_over_noise == NULL){
		vPortFree(all_peaks);
		return;
	}
	LcdCtrlUnlock();

	for(uint16_t i = 0; i < size_all_peaks; i++)
	{
		if(all_peaks[i].amp > avgsq_noise)
		{		
			peaks_over_noise[count_pks].amp = all_peaks[i].amp;
			peaks_over_noise[count_pks].loc = all_peaks[i].loc;
			sum_pks = sum_pks + all_peaks[i].amp;
			count_pks++;		
			if(all_peaks[i].amp > max_pks)
			{
				max_pks = all_peaks[i].amp;
			}
		}			
	}

	vPortFree(all_peaks); // free pointer all_peaks
	
//	LOG_DEBUGF("size_peaks_over_noise: %d\r\n", size_peaks_over_noise);

	float32_t avg_pks = sum_pks / (float32_t)count_pks;     //take average
//	LOG_DEBUGF("avg_pks %f", avg_pks);
	*flag_lowsig = false;
	*good_found = 0;
	if(count_pks < min_count_p)       //number of pulses too small ,less than 20 (in 10K samples)
	{																	
		 *flag_lowsig = true;
			vPortFree(peaks_over_noise); // free pointer peaks_over_noise
			LOG_DEBUGF("number of pulses too small");
		 return;
	}
	
	//Find average of Pulses and average of No-Pulses
	float32_t th_adapt   = (avg_pks * (float32_t)0.25);  //initial value
	uint16_t index = 0;
	
	float32_t limit_search = 0.0;
	

	if (xz_add == true)
	{
		 limit_search = avg_pks * (float32_t)3.0;    //run more searches if x and z are combined
	}
	else
	{
		 limit_search = avg_pks * (float32_t)2.5;    //run less searches		
	}
//	LOG_DEBUGF("limit search %f", limit_search);
//	if((th_adapt_xyz!=0) && (limit_search > (float32_t)1.25*th_adapt_xyz) && (th_adapt_xyz > (float32_t)0.3*limit_search))
//	{
//		limit_search = (float32_t)1.25*th_adapt_xyz;
//	}
//	LOG_DEBUGF("th_adapt %f, max_pks :%f, limit search :%f", th_adapt, max_pks, limit_search);
/************************************************************
%Evaluate performance of waveform (square_data) with increasing value
%of th_adapt, then put the results in the table pnp_data. 
%th_adapt starts from avg_pks * 0.4. 
%
%************************************************************/
	uint16_t first_pk = 0;
	uint16_t last_pk = 0;
	uint16_t count_p = 0;
	uint16_t count_np = 0;
	uint16_t sum_p = 0;
	uint16_t sum_np = 0;
	uint16_t np_window = 0;
	uint16_t sum_diff_np = 0;
	uint16_t lastnp_window = 0;
	uint16_t time_gap = 0;	
	
	
	const uint16_t  MAX_SIZE_PNP_TABLE	= 250;
	const uint16_t num_search = MAX_SIZE_PNP_TABLE - 1;
	pnp_data_table_t *pnp_data = NULL;
	LcdCtrlLock();
	pnp_data = (pnp_data_table_t*)pvPortMalloc(MAX_SIZE_PNP_TABLE * sizeof(pnp_data_table_t));
	LcdCtrlUnlock();
	if(pnp_data == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :pnp_data");
		Error_Handler();
	}
	memset(pnp_data, 0, MAX_SIZE_PNP_TABLE * sizeof(pnp_data_table_t));
	
	float32_t adapt_step;
	if(max_pks < limit_search)
	{
		adapt_step = (max_pks - th_adapt)/(float32_t)num_search;
	}
	else
	{
		adapt_step = (limit_search - th_adapt)/(float32_t)num_search;
	}		
	
//	LOG_DEBUGF("adapt_step %f", adapt_step);
	
	while( (th_adapt < max_pks)&&(th_adapt < limit_search)&&(index < MAX_SIZE_PNP_TABLE) )
	{
		th_adapt = th_adapt + (float32_t)adapt_step;
		first_pk = 0;
		last_pk = 0;
		count_p = 0;
		count_np = 0;
		sum_p = 0;
		sum_np = 0;
		np_window = 0;
		sum_diff_np = 0;
		lastnp_window = 0;			
			
		uint16_t k = 0;
		for(uint16_t i = 0; i < size_peaks_over_noise; i++)
		{
			time_gap = 1000;
			if(peaks_over_noise[i].amp > th_adapt)
			{
				k = i + 1;
				while(k < (size_peaks_over_noise-1)) //look forward to find peak above th_noise
				{
					if (peaks_over_noise[k].amp > th_adapt)
					{
						 time_gap = peaks_over_noise[k].loc - peaks_over_noise[i].loc;
						 break;
					}
					k = k +1;
				}
				if ((first_pk == 0) && (time_gap < th_timegap)) //case 1: this pulse is first one in pulse-group
				{
					first_pk = peaks_over_noise[i].loc;
					if (last_pk != 0)
					{
						 np_window = first_pk - last_pk;

						 sum_np = sum_np + np_window;
						 count_np = count_np + 1; 
						 if (count_np >= 2)
						 {
								 uint16_t diff_np = abs(np_window - lastnp_window);
								 sum_diff_np = sum_diff_np + diff_np;
						 } 
						 lastnp_window = np_window;
					 } 						
				}
				else if((first_pk != 0) && (time_gap >= th_timegap)) //case 2: last pulse in pulse-group
				{
					last_pk = peaks_over_noise[i].loc;
					sum_p = sum_p + (last_pk - first_pk);
					count_p = count_p + 1;
					first_pk = 0;						
				}
				else if((first_pk == 0) &&(time_gap >= th_timegap))  //case 3: first and only pulse in pulse-group
				{
					first_pk = peaks_over_noise[i].loc;
					if (last_pk != 0)
					{
						 np_window = first_pk - last_pk;
						 
						 sum_np = sum_np + np_window;
						 count_np = count_np + 1; 
						 if (count_np >= 2)
						 {
								 uint16_t diff_np = abs(np_window - lastnp_window);
								 sum_diff_np = sum_diff_np + diff_np;
						 } 
						 lastnp_window = np_window;
					}
					last_pk = peaks_over_noise[i].loc;

					first_pk = 0;
					sum_p = sum_p + one_pulse;
					count_p = count_p + 1;						
				}
			} // end of if
		} // end of for
		
		float32_t avg_p = 0, avg_np = 0, pnp_value = 0, avg_diff_np = 0, np_ratio =0;
		if((count_p > 0) && (count_np > 1)) 
		{
			avg_p = (float32_t)sum_p / count_p;    //average value of all pulse periods
			//printf("sum_p :%d , count_p : %d\r\n", sum_p, count_p);			
			avg_np = (float32_t)sum_np / count_np; //average value of all no-pulse periods
			pnp_value = (float32_t)avg_np / (avg_np + avg_p); //formula for Pulse and No-Pulse ratio
			avg_diff_np = (float32_t)sum_diff_np / (count_np - 1);
			if(avg_diff_np != 0)
			{
				np_ratio = (float32_t)avg_np / avg_diff_np;
			}
			else
			{
				np_ratio = 0;				
			}
		}			
		pnp_data[index].index = index;
		pnp_data[index].th_adapt = th_adapt;
		pnp_data[index].flag_lowsig = *flag_lowsig;
		pnp_data[index].avg_pks = avg_pks;
		pnp_data[index].avg_p = avg_p;
		pnp_data[index].avg_np = avg_np;
		pnp_data[index].count_p = count_p;
		pnp_data[index].count_np = count_np;
		pnp_data[index].pnp_value = pnp_value;
		pnp_data[index].avg_diff_np = avg_diff_np;
		pnp_data[index].np_ratio = np_ratio;
//			printf("%d, %f, %d, %f, %f, %f, %d, %d, %f, %f, %f\r\n", pnp_data[index].index, pnp_data[index].th_adapt, pnp_data[index].flag_lowsig, pnp_data[index].avg_pks, pnp_data[index].avg_p, pnp_data[index].avg_np
//			, pnp_data[index].count_p, pnp_data[index].count_np, pnp_data[index].pnp_value, pnp_data[index].avg_diff_np, pnp_data[index].np_ratio);
		index = index + 1;

//		else //else if (*flag_lowsig == 1) , signal is too small
//		{
//			
//		}
		
		//count_np < min_count_p, means there are only few peaks left at end of table
		//However, at beginning of table count_p < min_count_p, but avg_np < avg_p				
    if ((count_p < min_count_p) && (avg_np > avg_p) )
		{
			break;
		}
	}
	vPortFree(peaks_over_noise); // free pointer peaks_over_noise
/*************************************************************************************
%Search table pnp_data that contains the performance values of square_data waveform,
%Looking for optimum row that has highest value of np_ratio to determine the optimum 
%value of th_adapt.
%Result will be inside pnp_opti vector.
%**********************************************************************************/
	float32_t max_np_ratio = 0;
	uint16_t opti_found = 0;
//	uint16_t new_opti_loc = 0;
	*good_found = 0;
	*opti_loc = 0;	
	LOG_DEBUGF("pnp index :%d \r\n", index);
	for(uint16_t i = 0; i < index; i++)
	{
		if( (pnp_data[i].avg_np > (pnp_data[i].avg_p*2)) && (pnp_data[i].np_ratio >= min_pnp_rt)
			&& (pnp_data[i].avg_np > min_np) && (pnp_data[i].avg_np < max_np))
		{
			opti_found = 1;
			if(pnp_data[i].np_ratio >= max_np_ratio)
			{
				max_np_ratio = pnp_data[i].np_ratio;
				*opti_loc = i;				
			}
		}
	}
//	LOG_DEBUGF("opti_found %d, opti_loc %d, np_ratio %f\r\n",opti_found, *opti_loc, max_np_ratio);
	if(opti_found == 1)
	{
		memcpy(pnp_opti, &pnp_data[*opti_loc], sizeof(pnp_data_table_t));
		*good_found = true;
	}
	else
	{
		memset(pnp_opti, 0, sizeof(pnp_data_table_t));
		*opti_loc = 0;
		*good_found = 0;
	}
	
	vPortFree(pnp_data);
//	LOG_DEBUGF("*opti_loc :%d \r\n", *opti_loc);
//	LOG_DEBUGF("np_ratio :%f \r\n", pnp_opti->np_ratio);
}


static uint16_t num_execute_pnp_pg = 0;
void pnp_background(float32_t * square_data, uint16_t data_size, float32_t avgsq_noise, bool xz_add, pnp_data_table_t * pnp_opti,
	float *avgPks, bool *good_found,  bool *flag_lowsig)
{
	uint16_t min_count_p = 20; //for 40s of data, at 40BPM, there are 27 beats, so we use 20 as min number of beats per 40s.
	if(data_size == ACTUAL_DATA_LEN)
	{
		min_count_p = 20;
	}
	else
	{
		min_count_p = 3;
	}
	/*Find all peaks of signal*/
	uint16_t size_all_peaks = find_size_peaks(square_data, data_size);	
//	LOG_DEBUGF("size_all_peaks: %d\r\n", size_all_peaks);

	Peak_ *all_peaks = NULL;
	LcdCtrlLock();
	if(size_all_peaks > 0){
		all_peaks = (Peak_*)pvPortMalloc(size_all_peaks * sizeof(Peak_));
	}
	LcdCtrlUnlock();
	if(all_peaks == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :all_peaks");
		return;
	}		
	
	find_all_peaks(square_data, data_size, all_peaks);
	
	//Find average values of all peaks that above th_noise
	float32_t sum_pks = 0;
	uint16_t count_pks = 0;
	float32_t max_pks = 0;

	uint16_t size_peaks_over_noise = 0;
	avgsq_noise *= 2;
	for(uint16_t i = 0; i < size_all_peaks; i++)
	{
		if(all_peaks[i].amp > avgsq_noise)
		{
			size_peaks_over_noise++;				
		}
	}

	Peak_ *peaks_over_noise = NULL;
	LcdCtrlLock();
	if(size_peaks_over_noise > 0){
		peaks_over_noise = (Peak_*)pvPortMalloc(size_peaks_over_noise * sizeof(Peak_));
	}
	if(peaks_over_noise == NULL){
		vPortFree(all_peaks);
		return;
	}	
	LcdCtrlUnlock();

	for(uint16_t i = 0; i < size_all_peaks; i++)
	{
		if(all_peaks[i].amp > avgsq_noise)
		{		
			peaks_over_noise[count_pks].amp = all_peaks[i].amp;
			peaks_over_noise[count_pks].loc = all_peaks[i].loc;
			sum_pks = sum_pks + all_peaks[i].amp;
			count_pks++;		
			if(all_peaks[i].amp > max_pks)
			{
				max_pks = all_peaks[i].amp;
			}
		}			
	}
	vPortFree(all_peaks); // free pointer all_peaks
	
//	LOG_DEBUGF("size_peaks_over_noise: %d\r\n", size_peaks_over_noise);

	float32_t avg_pks = sum_pks / (float32_t)count_pks;     //take average
	*avgPks = avg_pks;
//	LOG_DEBUGF("avg_pks %f", avg_pks);
	*flag_lowsig = false;
	*good_found = 0;
	if(count_pks < min_count_p)       //number of pulses too small ,less than 20 (in 10K samples)
	{																	
		 *flag_lowsig = true;
			vPortFree(peaks_over_noise); // free pointer peaks_over_noise
			LOG_DEBUGF("number of pulses too small");
		 return;
	}
	
	//Find average of Pulses and average of No-Pulses
	float32_t th_adapt   = (avg_pks * (float32_t)0.4);  //initial value
	uint16_t index = 0;
	
	float32_t limit_search = 0.0;
	if (xz_add == true)
	{
		 limit_search = avg_pks * (float32_t)3.0;    //run more searches if x and z are combined
	}
	else
	{
		 limit_search = avg_pks * (float32_t)2.5;    //run less searches		
	}
//	LOG_DEBUGF("th_adapt %f, max_pks :%f, limit search :%f \r\n", th_adapt, max_pks, limit_search);
/************************************************************
%Evaluate performance of waveform (square_data) with increasing value
%of th_adapt, then put the results in the table pnp_data. 
%th_adapt starts from avg_pks * 0.4. 
%
%************************************************************/
	uint16_t first_pk = 0;
	uint16_t last_pk = 0;
	uint16_t count_p = 0;
	uint16_t count_np = 0;
	uint16_t sum_p = 0;
	uint16_t sum_np = 0;
	uint16_t np_window = 0;
	uint16_t sum_diff_np = 0;
	uint16_t lastnp_window = 0;
	uint16_t time_gap = 0;	
	
	uint16_t  MAX_SIZE_PNP_TABLE	= 800;
	if(num_execute_pnp_pg <= 21)
	{
		num_execute_pnp_pg++;
	}
	else
	{
		 MAX_SIZE_PNP_TABLE	= 8000;
	}
	LOG_DEBUGF("MAX_SIZE_PNP_TABLE %d", MAX_SIZE_PNP_TABLE);
	
	float32_t adapt_step = 100;
	if(max_pks < limit_search)
	{
		adapt_step = (max_pks - th_adapt)/(float32_t)(MAX_SIZE_PNP_TABLE-1);
	}
	else
	{
		adapt_step = (limit_search - th_adapt)/(float32_t)(MAX_SIZE_PNP_TABLE-1);
	}	

	pnp_data_table_t *pnp_data = NULL;
	LcdCtrlLock();
	pnp_data = (pnp_data_table_t*)pvPortMalloc(MAX_SIZE_PNP_TABLE * sizeof(pnp_data_table_t));
	LcdCtrlUnlock();
	if(pnp_data == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :pnp_data");
		Error_Handler();
	}	
	memset(pnp_data, 0, MAX_SIZE_PNP_TABLE * sizeof(pnp_data_table_t));
//	LOG_DEBUGF("adapt_step %f", adapt_step);
	
	while( (th_adapt < max_pks)&&(th_adapt < limit_search)&&(index < MAX_SIZE_PNP_TABLE) )
	{
		th_adapt = th_adapt + (float32_t)adapt_step;
		first_pk = 0;
		last_pk = 0;
		count_p = 0;
		count_np = 0;
		sum_p = 0;
		sum_np = 0;
		np_window = 0;
		sum_diff_np = 0;
		lastnp_window = 0;			
			
		uint16_t k = 0;
		for(uint16_t i = 0; i < size_peaks_over_noise; i++)
		{
			time_gap = 1000;
			if(peaks_over_noise[i].amp > th_adapt)
			{
				k = i + 1;
				while(k < (size_peaks_over_noise-1)) //look forward to find peak above th_noise
				{
					if (peaks_over_noise[k].amp > th_adapt)
					{
						 time_gap = peaks_over_noise[k].loc - peaks_over_noise[i].loc;
						 break;
					}
					k = k +1;
				}
				if ((first_pk == 0) && (time_gap < th_timegap)) //case 1: this pulse is first one in pulse-group
				{
					first_pk = peaks_over_noise[i].loc;
					if (last_pk != 0)
					{
						 np_window = first_pk - last_pk;

						 sum_np = sum_np + np_window;
						 count_np = count_np + 1; 
						 if (count_np >= 2)
						 {
								 uint16_t diff_np = abs(np_window - lastnp_window);
								 sum_diff_np = sum_diff_np + diff_np;
						 } 
						 lastnp_window = np_window;
					 } 						
				}
				else if((first_pk != 0) && (time_gap >= th_timegap)) //case 2: last pulse in pulse-group
				{
					last_pk = peaks_over_noise[i].loc;
					sum_p = sum_p + (last_pk - first_pk);
					count_p = count_p + 1;
					first_pk = 0;						
				}
				else if((first_pk == 0) &&(time_gap >= th_timegap))  //case 3: first and only pulse in pulse-group
				{
					first_pk = peaks_over_noise[i].loc;
					if (last_pk != 0)
					{
						 np_window = first_pk - last_pk;
						 
						 sum_np = sum_np + np_window;
						 count_np = count_np + 1; 
						 if (count_np >= 2)
						 {
								 uint16_t diff_np = abs(np_window - lastnp_window);
								 sum_diff_np = sum_diff_np + diff_np;
						 } 
						 lastnp_window = np_window;
					}
					last_pk = peaks_over_noise[i].loc;

					first_pk = 0;
					sum_p = sum_p + one_pulse;
					count_p = count_p + 1;						
				}
			} // end of if
		} // end of for
		
		float32_t avg_p = 0, avg_np = 0, pnp_value = 0, avg_diff_np = 0, np_ratio =0;
		if((count_p > 0) && (count_np > 1)) 
		{
			avg_p = (float32_t)sum_p / count_p;    //average value of all pulse periods
			//printf("sum_p :%d , count_p : %d\r\n", sum_p, count_p);			
			avg_np = (float32_t)sum_np / count_np; //average value of all no-pulse periods
			pnp_value = (float32_t)avg_np / (avg_np + avg_p); //formula for Pulse and No-Pulse ratio
			avg_diff_np = (float32_t)sum_diff_np / (count_np - 1);
			if(avg_diff_np != 0)
			{
				np_ratio = (float32_t)avg_np / avg_diff_np;
			}
			else
			{
				np_ratio = 0;				
			}
		}			
		pnp_data[index].index = index;
		pnp_data[index].th_adapt = th_adapt;
		pnp_data[index].flag_lowsig = *flag_lowsig;
		pnp_data[index].avg_pks = avg_pks;
		pnp_data[index].avg_p = avg_p;
		pnp_data[index].avg_np = avg_np;
		pnp_data[index].count_p = count_p;
		pnp_data[index].count_np = count_np;
		pnp_data[index].pnp_value = pnp_value;
		pnp_data[index].avg_diff_np = avg_diff_np;
		pnp_data[index].np_ratio = np_ratio;
//			printf("%d, %f, %d, %f, %f, %f, %d, %d, %f, %f, %f\r\n", pnp_data[index].index, pnp_data[index].th_adapt, pnp_data[index].flag_lowsig, pnp_data[index].avg_pks, pnp_data[index].avg_p, pnp_data[index].avg_np
//			, pnp_data[index].count_p, pnp_data[index].count_np, pnp_data[index].pnp_value, pnp_data[index].avg_diff_np, pnp_data[index].np_ratio);
		index = index + 1;

//		else //else if (*flag_lowsig == 1) , signal is too small
//		{
//			
//		}
		
		//count_np < min_count_p, means there are only few peaks left at end of table
		//However, at beginning of table count_p < min_count_p, but avg_np < avg_p				
    if ((count_p < min_count_p) && (avg_np > avg_p) )
		{
			break;
		}
	}
	vPortFree(peaks_over_noise); // free pointer peaks_over_noise
/*************************************************************************************
%Search table pnp_data that contains the performance values of square_data waveform,
%Looking for optimum row that has highest value of np_ratio to determine the optimum 
%value of th_adapt.
%Result will be inside pnp_opti vector.
%**********************************************************************************/
	float32_t max_np_ratio = 0;
	uint16_t opti_found_loc = 0;
//	uint16_t new_opti_loc = 0;
	*good_found = 0;
//	LOG_DEBUGF("index :%d \r\n", index);
	for(uint16_t i = 0; i < index; i++)
	{
		if( (pnp_data[i].avg_np > (pnp_data[i].avg_p*2)) && (pnp_data[i].np_ratio >= min_pnp_bg)
			&& (pnp_data[i].avg_np > min_np) && (pnp_data[i].avg_np < max_np))
		{
			if(pnp_data[i].np_ratio >= max_np_ratio)
			{
				opti_found_loc = i;
				max_np_ratio = pnp_data[i].np_ratio;
			}
		}
	}
//	LOG_DEBUGF("opti_found %d, opti_loc %d, np_ratio %f\r\n",opti_found, *opti_loc, max_np_ratio);
	if(opti_found_loc)
	{
		LOG_DEBUGF("PnpBg good");
		memcpy(pnp_opti, &pnp_data[opti_found_loc], sizeof(pnp_data_table_t));
		*good_found = true;
	}
	else
	{
		LOG_DEBUGF("PnpBg bad");
		memset(pnp_opti, 0, sizeof(pnp_data_table_t));
		*good_found = 0;
	}
	vPortFree(pnp_data);
//	LOG_DEBUGF("*opti_loc :%d \r\n", *opti_loc);
//	LOG_DEBUGF("np_ratio :%f \r\n", pnp_opti->np_ratio);

}

void PnpBGTaskFn(void *argument)
{
	osThreadSuspend(pnpBGTaskHandle);
	while(1)
	{
		LOG_DEBUGF("Start find alpha pnp");
	
		actual_data_t *sq_data = NULL;  // for filter results and square results
		LcdCtrlLock();
		sq_data = (actual_data_t *)pvPortMalloc(sizeof(actual_data_t)); 
		LcdCtrlUnlock();
		if(sq_data == NULL)
		{
			LOG_DEBUGF("Could not allocate memory :sq_data");
			Error_Handler();
		}
		
		float32_t *sum_data_bpf = NULL;  // for sum bfp results
		LcdCtrlLock();
		sum_data_bpf = (float32_t *)pvPortMalloc(ACTUAL_DATA_LEN*sizeof(float32_t)); 
		LcdCtrlUnlock();
		if(sum_data_bpf == NULL)
		{
			LOG_DEBUGF("Could not allocate memory :sum_data_bpf");
			Error_Handler();
		}	
		
		filter_data(BPF_0P7_20, pnp_backgound_data.x1, sq_data->x1, ACTUAL_DATA_LEN);
		filter_data(BPF_0P7_20, pnp_backgound_data.y1, sq_data->y1, ACTUAL_DATA_LEN);
		filter_data(BPF_0P7_20, pnp_backgound_data.z1, sq_data->z1, ACTUAL_DATA_LEN);
#ifdef PAD_3_SENSOR
		filter_data(BPF_0P7_20, pnp_backgound_data.x2, sq_data->x2, ACTUAL_DATA_LEN);
		filter_data(BPF_0P7_20, pnp_backgound_data.y2, sq_data->y2, ACTUAL_DATA_LEN);
		filter_data(BPF_0P7_20, pnp_backgound_data.z2, sq_data->z2, ACTUAL_DATA_LEN);
		filter_data(BPF_0P7_20, pnp_backgound_data.x3, sq_data->x3, ACTUAL_DATA_LEN);
		filter_data(BPF_0P7_20, pnp_backgound_data.y3, sq_data->y3, ACTUAL_DATA_LEN);
		filter_data(BPF_0P7_20, pnp_backgound_data.z3, sq_data->z3, ACTUAL_DATA_LEN);	
#endif
		pnp_bg_index = 0;

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
		float32_t avg_energy_out = 0;   //this value will need to be adjusted later
		uint8_t num_moves = search_movements(sum_data_bpf, ACTUAL_DATA_LEN, avg_energy_xyz, movements, &avg_energy_out, NULL, NULL);

		if(num_moves == 0)
		{	
			pnp_data_table_t pnp_opti_x1 = {0};
			pnp_data_table_t pnp_opti_y1 = {0};
			pnp_data_table_t pnp_opti_z1 = {0};
#ifdef PAD_3_SENSOR
			pnp_data_table_t pnp_opti_x2 = {0};
			pnp_data_table_t pnp_opti_y2 = {0};
			pnp_data_table_t pnp_opti_z2 = {0};
			pnp_data_table_t pnp_opti_x3 = {0};
			pnp_data_table_t pnp_opti_y3 = {0};
			pnp_data_table_t pnp_opti_z3 = {0};
#endif
	//			pnp_data_table_t pnp_opti_xz;	
			bool good_found_x1 = false, good_found_y1 = false, good_found_z1 = false;
			bool flag_lowsig_x1 = false, flag_lowsig_y1 = false, flag_lowsig_z1 = false;
			float avgPks_x1 = 0, avgPks_y1 = 0, avgPks_z1 = 0;
			float alpha_x1_temp = 0, alpha_y1_temp = 0, alpha_z1_temp = 0;
#ifdef PAD_3_SENSOR
			bool good_found_x2 = false, good_found_y2 = false, good_found_z2 = false, good_found_x3 = false, good_found_y3 = false, good_found_z3 = false;//, good_found_xz = false;
			bool flag_lowsig_x2 = false , flag_lowsig_y2 = false, flag_lowsig_z2 = false, flag_lowsig_x3 = false, flag_lowsig_y3 = false, flag_lowsig_z3 = false;//, flag_lowsig_xz = false;
			float avgPks_x2 = 0, avgPks_y2 = 0, avgPks_z2 = 0, avgPks_x3 = 0, avgPks_y3 = 0, avgPks_z3 = 0;
			float alpha_x2_temp = 0, alpha_y2_temp = 0, alpha_z2_temp = 0, alpha_x3_temp = 0, alpha_y3_temp = 0, alpha_z3_temp = 0;
#endif

			pnp_background(sq_data->x1, ACTUAL_DATA_LEN, avgsq_noise_x, false, &pnp_opti_x1, &avgPks_x1, &good_found_x1,  &flag_lowsig_x1);
			pnp_background(sq_data->y1, ACTUAL_DATA_LEN, avgsq_noise_y, false, &pnp_opti_y1, &avgPks_y1, &good_found_y1,  &flag_lowsig_y1);
			pnp_background(sq_data->z1, ACTUAL_DATA_LEN, avgsq_noise_z, false, &pnp_opti_z1, &avgPks_z1, &good_found_z1,  &flag_lowsig_z1);
#ifdef PAD_3_SENSOR
			pnp_background(sq_data->x2, ACTUAL_DATA_LEN, avgsq_noise_x, false, &pnp_opti_x2, &avgPks_x2, &good_found_x2,  &flag_lowsig_x2);
			pnp_background(sq_data->y2, ACTUAL_DATA_LEN, avgsq_noise_y, false, &pnp_opti_y2, &avgPks_y2, &good_found_y2,  &flag_lowsig_y2);
			pnp_background(sq_data->z2, ACTUAL_DATA_LEN, avgsq_noise_z, false, &pnp_opti_z2, &avgPks_z2, &good_found_z2,  &flag_lowsig_z2);
			pnp_background(sq_data->x3, ACTUAL_DATA_LEN, avgsq_noise_x, false, &pnp_opti_x3, &avgPks_x3, &good_found_x3,  &flag_lowsig_x3);
			pnp_background(sq_data->y3, ACTUAL_DATA_LEN, avgsq_noise_y, false, &pnp_opti_y3, &avgPks_y3, &good_found_y3,  &flag_lowsig_y3);
			pnp_background(sq_data->z3, ACTUAL_DATA_LEN, avgsq_noise_z, false, &pnp_opti_z3, &avgPks_z3, &good_found_z3,  &flag_lowsig_z3);
#endif
#ifdef PAD_3_SENSOR
			dev_params.PnpBgRatio1.X = round(pnp_opti_x1.np_ratio);
			dev_params.PnpBgRatio1.Y = round(pnp_opti_y1.np_ratio);
			dev_params.PnpBgRatio1.Z = round(pnp_opti_z1.np_ratio);
			dev_params.PnpBgRatio2.X = round(pnp_opti_x2.np_ratio);
			dev_params.PnpBgRatio2.Y = round(pnp_opti_y2.np_ratio);
			dev_params.PnpBgRatio2.Z = round(pnp_opti_z2.np_ratio);
			dev_params.PnpBgRatio3.X = round(pnp_opti_x3.np_ratio);
			dev_params.PnpBgRatio3.Y = round(pnp_opti_y3.np_ratio);
			dev_params.PnpBgRatio3.Z = round(pnp_opti_z3.np_ratio);
			dev_params.AvgPksOpti1.X = round(avgPks_x1);
			dev_params.AvgPksOpti1.Y = round(avgPks_y1);
			dev_params.AvgPksOpti1.Z = round(avgPks_z1);
			dev_params.AvgPksOpti2.X = round(avgPks_x2);
			dev_params.AvgPksOpti2.Y = round(avgPks_y2);
			dev_params.AvgPksOpti2.Z = round(avgPks_z2);
			dev_params.AvgPksOpti3.X = round(avgPks_x3);
			dev_params.AvgPksOpti3.Y = round(avgPks_y3);
			dev_params.AvgPksOpti3.Z = round(avgPks_z3);
			dev_params.PnpBgRatioSum = 1;
			if(good_found_x1 == true && flag_lowsig_x1 == false)
			{
				alpha_x1_temp = round(pnp_opti_x1.np_ratio);
			}
			else
			{
				alpha_x1_temp = 0;
			}
			if(good_found_y1 == true && flag_lowsig_y1 == false)
			{
				alpha_y1_temp = round(pnp_opti_y1.np_ratio);
			}
			else
			{
				alpha_y1_temp = 0;
			}
			if(good_found_z1 == true && flag_lowsig_z1 == false)
			{
				alpha_z1_temp = round(pnp_opti_z1.np_ratio);
			}
			else
			{
				alpha_z1_temp = 0;
			}
			if(good_found_x2 == true && flag_lowsig_x2 == false)
			{
				alpha_x2_temp = round(pnp_opti_x2.np_ratio);
			}
			else
			{
				alpha_x2_temp = 0;
			}
			if(good_found_y2 == true && flag_lowsig_y2 == false)
			{
				alpha_y2_temp = round(pnp_opti_y2.np_ratio);
			}
			else
			{
				alpha_y2_temp = 0;
			}
			if(good_found_z2 == true && flag_lowsig_z2 == false)
			{
				alpha_z2_temp = round(pnp_opti_z2.np_ratio);
			}
			else
			{
				alpha_z2_temp = 0;
			}
			if(good_found_x3 == true && flag_lowsig_x3 == false)
			{
				alpha_x3_temp = round(pnp_opti_x3.np_ratio);
			}
			else
			{
				alpha_x3_temp = 0;
			}
			if(good_found_y3 == true && flag_lowsig_y3 == false)
			{
				alpha_y3_temp = round(pnp_opti_y3.np_ratio);
			}
			else
			{
				alpha_y3_temp = 0;
			}
			if(good_found_z3 == true && flag_lowsig_z3 == false)
			{
				alpha_z3_temp = round(pnp_opti_z3.np_ratio);
			}
			else
			{
				alpha_z3_temp = 0;
			}
#else
			dev_params.PnpBgRatio1.X = round(pnp_opti_x1.np_ratio);
			dev_params.PnpBgRatio1.Y = round(pnp_opti_y1.np_ratio);
			dev_params.PnpBgRatio1.Z = round(pnp_opti_z1.np_ratio);
			dev_params.AvgPksOpti1.X = round(avgPks_x1);
			dev_params.AvgPksOpti1.Y = round(avgPks_y1);
			dev_params.AvgPksOpti1.Z = round(avgPks_z1);
			dev_params.PnpBgRatioSum = 1;
			if(good_found_x1 == true && flag_lowsig_x1 == false)
			{
				alpha_x1_temp = round(pnp_opti_x1.np_ratio);
			}
			else
			{
				alpha_x1_temp = 0;
			}
			if(good_found_y1 == true && flag_lowsig_y1 == false)
			{
				alpha_y1_temp = round(pnp_opti_y1.np_ratio);
			}
			else
			{
				alpha_y1_temp = 0;
			}
			if(good_found_z1 == true && flag_lowsig_z1 == false)
			{
				alpha_z1_temp = round(pnp_opti_z1.np_ratio);
			}
			else
			{
				alpha_z1_temp = 0;
			}
#endif
			
			pnp_data_table_t pnp_opti_xz = {0};	
			bool good_found_xz = false;
			bool flag_lowsig_xz = false;
			float avgPks_xz = 0;
			
			// use sum_data_bpf to save memory
			for(uint16_t k = 0; k < ACTUAL_DATA_LEN; k++)
			{
#ifdef PAD_3_SENSOR
				sum_data_bpf[k] = sq_data->x1[k]*alpha_x1_temp + sq_data->y1[k]*alpha_y1_temp + sq_data->z1[k]*alpha_z1_temp + sq_data->x2[k]*alpha_x2_temp + sq_data->y2[k]*alpha_y2_temp
				+ sq_data->z2[k]*alpha_z2_temp + sq_data->x3[k]*alpha_x3_temp + sq_data->y3[k]*alpha_y3_temp + sq_data->z3[k]*alpha_z3_temp;
#else
				sum_data_bpf[k] = sq_data->x1[k]*alpha_x1_temp + sq_data->y1[k]*alpha_y1_temp + sq_data->z1[k]*alpha_z1_temp;
#endif
			}

			pnp_background(sum_data_bpf, ACTUAL_DATA_LEN, avgsq_noise_xyz, true, &pnp_opti_xz, &avgPks_xz, &good_found_xz,  &flag_lowsig_xz);
			LOG_DEBUGF("flag_lowsig_xz %d, np_ratio %f, good_found_xz %d", flag_lowsig_xz, pnp_opti_xz.np_ratio, good_found_xz);
			if((flag_lowsig_xz == true) || (pnp_opti_xz.np_ratio < min_pnp_bg) || (good_found_xz == false)){
				if(flag_lowsig_xz){
					dev_params.PnpBgRatioSum = 0;
				}
				BedManagerPnpPredict(BED_MANAGER_PNP_BED_NOT_GOOD);
				LOG_DEBUGF("Data not good");
			}
			else
			{
				Pnp_bg = pnp_opti_xz.np_ratio;
				dev_params.PnpBgRatioSum = Pnp_bg;
				BedManagerPnpPredict(BED_MANAGER_PNP_BED_GOOD);
				alpha_x1 = alpha_x1_temp;
				alpha_y1 = alpha_y1_temp;
				alpha_z1 = alpha_z1_temp;
#ifdef PAD_3_SENSOR
				alpha_x2 = alpha_x2_temp;
				alpha_y2 = alpha_y2_temp;
				alpha_z2 = alpha_z2_temp;
				alpha_x3 = alpha_x3_temp;
				alpha_y3 = alpha_y3_temp;
				alpha_z3 = alpha_z3_temp;
#endif
				th_adapt_xyz = pnp_opti_xz.th_adapt;   //optimum th_adapt

				LOG_DEBUGF("alpha_x1 %f, alpha_y1 %f, alpha_z1 %f, alpha_x2 %f, alpha_y2 %f, alpha_z2 %f, alpha_x3 %f, alpha_y3 %f, alpha_z3 %f",
					alpha_x1, alpha_y1, alpha_z1, alpha_x2, alpha_y2, alpha_z2, alpha_x3, alpha_y3, alpha_z3);
				LOG_DEBUGF("good found, th_adapt_xyz %f", th_adapt_xyz);
			}
		}
		else{
			LOG_DEBUGF("Movements %d", num_moves);
			BedManagerPnpPredict(BED_MANAGER_PNP_MOVEMENT);
		}

		vPortFree(sq_data);
		vPortFree(sum_data_bpf);	
		osThreadSuspend(pnpBGTaskHandle);
	}
}

dev_params_t* get_dev_params()
{
	return &dev_params;
}

float get_pnp_bg(void)
{
	if(BedManagerPnpGet())
	{
		return Pnp_bg;
	}
	return 0;
}

void reset_pnp_bg(void)
{
	memset(&dev_params, 0, sizeof(dev_params_t));
	num_execute_pnp_pg = 0;
	Pnp_bg = 0;
}

