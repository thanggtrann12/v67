/***************************************************************************
	%OnSky Inc. Confidential and Proprietary
	%Algorithm written by Hung Nguyen
	%C code written by Thien Phan
	%June 15, 2020
	%
	%Heart Rate Algorithm: this function is the R-peak Detection algorithm that 
	%searches for the heart-beat pulses and calculate its heart beat rate in 
	%BPM (beat per minute). 
	%For Heart Rate Algorithm, we use 4 seconds of ADC data for each sensor, that
	%is 1000 ADC samples at 250Hz sampling rate (sampling period: 0.004s).
	%
	%This version does not use S3S algorithm as in the architecture spec. Instead it
	%selects maximum peak within a pulse group as the R-peak (R-pulse).
	%
	% -square_data: this is input vector that includes square data of the ADC output. 
	%The function will go through this data and look for R-peaks or heart beat pulses.
	% -th_adapt: this is the threshold value. Below this threshold are low pulses 
	%that will not be used for R-peak search. If th_adapt = 0, then pnp block
	%is not ready to input it to the HR detection block.
	% -movements: this is an input vector that includes locations that ADC signals
	%are very high due to movement on the bed. There can be multiple rows that 
	%specify multiple movements. Each row has 2 columns: first column is the location
	%of the start of movement, second column is the location of the end of movement.
	%If movements = [0 0], then there is no movement in that data block.
	% -timegap_in and timegap_out: timegap is adaptive max_timegap in hr_detect.m function.
	%Timegap_in is the input from previous data block and timegap_out is the output
	%of adaptive max_timegap in this data block.Timegap value is based on the average
	%value of the np_window (difference of R-to-R pulses). Timegap is important to 
	%to define the boundary of a pulse group. Timegap (max_timegap) is continued
	%to be adjusted (or adaptive) on the fly using average of previous timegaps.
	%
	%
	%
	%***************************************************************************

	%****************************************************************************
	%Find avg_pks that is the average of all peaks above noise and without movements.
	%avg_pks_half is (avg_pks * 0.5), that is the temporary threshold for R_peak
	%detection when the optimum th_adapt is not available. 
	%When optimum th_adapt is available from pnp algorithm, then the optimum
	%threshold for R_peak detection is the average of th_adapt and avg_pks_half.
	%
	%The low_peak_th is the optimum threshold such that all peaks that are below it will
	%not be considered in the heart rate search.
	%
	%****************************************************************************/
#include "stdlib.h"
#include "main.h"
#include "hr_rr.h"
#include "dsp.h"
#include "Utils.h"

#define TIMEGAP_FACTOR (float32_t)0.45
#define SAMPLE_TIME (float32_t)0.004  //250Hz, that is 0.004 second as ADC sampling time

void hr_detect(float32_t * square_data, uint16_t data_size, float32_t th_adapt, movements_t* movements, uint8_t num_moves, bool xz_add, uint16_t timegap_in, float32_t avgsq_noise,
		bool *heart_lowsig, uint16_t *timegap_out, uint16_t *heart_beats, uint8_t *HR, float *HRV)
{
	//	LOG_DEBUGF("timegap_in %d", timegap_in);

	LOG_DEBUGF("num_moves %d", num_moves);
	uint16_t rr_limit = 385;            //R-R interval limit for adjusting max_timegap
	uint16_t  min_count_pk = (uint16_t)data_size*20/10000;  //minimum number of peaks above th_noise

	float32_t th_noise = avgsq_noise * 3; //do not consider peaks below this threshold, it is noise

	//At locations that are within 'movements' (user moves on bed), change
	//data to 0, so that they will not be used in finding peaks and R-peaks.
	for(uint8_t i = 0; i < num_moves; i++)	
	{
		for(uint16_t k = movements[i].loc_start; k < movements[i].loc_end; k++)
		{
			square_data[k] = 0;
		}	
	}		

	/*Find all peaks of signal*/
	uint16_t size_all_peaks = find_size_peaks(square_data, data_size);	
	LOG_DEBUGF("size_all_peaks: %d\r\n", size_all_peaks);
	if(size_all_peaks < 1){
		return;
	}
	Peak_ *all_peaks = NULL;
	all_peaks = (Peak_*)pvPortMalloc(size_all_peaks * sizeof(Peak_));
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

	uint16_t size_pks_over_noise = 0;
	for(uint16_t i = 0; i < size_all_peaks; i++)
	{
		if(all_peaks[i].amp > th_noise)
		{
			size_pks_over_noise++;				
		}
	}
	if(size_pks_over_noise < 1)
	{
		vPortFree(all_peaks); // free pointer all_peaks
		LOG_DEBUGF("size_pks_over_noise: %d\r\n", size_pks_over_noise);
		return;
	}
	Peak_ *pks_over_noise = NULL;
	pks_over_noise = (Peak_*)pvPortMalloc(size_pks_over_noise * sizeof(Peak_)); 	
	if(pks_over_noise == NULL){
		vPortFree(all_peaks); // free pointer all_peaks
		return;
	}

	for(uint16_t i = 0; i < size_all_peaks; i++)
	{
		if(all_peaks[i].amp > th_noise)
		{		
			pks_over_noise[count_pks].amp = all_peaks[i].amp;
			pks_over_noise[count_pks].loc = all_peaks[i].loc;
			sum_pks = sum_pks + all_peaks[i].amp;
			count_pks++;		
			if(all_peaks[i].amp > max_pks)
			{
				max_pks = all_peaks[i].amp;
			}
		}			
	}

	vPortFree(all_peaks); // free pointer all_peaks
	//	LOG_DEBUGF("size_pks_over_noise: %d\r\n", size_pks_over_noise);
	*heart_lowsig = false;

	if(count_pks == 0 || count_pks < min_count_pk)       //number of pulses too small ,less than 20 (in 10K samples)
	{																	
		*heart_lowsig = true;
		LOG_DEBUGF("heart_lowsig %d", *heart_lowsig);
		return;
	}	
	float32_t avg_pks = sum_pks / (float32_t)count_pks;     //take average
	float32_t low_peak_th = 0;
	//Calculate main threshold low_peak_th for R-peak detection
	if (th_adapt == 0)  {             //if there is no th_adapt from pnp block
		low_peak_th = avg_pks;   //then main threshold is avg_pks
	}
	else{
		low_peak_th = th_adapt;
	}

	/***********************************************************************
	%Find all peaks that are above the threshold low_peak_th.
	%.
	%
	%***********************************************************************/
	//	LOG_DEBUGF("low_peak_th: %f\r\n", low_peak_th);
	uint16_t size_pks_over_th = 0;
	for(uint16_t i = 0; i < size_pks_over_noise; i++)
	{
		if (pks_over_noise[i].amp > low_peak_th)
		{		
			size_pks_over_th++;
		}
	}

	Peak_ *pks_over_th = NULL;
	if(size_pks_over_th > 0){
		pks_over_th = (Peak_*)pvPortMalloc(size_pks_over_th * sizeof(Peak_));
	}
	if(pks_over_th == NULL){
		vPortFree(pks_over_noise); // free pointer all_peaks
		return;
	}	

	float32_t  sum_pks_th = 0.0;
	uint16_t  count_pks_th = 0;
	for(uint16_t i = 0; i < size_pks_over_noise; i++)
	{
		if (pks_over_noise[i].amp > low_peak_th)
		{
			pks_over_th[count_pks_th].amp = pks_over_noise[i].amp;
			pks_over_th[count_pks_th].loc = pks_over_noise[i].loc;
			//			 printf("pks_over_amp: %f, pks_over_loc: %d\r\n", pks_over_th[count_pks_th].amp, pks_over_th[count_pks_th].loc);
			sum_pks_th = sum_pks_th + pks_over_th[i].amp;
			count_pks_th = count_pks_th + 1;
		}	
	}
	vPortFree(pks_over_noise); // free pointer pks_over_noise

	//float32_t avg_pks_th = sum_pks_th / count_pks_th;     //take average

	/**********************************************************************
	%Find R-peaks (heart beat pulses) by going through vector [pks_over_th,
	%locs_over_th]. Search for pulse groups (first pulse, mid-pulses and last
	%pulses). For each pulse group, select the maximum peak as the R-peak
	%(R-pulse). Note that it does not use the S3S algorithm in this version.
	%
	%For each peak, look for timegap_before and timegap_after. To be in a 
	%pulse group, timegap_before and timegap_after must be less than 
	%max_timegap (defined as a constant at the top of the code). 
	%
	%***********************************************************************/  	
	//	LOG_DEBUGF("size_pks_over_th %d", size_pks_over_th);

	Peak_ *rpks = NULL;
	rpks = (Peak_*)pvPortMalloc(size_pks_over_th * sizeof(Peak_)); 			//R-peaks vector
	if(rpks == NULL){
		vPortFree(pks_over_th);
		vPortFree(pks_over_noise); // free pointer all_peaks
		return;
	}

	uint16_t timegap_before = 0;
	uint16_t timegap_after = 0;
	uint16_t index_rpks = 0;
	uint16_t previous_rlocs = 0;
	uint16_t max_timegap = timegap_in;  //initial max timegap between peaks within pulse group
	//default value is 100

	uint16_t rr1 = (uint16_t)timegap_in / TIMEGAP_FACTOR;   //initial value of R-R window
	//default timegap_in is 100, so
	//rr1 = 100/0.4 = 250 (equivalent
	//to 60beats per 60s
	uint16_t rr2 = rr1;       //initial value
	uint16_t rr3 = rr1;       //initial value
	uint16_t rr4 = rr1;       //initial value
	uint16_t rr5 = rr1;       //initial value
	uint16_t rr6 = rr1;       //initial value
	uint16_t fpk_zone = (uint16_t)roundf((float32_t)(rr1 - (max_timegap * 2)));
	uint16_t max_i = 0, first_pk =0, last_pk = 0, max_to_next = 0, current_to_rpk = 0;
	float32_t max_pk = 0.0, current_pk = 0.0;
	for(uint16_t i = 0; i < size_pks_over_th; i++)
	{
		current_pk = pks_over_th[i].amp;
		if (i == 0)
		{
			timegap_before = max_timegap + 10; //make assumption first pk
		}
		else
		{
			timegap_before = abs(pks_over_th[i].loc - pks_over_th[i-1].loc);
		}
		if (i == (size_pks_over_th - 1))
		{
			timegap_after = max_timegap + 10; //make assumption first pk
		}
		else
		{
			timegap_after = abs(pks_over_th[i].loc - pks_over_th[i+1].loc);
		}

		//Find time from next peak of current peak to max_pk (backward))
		if(i == (size_pks_over_th - 1))
		{
			max_to_next = max_timegap + 10;
		}
		else if (max_pk != 0)
		{
			//this is only used when the peaks are in middle
			max_to_next = abs( pks_over_th[i+1].loc - pks_over_th[max_i].loc );
		}
		else
		{
			max_to_next = 0; //when max_pk = 0, new pulse group
		}

		//Find duration from current peak to last R-pulse
		if((max_i != 0) && (i != 0))
		{
			current_to_rpk = abs( pks_over_th[i].loc - pks_over_th[max_i].loc );
		}
		else
		{
			current_to_rpk  = rr1;  //default
		}

		//********************************************
		//Checking through 5 cases
		if( ((timegap_before > (max_timegap + fpk_zone)) || (current_to_rpk > (max_timegap + fpk_zone))) //case 1: first pulse in a pulse group
				&& (timegap_after <= max_timegap) && (first_pk == 0) ) 
		{
			max_i = i;                      //then possible max pk is this i location
			max_pk  = pks_over_th[i].amp;       //possible max pk
			first_pk = 1;
		}
		else if( ((timegap_before <= max_timegap) && (max_to_next <= max_timegap)&&  //case 2a: middle pulse in a pulse group
				(current_pk < max_pk) && (first_pk == 1)) ||
				((timegap_before <= max_timegap) && (timegap_after <= max_timegap)&&//case 2a: also middle pulse in pulse group
						(current_pk >= max_pk) && (first_pk == 1)) )
		{
			if (pks_over_th[i].amp >= max_pk)
			{
				max_i = i;                      //then possible max pk is this i loc
				max_pk  = pks_over_th[i].amp;       //possible max pk
			}
		}
		else if((timegap_before <= max_timegap) &&  //case 3: last pk in a pulse group
				((timegap_after > max_timegap)||(max_to_next > max_timegap)) && (first_pk == 1) )
		{
			if (pks_over_th[i].amp >= max_pk)
			{
				max_i = i;                     //then possible max pk is this i loc
				max_pk  = pks_over_th[i].amp;      //possible max pk
			}
			last_pk  = 1;  //end of pulse group
		}
		else if((current_to_rpk <= (max_timegap + fpk_zone)) && (first_pk == 0))
			//case 4: single pulse,  in middle of np-window, that is false_pulse zone
		{

		}
		else // case 5: single pulse
		{
			max_i = i;                      //then max pk is this i loc
			max_pk = pks_over_th[i].amp;
			last_pk = 1;                    //end of pulse group
		}

		if (last_pk == 1)        //if it is last pk in a pulse group or single pulse
		{
			rpks[index_rpks].amp = pks_over_th[max_i].amp;
			rpks[index_rpks].loc = pks_over_th[max_i].loc;
			index_rpks++;

			//Update average of max_timegap (it is adaptive value)
			//shift left 3 previous R-R values
			rr1 = rr2;
			rr2 = rr3;
			rr3 = rr4;
			rr4 = rr5;
			rr5 = rr6;
			//update rr4
			uint16_t new_rr = pks_over_th[max_i].loc - previous_rlocs;

			if ((index_rpks > 4) && (new_rr <= rr_limit))
			{
				rr6 = new_rr;
			}
			else{
				rr6 = rr5;
			}
			float32_t avg_rr = (float32_t)(rr1 + rr2 + rr3 + rr4) / 4; //Average of 4 R-R windows
			max_timegap = (uint16_t)round(avg_rr * TIMEGAP_FACTOR);   //default timegap_factor = 0.45
			fpk_zone = (uint16_t)roundf((float32_t)(avg_rr - (max_timegap * 2))); //this is the duration of False-Peak Zone,
			// any pulses are in this zone are considered False Peaks.

			//save value of current location of R-peak
			previous_rlocs = pks_over_th[max_i].loc;   //save this value for avg max_timegap

			//Reset to find next pulse group
			last_pk = 0;
			first_pk = 0;
			max_pk = 0;
		}
	}
	//		for(uint16_t i = 0; i < index_rpks; i++)
	//		{
	//			LOG_DEBUGF("rpks[%d].loc = %d", i, rpks[i].loc);
	//		}

	//Update timegap_out to input to next data block
	*timegap_out = max_timegap;   //it is equal to the last max_timegap value
	//if R-pulse to R-pulse = 250 samples, then
	//timegap_out is 100 (optimum number)
	//timegap_out is used for next data
	//block as max_timegap

	/***************************************************************
    %Calculate Heart Rate (HR)
    %Sampling frequency: 250Hz (each sampling period is 0.004s)
    %Calculate Heart Rate Variability (HRV) using Root Mean Square 
    %Standard Deviation (SMSSD)
    %***************************************************************/

	//Calculate Heart Rate (BPM: beats per minute)
	*heart_beats = index_rpks;
	float32_t data_time = data_size * SAMPLE_TIME; //time duration of whole data
	if(num_moves > 0)
	{
		uint16_t sum_time_move = 0;
		sum_time_move += ( movements[1].loc_end - movements[1].loc_start )*num_moves*SAMPLE_TIME;
		data_time -= sum_time_move;
	}
	//		hr_rr_data->actual_hr = (uint8_t)round((*heart_beats * 60) / data_time); //heart rate in beats per minute

	/******************************************************
    %Calculate Heart Rate Variability (HRV) (in seconds)
    %using Root Mean Square of Successive Diffences (RMSSD)
    %******************************************************/	
	float32_t diff_rr = 0;
	float32_t sum_hrv = 0;
	uint16_t count = 0, count_instant_hr = 0;
	uint16_t np_window = 0, pre_np_window = 0, sum_rr_interval = 0;
	bool move_found = false;

	for(uint16_t i = 1; i < index_rpks; i++)
	{
		move_found = false;
		for(uint16_t k = 0; k < num_moves; k++)
		{
			if( (rpks[i-1].loc < movements[k].loc_start && rpks[i].loc > movements[k].loc_end))
			{
				move_found = true;
				LOG_DEBUGF("move_found %d", move_found);
			}
		}
		if(move_found == false)
		{
			np_window = rpks[i].loc - rpks[i-1].loc;
			if(np_window > 125 && np_window < 360)// && rpks[i-1].loc >= 100)
			{
				sum_rr_interval += np_window;
				count_instant_hr++;
			}
			//				LOG_DEBUGF("np_window %d", np_window);
			if(pre_np_window != 0)
			{
				diff_rr = (float32_t)(np_window - pre_np_window); //Difference between windows
				//					LOG_DEBUGF("diff_rr %f", diff_rr*diff_rr);
				sum_hrv += diff_rr*diff_rr;
				count++;
			}
			//			printf("%d\r\n", rpks[i].loc);
			pre_np_window = np_window;
		}
	}
	//		printf("%f %d\r\n", sum_hrv, count);
	if(sum_rr_interval!=0 && count_instant_hr > 2)
	{
		*HR = (uint8_t)roundf((float32_t)(60*Fs*count_instant_hr)/sum_rr_interval); //heart rate in beats per minute
		LOG_DEBUGF("HR %d", *HR);
		if(data_size == ACTUAL_DATA_LEN)
		{

			if(count > 0){
				*HRV = sqrt(sum_hrv*SAMPLE_TIME*SAMPLE_TIME / count) * 1000;  //HRV in ms
			}
			LOG_DEBUGF("HRV %f", *HRV);
		}
	}

	//Take square root
	vPortFree(pks_over_th); // free pointer pks_over_th
	vPortFree(rpks); // free pointer rpks
}
