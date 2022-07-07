 /***************************************************************************
	%OnSky Inc. Confidential and Proprietary
	%Algorithm written by Hung Nguyen
	%C code written by Thien Phan
	%December, 2020
	%
	*/
#include "stdlib.h"
#include "main.h"
#include "resp_detect.h"
#include "settings.h"
#include "dsp.h"
#include "Utils.h"
#include "LcdCtrl.h"

#define MAX_SIZE_RESP 120
#define MAX_SIZE_RESP_LIFELINE 30

float32_t eb_th_resp_freq = 3e7;
float32_t eb_th_resp_time = 2000;
extern float avg_energy_xy, avg_energy_resp;
static uint8_t time_warning = 40, time_emergency = 60; // 10 seconds and 20 seconds
//static uint8_t time_warning = 10, time_emergency = 10; // 10 seconds and 20 seconds
static const uint8_t  factor_mean_pk = 10;   //avg_pk_resp * factor_avg_pk is the limit to
																		//determine a peak is noise/movement or not
static const uint8_t factor_resp_freq = 3;
static const uint8_t factor_resp_time = 3;

void set_time_nobreathing(nobreathing_mode_t mode)
{
	reset_lifeline_resp();
	if(mode == NOBREATH_DEMO)
	{
		time_warning = 8;
		time_emergency = 16;
	}	
	else if(mode == NOBREATH_HIGH)
	{
		time_warning = 30;
		time_emergency = 50;
	}
	else if(mode == NOBREATH_MEDIUM_2)
	{
		time_warning = 50;
		time_emergency = 70;
	}
	else if(mode == NOBREATH_MEDIUM_1)
	{
		time_warning = 60;
		time_emergency = 80;
	}
	else if(mode == NOBREATH_LOW)
	{
		time_warning = 80;
		time_emergency = 100;
	}
}


uint8_t check_valid_respRate(uint8_t rr, bool clean)
{
	uint8_t rrOut = ERROR_CASE;
	static uint8_t preRR = 0;
	static bool holdFlag = false;
	const uint8_t disRR = 8;
	if(clean == true)
	{
		preRR = 0;
		holdFlag = false;
		return 0;
	}
	if(preRR == 0)
	{
    	if(rr >= 7 && rr <= 27)
    	{
    		preRR = rr;
    		return rr;
    	}
    	else{
    		preRR = 0;
    	    return ERROR_CASE;
    	}
	}
	int16_t subRr = rr - preRR;
	if(abs(subRr) > disRR && holdFlag == false)
	{
		rrOut = PREVIOUS_CASE;
		holdFlag = true;
	}
	else
	{
		if(rr >= 7 && rr <= 27)
		{
			rrOut = rr;
			preRR = rr;
			holdFlag = false;
		}
	}

	return rrOut;
}

//void test_resp(void)
//{
//	uint8_t rr;
//	resp_data_t *resp_data = (resp_data_t *)raw_dataxy;
//	resp_detect(resp_data, RESP_RAW_DATA_LEN, &rr);
//}

void resp_detect(resp_data_t *resp_data, const uint16_t data_size, uint8_t *RR, uint8_t preRR)
{		
	 *RR = PREVIOUS_CASE;
	
	const uint16_t down_size = data_size/2;
	
	LcdCtrlLock();
	float32_t *sum_sq_xy_resp = NULL;  // for sum of sq x_bpf (method 2)
 	sum_sq_xy_resp = (float32_t *)pvPortMalloc(data_size*sizeof(float32_t));
	if(sum_sq_xy_resp == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sum_sq_xy_resp");
		Error_Handler();
	}	
	
	float32_t *sum_sq_xy_bpf = NULL;  // for fft input (method1)  and filter results (method2)
 	sum_sq_xy_bpf = (float32_t *)pvPortMalloc(data_size*sizeof(float32_t));
	if(sum_sq_xy_bpf == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sum_sq_xy_bpf");
		Error_Handler();
	}

	float32_t *sum_sq_xy_fft = NULL;  // for fft results
 	sum_sq_xy_fft = (float32_t *)pvPortMalloc(down_size*sizeof(float32_t));
	if(sum_sq_xy_fft == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sum_sq_xy_fft");
		Error_Handler();
	}		
	
	float32_t *sum_data_bpf = NULL;  // for sum bfp results -> movements
 	sum_data_bpf = (float32_t *)pvPortMalloc(data_size*sizeof(float32_t)); 
	if(sum_data_bpf == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sum_data_bpf");
		Error_Handler();
	}	
	
	resp_data_t *resp_data_bpf = NULL;  // for sum bfp results -> movements
 	resp_data_bpf = (resp_data_t *)pvPortMalloc(sizeof(resp_data_t)); 
	if(resp_data_bpf == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :resp_data_bpf");
		Error_Handler();
	}		
	LcdCtrlUnlock();

	filter_data(BPF_0P7_20, resp_data->x1, resp_data_bpf->x1, data_size);
	filter_data(BPF_0P7_20, resp_data->y1, resp_data_bpf->y1, data_size);
#ifdef PAD_3_SENSOR
	filter_data(BPF_0P7_20, resp_data->x2, resp_data_bpf->x2, data_size);
	filter_data(BPF_0P7_20, resp_data->y2, resp_data_bpf->y2, data_size);
	filter_data(BPF_0P7_20, resp_data->x3, resp_data_bpf->x3, data_size);
	filter_data(BPF_0P7_20, resp_data->y3, resp_data_bpf->y3, data_size);
#endif

	for(uint16_t i = 0; i < data_size; i++)
	{
#ifdef PAD_3_SENSOR
		sum_data_bpf[i] = resp_data_bpf->x1[i] + resp_data_bpf->y1[i] + resp_data_bpf->x2[i] + resp_data_bpf->y2[i] + resp_data_bpf->x3[i] + resp_data_bpf->y3[i];
		sum_sq_xy_resp[i] = resp_data_bpf->x1[i]*resp_data_bpf->x1[i] + resp_data_bpf->y1[i]*resp_data_bpf->y1[i] + resp_data_bpf->x2[i]*resp_data_bpf->x2[i]
						+ resp_data_bpf->y2[i]*resp_data_bpf->y2[i]+ resp_data_bpf->x3[i]*resp_data_bpf->x3[i] + resp_data_bpf->y3[i]*resp_data_bpf->y3[i]; // for method 2 (time domain)
#else
		sum_data_bpf[i] = resp_data_bpf->x1[i] + resp_data_bpf->y1[i];
		sum_sq_xy_resp[i] = resp_data_bpf->x1[i]*resp_data_bpf->x1[i] + resp_data_bpf->y1[i]*resp_data_bpf->y1[i]; // for method 2 (time domain)
#endif
		if(!(i%2))
		{
			sum_sq_xy_bpf[i] = sum_sq_xy_resp[i];
		}
		else{
			sum_sq_xy_bpf[i] = 0;
		}
	}

// downsize and create fft input (sum_sq_xy_bpf)

	vPortFree(resp_data_bpf);

	//Call search_movements function for 32.786-second data (data after BPF)
	movements_t movements[MAX_MOV_40s_NUM] = {0};
	float32_t avg_energy_out = 0;   //this value will need to be adjusted later
	uint8_t num_moves = search_movements(sum_data_bpf, data_size, avg_energy_xy, movements, &avg_energy_out, NULL, NULL);
	vPortFree(sum_data_bpf);

	/*Method1 : sum FFT Square X*/
	fft_f32(sum_sq_xy_bpf, sum_sq_xy_fft, down_size);

	//Find max of peaks that is the position of breathing
	const uint8_t byteStart = 2, range = 25;
	const uint16_t size_all_pks1 = find_size_peaks(&sum_sq_xy_fft[byteStart], range);
	LOG_DEBUGF("size_all_pks %d",size_all_pks1);
	
	float32_t peak_max = 0, th_fft_resp = eb_th_resp_freq;
	uint16_t index_pk_max = 0, count_pk_method1 = 0;
	uint8_t breaths_method1 = 0, resp_rate_method1 = 0;
	float32_t time = (float32_t)data_size/250;
	Peak_ *all_pks = NULL;
	if(size_all_pks1){
		all_pks = (Peak_ *)pvPortMalloc(size_all_pks1*sizeof(Peak_));
	}
	if(all_pks == NULL){
		LOG_DEBUGF("Could not allocate memory: all_pks");
	}
	else{
		find_all_peaks(&sum_sq_xy_fft[byteStart], range, all_pks);

		for(uint16_t i = 0; i < size_all_pks1; i++)
		{
			LOG_DEBUGF("all_pks[%d].amp = %f, all_pks[%d].loc = %d", i, all_pks[i].amp, i, all_pks[i].loc);
			if(all_pks[i].amp > peak_max)
			{
				peak_max = all_pks[i].amp;
				index_pk_max = i;
			}
		}
		LOG_DEBUGF("peak_max %f", peak_max);
		if(peak_max != 0)
		{
			uint8_t breaths_method1_temp = all_pks[index_pk_max].loc + byteStart;
			LOG_DEBUGF("breaths_method1_temp %d", breaths_method1_temp);
			if((peak_max >= th_fft_resp) && (breaths_method1_temp >= 4) && (breaths_method1_temp <= 16))
			{
				breaths_method1 = breaths_method1_temp; // number of breaths
			}
			else{
				breaths_method1 = 0; // number of breaths
			}

			for(uint16_t i = 2; i < size_all_pks1; i++)
			{
				if((all_pks[i].amp > th_fft_resp) && (all_pks[i].amp > (peak_max*0.4)))
				{
					count_pk_method1++;
				}
			}
		}
		else{
			//breaths_freq is not valid
			breaths_method1 = 0;
		}

		LOG_DEBUGF("breaths_method1 %d", breaths_method1);
		resp_rate_method1  =  (uint8_t)roundf((float32_t)breaths_method1 * (60 / time));   //breaths per minute
		LOG_DEBUGF("resp_rate_method1 %d", resp_rate_method1);
		
		vPortFree(all_pks);
	}
	vPortFree(sum_sq_xy_fft);
	
	
/*==================================Method 2============================================*/
//Method 2: Time domain for respiration - Use squares of X1, X2, and X3.	
	static float pre_mean_resp = 1e+20;
	filter_data(BPF_0P2_2P0, sum_sq_xy_resp, sum_sq_xy_bpf, data_size);			 //Apply BandPass Filter FIR
	filter_data(LPF_1P0, sum_sq_xy_bpf, sum_sq_xy_resp, data_size); //Apply Cascade FIR LPF

	vPortFree(sum_sq_xy_bpf);
	
	uint8_t resp_rate_method2 = 0;
	const uint16_t size_pks_timeDomain = find_size_peaks(sum_sq_xy_resp, data_size);
	LOG_DEBUGF("size_pks_timeDomain %d", size_pks_timeDomain);
	Peak_ *all_peaks = NULL;  
	if(size_pks_timeDomain > 0){
 		all_peaks = (Peak_ *)pvPortMalloc(size_pks_timeDomain*sizeof(Peak_));
	}
	if(all_peaks != NULL)
	{
		find_all_peaks(sum_sq_xy_resp, data_size, all_peaks);
		float32_t sum = 0;
		uint16_t sizeMean = 0;
		for(uint16_t i = 0; i < size_pks_timeDomain; i++)
		{
			if(all_peaks[i].amp < (float)(pre_mean_resp*factor_mean_pk)){
				sum += all_peaks[i].amp;
				sizeMean++;
			}
		}	
		
		if(sizeMean > 1 && sizeMean < MAX_SIZE_RESP)
		{
			float mean_resp = 0;
			mean_resp = sum/(float32_t)sizeMean;
			LOG_DEBUGF("size_pks_timeDomain %d, mean_resp %f", sizeMean, mean_resp);
			pre_mean_resp = mean_resp;
			float32_t *resp_signal;	
			resp_signal = (float32_t *)pvPortMalloc(sizeMean*sizeof(float32_t));
			if(resp_signal == NULL){
				LOG_DEBUGF("Could not allocate memory: resp_signal");
				Error_Handler();
			}
				
			for(uint16_t i = 0; i < sizeMean; i++)
			{
					resp_signal[i] = all_peaks[i].amp - mean_resp;
			}		
		 //***********************************************
		 //Use Zero-crossing algorithm
		 //***********************************************
			float32_t res_pos_th = 2e3;
			float32_t res_neg_th = -2e3;
			
			uint8_t zcp_index[MAX_SIZE_RESP] = {0};
			uint8_t zcn_index[MAX_SIZE_RESP] = {0};	
			uint8_t index_zcp = 1, index_zcn = 1;
			// find zero points
			for(uint8_t i = 1; i < (sizeMean-1); i++)
			{
				if(resp_signal[i] < 0 && resp_signal[i+1] > 0)
				{
					zcp_index[index_zcp++] = i;
				}
				else if(resp_signal[i] > 0 && resp_signal[i+1] < 0)
				{
					zcn_index[index_zcn++] = i;
				}
			}
			zcp_index[0] = 0;
			zcp_index[index_zcp++] = sizeMean-1;
			zcn_index[0] = 0;
			zcn_index[index_zcn++] = sizeMean-1;
			LOG_DEBUGF("index_zcp %d, index_zcn %d", index_zcp, index_zcn);
			uint8_t index_valid_zcp = 0, index_valid_zcn = 0;
			uint8_t pos_breaths = 0, neg_breaths = 0;
			int8_t loc_max = 0, loc_min = 0;
			if(index_zcp > 2)
			{
				for(uint8_t i = 0; i < (index_zcp-1); i++)
				{
//					LOG_DEBUGF("zcp_index[%d] = %d", i, zcp_index[i]);
					loc_max = check_valid_zcp(resp_signal, zcp_index[i], zcp_index[i+1], res_pos_th);
					if(loc_max != -1)
					{
//						LOG_DEBUGF("loc_max %d", loc_max);
						index_valid_zcp++;
					}
				}
				pos_breaths = index_valid_zcp;
			}
			else{
				pos_breaths = 0;	
			}
			
			if(index_zcp > 2)
			{				
				for(uint8_t i = 0; i < (index_zcn-1); i++)
				{
//					LOG_DEBUGF("zcn_index[%d] = %d", i, zcn_index[i]);
					loc_min = check_valid_zcn(resp_signal, zcn_index[i], zcn_index[i+1], res_neg_th);
					if(loc_min != -1)
					{
//						LOG_DEBUGF("loc_min %d", loc_min);
						index_valid_zcn++;
					}
				}	
				neg_breaths = index_valid_zcn;
			}		
			else{
				neg_breaths = 0;	
			}
//			LOG_DEBUGF("pos_breaths %d, neg_breaths %d", pos_breaths, neg_breaths);
			float32_t breaths_time = 0;
			if( ((pos_breaths == 0)&&(neg_breaths==0)) || ((pos_breaths == 0)&&(neg_breaths==1)))
			{
				breaths_time = 0;
			}
			else if((pos_breaths == 1)&&(neg_breaths == 0))
			{
				breaths_time = 1;
			}
			else{
				breaths_time = (float)(pos_breaths+neg_breaths)/2;
			}
			LOG_DEBUGF("breaths_method2 %f", breaths_time);
			resp_rate_method2 = (uint8_t)roundf(breaths_time * (60 / time));   //breaths per minute
			LOG_DEBUGF("resp_rate_method2 %d",resp_rate_method2);

			vPortFree(resp_signal);
		}
		vPortFree(all_peaks);
	}
	vPortFree(sum_sq_xy_resp);
	
	
/*************************************************************
%Selection of respiration result from 2 methods (frequency domain
%and time domain).
%Note that method 2 is not accurate when number of breaths is
%greater than 15. Therefore, if method 1 result is greater than
%15, then it is more likely it is the result. 
%*************************************************************/
  //method1 result -> resp_rate
  //method2 result _> number_of_breath_per_minute;
	LOG_DEBUGF("count_pk_method1 %d", count_pk_method1);
	uint8_t breaths_selected = 0;
	if( (count_pk_method1 > 2) || breaths_method1 == 0)
	{
		breaths_selected = resp_rate_method2;
		LOG_DEBUGF("Select mothod2, breaths_selected %d", breaths_selected);
	}
	else if(count_pk_method1 <= 2)
	{
		breaths_selected = resp_rate_method1;
		LOG_DEBUGF("Select mothod1, breaths_selected %d", breaths_selected);
	}
	else{
		LOG_DEBUGF("Invalid resp rate!");
		breaths_selected = ERROR_CASE; // invalid resp
	}

	if( ( (preRR < 7 || preRR > 27) && (SettingGetModeOperation().mode == OPE_MODE_AUTO) ) && (breaths_selected >= 7) && (breaths_selected <= 27) )
	{
		*RR = breaths_selected;
		return;
	}

    if (breaths_selected != ERROR_CASE)
    {
        *RR = check_valid_respRate(breaths_selected, false);
    }

	if(num_moves != 0)
	{
		LOG_DEBUGF("Movements %d", num_moves);
		*RR = PREVIOUS_CASE;
	}
}

typedef struct{
	float32_t resp;
	uint16_t time_nobreath;
	resp_status_t resp_status;
}resp_status_para_t;
resp_status_para_t pre_resp = {0, 0, RESP_NORMAL};

static float avg_mean_resp_arr[4] = {0};
static float avg_normal_mean_resp_arr[4] = {0};

void store_normal_mean(float normal_mean)
{
	if(avg_normal_mean_resp_arr[0] == 0 && avg_normal_mean_resp_arr[1] == 0 && avg_normal_mean_resp_arr[2] == 0 && avg_normal_mean_resp_arr[3] == 0)
	{
		avg_normal_mean_resp_arr[3] = normal_mean;
		avg_normal_mean_resp_arr[2] = normal_mean;
		avg_normal_mean_resp_arr[1] = normal_mean;
		avg_normal_mean_resp_arr[0] = normal_mean;
	}
	else{
		avg_normal_mean_resp_arr[3] = avg_normal_mean_resp_arr[2];
		avg_normal_mean_resp_arr[2] = avg_normal_mean_resp_arr[1];
		avg_normal_mean_resp_arr[1] = avg_normal_mean_resp_arr[0];
		avg_normal_mean_resp_arr[0] = normal_mean;
	}
}

float get_normal_mean(void){
	float sum = 0;
	for(uint8_t i = 0; i < 4; i++)
	{
		sum += avg_normal_mean_resp_arr[i];
	}
	return sum/4;
}

float get_avg_mean_resp(void)
{
	float sum = 0;
	for(uint8_t i = 0; i < 4; i++)
	{
		sum += avg_mean_resp_arr[i];
	}
	return sum/4;
}

void set_avg_mean_resp(float mean_resp)
{
	avg_mean_resp_arr[3] = avg_mean_resp_arr[2];
	avg_mean_resp_arr[2] = avg_mean_resp_arr[1];
	avg_mean_resp_arr[1] = avg_mean_resp_arr[0];
	avg_mean_resp_arr[0] = mean_resp;
}

static uint16_t pre_time_apnea = 0, countApnea = 0, countApnea40s = 0;
static float pre_resp_apnea = 0;
uint16_t getCountApnea(void){
	LOG_DEBUGF("countApnea %d", countApnea);
	uint16_t temp = countApnea;
	countApnea = 0;
	return temp;
}

uint16_t getCountApnea40s(void){
	LOG_DEBUGF("countApnea40s %d", countApnea40s);
	uint16_t temp = countApnea40s;
	countApnea40s = 0;
	return temp;
}

void resetApnea(void){
	pre_time_apnea = 0;
	countApnea = 0;
	countApnea40s = 0;
	pre_resp_apnea = 0;
}

static resp_status_t resp_stop_detect(float32_t *resp, uint16_t size_resp, const float32_t th_resp_time, uint8_t* inhale_count, bool* is_apnea)
{
	LOG_DEBUGF("linelife_resp");
	resp_status_t resp_status = RESP_NORMAL;
	const float time_window = (float)10;
	*inhale_count = 0;
	*is_apnea = false;
	float current_resp = 0;
	bool neg2pos_flag = false;
	uint16_t time_nobreath = 0, time_apnea = 0, start_loc = 0, start_loc_apnea = 0;
	bool is_inhale = false;
	static bool apneaCalculatedFlag = false;
	for(int16_t i = 0; i < (size_resp-1); i++) //start from -1 to take care last sample
	{
		LOG_DEBUGF("linelife_resp ------------------");
		if(i == 0){
			current_resp = pre_resp.resp;
		}
		else if(resp[i] < 0 && resp[i+1] >= 0){ // if current sample is neg and next sample is pos
			neg2pos_flag = true;                 // --> set negative to positive flag
			current_resp = resp[i];
		}
		else{
			current_resp = resp[i];
		}

		float diff = resp[i+1] - current_resp;
		bool pos_slope = false;//positive slope
		if(diff > 0){
			pos_slope = true;
		}

		is_inhale = false;
		if( (pos_slope == true) && ( (current_resp <= 0 && resp[i+1] > th_resp_time) || (current_resp < th_resp_time && diff >= th_resp_time && current_resp > 0)
			|| (current_resp >= th_resp_time && diff >= th_resp_time && neg2pos_flag == true) ))
		{
//			LOG_DEBUG("is_inhale");
			is_inhale = true;
			neg2pos_flag = false; // this inhale reset negative to positive flag
		}
		else{
			is_inhale = false; // dont consider exhale or no inhale
		}

		if (is_inhale == true)    //when inhale, then
		{
			(*inhale_count)++;
			start_loc = i+1;
			pre_resp.time_nobreath = 0;
			pre_resp.resp_status = RESP_NORMAL;

			start_loc_apnea = i + 1;
			pre_time_apnea = 0;
			apneaCalculatedFlag = false;
		}


		time_apnea = pre_time_apnea + (i + 1) - start_loc_apnea;
		float time_apnea_sec = ((float)((float)time_apnea * time_window) / (size_resp-1)); //convert to seconds
		LOG_DEBUGF("time_apnea %d, time_apnea_sec %f", time_apnea, time_apnea_sec);
		if(time_apnea_sec >= 20.0 && apneaCalculatedFlag == false)
		{
			apneaCalculatedFlag = true;
			*is_apnea = true;
			countApnea++;
			countApnea40s++;
			LOG_DEBUGF("countApnea %d, countApnea40s %d", countApnea, countApnea40s);
			//pre_time_apnea = 0;
			//start_loc_apnea = i + 1;
		}
		
		/*-----------
		- For vibrator
		- Find no breathing time
		-----------*/
		time_nobreath = pre_resp.time_nobreath + (i+1) - start_loc;

		LOG_DEBUGF("pre_time_nobreath %d, start_loc %d, %d", pre_resp.time_nobreath, start_loc, (i+1));
		LOG_DEBUGF("time_nobreath %d", time_nobreath);
		float time_nobreath_sec = ((float)((float)time_nobreath * time_window) / (size_resp-1)); //convert to seconds
		LOG_DEBUGF("time_nobreath_sec %f", time_nobreath_sec);
		//Check if passing time for warning (turn on vibrate)

		if (time_nobreath_sec >= (float)time_warning)  //if time without breath reach time for warning and vibrate
		{
			if(pre_resp.resp_status == RESP_NORMAL){
				resp_status = RESP_VIBRATE;
				pre_resp.resp_status = RESP_VIBRATE;
				LOG_DEBUGF("Respiration Warning");
			}

			/*-----------
			- For Emergency
			-----------*/
			if (pre_resp.resp_status == RESP_VIBRATE && time_nobreath_sec >= (float)time_emergency)  //if time without breath reach time for warning and vibrate
			{
				resp_status = RESP_EMERGENCY;
				pre_resp.resp_status = RESP_NORMAL;
		        start_loc = i + 1;
		        reset_lifeline_resp();
				LOG_DEBUGF("RESP_EMERGENCY");
				return resp_status;
			}
		}
	}
  //Check end of data to set current_start_flag and current_time_nobreath
	pre_resp.resp = resp[size_resp-1];               //this resp value go to next data
	pre_resp.time_nobreath = time_nobreath;
	pre_time_apnea = time_apnea;
	return resp_status;
}

//void thien_testResp(void)
//{
////	resplife_data_t *thien = (resplife_data_t *)rawResp;
////	instant_resp(thien, RESP_LIFE_RAW_DATA_LEN);
////	while(1){};
//}

void instant_resp(resplife_data_t* resplife_data, const uint16_t data_size, float avg_normal_mean_resp, float avg_mean_resp_in, bool start_resp, bool is_normal, bool hold_normal_mean, float* resp_out, uint16_t *size_resp, float* avg_mean_resp_out)
{
	LOG_DEBUGF("resp stopping detect");
	*size_resp = 0;
	resplife_data_t *resplife_data_bpf = NULL;  // for filter results and square results
 	resplife_data_bpf = (resplife_data_t *)pvPortMalloc(sizeof(resplife_data_t));
	if(resplife_data_bpf == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :resplife_data");
		Error_Handler();
	}

	float *sq_data = NULL;  // for filter results and square results
 	sq_data = (float *)pvPortMalloc(RESP_LIFE_RAW_DATA_LEN*sizeof(float));
	if(sq_data == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sq_data");
		Error_Handler();
	}

	filter_data(BPF_2P0_10, resplife_data->x1, resplife_data_bpf->x1, data_size);
	filter_data(BPF_2P0_10, resplife_data->y1, resplife_data_bpf->y1, data_size);
#ifdef PAD_3_SENSOR
	filter_data(BPF_2P0_10, resplife_data->x2, resplife_data_bpf->x2, data_size);
	filter_data(BPF_2P0_10, resplife_data->y2, resplife_data_bpf->y2, data_size);
	filter_data(BPF_2P0_10, resplife_data->x3, resplife_data_bpf->x3, data_size);
	filter_data(BPF_2P0_10, resplife_data->y3, resplife_data_bpf->y3, data_size);

	for(uint16_t i = 0; i < RESP_LIFE_RAW_DATA_LEN; i++)
	{
		sq_data[i] =  resplife_data_bpf->x1[i]*resplife_data_bpf->x1[i] + resplife_data_bpf->y1[i]*resplife_data_bpf->y1[i] + resplife_data_bpf->x2[i]*resplife_data_bpf->x2[i]
						+ resplife_data_bpf->y2[i]*resplife_data_bpf->y2[i]+ resplife_data_bpf->x3[i]*resplife_data_bpf->x3[i] + resplife_data_bpf->y3[i]*resplife_data_bpf->y3[i];
    }
#else
	for(uint16_t i = 0; i < RESP_LIFE_RAW_DATA_LEN; i++)
	{
		sq_data[i] =  resplife_data_bpf->x1[i]*resplife_data_bpf->x1[i] + resplife_data_bpf->y1[i]*resplife_data_bpf->y1[i];
    }
#endif
	vPortFree(resplife_data_bpf);

	float *resp_data_bpf = NULL;
	resp_data_bpf = (float *)pvPortMalloc(RESP_LIFE_RAW_DATA_LEN*sizeof(float32_t));
	if(resp_data_bpf == NULL){
		LOG_DEBUGF("Could not allocate memory :resp_data_bpf");
		Error_Handler();
	}
	float *resp_data_lpf = NULL;
	resp_data_lpf = (float *)pvPortMalloc(RESP_LIFE_RAW_DATA_LEN*sizeof(float32_t));
	if(resp_data_lpf == NULL){
		LOG_DEBUGF("Could not allocate memory :resp_data_lpf");
		Error_Handler();
	}

	filter_data(BPF_0P2_2P0, sq_data, resp_data_bpf, RESP_LIFE_RAW_DATA_LEN);			 //Apply BandPass Filter FIR
	filter_data(LPF_1P0, resp_data_bpf, resp_data_lpf, RESP_LIFE_RAW_DATA_LEN); //Apply Cascade FIR LPF
	vPortFree(sq_data);
	vPortFree(resp_data_bpf);

	const uint16_t size_all_pks2 = find_size_peaks(resp_data_lpf, RESP_LIFE_RAW_DATA_LEN);
	LOG_DEBUGF("size_all_pks2 %d", size_all_pks2);
	Peak_ *all_peaks = NULL;
	if(size_all_pks2 > 0){
 		all_peaks = (Peak_ *)pvPortMalloc(size_all_pks2*sizeof(Peak_));
	}
	if(all_peaks != NULL)
	{
		find_all_peaks(resp_data_lpf, RESP_LIFE_RAW_DATA_LEN, all_peaks);
		float32_t sum = 0;
		for(uint16_t i = 0; i < size_all_pks2; i++)
		{
			sum += all_peaks[i].amp;
		}
		float32_t th_xr_lpf = (float32_t)0.01 *( sum/(float32_t)size_all_pks2);
		sum = 0;
		uint16_t count_mean = 0;
		for(uint16_t i = 0; i < size_all_pks2; i++)
		{
			LOG_DEBUGF("all_peaks[%d].amp = %f, loc %d", i, all_peaks[i].amp, all_peaks[i].loc);
			if(all_peaks[i].amp < (float)(avg_mean_resp_in*factor_mean_pk) && all_peaks[i].amp > th_xr_lpf){
				sum += all_peaks[i].amp;
				count_mean++;
			}
		}
		if(count_mean > 1 && count_mean < MAX_SIZE_RESP_LIFELINE)
		{
			resp_out[0] = 0;
			uint8_t index_resp = 1;

			// calculate mean resp
			float mean_resp = (float)sum/count_mean;
			* avg_mean_resp_out = mean_resp;

			float avg_mean_2 = 0;
			if(start_resp == true) // first time
			{
				avg_mean_2 = mean_resp;
			}
			else if((is_normal == true)&&(hold_normal_mean == false)){
				avg_mean_2 = (avg_normal_mean_resp + mean_resp)/2;
			}
			else{
				avg_mean_2 = avg_normal_mean_resp;
			}
			LOG_DEBUGF("avg_mean_2 %f, mean_resp %f, avg_normal_mean_resp %f", avg_mean_2, mean_resp, avg_normal_mean_resp);
			for(uint16_t i = 0; i < size_all_pks2; i++)
			{
				if(all_peaks[i].amp > th_xr_lpf)
				{
					resp_out[index_resp] = all_peaks[i].amp - avg_mean_2;
					LOG_DEBUGF("resp_out[%d].amp %f", index_resp, resp_out[index_resp]);
					index_resp++;
				}
			}
			*size_resp = index_resp;
		}
		else{
			LOG_DEBUGF("count_mean = %d", count_mean);
		}
		vPortFree(all_peaks);
	}
	vPortFree(resp_data_lpf);
}

static float avg_mean_resp_out = 2e+5, avg_mean_resp_in = 0, avg_normal_mean_resp = 0;
static uint8_t num_mov_lifeline = 0, inhale_count = 0;
static bool is_apnea = false, hold_normal_mean = false;
static uint32_t index_lifeline = 0, m_movements = 0;
static float normal_mean = 0, avg_energy_in_lf = 10e20;

void reset_lifeline_resp(void)
{
	pre_time_apnea = 0;
	pre_resp_apnea = 0;
	memset(avg_normal_mean_resp_arr, 0, sizeof(avg_normal_mean_resp_arr));
	memset(avg_mean_resp_arr, 0, sizeof(avg_mean_resp_arr));
	memset(&pre_resp, 0, sizeof(resp_status_para_t));
	avg_mean_resp_out = 2e+5;
	avg_mean_resp_in = 0;
	avg_normal_mean_resp = 0;
	num_mov_lifeline = 0;
	inhale_count = 0;
	is_apnea = false;
	hold_normal_mean = false;
	index_lifeline = 0;
	m_movements = 0;
	normal_mean = 0;
	avg_energy_in_lf = avg_energy_resp;
}

resp_status_t lifeline_resp(resplife_data_t* resplife_data, const uint16_t data_size)
{
	resp_status_t resp_status = RESP_NORMAL;
	index_lifeline++;
	bool is_normal = false, is_movements = false;
	if (avg_mean_resp_out < (avg_normal_mean_resp * 2)){
        is_normal = true;
    }
    else{
        is_normal = false;
    }
	if(num_mov_lifeline > 0){
		is_movements = true;
		m_movements = index_lifeline;
	}
	else{
		is_movements = false;
	}
	LOG_DEBUGF("is_normal %d, is_movements %d", is_normal, is_movements);

	if((inhale_count > 1) || (is_movements == true) || (index_lifeline <= (m_movements + 4) &&  index_lifeline >= m_movements) )
	{
		hold_normal_mean = 0; // release avg_normal_mean_resp when emergency or movements in last cycle because mean may change due to changing position
	}
	else if((is_apnea == true) && (index_lifeline>5))
	{
		hold_normal_mean = true; //hold avg_normal_mean_resp when apnea is detected
	}
	else if(is_normal == false)
	{
		hold_normal_mean = false;
	}
	else{
		hold_normal_mean = hold_normal_mean;
	}

    //Adaptive method: Handling vector all_avg_mean_resp that has
    //4 previous normal mean_resp
    //Shift right vector and put new mean_resp in first position
    //If mean_resp of previous cycle is too high (>2X) then don't do anything
	bool start_resp = false;
    if( (index_lifeline == 1) || (is_movements == true)) //at first data cycle
	{
        start_resp = true;
	}
    else if( (index_lifeline == 2)|| (index_lifeline == m_movements+1) )  //at second data cycle
	{
        set_avg_mean_resp(avg_mean_resp_out);
        set_avg_mean_resp(avg_mean_resp_out);
        set_avg_mean_resp(avg_mean_resp_out);
        set_avg_mean_resp(avg_mean_resp_out);
        avg_normal_mean_resp = get_avg_mean_resp();
        start_resp = 1;
	}
    else if( (index_lifeline <= 5) || ((index_lifeline <= m_movements+4) && (index_lifeline > m_movements+1)) )
    {
        set_avg_mean_resp(avg_mean_resp_out);
                             //shift right vector, push new mean_resp in push last mean_resp out
        avg_normal_mean_resp = get_avg_mean_resp(); //average
        start_resp = 1;
    }
    else if( (hold_normal_mean == 1) )  //hold mean , don't update
    {
        avg_normal_mean_resp = normal_mean; //use older value of avg_normal_mean as before
        set_avg_mean_resp(normal_mean);
        set_avg_mean_resp(normal_mean);
        set_avg_mean_resp(normal_mean);
        set_avg_mean_resp(normal_mean);
        start_resp = 0;
    }
    else if (is_normal == 1)             //normal cases, continue to update
    {
    	set_avg_mean_resp(avg_mean_resp_out);	     //shift right vector, push new mean_resp in push last mean_resp out
        avg_normal_mean_resp = get_avg_mean_resp(); //average
        start_resp = 0;
    }
    else
    {
    	set_avg_mean_resp(avg_mean_resp_out);
                             //shift right vector, push new mean_resp in, push last mean_resp out
        avg_normal_mean_resp = get_avg_mean_resp(); //average
        start_resp = 0;
    }
    avg_mean_resp_in = avg_mean_resp_out;
    if((hold_normal_mean == 0)&&(is_normal == 1)&& (is_movements == 0)&&(is_apnea == 0) && (inhale_count > 0))
    {
    	store_normal_mean(avg_mean_resp_out);
    	normal_mean = get_normal_mean();
    }

    float avg_mean_resp_in = avg_mean_resp_out;
    float resp_out[MAX_SIZE_RESP_LIFELINE];
    uint16_t size_resp = 0;
    instant_resp(resplife_data, data_size, avg_normal_mean_resp, avg_mean_resp_in, start_resp, is_normal, hold_normal_mean, resp_out, &size_resp, &avg_mean_resp_out);

    if(size_resp > 2){
		movements_t movements_lifeline[20] = {0};
		float avg_energy_out = 0;
		num_mov_lifeline = search_movements_resp(&resp_out[1], (size_resp-1), avg_energy_in_lf, 10, movements_lifeline, &avg_energy_out);

		if(num_mov_lifeline == 0)
		{
			avg_energy_in_lf = avg_energy_out;
			resp_status = resp_stop_detect(resp_out, (size_resp), eb_th_resp_time, &inhale_count, &is_apnea);
			if(get_nobreathing_setting() == NOBREATH_DISABLE || SettingGetModeOperation().mode == OPE_MODE_MANUAL)
			{
				resp_status = RESP_NORMAL;
			}
		}
		else{
			LOG_DEBUGF("num_mov_lifeline %d", num_mov_lifeline);
			reset_lifeline_resp();
		}
    }
    else{
    	reset_lifeline_resp();
    }
	return resp_status;
}

void find_resp_noise(actual_data_t *resp_data, const uint16_t data_size, float *eb_th_resp_freq, float *eb_th_resp_time)
{
	const uint16_t down_size = data_size/2;

	LcdCtrlLock();
	float32_t *sum_sq_x_resp = NULL;  // for sum of sq x_bpf (method 2)
 	sum_sq_x_resp = (float32_t *)pvPortMalloc(data_size*sizeof(float32_t)); 
	if(sum_sq_x_resp == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sum_sq_x_resp");
		Error_Handler();
	}		
	
	float32_t *sum_sq_x_bpf = NULL;  // for fft input (method1)  and filter results (method2)
 	sum_sq_x_bpf = (float32_t *)pvPortMalloc(data_size*sizeof(float32_t)); 
	if(sum_sq_x_bpf == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sum_sq_x");
		Error_Handler();
	}	
	
	resp_data_t *resp_data_bpf = NULL;  // for sum bfp results -> movements
 	resp_data_bpf = (resp_data_t *)pvPortMalloc(sizeof(resp_data_t)); 
	if(resp_data_bpf == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :resp_data_bpf");
		Error_Handler();
	}	

	float32_t *sum_sq_x_fft = NULL;  // for fft results
 	sum_sq_x_fft = (float32_t *)pvPortMalloc(down_size*sizeof(float32_t)); 
	if(sum_sq_x_fft == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sum_sq_x_fft");
		Error_Handler();
	}		
	
	LcdCtrlUnlock();

	filter_data(BPF_0P7_20, resp_data->x1, resp_data_bpf->x1, data_size);
    filter_data(BPF_0P7_20, resp_data->y1, resp_data_bpf->y1, data_size);
#ifdef PAD_3_SENSOR
	filter_data(BPF_0P7_20, resp_data->x2, resp_data_bpf->x2, data_size);
    filter_data(BPF_0P7_20, resp_data->y2, resp_data_bpf->y2, data_size);
	filter_data(BPF_0P7_20, resp_data->x3, resp_data_bpf->x3, data_size);
    filter_data(BPF_0P7_20, resp_data->y3, resp_data_bpf->y3, data_size);
#endif
	for(uint16_t i = 0; i < data_size; i++)
	{
#ifdef PAD_3_SENSOR
		sum_sq_x_resp[i] = resp_data_bpf->x1[i]*resp_data_bpf->x1[i] + resp_data_bpf->y1[i]*resp_data_bpf->y1[i] + resp_data_bpf->x2[i]*resp_data_bpf->x2[i]
        + resp_data_bpf->y2[i]*resp_data_bpf->y2[i] + resp_data_bpf->x3[i]*resp_data_bpf->x3[i] + resp_data_bpf->y3[i]*resp_data_bpf->y3[i]; // for method 2 (time domain)
#else
		sum_sq_x_resp[i] = resp_data_bpf->x1[i]*resp_data_bpf->x1[i] + resp_data_bpf->y1[i]*resp_data_bpf->y1[i];
#endif
		if(!(i%2))
		{
			sum_sq_x_bpf[i] = sum_sq_x_resp[i];
		}
		else{
			sum_sq_x_bpf[i] = 0;
		}
	}	
	vPortFree(resp_data_bpf);	
	/*Method1 : sum FFT Square X*/
	fft_f32(sum_sq_x_bpf, sum_sq_x_fft, down_size);	

	const uint16_t size_all_pks1 = find_size_peaks(&sum_sq_x_fft[1], down_size);
	//LOG_DEBUGF("size_all_pks %d",size_all_pks1);
	
	Peak_ *all_pks = NULL;
	if(size_all_pks1 > 0){
		all_pks = (Peak_ *)pvPortMalloc(size_all_pks1*sizeof(Peak_));
	}
	if(all_pks == NULL){
		LOG_DEBUGF("Could not allocate memory: all_pks");
	}
	else
	{
		find_all_peaks(&sum_sq_x_fft[1], down_size, all_pks);
		float sum = 0;
		for(uint16_t i = 0; i < size_all_pks1; i++)
		{
			sum += all_pks[i].amp;
		}
		*eb_th_resp_freq = (float)((float)sum/size_all_pks1) * factor_resp_freq;
	}		
	vPortFree(sum_sq_x_fft);
	
	/*--------------------time domain----------------------*/ 
	filter_data(BPF_0P2_2P0, sum_sq_x_resp, sum_sq_x_bpf, data_size);			 //Apply BandPass Filter FIR
	filter_data(LPF_1P0, sum_sq_x_bpf, sum_sq_x_resp, data_size); //Apply Cascade FIR LPF

	vPortFree(sum_sq_x_bpf);	
	
	float xr_resp[100];
	float sum_xr_resp = 0;
	uint16_t count_xr_resp = 0;
	
	const uint16_t size_all_pks_time = find_size_peaks(sum_sq_x_resp, data_size);
//	LOG_DEBUGF("size_all_pks_time %d", size_all_pks_time);
	Peak_ *all_peaks = NULL;  
	if(size_all_pks_time > 0){
 		all_peaks = (Peak_ *)pvPortMalloc(size_all_pks_time*sizeof(Peak_)); 
	}
	if(all_peaks != NULL)
	{
		find_all_peaks(sum_sq_x_resp, data_size, all_peaks);		
		float32_t sum_time_domain = 0;
		for(uint16_t i = 0; i < size_all_pks_time; i++)
		{
			sum_time_domain += all_peaks[i].amp;
		}
		float32_t th_xr_lpf = (float32_t)0.25 *( sum_time_domain/(float32_t)size_all_pks_time); 
//LOG_DEBUGF("th_xr_lpf %f", th_xr_lpf);

		for(uint16_t i = 0; i < size_all_pks_time; i++)
		{			
			if(all_peaks[i].amp > th_xr_lpf)
			{
				xr_resp[count_xr_resp] = all_peaks[i].amp;
				sum_xr_resp += xr_resp[count_xr_resp];
				count_xr_resp++;
				if(count_xr_resp >= 100)
				{
					break;
				}
			}
		}
		
		float mean_resp = (float)sum_xr_resp/count_xr_resp;
//		LOG_DEBUGF("count_xr_resp %d, mean_resp %f", count_xr_resp, mean_resp);
		for(uint16_t i = 0; i < count_xr_resp; i++)
		{
			xr_resp[i] -= mean_resp;
//			LOG_DEBUGF("xr_resp[%d] = %f", i, xr_resp[i]);
		}
		
		const uint16_t size_all_pks_resp = find_size_peaks(xr_resp, count_xr_resp);
		Peak_ *all_pks_resp = NULL;
		if(size_all_pks_resp > 0){
			all_pks_resp = (Peak_ *)pvPortMalloc(size_all_pks_resp*sizeof(Peak_));
		}
		if(all_pks_resp == NULL){
			LOG_DEBUGF("Could not allocate memory: all_pks_resp");
		}
		else
		{
			find_all_peaks(xr_resp, count_xr_resp, all_pks_resp);
			
			float sum_resp = 0;
			for(uint16_t i = 0; i < size_all_pks_resp; i++)
			{
				sum_resp += all_pks_resp[i].amp;
			}
			float avg_pks_time = (float)sum_resp/size_all_pks_resp;
//			LOG_DEBUGF("sum_resp %f, size_all_pks_resp %d, avg_pks_time %f", sum_resp, size_all_pks_resp, avg_pks_time);
			*eb_th_resp_time = (float)avg_pks_time * factor_resp_time;
		}		
	}
	vPortFree(sum_sq_x_resp);
	

}


void find_thresh_mov_resp(actual_data_t* resp_data, const uint16_t data_size, float* energy_mov_resp)
{
	float32_t *sum_sq_xy_resp = NULL;  // for sum of sq x_bpf (method 2)
	LcdCtrlLock();
	sum_sq_xy_resp = (float32_t *)pvPortMalloc(data_size*sizeof(float32_t));
	if(sum_sq_xy_resp == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sum_sq_xy_resp");
		Error_Handler();
	}		
	
	float32_t *sum_sq_xy_bpf = NULL;  // for fft input (method1)  and filter results (method2)
	sum_sq_xy_bpf = (float32_t *)pvPortMalloc(data_size*sizeof(float32_t));
	if(sum_sq_xy_bpf == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :sum_sq_xy_bpf");
		Error_Handler();
	}	
	
	resp_data_t *resp_data_bpf = NULL;  // for sum bfp results -> movements
 	resp_data_bpf = (resp_data_t *)pvPortMalloc(sizeof(resp_data_t)); 
	if(resp_data_bpf == NULL)
	{
		LOG_DEBUGF("Could not allocate memory :resp_data_bpf");
		Error_Handler();
	}

	LcdCtrlUnlock();
	
	filter_data(BPF_0P7_20, resp_data->x1, resp_data_bpf->x1, data_size);
    filter_data(BPF_0P7_20, resp_data->y1, resp_data_bpf->y1, data_size);
#ifdef PAD_3_SENSOR
	filter_data(BPF_0P7_20, resp_data->x2, resp_data_bpf->x2, data_size);
    filter_data(BPF_0P7_20, resp_data->y2, resp_data_bpf->y2, data_size);
	filter_data(BPF_0P7_20, resp_data->x3, resp_data_bpf->x3, data_size);
    filter_data(BPF_0P7_20, resp_data->y3, resp_data_bpf->y3, data_size);

	for(uint16_t i = 0; i < data_size; i++)
	{
		sum_sq_xy_resp[i] = resp_data_bpf->x1[i]*resp_data_bpf->x1[i] + resp_data_bpf->y1[i]*resp_data_bpf->y1[i] + resp_data_bpf->x2[i]*resp_data_bpf->x2[i]
        + resp_data_bpf->y2[i]*resp_data_bpf->y2[i] + resp_data_bpf->x3[i]*resp_data_bpf->x3[i] + resp_data_bpf->y3[i]*resp_data_bpf->y3[i];
    }
#else
	for(uint16_t i = 0; i < data_size; i++)
	{
		sum_sq_xy_resp[i] = resp_data_bpf->x1[i]*resp_data_bpf->x1[i] + resp_data_bpf->y1[i]*resp_data_bpf->y1[i];
    }
#endif
	vPortFree(resp_data_bpf);	
	/*--------------------time domain----------------------*/ 
	filter_data(BPF_0P2_2P0, sum_sq_xy_resp, sum_sq_xy_bpf, data_size);			 //Apply BandPass Filter FIR
	filter_data(LPF_1P0, sum_sq_xy_bpf, sum_sq_xy_resp, data_size); //Apply Cascade FIR LPF

	vPortFree(sum_sq_xy_bpf);
	
	float xr_resp[100];
	float sum_xr_resp = 0;
	uint16_t count_xr_resp = 0;
	
	const uint16_t size_all_pks_time = find_size_peaks(sum_sq_xy_resp, data_size);
	Peak_ *all_peaks = NULL;
	if(size_all_pks_time > 0){
 		all_peaks = (Peak_ *)pvPortMalloc(size_all_pks_time*sizeof(Peak_)); 
	}
	if(all_peaks != NULL)
	{
		find_all_peaks(sum_sq_xy_resp, data_size, all_peaks);
		float32_t sum_time_domain = 0;
		for(uint16_t i = 0; i < size_all_pks_time; i++)
		{
			sum_time_domain += all_peaks[i].amp;
		}
		float32_t th_xr_lpf = (float32_t)0.25 *( sum_time_domain/(float32_t)size_all_pks_time); 

		for(uint16_t i = 0; i < size_all_pks_time; i++)
		{			
			if(all_peaks[i].amp > th_xr_lpf)
			{
				xr_resp[count_xr_resp] = all_peaks[i].amp;
				sum_xr_resp += xr_resp[count_xr_resp];
				count_xr_resp++;
				if(count_xr_resp >= 100)
				{
					break;
				}
			}
		}
		
		float mean_resp = (float)sum_xr_resp/count_xr_resp;
		for(uint16_t i = 0; i < count_xr_resp; i++)
		{
			xr_resp[i] -= mean_resp;
		}
		
		// find threshold mov resp
		movements_t movements[40] = {0};
		
		search_movements_resp(xr_resp, count_xr_resp, (float)10e20, (float)32.768, movements, energy_mov_resp);
	}
	vPortFree(sum_sq_xy_resp);
}
