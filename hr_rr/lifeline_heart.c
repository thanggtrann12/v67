   /************************************************************
   % 
   %OnSky Inc. Confidential and Proprietary
   %Algorithm written by Hung Nguyen
   %C code written by Thien Phan
   %October, 2020
   %
   %
   %
   %****************************************************************/
	
#include "main.h"
#include "hr_rr.h"
#include "dsp.h"
#include "lifesos.h"
#include "Utils.h"

bool lifeline_heart(float32_t *sq_data, uint16_t heart_data_size, movements_t *movements, uint8_t num_movs, float32_t th_adapt, float32_t noise_dft, float32_t avgsq_noise, bool *occupied, lavie_data_t *lavie_data, bool good_found)
{	
	static uint8_t previous_beats[3] = {4, 4, 4}; // about 4 beats per 4s-window
	static uint16_t previous_snr[3] = {5000, 5000, 5000};		 // default values
  static uint16_t  timegap_in = 100;
	uint16_t timegap_out = 0;    //default value for timegap (= max_timegap), assuming 
	
	uint16_t num_beats_th = (uint16_t)roundf((float)heart_data_size*2/1000);
	// threshold: if number of heart beats is below 2 per 4s-window, then it is emergency
	float32_t life_dft_th = noise_dft*10;
	
//	float32_t th_adapt = 0; // use estimaetd avg_pks in hr_detetct, no need to use pnp.m
	
  bool xz_add = true;     //this is combined waveforms with multiple sensors

  //Find signal-to-noise ratio and signal - noise	
	float32_t snr_dft = 0;
	snr_heart(sq_data, heart_data_size, noise_dft, &snr_dft);
//	LOG_DEBUGF("noise_dft %f, snr_dft %f", noise_dft, snr_dft);
	
	uint16_t heart_beats = 0;	
	bool heart_lowsig = false;
	uint8_t Heart_Rate = 0;
	float HRV = 0;
	hr_detect(sq_data, heart_data_size, th_adapt, movements, num_movs, xz_add, timegap_in, avgsq_noise, &heart_lowsig, &timegap_out, &heart_beats, &Heart_Rate, &HRV);	



	LOG_DEBUGF("heart_beats : %d, Instant_hr : %d, lowsig: %d, timegap_out: %d",heart_beats, Heart_Rate, heart_lowsig, timegap_out);

	if(Heart_Rate < 40 || Heart_Rate > 200 || heart_lowsig == true)
	{
		LOG_DEBUGF("HR Error case");
		lavie_data->hr_rr.actual_hr = ERROR_CASE; 
	}	
	else
	{
		if(num_movs == 0)
		{
			if(good_found == true) // if data is not good and has movements -> don't show instant HR
			{
				lavie_data->hr_rr.instant_hr = Heart_Rate;
				if(timegap_out > 40 && timegap_out < 150)
				{
					LOG_DEBUGF("timegap_out %d", timegap_out);
					timegap_in = timegap_out;
				}
			}
			else
			{
				LOG_DEBUGF("signal instant weak");
				lavie_data->hr_rr.instant_hr = WEAK_CASE; // data weak
			}					
		}
		else
		{
			lavie_data->hr_rr.instant_hr = PREVIOUS_CASE;
		}
	}

	
 /***************************************************************
 'occupied_4s' vector indicates the status of the capacitive sensor
 for each time-sample that the bed has a person lying on it or
 not (occupied = 1 (there is a person); occupied = 0 (empty bed).
 
 If 'occupied_4s' is equal 1 partially within 4s-window, then 
 'occupied_win' = 0; if 'occupied_4s' is fully equaly 1 within
 4s-window then 'occupied_win' = 1. This is to make sure that
 occupied_win signal is stable.
 ***************************************************************/
	
	bool occupied_win = true;
	for(uint16_t i = 0; i < heart_data_size; i++)
	{
		if(occupied[i] == false)
		{
			occupied_win = false; // one or some occupied_4s samples is 0
			break;
		}
	}


   /*************************************************************
   Checking for heart stopping condition.
   Checking number of heart beats	*/
	bool previous_heart_flag = false;
	if((previous_beats[0] >= num_beats_th) || (previous_beats[1] >= num_beats_th) || (previous_beats[2] >= num_beats_th))
	{
		previous_heart_flag = true;
	}
//	LOG_DEBUGF("previous_heart_flag %d", previous_heart_flag);
	
	//default num_beats_th = 2 (per 4s),
	//this condition means heart rate is
	//less than 40BPM, that is too low
	bool hr_too_low_flag = false;
	static bool pre_hr_too_low_flag = false;
	if( ((heart_beats < num_beats_th) || (heart_lowsig == true)) && (occupied_win == true) )
	{
		hr_too_low_flag = true;
	}
//	LOG_DEBUGF("hr_too_low_flag %d", hr_too_low_flag);
	
  //Checking SNR
	bool previous_snr_flag = false;
	if(previous_snr[0] >= life_dft_th || previous_snr[1] >= life_dft_th || previous_snr[2] >= life_dft_th)
	{
		previous_snr_flag = true;
	}	
//	LOG_DEBUGF("previous_snr_flag %d", previous_snr_flag);
	
	// default snr threshold is 33dB (per 4s). This conditon means heart signal (enery) is too slow
	bool snr_too_low_flag = false;
	static bool pre_snr_too_low_flag = false;
	if( snr_dft < life_dft_th && occupied_win == true)
	{
		snr_too_low_flag = true;
	}
//	LOG_DEBUGF("snr_too_low_flag %d", snr_too_low_flag);
	
	/*Heart stopping condition is when number of beats is too low 
	(less than 2 per 4s-window) OR SNR is too low (less than noise)*/
	bool heart_stop = false, heart_stop1 = false, heart_stop2 = false, heart_stop3 = false;

   if ((hr_too_low_flag == true) && (snr_too_low_flag == true)) 
	 {
                          //both hr and snr are too low at same time
      heart_stop1 = true;    //this is emergency condition	
	 }
   if ((hr_too_low_flag == true) && (pre_hr_too_low_flag == true)) 
	 {                         //hr_too_low for 2 consecutive windows
      heart_stop2 = true;    //this is also emergency 
	 }
   if ((snr_too_low_flag == true) && (pre_snr_too_low_flag == true)) 
	 {
                          //snr_too_low for 2 consecutive windows
      heart_stop3 = true;    //this is also emergency 
	 }	
	LOG_DEBUGF("heart_stop1 = %d, heart_stop2 = %d, heart_stop3 = %d, previous_snr_flag = %d, previous_heart_flag = %d",
	heart_stop1, heart_stop2, heart_stop3, previous_snr_flag, previous_heart_flag);
	if( ((heart_stop1 == true)||(heart_stop2 == true)||(heart_stop3 == true)) //
		&& ((previous_snr_flag == true)||(previous_heart_flag == true))) 	 
	{
		heart_stop = true;
	}

	pre_hr_too_low_flag = hr_too_low_flag;
	pre_snr_too_low_flag = snr_too_low_flag;
	
	previous_beats[0] = previous_beats[1];
	previous_beats[1] = previous_beats[2];
	previous_beats[2] = heart_beats;
	
	previous_snr[0] = previous_snr[1];
	previous_snr[1] = previous_snr[2];
	previous_snr[2] = snr_dft;
	
	return heart_stop;
}

