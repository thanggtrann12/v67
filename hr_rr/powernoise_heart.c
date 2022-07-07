   /************************************************************
   % 
   %OnSky Inc. Confidential and Proprietary
   %Algorithm written by Hung Nguyen
   %C code written by Thien Phan
   %October, 2020
   %
   %This function powernoise_heart.m will calculate average power
   %of noise during empty bed. This value will be saved and to be used
   %in SNR calculation. This function only runs after start-up the 
   %bed sensor and to be updated during the time the bed is empty.
   %****************************************************************
   %
   %heart_window = 1000; %default is 4s for each window 
   %emptybed_data is data vector of emptybed with 10K samples (40s).
   %It will be broken down into 10 4s-windows (1K samples) to 
   %calculate power of noise below.
   %We consider empty bed data is noise.
   %avgpower_eb_4s_out: is average power of noise (emptybed) 
   %for 4s-windows. It will be saved in memory and will be used
   %in snr_heart.m to calculate signal-to-noise ratio.
   %
   %***************************************************************/

#include "main.h"
#include "hr_rr.h"
#include "dsp.h"
#include "lifesos.h"

void powernoise_heart(float32_t* emptybed_sq_data, uint16_t emptydata_size, uint16_t heart_window, float32_t* avgdft_eb)
{
	uint16_t num_windows = (uint16_t)roundf((float32_t)emptydata_size/heart_window);
	
	float32_t sum_dft_eb = 0;
	uint16_t start_win = 0; //start of each sliding window
	
	uint16_t fft_size = 0;
	for(uint16_t i = 0; i < num_windows; i++)
	{
		start_win = heart_window*i; //start of each sliding window
		
		if (heart_window == 1000)   //4s-window
		{
			fft_size = 1024;
		}
		else if (heart_window == 1250)   //5s-window
		{
			 fft_size = 2048;
		}		
		else if (heart_window == 1500)   //6s-window
		{
			 fft_size = 2048;
		}
		else if (heart_window == 2000)  //8s-window
		{
			fft_size = 2048;
		}
		else
		{
			PLOG("data size is not allowed!!!");
			Error_Handler();
		}
		float32_t *input_fft = NULL;
		input_fft = (float32_t *)pvPortMalloc(fft_size*2*sizeof(float32_t)); 
		if(input_fft == NULL)
		{
			PLOG("Could not allocate memory :input_fft");
			Error_Handler();		
		}
		float32_t *output_fft = NULL;
		output_fft = (float32_t *)pvPortMalloc(fft_size*sizeof(float32_t)); 
		if(output_fft == NULL)
		{
			PLOG("Could not allocate memory :output_fft");
			Error_Handler();			
		}
		
		for(uint16_t k = 0; k < (heart_window); k++)
		{
			input_fft[k*2] = emptybed_sq_data[start_win+k];
			input_fft[k*2+1] = 0;
		}
		for(uint16_t k = heart_window*2; k < (fft_size*2); k++)
		{
			input_fft[k] = 0;
		}	

		fft_f32(input_fft, output_fft, fft_size);

		uint16_t point_15Hz = (fft_size * 15) / Fs;
		
//		PLOG("point_15Hz %d",point_15Hz);
		uint16_t size_all_pks = find_size_peaks(&output_fft[3], (point_15Hz-3));
		Peak_ *all_pks = NULL;
		if(size_all_pks > 0){
			all_pks = (Peak_*)pvPortMalloc(size_all_pks * sizeof(Peak_));
		}
		if(all_pks == NULL)
		{
			PLOG("Could not allocate memory :all_pks");
			vPortFree(input_fft); // free pointer input_fft
			vPortFree(output_fft); // free pointer output_fft			
			return;
		}			
		
		find_all_peaks(&output_fft[3], (point_15Hz - 3) , all_pks);
		
		vPortFree(input_fft); // free pointer input_fft
		vPortFree(output_fft); // free pointer output_fft
		
		float32_t sum_pks = 0;
		for(uint16_t k = 0; k < size_all_pks; k ++)
		{
//			PLOG("all_pks[%d].amp = %f, size_all_pks %d",k,  all_pks[k].amp, size_all_pks);
			sum_pks += all_pks[k].amp;
		}
				
		sum_dft_eb += sum_pks/(float32_t)size_all_pks;
		vPortFree(all_pks); // free pointer all_peaks
	}
	*avgdft_eb = sum_dft_eb/(float32_t)num_windows;
	PLOG("avgdft_eb %f", *avgdft_eb);
}
