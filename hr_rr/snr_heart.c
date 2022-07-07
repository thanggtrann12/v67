   /************************************************************
   % 
   %OnSky Inc. Confidential and Proprietary
   %Algorithm written by Hung Nguyen
   %C code written by Thien Phan
   %October, 2020
   %
   %snr_heart.m is a Matlab function
   %It includes algorithm to calculate Signal-to-Noise Ratio (SNR).
   %It is used to indicate whether the heart signal (ADC output)
   %is good or not. If "signal_weak" = 1, then the bed sensor
   %will send warning to user. 
   %SNR will indicate if signal is too low and it indicates 
   %heart is stopping, that is an emergency.
   %We use SNR, RSSI and Noise from communication theory and apply
   %them to bed sensing technique to know the quality of the 
   %heart signals. 
   %
   %
   %****************************************************************/
#include "main.h"
#include "hr_rr.h"
#include "lifesos.h"
#include "dsp.h"

void snr_heart(float32_t *heart_square_4s, uint16_t data_size, float32_t noise_dft, float32_t *snr_dft)
{
	uint16_t fft_size = 0;
	if (data_size == 1000)   //4s-window
	{
		fft_size = 1024;
	}
	else if (data_size == 1250)   //5s-window
	{
		 fft_size = 2048;
	}		
	else if (data_size == 1500)   //6s-window
	{
		 fft_size = 2048;
	}
	else if (data_size == 2000)  //8s-window
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
	
	for(uint16_t k = 0; k < (data_size); k++)
	{
		input_fft[k*2] = heart_square_4s[k];
		input_fft[k*2+1] = 0;
	}
	for(uint16_t k = data_size*2; k < (fft_size*2); k++)
	{
		input_fft[k] = 0;
	}			
	
	fft_f32(input_fft, output_fft, fft_size);

	uint16_t point_15Hz = (uint16_t)(fft_size * 15) / Fs;//default: 60
//	PLOG("point_15Hz %d", point_15Hz);
	
	uint16_t size_all_pks = find_size_peaks(&output_fft[3], (point_15Hz - 2));
//	PLOG("snr_heart: size_all_pks = %d", size_all_pks);
	
	Peak_ *all_pks = NULL;
	if(size_all_pks > 0){
		all_pks = (Peak_*)pvPortMalloc(size_all_pks * sizeof(Peak_));
	}
	if(all_pks == NULL)
	{
		vPortFree(input_fft); // free pointer input_fft
		vPortFree(output_fft); // free pointer output_fft		
		PLOG("Could not allocate memory :all_pks");
		return;
	}	
	
	find_all_peaks(&output_fft[3], (point_15Hz - 2) , all_pks);
	
	vPortFree(input_fft); // free pointer input_fft
	vPortFree(output_fft); // free pointer output_fft

	float32_t max_pk_dft = 0;
	for(uint16_t i = 0; i < size_all_pks; i++)
	{
//		PLOG("snr_heart: all_pks[%d].amp = %f", i, all_pks[i].amp);
		if(all_pks[i].amp > max_pk_dft)
		{
			max_pk_dft = all_pks[i].amp;
		}
	}
	vPortFree(all_pks); // free pointer all_peaks
   /********************************************************
   %Calculate SNR using max_pk_dft - noise_dft
   %max_pk_dft: maximum peak in DFT of 4s-windows.
   %noise_dft: avgnoise_4s, that is average noise level (DFT) of empty bed data (4s-windows).
   %*******************************************************/	
	*snr_dft = max_pk_dft - noise_dft;
}
