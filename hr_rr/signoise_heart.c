   /************************************************************
   % 
   %OnSky Inc. Confidential and Proprietary
   %Algorithm written by Hung Nguyen
   %C code written by Thien Phan
   %October, 2020
   %
   %This function calculates the average noise level of a signal
   %vector in time domain. The signal can be ADC, ADC after BPF, or
   %square data. 
   %Note: this signoise_heart is different from powernoise_heart.m
   %that it calculates average signal level in time domain while
   %the powernoise_hear is in frequency domain (DFT).
   %
   %
   %***************************************************************/
#include "main.h"
#include "hr_rr.h"
#include "dsp.h"


float32_t signoise_heart(float32_t *signal_data, uint32_t size_of_data)
{
	float32_t avgsig_noise = 0;
	
	uint16_t size_all_peaks = find_size_peaks(signal_data, size_of_data);
	Peak_ *all_peaks = NULL;
	if(size_all_peaks > 0)
	{
		all_peaks = (Peak_*)pvPortMalloc(size_all_peaks * sizeof(Peak_));
	}
	if(all_peaks == NULL)
	{
		PLOG("Could not allocate memory :all_peaks");
		return avgsig_noise;		
	}
	
	find_all_peaks(signal_data, size_of_data, all_peaks);
	float32_t sum_peaks = 0;
	for(uint16_t i = 0; i < size_all_peaks; i++)
	{
		sum_peaks += all_peaks[i].amp;
	}
	avgsig_noise = sum_peaks/(float32_t)size_all_peaks;
	
	vPortFree(all_peaks);
	return avgsig_noise;
}
