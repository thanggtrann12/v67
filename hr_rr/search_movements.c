 /************************************************************
 % 
 %OnSky Inc. Confidential and Proprietary
 %Algorithm written by Hung Nguyen
 %C code written by Thien Phan
 %October, 2020
 %
 %search_movements is a Matlab function that searches for 
 %movement segments inside a data block (heart_data_bpf).
 %It calculates energy of sliding windows and compare it with
 %average energy from previous data block. If the energy of a
 %sliding window is greater than the threshold 'move_th' then
 %it will mark that window as movement and the start location
 %and end location of that window will be entered into the 
 %'movements' output vector. 
 %
 %If there is no movements in that data block, then output
 %movements = [0 0].
 %If there is movements then output movements will be like this:
 %movements = [loc_start1   loc_end1
 %             loc_start2   loc_end2
 %             loc_start3   loc_end3]
 %**************************************************************/
 
#include "main.h"
#include "hr_rr.h"

uint8_t search_movements(const float *heart_data_bpf,const uint16_t data_size, float avg_energy, movements_t* movements, float* avg_energy_out, float* avg_energy_all_out, float *maxPeakOut)
{
	const float *heart_data = heart_data_bpf;
	uint8_t num_moves = 0;
	uint16_t window_size = WINDOW_SIZE_MOV;  //250 samples (250 * 0.004s = 1 seconds)
	float move_th = avg_energy*12; //threshold to indicate movements or not
                             //Note: need to adjust this value by 
                             //doing more experiments	
	
	uint8_t slide_windows = (data_size/ window_size); //number of sliding windows
//	PLOG("move_th %f, slide_windows %d", move_th, slide_windows);
	uint16_t start_wck = 0;
	uint16_t end_wck = 0;
	float rssi = 0, sum_energy = 0, sum_all_energy = 0, power = 0, maxPeak = 0;
	for(uint8_t i = 0; i < slide_windows; i++)
	{
		start_wck = window_size*i; //start of each sliding window
		end_wck = start_wck + window_size; //end of each sliding window
    //Calculate Energy for each sliding window
    //use RSSI: Receive Signal Strength Indication (from Signal Processing theory)		
		rssi = 0;//energy of sliding window 
		//PLOG("start_wck %d, end_wck %d", start_wck, end_wck);
		for(uint16_t k = start_wck; k < end_wck; k++)
		{
			power = heart_data[k]*heart_data[k];
			rssi += power;
			if(maxPeak < power)
			{
				maxPeak = power;
			}
		}
//		PLOG("rssi %f", rssi);
		//Find sliding window that has movement
		if (rssi > move_th)    //energy is too high due to movements
		{
			movements[num_moves].loc_start = start_wck;
			movements[num_moves].loc_end = end_wck;
			num_moves++;
		}
		else
		{
			sum_energy = sum_energy + rssi;
		}
		sum_all_energy += rssi;
	}
	if(slide_windows != num_moves){
		*avg_energy_out = sum_energy / (float)(slide_windows-num_moves); 
	}
	*avg_energy_all_out = sum_all_energy/slide_windows;
	if(num_moves > 0)
	{
		*maxPeakOut = maxPeak;
	}
	else{
		*maxPeakOut = 0;
	}
//	PLOG("num_moves %d", num_moves);
	
	return num_moves;
}
