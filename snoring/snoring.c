#include "tim.h"
#include "adc.h"
#include "Utils.h"
#include "util/util.h"
#include "snoring.h"
#include "dsp.h"


#define	SNOR_HIGH_TH	(float)500.0//threshold for high snoring (power)
#define	SNOR_MEDIUM_TH	(float)150.0
#define	SNOR_LOW_TH		(float)25.0
#define	SNOR_NOISY_TH	(float)100.0

#define SIZE_ADC_MIC_DATA (uint32_t)8192
static __IO uint32_t ADCValue = 0, indexMicData = 0;
static float MicData[SIZE_ADC_MIC_DATA+2] __attribute__((section(STR("_raw_mic"))));

void SnoringGetAdc(){
	if(indexMicData < SIZE_ADC_MIC_DATA)
	{
		SCB_InvalidateDCache_by_Addr((uint32_t*)&ADCValue, 1);
		MicData[indexMicData++] = (float)ADCValue;
		SCB_CleanDCache_by_Addr((uint32_t *)&ADCValue, 1);
	}
}

static void SnoringResetIndex(){
	indexMicData = 0;
}

bool SnoringIsReady(){
	if(indexMicData >= SIZE_ADC_MIC_DATA){
		return true;
	}
	return false;
}

void SnoringStartTimerGetAdc(){
	SnoringResetIndex();
	HAL_TIM_Base_Start_IT(&htim7);
}

void SnoringStopTimerGetAdc(){
	SnoringResetIndex();
	HAL_TIM_Base_Stop_IT(&htim7);
}

void SnoringStartDmaAdc(){
	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCValue, 1) != HAL_OK)
	{
	/* Start Conversation Error */
		LOG_ERROR("Cannot start dma adc");
	}
}

void SnoringStopDmaAdc(){
	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCValue, 1) != HAL_OK)
	{
	/* Start Conversation Error */
		LOG_ERROR("Cannot stop dma adc");
	}
}

void SnoringStartGetAdc(){
	HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
	SnoringStartDmaAdc();
	SnoringStartTimerGetAdc();
}

void SnoringStopGetAdc(){
	SnoringStopDmaAdc();
	SnoringStopTimerGetAdc();
}

void SnoringTest(){
//	bool tui = 0;
//	detectSnoring(&tui);
	SnoringStartGetAdc();
	while(!SnoringIsReady());
	for(uint16_t i = 0; i < SIZE_ADC_MIC_DATA; i++)
	{
		printf("%f\r\n", MicData[i]);
	}
	while(1){};
}

#define SIZE_PEAK_SNORING (uint16_t)36 //end at byte 40,  start from byte 4 // dataOutFft[3]
#define SIZE_PEAK_NOISE (uint16_t)15 // end byte 30, start from byte 15 // dataOutFft[14]
SnoringLevel_t SnoringDetect(uint8_t *snorNoise)
{
	LOG_DEBUGF("---Start detect snoring---");
	SnoringLevel_t SnoringLevel = SNORING_NONE;
	const uint16_t dataSize = SIZE_ADC_MIC_DATA;

	*snorNoise = 0;
	float* adcMicData = MicData;

	float *adcMicDataBpf = NULL;  // for filter results and square results
	adcMicDataBpf = (float *)pvPortMalloc(dataSize*sizeof(float));
	if(adcMicDataBpf == NULL)
	{
		LOG_DEBUGF((char*)"Could not allocate memory :adcMicDataBpf");
		Error_Handler();
	}

	float *dataInFft = NULL;  // for filter results and square results
	dataInFft = (float *)pvPortMalloc(dataSize*sizeof(float));
	if(dataInFft == NULL)
	{
		LOG_DEBUGF((char*)"Could not allocate memory :dataInFft");
		Error_Handler();
	}

	const uint16_t downSize = dataSize/2;
	float *dataOutFft = NULL;  // for filter results and square results
	dataOutFft = (float *)pvPortMalloc(downSize*sizeof(float));
	if(dataOutFft == NULL)
	{
		LOG_DEBUGF((char*)"Could not allocate memory :dataOutFft");
		Error_Handler();
	}

	filter_data(BPF_0P05_1P0, adcMicData, adcMicDataBpf, dataSize);

	for(uint16_t i = 0; i < dataSize; i++){
		if(!(i%2))
		{
			dataInFft[i] = adcMicDataBpf[i];
		}
		else{
			dataInFft[i] = 0;
		}
	}
	vPortFree(adcMicDataBpf); // Free adcMicDataBpf
	fft_f32(dataInFft, dataOutFft, downSize);
	vPortFree(dataInFft);

	Peak_ allPeak[SIZE_PEAK_SNORING];
	const uint16_t sizeAllPeaks = find_all_peaks(&dataOutFft[3], SIZE_PEAK_SNORING, allPeak);

	uint16_t indexPossSnoring = 0;
	float maxPeak = 0;
	float possibleSnoring[SIZE_PEAK_SNORING];
	if(sizeAllPeaks == 0)
	{
		SnoringLevel = SNORING_NONE;
	}
	else{
		for(uint16_t i = 0; i < sizeAllPeaks; i++)
		{
			//printf("allPeakSnor[%d] = %f\r\n", i, allPeak[i].amp);
			if(allPeak[i].amp > SNOR_LOW_TH)
			{
				possibleSnoring[indexPossSnoring++] = allPeak[i].amp;
			}
			if(allPeak[i].amp > maxPeak){
				maxPeak = allPeak[i].amp;
			}
		}
		if(indexPossSnoring == 0)
		{
			SnoringLevel = SNORING_NONE;
		}
		else{
			LOG_DEBUGF("maxPeak %f", maxPeak);
			uint8_t countS = 0;
            for(uint16_t k = 0; k < indexPossSnoring; k++)
            {
                if (possibleSnoring[k] > (maxPeak*0.6)) //60% of max_pks
                {
                    countS++;
            	}
            }
            if(countS <= 3){
            	if(maxPeak > SNOR_HIGH_TH){
            		SnoringLevel = SNORING_HIGH;
            	}
            	else if(maxPeak > SNOR_MEDIUM_TH){
            		SnoringLevel = SNORING_MEDIUM;
            	}
            	else{
            		SnoringLevel = SNORING_LOW;
            	}
            }
		}
	}


	Peak_ allPeakNoise[SIZE_PEAK_NOISE];
	const uint16_t sizeAllPeaksNoise = find_all_peaks(&dataOutFft[14], SIZE_PEAK_NOISE, allPeakNoise);
	float avgNoise = 0;
	if(sizeAllPeaksNoise != 0)
	{
		for(uint16_t i = 0; i < sizeAllPeaksNoise; i++){
			//printf("allPeak[%d] = %f\r\n", i, allPeakNoise[i].amp);
			avgNoise += allPeakNoise[i].amp;
		}
		avgNoise /= sizeAllPeaksNoise;
		//printf("avgNoise = %f\r\n", avgNoise);
	}
	if(avgNoise > SNOR_NOISY_TH){
		*snorNoise = 1;
	}
	vPortFree(dataOutFft); // Free adcMicDataBpf
	LOG_DEBUGF("Snoring Level: %d, snorNoise %d", SnoringLevel, *snorNoise);
	if(*snorNoise == 1){
		SnoringLevel = SNORING_NONE;
	}
	SnoringResetIndex();
	return SnoringLevel;
}



