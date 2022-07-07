#ifndef INC_SNORING_H_
#define INC_SNORING_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdbool.h"
#include "stdint.h"
#include "LcdUpdate.h"

bool SnoringIsReady();
void SnoringGetAdc();
void SnoringStartTimerGetAdc();
void SnoringStopTimerGetAdc();
void SnoringStartDmaAdc();
void SnoringStopDmaAdc();
void SnoringStartGetAdc();
void SnoringStopGetAdc();
SnoringLevel_t SnoringDetect(uint8_t *snorNoise);
void SnoringTest();
#ifdef __cplusplus
}
#endif

#endif /* INC_UTILS_H_ */
/*@}*/

