#ifndef __SLEEP_STAGES_H
#define __SLEEP_STAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"

#define SLEEP_STAGE_MAX_SIZE 150

typedef enum{
	SLEEP_STATUS_OK,
	SLEEP_STATUS_NOT_READY,
	SLEEP_STATUS_ERROR,
}SleepStageStatus_t;

typedef enum{
	/* Sleep stages */
	SLEEP_STAGE_HIDDEN,
	SLEEP_STAGE_AWAKE_TYPE,
	SLEEP_STAGE_REM_TYPE,
	SLEEP_STAGE_LIGHT_TYPE,
	SLEEP_STAGE_DEEP_TYPE,
	SLEEP_STAGE_REM_AWAKE_TYPE,
	SLEEP_STAGE_AWAKE_REM_TYPE,
	SLEEP_STAGE_AWAKE_DEEP_TYPE,
}SleepStageType_t;

typedef struct{
    uint8_t hrMax[SLEEP_STAGE_MAX_SIZE];
    uint8_t hrAvg[SLEEP_STAGE_MAX_SIZE];
    uint8_t hrMin[SLEEP_STAGE_MAX_SIZE];
    uint8_t rrMax[SLEEP_STAGE_MAX_SIZE];
    uint8_t rrAvg[SLEEP_STAGE_MAX_SIZE];
    uint8_t rrMin[SLEEP_STAGE_MAX_SIZE];
    int8_t bodyTempVarMax[SLEEP_STAGE_MAX_SIZE];
    int8_t bodyTempVarAvg[SLEEP_STAGE_MAX_SIZE];
    int8_t bodyTempVarMin[SLEEP_STAGE_MAX_SIZE];
    int8_t bodyTempMax[SLEEP_STAGE_MAX_SIZE];
    int8_t bodyTempAvg[SLEEP_STAGE_MAX_SIZE];
    int8_t bodyTempMin[SLEEP_STAGE_MAX_SIZE];
    int8_t roomTempAvg[SLEEP_STAGE_MAX_SIZE];
    uint16_t motions[SLEEP_STAGE_MAX_SIZE];
    uint16_t apnea[SLEEP_STAGE_MAX_SIZE];
    uint16_t snoring[SLEEP_STAGE_MAX_SIZE];
    int32_t pnpBgWeak[SLEEP_STAGE_MAX_SIZE];
    int32_t pnpBgOk[SLEEP_STAGE_MAX_SIZE];
    int32_t pnpBgGood[SLEEP_STAGE_MAX_SIZE];
    int32_t pnpBgVGood[SLEEP_STAGE_MAX_SIZE];
    int32_t pnpBgExcel[SLEEP_STAGE_MAX_SIZE];
    float movEnergyMax[SLEEP_STAGE_MAX_SIZE];
    float movPeakMax[SLEEP_STAGE_MAX_SIZE];
    uint32_t unixTime[SLEEP_STAGE_MAX_SIZE];
}SleepStageData_t;

typedef struct{
    uint32_t totalTime;
    uint32_t totalSleepTime;
    uint32_t timeAwake;
    uint32_t timeRem;
    uint32_t timeLight;
    uint32_t timeDeep;

    uint8_t percentAwake;
    uint8_t percentRem;
    uint8_t percentLight;
    uint8_t percentDeep;

    uint8_t scoreApnea;
    uint8_t scoreSnoring;
    uint8_t scoreHr;
    uint8_t scoreRem;
    uint8_t scoreDeep;
    uint8_t scoreSleepTime;
    uint8_t scoreBtc;
    uint8_t scoreRoomTemp;

    uint8_t perfectScore;
    uint8_t percentScore;
    uint8_t worstScore1;
    uint8_t worstScore2;
    uint16_t totalApnea;
    uint16_t totalSnoring;
    uint16_t totalSnoringPerHour;

    uint8_t avgHr;
    uint8_t avgRr;
	uint8_t allHrMax;
	uint8_t allHrMin;
    int8_t btcMax;
    int8_t btcMin;
    int8_t avgRoomTemp;
    int8_t avgBodyTemp;

    uint8_t vitalSinal[5];

    SleepStageType_t sleepStageArr[SLEEP_STAGE_MAX_SIZE];
    uint8_t bedArr[SLEEP_STAGE_MAX_SIZE];
    uint8_t hrArr[SLEEP_STAGE_MAX_SIZE];
    uint8_t rrArr[SLEEP_STAGE_MAX_SIZE];
    uint8_t snoringArr[SLEEP_STAGE_MAX_SIZE];
    uint16_t apneaArr[SLEEP_STAGE_MAX_SIZE];
    int8_t btcArr[SLEEP_STAGE_MAX_SIZE];
    uint8_t roomTempArr[SLEEP_STAGE_MAX_SIZE];
    uint32_t unixTime[SLEEP_STAGE_MAX_SIZE];
}__attribute__((packed))SleepStageOutput_t;

typedef struct{
	uint8_t worstScore1;
	uint8_t worstScore2;
	float minusMax1;
	float minusMax2;
}__attribute__((packed))WorstScore_t;

typedef struct{
	uint32_t timeStart;
	uint32_t timeEnd;
}SleepScoreTime_t;

int SleepStagesProcess();
void SleepStagesCal();
int SleepStagesParseData();
void SleepStagesUpdateLcd();
SleepScoreTime_t SleepScoreTime();
int SleepStagesParseSleepScoreData(SleepStageOutput_t *sleepInfo);
void SleepStagesGetSleepScoreAndUpdateLcd();

#ifdef __cplusplus
}
#endif

#endif
