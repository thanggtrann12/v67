#include "Rtos.h"
#include "define.h"
#include <util/util.h>
#include "Utils.h"
#include "dsp.h"
#include "SleepStages.h"
#include "Server.h"
#include "cJSON.h"
#include "rtcLavie.h"
#include "settings.h"
#include "MacAddress.h"
#include "LcdUpdate.h"
#include "OslavieMQTTClient.h"

#define SLEEP_TH_MOVENERGY_FACTOR   0.5
#define SLEEP_TH_REM_AMP_FACTOR    0.4
#define SLEEP_TH_SLOPE_UP_FACTOR    0.5
#define SLEEP_TH_SLOPE_DOWN_FACTOR    0.5
#define SLEEP_TH_MOTION_FACTOR    0.5

#define SLEEP_TH_EMPTY_BED_TIME    605
#define SLEEP_TIME_INTERVAL    10

#define SLEEP_PT_APNEA_LOW 0.25
#define SLEEP_PT_APNEA_HIGH 1.0
#define SLEEP_SCORE_DEFAULT_APNEA 9

#define SLEEP_PT_SNORING 0.25
#define SLEEP_SCORE_NO_SNORING 9

#define SLEEP_PT_HR 1.0
#define SLEEP_HR_HIGH 85 //normal high BPM
#define SLEEP_HR_LOW 50 //normal low BPM
#define SLEEP_SCORE_DEFAULT_HR 9 //default total score

#define SLEEP_PT_REM_MINUS 1.00
#define SLEEP_PT_REM_PLUS  0.5
#define SLEEP_NORMAL_REM_LOW 20 //normal rem time: 20% of total sleep time
#define SLEEP_NORMAL_REM_HIGH 27 //normal rem time: 25% of total sleep time
#define SLEEP_SCORE_DEFAULT_REM 9		//total score defaul is 10 points

#define SLEEP_PT_DEEP_MINUS 1.0
#define SLEEP_PT_DEEP_PLUS  0.5
#define SLEEP_NORMAL_DEEP_LOW 15 //normal rem time: 13% of total sleep time
#define SLEEP_NORMAL_DEEP_HIGH 23 //normal rem time: 23% of total sleep time
#define SLEEP_SCORE_DEFAULT_DEEP 9		//total score defaul is 10 points

#define SLEEP_PT_SLEEPTIME_MINUS 1.00 //per 60 minutes
#define SLEEP_PT_SLEEPTIME_PLUS  0.5
#define SLEEP_NORMAL_SLEEPTIME_LOW 360 //360 minutes = 6.0 hours
#define SLEEP_NORMAL_SLEEPTIME_HIGH 480 //480 minutes = 8.0 hours
#define SLEEP_SCORE_DEFAULT_SLEEPTIME 9		//total score defaul is 10 points

#define SLEEP_PT_BTC 2.00 //btc: body temperature change
#define SLEEP_NORMAL_BTC_LOW -1 //normal btc between -1C to +1C
#define SLEEP_NORMAL_BTC_HIGH 1 //480 minutes = 8.0 hours
#define SLEEP_SCORE_DEFAULT_BTC 9		//total score defaul is 10 points

#define SLEEP_PT_ROOMTEMP 1.0
#define SLEEP_NORMAL_ROOMTEMP_LOW 19
#define SLEEP_NORMAL_ROOMTEMP_HIGH 23
#define SLEEP_SCORE_DEFAULT_ROOMTEMP 9

static float SleepScoreApnea(uint16_t totalApnea, WorstScore_t* worstScore);
static float SleepScoreSnoring(uint16_t totalSnoring, float totalSnoringPerHour, WorstScore_t* worstScore);
static float SleepScoreHr(uint8_t maxAllHr, uint8_t minAllHr, WorstScore_t* worstScore);
static float SleepScoreRem(float percentRem, WorstScore_t* worstScore);
static float SleepScoreDeep(float percentDeep, WorstScore_t* worstScore);
static float SleepScoreSleepTime(float totalSleepTime, WorstScore_t* worstScore);
static float SleepScoreBtc(int8_t maxBtc, int8_t minBtc, WorstScore_t* worstScore);
static float SleepScoreRoomTemp(int16_t avgAllRoomTemp, WorstScore_t* worstScore);
static int SleepStagesReponse(SleepStageOutput_t* SleepStageOutput, uint16_t dataIndex);


static SleepStageData_t SleepStageDataArr __attribute__((section(STR("_sleep_data"))));
static SleepStageOutput_t SleepStageOutput __attribute__((section(STR("_sleep_data"))));
static uint16_t dataIndex = 0;
static bool sleep9Flag, sleep21Flag;
static SleepScoreTime_t timeStartEnd = {0,0};

SleepScoreTime_t SleepScoreTime(){
	return timeStartEnd;
}


int SleepStagesParseData(){
	dataIndex = 0;
	HttpRpData_t hisData;
	Time_t currentTime = RTC_get_localTime();

	memset(&SleepStageDataArr,0,sizeof(SleepStageDataArr));
	memset(&SleepStageOutput,0,sizeof(SleepStageOutput));

	time_t timeStartUnix, timeEndUnix;
	if((currentTime.tm_hour >= 9) && (currentTime.tm_hour < 21) && (sleep9Flag == true) && (sleep21Flag == true)){
		sleep9Flag = false;
		sleep21Flag = false;
	}

	if((sleep21Flag == false) && (currentTime.tm_hour >= 21) && (currentTime.tm_hour <= 23)){
		currentTime.tm_hour = 21;
		currentTime.tm_min = 0;
		currentTime.tm_sec = 0;
		timeEndUnix = mktime(&currentTime) - (SettingGetTimeZone() * 3600) - 1;
		timeStartUnix = timeEndUnix - 86400 + 1;

		LOG_DEBUGF("9PM: timeStartUnix %lu, timeEndUnix: %lu", (uint32_t)timeStartUnix, (uint32_t)timeEndUnix);
		if(ServerRequestGetHistoryRaw((uint32_t)timeStartUnix, (uint32_t)timeEndUnix, &hisData) == SUCCESS){
			sleep9Flag = true;
			sleep21Flag = true;
			LOG_DEBUGF("sleep9Flag %d, sleep21Flag: %d", sleep9Flag, sleep21Flag);
		}
		else{
			return SLEEP_STATUS_ERROR;
		}
	}
	else if((sleep21Flag == false) && (currentTime.tm_hour < 9)){
		currentTime.tm_hour = 21;
		currentTime.tm_min = 0;
		currentTime.tm_sec = 0;
		timeEndUnix = mktime(&currentTime) - (SettingGetTimeZone() * 3600) - 86400 - 1;
		timeStartUnix = timeEndUnix - 86400 + 1;

		LOG_DEBUGF("9PM: timeStartUnix %lu, timeEndUnix: %lu", (uint32_t)timeStartUnix, (uint32_t)timeEndUnix);
		if(ServerRequestGetHistoryRaw((uint32_t)timeStartUnix, (uint32_t)timeEndUnix, &hisData) == SUCCESS){
			sleep9Flag = true;
			sleep21Flag = true;
			LOG_DEBUGF("sleep9Flag %d, sleep21Flag: %d", sleep9Flag, sleep21Flag);
		}
		else{
			return SLEEP_STATUS_ERROR;
		}
	}
	else if((sleep9Flag == false) && (currentTime.tm_hour >= 9)){
		currentTime.tm_hour = 9;
		currentTime.tm_min = 0;
		currentTime.tm_sec = 0;
		timeEndUnix = mktime(&currentTime) - (SettingGetTimeZone() * 3600) - 1;
		timeStartUnix = timeEndUnix - 43200 + 1;

		LOG_DEBUGF("9AM: timeStartUnix %lu, timeEndUnix: %lu", (uint32_t)timeStartUnix, (uint32_t)timeEndUnix);
		if(ServerRequestGetHistoryRaw((uint32_t)timeStartUnix, (uint32_t)timeEndUnix, &hisData) == SUCCESS){
			sleep9Flag = true;
			LOG_DEBUGF("sleep9Flag %d, sleep21Flag: %d", sleep9Flag, sleep21Flag);
		}
		else{
			return SLEEP_STATUS_ERROR;
		}
	}
	else{
		return SLEEP_STATUS_NOT_READY;
	}

	cJSON *historyAll = cJSON_Parse(hisData.data);
	vPortFree(hisData.data);
	if(historyAll == NULL)
	{
		vPortFree(hisData.data);
		return -1;
	}
	int sizeArr = cJSON_GetArraySize(historyAll);
	LOG_DEBUGF("sizeArr : %d\n", sizeArr);
	if(sizeArr > SLEEP_STAGE_MAX_SIZE){
		return SLEEP_STATUS_ERROR;
	}

	Time_t utcTime = {0};
	char timeStr[30];
	for(uint16_t i = sizeArr-1; i >= 0; i--)
	{
		cJSON *his10min = cJSON_GetArrayItem(historyAll, i);
		if(NULL == his10min)
		{
			break;
		}
		cJSON * obj = cJSON_GetObjectItem(his10min, "Heart_Rate");
		if(NULL != obj)
		{
			SleepStageDataArr.hrMin[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 0)->valuestring, (char **)NULL, 10);
			SleepStageDataArr.hrAvg[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 1)->valuestring, (char **)NULL, 10);
			SleepStageDataArr.hrMax[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 2)->valuestring, (char **)NULL, 10);
		}
		obj = cJSON_GetObjectItem(his10min, "Resp_Rate");
		if(NULL != obj)
		{
			SleepStageDataArr.rrMin[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 0)->valuestring, (char **)NULL, 10);
			SleepStageDataArr.rrAvg[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 1)->valuestring, (char **)NULL, 10);
			SleepStageDataArr.rrMax[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 2)->valuestring, (char **)NULL, 10);
		}
		obj = cJSON_GetObjectItem(his10min, "Body_Temp_Var");
		if(NULL != obj)
		{
			SleepStageDataArr.bodyTempVarMin[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 0)->valuestring, (char **)NULL, 10);
			SleepStageDataArr.bodyTempVarAvg[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 1)->valuestring, (char **)NULL, 10);
			SleepStageDataArr.bodyTempVarMax[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 2)->valuestring, (char **)NULL, 10);
		}
		obj = cJSON_GetObjectItem(his10min, "Body_Temp");
		if(NULL != obj)
		{
			SleepStageDataArr.bodyTempMin[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 0)->valuestring, (char **)NULL, 10);
			SleepStageDataArr.bodyTempAvg[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 1)->valuestring, (char **)NULL, 10);
			SleepStageDataArr.bodyTempMax[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 2)->valuestring, (char **)NULL, 10);
		}
		obj = cJSON_GetObjectItem(his10min, "Room_Temp");
		if(NULL != obj)
		{
			SleepStageDataArr.roomTempAvg[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 1)->valuestring, (char **)NULL, 10);
		}
		obj = cJSON_GetObjectItem(his10min, "Total_Motions");
		if(NULL != obj)
		{
			SleepStageDataArr.motions[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 1)->valuestring, (char **)NULL, 10);
		}
		obj = cJSON_GetObjectItem(his10min, "Total_Apnea");
		if(NULL != obj)
		{
			SleepStageDataArr.apnea[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 1)->valuestring, (char **)NULL, 10);
		}
		obj = cJSON_GetObjectItem(his10min, "Total_Snoring");
		if(NULL != obj)
		{
			SleepStageDataArr.snoring[dataIndex] =  (uint8_t)strtol(cJSON_GetArrayItem(obj, 0)->valuestring, (char **)NULL, 10);
			if(SleepStageDataArr.snoring[dataIndex] > 1){
				SleepStageDataArr.snoring[dataIndex] = 1;
			}
			else{
				SleepStageDataArr.snoring[dataIndex] = 0;
			}
		}
		obj = cJSON_GetObjectItem(his10min, "Th_rt");
		if(NULL != obj)
		{
			SleepStageDataArr.movEnergyMax[dataIndex] =  strtof(cJSON_GetArrayItem(obj, 2)->valuestring, (char **)NULL);
			SleepStageDataArr.movPeakMax[dataIndex] =  strtof(cJSON_GetArrayItem(obj, 0)->valuestring, (char **)NULL);
		}
		obj = cJSON_GetObjectItem(his10min, "Pnp_bg");
		if(NULL != obj)
		{
			SleepStageDataArr.pnpBgWeak[dataIndex] =  strtol(cJSON_GetArrayItem(obj, 0)->valuestring, (char **)NULL, 10);
			SleepStageDataArr.pnpBgOk[dataIndex] =  strtol(cJSON_GetArrayItem(obj, 1)->valuestring, (char **)NULL, 10);
			SleepStageDataArr.pnpBgGood[dataIndex] =  strtol(cJSON_GetArrayItem(obj, 2)->valuestring, (char **)NULL, 10);
			if(cJSON_GetArrayItem(obj, 3) != NULL){
				SleepStageDataArr.pnpBgVGood[dataIndex] =  strtol(cJSON_GetArrayItem(obj, 3)->valuestring, (char **)NULL, 10);
			}
			if(cJSON_GetArrayItem(obj, 4) != NULL){
				SleepStageDataArr.pnpBgExcel[dataIndex] =  strtol(cJSON_GetArrayItem(obj, 4)->valuestring, (char **)NULL, 10);
			}
		}
		obj = cJSON_GetObjectItem(his10min, "t");
		if(NULL != obj)
		{
			memset(timeStr, 0x00, sizeof(timeStr));
			strcpy(timeStr, obj->valuestring);
			char * token = strtok(obj->valuestring, "-");
			utcTime.tm_year = strtol(token, (char **)NULL, 10) - 1900;
			token = strtok(NULL, "-");
			utcTime.tm_mon = strtol(token, (char **)NULL, 10) - 1;
			token = strtok(NULL, "T");
			utcTime.tm_mday = strtol(token, (char **)NULL, 10);
			token = strtok(NULL, ":");
			utcTime.tm_hour = strtol(token, (char **)NULL, 10);
			token = strtok(NULL, ":");
			utcTime.tm_min = strtol(token, (char **)NULL, 10);
			token = strtok(NULL, "Z");
			utcTime.tm_sec = strtol(token, (char **)NULL, 10);
			SleepStageDataArr.unixTime[dataIndex] = mktime(&utcTime);
		}

		//		LOG_DEBUGF("His %d : %d %d %d %d %d %d %d %d %d %d %d %f %f %d %d %d", dataIndex,
		//				SleepStageDataArr.hrMax[dataIndex],
		//				SleepStageDataArr.hrAvg[dataIndex],
		//				SleepStageDataArr.hrMin[dataIndex],
		//				SleepStageDataArr.rrMax[dataIndex],
		//				SleepStageDataArr.rrAvg[dataIndex],
		//				SleepStageDataArr.rrMin[dataIndex],
		//				SleepStageDataArr.bodyTempMax[dataIndex],
		//				SleepStageDataArr.bodyTempAvg[dataIndex],
		//				SleepStageDataArr.bodyTempMin[dataIndex],
		//				SleepStageDataArr.motions[dataIndex],
		//				SleepStageDataArr.apnea[dataIndex],
		//				SleepStageDataArr.movEnergyMax[dataIndex],
		//				SleepStageDataArr.movPeakMax[dataIndex],
		//				SleepStageDataArr.snoring[dataIndex],
		//				SleepStageDataArr.roomTempAvg[dataIndex],
		//				SleepStageDataArr.unixTime[dataIndex]
		//				);

		dataIndex++;
	}
	cJSON_Delete(historyAll);

	return SLEEP_STATUS_OK;
}

int SleepStagesProcess()
{
	int ret = SleepStagesParseData();
	if((ret == SLEEP_STATUS_OK) && (dataIndex >= 5)){
		SleepStagesCal();
		SleepStagesReponse(&SleepStageOutput, dataIndex);
		SleepStagesUpdateLcd();
		return SLEEP_STATUS_OK;
	}
	if((ret == SLEEP_STATUS_OK) && (dataIndex < 5)){
		return SLEEP_STATUS_NOT_READY;
	}
	else if(ret == SLEEP_STATUS_NOT_READY){
		return SLEEP_STATUS_NOT_READY;
	}
	return SLEEP_STATUS_ERROR;
}

void SleepStagesCal()
{
	const uint16_t dataSize = dataIndex;

	uint8_t hrDiff[SLEEP_STAGE_MAX_SIZE] = {0};
	uint8_t rrDiff[SLEEP_STAGE_MAX_SIZE] = {0};
	float hrRrCombine[SLEEP_STAGE_MAX_SIZE+2] = {0};
	float sumPnpWeak = 0, sumPnpOk = 0, sumPnpGood = 0, sumPnpVGood = 0, sumPnpExcel = 0, sumPnp = 0;
	//float bodyTempDiff[SLEEP_STAGE_MAX_SIZE] = {0};
	for(uint16_t i = 0; i < dataSize; i++)
	{
		hrDiff[i] = SleepStageDataArr.hrMax[i] - SleepStageDataArr.hrMin[i]; // differences in HR: level of irregular HR
		rrDiff[i] = SleepStageDataArr.rrMax[i] - SleepStageDataArr.rrMin[i]; // differences in RR: level of irregular RR
		hrRrCombine[i] = (float)hrDiff[i]*(float)rrDiff[i]; // this is the leading indicator for HR and RR changing synchronously in cases of Deep sleep and REM
		//bodyTempDiff[i] = SleepStageDataArr.bodyTempMax[i] - SleepStageDataArr.bodyTempMin[i]; //body temp changes
	}
	hrRrCombine[SLEEP_STAGE_MAX_SIZE] = 0;
	hrRrCombine[SLEEP_STAGE_MAX_SIZE+1] = 0;

	/*Find time location when user enters the bed (bed is not empty) more than 600s (10min) means bed is empty*/
	bool occupiedBed[SLEEP_STAGE_MAX_SIZE];
	memset(occupiedBed, 0, sizeof(occupiedBed));
	for(uint16_t i = 0; i < (dataSize-1); i++)
	{
		uint32_t time_diff = SleepStageDataArr.unixTime[i+1] - SleepStageDataArr.unixTime[i];
		if(time_diff > SLEEP_TH_EMPTY_BED_TIME)
		{
			occupiedBed[i] = 0;
		}
		else{
			occupiedBed[i] = 1;
		}
		SleepStageOutput.bedArr[i] = occupiedBed[i];
	}
	occupiedBed[dataSize-1] = occupiedBed[dataSize-2];
	SleepStageOutput.bedArr[dataSize-1] = occupiedBed[dataSize-1];
	for(uint16_t i = 0; i < dataSize; i++)
	{
		SleepStageOutput.unixTime[i] = SleepStageDataArr.unixTime[i];
		SleepStageOutput.hrArr[i] = SleepStageDataArr.hrAvg[i];
		SleepStageOutput.rrArr[i] = SleepStageDataArr.rrAvg[i];
		SleepStageOutput.snoringArr[i] = SleepStageDataArr.snoring[i];
		SleepStageOutput.apneaArr[i] = SleepStageDataArr.apnea[i];
		SleepStageOutput.btcArr[i] = SleepStageDataArr.bodyTempVarAvg[i];
		SleepStageOutput.roomTempArr[i] = SleepStageDataArr.roomTempAvg[i];

		sumPnpWeak += (float)SleepStageDataArr.pnpBgWeak[i];
		sumPnpOk += (float)SleepStageDataArr.pnpBgOk[i];
		sumPnpGood += (float)SleepStageDataArr.pnpBgGood[i];
		sumPnpVGood += (float)SleepStageDataArr.pnpBgVGood[i];
		sumPnpExcel += (float)SleepStageDataArr.pnpBgExcel[i];
	}
	sumPnp = sumPnpWeak + sumPnpOk + sumPnpGood + sumPnpVGood + sumPnpExcel;

	SleepStageOutput.vitalSinal[1] = (uint8_t)(roundf(sumPnpOk/sumPnp));
	SleepStageOutput.vitalSinal[2] = (uint8_t)(roundf(sumPnpGood/sumPnp));
	SleepStageOutput.vitalSinal[3] = (uint8_t)(roundf(sumPnpVGood/sumPnp));
	SleepStageOutput.vitalSinal[4] = (uint8_t)(roundf(sumPnpExcel/sumPnp));
	SleepStageOutput.vitalSinal[0] = 100 - SleepStageOutput.vitalSinal[1] - SleepStageOutput.vitalSinal[2] - SleepStageOutput.vitalSinal[3] - SleepStageOutput.vitalSinal[4];


	/*Find threshold of combine_hr_rr amplitude for REM stage*/
	//    Peak_ *remPeak = (Peak_ *)pvPortMalloc(sizeof(Peak_)*dataSize);
	//    uint16_t sizeRemPeak = find_all_peaks(hrRrCombine, dataSize, remPeak);
	//    float remPeakMax = findMaxPeak(remPeak, sizeRemPeak);

	float remPeakMax = findMaxF(&hrRrCombine[1], dataSize-1);
	float thRemAmp = remPeakMax * SLEEP_TH_REM_AMP_FACTOR;

	/*Find threshold of combine_hr_rr slope-up for REM stage and threshold of combine_hr_rr slope-down for Light stage*/
	int16_t MaxSlopeUp = 0;
	for(uint16_t i = 0; i < (dataSize-1); i++)
	{
		int16_t combineDiff = hrRrCombine[i+1] - hrRrCombine[i];
		if(combineDiff >= MaxSlopeUp)
		{
			MaxSlopeUp = combineDiff;
		}
	}
	float thSlopeUp = SLEEP_TH_SLOPE_UP_FACTOR*MaxSlopeUp;
	float thSlopeDown = thSlopeUp;

	/*Find thresholds for motion and movenergy*/
	float thMotion = SLEEP_TH_MOTION_FACTOR*findMax( (uint16_t *)&SleepStageDataArr.motions[1], dataSize-2);
	float thMovEnergy = SLEEP_TH_MOVENERGY_FACTOR*findMaxF( (float *)&SleepStageDataArr.movEnergyMax[1], dataSize-2);

	/*Detect sleep stages*/
	float preLightDeep = 0.3*thSlopeUp;
	SleepStageType_t currentStage = SLEEP_STAGE_AWAKE_TYPE;
	SleepStageType_t preCurrentStage;
	uint16_t timeAwakeCount = 1, timeRemCount = 0, timeRemAwakeCount = 0, timeLightCount = 0, timeDeepCount = 0;
	SleepStageOutput.sleepStageArr[0] = currentStage;
	for(uint16_t i = 1; i < dataSize; i++)
	{
		preCurrentStage = currentStage;
		/*Difference between combine(i) and combine(i-1)*/
		float diffComb = fabs(hrRrCombine[i] - hrRrCombine[i-1]);

		/*Find transition from empty to occupied*/
		bool emptyToOcc = false;
		if((occupiedBed[i-1] == 0)&&(occupiedBed[i] == 1)){
			emptyToOcc = true;  //empty to occupied status
		}
		/*Special case: Look forward to find possible Deep*/
		float diffComb0 = hrRrCombine[i] - hrRrCombine[i-1];
		float diffComb1 = 0, diffComb2 = 0;
		if(i < (dataSize - 2))
		{
			diffComb1 = hrRrCombine[i+1] - hrRrCombine[i];
			diffComb2 = hrRrCombine[i+2] - hrRrCombine[i];
		}
		else if(i == (dataSize - 2)){
			diffComb1 = hrRrCombine[i+1] - hrRrCombine[i];
		}

		bool possibleDeep = false;
		if( ((diffComb0 < 0) && (fabs(diffComb0) >= thSlopeDown)) && (hrRrCombine[i+1] > hrRrCombine[i]) && (hrRrCombine[i+2] > hrRrCombine[i])
				&& ( ((diffComb1 > 0)&&(fabs(diffComb1) > thSlopeUp)) ||
						((diffComb1 > 0)&&(hrRrCombine[i+1] >= thRemAmp)) ||
						((diffComb2 > 0)&&(fabs(diffComb2) > thSlopeUp)) ||
						((diffComb2 > 0)&&(hrRrCombine[i+2] >= thRemAmp)) ||
						(((diffComb1 > preLightDeep)) && ((SleepStageDataArr.motions[i] <= thMotion) || (SleepStageDataArr.movEnergyMax[i] <= thMovEnergy)))
				))
		{
			possibleDeep = true;
		}
		/*Special case: Look backward to find possible REM*/
		bool possibleRem = false;
		if( (i > 1) && (hrRrCombine[i] >= hrRrCombine[i-1]) && (hrRrCombine[i-1] >= hrRrCombine[i-2]) && ((hrRrCombine[i] - hrRrCombine[i-2]) >= thSlopeUp)
				&& ( (SleepStageDataArr.motions[i] <= thMotion) || (SleepStageDataArr.movEnergyMax[i] <= thMovEnergy)) )
		{
			possibleRem = true;
		}
		/*Special case: Look backward and forward for possible Light*/
		bool possibleLight = false;
		if( (hrRrCombine[i] < thRemAmp) && (diffComb0 > 0) && (fabs(diffComb0) < thSlopeUp)
				&& (fabs(diffComb0) >= (0.3*thSlopeUp)) && (diffComb1 < 0) )
		{
			possibleLight = true;
		}
		/*Special case: REM in first part and then Awake in next part of 10 min*/
		bool possibleRemAwake = false;
		if( (hrRrCombine[i] >= hrRrCombine[i-1]) && ( (diffComb >= thSlopeUp) || (hrRrCombine[i] >= thRemAmp) ) &&
				(SleepStageDataArr.motions[i] > thMotion) && (SleepStageDataArr.movEnergyMax[i] > thMovEnergy) )
		{
			possibleRemAwake = true;
		}

		/*Algorithm to detect sleep stages*/
		if( (emptyToOcc == true) || ( (SleepStageDataArr.motions[i] > thMotion) && (SleepStageDataArr.movEnergyMax[i] > thMovEnergy)
				&& (possibleRemAwake == false) && (SleepStageDataArr.snoring[i] == 0)) )
		{
			currentStage = SLEEP_STAGE_AWAKE_TYPE;
		}
		else if(preCurrentStage == SLEEP_STAGE_AWAKE_TYPE || preCurrentStage == SLEEP_STAGE_REM_AWAKE_TYPE) // Previous stage is Awake or Rem-Awake
		{
			if(possibleDeep == true){
				currentStage = SLEEP_STAGE_DEEP_TYPE;
			}
			/*it is in Light both motion and movenergy must be lower than threshold*/
			else if( ((hrRrCombine[i] < hrRrCombine[i-1]) || (hrRrCombine[i] < thRemAmp)) &&
					( (SleepStageDataArr.motions[i] <= thMotion) || (SleepStageDataArr.movEnergyMax[i] <= thMovEnergy)) ){
				currentStage = SLEEP_STAGE_LIGHT_TYPE;
			}
			/*do not pass th_slopedown then it is in REM both motion and movenergy must be lower than threshold*/
			else if( ( (hrRrCombine[i] >= thRemAmp) || ( (hrRrCombine[i] <= hrRrCombine[i-1]) && (diffComb < thSlopeDown) ) ) &&
					( (SleepStageDataArr.motions[i] <= thMotion) || (SleepStageDataArr.movEnergyMax[i] <= thMovEnergy) ) ){
				currentStage = SLEEP_STAGE_REM_TYPE;
			}
			else{
				currentStage = SLEEP_STAGE_AWAKE_TYPE;
			}
		}
		else if(preCurrentStage == SLEEP_STAGE_REM_TYPE){
			if(possibleDeep == true){
				currentStage = SLEEP_STAGE_DEEP_TYPE;
			}
			else if( (hrRrCombine[i] < hrRrCombine[i-1]) && ( (diffComb >= thSlopeDown) || (hrRrCombine[i] <= thRemAmp) )
					&& ( (SleepStageDataArr.motions[i] <= thMotion) || (SleepStageDataArr.movEnergyMax[i] <= thMovEnergy))){
				currentStage = SLEEP_STAGE_LIGHT_TYPE;
			}
			//			else if( (hrRrCombine[i] >= hrRrCombine[i-1]) && (SleepStageDataArr.motions[i] <= thMotion) && (SleepStageDataArr.movEnergyMax[i] <= thMovEnergy))
			//			{
			//				currentStage = SLEEP_STAGE_REM_TYPE;
			//			}
			else{
				currentStage = SLEEP_STAGE_REM_TYPE;
			}
		}
		else if(preCurrentStage == SLEEP_STAGE_LIGHT_TYPE){
			if(possibleDeep == true)
			{
				currentStage = SLEEP_STAGE_DEEP_TYPE;
			}
			else if(possibleRem == true){
				currentStage = SLEEP_STAGE_REM_TYPE;
			}
			else if(possibleRemAwake == true){
				currentStage = SLEEP_STAGE_REM_AWAKE_TYPE;
			}
			else if( (hrRrCombine[i] >= hrRrCombine[i-1]) && ( (diffComb >= thSlopeUp) || (hrRrCombine[i] >= thRemAmp) ) &&
					( (SleepStageDataArr.motions[i] <= thMotion) || (SleepStageDataArr.movEnergyMax[i] <= thMovEnergy) ) ){
				currentStage = SLEEP_STAGE_REM_TYPE;
			}
			else if( ( (hrRrCombine[i] <= hrRrCombine[i-1]) || (SleepStageDataArr.bodyTempVarMin[i] < 0) ) &&
					( (SleepStageDataArr.motions[i] <= thMotion) || (SleepStageDataArr.movEnergyMax[i] <= thMovEnergy) ) ){
				currentStage = SLEEP_STAGE_DEEP_TYPE;
				preLightDeep = (preLightDeep + diffComb)/2;
			}
			else{
				currentStage = SLEEP_STAGE_LIGHT_TYPE;
			}
		}
		else if(preCurrentStage == SLEEP_STAGE_DEEP_TYPE){
			if(possibleRem == true){
				currentStage = SLEEP_STAGE_REM_TYPE;
			}
			else if(possibleRemAwake == true){
				currentStage = SLEEP_STAGE_REM_AWAKE_TYPE;
			}
			else if( (hrRrCombine[i] > hrRrCombine[i-1]) && ( (diffComb >= thSlopeUp) || (hrRrCombine[i] >= thRemAmp) ) &&
					( (SleepStageDataArr.motions[i] <= thMotion) || (SleepStageDataArr.movEnergyMax[i] <= thMovEnergy) ) ){
				currentStage = SLEEP_STAGE_REM_TYPE;
			}
			else if( (possibleLight == true) ||
					( (hrRrCombine[i] > hrRrCombine[i-1]) && (diffComb >= preLightDeep) && ( (SleepStageDataArr.motions[i] <= thMotion) || (SleepStageDataArr.movEnergyMax[i] <= thMovEnergy) ) ) ){
				currentStage = SLEEP_STAGE_LIGHT_TYPE;
			}
			else{
				currentStage = SLEEP_STAGE_DEEP_TYPE;
			}
		}

		if(currentStage == SLEEP_STAGE_AWAKE_TYPE){
			timeAwakeCount++;
		}
		else if(currentStage == SLEEP_STAGE_REM_TYPE){
			timeRemCount++;
		}
		else if(currentStage == SLEEP_STAGE_REM_AWAKE_TYPE){
			timeRemAwakeCount++;
		}
		else if(currentStage == SLEEP_STAGE_LIGHT_TYPE){
			timeLightCount++;
		}
		else if(currentStage == SLEEP_STAGE_DEEP_TYPE){
			timeDeepCount++;
		}
		else{
			timeAwakeCount = timeAwakeCount;
		}

		SleepStageOutput.sleepStageArr[i] = currentStage;
	}


	/*Calculate Sleep statistic results and Sleep Score*/
	float timeRemAwake = (float)SLEEP_TIME_INTERVAL*timeRemAwakeCount;
	float timeAwake = (float)SLEEP_TIME_INTERVAL*timeAwakeCount + (float)timeRemAwakeCount/4; //half of time_remawake is rem, other half is awake
	float timeRem = (float)SLEEP_TIME_INTERVAL*timeRemCount + (float)timeRemAwake*3/4; //half of time_remawake is rem other half is awake
	float timeLight = (float)SLEEP_TIME_INTERVAL*timeLightCount;
	float timeDeep = (float)SLEEP_TIME_INTERVAL*timeDeepCount;
	//float timeSum = timeAwake + timeRem + timeLight + timeDeep;
	float totalTime = SLEEP_TIME_INTERVAL*dataSize;
	/*Calculate percentage of sleep stages*/
	float percentAwake = timeAwake*100/totalTime;
	float percentRem = timeRem*100/totalTime;
	float percentLight = timeLight*100/totalTime;
	float percentDeep = 100 - percentAwake - percentRem - percentLight;
	//float percentTotal = percentAwake + percentRem + percentLight + percentDeep;

	uint16_t totalApnea = 0, totalSnoring = 0;
	float avgHr = 0, avgRr = 0;
	uint8_t maxAllHr = SleepStageDataArr.hrMax[0], minAllHr = SleepStageDataArr.hrMin[0];
	int8_t maxBtc = SleepStageDataArr.bodyTempVarMax[0], minBtc = SleepStageDataArr.bodyTempVarMin[0];
	int16_t avgAllRoomTemp = 0;
	uint16_t cntRT = 0;
	uint16_t cntBT = 0;
	int16_t avgAllBodyTemp = 0;
	for(uint16_t i = 0; i < dataSize; i++)
	{
		totalApnea += SleepStageDataArr.apnea[i];
		totalSnoring += SleepStageDataArr.snoring[i];
		avgHr += (float)SleepStageDataArr.hrAvg[i];
		avgRr += (float)SleepStageDataArr.rrAvg[i];
		if(SleepStageDataArr.hrAvg[i] > maxAllHr){
			maxAllHr = SleepStageDataArr.hrAvg[i];
		}
		if(SleepStageDataArr.hrAvg[i] < minAllHr){
			minAllHr = SleepStageDataArr.hrAvg[i];
		}
		if(SleepStageDataArr.bodyTempVarAvg[i] > maxBtc){
			maxBtc = SleepStageDataArr.bodyTempVarAvg[i];
		}
		if(SleepStageDataArr.bodyTempVarAvg[i] < minBtc){
			minBtc = SleepStageDataArr.bodyTempVarAvg[i];
		}
		if(SleepStageDataArr.roomTempAvg[i] != 127){
			avgAllRoomTemp += SleepStageDataArr.roomTempAvg[i];
			cntRT++;
		}
		if(SleepStageDataArr.bodyTempAvg[i] != 127){
			avgAllBodyTemp += SleepStageDataArr.bodyTempAvg[i];
			cntBT++;
		}
	}
	avgAllRoomTemp = (int16_t)roundf((float)avgAllRoomTemp/cntRT);
	avgAllBodyTemp = (int16_t)roundf((float)avgAllBodyTemp/cntBT);

	avgHr = avgHr/dataSize;
	avgRr = avgRr/dataSize;
	float totalSnoringPerHour = (float)totalSnoring*60/totalTime;


	WorstScore_t worstScore = {1,1,0};
	float scoreApnea = SleepScoreApnea(totalApnea, &worstScore);
	float scoreSnoring = SleepScoreSnoring(totalSnoring, totalSnoringPerHour, &worstScore);
	float scoreHr = SleepScoreHr(maxAllHr, minAllHr, &worstScore);
	float scoreRem = SleepScoreRem(percentRem, &worstScore);
	float scoreDeep = SleepScoreDeep(percentDeep, &worstScore);
	float scoreSleepTime = SleepScoreSleepTime(totalTime - timeAwake, &worstScore);
	float scoreBtc = SleepScoreBtc(maxBtc, minBtc, &worstScore);
	float scoreRoomTemp = SleepScoreRoomTemp(avgAllRoomTemp, &worstScore);

	float totalScore = scoreApnea + scoreSnoring + scoreHr + scoreRem + scoreDeep + scoreSleepTime + scoreBtc + scoreRoomTemp;
	float percentScore = totalScore*100/80;

	LOG_DEBUGF("scoreApnea %f, scoreSnoring %f, scoreHr %f, scoreRem %f, scoreDeep %f, scoreSleepTime %f, scoreBtc %f, scoreRoomTemp %f, totalScore %f, percentScore %f",
			scoreApnea, scoreSnoring, scoreHr, scoreRem, scoreDeep, scoreSleepTime, scoreBtc, scoreRoomTemp, totalScore, percentScore);

	SleepStageOutput.totalTime = (uint32_t)roundf(totalTime);
	SleepStageOutput.totalSleepTime = (uint32_t)roundf((totalTime - timeAwake));
	SleepStageOutput.timeAwake = (uint32_t)roundf(timeAwake);
	SleepStageOutput.timeRem = (uint32_t)roundf(timeRem);
	SleepStageOutput.timeLight = (uint32_t)roundf(timeLight);
	SleepStageOutput.timeDeep = (uint32_t)roundf(timeDeep);

	SleepStageOutput.percentAwake = (uint8_t)roundf(percentAwake);
	SleepStageOutput.percentRem = (uint8_t)roundf(percentRem);
	SleepStageOutput.percentLight = (uint8_t)roundf(percentLight);
	SleepStageOutput.percentDeep = (uint8_t)roundf(percentDeep);

	SleepStageOutput.scoreApnea = (uint8_t)roundf(scoreApnea);
	SleepStageOutput.scoreSnoring = (uint8_t)roundf(scoreSnoring);
	SleepStageOutput.scoreHr = (uint8_t)roundf(scoreHr);
	SleepStageOutput.scoreRem = (uint8_t)roundf(scoreRem);
	SleepStageOutput.scoreDeep = (uint8_t)roundf(scoreDeep);
	SleepStageOutput.scoreSleepTime = (uint8_t)roundf(scoreSleepTime);
	SleepStageOutput.scoreBtc = (uint8_t)roundf(scoreBtc);
	SleepStageOutput.scoreRoomTemp = (uint8_t)roundf(scoreRoomTemp);
	SleepStageOutput.perfectScore = (uint8_t)80;
	SleepStageOutput.percentScore = (uint8_t)roundf(percentScore);

	SleepStageOutput.worstScore1 = worstScore.worstScore1;
	SleepStageOutput.worstScore2 = worstScore.worstScore2;
	SleepStageOutput.totalApnea = totalApnea;
	SleepStageOutput.totalSnoring = totalSnoring;
	SleepStageOutput.totalSnoringPerHour = (uint16_t)roundf(totalSnoringPerHour);

	SleepStageOutput.avgHr = (uint8_t)roundf(avgHr);
	SleepStageOutput.avgRr = (uint8_t)roundf(avgRr);
	SleepStageOutput.allHrMax = maxAllHr;
	SleepStageOutput.allHrMin = minAllHr;
	SleepStageOutput.btcMin = (uint8_t)minBtc;
	SleepStageOutput.btcMax = (uint8_t)maxBtc;
	SleepStageOutput.avgRoomTemp = (int8_t)avgAllRoomTemp;
	SleepStageOutput.avgBodyTemp = (int8_t)avgAllBodyTemp;

	//	for(int i = 0; i < dataSize; i++){
	//		if(SleepStageOutput.bedArr[i] == 0){
	//			SleepStageOutput.apneaArr[i] = 0;
	//			SleepStageOutput.snoringArr[i] = 0;
	//			SleepStageOutput.btcArr[i] = 0;
	//			SleepStageOutput.hrArr[i] = 0;
	//			SleepStageOutput.rrArr[i] = 0;
	//			SleepStageOutput.sleepStageArr[i] = 0;
	//		}
	//	}
}



/*Sleep score with apnea
 total is 9 points. If apnea is 1, then subtract 0.25.
 If apnea is > 10, then subtract 1.0 for the difference diff(apnea, 10), and still subtract 0.25 * 10.
 */
static float SleepScoreApnea(uint16_t totalApnea, WorstScore_t* worstScore){
	float minusApnea;
	if(totalApnea <= 10){
		minusApnea = ((float)totalApnea*SLEEP_PT_APNEA_LOW);
	}
	else{
		minusApnea = (float)(10*SLEEP_PT_APNEA_LOW) + (float)((totalApnea - 10)*SLEEP_PT_APNEA_HIGH);
	}
	if(minusApnea > worstScore->minusMax1){
		worstScore->minusMax2 = worstScore->minusMax1;
		worstScore->worstScore2 = worstScore->worstScore1;
		worstScore->worstScore1 = 1;
		worstScore->minusMax1 = minusApnea;
	}
	else if(minusApnea > worstScore->minusMax2){
		worstScore->minusMax2 = minusApnea;
		worstScore->worstScore2 = 1;
	}

	float scoreApnea = SLEEP_SCORE_DEFAULT_APNEA - minusApnea;
	if(scoreApnea < 0){
		scoreApnea = 0;
	}
	return scoreApnea;
}

/*Sleep score with Snoring:
 No snoring = 9 points. Count total number of snoring for
 each hour (6 times per hour).
 */
static float SleepScoreSnoring(uint16_t totalSnoring, float totalSnoringPerHour, WorstScore_t* worstScore){
	float minusSnoring = totalSnoringPerHour*SLEEP_PT_SNORING;
	float scoreSnoring = SLEEP_SCORE_NO_SNORING - minusSnoring;

	if(minusSnoring > worstScore->minusMax1){
		worstScore->minusMax2 = worstScore->minusMax1;
		worstScore->worstScore2 = worstScore->worstScore1;
		worstScore->worstScore1 = 2;
		worstScore->minusMax1 = minusSnoring;
	}
	else if(minusSnoring > worstScore->minusMax2){
		worstScore->minusMax2 = minusSnoring;
		worstScore->worstScore2 = 2;
	}

	if(scoreSnoring < 0){
		scoreSnoring = 0;
	}
	return scoreSnoring;
}

/*Sleep score with HR:
 Normal is From 50BPM to 85BPM at sleeping, Total HR points = 9.
 If HR is less than 50BPM, then find diff(HR,50).
 If HR is higher than 85BPM, then find diff(HR, 85).
 Subtract 1.0 for each diff. For example, diff = 5,
 then subtract 1.0 * 5 = 5 from 10.
 */
static float SleepScoreHr(uint8_t maxAllHr, uint8_t minAllHr, WorstScore_t* worstScore){
	uint8_t diffHrHigh = 0;
	if(maxAllHr > SLEEP_HR_HIGH){
		diffHrHigh = abs(maxAllHr - SLEEP_HR_HIGH); //difference between max of hr and 85BPM
	}

	uint8_t diffHrLow = 0;
	if(minAllHr < SLEEP_HR_LOW){
		diffHrLow = abs(minAllHr - SLEEP_HR_LOW); //difference between min of hr and 50BPM
	}
	float minusHrHigh = (float)diffHrHigh*SLEEP_PT_HR;
	float minusHrLow = (float)diffHrLow*SLEEP_PT_HR;

	if(minusHrHigh > worstScore->minusMax1){
		worstScore->minusMax2 = worstScore->minusMax1;
		worstScore->worstScore2 = worstScore->worstScore1;
		worstScore->worstScore1 = 3;
		worstScore->minusMax1 = minusHrHigh;
	}
	else if(minusHrHigh > worstScore->minusMax2){
		worstScore->minusMax2 = minusHrHigh;
		worstScore->worstScore2 = 3;
	}

	if(minusHrLow > worstScore->minusMax1){
		worstScore->minusMax2 = worstScore->minusMax1;
		worstScore->worstScore2 = worstScore->worstScore1;
		worstScore->worstScore1 = 4;
		worstScore->minusMax1 = minusHrLow;
	}
	else if(minusHrLow > worstScore->minusMax2){
		worstScore->minusMax2 = minusHrLow;
		worstScore->worstScore2 = 4;
	}

	float scoreHr = SLEEP_SCORE_DEFAULT_HR - minusHrHigh - minusHrLow;
	return scoreHr;
}

/*Sleep Score with REM time
Normal is 20% to 27% of total time sleep. The more is the better.
Total points = 9. Subtract 1.0 for each diff(REM, 20%) for REM < 20 ,
add 0.5 for each diff(REM, 27%) for REM > 27%.
 */
static float SleepScoreRem(float percentRem, WorstScore_t* worstScore){
	float plusRem = 0, minusRem = 0;
	if(percentRem > SLEEP_NORMAL_REM_HIGH){
		plusRem = SLEEP_PT_REM_PLUS*(fabs(percentRem - SLEEP_NORMAL_REM_HIGH));
	}
	else if(percentRem < SLEEP_NORMAL_REM_LOW){
		minusRem = SLEEP_PT_REM_MINUS*(fabs(percentRem - SLEEP_NORMAL_REM_LOW));
	}

	if(minusRem > worstScore->minusMax1){
		worstScore->minusMax2 = worstScore->minusMax1;
		worstScore->worstScore2 = worstScore->worstScore1;
		worstScore->worstScore1 = 5;
		worstScore->minusMax1 = minusRem;
	}
	else if(minusRem > worstScore->minusMax2){
		worstScore->minusMax2 = minusRem;
		worstScore->worstScore2 = 5;
	}

	float scoreRem = SLEEP_SCORE_DEFAULT_REM + plusRem - minusRem;
	if(scoreRem > 12){
		scoreRem = 12;
	}
	else if(scoreRem < 0){
		scoreRem = 0;
	}
	return scoreRem;
}

/*Sleep Score with Deep sleep time
Normal is 15% to 23% of total time sleep. The more is the better.
Total points = 9. Subtract 1.0 for each diff(deep, 15%) for deep < 15 ,
add 0.5 for each diff(deep, 23%) for deep > 23%.
 */
static float SleepScoreDeep(float percentDeep, WorstScore_t* worstScore){
	float plusDeep = 0, minusDeep = 0;
	if(percentDeep > SLEEP_NORMAL_DEEP_HIGH){
		plusDeep = SLEEP_PT_DEEP_PLUS*(fabs(percentDeep - SLEEP_NORMAL_DEEP_HIGH));
	}
	else if(percentDeep < SLEEP_NORMAL_DEEP_LOW){
		minusDeep = SLEEP_PT_DEEP_MINUS*(fabs(percentDeep - SLEEP_NORMAL_DEEP_LOW));
	}

	if(minusDeep > worstScore->minusMax1){
		worstScore->minusMax2 = worstScore->minusMax1;
		worstScore->worstScore2 = worstScore->worstScore1;
		worstScore->worstScore1 = 6;
		worstScore->minusMax1 = minusDeep;
	}
	else if(minusDeep > worstScore->minusMax2){
		worstScore->minusMax2 = minusDeep;
		worstScore->worstScore2 = 6;
	}

	float scoreDeep = SLEEP_SCORE_DEFAULT_DEEP + plusDeep - minusDeep;
	if(scoreDeep > 12){
		scoreDeep = 12;
	}
	else if(scoreDeep < 0){
		scoreDeep = 0;
	}
	return scoreDeep;
}

/*Sleep score with total sleep time:
 Normal sleep time is 6.0 hours (360 min) to 8 hours (480 min),
 total points = 9. If sleeping less than 6.0 hours,
 then subtract 1.0 for each 1 hour
 less than 6.0. And add 0.5 for each 1 hour more than 8.0.
 */
static float SleepScoreSleepTime(float totalSleepTime, WorstScore_t* worstScore){
	float minusSleepTime = 0, plusSleepTime = 0;
	if(totalSleepTime > SLEEP_NORMAL_SLEEPTIME_HIGH){
		plusSleepTime = (SLEEP_PT_SLEEPTIME_PLUS*(fabs(totalSleepTime - SLEEP_NORMAL_SLEEPTIME_HIGH)))/60;
	}
	else if(totalSleepTime < SLEEP_NORMAL_SLEEPTIME_LOW){
		minusSleepTime = (SLEEP_PT_SLEEPTIME_MINUS*(fabs(totalSleepTime - SLEEP_NORMAL_SLEEPTIME_LOW)))/60;
	}

	if(minusSleepTime > worstScore->minusMax1){
		worstScore->minusMax2 = worstScore->minusMax1;
		worstScore->worstScore2 = worstScore->worstScore1;
		worstScore->worstScore1 = 7;
		worstScore->minusMax1 = minusSleepTime;
	}
	else if(minusSleepTime > worstScore->minusMax2){
		worstScore->minusMax2 = minusSleepTime;
		worstScore->worstScore2 = 7;
	}

	float scoreSleepTime = SLEEP_SCORE_DEFAULT_SLEEPTIME + plusSleepTime - minusSleepTime;
	return scoreSleepTime;
}


/*Sleep Score with Body Temperature Change
 Normal is -1C to +1C. Total points = 9. Subtract 2.0 for each 1C less than
 -1C or each 1C higher than +1C
 */
static float SleepScoreBtc(int8_t maxBtc, int8_t minBtc, WorstScore_t* worstScore){
	float minusBtcMax = 0, minusBtcMin = 0;
	if(maxBtc > SLEEP_NORMAL_BTC_HIGH){
		minusBtcMax = SLEEP_PT_BTC*((float)abs(maxBtc - SLEEP_NORMAL_BTC_HIGH));
	}
	if(minBtc < SLEEP_NORMAL_BTC_LOW){
		minusBtcMin = SLEEP_PT_BTC*((float)abs(minBtc - SLEEP_NORMAL_BTC_LOW));
	}

	if(minusBtcMax > worstScore->minusMax1){
		worstScore->minusMax2 = worstScore->minusMax1;
		worstScore->worstScore2 = worstScore->worstScore1;
		worstScore->worstScore1 = 8;
		worstScore->minusMax1 = minusBtcMax;
	}
	else if(minusBtcMax > worstScore->minusMax2){
		worstScore->minusMax2 = minusBtcMax;
		worstScore->worstScore2 = 8;
	}

	if(minusBtcMin > worstScore->minusMax1){
		worstScore->minusMax2 = worstScore->minusMax1;
		worstScore->worstScore2 = worstScore->worstScore1;
		worstScore->worstScore1 = 9;
		worstScore->minusMax1 = minusBtcMin;
	}
	else if(minusBtcMin > worstScore->minusMax2){
		worstScore->minusMax2 = minusBtcMin;
		worstScore->worstScore2 = 9;
	}

	float scoreBtc = SLEEP_SCORE_DEFAULT_BTC - minusBtcMax - minusBtcMin;
	if(scoreBtc < 0){
		scoreBtc = 0;
	}
	return scoreBtc;
}

/*Sleep Score with Room Temperature
 Normal is 19C to 23C. Total points = 9. Subtract 1.0 for each 1C less than
 19C or each 1C higher than 23C.
 */
static float SleepScoreRoomTemp(int16_t avgAllRoomTemp, WorstScore_t* worstScore){
	float minusRoomTemp = 0;
	if(avgAllRoomTemp > SLEEP_NORMAL_ROOMTEMP_HIGH){
		minusRoomTemp = SLEEP_PT_ROOMTEMP*((float)abs(avgAllRoomTemp - SLEEP_NORMAL_ROOMTEMP_HIGH));
	}
	else if(avgAllRoomTemp < SLEEP_NORMAL_ROOMTEMP_LOW){
		minusRoomTemp = SLEEP_PT_ROOMTEMP*((float)abs(avgAllRoomTemp - SLEEP_NORMAL_ROOMTEMP_LOW));
	}

	if(minusRoomTemp > worstScore->minusMax1){
		worstScore->minusMax2 = worstScore->minusMax1;
		worstScore->worstScore2 = worstScore->worstScore1;
		worstScore->worstScore1 = 10;
		worstScore->minusMax1 = minusRoomTemp;
	}
	else if(minusRoomTemp > worstScore->minusMax2){
		worstScore->minusMax2 = minusRoomTemp;
		worstScore->worstScore2 = 10;
	}

	float scoreRoomTemp = SLEEP_SCORE_DEFAULT_ROOMTEMP - minusRoomTemp;
	if(scoreRoomTemp < 0){
		scoreRoomTemp = 0;
	}
	return scoreRoomTemp;
}



char* Unum2str(uint32_t num){
	static char str[10];
	memset(str, 0, sizeof(str));
	sprintf(str, "%lu", num);
	return str;
}

char* Num2str(int32_t num){
	static char str[10];
	memset(str, 0, sizeof(str));
	sprintf(str, "%ld", num);
	return str;
}

void SleepStagesUpdateLcd(){
	SleepStageOutput_t  *sleepStageOutput = &SleepStageOutput;
	uint8_t test[7] = {sleepStageOutput->percentScore,0,0,0,0,0,0};
	SleepHistory_t sleepHistory;
	uint16_t dataSize = dataIndex;
	memcpy(sleepHistory.sleepScore,test,7);
	LcdUpdateSleepScoreHistory(&sleepHistory);

	SleepData_t sleepData;
	sleepData.totalSleepTime.h = sleepStageOutput->totalSleepTime/60;
	sleepData.totalSleepTime.m = (sleepStageOutput->totalSleepTime%60);
	sleepData.timeInBed.h = sleepStageOutput->totalTime/60;
	sleepData.timeInBed.m = (sleepStageOutput->totalTime%60);

	sleepData.percentAwake = sleepStageOutput->percentAwake;
	sleepData.percentLight = sleepStageOutput->percentLight;
	sleepData.percentRem = sleepStageOutput->percentRem;
	sleepData.percentDeep = sleepStageOutput->percentDeep;
	sleepData.avgHR = sleepStageOutput->avgHr;
	sleepData.avgRR = sleepStageOutput->avgRr;
	sleepData.totalApnea = sleepStageOutput->totalApnea;
	sleepData.totalSnoring = sleepStageOutput->totalSnoring;
	sleepData.bodyTempVariationMin = sleepStageOutput->btcMin;
	sleepData.bodyTempVariationMax = sleepStageOutput->btcMax;
	sleepData.roomTemp = sleepStageOutput->avgRoomTemp;
	sleepData.bodyTemp = sleepStageOutput->avgBodyTemp;
	sleepData.locMax1 = sleepStageOutput->worstScore1;
	sleepData.locMax2 = sleepStageOutput->worstScore2;
	sleepData.allHrMax = sleepStageOutput->allHrMax;
	sleepData.allHrMin = sleepStageOutput->allHrMin;
	sleepData.avgRoomTemp = sleepStageOutput->avgRoomTemp;

	sleepData.scoreApnea = sleepStageOutput->scoreApnea;
	sleepData.scoreSnoring = sleepStageOutput->scoreSnoring;
	sleepData.scoreHr = sleepStageOutput->scoreHr;
	sleepData.scoreRem = sleepStageOutput->scoreRem;
	sleepData.scoreDeep = sleepStageOutput->scoreDeep;
	sleepData.scoreSleeptime = sleepStageOutput->scoreSleepTime;
	sleepData.scoreBtc = sleepStageOutput->scoreBtc;
	sleepData.scoreRoomtemp = sleepStageOutput->scoreRoomTemp;
	sleepData.totalSnoringPerHour = sleepStageOutput->totalSnoringPerHour;

	Time_t *timeDate;
	time_t timeStartUtc = sleepStageOutput->unixTime[0] + (SettingGetTimeZone() * 3600);
	time_t timeEndUtc = sleepStageOutput->unixTime[dataSize-1] + (SettingGetTimeZone() * 3600);

	timeDate = localtime(&timeStartUtc);
	memcpy(&sleepData.timeStart,timeDate,sizeof(Time_t));
	timeDate = localtime(&timeEndUtc);
	memcpy(&sleepData.timeEnd,timeDate,sizeof(Time_t));

	LOG_WARN("sleepData.timeStart  :  %d  sleepData.timeEnd : %d ",sleepData.timeStart,sleepData.timeEnd);

	LcdUpdateSleepData(&sleepData);

	GrSleepData_t GrSleepData[144];
	uint32_t offset = 0;
	for(int i = 0; i < dataSize; i++){

		time_t timeUnixLocal = sleepStageOutput->unixTime[i] + (time_t)(SettingGetTimeZone() * 3600);
		timeDate = localtime(&timeUnixLocal);

		GrSleepData[i+offset].sleepStates.time.h = timeDate->tm_hour;
		GrSleepData[i+offset].sleepStates.time.m = timeDate->tm_min;
		GrSleepData[i+offset].sleepStates.value = sleepStageOutput->sleepStageArr[i];
		GrSleepData[i+offset].bed = sleepStageOutput->bedArr[i];
		GrSleepData[i+offset].heartRate = sleepStageOutput->hrArr[i];
		GrSleepData[i+offset].breathingRate = sleepStageOutput->rrArr[i];
		GrSleepData[i+offset].snoring = sleepStageOutput->snoringArr[i];
		GrSleepData[i+offset].breathingPaused = sleepStageOutput->apneaArr[i];
		GrSleepData[i+offset].bodyTempVariation = sleepStageOutput->btcArr[i];
		GrSleepData[i+offset].roomTemp = sleepStageOutput->roomTempArr[i];

		if(((sleepStageOutput->unixTime[i+1] - sleepStageOutput->unixTime[i]) > 1190)&&(i < dataSize - 1)){
			offset++;
			time_t timeUnixLocal = sleepStageOutput->unixTime[i] + (time_t)(SettingGetTimeZone() * 3600) + 600;
			timeDate = localtime(&timeUnixLocal);
			GrSleepData[i+offset].sleepStates.time.h = timeDate->tm_hour;
			GrSleepData[i+offset].sleepStates.time.m = timeDate->tm_min;
			GrSleepData[i+offset].sleepStates.value = SLEEP_STAGE_HIDDEN;
			GrSleepData[i+offset].bed = 0;
			GrSleepData[i+offset].heartRate = 0;
			GrSleepData[i+offset].breathingRate = 0;
			GrSleepData[i+offset].snoring = 0;
			GrSleepData[i+offset].breathingPaused = 0;
			GrSleepData[i+offset].bodyTempVariation = 0;
			GrSleepData[i+offset].roomTemp = 0;
		}
	}
	LcdUpdateGrSleep(GrSleepData, (uint8_t)(dataSize + offset));
}

int SleepStagesReponse(SleepStageOutput_t* SleepStageOutput, uint16_t dataSize){
	cJSON *item;
	cJSON *root=cJSON_CreateObject();
	if (!root)
	{
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());
		return ERROR;
	}
	else
	{
		item=cJSON_CreateString(MacAddressGet());
		cJSON_AddItemToObject(root,"dev_id",item);

		cJSON *SleepOuputArr = cJSON_CreateArray();

		cJSON *SleepOuputArrObj = cJSON_CreateObject();


		item=cJSON_CreateString(Unum2str(SleepStageOutput->totalTime));
		cJSON_AddItemToObject(SleepOuputArrObj,"totalTime",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->totalSleepTime));
		cJSON_AddItemToObject(SleepOuputArrObj,"totalSleepTime",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->timeAwake));
		cJSON_AddItemToObject(SleepOuputArrObj,"timeAwake",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->timeRem));
		cJSON_AddItemToObject(SleepOuputArrObj,"timeRem",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->timeLight));
		cJSON_AddItemToObject(SleepOuputArrObj,"timeLight",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->timeDeep));
		cJSON_AddItemToObject(SleepOuputArrObj,"timeDeep",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->percentAwake));
		cJSON_AddItemToObject(SleepOuputArrObj,"percentAwake",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->percentRem));
		cJSON_AddItemToObject(SleepOuputArrObj,"percentRem",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->percentLight));
		cJSON_AddItemToObject(SleepOuputArrObj,"percentLight",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->percentDeep));
		cJSON_AddItemToObject(SleepOuputArrObj,"percentDeep",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->scoreApnea));
		cJSON_AddItemToObject(SleepOuputArrObj,"scoreApnea",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->scoreSnoring));
		cJSON_AddItemToObject(SleepOuputArrObj,"scoreSnoring",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->scoreHr));
		cJSON_AddItemToObject(SleepOuputArrObj,"scoreHr",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->scoreRem));
		cJSON_AddItemToObject(SleepOuputArrObj,"scoreRem",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->scoreDeep));
		cJSON_AddItemToObject(SleepOuputArrObj,"scoreDeep",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->scoreSleepTime));
		cJSON_AddItemToObject(SleepOuputArrObj,"scoreSleepTime",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->scoreBtc));
		cJSON_AddItemToObject(SleepOuputArrObj,"scoreBtc",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->scoreRoomTemp));
		cJSON_AddItemToObject(SleepOuputArrObj,"scoreRoomTemp",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->perfectScore));
		cJSON_AddItemToObject(SleepOuputArrObj,"perfectScore",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->percentScore));
		cJSON_AddItemToObject(SleepOuputArrObj,"percentScore",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->worstScore1));
		cJSON_AddItemToObject(SleepOuputArrObj,"worstScore1",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->worstScore2));
		cJSON_AddItemToObject(SleepOuputArrObj,"worstScore2",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->totalApnea));
		cJSON_AddItemToObject(SleepOuputArrObj,"totalApnea",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->totalSnoring));
		cJSON_AddItemToObject(SleepOuputArrObj,"totalSnoring",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->totalSnoringPerHour));
		cJSON_AddItemToObject(SleepOuputArrObj,"totalSnoringPerHour",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->avgHr));
		cJSON_AddItemToObject(SleepOuputArrObj,"avgHr",item);

		item=cJSON_CreateString(Unum2str(SleepStageOutput->avgRr));
		cJSON_AddItemToObject(SleepOuputArrObj,"avgRr",item);

		item=cJSON_CreateString(Num2str(SleepStageOutput->btcMax));
		cJSON_AddItemToObject(SleepOuputArrObj,"btcMax",item);

		item=cJSON_CreateString(Num2str(SleepStageOutput->avgBodyTemp));
		cJSON_AddItemToObject(SleepOuputArrObj,"btcMin",item);

		item=cJSON_CreateString(Num2str(SleepStageOutput->avgRoomTemp));
		cJSON_AddItemToObject(SleepOuputArrObj,"avgRoomTemp",item);

		item=cJSON_CreateString(Num2str(SleepStageOutput->avgBodyTemp));
		cJSON_AddItemToObject(SleepOuputArrObj,"avgBodyTemp",item);

		cJSON *vitalSignalArr = cJSON_CreateArray();
		int i = 0;
		for(i = 0; i < 5; i++){
			item=cJSON_CreateString(Num2str(SleepStageOutput->vitalSinal[i]));
			cJSON_AddItemToObject(vitalSignalArr,"",item);
		}
		cJSON_AddItemToObject(SleepOuputArrObj,"vitalSignalArr",vitalSignalArr);

		cJSON *sleepStageArr = cJSON_CreateArray();
		cJSON *bedArr = cJSON_CreateArray();
		cJSON *hrArr = cJSON_CreateArray();
		cJSON *rrArr = cJSON_CreateArray();
		cJSON *snoringArr = cJSON_CreateArray();
		cJSON *apneaArr = cJSON_CreateArray();
		cJSON *btcArr = cJSON_CreateArray();
		cJSON *roomTempArr = cJSON_CreateArray();
		cJSON *unixTime = cJSON_CreateArray();

		timeStartEnd.timeStart = SleepStageOutput->unixTime[0];
		timeStartEnd.timeEnd = SleepStageOutput->unixTime[dataSize - 1];

		for(i = 0; i < dataSize; i++){
			item=cJSON_CreateString(Num2str(SleepStageOutput->sleepStageArr[i]));
			cJSON_AddItemToObject(sleepStageArr,"",item);

			item=cJSON_CreateString(Num2str(SleepStageOutput->bedArr[i]));
			cJSON_AddItemToObject(bedArr,"",item);

			item=cJSON_CreateString(Num2str(SleepStageOutput->hrArr[i]));
			cJSON_AddItemToObject(hrArr,"",item);

			item=cJSON_CreateString(Num2str(SleepStageOutput->rrArr[i]));
			cJSON_AddItemToObject(rrArr,"",item);

			item=cJSON_CreateString(Num2str(SleepStageOutput->snoringArr[i]));
			cJSON_AddItemToObject(snoringArr,"",item);

			item=cJSON_CreateString(Num2str(SleepStageOutput->apneaArr[i]));
			cJSON_AddItemToObject(apneaArr,"",item);

			item=cJSON_CreateString(Num2str(SleepStageOutput->btcArr[i]));
			cJSON_AddItemToObject(btcArr,"",item);

			item=cJSON_CreateString(Num2str(SleepStageOutput->roomTempArr[i]));
			cJSON_AddItemToObject(roomTempArr,"",item);

			item=cJSON_CreateString(Num2str(SleepStageOutput->unixTime[i]));
			cJSON_AddItemToObject(unixTime,"",item);


		}
		cJSON_AddItemToObject(SleepOuputArrObj,"sleepStageArr",sleepStageArr);
		cJSON_AddItemToObject(SleepOuputArrObj,"bedArr",bedArr);
		cJSON_AddItemToObject(SleepOuputArrObj,"hrArr",hrArr);
		cJSON_AddItemToObject(SleepOuputArrObj,"rrArr",rrArr);
		cJSON_AddItemToObject(SleepOuputArrObj,"snoringArr",snoringArr);
		cJSON_AddItemToObject(SleepOuputArrObj,"apneaArr",apneaArr);
		cJSON_AddItemToObject(SleepOuputArrObj,"btcArr",btcArr);
		cJSON_AddItemToObject(SleepOuputArrObj,"roomTempArr",roomTempArr);
		cJSON_AddItemToObject(SleepOuputArrObj,"unixTimeArr",unixTime);



		cJSON_AddItemToArray(SleepOuputArr,SleepOuputArrObj);

		cJSON_AddItemToObject(root,"datasleepscore",SleepOuputArr);
	}
	char * out = NULL;
	out = cJSON_PrintUnformatted(root);
	printf("%s\n", out);
	OsMQTTPublish(SLEEP_STAGES_RESPONSE_ID, out, 0);
	cJSON_Delete(root);
	if(out!=NULL)
	{
		vPortFree(out);
	}
	return SUCCESS;
}

int SleepStagesParseSleepScoreData(SleepStageOutput_t *sleepInfo){
	HttpRpData_t httpSleepData;
	dataIndex = 0;

	if(SUCCESS != ServerRequestGetSleepScore(0,0, &httpSleepData)){
		LOG_ERROR("ServerRequestGetSleepScore error");
		return ERROR;
	}

	cJSON *jSleepData = cJSON_Parse(httpSleepData.data);
	vPortFree(httpSleepData.data);
	if(jSleepData == NULL)
	{
		vPortFree(httpSleepData.data);
		return ERROR;
	}
	int sizeArr = cJSON_GetArraySize(jSleepData);
	LOG_DEBUGF("jSleepData array size : %d\n", sizeArr);

	if(sizeArr == 0){
		cJSON_Delete(jSleepData);
//		memset(sleepInfo,0,sizeof(SleepStageOutput_t));
		return ERROR;
	}
	else{
//		memset(sleepInfo,0,sizeof(SleepStageOutput_t));
		cJSON *jSleep = cJSON_GetArrayItem(jSleepData, 0);
		if(NULL == jSleep)
		{
			return ERROR;
		}

		cJSON *jTotalTime = cJSON_GetObjectItemCaseSensitive(jSleep, "totalTime");
		if(jTotalTime == NULL){
			return ERROR;
		}
		else{
			sleepInfo->totalTime = atoi(jTotalTime->valuestring);
		}

		cJSON *jTimeAwake= cJSON_GetObjectItemCaseSensitive(jSleep, "timeAwake");
		if(jTimeAwake == NULL){
			return ERROR;
		}
		else{
			sleepInfo->timeAwake = atoi(jTimeAwake->valuestring);
		}

		cJSON *jTimeRem= cJSON_GetObjectItemCaseSensitive(jSleep, "timeRem");
		if(jTimeRem == NULL){
			return ERROR;
		}
		else{
			sleepInfo->timeRem = atoi(jTimeRem->valuestring);
		}

		cJSON *jTimeLight= cJSON_GetObjectItemCaseSensitive(jSleep, "timeLight");
		if(jTimeLight == NULL){
			return ERROR;
		}
		else{
			sleepInfo->timeLight = atoi(jTimeLight->valuestring);
		}

		cJSON *jTimeDeep= cJSON_GetObjectItemCaseSensitive(jSleep, "timeDeep");
		if(jTimeDeep == NULL){
			return ERROR;
		}
		else{
			sleepInfo->timeDeep = atoi(jTimeDeep->valuestring);
		}

		cJSON *jPercentAwake= cJSON_GetObjectItemCaseSensitive(jSleep, "percentAwake");
		if(jPercentAwake == NULL){
			return ERROR;
		}
		else{
			sleepInfo->percentAwake = atoi(jPercentAwake->valuestring);
		}

		cJSON *jPercentRem= cJSON_GetObjectItemCaseSensitive(jSleep, "percentRem");
		if(jPercentRem == NULL){
			return ERROR;
		}
		else{
			sleepInfo->percentRem = atoi(jPercentRem->valuestring);
		}

		cJSON *jPercentLight= cJSON_GetObjectItemCaseSensitive(jSleep, "percentLight");
		if(jPercentLight == NULL){
			return ERROR;
		}
		else{
			sleepInfo->percentLight = atoi(jPercentLight->valuestring);
		}

		cJSON *jPercentDeep= cJSON_GetObjectItemCaseSensitive(jSleep, "percentDeep");
		if(jPercentDeep == NULL){
			return ERROR;
		}
		else{
			sleepInfo->percentDeep = atoi(jPercentDeep->valuestring);
		}

		cJSON *jScoreApnea= cJSON_GetObjectItemCaseSensitive(jSleep, "scoreApnea");
		if(jScoreApnea == NULL){
			return ERROR;
		}
		else{
			sleepInfo->scoreApnea = atoi(jScoreApnea->valuestring);
		}

		cJSON *jScoreSnoring= cJSON_GetObjectItemCaseSensitive(jSleep, "scoreSnoring");
		if(jScoreSnoring == NULL){
			return ERROR;
		}
		else{
			sleepInfo->scoreSnoring = atoi(jScoreSnoring->valuestring);
		}

		cJSON *jScoreHr= cJSON_GetObjectItemCaseSensitive(jSleep, "scoreHr");
		if(jScoreHr == NULL){
			return ERROR;
		}
		else{
			sleepInfo->scoreHr = atoi(jScoreHr->valuestring);
		}

		cJSON *jScoreRem= cJSON_GetObjectItemCaseSensitive(jSleep, "scoreRem");
		if(jScoreRem == NULL){
			return ERROR;
		}
		else{
			sleepInfo->scoreRem = atoi(jScoreRem->valuestring);
		}

		cJSON *jScoreDeep= cJSON_GetObjectItemCaseSensitive(jSleep, "scoreDeep");
		if(jScoreDeep == NULL){
			return ERROR;
		}
		else{
			sleepInfo->scoreDeep = atoi(jScoreDeep->valuestring);
		}

		cJSON *jScoreSleepTime= cJSON_GetObjectItemCaseSensitive(jSleep, "scoreSleepTime");
		if(jScoreSleepTime == NULL){
			return ERROR;
		}
		else{
			sleepInfo->scoreSleepTime = atoi(jScoreSleepTime->valuestring);
		}

		cJSON *jScoreBtc= cJSON_GetObjectItemCaseSensitive(jSleep, "scoreBtc");
		if(jScoreBtc == NULL){
			return ERROR;
		}
		else{
			sleepInfo->scoreSleepTime = atoi(jScoreBtc->valuestring);
		}

		cJSON *jSoreRoomTemp= cJSON_GetObjectItemCaseSensitive(jSleep, "scoreRoomTemp");
		if(jSoreRoomTemp == NULL){
			return ERROR;
		}
		else{
			sleepInfo->scoreRoomTemp = atoi(jSoreRoomTemp->valuestring);
		}

		cJSON *jPerfectScore= cJSON_GetObjectItemCaseSensitive(jSleep, "perfectScore");
		if(jPerfectScore == NULL){
			return ERROR;
		}
		else{
			sleepInfo->perfectScore = atoi(jPerfectScore->valuestring);
		}

		cJSON *jPercentScore = cJSON_GetObjectItemCaseSensitive(jSleep, "percentScore");
		if(jPercentScore == NULL){
			return ERROR;
		}
		else{
			sleepInfo->percentScore = atoi(jPercentScore->valuestring);
		}

		cJSON *jWorstScore1 = cJSON_GetObjectItemCaseSensitive(jSleep, "worstScore1");
		if(jWorstScore1 == NULL){
			return ERROR;
		}
		else{
			sleepInfo->worstScore1 = atoi(jWorstScore1->valuestring);
		}

		cJSON *jWorstScore2 = cJSON_GetObjectItemCaseSensitive(jSleep, "worstScore2");
		if(jWorstScore2 == NULL){
			return ERROR;
		}
		else{
			sleepInfo->worstScore2 = atoi(jWorstScore2->valuestring);
		}


		cJSON *jTotalApnea = cJSON_GetObjectItemCaseSensitive(jSleep, "totalApnea");
		if(jTotalApnea == NULL){
			return ERROR;
		}
		else{
			sleepInfo->totalApnea = atoi(jTotalApnea->valuestring);
		}

		cJSON *totalSnoring = cJSON_GetObjectItemCaseSensitive(jSleep, "totalSnoring");
		if(totalSnoring == NULL){
			return ERROR;
		}
		else{
			sleepInfo->totalSnoring = atoi(totalSnoring->valuestring);
		}

		cJSON *totalSnoringPerHour = cJSON_GetObjectItemCaseSensitive(jSleep, "totalSnoringPerHour");
		if(totalSnoringPerHour == NULL){
			return ERROR;
		}
		else{
			sleepInfo->totalSnoringPerHour = atoi(totalSnoringPerHour->valuestring);
		}

		cJSON *avgHr = cJSON_GetObjectItemCaseSensitive(jSleep, "avgHr");
		if(avgHr == NULL){
			return ERROR;
		}
		else{
			sleepInfo->avgHr = atoi(avgHr->valuestring);
		}

		cJSON *avgRr = cJSON_GetObjectItemCaseSensitive(jSleep, "avgRr");
		if(avgRr == NULL){
			return ERROR;
		}
		else{
			sleepInfo->avgRr = atoi(avgRr->valuestring);
		}

		cJSON *btcMax = cJSON_GetObjectItemCaseSensitive(jSleep, "btcMax");
		if(btcMax == NULL){
			return ERROR;
		}
		else{
			sleepInfo->btcMax = atoi(btcMax->valuestring);
		}

		cJSON *btcMin = cJSON_GetObjectItemCaseSensitive(jSleep, "btcMin");
		if(btcMin == NULL){
			return ERROR;
		}
		else{
			sleepInfo->btcMin = atoi(btcMin->valuestring);
		}

		cJSON *avgRoomTemp = cJSON_GetObjectItemCaseSensitive(jSleep, "avgRoomTemp");
		if(avgRoomTemp == NULL){
			return ERROR;
		}
		else{
			sleepInfo->avgRoomTemp = atoi(avgRoomTemp->valuestring);
		}

		cJSON *jSleepStageArrs = cJSON_GetObjectItemCaseSensitive(jSleep, "sleepStageArr");
		cJSON *jSleepStageArr = NULL;
		uint16_t idxArr = 0;
		if(jSleepStageArrs == NULL){
			return ERROR;
		}
		else{
			cJSON_ArrayForEach(jSleepStageArr, jSleepStageArrs){
				sleepInfo->sleepStageArr[idxArr] = atoi(jSleepStageArr->valuestring);
				idxArr++;
				if(idxArr > SLEEP_STAGE_MAX_SIZE){
					return ERROR;
				}
			}
		}

		cJSON *jBedArrs = cJSON_GetObjectItemCaseSensitive(jSleep, "bedArr");
		cJSON *jBedArr = NULL;
		idxArr = 0;
		if(jBedArrs == NULL){
			return ERROR;
		}
		else{
			cJSON_ArrayForEach(jBedArr, jBedArrs){
				sleepInfo->bedArr[idxArr] = atoi(jBedArr->valuestring);
				idxArr++;
				if(idxArr > SLEEP_STAGE_MAX_SIZE){
					return ERROR;
				}
			}
		}

		cJSON *jHrArrs = cJSON_GetObjectItemCaseSensitive(jSleep, "hrArr");
		cJSON *jHrArr = NULL;
		idxArr = 0;
		if(jHrArrs == NULL){
			return ERROR;
		}
		else{
			cJSON_ArrayForEach(jHrArr, jHrArrs){
				sleepInfo->hrArr[idxArr] = atoi(jHrArr->valuestring);
				idxArr++;
				if(idxArr > SLEEP_STAGE_MAX_SIZE){
					return ERROR;
				}
			}
		}


		cJSON *jRrArrs = cJSON_GetObjectItemCaseSensitive(jSleep, "rrArr");
		cJSON *jRrArr = NULL;
		idxArr = 0;
		if(jRrArrs == NULL){
			return ERROR;
		}
		else{
			cJSON_ArrayForEach(jRrArr, jRrArrs){
				sleepInfo->rrArr[idxArr] = atoi(jRrArr->valuestring);
				idxArr++;
				if(idxArr > SLEEP_STAGE_MAX_SIZE){
					return ERROR;
				}
			}
		}

		cJSON *jSnoringArrs = cJSON_GetObjectItemCaseSensitive(jSleep, "snoringArr");
		cJSON *jSnoringArr = NULL;
		idxArr = 0;
		if(jSnoringArrs == NULL){
			return ERROR;
		}
		else{
			cJSON_ArrayForEach(jSnoringArr, jSnoringArrs){
				sleepInfo->snoringArr[idxArr] = atoi(jSnoringArr->valuestring);
				idxArr++;
				if(idxArr > SLEEP_STAGE_MAX_SIZE){
					return ERROR;
				}
			}
		}


		cJSON *jApneaArrs = cJSON_GetObjectItemCaseSensitive(jSleep, "apneaArr");
		cJSON *jApneaArr = NULL;
		idxArr = 0;
		if(jApneaArrs == NULL){
			return ERROR;
		}
		else{
			cJSON_ArrayForEach(jApneaArr, jApneaArrs){
				sleepInfo->apneaArr[idxArr] = atoi(jApneaArr->valuestring);
				idxArr++;
				if(idxArr > SLEEP_STAGE_MAX_SIZE){
					return ERROR;
				}
			}
		}


		cJSON *jBtcArrs = cJSON_GetObjectItemCaseSensitive(jSleep, "btcArr");
		cJSON *jBtcArr = NULL;
		idxArr = 0;
		if(jBtcArrs == NULL){
			return ERROR;
		}
		else{
			cJSON_ArrayForEach(jBtcArr, jBtcArrs){
				sleepInfo->btcArr[idxArr] = atoi(jBtcArr->valuestring);
				idxArr++;
				if(idxArr > SLEEP_STAGE_MAX_SIZE){
					return ERROR;
				}
			}
		}

		cJSON *jRoomTempArrs = cJSON_GetObjectItemCaseSensitive(jSleep, "roomTempArr");
		cJSON *jRoomTempArr = NULL;
		idxArr = 0;
		if(jRoomTempArrs == NULL){
			return ERROR;
		}
		else{
			cJSON_ArrayForEach(jRoomTempArr, jRoomTempArrs){
				sleepInfo->roomTempArr[idxArr] = atoi(jRoomTempArr->valuestring);
				idxArr++;
				if(idxArr > SLEEP_STAGE_MAX_SIZE){
					return ERROR;
				}
			}
		}

		cJSON *vitalSignalArrs = cJSON_GetObjectItemCaseSensitive(jSleep, "vitalSignalArr");
		cJSON *vitalSignalArr = NULL;
		idxArr = 0;
		if(vitalSignalArrs == NULL){
			return ERROR;
		}
		else{
			cJSON_ArrayForEach(vitalSignalArr, vitalSignalArrs){
				sleepInfo->vitalSinal[idxArr] = atoi(vitalSignalArr->valuestring);
				idxArr++;
				if(idxArr > 5){
					return ERROR;
				}
			}
		}

		cJSON *unixTimeArrs = cJSON_GetObjectItemCaseSensitive(jSleep, "unixTimeArr");
		cJSON *unixTimeArr = NULL;
		idxArr = 0;
		if(vitalSignalArrs == NULL){
			return ERROR;
		}
		else{
			cJSON_ArrayForEach(unixTimeArr, unixTimeArrs){
				sleepInfo->unixTime[idxArr] = atoi(unixTimeArr->valuestring);
				idxArr++;
				if(idxArr > SLEEP_STAGE_MAX_SIZE){
					return ERROR;
				}
			}
		}
		dataIndex = idxArr;
	}

	return SUCCESS;
}


void SleepStagesGetSleepScoreAndUpdateLcd(){
	if(SUCCESS == SleepStagesParseSleepScoreData(&SleepStageOutput)){
		SleepStagesUpdateLcd();
	}
}
