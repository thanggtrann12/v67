/*
 * rtcLavie.c
 *
 *  Created on: May 31, 2021
 *      Author: Thien Phan
 */
#include "rtc.h"
#include "rtcLavie.h"
#include "log.h"
#include <time.h>
#include <string.h>
#include "Utils.h"
#include "Config.h"
#include "settings.h"

static char timeZone[256];

void RctSetTimeZone(float_t timeZ, char *timeStr){
	SettingSetTimeZone(timeZ);
	bzero(timeZone,sizeof(timeZone));
	strcpy(timeZone,timeStr);
}

float_t RtcGetOffsetTime(){
	return SettingGetTimeZone();
}

char* RtcGetTimeZone(){
	return timeZone;
}

bool RTC_CalendarConfig(Time_t time)
{
	LOG_DEBUG("RTC Calendar Config");
	if((time.tm_year - 100) < 21){
		LOG_ERROR("Time wifi incorrect");
		return false;
	}

	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;

	/* Set Time: 15:00:00 */
	stimestructure.Hours = time.tm_hour;
	stimestructure.Minutes = time.tm_min;
	stimestructure.Seconds = time.tm_sec;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	sdatestructure.Year = time.tm_year - 100;
	sdatestructure.Month = time.tm_mon + 1;
	sdatestructure.Date = time.tm_mday;
	sdatestructure.WeekDay = time.tm_wday + 1;

	if(HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}


	/*##-3- Writes a data in a RTC Backup data Register1 #######################*/
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
	return true;
}


bool check_configed_rtc(void)
{
	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0x32F2)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/**
 * @brief  Display the current time and date.
 * @retval None
 */
char *RTC_Time_String(bool type)
{
	static char date_time[TIME_SIZE];

	Time_t time = {0};
	if(type == 0){
		time = RTC_get_utcTime();
	}
	else{
		time = RTC_get_localTime();
	}


	sprintf((char *)date_time, "%d-%02d-%02dT%02d:%02d:%02dZ", 1900 + time.tm_year, time.tm_mon + 1, time.tm_mday,
			time.tm_hour, time.tm_min, time.tm_sec);
	return date_time;
}

Time_t RTC_get_utcTime(void)
{
	Time_t utcTime;
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructureget;

	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	utcTime.tm_year = sdatestructureget.Year + 100;
	utcTime.tm_mon = sdatestructureget.Month - 1;
	utcTime.tm_mday = sdatestructureget.Date;
	utcTime.tm_hour = stimestructureget.Hours;
	utcTime.tm_min = stimestructureget.Minutes;
	utcTime.tm_sec = stimestructureget.Seconds;
	utcTime.tm_wday = sdatestructureget.WeekDay - 1;
	return utcTime;
}

Time_t RTC_get_localTime(void)
{
	Time_t utcTime = {0};
	Time_t localTime = {0};
	utcTime = RTC_get_utcTime();

	time_t time = mktime(&utcTime);

	time += (time_t)(SettingGetTimeZone() * 3600);

	Time_t *t = localtime(&time);
	memcpy(&localTime,t,sizeof(localTime));
	return localTime;
}

