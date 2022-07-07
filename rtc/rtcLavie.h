/*
 * rtcLavie.h
 *
 *  Created on: May 31, 2021
 *      Author: Thien Phan
 */

#ifndef RTC_RTCLAVIE_H_
#define RTC_RTCLAVIE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "define.h"
#include "stdint.h"
#include "stdbool.h"
#include <time.h>
#include <math.h>

typedef struct tm Time_t;

typedef enum {DAY_OF_WEEK, YEAR, MONTH, DATE, HOURS, MINUTES, SECONDS}time_type_t;
bool RTC_CalendarConfig(Time_t time);
bool check_configed_rtc(void);
char *RTC_Time_String(bool type);
Time_t RTC_get_utcTime(void);
Time_t RTC_get_localTime(void);
void RctSetTimeZone(float_t timeZ, char *timeStr);
float_t RtcGetOffsetTime();
char* RtcGetTimeZone();
#ifdef __cplusplus
}
#endif

#endif /* RTC_RTCLAVIE_H_ */
