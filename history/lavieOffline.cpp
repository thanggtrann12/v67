/*
 * lavieOffline.cpp
 *
 *  Created on: Jun 2, 2021
 *      Author: Thien Phan
 */

#include <history/lavieOffline.h>
#include <string.h>
#include <cstdlib>
#include "stm32f7xx_hal.h"
#include "OslavieMqttHandle.h"
#include "rtcLavie.h"
#include "resp_detect.h"
#include "Utils.h"
#include "MacAddress.h"

#define MIN_RR 5
#define MAX_RR 40
#define MIN_HR 40
#define MAX_HR 120


extern osMessageQueueId_t Hub2Broker_QHandle;

lavieOffline::lavieOffline() {
	// TODO Auto-generated constructor stub
	this->indexOffline = 0;
}

lavieOffline::~lavieOffline() {
	// TODO Auto-generated destructor stub
	this->indexOffline = 0;
}

lavie_data_t lavieOffline::get_offline_data(uint16_t index)
{
	return this->lavieRTdata15M[index];
}

void lavieOffline::set_offline_data(lavie_data_t lavie_data)
{
	if(this->indexOffline == 0)
	{
		char *rtc_time = RTC_Time_String(0);
		memcpy(this->time, rtc_time, sizeof(time));
		memset(this->lavieRTdata15M, 0, NUM_DATA_15M*sizeof(lavie_data_t));
	}

	this->lavieRTdata15M[this->indexOffline] = lavie_data;
	this->indexOffline++;
	if(this->indexOffline >= NUM_DATA_15M)
	{
		this->indexOffline = 0;
		// calculate min, max, avg, total
		calculate_offline_data(lavieRTdata15M, NUM_DATA_15M);
		publish_offline_payload();
	}
}

void lavieOffline::reset_index_offline(void)
{
	this->indexOffline = 0;
}

void lavieOffline::calculate_offline_data(lavie_data_t *lavieRtData15M, uint16_t size)
{
	this->instantHrMin = 0;
	this->instantHrAvg = 0;
	this->instantHrMax = 0;
	this->actualHrMin = 0;
	this->actualHrAvg = 0;
	this->actualHrMax = 0;
	this->rrMin = 0;
	this->rrAvg = 0;
	this->rrMax = 0;
	this->totalApnea = 0;
	this->totalMotions = 0;
	this->totalSnoring = 0;
	this->Th_rtMin = 0;
	this->Th_rtAvg = 0;
	this->Th_rtMax = 0;

	this->Th_bgMin = 0;
	this->Th_bgAvg = 0;
	this->Th_bgMax = 0;

	this->roomTempMin = 0;
	this->roomTempAvg = 0;
	this->roomTempMax = 0;

	this->roomHumMin = 0;
	this->roomHumAvg = 0;
	this->roomHumMax = 0;

	this->bodyTempMin = 0;
	this->bodyTempAvg = 0;
	this->bodyTempMax = 0;

	this->bodyTempVarMin = 0;
	this->bodyTempVarAvg = 0;
	this->bodyTempVarMax = 0;

	this->pnpBgWeak = 0;
	this->pnpBgOk = 0;
	this->pnpBgGood = 0;
	this->pnpBgVGood = 0;
	this->pnpBgExcel = 0;

	this->pnpRtMin = 0;
	this->pnpRtAvg = 0;
	this->pnpRtMax = 0;

	this->movEnergyMin = 0;
	this->movEnergyAvg = 0;
	this->movEnergyMax = 0;

	this->maxPeak = 0;

	uint8_t sizeInstantHr = 1, sizeActualHr = 1, sizeRr = 1, sizeTh_rt = 1, sizeTh_bg = 1, sizePnpRt = 1,
	 sizeRoomTemp = 1, sizeRoomHum = 1, sizeBodyTemp = 1, sizeBodyTempVar = 1, sizeMovEnergy = 1;
//	uint32_t sumInstantHr = 0, sumActualHr = 0, sumRr = 0, sumRoomTemp = 0, sumBodyTemp = 0, sumRoomHum = 0;
//	float  sumTh_rtAvg = 0, sumTh_rt = 0, sumTh_bg = 0, sumPnpBg = 0, sumPnpRt = 0;

	// total apnea
	this->totalApnea = getCountApnea();
	for(uint16_t i = 0; i < size; i++)
	{
		// instantHr
		if((lavieRtData15M[i].hr_rr.instant_hr >= MIN_HR) && (lavieRtData15M[i].hr_rr.instant_hr <= MAX_HR))
		{
			if(this->instantHrMin == 0 && this->instantHrMax == 0 && this->instantHrAvg == 0)
			{
				this->instantHrMin = lavieRtData15M[i].hr_rr.instant_hr;
				this->instantHrAvg = lavieRtData15M[i].hr_rr.instant_hr;
				this->instantHrMax = lavieRtData15M[i].hr_rr.instant_hr;
			}
			else{
				this->instantHrAvg += lavieRtData15M[i].hr_rr.instant_hr;
				sizeInstantHr++;
			}
			if(this->instantHrMin > lavieRtData15M[i].hr_rr.instant_hr)
			{
				this->instantHrMin = lavieRtData15M[i].hr_rr.instant_hr;
			}
			if(this->instantHrMax < lavieRtData15M[i].hr_rr.instant_hr)
			{
				this->instantHrMax = lavieRtData15M[i].hr_rr.instant_hr;
			}
		}

		// ActualHr
		if((lavieRtData15M[i].hr_rr.actual_hr >= MIN_HR) && (lavieRtData15M[i].hr_rr.actual_hr <= MAX_HR))
		{
			if(this->actualHrMin == 0 && this->actualHrMax == 0 && this->actualHrAvg == 0)
			{
				this->actualHrMin = lavieRtData15M[i].hr_rr.actual_hr;
				this->actualHrAvg = lavieRtData15M[i].hr_rr.actual_hr;
				this->actualHrMax = lavieRtData15M[i].hr_rr.actual_hr;
			}
			else
			{
				this->actualHrAvg += lavieRtData15M[i].hr_rr.actual_hr;
				sizeActualHr++;
			}
			if(this->actualHrMin > lavieRtData15M[i].hr_rr.actual_hr)
			{
				this->actualHrMin = lavieRtData15M[i].hr_rr.actual_hr;
			}
			if(this->actualHrMax < lavieRtData15M[i].hr_rr.actual_hr)
			{
				this->actualHrMax = lavieRtData15M[i].hr_rr.actual_hr;
			}
		}

		// RR
		if((lavieRtData15M[i].hr_rr.rr >= MIN_RR) && (lavieRtData15M[i].hr_rr.rr <= MAX_RR))
		{
			if(this->rrMin == 0 && this->rrMax == 0 && this->rrAvg == 0)
			{
				this->rrMin = lavieRtData15M[i].hr_rr.rr;
				this->rrAvg = lavieRtData15M[i].hr_rr.rr;
				this->rrMax = lavieRtData15M[i].hr_rr.rr;
			}
			else
			{
				this->rrAvg += lavieRtData15M[i].hr_rr.rr;
				sizeRr++;
			}
			if(this->rrMin > lavieRtData15M[i].hr_rr.rr)
			{
				this->rrMin = lavieRtData15M[i].hr_rr.rr;
			}
			if(this->rrMax < lavieRtData15M[i].hr_rr.rr)
			{
				this->rrMax = lavieRtData15M[i].hr_rr.rr;
			}
		}

		// total motions
		this->totalMotions += lavieRtData15M[i].hr_rr.motions;
		// total snoring
		this->totalSnoring += lavieRtData15M[i].hr_rr.snoring;
		// threshold realtime
		if(lavieRtData15M[i].hr_rr.Th_rt > 0)
		{
			if(this->Th_rtMin == 0 && this->Th_rtMax == 0 && this->Th_rtAvg == 0)
			{
				this->rrMin = lavieRtData15M[i].hr_rr.Th_rt;
				this->rrAvg = lavieRtData15M[i].hr_rr.Th_rt;
				this->rrMax = lavieRtData15M[i].hr_rr.Th_rt;
			}
			else
			{
				this->Th_rtAvg += lavieRtData15M[i].hr_rr.Th_rt;
				sizeTh_rt++;
			}

			if(this->Th_rtMin > lavieRtData15M[i].hr_rr.Th_rt)
			{
				this->Th_rtMin = lavieRtData15M[i].hr_rr.Th_rt;
			}
			if(this->Th_rtMax < lavieRtData15M[i].hr_rr.Th_rt)
			{
				this->Th_rtMax = lavieRtData15M[i].hr_rr.Th_rt;
			}
		}

		// threshold background
		if(lavieRtData15M[i].hr_rr.Th_bg > 0)
		{
			if(this->Th_bgMin == 0 && this->Th_bgMax == 0 && this->Th_bgAvg == 0)
			{
				this->Th_bgMin = lavieRtData15M[i].hr_rr.Th_bg;
				this->Th_bgAvg = lavieRtData15M[i].hr_rr.Th_bg;
				this->Th_bgMax = lavieRtData15M[i].hr_rr.Th_bg;
			}
			else
			{
				this->Th_bgAvg += lavieRtData15M[i].hr_rr.Th_bg;
				sizeTh_bg++;
			}
			if(this->Th_bgMin > lavieRtData15M[i].hr_rr.Th_bg)
			{
				this->Th_bgMin = lavieRtData15M[i].hr_rr.Th_bg;
			}
			if(this->Th_bgMax < lavieRtData15M[i].hr_rr.Th_bg)
			{
				this->Th_bgMax = lavieRtData15M[i].hr_rr.Th_bg;
			}
		}

		// pnp bg
		if(lavieRtData15M[i].hr_rr.pnp_bg > 0)
		{
			if(lavieRtData15M[i].hr_rr.pnp_bg < 3.0)
			{
				this->pnpBgWeak += lavieRtData15M[i].hr_rr.pnp_bg;
			}
			else if(lavieRtData15M[i].hr_rr.pnp_bg < 5.0)
			{
				this->pnpBgOk += lavieRtData15M[i].hr_rr.pnp_bg;
			}
			else if(lavieRtData15M[i].hr_rr.pnp_bg < 10.0)
			{
				this->pnpBgGood += lavieRtData15M[i].hr_rr.pnp_bg;
			}
			else if(lavieRtData15M[i].hr_rr.pnp_bg < 15.0)
			{
				this->pnpBgVGood += lavieRtData15M[i].hr_rr.pnp_bg;
			}
			else
			{
				this->pnpBgExcel += lavieRtData15M[i].hr_rr.pnp_bg;
			}
		}

		// pnp rt
		if(lavieRtData15M[i].hr_rr.pnp_rt > 0)
		{
			if(this->pnpRtMin == 0 && pnpRtMax == 0 && pnpRtAvg == 0)
			{
				this->pnpRtMin = lavieRtData15M[i].hr_rr.pnp_rt;
				this->pnpRtAvg = lavieRtData15M[i].hr_rr.pnp_rt;
				this->pnpRtMax = lavieRtData15M[i].hr_rr.pnp_rt;
			}
			else
			{
				this->pnpRtAvg += lavieRtData15M[i].hr_rr.pnp_rt;
				sizePnpRt++;
			}
			if(this->pnpRtMin > lavieRtData15M[i].hr_rr.pnp_rt)
			{
				this->pnpRtMin = lavieRtData15M[i].hr_rr.pnp_rt;
			}
			if(this->pnpRtMax < lavieRtData15M[i].hr_rr.pnp_rt)
			{
				this->pnpRtMax = lavieRtData15M[i].hr_rr.pnp_rt;
			}
		}

		// threshold roomTemp
		if(lavieRtData15M[i].room_temp)
		{
			if(this->roomTempMin == 0 && this->roomTempMax == 0 && this->roomTempAvg == 0)
			{
				this->roomTempMin = lavieRtData15M[i].room_temp;
				this->roomTempAvg = lavieRtData15M[i].room_temp;
				this->roomTempMax = lavieRtData15M[i].room_temp;
			}
			else
			{
				this->roomTempAvg += lavieRtData15M[i].room_temp;
				sizeRoomTemp++;
			}

			if(this->roomTempMin > lavieRtData15M[i].room_temp)
			{
				this->roomTempMin = lavieRtData15M[i].room_temp;
			}
			if(this->roomTempMax < lavieRtData15M[i].room_temp)
			{
				this->roomTempMax = lavieRtData15M[i].room_temp;
			}
		}

		// threshold roomHum
		if(lavieRtData15M[i].room_hum > 0)
		{
			if(this->roomHumMin == 0 && this->roomHumMax == 0 && this->roomHumAvg == 0)
			{
				this->roomHumMin = lavieRtData15M[i].room_hum;
				this->roomHumAvg = lavieRtData15M[i].room_hum;
				this->roomHumMax = lavieRtData15M[i].room_hum;
			}
			else
			{
				this->roomHumAvg += lavieRtData15M[i].room_hum;
				sizeRoomHum++;
			}

			if(this->roomHumMin > lavieRtData15M[i].room_hum)
			{
				this->roomHumMin = lavieRtData15M[i].room_hum;
			}
			if(this->roomHumMax < lavieRtData15M[i].room_hum)
			{
				this->roomHumMax = lavieRtData15M[i].room_hum;
			}
		}

		// threshold bodyTemp
		if(lavieRtData15M[i].body_temp > BODY_TEMP_MIN && lavieRtData15M[i].body_temp < BODY_TEMP_MAX)
		{
			if(sizeBodyTemp == 1)
			{
				this->bodyTempMin = lavieRtData15M[i].body_temp;
				this->bodyTempAvg = lavieRtData15M[i].body_temp;
				this->bodyTempMax = lavieRtData15M[i].body_temp;
			}
			else
			{
				this->bodyTempAvg += lavieRtData15M[i].body_temp;
				sizeBodyTemp++;
			}

			if(this->bodyTempMin > lavieRtData15M[i].body_temp)
			{
				this->bodyTempMin = lavieRtData15M[i].body_temp;
			}
			if(this->bodyTempMax < lavieRtData15M[i].body_temp)
			{
				this->bodyTempMax = lavieRtData15M[i].body_temp;
			}
		}
		// threshold bodyTempVar
		if(lavieRtData15M[i].body_temp_var != BODY_TEMP_BLANK_CASE)
		{
			if(sizeBodyTempVar == 1)
			{
				this->bodyTempVarMin = lavieRtData15M[i].body_temp_var;
				this->bodyTempVarAvg = lavieRtData15M[i].body_temp_var;
				this->bodyTempVarMax = lavieRtData15M[i].body_temp_var;
			}
			else
			{
				this->bodyTempVarAvg += lavieRtData15M[i].body_temp_var;
				sizeBodyTempVar++;
			}

			if(this->bodyTempVarMin > lavieRtData15M[i].body_temp_var)
			{
				this->bodyTempVarMin = lavieRtData15M[i].body_temp_var;
			}
			if(this->bodyTempVarMax < lavieRtData15M[i].body_temp_var)
			{
				this->bodyTempVarMax = lavieRtData15M[i].body_temp_var;
			}
		}

		// threshold bodyTemp
		if(lavieRtData15M[i].hr_rr.movEnergy != 0)
		{
			if(sizeMovEnergy == 1)
			{
				this->movEnergyMin = lavieRtData15M[i].hr_rr.movEnergy;
				this->movEnergyAvg = lavieRtData15M[i].hr_rr.movEnergy;
				this->movEnergyMax = lavieRtData15M[i].hr_rr.movEnergy;
			}
			else
			{
				this->movEnergyAvg += lavieRtData15M[i].hr_rr.movEnergy;
				sizeMovEnergy++;
			}
			if(this->movEnergyMin > lavieRtData15M[i].hr_rr.movEnergy)
			{
				this->movEnergyMin = lavieRtData15M[i].hr_rr.movEnergy;
			}
			if(this->movEnergyMax < lavieRtData15M[i].hr_rr.movEnergy)
			{
				this->movEnergyMax = lavieRtData15M[i].hr_rr.movEnergy;
			}
		}

		if(this->maxPeak < lavieRtData15M[i].hr_rr.maxPeak){
			this->maxPeak = lavieRtData15M[i].hr_rr.maxPeak;
		}
	}
	this->instantHrAvg = instantHrAvg/sizeInstantHr;
	this->actualHrAvg = actualHrAvg/sizeActualHr;
	this->rrAvg = rrAvg/sizeRr;
	this->Th_rtAvg = Th_rtAvg/sizeTh_rt;
	this->Th_bgAvg = Th_bgAvg/sizeTh_bg;
	this->pnpRtAvg = pnpRtAvg/sizePnpRt;
	this->bodyTempVarAvg = bodyTempVarAvg/sizeBodyTempVar;
	this->bodyTempAvg = bodyTempAvg/sizeBodyTemp;
	if(this->bodyTempAvg == 0)
	{
		this->bodyTempAvg = BODY_TEMP_BLANK_CASE;
		this->bodyTempMax = BODY_TEMP_BLANK_CASE;
		this->bodyTempMin = BODY_TEMP_BLANK_CASE;
	}
	this->roomHumAvg = roomHumAvg/sizeRoomHum;
	this->roomTempAvg = roomTempAvg/sizeRoomTemp;
	this->movEnergyAvg = movEnergyAvg/sizeMovEnergy;
	this->totalSnoring = (uint16_t)roundf((float)this->totalSnoring/size);
}

void lavieOffline::publish_offline_payload(void)
{
	message_publish_t data_publish;
	data_publish.TopicID = LOG_RESPONSE_TOPIC_ID;

	sprintf(data_publish.payload, offline_data_format, MacAddressGet(), instantHrMin, instantHrAvg, instantHrMax, actualHrMin, actualHrAvg, actualHrMax,
	rrMin, rrAvg, rrMax, 0, 0, 0, 0, totalMotions, 0, roomTempMin, roomTempAvg, roomTempMax, roomHumMin, roomHumAvg, roomHumMax, bodyTempMin, bodyTempAvg, bodyTempMax, bodyTempVarMin, bodyTempVarAvg, bodyTempVarMax,
	totalSnoring, 0, totalApnea, 0, (int32_t)pnpBgWeak, (int32_t)pnpBgOk, (int32_t)pnpBgGood, (int32_t)pnpBgVGood, (int32_t)pnpBgExcel, pnpRtMin, pnpRtAvg, pnpRtMax, Th_bgMin, Th_bgAvg, Th_bgMax, maxPeak, movEnergyAvg, movEnergyMax, 1, time);

	osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
}

