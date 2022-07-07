/*
 * Emergency.c
 *
 *  Created on: Mar 14, 2022
 *      Author: Thien Phan
 */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "osMain.h"
#include "tim.h"
#include "log.h"
#include "rtcLavie.h"
#include "OslavieMqttHandle.h"
#include "define.h"
#include "hr_rr.h"
#include "resp_detect.h"
#include "master_process.h"
#include "serial_master.h"
#include "mqtt_interface.h"
#include "MQTTClient.h"
#include "TempHum.h"
#include "WifiHandle.h"
#include "Rtos.h"
#include "Lcd.h"
#include "LcdUpdate.h"
#include "UiConfig.h"
#include "LcdCtrl.h"
#include "Server.h"
#include "pnp.h"
#include "Calib.h"
#include "MacAddress.h"
#include "SleepStages.h"
#include "IoCtrl.h"
#include "Emergency.h"

static EmergencyShow_t EmergencyShow = EMERGENCY_NORMAL;
UserSosMode_t UserSosMode = EMERGENCY_TASK_NORMAL_MODE;

extern wifi_states WifiThreadMode;
static __IO uint16_t count_pre_sos = 0;
static osSemaphoreId_t semEmer;

EmergencyShow_t getEmergencyStatus(void){
	return EmergencyShow;
}

void setEmergencyStatus(EmergencyShow_t Emer){
	EmergencyShow = Emer;
}

void sos_emergency(EmergencyShow_t emerCase)
{
	if(UserSosMode == EMERGENCY_TASK_NORMAL_MODE)
	{
		count_pre_sos = 0;
		setEmergencyStatus(emerCase);
		if(osSemaphoreRelease(semEmer) != osOK)
		{
			osSemaphoreAcquire(semEmer, 0);
			osSemaphoreRelease(semEmer);
		}
	}
}

void EmergencyTaskFn(void *argument)
{
	semEmer = NULL;
	semEmer = osSemaphoreNew(1, 0, NULL);
	if(semEmer == NULL){
		HAL_NVIC_SystemReset();
	}
	while(1)
	{
		switch(UserSosMode)
		{
			case EMERGENCY_TASK_NORMAL_MODE:
			{
				if(osSemaphoreAcquire(semEmer, osWaitForever) == osOK)
				{
					UserSosMode = EMERGENCY_TASK_PRE_SOS_MODE;
				}
				break;
			}
			case EMERGENCY_TASK_PRE_SOS_MODE:
			{
				if(count_pre_sos++ == 0)
				{
					if(LcdCtrlIsSleeping()){
						LcdCtrlSleepOutDisplay();
						osDelay(600);
					}
					LcdUpdateEmergencyShow(getEmergencyStatus());
					buzzer_on();
				}
				if(count_pre_sos >= 50 || UserSosMode == EMERGENCY_TASK_SOS_MODE) //5seconds
				{
					UserSosMode = EMERGENCY_TASK_SOS_MODE;
					break;
				}
				osDelay(100);
				break;
			}
			case EMERGENCY_TASK_SOS_MODE:
			{
				if(WifiThreadMode == WIFI_CONNECTED_STATE)
				{
					PLOG("SOS Emergency!!!");
					if(publish_sos_data(SAFETY_CALLING_TOPIC_ID, getEmergencyStatus()) == SUCCESSS)
					{
						PLOG("publish success");
						UserSosMode = EMERGENCY_TASK_NORMAL_MODE;
						break;
					}
				}
				osDelay(100);
				break;
			}
			default:
			{
				UserSosMode = EMERGENCY_TASK_NORMAL_MODE;
				break;
			}
		}
	}
}



