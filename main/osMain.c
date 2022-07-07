/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
#include "BedManager.h"
#include "SelfFix.h"
#include "SelfReset.h"
/* USER CODE END Includes */

RoomTH_t RoomTH = {0,0};

void sos_emergency(EmergencyShow_t emerCase);

WifiConfigInfo_t wifi_info;
__IO bool publishFailFlag = false;
__IO display_mode_t DisplayMode;

SemaphoreHandle_t semWaitNw;

wifi_states WifiThreadMode;
uint8_t MAC_ADDRESS[18];
static bool config_rtc_flag;
static bool rqConfigWifi = false;
static bool flagRqSleepScore = true;
bool flagGetSeepScoreSever = true;

void WifiReset(){
	WifiThreadMode = WIFI_INIT_STATE;
	LcdUpdateWifiStatus(WIFI_DISCONNECT);
	UgUpdateStatusWifi(WL_DISCONNECTED);
	ServerUpdateStatusWifi(WL_DISCONNECTED);
	WifiComHardResetEsp32();
}

int WifiHandleSignalWifiStatus(DpStatusConnect_t *dt,uint16_t size){
	UgUpdateStatusWifi(dt->status);
	ServerUpdateStatusWifi(dt->status);
	if(dt->status == WL_CONNECTED){
		WifiThreadMode = WIFI_INIT_STATE;
		LOG_DEBUG("WL_CONNECTED");
	}
	else if(dt->status == WL_DISCONNECTED){
		WifiThreadMode = WIFI_INIT_STATE;
		LOG_WARN("WL_DISCONNECTED");
		LcdUpdateWifiStatus(WIFI_DISCONNECT);
	}
	else if(dt->status == WL_SOCKET_CLOSE){
		WifiThreadMode = WIFI_INIT_STATE;
		LOG_WARN("WL_SOCKET_CLOSE");
		LcdUpdateWifiStatus(WIFI_DISCONNECT);
	}
	else if(dt->status == WL_SOCKET_OPEN){
		LOG_DEBUG("WL_SOCKET_OPEN");
		if(pdTRUE != xSemaphoreGive(semWaitNw)){
			LOG_ERROR("xSemaphoreGive(semWaitNw) failed");
		}
	}
	else if(dt->status == WL_OTA_SUCCESS){
		LOG_DEBUG("WL_OTA_SUCCESS");
	}
	else if(dt->status == WL_OTA_FAILED){
		LOG_WARN("WL_OTA_FAILED");
	}
	return HAL_OK;
}

void WifiTaskFn(void *argument)
{
	semWaitNw = xSemaphoreCreateBinary();
	WifiThreadMode = WIFI_INIT_STATE;
	for(;;)
	{
		if(publishFailFlag){
			publishFailFlag = false;
			WifiReset();
		}
		switch (WifiThreadMode) {
		case WIFI_INIT_STATE:
		{
			if(rqConfigWifi){
				IoCtrlLedOn(LED_YELLOW);
				LOG_DEBUG("User button rqconfig wifi");
				if( HAL_OK == WifiComStartSmartConfig()){
					LOG_DEBUG("WifiGotoSmartConfig success");
					rqConfigWifi = false;
					WifiThreadMode = WIFI_INIT_STATE;
					LcdUpdateWifiStatus(WIFI_DISCONNECT);
					UgUpdateStatusWifi(WL_DISCONNECTED);
					ServerUpdateStatusWifi(WL_DISCONNECTED);
					DelayMs(1000);
				}
				else{
					LOG_ERROR("WifiGotoSmartConfig failed");
					break;
				}
			}
			else{
				IoCtrlLedOn(LED_RED);
			}
			LOG_DEBUG("--------------------* Init WIFI *--------------------");
			if(pdTRUE != xSemaphoreTake(semWaitNw,portMAX_DELAY)){
				LOG_DEBUG("xSemaphoreTake(semWaitNw) failed");
				break;
			}
			LOG_DEBUG("network ready");
			if(HAL_OK == WifiComGetInfoWifi(&wifi_info))
			{
				LOG_DEBUG("SSID: %s", wifi_info.ssid);
				LcdUpdateWifiSSID((char*)wifi_info.ssid);
				LcdUpdateWifiPwd((char*)wifi_info.password);
				LOG_DEBUG("PASSKEY: %s", wifi_info.password);
				WifiThreadMode = WIFI_CONNECT_STATE;
				IoCtrlLedOn(LED_GREEN);
				LcdUpdateWifiStatus(WIFI_CONNECT);
			}
			else{
				osDelay(100);
			}
			break;
		}
		case WIFI_CONNECT_STATE:{
			LOG_DEBUG("--------------* Connecting MQTT Broker *----------");
			static uint8_t connect_fail = 0;
			if(mqtt_init() == SUCCESSS)
			{
				connect_fail = 0;
				publishFailFlag = false;
				WifiThreadMode = WIFI_CONNECTED_STATE;
				IoCtrlLedOn(LED_BLUE);
				LOG_DEBUG("--------------* MQTT Connected *----------");

				struct tm time;
				if(HAL_OK == WifiComUtcTime(&time))
				{
					float_t timez;
					char timeZone[256];
					bzero(timeZone,sizeof(timeZone));
					if( SUCCESS == ServerGetTimeZone(&timez,timeZone)){
						RctSetTimeZone(timez,timeZone);
						LocalTimeResponse();
						HubModeOperationResponse();
					}
					else{
						WifiThreadMode = WIFI_INIT_STATE;
						WifiReset();
						break;
					}
					RTC_CalendarConfig(time);
					config_rtc_flag = true;
				}

				if(SettingUserInfoExisted()){
					UserInfoSt_t userInfo;
					SettingGetUserInfo(&userInfo);
					ServerRequest(SERVER_RQ_LOGIN, &userInfo, sizeof(userInfo));
				}
			}
			else
			{
				connect_fail++;
				if(connect_fail > 2)
				{
					connect_fail = 0;
					WifiThreadMode = WIFI_INIT_STATE;
				}
				osDelay(2000);
			}
			break;
		}
		case WIFI_CONNECTED_STATE:{
			static uint32_t check_rtc_count = 0;
			if(publishFailFlag == true)
			{
				WifiThreadMode = WIFI_CONNECT_STATE;
				LOG_ERROR("ERROR:----> PUBLISH FAILED");
				return;
			}
			if(flagRqSleepScore == false){
#if !U_CONFIG_TEST_SLEEP_SCORE
				if((osKernelGetTickCount() - check_rtc_count) > 300000)//5 minutes
#else
				if((osKernelGetTickCount() - check_rtc_count) > 10000)//test
#endif
				{
					check_rtc_count = osKernelGetTickCount();
					if(config_rtc_flag == false){
						struct tm time;
						if(HAL_OK == WifiComUtcTime(&time))
						{
							RTC_CalendarConfig(time);
							config_rtc_flag = true;
						}
					}
					flagRqSleepScore = true;
				}
			}
			else{
				if(ServerCheckToken()){
					if(SLEEP_STATUS_NOT_READY == SleepStagesProcess()){
						if(flagGetSeepScoreSever == true){
							SleepStagesGetSleepScoreAndUpdateLcd();
							flagGetSeepScoreSever = false;
						}
					}
					flagRqSleepScore = false;
				}
			}
			static message_publish_t publish_data;
			osStatus_t	Qstatus = osMessageQueueGet(Hub2Broker_QHandle, &publish_data, NULL, 100);
			if (Qstatus == osOK)
			{
				/*Send to cloud*/
				if(publish_data.retain == 1){
					if(OsMQTTPublish(publish_data.TopicID, publish_data.payload, true) != SUCCESSS)
					{
						LOG_ERROR("PUBLISH FAIL");
						publishFailFlag = true;
						osMessageQueuePut(Hub2Broker_QHandle,&publish_data,0,0);
					}
				}
				else{
					if(OsMQTTPublish(publish_data.TopicID, publish_data.payload, false) != SUCCESSS)
					{
						LOG_ERROR("PUBLISH FAIL");
						publishFailFlag = true;
						osMessageQueuePut(Hub2Broker_QHandle,&publish_data,0,0);
					}
				}
			}
			break;
		}
		default:
			break;
		}
	}
}

/* USER CODE BEGIN Header_hr_rr_thread */
/**
 * @brief Function implementing the HrRrThread thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_hr_rr_thread */
void HrRrTaskFn(void *argument)
{
	/* USER CODE BEGIN hr_rr_thread */
	/* Infinite loop */
	for(;;)
	{
		MasterProcess();
		osDelay(1000);
	}
	/* USER CODE END hr_rr_thread */
}

void DataMasterTaskFn(void *argument)
{
	/* USER CODE BEGIN cloudcom_thread */
	/* Infinite loop */
	osDelay(24500);
	buzzer_on();
	osDelay(200);
	buzzer_off();
	//collect_master_rawData();
	MasterBoardManager();
	for(;;)
	{
		osDelay(1000);
	}
	/* USER CODE END cloudcom_thread */
}

/* USER CODE BEGIN Header_display_thread */
/**
 * @brief Function implementing the DisplayThread thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_display_thread */
static hrRr2Display_t hrRrData = {0};
hrRr2Display_t osMainGetHrRr(){
	return hrRrData;
}
void user_main_mode(void)
{
	static hrRr2Display_t hrRr2Data = {0};
	osStatus_t Qstatus = osMessageQueueGet(HrRr2User_QHandle, &hrRr2Data, NULL, 100);
	if(Qstatus == osOK)
	{
		memcpy(&hrRrData,&hrRr2Data,sizeof(hrRrData));
		if(hrRr2Data.type == HAVE_MOV_DATA)
		{
			LcdCtrlSleepOutDisplay();
			osDelay(50);
		}
		if(hrRr2Data.type == INSTANT_DATA || hrRr2Data.type == EMPTY_DATA)
		{
			if(hrRr2Data.lavie_data.hr_rr.seizure == true)
			{
				LOG_DEBUG("SEIZURE_SOS");
				sos_emergency(EMERGENCY_SEIZURE);
				hrRr2Data.lavie_data.hr_rr.seizure = 0;
			}
			//update body temp
			SensorVal_t ssv = {hrRr2Data.lavie_data.body_temp, 0, hrRr2Data.lavie_data.body_temp_var};
			LcdUpdateTempBody(ssv);
		}
		if(hrRr2Data.type == RESP_LIFE_DATA)
		{
			// update respiration rate
			if(hrRr2Data.lavie_data.hr_rr.resp_status == RESP_VIBRATE)
			{
				LOG_DEBUG("RESP_WARNING");
				buzzer_on();
				osDelay(100);
				buzzer_off();
				osDelay(50);
				buzzer_on();
				osDelay(100);
				buzzer_off();
			}
			else if(hrRr2Data.lavie_data.hr_rr.resp_status == RESP_EMERGENCY)
			{
				LOG_DEBUG("RESP_EMERGENCY");
				sos_emergency(EMERGENCY_BREATHING_STOP);
			}
			hrRr2Data.lavie_data.hr_rr.resp_status = RESP_NORMAL;
		}
		if(hrRr2Data.type == SNORING_DATA  || hrRr2Data.type == EMPTY_DATA)
		{
			// update snoring
			LcdUpdateSnoringNoise(hrRr2Data.lavie_data.hr_rr.snoring_noisy);
			LcdUpdateSnoringLevel(hrRr2Data.lavie_data.hr_rr.snoring);
		}

		/*Show quality signal*/
		RealTimeSignal_t rtSignal = RT_SIG_NOT_OCCUPIED;
		float Pnp_bg = get_pnp_bg();
		if(Pnp_bg > 0 && Pnp_bg < (float)3.0){
			rtSignal = RT_SIG_WEAK;
		}
		else if(Pnp_bg >= (float)3.0 && Pnp_bg < (float)5){
			rtSignal = RT_SIG_OK;
		}
		else if(Pnp_bg >= (float)5 && Pnp_bg < (float)10){
			rtSignal = RT_SIG_GOOD;
		}
		else if(Pnp_bg >= (float)10 && Pnp_bg < (float)15){
			rtSignal = RT_SIG_VERY_GOOD;
		}
		else if(Pnp_bg >= (float)15){
			rtSignal = RT_SIG_EXCELLENT;
		}
		LcdUpdateRtSignal(rtSignal);

		if(hrRr2Data.type == HEART_RATE_DATA || hrRr2Data.type == EMPTY_DATA)
		{
			/*Show actual heart*/
			uint8_t heartRate = 0;
			if(hrRr2Data.lavie_data.hr_rr.actual_hr == ERROR_CASE)
			{
				heartRate = VAL_HR_RR_E;
			}
			else if(hrRr2Data.lavie_data.hr_rr.actual_hr == 0)
			{
				heartRate = VAL_HR_RR_HIDE;
			}
			else if(hrRr2Data.lavie_data.hr_rr.actual_hr == BLANK_CASE)
			{
				heartRate = VAL_HR_RR_HIDE;
			}
			else{
				heartRate = hrRr2Data.lavie_data.hr_rr.actual_hr;
			}
			LcdUpdateHeartRate(heartRate);
			LcdUpdateApnea(hrRr2Data.lavie_data.hr_rr.apnea);
			/*Show motions*/
			LcdUpdateMotion(hrRr2Data.lavie_data.hr_rr.motions);
			//			SelfResetCheck(hrRr2Data.lavie_data.hr_rr.motions, RTC_get_localTime());
			//			SelfFixCheck(hrRr2Data.lavie_data.hr_rr.motions, BedManagerCapaGet());
		}

		if(hrRr2Data.type == RESP_RATE_DATA || hrRr2Data.type == EMPTY_DATA)
		{
			/*Show resp rate*/
			uint8_t respRate = 0;
			if(hrRr2Data.lavie_data.hr_rr.rr == ERROR_CASE)
			{
				respRate = VAL_HR_RR_E;
			}
			else if(hrRr2Data.lavie_data.hr_rr.rr == 0)
			{
				respRate = VAL_HR_RR_HIDE;
			}
			else if(hrRr2Data.lavie_data.hr_rr.rr == BLANK_CASE)
			{
				respRate = VAL_HR_RR_HIDE;
			}
			else{
				respRate = hrRr2Data.lavie_data.hr_rr.rr;
			}
			LcdUpdateRespiration(respRate);
		}

		if(hrRr2Data.lavie_data.bed_occupied == false)
		{

			LcdUpdateImageLung(IMAG_LUNG_SMALL);
			LcdUpdateImageHeart(IMAG_HEART_SMALL);
			LcdUpdateImageHeartHv(IMAG_HEART_SMALL);
		}
	}

	static uint32_t tickUpdateTime = 0, tickUpdateTHroom = 0;
	if(osKernelGetTickCount() - tickUpdateTime > 1000) //1 second
	{
		tickUpdateTime = osKernelGetTickCount();

		Time_t time = RTC_get_localTime();
		//		LcdUpdateTime(time);
		if((time.tm_hour == 6) && (time.tm_min == 0) && (time.tm_sec == 0)){
			LcdUpdateTextNotify("");
		}
	}
	if(osKernelGetTickCount() - tickUpdateTHroom > 4000) //4 second
	{
		tickUpdateTHroom = osKernelGetTickCount();
		if(!GetTempAndHumi(&RoomTH.temp, &RoomTH.hum))
		{
			SensorVal_t ssv = {RoomTH.temp, 0, 0};
			LcdUpdateTempRoom(ssv);
			ssv.val = RoomTH.hum;
			LcdUpdateHumidityRoom(ssv);
		}
	}

	/*Blink lung, heart image if have heart and respiration*/
	static uint32_t count_blink_heart = 0, count_blink_lung = 0;
	static bool blink_heart = false, blink_lung = false;
	if(hrRr2Data.lavie_data.bed_occupied == true && (osKernelGetTickCount() - count_blink_heart > 1000))
	{
		count_blink_heart = osKernelGetTickCount();
		blink_heart ^= 1;
		LcdUpdateImageHeart(blink_heart);
	}

	if(hrRr2Data.lavie_data.bed_occupied == true && (osKernelGetTickCount() - count_blink_lung > 3000))
	{
		count_blink_lung = osKernelGetTickCount();
		blink_lung ^= 1;
		LcdUpdateImageLung(blink_lung);
	}
}

void user_calibration_empty_bed_mode()
{
	LcdCtrlKeepDisplayOn();
	if(CalibEmptyProcess())
	{
		PLOG("Calibration completed");
	}
	else{
		PLOG("Calibration failed");
	}
	DisplayMode = USER_NORMAL_MODE;
	LcdCtrlNormalMode();
	//	osDelay(2000);
	//	HAL_NVIC_SystemReset();
}

void user_calibration_person_lying_mode()
{
	LcdCtrlKeepDisplayOn();
	if(CalibPersonProcess())
	{
		PLOG("Calibration completed");
	}
	else{
		PLOG("Calibration failed");
	}
	DisplayMode = USER_NORMAL_MODE;
	LcdCtrlNormalMode();
	//	osDelay(2000);
	//	HAL_NVIC_SystemReset();
}
static void (*display_state_table[3])(void) = {
		user_main_mode,
		user_calibration_empty_bed_mode,
		user_calibration_person_lying_mode
};

void DisplayTaskFn(void *argument)
{
	UgSetFwVerHub( SettingGetFwVersion());
	DelayMs(1500);
	//	LcdUpdateTextNotify(msgTextNotiWc);
	LcdUpdateWifiStatus(WIFI_DISCONNECT);
	LcdUpdateHubFwVersion(SettingGetFwVersion());
	LcdUpdateMACAddress(MacAddressGet());
	DelayMs(100);
	MasterModeOperation_t masterMode;
	FwVersion_t fwVerMaster = {0,0,0};
	if(SUCCESS == MasterGetModeOperation(&masterMode)){
		if(masterMode == MASTER_MODE_OPERATION_APP){
			if(SUCCESS == MasterGetFwVersion(&fwVerMaster)){
				LOG_DEBUG("MasterFwVer V%d.%d.%d",fwVerMaster.major,fwVerMaster.minor,fwVerMaster.revision);
				LcdUpdateMasterFwVersion(fwVerMaster);
				UgSetFwVerMater(fwVerMaster);

				if(SettingGetCalibStatus() == SETTING_CALIB_NONE){
					LcdUpdateMsgError(msgCalibRequest);
					response_popup_message(msgCalibRequest);
				}
				else if(SettingGetCalibStatus() == SETTING_CALIB_ONLY_PERSON)
				{
					LcdUpdateMsgError(msgCalibRequestEmpty);
					response_popup_message(msgCalibRequestEmpty);
				}
				else if(SettingGetCalibStatus() == SETTING_CALIB_ONLY_EMPTY)
				{
					LcdUpdateMsgError(msgCalibRequestUser);
					response_popup_message(msgCalibRequestUser);
				}
				else{
					//					LcdUpdateMsgHome(msgStartingUp);
					//					response_popup_message(msgStartingUp);
					//LcdUpdateMsgHome("System is started successfully! Enjoy your sleep!");
				}
			}
			else{
				LOG_ERROR("get fwver master failed");
			}
		}
		else{
			LcdUpdateMsgError(msgFwPadError);
		}
	}
	else{
		LOG_ERROR("Pad Not Detected!");
		LcdUpdateMsgError(msgPadNotDetected);
	}
	if(BedManagerPreAutoModeGet()){
		UmainEnableShowNotify();
	}

	LcdUpdateMasterFwVersion(fwVerMaster);
	UgSetFwVerMater(fwVerMaster);
	osDelay(800);
	DisplayMode = USER_NORMAL_MODE;
	for(;;)
	{
		display_state_table[DisplayMode]();
		osThreadYield();
	}
}

#include "event_groups.h"
#include <stdbool.h>

#define BUT_SOS_PRESS_BIT   		(1UL << 0UL)
#define BUT_SOS_RELEASE_BIT   		(1UL << 1UL)
#define TIME_WAIT_SOS				(1000*5)

static EventGroupHandle_t  xEventSosBut;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if 0
	if (GPIO_Pin == GPIO_PIN_0)
	{
		if(GPIO_Pin == SOS_BTN_A_Pin){
			if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(SOS_BTN_A_GPIO_Port, SOS_BTN_A_Pin)){
				BaseType_t xHigherPriorityTaskWoken, xResult;
				xHigherPriorityTaskWoken = pdFALSE;
				xResult = xEventGroupSetBitsFromISR(
						xEventSosBut,
						BUT_SOS_PRESS_BIT,
						&xHigherPriorityTaskWoken );
				if( xResult != pdFAIL )
				{
					portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
				}
			}
			else{
				BaseType_t xHigherPriorityTaskWoken, xResult;
				xHigherPriorityTaskWoken = pdFALSE;
				xResult = xEventGroupSetBitsFromISR(
						xEventSosBut,
						BUT_SOS_RELEASE_BIT,
						&xHigherPriorityTaskWoken );
				if( xResult != pdFAIL )
				{
					portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
				}
			}
		}
	}
#else
	if (GPIO_Pin == GPIO_PIN_0)
	{
		sos_emergency(EMERGENCY_SOS_BUTTON);
	}
	else if(GPIO_Pin == BUTTON_USER_Pin){
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_USER_GPIO_Port, BUTTON_USER_Pin)){
			rqConfigWifi = true;
			WifiThreadMode = WIFI_INIT_STATE;
			IoCtrlLedOn(LED_YELLOW);
		}
	}
#endif
}

void SosButtonCheckFn(void *argument)
{
	bool isPress = false;
	uint32_t timePre = 0;
	uint32_t timeWait = portMAX_DELAY;
	EventBits_t uxBits;
	xEventSosBut  =  xEventGroupCreate();
	if(xEventSosBut == NULL){
		LOG_ERROR("xEventSosBut create failed");
	}
	while(true)
	{
		uxBits = xEventGroupWaitBits(
				xEventSosBut,
				BUT_SOS_PRESS_BIT|BUT_SOS_RELEASE_BIT,
				pdTRUE,
				pdFALSE,
				timeWait);
		//		LOG_DEBUG("ev : 0x%x",uxBits);
		if((uxBits & BUT_SOS_PRESS_BIT) != 0){
			//DisplayMode = USER_NORMAL_MODE;
			timePre = GetTickMs();
			timeWait = TIME_WAIT_SOS;
			isPress = true;
			//			LOG_DEBUG("But press : %d",timePre);
		}
		if((uxBits & BUT_SOS_RELEASE_BIT) != 0){
			//			LOG_DEBUG("But release : %d",GetTickMs());
			if((GetTickMs() - timePre)  > TIME_WAIT_SOS){
				if(isPress){
					sos_emergency(EMERGENCY_SOS_BUTTON);
				}
			}
			else{
				if(LcdCtrlGetStatusDisplay() == DIS_STATUS_ON){
					LcdCtrlSleepInDisplay();
				}
				else{
					LcdCtrlSleepOutDisplay();
				}
			}
			timeWait = portMAX_DELAY;
			isPress = false;
		}
		if((uxBits &(BUT_SOS_PRESS_BIT|BUT_SOS_RELEASE_BIT)) == 0){
			if(isPress){
				sos_emergency(EMERGENCY_SOS_BUTTON);
				isPress = false;
			}
		}
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

