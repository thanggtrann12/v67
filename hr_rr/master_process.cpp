/***********************************************************************
    %OnSky Inc. Confidential and Proprietary
    %October 2020
		%Code written by Thien Phan
    %***********************************************************************/
#include "gpio.h"
#include "usart.h"
#include "master_process.h"
#include "resp_detect.h"
#include "settings.h"
#include "rtc.h"
#include "serial_master.h"
#include "OslavieMqttHandle.h"
#include "hr_rr.h"
#include "dsp.h"
#include "lifesos.h"
#include "Lcd.h"
#include "lavieOffline.h"
#include "LcdUpdate.h"
#include "LcdCtrl.h"
#include "util/util.h"
#include "pnp.h"
#include "Rtos.h"
#include "snoring.h"
#include "BedManager.h"

#define INVALID_VALUE 255
#define TIME_5MINUTES (uint32_t)300000
#define TIME_3MINUTES (uint32_t)180000

extern uint16_t pnp_bg_index;
extern RoomTH_t RoomTH;

float32_t avgsq_noise_x = 5e+03, avgsq_noise_y = 3.2e+03, avgsq_noise_z = 2e+03, avgsq_noise_xyz = 2.5e+04, avg_energy_xy = MOV_MAX_XY_ENERGY, avg_energy_xyz = MOV_MAX_XYZ_ENERGY, avg_energy_resp = MOV_MAX_RESP_ENERGY;
float32_t noise_dft = 175000000;

instant_data_t instant_data __attribute__((section(STR("_instant_data")))) __attribute__((aligned(4)));
actual_data_t  actual_data __attribute__((section(STR("_actual_data")))) __attribute__((aligned(4)));
actual_data_t  pnp_backgound_data __attribute__((section(STR("_pnp_backgound_data")))) __attribute__((aligned(4)));
resp_data_t    resp_data __attribute__((section(STR("_resp_data")))) __attribute__((aligned(4)));
resplife_data_t resplife_data __attribute__((section(STR("_resplife_data")))) __attribute__((aligned(4)));

static uint32_t timeBaseCheckBodyTemp = 0, timeBaseCheckSeizre = 0;
static uint32_t timeCheckBodyTemp = TIME_5MINUTES;//5 minutes

static lavie_data_t lavie_data_combine = {0};
static uint16_t actual_index = 0;
static uint16_t instant_index = 0;
static uint16_t resp_index = 0;
static uint16_t resplife_index = 0;
static lavieOffline lavieOfflineData;
static int8_t preBodyTemp, holdHr;

static void SendParamsEmpty();

void HrRr2Display(hrRrDataType DATA_ID, lavie_data_t* lavie_data)
{
	hrRr2Display_t hrRr2DisplayData;
	hrRr2DisplayData.type = DATA_ID;
	memcpy(&hrRr2DisplayData.lavie_data, lavie_data, sizeof(lavie_data_t));
	osMessageQueuePut(HrRr2User_QHandle, &hrRr2DisplayData, 0, 0);
}

typedef enum{
	MASTER_BOARD_GET_VERSION,
	MASTER_BOARD_RESET,
	MASTER_BOARD_GET_DATA,
}MasterTaskState_t;
MasterTaskState_t MasterTaskState = MASTER_BOARD_GET_VERSION;

void MasterBoardManager(void)
{
	uint8_t countRequestFail = 0, disconnectFlag = false;
	while(1)
	{
		switch(MasterTaskState)
		{
			case MASTER_BOARD_GET_DATA:
			{
				if(request_hr_data() != true)
				{
					countRequestFail++;
					if(countRequestFail >= 5)
					{
						EmptyProcess(true);
						countRequestFail = 0;
						LOG_DEBUGF((char*)"request master data failed %d", countRequestFail);
						MasterTaskState = MASTER_BOARD_GET_VERSION;
						break;
					}
				}
				osDelay(1);
				break;
			}
			case MASTER_BOARD_GET_VERSION:
			{
				FwVersion_t fwVerMaster = {0,0,0};
				if(SUCCESS == MasterGetFwVersion(&fwVerMaster))
				{
					LOG_DEBUG((char*)"MasterFwVer V%d.%d.%d",fwVerMaster.major,fwVerMaster.minor,fwVerMaster.revision);
					LcdUpdateMasterFwVersion(fwVerMaster);
					UgSetFwVerMater(fwVerMaster);
					if(master_start_sampling() == true)
					{
						if(disconnectFlag == true)
						{
							LcdUpdateHomePageShow();
							disconnectFlag = false;
						}
						LOG_DEBUG((char*)"Start Sampling");
						if(SettingGetModeOperation().mode == OPE_MODE_MANUAL)
						{
							OccupiedProcess(false);
						}
						MasterTaskState = MASTER_BOARD_GET_DATA;
					}
				}
				else{
					if(LcdCtrlIsSleeping() == true){
						LcdCtrlSleepOutDisplay();
						osDelay(600);
					}
					disconnectFlag = true;
					LcdUpdateMsgError(msgPadNotDetected);
					MasterTaskState = MASTER_BOARD_RESET;
				}
				break;
			}
			case MASTER_BOARD_RESET:
			{
				osDelay(2000);
				HardResetMaster();
				osDelay(8000);
				MasterTaskState = MASTER_BOARD_GET_VERSION;
				break;
			}
			default:
				MasterTaskState = MASTER_BOARD_RESET;
				break;
		}
	}
}

void MasterProcess(void)
{
	static bool flagResumPnp = false;
	data_master_node data_rev;
	osStatus_t Qstatus;	

	lavie_data_t lavie_data = {0};

	lavie_data_combine.hr_rr.actual_hr = BLANK_CASE;
	lavie_data_combine.hr_rr.instant_hr = BLANK_CASE;
	lavie_data_combine.hr_rr.rr = BLANK_CASE;
	lavie_data_combine.body_temp = BODY_TEMP_BLANK_CASE;
	lavie_data_combine.hr_rr.snoring = SNORING_NONE;
	lavie_data_combine.bed_occupied = false;

	for(;;)
	{
		Qstatus = osMessageQueueGet(Master_QHandle, &data_rev, NULL, 300000); // send device sensor if have no hrrr data in 5 minutes
		if (Qstatus == osOK)
		{
			if(pnp_bg_index < ACTUAL_DATA_LEN)
			{
				flagResumPnp = true;
				pnp_backgound_data.x1[pnp_bg_index] = (float)data_rev.adc1_x;
				pnp_backgound_data.y1[pnp_bg_index] = (float)data_rev.adc1_y;
				pnp_backgound_data.z1[pnp_bg_index] = (float)data_rev.adc1_z;
#ifdef PAD_3_SENSOR
				pnp_backgound_data.x2[pnp_bg_index] = (float)data_rev.adc2_x;
				pnp_backgound_data.y2[pnp_bg_index] = (float)data_rev.adc2_y;
				pnp_backgound_data.z2[pnp_bg_index] = (float)data_rev.adc2_z;
				pnp_backgound_data.x3[pnp_bg_index] = (float)data_rev.adc3_x;
				pnp_backgound_data.y3[pnp_bg_index] = (float)data_rev.adc3_y;
				pnp_backgound_data.z3[pnp_bg_index] = (float)data_rev.adc3_z;
#endif
				pnp_bg_index++;
			}
			else{
				if(flagResumPnp){
					if(osThreadResume(pnpBGTaskHandle) == osOK)
					{
						flagResumPnp = false;
					}
				}
			}

			instant_data.x1[instant_index] = (float)data_rev.adc1_x;
			instant_data.y1[instant_index] = (float)data_rev.adc1_y;
			instant_data.z1[instant_index] = (float)data_rev.adc1_z;
#ifdef PAD_3_SENSOR
			instant_data.x2[instant_index] = (float)data_rev.adc2_x;
			instant_data.y2[instant_index] = (float)data_rev.adc2_y;
			instant_data.z2[instant_index] = (float)data_rev.adc2_z;
			instant_data.x3[instant_index] = (float)data_rev.adc3_x;
			instant_data.y3[instant_index] = (float)data_rev.adc3_y;
			instant_data.z3[instant_index] = (float)data_rev.adc3_z;
#endif
			instant_index++;
			if(instant_index >= HEART_WINDOW) // enough 5 seconds -> calculate heart rate and respiration rate
			{
				instant_index = 0;
				memset(&lavie_data, 0, sizeof(lavie_data));
				linesos(&instant_data, &lavie_data);
				if((osKernelGetTickCount() - timeBaseCheckSeizre > 120000)) //2minutes
				{
					lavie_data_combine.hr_rr.seizure = lavie_data.hr_rr.seizure;
				}
				HrRr2Display(INSTANT_DATA, &lavie_data_combine);
				lavie_data_combine.hr_rr.seizure = 0;
				if(LcdCtrlIsSleeping() && lavie_data.hr_rr.numMov5s && (!BedManagerPnpGet()))
				{
					HrRr2Display(HAVE_MOV_DATA, &lavie_data_combine);
				}
				if(!BedManagerCapaGet() && !(lavie_data_combine.bed_occupied) && lavie_data.hr_rr.numMov5s && SettingGetModeOperation().mode == OPE_MODE_MANUAL)
				{
					BedManagerPredictUserNormalModeSet(true);
				}
			}

			BedManagerCapaSet(data_rev.occupancy);

			if(BedManagerPnpGet() || (SettingGetModeOperation().mode == OPE_MODE_MANUAL))
			{
				if(SettingGetModeOperation().mode == OPE_MODE_AUTO)
				{
						lavie_data_combine.bed_occupied = true;
				}
#ifdef PAD_3_SENSOR
				if((osKernelGetTickCount() - timeBaseCheckBodyTemp) > timeCheckBodyTemp)
				{
					timeBaseCheckBodyTemp = osKernelGetTickCount();
					timeCheckBodyTemp = TIME_3MINUTES;
					if(data_rev.bodyTemp >= BODY_TEMP_MIN && data_rev.bodyTemp < BODY_TEMP_MAX){
						lavie_data_combine.body_temp = data_rev.bodyTemp + 1;

						if(preBodyTemp == 0){
							lavie_data_combine.body_temp_var = 0;
						}
						else{
							lavie_data_combine.body_temp_var = lavie_data_combine.body_temp - preBodyTemp;
						}
						preBodyTemp = lavie_data_combine.body_temp;
					}
				}
#endif
				resplife_data.x1[resplife_index] = (float)data_rev.adc1_x;
				resplife_data.y1[resplife_index] = (float)data_rev.adc1_y;
#ifdef PAD_3_SENSOR
				resplife_data.x2[resplife_index] = (float)data_rev.adc2_x;
				resplife_data.y2[resplife_index] = (float)data_rev.adc2_y;
				resplife_data.x3[resplife_index] = (float)data_rev.adc3_x;
				resplife_data.y3[resplife_index] = (float)data_rev.adc3_y;
#endif
				resplife_index++;
				resp_data.x1[resp_index] = (float)data_rev.adc1_x;
				resp_data.y1[resp_index] = (float)data_rev.adc1_y;
#ifdef PAD_3_SENSOR
				resp_data.x2[resp_index] = (float)data_rev.adc2_x;
				resp_data.y2[resp_index] = (float)data_rev.adc2_y;
				resp_data.x3[resp_index] = (float)data_rev.adc3_x;
				resp_data.y3[resp_index] = (float)data_rev.adc3_y;
#endif
				resp_index++;

				actual_data.x1[actual_index] = (float)data_rev.adc1_x;
				actual_data.y1[actual_index] = (float)data_rev.adc1_y;
				actual_data.z1[actual_index] = (float)data_rev.adc1_z;
#ifdef PAD_3_SENSOR
				actual_data.x2[actual_index] = (float)data_rev.adc2_x;
				actual_data.y2[actual_index] = (float)data_rev.adc2_y;
				actual_data.z2[actual_index] = (float)data_rev.adc2_z;
				actual_data.x3[actual_index] = (float)data_rev.adc3_x;
				actual_data.y3[actual_index] = (float)data_rev.adc3_y;
				actual_data.z3[actual_index] = (float)data_rev.adc3_z;
#endif
				actual_index++;

				if(resplife_index >= RESP_LIFE_RAW_DATA_LEN)
				{
					resplife_index = 0;
					if(lavie_data_combine.bed_occupied == true){
						resp_status_t resp_status = lifeline_resp(&resplife_data, RESP_LIFE_RAW_DATA_LEN);
						lavie_data_combine.hr_rr.resp_status = resp_status;
						HrRr2Display(RESP_LIFE_DATA, &lavie_data_combine);
						if(lavie_data_combine.hr_rr.resp_status != RESP_NORMAL)
						{
							lavie_data_combine.hr_rr.resp_status = RESP_NORMAL;
						}
					}
				}
				if(actual_index >= ACTUAL_DATA_LEN) // enough 40 seconds -> calculate heart rate and respiration rate
				{
					actual_index = 0;
					memset(&lavie_data, 0, sizeof(lavie_data));
					hr_rr(&actual_data, &lavie_data, lavie_data_combine.hr_rr.actual_hr);
					lavie_data_combine.hr_rr.pnp_rt = lavie_data.hr_rr.pnp_rt;
					lavie_data_combine.hr_rr.pnp_bg = get_pnp_bg();
					lavie_data_combine.hr_rr.motions = lavie_data.hr_rr.motions;
					lavie_data_combine.hr_rr.Th_bg = lavie_data.hr_rr.Th_bg;
					lavie_data_combine.hr_rr.Th_rt = lavie_data.hr_rr.Th_rt;
					lavie_data_combine.room_hum = RoomTH.hum;
					lavie_data_combine.room_temp = RoomTH.temp;
					lavie_data_combine.hr_rr.apnea = getCountApnea40s();
					lavie_data_combine.hr_rr.movEnergy = lavie_data.hr_rr.movEnergy;
					lavie_data_combine.hr_rr.maxPeak = lavie_data.hr_rr.maxPeak;
					if(lavie_data_combine.hr_rr.motions != 0 && timeCheckBodyTemp == TIME_3MINUTES){
						timeBaseCheckBodyTemp = osKernelGetTickCount();
					}
					if(lavie_data.hr_rr.actual_hr == PREVIOUS_CASE || lavie_data.hr_rr.actual_hr == WEAK_CASE
							|| lavie_data.hr_rr.actual_hr == ERROR_CASE)
					{
						if(lavie_data.hr_rr.actual_hr != PREVIOUS_CASE)
						{
							if(holdHr >= 3){
								lavie_data_combine.hr_rr.actual_hr = BLANK_CASE;
								lavie_data_combine.hr_rr.motions = 0;
								lavie_data_combine.hr_rr.instant_hr = BLANK_CASE;
								lavie_data_combine.hr_rr.rr = BLANK_CASE;
								lavie_data_combine.body_temp = BODY_TEMP_BLANK_CASE;
								lavie_data_combine.body_temp_var = BODY_TEMP_BLANK_CASE;
								lavie_data_combine.hr_rr.snoring_noisy = 0;
								lavie_data_combine.hr_rr.snoring = SNORING_NONE;
								lavie_data_combine.bed_occupied = false;
								lavieOfflineData.reset_index_offline();
								HrRr2Display(EMPTY_DATA, &lavie_data_combine);
								publish_hrrr_data(lavie_data_combine);
							}
							else{
								holdHr++;
								lavie_data_combine.hr_rr.motions = 0;
								HrRr2Display(HEART_RATE_DATA, &lavie_data_combine);
								HrRr2Display(RESP_RATE_DATA, &lavie_data_combine);
								publish_hr_data(lavie_data_combine);
							}
							LOG_WARN((char*)"holdHr : %d",holdHr);
						}
						else{
							LOG_WARN((char*)"Reset hold Hr");
							holdHr = 0;
							HrRr2Display(HEART_RATE_DATA, &lavie_data_combine);
							//HrRr2Display(RESP_RATE_DATA, &lavie_data_combine);
							publish_hr_data(lavie_data_combine);
						}
					}
					else
					{
						holdHr = 0;
						if((BedManagerPnpGet() == true) || (lavie_data_combine.hr_rr.actual_hr == BLANK_CASE && BedManagerPredictUserNormalModeGet())){
							LOG_WARN((char*)"Show HrRr: FastShowing = %d", BedManagerPredictUserNormalModeGet());
							BedManagerPredictUserNormalModeSet(false);
							lavie_data_combine.bed_occupied = true;
							lavie_data_combine.hr_rr.actual_hr = lavie_data.hr_rr.actual_hr;
							lavie_data_combine.hr_rr.hrv = lavie_data.hr_rr.hrv;
							HrRr2Display(HEART_RATE_DATA, &lavie_data_combine);
							publish_hr_data(lavie_data_combine);
							lavieOfflineData.set_offline_data(lavie_data_combine);
						}
						else{
							LOG_WARN((char*)"BedManagerPnpGet() == false");
							HrRr2Display(HEART_RATE_DATA, &lavie_data_combine);
							//HrRr2Display(RESP_RATE_DATA, &lavie_data_combine);
							publish_hr_data(lavie_data_combine);
						}
					}
				}
				if(resp_index >= RESP_RAW_DATA_LEN) // enough RESP_RAW_DATA_LEN samples -> find respiration
				{
					resp_index = 0;
					memset(&lavie_data, 0, sizeof(lavie_data));
					resp_detect(&resp_data, RESP_RAW_DATA_LEN, &lavie_data.hr_rr.rr, lavie_data_combine.hr_rr.rr);
					if(lavie_data.hr_rr.rr == PREVIOUS_CASE || lavie_data.hr_rr.rr == WEAK_CASE
							|| lavie_data.hr_rr.rr == ERROR_CASE)
					{
					}
					else if(lavie_data_combine.bed_occupied == true)
					{
						lavie_data_combine.hr_rr.rr = lavie_data.hr_rr.rr;
						lavie_data_combine.hr_rr.motions = INVALID_VALUE;
						lavie_data_combine.hr_rr.apnea = INVALID_VALUE;
						HrRr2Display(RESP_RATE_DATA, &lavie_data_combine);
						publish_rr_data(lavie_data_combine);
					}
				}
				if(SnoringIsReady() == true){
					lavie_data_combine.hr_rr.snoring_noisy = 0;
					lavie_data_combine.hr_rr.snoring = SnoringDetect(&lavie_data_combine.hr_rr.snoring_noisy);
					if(lavie_data_combine.bed_occupied == true)
					{
						HrRr2Display(SNORING_DATA, &lavie_data_combine);
					}
				}
			}
			else{
				SendParamsEmpty();
			}
		}
	}
}

static void SendParamsEmpty()
{
	//static lavie_data_t lavie_data;
	static uint32_t timeBase = 0;
	if((osKernelGetTickCount() - timeBase) >= 300000) // 5minutes
	{
		timeBase = osKernelGetTickCount();
//		lavie_data.hr_rr.actual_hr = BLANK_CASE;
//		lavie_data.hr_rr.instant_hr = BLANK_CASE;
//		lavie_data.hr_rr.rr = BLANK_CASE;
//		lavie_data.body_temp = BODY_TEMP_BLANK_CASE;
//		lavie_data.body_temp_var = BODY_TEMP_BLANK_CASE;
//		lavie_data.hr_rr.snoring = SNORING_NONE;
//		lavie_data.hr_rr.snoring_noisy = 0;
//		lavie_data.hr_rr.apnea = 0;
//		publish_hrrr_data(lavie_data);
	}
}

void EmptyProcess(bool resetAllFlag)
{
	LOG_WARN((char *)"EmptyProcess %d", resetAllFlag);
	//turn on display
	if(resetAllFlag == true)
	{
		if(LcdCtrlIsSleeping() == true){
			LcdCtrlSleepOutDisplay();
			osDelay(150);
		}

		SnoringStopGetAdc();
		lavieOfflineData.reset_index_offline();
		reset_pnp_bg();
		osMessageQueueReset(Master_QHandle);
		osMessageQueueReset(HrRr2User_QHandle);
		reset_lifeline_resp();
		resetApnea();
		check_valid_respRate(0, true);
		actual_index = 0;
		instant_index = 0;
		resp_index = 0;
		pnp_bg_index = 0;
		resplife_index = 0;
		preBodyTemp = 0;
		SeizureReset();
	}
	memset(&lavie_data_combine, 0, sizeof(lavie_data_combine));
	lavie_data_combine.hr_rr.actual_hr = BLANK_CASE;
	lavie_data_combine.hr_rr.motions = 0;
	lavie_data_combine.hr_rr.instant_hr = BLANK_CASE;
	lavie_data_combine.hr_rr.rr = BLANK_CASE;
	lavie_data_combine.body_temp = BODY_TEMP_BLANK_CASE;
	lavie_data_combine.body_temp_var = BODY_TEMP_BLANK_CASE;
	lavie_data_combine.hr_rr.snoring_noisy = 0;
	lavie_data_combine.hr_rr.snoring = SNORING_NONE;
	HrRr2Display(EMPTY_DATA, &lavie_data_combine);
	publish_hrrr_data(lavie_data_combine);
}

void EmptyBedUpdate(bool val)
{
	if(val == false){
		lavie_data_combine.body_temp = BODY_TEMP_BLANK_CASE;
		lavie_data_combine.body_temp_var = BODY_TEMP_BLANK_CASE;
	}
	publish_hrrr_data(lavie_data_combine);
}


void OccupiedProcess(bool startFlag)
{
	LOG_WARN((char *)"OccupiedProcess  : %d", startFlag);
	//turn on display
	if(LcdCtrlIsSleeping() == true){
		LcdCtrlSleepOutDisplay();
		osDelay(150);
	}
	SnoringStartGetAdc();
	reset_pnp_bg();
#ifdef PAD_3_SENSOR
	lavie_data_combine.body_temp = 0;
#else
	lavie_data_combine.body_temp = BODY_TEMP_BLANK_CASE;
#endif
	lavie_data_combine.body_temp_var = 0;
	preBodyTemp = 0;
	lavie_data_combine.hr_rr.actual_hr = BLANK_CASE;
	lavie_data_combine.hr_rr.instant_hr = BLANK_CASE;
	lavie_data_combine.hr_rr.motions = 0;
	lavie_data_combine.hr_rr.rr = BLANK_CASE;
	lavie_data_combine.body_temp = BODY_TEMP_BLANK_CASE;
	lavie_data_combine.body_temp_var = BODY_TEMP_BLANK_CASE;
	lavie_data_combine.hr_rr.snoring_noisy = 0;
	lavie_data_combine.hr_rr.snoring = SNORING_NONE;
	if(startFlag){
		lavie_data_combine.bed_occupied = true;
	}
	else{
		lavie_data_combine.bed_occupied = false;
	}
	timeCheckBodyTemp = TIME_5MINUTES;
	timeBaseCheckBodyTemp = osKernelGetTickCount();
	timeBaseCheckSeizre = timeBaseCheckBodyTemp;
}
