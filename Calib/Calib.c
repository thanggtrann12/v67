/*
 * Calib.c
 *
 *  Created on: Nov 10, 2021
 *      Author: Thien Phan
 */
#include "Calib.h"
#include "Rtos.h"
#include "dsp.h"
#include "hr_rr.h"
#include "lifesos.h"
#include "serial_master.h"
#include "OslavieMqttHandle.h"
#include "LcdUpdate.h"
#include "MacAddress.h"
#include "settings.h"

#define TIME_WAIT_CALIB (uint16_t)60

CalibStatus_t CalibStatus = CALIB_NONE;
extern actual_data_t  actual_data;
extern float32_t noise_dft;

bool calib_person(calib_params_t *calib_params);
bool calib_empty(calib_params_t *calib_params);
void find_data_noise(actual_data_t* actual_data, calib_params_t *calib_params);
void find_person_params(actual_data_t* actual_data, calib_params_t *calib_params);

void CalibPrepare(){
	osMutexAcquire(MasterMutexHandle, osWaitForever);
		osThreadSuspend(dataMasterTaskHandle);
		osThreadSuspend(hrRrTaskHandle);
	osMutexRelease(MasterMutexHandle);
	master_stop_sampling();
	osMessageQueueReset(Master_QHandle);
	EmptyProcess(true);
}

void CalibExit(){
	osMessageQueueReset(Master_QHandle);
	osThreadResume(dataMasterTaskHandle);
	osThreadResume(hrRrTaskHandle);
}

bool CalibEmptyProcess(void)
{
	LOG_DEBUGF((char*)"-----------Start calib empty---------");
	CalibPrepare();
	// keep display on in calibration process
	CalibStatus = CALIB_NONE;
	bool successFlag = false;
	calib_params_t calib_params = {0};

	CalibStatus = CALIB_WAIT_EMPTY;
	response_calibration_status_message(); // send message to the app
	LcdUpdateTextCalibrationPopUp(msgCalibEbWait);

	if(calib_empty(&calib_params) == true)
	{
		SettingSaveCalibEmptyParams(calib_params);
		if(SettingGetCalibStatus() == SETTING_CALIB_FULL || SettingGetCalibStatus() == SETTING_CALIB_ONLY_EMPTY){
			LcdUpdateMsgHome(msgCalibDone);
			CalibStatus = CALIB_EMPTY_DONE;
			response_calibration_status_message(); // send message to the app
			buzzerNotySuccess(); //turn on the notification bell
			osDelay(2000);
			successFlag = true;
		}
		else{
			CalibStatus = CALIB_ERROR_HAVE_MOVEMENTS;
			response_calibration_status_message(); // send message to the app
			LcdUpdateMsgError(msgCalibErr);
			buzzerNotyFail();
		}
	}
	else{
		response_calibration_status_message(); // send message to the app
		LcdUpdateMsgError(msgCalibErr);
		buzzerNotyFail();
	}
	CalibStatus = CALIB_NONE;
	CalibExit();
	return successFlag;
}

bool CalibPersonProcess(void)
{
	LOG_DEBUGF((char*)"-----------Start calib person---------");
	CalibPrepare();
	// keep display on in calibration process
	CalibStatus = CALIB_NONE;
	bool successFlag = false;
	calib_params_t calib_params = {0};

	CalibStatus = CALIB_WAIT_PERSON;
	response_calibration_status_message(); // send message to the app
	LcdUpdateTextCalibrationPopUp(msgCalibUserWait);

	if(calib_person(&calib_params) == true)
	{
		SettingSaveCalibPersonParams(calib_params);
		if(SettingGetCalibStatus() == SETTING_CALIB_FULL || SettingGetCalibStatus() == SETTING_CALIB_ONLY_PERSON){
			LcdUpdateMsgHome(msgCalibDone);
			CalibStatus = CALIB_PERSON_DONE;
			response_calibration_status_message(); // send message to the app
			buzzerNotySuccess(); //turn on the notification bell
			osDelay(2000);
			successFlag = true;
		}
		else{
			CalibStatus = CALIB_ERROR_HAVE_MOVEMENTS;
			response_calibration_status_message(); // send message to the app
			LcdUpdateMsgError(msgCalibErr);
			buzzerNotyFail();
		}
	}
	else{
		response_calibration_status_message(); // send message to the app
		LcdUpdateMsgHome(msgCalibDone);
		buzzerNotyFail();
	}
	CalibStatus = CALIB_NONE;
	CalibExit();
	return successFlag;
}

bool calib_empty(calib_params_t *calib_params){
	bool sucessFlag = false;
	// start calib empty when the bed is empty for 20 seconds continuously, break if the bed isnot empty in 60s
	data_master_node data_rev;
	uint16_t count_prepare = 0, time_prepare = 20; //seconds
//#ifdef PAD_3_SENSOR
//	uint16_t timeout = 60, count_timeout = 0;
//	while(1)
//	{
//		if(check_master_occupied() == false)
//		{
//			// the bed is empty
//			count_timeout = 0;
//			count_prepare++;
//			LOG_DEBUGF((char*)"bed_empty %d", count_prepare);
//			if(count_prepare >= time_prepare)
//			{
//				break;
//			}
//		}
//		else{ // the bed is not empty
//			count_prepare = 0;
//		}
//		count_timeout++;
//		if(count_timeout >= timeout)
//		{
//			break;
//		}
//		osDelay(1000);
//	}
//#else
	time_prepare = 30;
	while(1)
	{
		count_prepare++;
		if(count_prepare >= time_prepare)
		{
			break;
		}
		osDelay(1000);
	}
//#endif
	if(count_prepare < time_prepare)
	{
		LOG_DEBUGF((char*)"CALIB_ERROR_BED_NOT_EMPTY");
		CalibStatus = CALIB_ERROR_BED_NOT_EMPTY;
		response_calibration_status_message(); // send message to the app
		return sucessFlag; // return failed
	}
	else
	{
		LOG_DEBUGF((char*)"Start calib empty bed, Get data empty");
		osMessageQueueReset(Master_QHandle);
		CalibStatus = CALIB_EMPTY;
		response_calibration_status_message(); // send message to the app
		LcdUpdateTextCalibrationPopUp(msgCalibEb);

		if(master_start_sampling() == true)
		{
			for(uint32_t i = 0; i < ACTUAL_DATA_LEN;)
			{
				if(request_hr_data() == true)
				{
					if (osMessageQueueGet(Master_QHandle, &data_rev, NULL, 3000) == osOK)
					{
						if(data_rev.occupancy == true)
						{
							LOG_DEBUGF((char*)"the bed is not empty");
							CalibStatus = CALIB_ERROR_BED_NOT_EMPTY;
							break;
						}
						actual_data.x1[i] = (float)data_rev.adc1_x;
						actual_data.y1[i] = (float)data_rev.adc1_y;
						actual_data.z1[i] = (float)data_rev.adc1_z;
#ifdef PAD_3_SENSOR
						actual_data.x2[i] = (float)data_rev.adc2_x;
						actual_data.y2[i] = (float)data_rev.adc2_y;
						actual_data.z2[i] = (float)data_rev.adc2_z;
						actual_data.x3[i] = (float)data_rev.adc3_x;
						actual_data.y3[i] = (float)data_rev.adc3_y;
						actual_data.z3[i] = (float)data_rev.adc3_z;
#endif
						i++;
					}
					else
					{
						CalibStatus = CALIB_ERROR_NO_MASTER;
						break;
					}
				}
				else
				{
					LOG_DEBUGF((char*)"request_calib_data() error or timeout");
					CalibStatus = CALIB_ERROR_NO_MASTER;
					break;
				}
			}
		}
		else{
			LOG_DEBUGF((char*)"master_start_sampling() error or timeout");
			CalibStatus = CALIB_ERROR_NO_MASTER;
		}
		master_stop_sampling();
		if(CalibStatus == CALIB_EMPTY)
		{
			// find calib params
			find_data_noise(&actual_data, calib_params);
			float eb_th_resp_freq = 0, eb_th_resp_time= 0;
			find_resp_noise(&actual_data, RESP_RAW_DATA_LEN, &eb_th_resp_freq, &eb_th_resp_time);
			calib_params->eb_th_resp_freq = eb_th_resp_freq;
			calib_params->eb_th_resp_time = eb_th_resp_time;

			sucessFlag = true;
		}
		else{
			response_calibration_status_message();
		}
	}
	return sucessFlag;
}


bool calib_person(calib_params_t *calib_params){
	bool sucessFlag = false;
	// start calib person when the bed is occupied for 20 seconds continuously, break if the bed is empty in 60s
	data_master_node data_rev;
	uint16_t count_prepare = 0, time_prepare = 20; //seconds
//#ifdef PAD_3_SENSOR
//	uint16_t timeout = 60, count_timeout = 0;
//	while(1)
//	{
//		if(check_master_occupied() == true)
//		{
//			// the bed is empty
//			count_timeout = 0;
//			count_prepare++;
//			LOG_DEBUGF((char*)"bed_occupied %d", count_prepare);
//			if(count_prepare >= time_prepare)
//			{
//				break;
//			}
//		}
//		else{ // the bed is not empty
//			count_prepare = 0;
//		}
//		count_timeout++;
//		if(count_timeout >= timeout)
//		{
//			break;
//		}
//		osDelay(1000);
//	}
//#else
	time_prepare = 30;
	while(1)
	{
		count_prepare++;
		if(count_prepare >= time_prepare)
		{
			break;
		}
		osDelay(1000);
	}
//#endif
	if(count_prepare < time_prepare)
	{
		LOG_DEBUGF((char*)"CALIB_ERROR_NO_PERSON");
		CalibStatus = CALIB_ERROR_NO_PERSON;
		response_calibration_status_message(); // send message to the app
		return sucessFlag; // return failed
	}
	else
	{
		LOG_DEBUGF((char*)"Get data calib person");
		CalibStatus = CALIB_PERSON;
		response_calibration_status_message(); // send message to the app
		LcdUpdateTextCalibrationPopUp(msgCaliUser);

		if(master_start_sampling() == true)
		{
			for(uint32_t i = 0; i < ACTUAL_DATA_LEN;)
			{
				if(request_hr_data() == true)
				{
					if (osMessageQueueGet(Master_QHandle, &data_rev, NULL, 3000) == osOK)
					{
//#ifdef PAD_3_SENSOR
//						if(data_rev.occupancy == false)
//						{
//							LOG_DEBUGF((char*)"Data have no person");
//							CalibStatus = CALIB_ERROR_NO_PERSON;
//							break;
//						}
//#endif
						actual_data.x1[i] = (float)data_rev.adc1_x;
						actual_data.y1[i] = (float)data_rev.adc1_y;
						actual_data.z1[i] = (float)data_rev.adc1_z;
#ifdef PAD_3_SENSOR
						actual_data.x2[i] = (float)data_rev.adc2_x;
						actual_data.y2[i] = (float)data_rev.adc2_y;
						actual_data.z2[i] = (float)data_rev.adc2_z;
						actual_data.x3[i] = (float)data_rev.adc3_x;
						actual_data.y3[i] = (float)data_rev.adc3_y;
						actual_data.z3[i] = (float)data_rev.adc3_z;
#endif
						i++;
					}
					else
					{
						LOG_DEBUGF((char*)"Timeout get Calib_QHandle");
						CalibStatus = CALIB_ERROR_NO_MASTER;
						break;
					}
				}
				else
				{
					LOG_DEBUGF((char*)"request_calib_data() error or timeout");
					CalibStatus = CALIB_ERROR_NO_MASTER;
					break;
				}
			}
			master_stop_sampling();
		}
		else{
			LOG_DEBUGF((char*)"master_start_sampling() error or timeout");
			CalibStatus = CALIB_ERROR_NO_MASTER;
		}
	}
	master_stop_sampling();
	if(CalibStatus != CALIB_PERSON){
		response_calibration_status_message(); // send message to the app
	}
	else
	{
		find_person_params(&actual_data, calib_params);
		find_thresh_mov_resp(&actual_data, RESP_RAW_DATA_LEN, &calib_params->avg_energy_resp);
		LOG_DEBUGF((char*)"avg_energy_resp %f", calib_params->avg_energy_resp);
		LOG_DEBUGF((char*)"calib person finished");

		sucessFlag = true;
	}
	return sucessFlag;
}

void find_data_noise(actual_data_t* actual_data, calib_params_t *calib_params)
{
	actual_data_t *sq_data = NULL;  // for filter results and square results
	sq_data = (actual_data_t *)pvPortMalloc(sizeof(actual_data_t));
	if(sq_data == NULL)
	{
		LOG_DEBUGF((char*)"Could not allocate memory :sq_data");
		Error_Handler();
	}

	float32_t *avg_sq = NULL;
	avg_sq = (float32_t *)pvPortMalloc(ACTUAL_DATA_LEN*sizeof(float32_t));
	if(avg_sq == NULL)
	{
		LOG_DEBUGF((char*)"Could not allocate memory :avg_sq");
		Error_Handler();
	}


	filter_data(BPF_0P7_20, actual_data->x1, sq_data->x1, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->y1, sq_data->y1, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->z1, sq_data->z1, ACTUAL_DATA_LEN);
#ifdef PAD_3_SENSOR
	filter_data(BPF_0P7_20, actual_data->x2, sq_data->x2, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->y2, sq_data->y2, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->z2, sq_data->z2, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->x3, sq_data->x3, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->y3, sq_data->y3, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->z3, sq_data->z3, ACTUAL_DATA_LEN);
#endif

	for(uint16_t i = 0; i < ACTUAL_DATA_LEN; i++)
	{
#ifdef PAD_3_SENSOR
		avg_sq[i] = (sq_data->x1[i]*sq_data->x1[i] + sq_data->x2[i]*sq_data->x2[i] + sq_data->x3[i]*sq_data->x3[i])/3;
#else
		avg_sq[i] = sq_data->x1[i]*sq_data->x1[i];
#endif
	}
	float avgsq_noise_x_temp = signoise_heart(avg_sq, ACTUAL_DATA_LEN);
	for(uint16_t i = 0; i < ACTUAL_DATA_LEN; i++)
	{
#ifdef PAD_3_SENSOR
		avg_sq[i] = (sq_data->y1[i]*sq_data->y1[i] + sq_data->y2[i]*sq_data->y2[i] + sq_data->y3[i]*sq_data->y3[i])/3;
#else
		avg_sq[i] = sq_data->y1[i]*sq_data->y1[i];
#endif
	}
	float avgsq_noise_y_temp = signoise_heart(avg_sq, ACTUAL_DATA_LEN);
	for(uint16_t i = 0; i < ACTUAL_DATA_LEN; i++)
	{
#ifdef PAD_3_SENSOR
		avg_sq[i] = (sq_data->z1[i]*sq_data->z1[i] + sq_data->z2[i]*sq_data->z2[i] + sq_data->z3[i]*sq_data->z3[i])/3;
#else
		avg_sq[i] = sq_data->z1[i]*sq_data->z1[i];
#endif
	}
	float avgsq_noise_z_temp = signoise_heart(avg_sq, ACTUAL_DATA_LEN);
	for(uint16_t i = 0; i < ACTUAL_DATA_LEN; i++)
	{
#ifdef PAD_3_SENSOR
		avg_sq[i] = sq_data->x1[i]*sq_data->x1[i] + sq_data->x2[i]*sq_data->x2[i] + sq_data->x3[i]*sq_data->x3[i]
				+ sq_data->y1[i]*sq_data->y1[i] + sq_data->y2[i]*sq_data->y2[i] + sq_data->y3[i]*sq_data->y3[i]
				+ sq_data->z1[i]*sq_data->z1[i] + sq_data->z2[i]*sq_data->z2[i] + sq_data->z3[i]*sq_data->z3[i];
#else
		avg_sq[i] = sq_data->x1[i]*sq_data->x1[i] + sq_data->y1[i]*sq_data->y1[i] + sq_data->z1[i]*sq_data->z1[i];
#endif
	}
	float avgsq_noise_xyz_temp = signoise_heart(avg_sq, ACTUAL_DATA_LEN);

	powernoise_heart(avg_sq, ACTUAL_DATA_LEN, HEART_WINDOW, &noise_dft);
	LOG_DEBUGF((char*)"avgsq_noise_x_temp %f, avgsq_noise_y_temp %f, avgsq_noise_z_temp %f, avgsq_noise_xyz_temp %f, noise_dft %f",
			avgsq_noise_x_temp, avgsq_noise_y_temp, avgsq_noise_z_temp, avgsq_noise_xyz_temp, noise_dft);

	calib_params->avgsq_noise_x = avgsq_noise_x_temp;
	calib_params->avgsq_noise_y = avgsq_noise_y_temp;
	calib_params->avgsq_noise_z = avgsq_noise_z_temp;
	calib_params->avgsq_noise_xyz = avgsq_noise_xyz_temp;

	// free memory
	vPortFree(sq_data); // free pointer sq_data
	vPortFree(avg_sq); // free pointer avg_sq
}

void find_person_params(actual_data_t* actual_data, calib_params_t *calib_params)
{
	float *sum_data_xy = NULL;  // for filter results and square results
	sum_data_xy = (float *)pvPortMalloc(ACTUAL_DATA_LEN*sizeof(float));
	if(sum_data_xy == NULL)
	{
		LOG_DEBUGF((char*)"Could not allocate memory :sum_data_xy");
		Error_Handler();
	}

	float *sum_data_xyz = NULL;  // for filter results and square results
	sum_data_xyz = (float *)pvPortMalloc(ACTUAL_DATA_LEN*sizeof(float));
	if(sum_data_xyz == NULL)
	{
		LOG_DEBUGF((char*)"Could not allocate memory :sum_data_xyz");
		Error_Handler();
	}

	actual_data_t *actual_data_bpf = NULL;
	actual_data_bpf = (actual_data_t *)pvPortMalloc(sizeof(actual_data_t));
	if(actual_data_bpf == NULL)
	{
		LOG_DEBUGF((char*)"Could not allocate memory :actual_data_bpf");
		Error_Handler();
	}

	filter_data(BPF_0P7_20, actual_data->x1, actual_data_bpf->x1, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->y1, actual_data_bpf->y1, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->z1, actual_data_bpf->z1, ACTUAL_DATA_LEN);
#ifdef PAD_3_SENSOR
	filter_data(BPF_0P7_20, actual_data->x2, actual_data_bpf->x2, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->y2, actual_data_bpf->y2, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->z2, actual_data_bpf->z2, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->x3, actual_data_bpf->x3, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->y3, actual_data_bpf->y3, ACTUAL_DATA_LEN);
	filter_data(BPF_0P7_20, actual_data->z3, actual_data_bpf->z3, ACTUAL_DATA_LEN);
#endif

	for(uint16_t i = 0; i < ACTUAL_DATA_LEN; i++)
	{
#ifdef PAD_3_SENSOR
		sum_data_xy[i] = actual_data_bpf->x1[i] + actual_data_bpf->x2[i] + actual_data_bpf->x3[i]
					+ actual_data_bpf->y1[i] + actual_data_bpf->y2[i] + actual_data_bpf->y3[i];

		sum_data_xyz[i] = actual_data_bpf->x1[i] + actual_data_bpf->x2[i] + actual_data_bpf->x3[i]
					+ actual_data_bpf->y1[i] + actual_data_bpf->y2[i] + actual_data_bpf->y3[i]
					+ actual_data_bpf->z1[i] + actual_data_bpf->z2[i] + actual_data_bpf->z3[i];
#else
		sum_data_xy[i] = actual_data_bpf->x1[i] + actual_data_bpf->y1[i];
		sum_data_xyz[i] = actual_data_bpf->x1[i] + actual_data_bpf->y1[i] + actual_data_bpf->z1[i];
#endif
	}

	float max_energy = 10e20;
	movements_t movements[MAX_MOV_40s_NUM] = {0};
	float32_t avg_energy_out_xy = 0;   //this value will need to be adjusted later
	uint8_t num_moves = search_movements(sum_data_xy, ACTUAL_DATA_LEN, max_energy, movements, &avg_energy_out_xy, NULL, NULL);

	float32_t avg_energy_out_xyz = 0;   //this value will need to be adjusted later
	num_moves = search_movements(sum_data_xyz, ACTUAL_DATA_LEN, max_energy, movements, &avg_energy_out_xyz, NULL, NULL);
	LOG_DEBUGF((char*)"num_moves %d, avg_energy_out_xy %f, avg_energy_out_xyz %f", num_moves, avg_energy_out_xy, avg_energy_out_xyz);

	calib_params->avg_energy_xy = avg_energy_out_xy;
	calib_params->avg_energy_xyz = avg_energy_out_xyz;

	vPortFree(sum_data_xy);
	vPortFree(sum_data_xyz);
	vPortFree(actual_data_bpf);
}

void disable_receive_master(void)
{
	HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
}

void enable_receive_master(void)
{
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

void response_calibration_status_message(void)
{
	message_publish_t data_publish;
	data_publish.retain = 0;
	data_publish.TopicID = CALIBRATION_RESPONSE_TOPIC_ID;
	memset(data_publish.payload,0,OSLAVIE_PUBLISH_LEN);
	if(CalibStatus == CALIB_NONE)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\"}", MacAddressGet(), "None");
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
	else if(CalibStatus == CALIB_EMPTY)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\",\"time\":\"%u\"}", MacAddressGet(), "CalibEmpty", TIME_WAIT_CALIB);
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
	else if(CalibStatus == CALIB_PERSON)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\",\"time\":\"%u\"}", MacAddressGet(), "CalibPerson", TIME_WAIT_CALIB);
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
	else if(CalibStatus == CALIB_WAIT_EMPTY)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\"}", MacAddressGet(), "CalibWaitEmpty");
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
	else if(CalibStatus == CALIB_WAIT_PERSON)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\"}", MacAddressGet(), "CalibWaitPerson");
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
	else if(CalibStatus == CALIB_PERSON_DONE)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\"}", MacAddressGet(), "CalibPerson_done");
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
	else if(CalibStatus == CALIB_EMPTY_DONE)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\"}", MacAddressGet(), "CalibEmpty_done");
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
	else if(CalibStatus == CALIB_ERROR_BED_NOT_EMPTY)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\"}", MacAddressGet(), "BedNotEmpty");
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
	else if(CalibStatus == CALIB_ERROR_HAVE_MOVEMENTS)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\"}", MacAddressGet(), "HaveMovements");
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
	else if(CalibStatus == CALIB_ERROR_NO_PERSON)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\"}", MacAddressGet(), "NoPerson");
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
	else if(CalibStatus == CALIB_ERROR_NO_MASTER)
	{
		sprintf(data_publish.payload, "{\"dev_id\":\"%s\",\"mode\":\"%s\"}", MacAddressGet(), "MasterNotDetected");
		osMessageQueuePut(Hub2Broker_QHandle, &data_publish, 0, 0);
	}
}



