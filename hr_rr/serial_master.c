/**
  ******************************************************************************
	* OnSky Inc. Confidential and Proprietary
  * @file    serial_master.c 
	
  * @author  Code written by Thien Phan - Embedded Developer
	
  * @brief   Communication beetween Hub and Master
  ******************************************************************************
  */
/* Data format beetween master and hub
		2 bytes      |  2 bytes |  1 bytes    |   N bytes  |   1  |
   SYNC data     |  length  |  Data ID    |    Data    | CRC  |
	 length = N + 6 bytes, CRC N + 5 bytes*/
#include <util/util.h>
#include "Utils.h"
#include "serial_master.h"
#include "rtc.h"
#include "master_process.h"
#include "hr_rr.h"
#include "settings.h"
#include "Config.h"
#include "Rtos.h"
#include <string.h>

#define BUFF_HM_SIZE (uint16_t)28
#define HEADER_HM_SIZE (uint16_t)2

static uint8_t HM_CRC_Cal8Bits(uint8_t *Buffer, uint16_t Size);
extern __IO FlagStatus UartHMTxReady;

static const uint8_t headerHM[HEADER_HM_SIZE] = {0xff, 0x55};
static __IO uint8_t dataHM[BUFF_HM_SIZE];
static uint8_t rxBufHM[BUFF_HM_SIZE] __attribute__((section(STR("_UartMasterBuf")))) __attribute__((aligned(32)));
static uint8_t txBufHM[BUFF_HM_SIZE] __attribute__((section(STR("_UartMasterBuf")))) __attribute__((aligned(32)));


void process_data_HM(void)
{
	SCB_InvalidateDCache_by_Addr((uint32_t*)rxBufHM, BUFF_HM_SIZE);
	for(uint8_t i = 0; i < (BUFF_HM_SIZE-1); i++)
	{
		if(rxBufHM[i] == headerHM[0] && rxBufHM[i+1] == headerHM[1])
		{
			/*SYNC between master and hub*/
			memcpy((uint8_t *)&dataHM, &rxBufHM[i], BUFF_HM_SIZE);
			osSemaphoreRelease(Semaphore_HMRx);
			break;
		}
	}
}
//void process_data_HM(void)
//{
//	SCB_InvalidateDCache_by_Addr((uint32_t*)rxBufHM, BUFF_HM_SIZE);
//	if(rxBufHM[0] == headerHM[0] && rxBufHM[1] == headerHM[1])
//	{
//		/*SYNC between master and hub*/
//		memcpy((uint8_t *)&dataHM, rxBufHM, BUFF_HM_SIZE);
//		osSemaphoreRelease(Semaphore_HMRx);
//	}
//}

HM_status_t get_HM_response(uint32_t timeout)
{
	HM_status_t HM_status = HmError;
	osStatus_t osStatus = osSemaphoreAcquire(Semaphore_HMRx, timeout);
	if(osStatus == osOK)
	{
		uint16_t length_payload = 0;
		memcpy((uint8_t *)&length_payload, (uint8_t *)&dataHM[HEADER_HM_SIZE], 2);
		//get data -> check CRC
		uint8_t HmCrc = HM_CRC_Cal8Bits((uint8_t *)dataHM, length_payload-1);
		if(HmCrc == dataHM[length_payload-1])
		{
			HM_status = HmOK;
		}
		else
		{
			HM_status = HmError;
			LOG_WARN("==========> Hm CRC incorrect!");
		}
	}
	else
	{
		HM_status = HmTimeout;
		LOG_DEBUGF("==========> HmTimeout");
	}	
//	SCB_InvalidateDCache_by_Addr((uint32_t*)rxBufHM, BUFF_HM_SIZE);
	return HM_status;
}

HM_status_t send_data_HM(uint8_t *data, uint16_t size, HM_CMD_t HM_CMD)
{
	osSemaphoreAcquire(Semaphore_HMRx, 0);

	HAL_UART_DMAStop(&huart6);
	SCB_CleanDCache_by_Addr((uint32_t *)txBufHM, BUFF_HM_SIZE);
	memset(rxBufHM,0,BUFF_HM_SIZE);
	if(HAL_UART_Receive_DMA(&huart6, (uint8_t *)&rxBufHM, BUFF_HM_SIZE) != HAL_OK)
	{
		//Error_Handler();
		LOG_DEBUG("HAL_UART_Receive_DMA Fail");
	}


	/*Reset there parameters for receive data HM*/
	memset((uint8_t *)dataHM, 0, BUFF_HM_SIZE);
	memset((uint8_t *)txBufHM, 0, BUFF_HM_SIZE);
	/**/
	uint16_t index = 0;
	uint16_t length_payload = HEADER_HM_SIZE + size + 4;  // length of payload
	if(length_payload > BUFF_HM_SIZE)
	{
		LOG_WARN("ERROR: exceeding the size of buf");
		return HmError;
	}
	
	memcpy((uint8_t *)txBufHM, (uint8_t *)headerHM, HEADER_HM_SIZE); // copy header to payload
	index += HEADER_HM_SIZE;
	memcpy((uint8_t *)&txBufHM[index], (uint8_t *)&length_payload, 2);	 // copy length to payload
	index += 2;
	txBufHM[index] = HM_CMD;  // copy HM_CMD to payload
	index += 1;
	memcpy((uint8_t *)&txBufHM[index], data, size);  // copy data to payload
	index += size;
	txBufHM[index] = HM_CRC_Cal8Bits((uint8_t *)txBufHM, index);
	index += 1;

	if(UartHMTxReady == RESET){
		if((osSemaphoreAcquire(Semaphore_HMTx, 200) != osOK)){
			LOG_WARN("TIMEOUT TX UART");
			UartHMTxReady = SET;
			return HmError;
		}
	}
	UartHMTxReady = RESET;		
	HAL_StatusTypeDef trasmit_stt = HAL_UART_Transmit_DMA(&huart6, (uint8_t *)txBufHM, BUFF_HM_SIZE);
	if(trasmit_stt != HAL_OK)
	{
		LOG_WARN("transmit master error: %d", trasmit_stt);
		UartHMTxReady = SET;
		return HmError;
	}
	return HmOK;
}
	

masterStatus_t check_master_occupied(void)
{
	masterStatus_t masterStatus = MasterTimeout;
	osMutexAcquire(MasterMutexHandle, osWaitForever);
	uint8_t data = 0;
	if(send_data_HM(&data, 1, CHECK_OCCUPIED_CMD) == HmOK)
	{
		if(get_HM_response(500) == HmOK)
		{
			//parse data
			if(dataHM[HEADER_HM_SIZE+2] == CHECK_OCCUPIED_CMD && dataHM[HEADER_HM_SIZE+3] == 1)
			{
				masterStatus = MasterOccupied;
			}
			else if(dataHM[HEADER_HM_SIZE+2] == CHECK_OCCUPIED_CMD && dataHM[HEADER_HM_SIZE+3] == 0)
			{
				LOG_DEBUGF("check occupied: Empty");
				masterStatus = MasterEmpty;
			}
		}else{
			LOG_WARN("check_master_occupied timeout");
			masterStatus = MasterTimeout;
		}
	}
	osMutexRelease(MasterMutexHandle);
	return masterStatus;
}

bool master_start_sampling(void)
{
	LOG_DEBUGF("master_start_sampling");
	bool started = false;
	osMutexAcquire(MasterMutexHandle, osWaitForever);
	uint8_t data = 0;
	if(send_data_HM(&data, 1, START_SAMPLING_CMD) == HmOK)
	{
		if(get_HM_response(1000) == HmOK)
		{
			//parse data
			if(dataHM[HEADER_HM_SIZE+2] == START_SAMPLING_CMD && dataHM[HEADER_HM_SIZE+3] == 1)
			{
				started = true;
			}
			else if(dataHM[HEADER_HM_SIZE+2] == START_SAMPLING_CMD && dataHM[HEADER_HM_SIZE+3] == 0)
			{
				
			}
		}else{
			LOG_WARN("master_start_sampling timeout");
		}
	}
	osMutexRelease(MasterMutexHandle);
	return started;
}

bool master_stop_sampling(void)
{
	LOG_DEBUGF("master_stop_sampling");
	bool stopped = false;
	osMutexAcquire(MasterMutexHandle, osWaitForever);
	uint8_t data = 0;
	if(send_data_HM(&data, 1, STOP_SAMPLING_CMD) == HmOK)
	{
		if(get_HM_response(500) == HmOK)
		{
			if(dataHM[HEADER_HM_SIZE+2] == STOP_SAMPLING_CMD && dataHM[HEADER_HM_SIZE+3] == 1)
			{
				stopped = true;
			}
			else if(dataHM[HEADER_HM_SIZE+2] == STOP_SAMPLING_CMD && dataHM[HEADER_HM_SIZE+3] == 0)
			{
			}
		}else{
			LOG_WARN("master_stop_sampling timeout");
		}
	}
	osMutexRelease(MasterMutexHandle);
	return stopped;
}

bool request_hr_data(void)
{
	data_master_node data_master_rx;
	bool stt = false;
	osMutexAcquire(MasterMutexHandle, osWaitForever);
	uint8_t data = 0;
	if(send_data_HM(&data, 1, HR_DATA_CMD) == HmOK)
	{
		if(get_HM_response(200) == HmOK)
		{
			if(dataHM[HEADER_HM_SIZE+2] == HR_DATA_CMD)
			{
				stt = true;
				memcpy((uint8_t *)&data_master_rx, (uint8_t *)&dataHM[HEADER_HM_SIZE + 3], sizeof(data_master_rx));
				if(osMessageQueuePut(Master_QHandle, &data_master_rx, 0, 0) != osOK)
				{
					LOG_WARN("\r\nFULL MEMORY!!!!!! %d\r\n", osMessageQueueGetSpace(Master_QHandle));
				}
			}
		}else{
			LOG_WARN("request_hr_data timeout");
		}
	}
	osMutexRelease(MasterMutexHandle);
	return stt;
}

static bool vibrateStatus = false;
bool vibrateIsOn(void){
	return vibrateStatus;
}

bool start_vibration(uint8_t time_vibrate)
{
	// duty cycle PWM vibration
	uint8_t duty = 0;
	vibrateVolume_t vibrateVolume = get_vibarate_setting();
	if(vibrateVolume == VIBRATE_DISABLE || vibrateStatus == true)
	{
		return false;
	}
	else if(vibrateVolume == VIBRATE_LOW){
		duty = 50;
	}
	else if(vibrateVolume == VIBRATE_MED){
		duty = 75;
	}
	else if(vibrateVolume == VIBRATE_HIGH){
		duty = 100;
	}
	bool stt = false;
	osMutexAcquire(MasterMutexHandle, osWaitForever);
	uint8_t data[2] = {duty, time_vibrate};	
	if(send_data_HM(data, 2, VIBRATE_EN_CMD) == HmOK)
	{
		if(get_HM_response(500) == HmOK)
		{
			//parse data
			if(dataHM[HEADER_HM_SIZE+2] == VIBRATE_EN_CMD)
			{
				stt = true;
				vibrateStatus = true;
			}
		}else{
			LOG_WARN("start_vibration timeout");
		}		
	}
	osMutexRelease(MasterMutexHandle);
	return stt;
}

bool stop_vibration(void)
{
	// duty cycle PWM vibration
	if(vibrateStatus == false){
		return false;
	}
	bool stt = false;
	osMutexAcquire(MasterMutexHandle, osWaitForever);
	uint8_t data[2] = {0, 0};
	if(send_data_HM(data, 2, VIBRATE_EN_CMD) == HmOK)
	{
		if(get_HM_response(500) == HmOK)
		{
			//parse data
			if(dataHM[HEADER_HM_SIZE+2] == VIBRATE_EN_CMD)
			{
				stt = true;
				vibrateStatus = false;
			}
		}else{
			LOG_WARN("stop_vibration timeout");
		}
	}
	osMutexRelease(MasterMutexHandle);
	return stt;
}

int MasterGetFwVersion(FwVersion_t *fwVer)
{
	int ret = ERROR;
	osMutexAcquire(MasterMutexHandle, osWaitForever);
	if(send_data_HM(NULL, 0, CMD_MASTER_GET_FW_VERSION) == HmOK)
	{
		if(get_HM_response(1000) == HmOK)
		{
//			LOG_HEXDUMP("Get Fw Rx",(uint8_t*)dataHM,BUFF_HM_SIZE);
			if(dataHM[HEADER_HM_SIZE+2] == CMD_MASTER_GET_FW_VERSION)
			{
				memcpy((uint8_t*)fwVer,(uint8_t*)&dataHM[HEADER_HM_SIZE+3],sizeof(FwVersion_t));
				ret = SUCCESS;
			}
		}else{
			LOG_WARN("MasterGetFwVersion timeout");
		}
	}
	osMutexRelease(MasterMutexHandle);
	return ret;
}

int MasterJumpToBoot()
{
	int ret = ERROR;
	osMutexAcquire(MasterMutexHandle, osWaitForever);
	if(send_data_HM(NULL, 0, CMD_MASTER_JUMP_TO_BOOTLOADER) == HmOK)
	{
		if(get_HM_response(5000) == HmOK)
		{
//			LOG_HEXDUMP("Get Fw Rx",(uint8_t*)dataHM,BUFF_HM_SIZE);
			if(dataHM[HEADER_HM_SIZE+2] == CMD_MASTER_JUMP_TO_BOOTLOADER)
			{
				LOG_DEBUG("MasterJumpToBoot success");
				ret = SUCCESS;
			}
			else{
				LOG_ERROR("MasterJumpToBoot failed");
			}
		}else{
			LOG_WARN("MasterJumpToBoot timeout");
		}
	}
	osMutexRelease(MasterMutexHandle);
	return ret;
}

int MasterGetModeOperation(MasterModeOperation_t *mode){
	int ret = ERROR;
	osMutexAcquire(MasterMutexHandle, osWaitForever);
	if(send_data_HM(NULL, 0, CMD_MASTER_GET_MODE_OPERATION) == HmOK)
	{
		if(get_HM_response(5000) == HmOK)
		{
//			LOG_HEXDUMP("Get Fw Rx",(uint8_t*)dataHM,BUFF_HM_SIZE);
			if(dataHM[HEADER_HM_SIZE+2] == CMD_MASTER_GET_MODE_OPERATION)
			{
				*mode = dataHM[HEADER_HM_SIZE+3];
				LOG_DEBUG("MasterGetModeOperation : %d",*mode);
				ret = SUCCESS;
			}
		}else{
			LOG_WARN("MasterGetModeOperation timeout");
		}
	}
	osMutexRelease(MasterMutexHandle);
	return ret;
}

static const uint8_t HM_CRC_Cal8Bits(uint8_t *Buffer, uint16_t Size)
{
	static const uint8_t CrcTable[] = {
			0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
			0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
			0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
			0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
			0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
			0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
			0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
			0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
			0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
			0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
			0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
			0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
			0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
			0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
			0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
			0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
	};
	uint8_t crc = 0;
  while(Size--)
  {
    crc = crc ^ *Buffer++; // Apply Byte
 
    crc = CrcTable[crc & 0xFF]; // One round of 8-bits
  }
 
  return(crc);
}
 
