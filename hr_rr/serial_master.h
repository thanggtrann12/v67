#ifndef __SERIAL_MASTER_H
#define __SERIAL_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stdbool.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h" 
#include "serial_master.h" 
#include "UgMaster.h"

typedef enum{
	HR_DATA_CMD = 0,
	START_SAMPLING_CMD = 1,
	STOP_SAMPLING_CMD = 2,
	CHECK_OCCUPIED_CMD = 3,
	VIBRATE_EN_CMD = 4,
	VIBRATE_DIS_CMD = 5,
	GUI_MODE_CMD = 6,
	NORMAL_MODE_CMD = 7,
	NACK_CMD = 8,
	CMD_MASTER_GET_FW_VERSION = 0x40,
	CMD_MASTER_JUMP_TO_BOOTLOADER = 0x41,
	CMD_MASTER_GET_MODE_OPERATION = 0x42,
}HM_CMD_t;

typedef enum { 
	HmOK = 0, 
	HmError = -1, 
	HmTimeout = -2,
}HM_status_t;

typedef enum {
	MasterTimeout = -1,
	MasterEmpty = 0,
	MasterOccupied = 1,
}masterStatus_t;

void process_data_HM(void);
void sof_reset_master(void);
HM_status_t send_data_HM(uint8_t *data, uint16_t size, HM_CMD_t HM_CMD);
bool request_hr_data(void);
masterStatus_t check_master_occupied(void);
bool start_vibration(uint8_t time_vibrate);
bool stop_vibration(void);
bool vibrateIsOn(void);
bool send_calib_empty_cmd(void);
bool send_calib_person_cmd(void);
bool master_start_sampling(void);
bool master_stop_sampling(void);

int MasterGetFwVersion(FwVersion_t *fwVer);
int MasterJumpToBoot();
int MasterGetModeOperation(MasterModeOperation_t *mode);

bool check_sum_data(uint8_t *buf, uint16_t len, uint8_t CK_A, uint8_t CK_B);
void create_check_sum_data(uint8_t *buf, uint8_t len, uint8_t* CK_A, uint8_t* CK_B);

#ifdef __cplusplus
}
#endif

#endif
