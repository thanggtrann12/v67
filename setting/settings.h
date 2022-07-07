#ifndef __SETTINGS_H
#define __SETTINGS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdbool.h"
#include "resp_detect.h"
#include "Config.h"
#include "Ug.h"
#include <math.h>
#include "LcdUpdate.h"
#include "Lcd.h"

#define SETTING_HAVE_USER_INFO							0xFAFBFCFD

typedef enum {
	BUZZER_DISABLE = 0,
	BUZZER_LOW,
	BUZZER_MED,
	BUZZER_HIGH
}buzzerVolume_t;

typedef enum {
	VIBRATE_DISABLE = 0,
	VIBRATE_LOW,
	VIBRATE_MED,
	VIBRATE_HIGH
}vibrateVolume_t;

typedef enum{
    PAD_TYPE_1_SENSOR,
    PAD_TYPE_3_SENSOR,
}PadType_t;

typedef enum{
	SETTING_CALIB_NONE,
    SETTING_CALIB_ONLY_PERSON,
	SETTING_CALIB_ONLY_EMPTY,
	SETTING_CALIB_FULL
}SettingCalibStatus_t;

typedef struct{
	uint8_t modeOta;
	uint8_t timeSleepLv;
	buzzerVolume_t buzzerVolume;
	nobreathing_mode_t nobreathing_mode;
	vibrateVolume_t vibrate;
	SeizureVolume_t seizure;
}setting_params_t;

typedef struct{
	uint32_t haveConfig;
	char userName[256];
	char password[64];
}UserInfoSt_t;

typedef struct{
	uint32_t haveConfig;
	char macAddress[18];
}MACSt_t;

typedef struct{
	FwVersion_t fwVer;
	UserInfoSt_t userInfo;
	uint32_t haveCf;
	float_t timeZone;
	uint32_t tempUnit;
	uint32_t userAtUnit;
	calib_params_t calib_params;
	setting_params_t setting_params;
	OperationModeCf_t modeOperation;
	PadType_t padType;
	uint8_t selfResetFlag;
	LanguagesType_t languagesType;
}lavie_params_t;

void config_buzzer_volume(buzzerVolume_t buzzerVolume);
void buzzer_disable(void);
void buzzer_enable(void);
void buzzer_off(void);
void buzzer_on(void);
void buzzerNotySuccess(void);
void buzzerNotyFail(void);
buzzerVolume_t get_buzzer_volume(void);
bool buzzer_isOn(void);

void init_lavie_params(void);
void seizure_setting(SeizureVolume_t mode);
SeizureVolume_t get_seizure_setting(void);
vibrateVolume_t get_vibarate_setting(void);
void vibrate_setting(vibrateVolume_t mode);
void vibrate_disable(void);
void vibrate_enable(void);
void set_nobreathing_setting(nobreathing_mode_t mode);
nobreathing_mode_t get_nobreathing_setting(void);
void SettingSaveCalibPersonParams(const calib_params_t calib_params);
void SettingSaveCalibEmptyParams(const calib_params_t calib_params);
FwVersion_t SettingGetFwVersion(void);
uint8_t SettingGetTimeSleepDislayLv();
void SettingSetTimeSleepDislayLv(uint8_t lv);
void SettingSetUserInfo(char* userName, char* password);
int SettingGetUserInfo(UserInfoSt_t *userInfo);
bool SettingUserInfoExisted();
void SettingClearUserInfo();
void SettingSetModeOta(UgModeSw_t mode);
UgModeSw_t SettingGetModeOta();
SettingCalibStatus_t SettingGetCalibStatus(void);
void SettingReset();
float_t SettingGetTimeZone();
void SettingSetTimeZone(float_t timezone);

TempUnit_t SettingGetTempUnit();
void SettingSetTempUnit(TempUnit_t unit);

UserAtriUnit_t SettingGetUserAtUnit();
void SettingSetUserAtUnit(UserAtriUnit_t unit);

OperationModeCf_t SettingGetModeOperation();
void SettingSetModeOperation(OperationModeCf_t mode);

PadType_t SettingGetPadType();
void SettingSetPadType(PadType_t padType_t);

uint8_t SettingGetSelfReset();
void SettingSetSelfReset(uint8_t flag);

LanguagesType_t SettingGetLanguagesType();
void SettingSetLanguagesType(LanguagesType_t langType);


void SettingSaveParamsSync(void);
#ifdef __cplusplus
}
#endif
#endif 
