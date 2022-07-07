#include "osMain.h"
#include "tim.h"
#include "settings.h"
#include "OslavieMQTTClient.h"
#include "Config.h"
#include "Flash.h"
#include "Utils.h"
#include "Lcd.h"
#include "serial_master.h"
#include "lifesos.h"
#include "Rtos.h"
#include "LcdCtrl.h"
#include "MacAddress.h"
#include "BedManager.h"

#define	FLASH_ADDR_USER_SAVE_CONFIG				(FLASH_ADDR_SAVE_CONFIG + FLASH_AREA_BOOT_CONFIG_SIZE + FLASH_AREA_MAC_ADDR_SIZE)
#define	FLASH_ADDR_USER_SAVE_CONFIG_BK			(FLASH_ADDR_SAVE_CONFIG_BK + FLASH_AREA_BOOT_CONFIG_SIZE_BK + FLASH_AREA_MAC_ADDR_SIZE)

static lavie_params_t lavie_params, pre_lavie_params;
extern float32_t avgsq_noise_x, avgsq_noise_y, avgsq_noise_z, avgsq_noise_xyz, avg_energy_xy, avg_energy_xyz, avg_energy_resp, noise_dft, eb_th_resp_freq, eb_th_resp_time;
nobreathing_mode_t preNoBreathingMode;

static int get_lavie_params_from_flash(lavie_params_t *lavie_params)
{
	if(pdTRUE != xSemaphoreTake(GetMutexCf(),portMAX_DELAY)){
		LOG_ERROR("take mutexCf failed");
	}
	int ret = HAL_OK;
	lavie_params_t paraIn,paraBk;
	ret = FlashRead((uint8_t*)&paraIn, FLASH_ADDR_USER_SAVE_CONFIG, sizeof(lavie_params_t));
	if(HAL_OK != ret){
		LOG_ERROR("read Lavie para failed");
	}

	ret = FlashRead((uint8_t*)&paraBk, FLASH_ADDR_USER_SAVE_CONFIG_BK, sizeof(lavie_params_t));
	if(HAL_OK != ret){
		LOG_ERROR("read Lavie para bk failed");
	}

	if((paraIn.haveCf != HAVE_CF_CODE)&&(paraBk.haveCf != HAVE_CF_CODE)){
		LOG_DEBUG("Lavie para config");
		memset((uint8_t*)lavie_params,0xff,sizeof(lavie_params_t));
	}
	else if((paraIn.haveCf == HAVE_CF_CODE)&&(paraBk.haveCf != HAVE_CF_CODE)){
		memcpy((uint8_t*)lavie_params,(uint8_t*)&paraIn,sizeof(lavie_params_t));
		LOG_DEBUG("Lavie para Write Bk");
		ret = FlashWrite((const uint8_t*)lavie_params, FLASH_ADDR_USER_SAVE_CONFIG_BK, sizeof(lavie_params_t),FLASH_MODE_WRITE_SINGLE);
		if(ret != HAL_OK){
			LOG_ERROR("Lavie para wite_bk failed");
		}
	}
	else if((paraIn.haveCf != HAVE_CF_CODE)&&(paraBk.haveCf == HAVE_CF_CODE)){
		memcpy((uint8_t*)lavie_params,(uint8_t*)&paraBk,sizeof(lavie_params_t));
		LOG_DEBUG("Lavie para Write In");
		ret = FlashWrite((const uint8_t*)lavie_params, FLASH_ADDR_USER_SAVE_CONFIG, sizeof(lavie_params_t),FLASH_MODE_WRITE_SINGLE);
		if(ret != HAL_OK){
			LOG_ERROR("Lavie para wite in failed");
		}
	}
	else{
		memcpy((uint8_t*)lavie_params,&paraIn,sizeof(lavie_params_t));
	}
	memcpy((uint8_t*)&pre_lavie_params, (uint8_t*)lavie_params, sizeof(lavie_params_t));
	if(pdTRUE != xSemaphoreGive(GetMutexCf())){
		LOG_ERROR("UgGetConfig failed");
		ret = HAL_ERROR;
	}
	return ret;
}

static int save_lavie_params_to_flash(lavie_params_t* lavie_params)
{
	if(memcmp(&pre_lavie_params, lavie_params, sizeof(lavie_params_t)) == 0)
	{
		return HAL_OK;
	}
	if(pdTRUE != xSemaphoreTake(GetMutexCf(),portMAX_DELAY)){
		LOG_ERROR("take mutexCf failed");
	}
	lavie_params->haveCf = HAVE_CF_CODE;
	int ret = FlashWrite((const uint8_t*)lavie_params, FLASH_ADDR_USER_SAVE_CONFIG, sizeof(lavie_params_t),FLASH_MODE_WRITE_SINGLE);
	if(ret != HAL_OK){
		LOG_ERROR("Lavie_save to inter failed");
	}
	ret = FlashWrite((const uint8_t*)lavie_params, FLASH_ADDR_USER_SAVE_CONFIG_BK, sizeof(lavie_params_t),FLASH_MODE_WRITE_SINGLE);
	if(ret != HAL_OK){
		LOG_ERROR("Lavie_save to exter failed");
	}
	if(ret == HAL_OK){
		memcpy((uint8_t*)&pre_lavie_params, (uint8_t*)lavie_params, sizeof(lavie_params_t));
	}
	if(pdTRUE != xSemaphoreGive(GetMutexCf())){
		LOG_ERROR("UgGetConfig failed");
	}
	return ret;
}

static SettingCalibStatus_t settingCalibStatus = SETTING_CALIB_NONE;
SettingCalibStatus_t SettingGetCalibStatus(void){
	return settingCalibStatus;
}

void init_calib_params(void)
{
	settingCalibStatus = SETTING_CALIB_NONE;
	bool emptyFlag = false, personFlag = false;
	LOG_DEBUGF("FLASH: avgsq_noise_x %f, avgsq_noise_y %f, avgsq_noise_z %f, avgsq_noise_xyz %f, avg_energy_xy %f, avg_energy_xyz %f, avg_energy_resp %f, eb_th_resp_freq %f, eb_th_resp_time %f",
			lavie_params.calib_params.avgsq_noise_x, lavie_params.calib_params.avgsq_noise_y, lavie_params.calib_params.avgsq_noise_z, lavie_params.calib_params.avgsq_noise_xyz,
			lavie_params.calib_params.avg_energy_xy, lavie_params.calib_params.avg_energy_xyz, lavie_params.calib_params.avg_energy_resp, lavie_params.calib_params.eb_th_resp_freq, lavie_params.calib_params.eb_th_resp_time);

	if(lavie_params.calib_params.avgsq_noise_x > 120 && lavie_params.calib_params.avgsq_noise_x < 400000
			&& lavie_params.calib_params.avgsq_noise_y > 120 && lavie_params.calib_params.avgsq_noise_y < 400000
			&& lavie_params.calib_params.avgsq_noise_z > 120 && lavie_params.calib_params.avgsq_noise_z < 400000
			&& lavie_params.calib_params.avgsq_noise_xyz > 120 && lavie_params.calib_params.avgsq_noise_xyz < 600000)
	{
		LOG_DEBUGF("Get calib params from flash");
		avgsq_noise_x = lavie_params.calib_params.avgsq_noise_x;
		avgsq_noise_y = lavie_params.calib_params.avgsq_noise_y;
		avgsq_noise_z = lavie_params.calib_params.avgsq_noise_z;
		avgsq_noise_xyz = lavie_params.calib_params.avgsq_noise_xyz;
		eb_th_resp_freq = lavie_params.calib_params.eb_th_resp_freq;
//		eb_th_resp_time = lavie_params.calib_params.eb_th_resp_time;
		emptyFlag = true;
	}
	if(lavie_params.calib_params.avg_energy_xy > 10e2 && lavie_params.calib_params.avg_energy_xy < 10e20
			&& lavie_params.calib_params.avg_energy_xyz > 10e2 && lavie_params.calib_params.avg_energy_xyz < 10e20
			&& lavie_params.calib_params.avg_energy_resp > 10e2 && lavie_params.calib_params.avg_energy_resp < 10e20)
	{
		avg_energy_xy = lavie_params.calib_params.avg_energy_xy;
		avg_energy_xyz = lavie_params.calib_params.avg_energy_xyz;
		avg_energy_resp = lavie_params.calib_params.avg_energy_resp;
		personFlag = true;
	}
	if(emptyFlag == true && personFlag == true)
	{
		settingCalibStatus = SETTING_CALIB_FULL;
	}
	else if(emptyFlag == true)
	{
		settingCalibStatus = SETTING_CALIB_ONLY_EMPTY;
	}
	else if(personFlag == true)
	{
		settingCalibStatus = SETTING_CALIB_ONLY_PERSON;
	}
	else{
		settingCalibStatus = SETTING_CALIB_NONE;
	}

	LOG_DEBUGF("avgsq_noise_x %f, avgsq_noise_y %f, avgsq_noise_z %f, avgsq_noise_xyz %f, mov_max_energy_x %f, mov_max_energy_xz %f, mov_max_resp %f, eb_th_resp_freq %f, eb_th_resp_time %f",
			avgsq_noise_x, avgsq_noise_y, avgsq_noise_z, avgsq_noise_xyz, avg_energy_xy, avg_energy_xyz, avg_energy_resp, eb_th_resp_freq, eb_th_resp_time);
}	

void init_setting_params(void)
{	
	if(lavie_params.timeZone == 0xffffffff){
		lavie_params.timeZone = TIME_ZONE_DEFAULT;
	}
	if(lavie_params.tempUnit > TEMP_UNIT_F){
		lavie_params.tempUnit = TEMP_UNIT_F;
	}
	if(lavie_params.userAtUnit > USER_AT_UNIT_USA){
		lavie_params.userAtUnit = USER_AT_UNIT_USA;
	}
	if(lavie_params.padType > PAD_TYPE_3_SENSOR){
		lavie_params.padType = PAD_TYPE_1_SENSOR;
	}
	if(lavie_params.setting_params.timeSleepLv > DISPLAY_SLEEP_TIME_LV9){
		lavie_params.setting_params.timeSleepLv = DISPLAY_SLEEP_TIME_LV4;
	}
	if((lavie_params.setting_params.buzzerVolume < BUZZER_DISABLE) || (lavie_params.setting_params.buzzerVolume > BUZZER_HIGH))
	{
		lavie_params.setting_params.buzzerVolume = BUZZER_MED;
	}
	if((lavie_params.setting_params.vibrate <= VIBRATE_DISABLE) || (lavie_params.setting_params.vibrate > VIBRATE_HIGH))
	{
		lavie_params.setting_params.vibrate = VIBRATE_MED;
	}	
	if((lavie_params.setting_params.seizure < SEIZURE_NONE) || (lavie_params.setting_params.seizure > SEIZURE_HIGH))
	{
		lavie_params.setting_params.seizure = SEIZURE_NONE;
	}
	if((lavie_params.setting_params.nobreathing_mode != NOBREATH_DISABLE) && (lavie_params.setting_params.nobreathing_mode != NOBREATH_DEMO)
			&& (lavie_params.setting_params.nobreathing_mode != NOBREATH_HIGH) && (lavie_params.setting_params.nobreathing_mode != NOBREATH_MEDIUM_1)
			&& (lavie_params.setting_params.nobreathing_mode != NOBREATH_MEDIUM_2)&& (lavie_params.setting_params.nobreathing_mode != NOBREATH_LOW))
	{
		lavie_params.setting_params.nobreathing_mode = NOBREATH_DISABLE;
	}	
	preNoBreathingMode = lavie_params.setting_params.nobreathing_mode;
#ifdef PAD_3_SENSOR
	if(lavie_params.modeOperation.mode == OPE_MODE_AUTO)
	{
		BedManagerPreAutoModeSet(true);
	}
	lavie_params.modeOperation.mode = OPE_MODE_MANUAL;
//	if(lavie_params.modeOperation.mode > OPE_MODE_MANUAL){
//		lavie_params.modeOperation.mode = OPE_MODE_MANUAL;
//	}
	if(lavie_params.modeOperation.flag > OPE_FLAG_MANUAL_ENABLE_RETURN_AUTO_WHEN_LYING_IN_BED){
		lavie_params.modeOperation.flag = OPE_FLAG_MANUAL_ENABLE_RETURN_AUTO_WHEN_LYING_IN_BED;
	}
#else
	lavie_params.modeOperation.mode = OPE_MODE_MANUAL;
#endif

	set_time_nobreathing(lavie_params.setting_params.nobreathing_mode);
	config_buzzer_volume(lavie_params.setting_params.buzzerVolume);
	LOG_DEBUGF("nobreathing_mode %d, seizure_setting %d, vibrate_setting %d, buzzer_setting %d",
			lavie_params.setting_params.nobreathing_mode, lavie_params.setting_params.seizure,
			lavie_params.setting_params.vibrate, lavie_params.setting_params.buzzerVolume);
}

void init_lavie_params(void)
{
	LOG_DEBUGF("Init lavie params");
	LcdModeConfigInit();
	MacAddressInit();
	get_lavie_params_from_flash(&lavie_params);
	init_calib_params();
	init_setting_params();
}

void SettingSaveCalibPersonParams(const calib_params_t calib_params)
{
	lavie_params.calib_params.avg_energy_xy = calib_params.avg_energy_xy;
	lavie_params.calib_params.avg_energy_xyz = calib_params.avg_energy_xyz;
	lavie_params.calib_params.avg_energy_resp = calib_params.avg_energy_resp;
	save_lavie_params_to_flash(&lavie_params);
	LOG_DEBUGF("FLASH: avgsq_noise_x %f, avgsq_noise_y %f, avgsq_noise_z %f, avgsq_noise_xyz %f, avg_energy_xy %f, avg_energy_xyz %f, avg_energy_resp %f, eb_th_resp_freq %f, eb_th_resp_time %f",
			lavie_params.calib_params.avgsq_noise_x, lavie_params.calib_params.avgsq_noise_y, lavie_params.calib_params.avgsq_noise_z, lavie_params.calib_params.avgsq_noise_xyz,
			lavie_params.calib_params.avg_energy_xy, lavie_params.calib_params.avg_energy_xyz, lavie_params.calib_params.avg_energy_resp, lavie_params.calib_params.eb_th_resp_freq, lavie_params.calib_params.eb_th_resp_time);
	init_calib_params();
}

void SettingSaveCalibEmptyParams(const calib_params_t calib_params)
{
	lavie_params.calib_params.avgsq_noise_x = calib_params.avgsq_noise_x;
	lavie_params.calib_params.avgsq_noise_y = calib_params.avgsq_noise_y;
	lavie_params.calib_params.avgsq_noise_z = calib_params.avgsq_noise_z;
	lavie_params.calib_params.avgsq_noise_xyz = calib_params.avgsq_noise_xyz;
	lavie_params.calib_params.eb_th_resp_freq = calib_params.eb_th_resp_freq;
	lavie_params.calib_params.eb_th_resp_time = calib_params.eb_th_resp_time;
	save_lavie_params_to_flash(&lavie_params);
	LOG_DEBUGF("FLASH: avgsq_noise_x %f, avgsq_noise_y %f, avgsq_noise_z %f, avgsq_noise_xyz %f, avg_energy_xy %f, avg_energy_xyz %f, avg_energy_resp %f, eb_th_resp_freq %f, eb_th_resp_time %f",
			lavie_params.calib_params.avgsq_noise_x, lavie_params.calib_params.avgsq_noise_y, lavie_params.calib_params.avgsq_noise_z, lavie_params.calib_params.avgsq_noise_xyz,
			lavie_params.calib_params.avg_energy_xy, lavie_params.calib_params.avg_energy_xyz, lavie_params.calib_params.avg_energy_resp, lavie_params.calib_params.eb_th_resp_freq, lavie_params.calib_params.eb_th_resp_time);
	init_calib_params();
}

void SettingSaveParams(void)
{
	osSemaphoreRelease(SemaphoreSaveSetting);
}
void SettingSaveParamsSync(){
	save_lavie_params_to_flash(&lavie_params);
}

void SettingReset(){
	memset(&lavie_params,0xff,sizeof(lavie_params));
	save_lavie_params_to_flash(&lavie_params);
}

void config_buzzer_volume(buzzerVolume_t buzzerVolume)
{
	if(buzzerVolume == BUZZER_LOW)
	{
		lavie_params.setting_params.buzzerVolume = BUZZER_LOW;
		HAL_GPIO_WritePin(BUZZ_EN1_GPIO_Port, BUZZ_EN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BUZZ_EN1_GPIO_Port, BUZZ_EN2_Pin, GPIO_PIN_SET);
		SettingSaveParams();
	}
	else if(buzzerVolume == BUZZER_MED)
	{
		lavie_params.setting_params.buzzerVolume = BUZZER_MED;
		HAL_GPIO_WritePin(BUZZ_EN1_GPIO_Port, BUZZ_EN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BUZZ_EN1_GPIO_Port, BUZZ_EN2_Pin, GPIO_PIN_RESET);
		SettingSaveParams();
	}
	else if(buzzerVolume == BUZZER_HIGH)
	{
		lavie_params.setting_params.buzzerVolume = BUZZER_HIGH;
		HAL_GPIO_WritePin(BUZZ_EN1_GPIO_Port, BUZZ_EN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BUZZ_EN1_GPIO_Port, BUZZ_EN2_Pin, GPIO_PIN_SET);
		SettingSaveParams();
	}
	else if(buzzerVolume == BUZZER_DISABLE)
	{
		buzzer_off();
		lavie_params.setting_params.buzzerVolume = BUZZER_DISABLE;
		HAL_GPIO_WritePin(BUZZ_EN1_GPIO_Port, BUZZ_EN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BUZZ_EN1_GPIO_Port, BUZZ_EN2_Pin, GPIO_PIN_RESET);
		SettingSaveParams();
	}
	else
	{
		return;
	}
}

static bool buzzer_stt = false;
void buzzer_on(void)
{
	if(lavie_params.setting_params.buzzerVolume != BUZZER_DISABLE)
	{
		if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK)
		{
			/* PWM Generation Error */
			//Error_Handler();
		}
		buzzer_stt =  true;
	}		
}

void buzzerNotySuccess(void){
	buzzer_on();
	osDelay(150);
	buzzer_off();
}

void buzzerNotyFail(void){
	buzzer_on();
	osDelay(150);
	buzzer_off();
	osDelay(150);
	buzzer_on();
	osDelay(150);
	buzzer_off();
}

void buzzer_off(void)
{
	if (HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3) != HAL_OK)
	{
		/* PWM Generation Error */
		//Error_Handler();
	}
	buzzer_stt =  false;
}

bool buzzer_isOn(void)
{
	if(lavie_params.setting_params.buzzerVolume != BUZZER_DISABLE && buzzer_stt == true)
	{
		return true;
	}
	else{
		return false;
	}
}

buzzerVolume_t get_buzzer_volume(void)
{
	return lavie_params.setting_params.buzzerVolume;
}


void seizure_setting(SeizureVolume_t mode)
{
	SeizureReset();
	lavie_params.setting_params.seizure = mode;
	SettingSaveParams();
}

SeizureVolume_t get_seizure_setting(void)
{
	return lavie_params.setting_params.seizure;
}

void vibrate_setting(vibrateVolume_t mode)
{
	if(mode == VIBRATE_DISABLE)
	{
		stop_vibration();
	}
	lavie_params.setting_params.vibrate = mode;
	SettingSaveParams();
}

vibrateVolume_t get_vibarate_setting(void)
{
	return lavie_params.setting_params.vibrate;
}

//uint32_t get_fw_address(void)
//{
//	return lavie_params.firmware_info.fw_address;
//}

FwVersion_t SettingGetFwVersion(void)
{
	lavie_params.fwVer.major = FIRMWARE_VERSION_MAJOR;
	lavie_params.fwVer.minor = FIRMWARE_VERSION_MINOR;
	lavie_params.fwVer.revision = FIRMWARE_REVISION;
	return lavie_params.fwVer;
}

void set_nobreathing_setting(nobreathing_mode_t mode)
{
	lavie_params.setting_params.nobreathing_mode = mode;
	SettingSaveParams();
}

nobreathing_mode_t get_nobreathing_setting(void)
{
	return lavie_params.setting_params.nobreathing_mode;
}

uint8_t SettingGetTimeSleepDislayLv() {
	return lavie_params.setting_params.timeSleepLv;
}

void SettingSetTimeSleepDislayLv(uint8_t lv){
	LcdCtrlResetTimeCheck();
	lavie_params.setting_params.timeSleepLv = lv;
	SettingSaveParams();
}

void SettingSetUserInfo(char* userName, char* password){
	LOG_DEBUG("%s :-: %s",userName,password);
	strcpy(lavie_params.userInfo.userName,userName);
	strcpy(lavie_params.userInfo.password,password);
	lavie_params.userInfo.haveConfig = SETTING_HAVE_USER_INFO;
	SettingSaveParams();
}

int SettingGetUserInfo(UserInfoSt_t *userInfo){
	if(userInfo == NULL){
		return ERROR;
	}
	if(lavie_params.userInfo.haveConfig == SETTING_HAVE_USER_INFO){
		memcpy(userInfo,&lavie_params.userInfo,sizeof(UserInfoSt_t));
		LOG_DEBUG("UserId : %s",userInfo->userName);
		LOG_DEBUG("Pwd : %s",userInfo->password);
		return SUCCESS;
	}
	else{
		bzero(userInfo,sizeof(UserInfoSt_t));
	}
	return ERROR;
}

bool SettingUserInfoExisted(){
	if(lavie_params.userInfo.haveConfig == SETTING_HAVE_USER_INFO){
		return true;
	}
	return false;
}

void SettingClearUserInfo(){
	LOG_WARN("SettingClearUserInfo");
	bzero(lavie_params.userInfo.userName,sizeof(lavie_params.userInfo.userName));
	bzero(lavie_params.userInfo.password,sizeof(lavie_params.userInfo.password));
	lavie_params.userInfo.haveConfig = 0xFFFFFFFF;
	SettingSaveParams();
}

void SettingSetModeOta(UgModeSw_t mode){
	lavie_params.setting_params.modeOta = mode;
	SettingSaveParams();
}

UgModeSw_t SettingGetModeOta(){
	if(lavie_params.setting_params.modeOta > UG_MODE_RELEASE){
		lavie_params.setting_params.modeOta = UG_MODE_RELEASE;
	}
	return lavie_params.setting_params.modeOta;
}

float_t SettingGetTimeZone(){
	return lavie_params.timeZone;
}

void SettingSetTimeZone(float_t timezone){
	lavie_params.timeZone =  timezone;
	SettingSaveParams();
}

TempUnit_t SettingGetTempUnit(){
	return lavie_params.tempUnit;
}

void SettingSetTempUnit(TempUnit_t unit){
	lavie_params.tempUnit =  unit;
	SettingSaveParams();
}

UserAtriUnit_t SettingGetUserAtUnit(){
	return lavie_params.userAtUnit;
}

void SettingSetUserAtUnit(UserAtriUnit_t unit){
	lavie_params.userAtUnit =  unit;
	SettingSaveParams();
}

OperationModeCf_t SettingGetModeOperation(){
	return lavie_params.modeOperation;
}

void SettingSetModeOperation(OperationModeCf_t mode){
	static OperationMode_t preOperationMode = OPE_MODE_AUTO;
	lavie_params.modeOperation = mode;
	if(preOperationMode != mode.mode)
	{
		preOperationMode = mode.mode;
		if(mode.mode == OPE_MODE_AUTO)
		{
			lavie_params.setting_params.nobreathing_mode = preNoBreathingMode;
			set_time_nobreathing(lavie_params.setting_params.nobreathing_mode);
		}
		else{
			preNoBreathingMode = lavie_params.setting_params.nobreathing_mode;
			lavie_params.setting_params.nobreathing_mode = NOBREATH_DISABLE;
			set_time_nobreathing(lavie_params.setting_params.nobreathing_mode);
		}
	}
	SettingSaveParams();
}

PadType_t SettingGetPadType()
{
	return lavie_params.padType;
}

void SettingSetPadType(PadType_t padType)
{
	lavie_params.padType = padType;
	SettingSaveParamsSync();
}

uint8_t SettingGetSelfReset(){
	return lavie_params.selfResetFlag;
}


void SettingSetSelfReset(uint8_t flag){
	lavie_params.selfResetFlag = flag;
	SettingSaveParamsSync();
}

LanguagesType_t SettingGetLanguagesType() {
	return lavie_params.languagesType;
}

void SettingSetLanguagesType(LanguagesType_t langType) {
	lavie_params.languagesType = langType;
	SettingSaveParams();
}
