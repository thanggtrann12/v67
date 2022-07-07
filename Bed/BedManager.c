/*
 * BedManager.c
 *
 *  Created on: Mar 30, 2022
 *      Author: Thien Phan
 */
#include "BedManager.h"
#include "settings.h"
#include "OslavieMqttHandle.h"
#include "gpio.h"

#define BED_EMPTY_CHECK_COUNT						3
#define BED_OCCUPIED_CHECK_COUNT					2

static bool preAutoMode = false;
static bool pnpBedMovFlag = false;
static uint8_t countEmptyBed, countOccBed;
static bool bedPnp = false;
static bool preCapa = false, capa = false;

static OperationMode_t modeOpePre = OPE_MODE_MANUAL;
bool BedManagerPreAutoModeGet()
{
	return preAutoMode;
}

void BedManagerPreAutoModeSet(bool mode)
{
	if(mode){
		modeOpePre = OPE_MODE_AUTO;
	}
	else{
		modeOpePre = OPE_MODE_MANUAL;
	}
	preAutoMode = mode;
}

void BedManagerReturnAutoMode()
{
	if(BedManagerPreAutoModeGet())
	{
		OperationModeCf_t OperationModeCf = SettingGetModeOperation();
		OperationModeCf.mode = OPE_MODE_AUTO;
		SettingSetModeOperation(OperationModeCf);
		if(OperationModeCf.mode == OPE_MODE_AUTO){
			UmainEnableShowTime();
		}
		BedManagerPreAutoModeSet(false);
		LcdUpdateModeOperation(OperationModeCf);
		HubModeOperationResponse();
	}
}

static bool bedCapaStt = false;
bool BedManagerCapaGet(){
	return bedCapaStt;
}

void BedManagerCapaSet(bool bedStt){
	OperationModeCf_t modeOpeG = SettingGetModeOperation();
	preCapa = capa;
	capa = bedStt;
	if(modeOpeG.mode != modeOpePre){
		LOG_WARN("modeOpePre :  %d",modeOpePre);
		if(modeOpeG.mode == OPE_MODE_AUTO){
			//			BedManagerPnpSet(bedStt);
			bedPnp = bedStt;
			countEmptyBed = 0;
			countOccBed = 0;
			pnpBedMovFlag = false;
			LOG_WARN("preBedPnp_reset :  %d  bedStt : %d", bedPnp, bedStt);
			if(bedStt == false){
				EmptyProcess(true);
			}
		}
		modeOpePre = modeOpeG.mode;
	}

	bool preCapaBedStt = bedCapaStt;
	if((!preCapaBedStt) && bedStt)
	{
		BedManagerPredictUserNormalModeSet(true);
		BedManagerReturnAutoMode();
		bedCapaStt = bedStt;
		BedManagerPnpSet(true);
		EmptyBedUpdate(bedPnp);
		if(LcdCtrlIsSleeping() == true){
			LcdCtrlSleepOutDisplay();
		}
	}
	else if(preCapaBedStt && (!bedStt))
	{
		BedManagerPredictUserNormalModeSet(false);
		BedManagerReturnAutoMode();
		bedCapaStt = bedStt;
		OperationModeCf_t modeOpe = SettingGetModeOperation();
		if(modeOpe.mode == OPE_MODE_AUTO){
			BedManagerPnpSet(false);
		}
		else{
			bedPnp = false;
			EmptyBedUpdate(bedPnp);
			if(LcdCtrlIsSleeping() == true){
				LcdCtrlSleepOutDisplay();
			}
		}
	}
}

bool BedManagerPnpGet()
{
	return bedPnp;
}


void BedManagerPnpSet(bool bedStt){
	bool preBedPnp = bedPnp;
	bedPnp = bedStt;
	countEmptyBed = 0;
	countOccBed = 0;
	pnpBedMovFlag = false;
	LOG_WARN("preBedPnp :  %d  bedStt : %d", preBedPnp, bedStt);
	OperationModeCf_t OperationModeCf = SettingGetModeOperation();
	if(OperationModeCf.mode == OPE_MODE_AUTO){
		if((preCapa == true) && (bedStt == false)){
			EmptyProcess(true);
			return;
		}
	}
	if(bedStt == false){
		EmptyProcess(false);
	}
	else if(preBedPnp == false && bedStt == true)
	{
		OccupiedProcess(true);
	}
}

BedManagerPnpSate_t bedManagerPnpState = BED_MANAGER_PNP_BED_NOT_GOOD;

BedManagerPnpSate_t BedManagerGetPnpState(){
	return bedManagerPnpState;
}
void BedManagerPnpPredict(BedManagerPnpSate_t BedManagerPnpSate)
{
	bedManagerPnpState = BedManagerPnpSate;
	if(BedManagerPnpSate == BED_MANAGER_PNP_MOVEMENT)
	{
		pnpBedMovFlag = true;
		countEmptyBed = 0;
		countOccBed = 0;
		//		LOG_DEBUGF("PnpBedPredict: BED_MANAGER_PNP_MOVEMENT");
		//		LOG_WARN("PnpBedPredict: BED_MANAGER_PNP_MOVEMENT");
		return;
	}
	if(BedManagerPnpSate == BED_MANAGER_PNP_BED_GOOD)
	{
		LOG_WARN("PnpBedPredict: BED_MANAGER_PNP_BED_GOOD");
		if(SettingGetModeOperation().mode == OPE_MODE_MANUAL){
			if(countOccBed + 1 >= BED_OCCUPIED_CHECK_COUNT){
				BedManagerPnpSet(true);
				countEmptyBed = 0;
			}
			else{
				countOccBed++;
			}
		}
		else{
			countEmptyBed = 0;
		}
	}
//	if(pnpBedMovFlag == false)
//	{
//		//LOG_DEBUGF("pnpBedMovFlag: %d", pnpBedMovFlag);
//		LOG_WARN("pnpBedMovFlag = false, dont change empty bed");
//		return;
//	}
	if(BedManagerPnpSate == BED_MANAGER_PNP_BED_NOT_GOOD)
	{
		LOG_WARN("PnpBedPredict: BED_MANAGER_PNP_BED_NOT_GOOD");
		if(SettingGetModeOperation().mode == OPE_MODE_MANUAL){
			if(countEmptyBed + 1 >= BED_EMPTY_CHECK_COUNT){
				if(bedCapaStt == 0){
					BedManagerPnpSet(false);
				}
			}
			else if(pnpBedMovFlag){
				countEmptyBed++;
			}
		}
		countOccBed = 0;
	}
	LOG_WARN("BedManagerPnpSate %d, pnpBedMovFlag %d, countEmptyBed %d, countOccBed %d",
			BedManagerPnpSate, pnpBedMovFlag, countEmptyBed, countOccBed);
}

static bool bedManagerHaveUserFlag = false;
void BedManagerPredictUserNormalModeSet(bool haveUserFlag)
{
	bedManagerHaveUserFlag = haveUserFlag;
}

bool BedManagerPredictUserNormalModeGet()
{
	return bedManagerHaveUserFlag;
}





