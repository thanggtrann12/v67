/*
 * BedManager.h
 *
 *  Created on: Mar 30, 2022
 *      Author: Thien Phan
 */
#ifndef BED_BEDMANAGER_H_
#define BED_BEDMANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"

typedef enum{
	BED_MANAGER_PNP_MOVEMENT,
	BED_MANAGER_PNP_BED_GOOD,
	BED_MANAGER_PNP_BED_NOT_GOOD,
}BedManagerPnpSate_t;

bool BedManagerCapaGet();
void BedManagerCapaSet(bool bedStt);
void BedManagerPnpSet(bool bedStt);
bool BedManagerPnpGet();
void BedManagerPnpPredict(BedManagerPnpSate_t BedManagerPnpSate);
void BedManagerPreAutoModeSet(bool mode);
bool BedManagerPreAutoModeGet();
BedManagerPnpSate_t BedManagerGetPnpState();
void BedManagerPredictUserNormalModeSet(bool haveUserFlag);
bool BedManagerPredictUserNormalModeGet();

#ifdef __cplusplus
}
#endif

#endif /* BED_BEDMANAGER_H_ */
