/*
 * Calib.h
 *
 *  Created on: Nov 10, 2021
 *      Author: Thien Phan
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef CALIB_CALIB_H_
#define CALIB_CALIB_H_

#include <stdbool.h>

typedef enum{
	CALIB_NONE,
	CALIB_WAIT_PERSON,
	CALIB_PERSON,
	CALIB_PERSON_DONE,
	CALIB_WAIT_EMPTY,
	CALIB_EMPTY,
	CALIB_EMPTY_DONE,
	CALIB_DONE,
	CALIB_ERROR_BED_NOT_EMPTY,
	CALIB_ERROR_HAVE_MOVEMENTS,
	CALIB_ERROR_NO_PERSON,
	CALIB_ERROR_NO_MASTER,
}CalibStatus_t;

bool CalibEmptyProcess(void);
bool CalibPersonProcess(void);
void response_calibration_status_message(void);


#ifdef __cplusplus
}
#endif

#endif /* CALIB_CALIB_H_ */
