/*
 * osMain.h
 *
 *  Created on: May 31, 2021
 *      Author: Thien Phan
 */

#ifndef MAIN_OSMAIN_H_
#define MAIN_OSMAIN_H_

#include "stdint.h"
#include "settings.h"


typedef enum {
	WIFI_INIT_STATE = 0,
	WIFI_CONNECT_STATE,
	WIFI_CONNECTED_STATE,
}wifi_states;

hrRr2Display_t osMainGetHrRr();

#endif /* MAIN_OSMAIN_H_ */
