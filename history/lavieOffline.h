/*
 * lavieOffline.h
 *
 *  Created on: Jun 2, 2021
 *      Author: Thien Phan
 */

#ifndef HISTORY_LAVIEOFFLINE_H_
#define HISTORY_LAVIEOFFLINE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "define.h"

#define NUM_DATA_15M (uint16_t)15 //10 minutes

class lavieOffline {
public:
	lavieOffline();
	virtual ~lavieOffline();

	lavie_data_t get_offline_data(uint16_t index);
	void set_offline_data(lavie_data_t lavie_data);
	void reset_index_offline(void);
	void calculate_offline_data(lavie_data_t *lavieRealtimeData, uint16_t size);
	void publish_offline_payload(void);

	uint16_t getIndexOffline() const {
		if(indexOffline > 0){
			return indexOffline - 1;
		}
		return indexOffline;
	}

private:
	uint16_t indexOffline;
	lavie_data_t lavieRTdata15M[NUM_DATA_15M];

	uint8_t instantHrMin;
	uint16_t instantHrAvg;
	uint8_t instantHrMax;
	uint8_t actualHrMin;
	uint16_t actualHrAvg;
	uint8_t actualHrMax;
	uint8_t rrMin;
	uint16_t rrAvg;
	uint8_t rrMax;
	uint16_t totalApnea;
	uint16_t totalMotions;
	uint16_t totalSnoring;
	uint32_t Th_rtMin;
	uint32_t Th_rtAvg;
	uint32_t Th_rtMax;
	uint32_t Th_bgMin;
	uint32_t Th_bgAvg;
	uint32_t Th_bgMax;
	int8_t roomTempMin;
	int16_t roomTempAvg;
	int8_t roomTempMax;
	uint8_t roomHumMin;
	uint16_t roomHumAvg;
	uint8_t roomHumMax;
	int8_t bodyTempMin;
	int16_t bodyTempAvg;
	int8_t bodyTempMax;
	int8_t bodyTempVarMin;
	int16_t bodyTempVarAvg;
	int8_t bodyTempVarMax;
	float pnpBgWeak;
	float pnpBgOk;
	float pnpBgGood;
	float pnpBgVGood;
	float pnpBgExcel;
	float pnpRtMin;
	float pnpRtAvg;
	float pnpRtMax;
	float movEnergyMin;
	float movEnergyAvg;
	float movEnergyMax;
	float maxPeak;
	bool bed_occupied;
	char time[TIME_SIZE];
};

#ifdef __cplusplus
}
#endif

#endif /* HISTORY_LAVIEOFFLINE_H_ */
