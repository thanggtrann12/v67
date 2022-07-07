#ifndef __THSENSOR_H
#define __THSENSOR_H
#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
typedef enum
{
    SOFT_RESET_CMD = 0x94 ,	
		READ_SERIAL_CMD = 0x89 ,	
    HIGH_ENABLED_CMD    = 0xFD,
    MEDIUM_ENABLED_CMD  = 0xF6,
    LOW_ENABLED_CMD     = 0xE0,
} SHT40_CMD;
int GetTempAndHumi(int8_t *temp, uint8_t *hum);
int8_t GetTemp(void);
#ifdef __cplusplus
}
#endif
#endif
