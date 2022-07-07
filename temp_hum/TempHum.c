#include <string.h>
#include "cmsis_os2.h"
#include "i2c.h"
#include "log.h"
#include "TempHum.h"
#include "LcdCtrl.h"

#define	SHT40_ADDR	0x44<<1       
#define CRC8_POLYNOMIAL 0x31  //x8 + x5 + x4 + 1

static uint8_t SHT40_Send_Cmd(SHT40_CMD cmd)
{
	uint8_t cmd_buffer = cmd;
	return HAL_I2C_Master_Transmit(&hi2c1, SHT40_ADDR, (uint8_t*)&cmd_buffer, 1, 1000);
}

uint8_t SHT40_ValGet(uint8_t* dat)
{
	uint8_t Error = HAL_OK;
	Error = SHT40_Send_Cmd(HIGH_ENABLED_CMD);
	if(Error != HAL_OK)
	{
		return Error;
	}	
	osDelay(10);

	return HAL_I2C_Master_Receive(&hi2c1, SHT40_ADDR, dat, 6, 1000);
}

void SHT40_reset(void)
{
	SHT40_Send_Cmd(SOFT_RESET_CMD);
	osDelay(20);
}

uint8_t CheckCrc8(uint8_t* const message, uint8_t initial_value)
{
	uint8_t  remainder;	    
	uint8_t  i = 0, j = 0;  

	remainder = initial_value;

	for(j = 0; j < 2;j++)
	{
			remainder ^= message[j];
			for (i = 0; i < 8; i++)
			{
					if (remainder & 0x80)
					{
							remainder = (remainder << 1)^CRC8_POLYNOMIAL;
					}
					else
					{
							remainder = (remainder << 1);
					}
			}
	}

	return remainder;
}


uint8_t SHT40_Dat_To_Float(uint8_t* const dat, float* temperature, float* humidity)
{
	uint16_t recv_temperature = 0;
	uint16_t recv_humidity = 0;

	if(CheckCrc8(dat, 0xFF) != dat[2] || CheckCrc8(&dat[3], 0xFF) != dat[5])
		return 1;
	
	recv_temperature = ((uint16_t)dat[0]<<8)|dat[1];

	*temperature = -45.0 + 175*((float)recv_temperature/65535);
	if(LcdCtrlGetStatusDisplay() == DIS_STATUS_ON){
		*temperature -= 2.2;
	}
	else{
		*temperature -= 1.2;
	}
	
	recv_humidity = ((uint16_t)dat[3]<<8)|dat[4];
	*humidity = 4.0 + 125 * (float)recv_humidity/65535;
	 if (*humidity > 100)
		*humidity = 100;
	 if (*humidity < 0)
	 	 *humidity = 0;
	
	return 0;
}

int GetTempAndHumi(int8_t *temp, uint8_t *hum)
{
	uint8_t Error = 0;
	float Temperature_C = 0,Humidity = 0;
	uint8_t I2CRXBuffer[6];
	memset(I2CRXBuffer, 0, sizeof(I2CRXBuffer));
	
	if(SHT40_ValGet(I2CRXBuffer) == HAL_OK)
	{
		Error = SHT40_Dat_To_Float(I2CRXBuffer, &Temperature_C, &Humidity);
		if(!Error)
		{			
			*temp = (int8_t)Temperature_C;
			*hum = (uint8_t)Humidity;
		}
	}
	else
	{
		Error = 1;
	}
	
	return Error;
}

int8_t GetTemp(void)
{
	uint8_t Error = 0;
	int8_t Temp = 37;
	float Temperature_C = 0,Humidity = 0;
	uint8_t I2CRXBuffer[6];
	memset(I2CRXBuffer, 0, sizeof(I2CRXBuffer));
	
	if(SHT40_ValGet(I2CRXBuffer) == HAL_OK)
	{
		Error = SHT40_Dat_To_Float(I2CRXBuffer, &Temperature_C, &Humidity);
		if(!Error)
		{			
			Temp = (int8_t)Temperature_C;
		}
	}
	else
	{
		Error = 1;
	}
	
	return Temp;
}
