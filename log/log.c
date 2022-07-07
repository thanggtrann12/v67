#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include "cmsis_os.h"
#include "log.h"
#include "main.h"
#include "usart.h"
#include "stm32f7xx_it.h"
#include "Config.h"

static void print_indent(FILE * stream, int num);

#define LOG_BUF_SIZE (uint16_t)1024
#define LOG_RX_SIZE (uint16_t)50
#define NUM_CMD_USER (uint16_t)4
extern osMutexId_t LogMutexHandle;
static char log_buf[LOG_BUF_SIZE];
static char logRx_buf[LOG_RX_SIZE];
__IO uint16_t logRx_buf_idx = 0;
__IO bool disable_log = false;
__IO bool gui_mode = false;

typedef enum{ENABLE_LOG, DISABLE_LOG, GUI_MODE, GUI_READ_SENSOR}logRx_id_t;
typedef struct{
	logRx_id_t log_id;
	uint8_t cmd_user[LOG_RX_SIZE];
}logRx_cmd_t;

logRx_cmd_t logRx_cmd[NUM_CMD_USER] = {
	{ENABLE_LOG, "enable_log"},
	{DISABLE_LOG, "disable_log"},
	{GUI_MODE, "gui_mode"},
	{GUI_READ_SENSOR, "gui_start_read"},
};

void pLog_Init(void)
{
	MX_USART1_UART_Init();
}

void enter_gui_mode(void)
{
	//printf("enter gui mode\r\n");
	gui_mode = true;
	//send_gui_mode_to_master();
}

/*function use interrupt*/
void parse_cmd_user(uint8_t rx_log)
{
	if(rx_log == 13) //enter
	{
		//printf("\r\n");
		if(logRx_buf_idx > 0)
		{
			for(uint16_t i = 0; i < NUM_CMD_USER; i++)
			{
				if(memcmp(&logRx_cmd[i].cmd_user, logRx_buf, logRx_buf_idx)==0)
				{
					switch(i)
					{
						case ENABLE_LOG:
							disable_log = false;
							//printf("enable log sucessfully!\r\n");
							break;
						case DISABLE_LOG:
							disable_log = true;
							//printf("disabled log sucessfully!\r\n");
							break;
						case GUI_MODE:
							disable_log = true;
							enter_gui_mode();			
							break;
						case GUI_READ_SENSOR:
							//printf("read sensor\r\n");
							//send_idle_signal_to_master();
							break;
						default:
							break;
					}
					break;					
				}
				if(i == (NUM_CMD_USER - 1))
				{
					printf("cmd not found\r\n");
				}
			}			
		}
		logRx_buf_idx = 0;
		memset(logRx_buf, 0, LOG_RX_SIZE);
	}
	else if(rx_log == 8) // backspace
	{		
	}
	else
	{
		logRx_buf[logRx_buf_idx++] = rx_log;
//		printf("%c", rx_log);
	}
}

void pLog(const char *funcstr, const char *format, ...)
{
	if(disable_log == false)
	{
		osMutexAcquire(LogMutexHandle, osWaitForever);

		int result = 0;
		int pos = 1;
		char *p;

		memset(log_buf, 0x20, sizeof(log_buf));
		log_buf[LOG_BUF_SIZE-1] = 0;

		//pos = snprintf(log_buf, sizeof(log_buf), "[%08ld.%03ld : ", (long)sys_tick_get_s(), (long)sys_tick_get_ms());
		log_buf[0] = '[';
		strcpy(&log_buf[1], funcstr);
		pos += strlen(funcstr);
		log_buf[pos] = ' ';
		log_buf[30] = ']';
		log_buf[31] = ' ';

		pos = 32;
		va_list list;
		va_start(list, format);
		result = vsnprintf(&log_buf[pos], sizeof(log_buf) - pos - 1, format, list);
		log_buf[sizeof(log_buf) - 1] = 0;
		va_end(list);
		if (result > 0)
		{
			p = log_buf;
			while (*p != '\0')
			{
				putchar(*p);
					p++;
			}

			putchar('\r');
			putchar('\n');
		}

		osMutexRelease(LogMutexHandle);
	}
}
//void pLog(const char *funcstr, const char *format, ...)
//{
//	if(disable_log == false)
//	{
//		osMutexAcquire(LogMutexHandle, osWaitForever);
//	    char buf[256];
//	    va_list vArgs;
//	    uint16_t lenBufSend;
//
//	    memset(buf,0,sizeof(buf));
//	    va_start(vArgs, format);
//	    lenBufSend = vsprintf(buf, format, vArgs);
//	    va_end(vArgs);
//	    HAL_UART_Transmit(&huart1,(uint8_t*)&buf,lenBufSend,10);
//	    HAL_UART_Transmit(&huart1,"\r\n",2,10);
//	    osMutexRelease(LogMutexHandle);
//	}
//}

void pBuf(char *buffer, unsigned int length, unsigned int indent)
{
	int i;

	fprintf(stdout, "\n-----------------------------------------------------------------------\n");

    if (length == 0 || buffer == NULL)
    	return;

    i = 0;
    while (i < length)
    {
        uint8_t array[16];
        int j;

        print_indent(stdout, indent);
        memcpy(array, buffer+i, 16);
        for (j = 0 ; j < 16 && i+j < length; j++)
        {
            fprintf(stdout, "%02X ", array[j]);
            if (j%4 == 3) fprintf(stdout, " ");
        }
        if (length > 16)
        {
            while (j < 16)
            {
                fprintf(stdout, "   ");
                if (j%4 == 3) fprintf(stdout, " ");
                j++;
            }
        }
        fprintf(stdout, " ");
        for (j = 0 ; j < 16 && i+j < length; j++)
        {
            if (isprint(array[j]))
                fprintf(stdout, "%c", array[j]);
            else
                fprintf(stdout, ".");
        }
        fprintf(stdout, " |\n");
        i += 16;
    }

    fprintf(stdout, "-----------------------------------------------------------------------\n");
    putchar('\n');
}

static void print_indent(FILE * stream, int num)
{
    int i;

    for ( i = 0 ; i < num ; i++)
        fprintf(stream, "    ");
}

///* Private function prototypes -----------------------------------------------*/
//#ifdef __GNUC__
///* With GCC, small printf (option LD Linker->Libraries->Small printf
//   set to 'Yes') calls __io_putchar() */
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
//
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART3 and Loop until the end of transmission */
//  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//
//  return ch;
//}


int _write(int file, char *ptr, int len)
{
#if LOG_OUT_UART
	HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 1000);
#else
	UsbCdcWrite((uint8_t *)ptr, len);
#endif
	return len;
}

