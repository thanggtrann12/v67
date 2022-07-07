#ifndef LOG_H_
#define LOG_H_

#include <stdint.h>

void parse_cmd_user(uint8_t rx_log);

#ifdef LOG_ENABLE

void pLog_Init(void);
void pLog(const char *funcstr, const char *format, ...);
void pBuf(char *buffer, unsigned int length, unsigned int indent);

// Print Log
#define PLOG_INIT()		  do { \
								pLog_Init(); \
							} while(0)

#define PLOG(msg, ...)		do { \
								pLog(__func__, msg, ##__VA_ARGS__); \
							} while(0)

#define PBUF(buf, length)	do { \
								/* pBuf(buf, length, 0); */ \
							} while(0)

#define STR_STATUS(S)                                           \
((S) == STATE_DEREGISTERED ? "STATE_DEREGISTERED" :             \
((S) == STATE_REG_PENDING ? "STATE_REG_PENDING" :               \
((S) == STATE_REGISTERED ? "STATE_REGISTERED" :                 \
((S) == STATE_REG_FAILED ? "STATE_REG_FAILED" :                 \
((S) == STATE_REG_UPDATE_PENDING ? "STATE_REG_UPDATE_PENDING" : \
((S) == STATE_REG_UPDATE_NEEDED ? "STATE_REG_UPDATE_NEEDED" :   \
((S) == STATE_REG_FULL_UPDATE_NEEDED ? "STATE_REG_FULL_UPDATE_NEEDED" :   \
((S) == STATE_DEREG_PENDING ? "STATE_DEREG_PENDING" :           \
((S) == STATE_BS_HOLD_OFF ? "STATE_BS_HOLD_OFF" :               \
((S) == STATE_BS_INITIATED ? "STATE_BS_INITIATED" :             \
((S) == STATE_BS_PENDING ? "STATE_BS_PENDING" :                 \
((S) == STATE_BS_FINISHED ? "STATE_BS_FINISHED" :               \
((S) == STATE_BS_FINISHING ? "STATE_BS_FINISHING" :             \
((S) == STATE_BS_FAILING ? "STATE_BS_FAILING" :                 \
((S) == STATE_BS_FAILED ? "STATE_BS_FAILED" :                   \
"Unknown")))))))))))))))

#define STR_MEDIA_TYPE(M)                                \
((M) == LWM2M_CONTENT_TEXT ? "LWM2M_CONTENT_TEXT" :      \
((M) == LWM2M_CONTENT_LINK ? "LWM2M_CONTENT_LINK" :      \
((M) == LWM2M_CONTENT_OPAQUE ? "LWM2M_CONTENT_OPAQUE" :  \
((M) == LWM2M_CONTENT_TLV ? "LWM2M_CONTENT_TLV" :        \
((M) == LWM2M_CONTENT_JSON ? "LWM2M_CONTENT_JSON" :      \
"Unknown")))))

#define STR_STATE(S)                                \
((S) == STATE_INITIAL ? "STATE_INITIAL" :      \
((S) == STATE_BOOTSTRAP_REQUIRED ? "STATE_BOOTSTRAP_REQUIRED" :      \
((S) == STATE_BOOTSTRAPPING ? "STATE_BOOTSTRAPPING" :  \
((S) == STATE_REGISTER_REQUIRED ? "STATE_REGISTER_REQUIRED" :        \
((S) == STATE_REGISTERING ? "STATE_REGISTERING" :      \
((S) == STATE_READY ? "STATE_READY" :      \
"Unknown"))))))

#else /* #ifdef LOG_ENABLE */

#define PLOG_INIT()
#define PLOG(msg, ...)
#define PBUF(buf, length)

#endif

#endif /* #ifndef LOG_H_ */
