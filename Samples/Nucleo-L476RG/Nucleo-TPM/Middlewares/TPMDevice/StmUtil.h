#include <stdbool.h>

#ifndef NDEBUG
#define dbgPrint(fmt, ...) fprintf(stderr, "%s: " fmt, GetLogStamp(), ##__VA_ARGS__);
#define dbgPrintAppend(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__);
#else
#define dbgPrint(fmt, ...) ((void)0)
#define dbgPrintAppend(fmt, ...) ((void)0)
#endif
#define logError(fmt, ...) dbgPrint("[ERROR] %s (%s@%u) - " fmt, __func__, __FILE__, __LINE__, ##__VA_ARGS__);
#define logWarning(fmt, ...) dbgPrint("[WARNING] %s (%s@%u) - " fmt, __func__, __FILE__, __LINE__, ##__VA_ARGS__);
#define logInfo(fmt, ...) dbgPrint("[Info] %s (%s@%u) - " fmt, __func__, __FILE__, __LINE__, ##__VA_ARGS__);
extern char logStampStr[40];

char* GetLogStamp(void);
int BlueButtonTransitionDetected(void);
void SetDutyCycleIndicator(bool on);
void KillUSBLink(void);
void SetRealTimeClock(time_t tm);
void ReadMcuInfo(unsigned char* serial, uint16_t *flashSize, uint16_t *mcuType, uint16_t *mcuRev);
void PerformSystemReset(void);
