#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include "stm32l4xx_hal.h"
#include "usb_device.h"
#include "StmUtil.h"

// RTC initialized by MX_RTC_Init
extern RTC_HandleTypeDef hrtc;

typedef unsigned char DEVICE_UNIQUE_ID_T[12];
#define DEVICE_UNIQUE_ID (*(DEVICE_UNIQUE_ID_T*)(UID_BASE))
#define DEVICE_FLASH_SIZE (*(uint16_t *)(FLASHSIZE_BASE))
#define DEVICE_TYPE (*(uint16_t *) (DBGMCU->IDCODE & 0x00000fff))
#define DEVICE_REV (*(uint16_t *) (DBGMCU->IDCODE >> 16))
char logStampStr[40] = {0};

GPIO_PinState BlueButtonLast = GPIO_PIN_SET;
int BlueButtonTransitionDetected(void)
{
    GPIO_PinState PPButton = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
    if((PPButton == GPIO_PIN_RESET) && (BlueButtonLast == GPIO_PIN_SET))
    {
        // Now pressed
        BlueButtonLast = PPButton;
        return 1;
    }
    else if((PPButton == GPIO_PIN_SET) && (BlueButtonLast == GPIO_PIN_RESET))
    {
        // Now released
        BlueButtonLast = PPButton;
        return -1;
    }
    // No change
    return 0;
}

void SetDutyCycleIndicator(bool on)
{
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


char* GetLogStamp(void)
{
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};
    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

    sprintf(logStampStr, "%04d.%02d.%02d-%02d:%02d:%02d.%03dGMT",
                date.Year + 2000,
                date.Month,
                date.Date,
                time.Hours,
                time.Minutes,
                time.Seconds,
                (int)((1000 / time.SecondFraction) * (time.SecondFraction - time.SubSeconds)));
    return logStampStr;
}

void KillUSBLink(void)
{
    dbgPrint("USB de-initialization...\r\n");
    MX_USB_DEVICE_DeInit();
}

void SetRealTimeClock(time_t tm)
{
    struct tm* local = localtime((time_t*)&tm);
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};
    date.Year = local->tm_year - 100;
    date.Month = local->tm_mon + 1;
    date.Date = local->tm_mday;
    date.WeekDay = local->tm_wday  + 1;
    time.Hours = local->tm_hour;
    time.Minutes = local->tm_min;
    time.Seconds = local->tm_sec;
    HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
}

void ReadMcuInfo(unsigned char* serial, uint16_t *flashSize, uint16_t *mcuType, uint16_t *mcuRev)
{
    if(serial)
    {
        memcpy(serial, DEVICE_UNIQUE_ID, sizeof(DEVICE_UNIQUE_ID));
    }
    if(flashSize)
    {
        *flashSize = DEVICE_FLASH_SIZE;
    }
    if(mcuType)
    {
        *mcuType = DEVICE_TYPE;
    }
    if(mcuRev)
    {
        *mcuRev = DEVICE_REV;
    }
}

void PerformSystemReset(void)
{
    dbgPrint("Executing NVIC_SystemReset()...\r\n");
    HAL_Delay(1);
    NVIC_SystemReset();
}
