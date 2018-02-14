/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          :
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
/* USER CODE BEGIN INCLUDE */
#include <stdint.h>
#include <time.h>
#include "main.h"
#include "stm32l4xx_hal.h"
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
#define CDC_RTS_MASK   0x0002
#define CDC_DTR_MASK   0x0001

/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Defines
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  64
typedef struct
{
    uint8_t bReqType;
    uint8_t bRequest;
    uint16_t wVal;
    uint16_t wIndex;
    uint16_t wLength;
} USBD_SETUP_PKT, *PUSBD_SETUP_PKT;

extern RTC_HandleTypeDef hrtc;

/* USER CODE END PRIVATE_DEFINES */
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_Variables
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Send Data over USB CDC are stored in this buffer       */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
USBD_CDC_LineCodingTypeDef LineCoding =
{
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
};
volatile uint8_t CDC_RTS = 0;  // RequestToSend
volatile uint8_t CDC_DTR = 0;  // DataTerminalReady

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */
static int8_t CDC_Init_FS     (void);
static int8_t CDC_DeInit_FS   (void);
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */ 
  
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = 
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,  
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CDC_Init_FS
  *         Initializes the CDC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{ 
  /* USER CODE BEGIN 3 */ 
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */ 
}

/**
  * @brief  CDC_DeInit_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */ 
  return (USBD_OK);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CDC_Control_FS
  *         Manage the CDC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  /* USER CODE BEGIN 5 */
  char parity[] = {'N', 'O', 'E', 'M', 'S'};
  uint8_t stop[] = {1, 15, 2};
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
 
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
 
    break;

  case CDC_SET_COMM_FEATURE:
 
    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */ 
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:
  {
      LineCoding.bitrate = pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24);
      LineCoding.format = pbuf[4];
      LineCoding.paritytype = pbuf[5];
      LineCoding.datatype = pbuf[6];
      dbgPrint("CDC_SET_LINE_CODING: %lu-%d%c%d\r\n", LineCoding.bitrate, LineCoding.datatype, parity[LineCoding.paritytype], stop[LineCoding.format]);
      break;
  }

  case CDC_GET_LINE_CODING:
  {
      pbuf[0] = (uint8_t)(LineCoding.bitrate);
      pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
      pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
      pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
      pbuf[4] = LineCoding.format;
      pbuf[5] = LineCoding.paritytype;
      pbuf[6] = LineCoding.datatype;
      dbgPrint("CDC_GET_LINE_CODING: %lu-%d%c%d\r\n", LineCoding.bitrate, LineCoding.datatype, parity[LineCoding.paritytype], stop[LineCoding.format]);
      break;
  }

  case CDC_SET_CONTROL_LINE_STATE:
  {
      PUSBD_SETUP_PKT setupPkt = (PUSBD_SETUP_PKT)pbuf;
      CDC_RTS = ((setupPkt->wVal & CDC_RTS_MASK) != 0);
      CDC_DTR = ((setupPkt->wVal & CDC_DTR_MASK) != 0);
      dbgPrint("CDC_SET_CONTROL_LINE_STATE: RTS=%d, DTR=%d\r\n", CDC_RTS, CDC_DTR);
      // Reset any ongoing cmd transfers
      receivingCmd = -1;
      receivingCursor = -1;
      cmdRspSize = -1;
      memset((void*)cmdRspBuf, 0x00, sizeof(cmdRspBuf));
      break;
  }

  case CDC_SEND_BREAK:
 
    break;    
    
  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
    if(receivingCmd > 0)
    {
        memcpy((void*)&cmdRspBuf[receivingCursor], (void*)Buf, *Len);
        receivingCursor += *Len;
        dbgPrint("CommandData(%d/%d)\r\n", receivingCursor, receivingCmd);
        if(receivingCursor >= receivingCmd)
        {
            cmdRspSize = receivingCursor;
            receivingCmd = -1;
            receivingCursor = -1;
        }
    }
    else if(sizeof(signalWrapper_t) > *Len)
    {
        dbgPrint("ERROR Invalid frame received.\r\n");
        return (USBD_FAIL);
    }
    else
    {
        pSignalWrapper_t sig = (pSignalWrapper_t)Buf;
        if(sig->s.magic == SIGNALMAGIC)
        {
            pSignalPayload_t payload;
            switch(sig->s.signal)
            {
                case SignalNothing:
                    if((sig->s.dataSize != 0) || (*Len != sizeof(signalWrapper_t)))
                    {
                        dbgPrint("ERROR Invalid data size.\r\n");
                        return (USBD_FAIL);
                    }
                    dbgPrint("SignalNothing\r\n");
                    break;

                case SignalReset:
                    if((sig->s.dataSize != 0) || (*Len != sizeof(signalWrapper_t)))
                    {
                        dbgPrint("ERROR Invalid data size.\r\n");
                        return (USBD_FAIL);
                    }
                    dbgPrint("SignalReset\r\n");
                    resetRequested = 1;
                    break;

                case SignalSetClock:
                    if((sig->s.dataSize != sizeof(unsigned int)) || (*Len != sizeof(signalWrapper_t) + sizeof(unsigned int)))
                    {
                        dbgPrint("ERROR Invalid data size.\r\n");
                        return (USBD_FAIL);
                    }
                    payload = (pSignalPayload_t)&Buf[sizeof(signalWrapper_t)];

                    struct tm* local = localtime((time_t*)&payload->SignalSetClockPayload.time);
                    RTC_TimeTypeDef time = {0};
                    RTC_DateTypeDef date = {0};
                    date.Year = local->tm_year - 100;
                    date.Month = local->tm_mon + 1;
                    date.Date = local->tm_mday;
                    date.WeekDay = local->tm_wday  + 1;
                    time.Hours = local->tm_hour;
                    time.Minutes = local->tm_min;
                    time.Seconds = local->tm_sec;
                    if((HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK) ||
                       (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK))
                    {
                        dbgPrint("ERROR HAL_RTC_SetTime or HAL_RTC_SetDate failed.\r\n");
                        return (USBD_FAIL);
                    }
                    dbgPrint("SignalSetClock(0x%08x)\r\n", payload->SignalSetClockPayload.time);
                    break;

                case SignalCancelOn:
                    if((sig->s.dataSize != 0) || (*Len != sizeof(signalWrapper_t)))
                    {
                        dbgPrint("ERROR Invalid data size.\r\n");
                        return (USBD_FAIL);
                    }
                    dbgPrint("SignalCancelOn\r\n");
                    _plat__SetCancel();
                    break;

                case SignalCancelOff:
                    if((sig->s.dataSize != 0) || (*Len != sizeof(signalWrapper_t)))
                    {
                        dbgPrint("ERROR Invalid data size.\r\n");
                        return (USBD_FAIL);
                    }
                    dbgPrint("SignalCancelOff\r\n");
                    _plat__ClearCancel();
                    break;

                case SignalCommand:
                    if((sig->s.dataSize == 0) ||
                       (*Len == sizeof(signalWrapper_t)) ||
                       (cmdRspSize != -1))
                    {
                        dbgPrint("ERROR Invalid data size.\r\n");
                        return (USBD_FAIL);
                    }
                    payload = (pSignalPayload_t)&Buf[sizeof(signalWrapper_t)];
                    unsigned int expected = sizeof(signalWrapper_t) + sizeof(unsigned int) * 2 + payload->SignalCommandPayload.cmdSize;
                    unsigned int maxAllowed = sizeof(cmdRspBuf);
                    if((*Len == expected) &&
                       (payload->SignalCommandPayload.cmdSize <= maxAllowed))
                    {
                        dbgPrint("SetLocality(%d)\r\n", payload->SignalCommandPayload.locality);
                        _plat__LocalitySet(payload->SignalCommandPayload.locality);
                        memcpy((void*)cmdRspBuf, (void*)payload->SignalCommandPayload.cmd, payload->SignalCommandPayload.cmdSize);
                        cmdRspSize = payload->SignalCommandPayload.cmdSize;
                        dbgPrint("SignalCommand(%d)\r\n", payload->SignalCommandPayload.cmdSize);
                    }
                    else if((*Len < expected) &&
                            (payload->SignalCommandPayload.cmdSize <= maxAllowed))
                    {
                        dbgPrint("SetLocality(%d)\r\n", payload->SignalCommandPayload.locality);
                        _plat__LocalitySet(payload->SignalCommandPayload.locality);
                        unsigned int dataSnip = *Len - (sizeof(signalWrapper_t) + sizeof(unsigned int) * 2);
                        dbgPrint("SignalCommand(%d/%d)\r\n", dataSnip, payload->SignalCommandPayload.cmdSize);
                        memcpy((void*)cmdRspBuf, (void*)payload->SignalCommandPayload.cmd, dataSnip);
                        receivingCmd = payload->SignalCommandPayload.cmdSize;
                        receivingCursor = dataSnip;
                    }
                    else
                    {
                        dbgPrint("ERROR Invalid command size.\r\n");
                        return (USBD_FAIL);
                    }
                    break;

                default:
                    return (USBD_FAIL);
                    break;
            }
        }
    }
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (USBD_OK);
  /* USER CODE END 6 */ 
}

/**
  * @brief  CDC_Transmit_FS
  *         Data send over USB IN endpoint are sent over CDC interface 
  *         through this function.           
  *         @note
  *         
  *                 
  * @param  Buf: Buffer of data to be send
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */ 
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  dbgPrint("ResponseData(%d)\r\n", Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */ 
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
