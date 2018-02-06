#include <stdio.h>
#include <stdint.h>
#include <tchar.h>
#include <Windows.h>

#include "FPLib.h"
#include "CompilerDependencies.h"
#include "Platform_fp.h"

COMMAND_INFO cmdInfo[] = {
{FPR_COMMAND_OPEN, 0, sizeof(DEV_INFO_FPR)},
{FPR_COMMAND_CLOSE, 0, 0},
{FPR_COMMAND_USBINTERNALCHECK, 0, 0},
{FPR_COMMAND_CHANGEBAUDRATE,0 ,0},
{FPR_COMMAND_SETIAPMODE, 0, 0},
{FPR_COMMAND_CMOSLED, 0, 0 },
{FPR_COMMAND_GETENROLLCOUNT, 0, 0 },
{FPR_COMMAND_CHECKENROLLED, 0, 0 },
{FPR_COMMAND_ENROLLSTART, 0, 0 },
{FPR_COMMAND_ENROLL1, 0, 0 },
{FPR_COMMAND_ENROLL2, 0, 0 },
{FPR_COMMAND_ENROLL3, 0, FP_TEMPLATE_SIZE },
{FPR_COMMAND_ISPRESSFINGER, 0, 0 },
{FPR_COMMAND_DELETEID, 0, 0 },
{FPR_COMMAND_DELETEALL, 0, 0 },
{FPR_COMMAND_VERIFY, 0, 0 },
{FPR_COMMAND_IDENTIFY, 0, 0 },
{FPR_COMMAND_VERIFYTEMPLATE, FP_TEMPLATE_SIZE, 0 },
{FPR_COMMAND_IDENTIFYTEMPLATE, FP_TEMPLATE_SIZE, 0 },
{FPR_COMMAND_CAPTUREFINGER, 0, 0 },
{FPR_COMMAND_MAKETEMPLATE, 0, FP_TEMPLATE_SIZE },
{FPR_COMMAND_GETIMAGE, 0, FP_IMAGE_SIZE },
{FPR_COMMAND_GETRAWIMAGE, 0, FP_RAW_IMAGE_SIZE },
{FPR_COMMAND_GETTEMPLATE, 0, FP_TEMPLATE_SIZE },
{FPR_COMMAND_SETTEMPLATE, FP_TEMPLATE_SIZE, 0 },
{FPR_COMMAND_GETDATABASESTART, 0, 0 },
{FPR_COMMAND_GETDATABASEEND, 0, 0 },
{FPR_COMMAND_SETSECURITYLEVEL, 0, 0 },
{FPR_COMMAND_GETSECURITYLEVEL, 0, 0 },
{FPR_COMMAND_INVALID, 0, 0}
};

HANDLE hFPR = INVALID_HANDLE_VALUE;
unsigned short fpTimeout = 30000;
char displayText[FP_DISPLAY_MAX_TEXT + 1] = { 0 };
unsigned int displayTextLen = 0;

FPR_ERROR_CODE InitializeFPR(char* port, char re_init)
{
    FPR_ERROR_CODE result = FPR_ERROR_ACK_SUCCESS;
    DEV_INFO_FPR devInfo = { 0 };

    if (hFPR != INVALID_HANDLE_VALUE)
    {
        if (!re_init)
        {
            goto Cleanup;
        }
        FPR_Close();
        CloseHandle(hFPR);
        hFPR = INVALID_HANDLE_VALUE;
    }

    if ((hFPR = CreateFileA(port, GENERIC_READ || GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL)) == INVALID_HANDLE_VALUE)
    {
        result = FPR_ERROR_NACK_COMM_ERR;
        goto Cleanup;
    }
    if (!PurgeComm(hFPR, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR))
    {
        result = FPR_ERROR_NACK_COMM_ERR;
        goto Cleanup;
    }
    DCB dcb;
    memset(&dcb, 0, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);
    dcb.BaudRate = CBR_9600;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fAbortOnError = TRUE;
    if (!SetCommState(hFPR, &dcb))
    {
        result = FPR_ERROR_NACK_COMM_ERR;
        goto Cleanup;
    }
    COMMTIMEOUTS to;
    to.ReadIntervalTimeout = 0;
    to.ReadTotalTimeoutMultiplier = 0;
    to.ReadTotalTimeoutConstant = 16;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 16;
    if (!SetCommTimeouts(hFPR, &to))
    {
        result = FPR_ERROR_NACK_COMM_ERR;
        goto Cleanup;
    }
    result = FPR_ChangeBaudrate(115200);
    if ((result != FPR_ERROR_ACK_SUCCESS) && (result != FPR_ERROR_NACK_TIMEOUT))
    {
        goto Cleanup;
    }
    result = FPR_ERROR_ACK_SUCCESS;
    dcb.BaudRate = CBR_115200;
    if (!SetCommState(hFPR, &dcb))
    {
        result = FPR_ERROR_NACK_COMM_ERR;
        goto Cleanup;

    }
    if ((result = FPR_Open(&devInfo)) != FPR_ERROR_ACK_SUCCESS)
    {
        result = FPR_ERROR_NACK_DEV_ERR;
        goto Cleanup;
    }
    if ((result = FPR_CMOSLEDControl(0)) != FPR_ERROR_ACK_SUCCESS)
    {
        goto Cleanup;
    }

Cleanup:
    if (result != FPR_ERROR_ACK_SUCCESS)
    {
        FPR_Close();
        CloseHandle(hFPR);
        hFPR = INVALID_HANDLE_VALUE;
    }
    return result;
}

unsigned char ReadCharFPR(int* timeout)
{
    unsigned int startTime = GetTickCount();
    unsigned char data = 0;
    unsigned int read = 0;
    COMMTIMEOUTS to;
    to.ReadIntervalTimeout = 0;
    to.ReadTotalTimeoutMultiplier = 0;
    to.ReadTotalTimeoutConstant = (unsigned int)*timeout;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 16;
    if (!SetCommTimeouts(hFPR, &to))
    {
        goto Cleanup;
    }
    if (!ReadFile(hFPR, &data, 1, (LPDWORD)&read, NULL))
    {
        goto Cleanup;
    }

Cleanup:
    *timeout -= GetTickCount() - startTime;
    return data;
}

void WriteCharFPR(unsigned char data, int* timeout)
{
    unsigned int startTime = GetTickCount();
    TransmitCommChar(hFPR, data);
    *timeout -= GetTickCount() - startTime;
}

void CloseFPR(void)
{
    CloseHandle(hFPR);
    hFPR = INVALID_HANDLE_VALUE;
}

static COMMAND_INFO* GetCommandInfo(
    FPR_COMMAND_CODE cmd)
{
    for (unsigned int n = 0; cmdInfo[n].cmd != FPR_COMMAND_INVALID; n++)
    {
        if (cmdInfo[n].cmd == cmd) return &cmdInfo[n];
    }
    return 0;
}

static unsigned short CheckSum(
    unsigned short sumIn,
    unsigned char* data,
    unsigned int len)
{
    unsigned short sum = sumIn;
    for (unsigned int n = 0; n < len; n++) sum += data[n];
    return sum;
}

static FPR_ERROR_CODE Execute(
    FPR_COMMAND_CODE cmd,
    unsigned int parameterIn,
    unsigned int* parameterOut,
    unsigned char* dataPktIn,
    unsigned int dataPktInSize,
    unsigned char* dataPktOut,
    unsigned int dataPktOutSize,
    int* timeout)
{
    FPR_ERROR_CODE result = FPR_ERROR_ACK_SUCCESS;
    COMMAND_INFO* info = GetCommandInfo(cmd);
    FPR_COMMAND_RESPONSE_PACKET_T in;
    FPR_COMMAND_RESPONSE_PACKET_T out;

    // check valid command
    if (info == 0)
    {
        result = FPR_ERROR_NACK_INVALID_PARAM;
        goto Cleanup;
    }

    // Send the command out
    in.s.startCode1 = 0x55;
    in.s.startCode2 = 0xAA;
    in.s.deviceId = 0x0001;
    in.s.parameter = parameterIn;
    in.s.cmd_rsp = (unsigned short)cmd;
    in.s.checkSum = CheckSum(0, (unsigned char*)&in, sizeof(in) - sizeof(in.s.checkSum));
    for (unsigned int n = 0; n < sizeof(in); n++)
    {
        if (*timeout <= 0)
        {
            result = FPR_ERROR_NACK_TIMEOUT;
            goto Cleanup;
        }
        WriteCharFPR(in.raw[n], timeout);
    }

    // Retrive the response
    for (unsigned int n = 0; n < sizeof(out); n++)
    {
        if (*timeout <= 0)
        {
            result = FPR_ERROR_NACK_TIMEOUT;
            goto Cleanup;
        }
        out.raw[n] = ReadCharFPR(timeout);
    }
    if ((out.s.startCode1 != 0x55) ||
        (out.s.startCode2 != 0xAA) ||
        (out.s.deviceId != 0x0001) ||
        (out.s.checkSum != CheckSum(0, (unsigned char*)&out, sizeof(out) - sizeof(out.s.checkSum))))
    {
        result = FPR_ERROR_NACK_COMM_ERR;
        goto Cleanup;
    }
    if (out.s.cmd_rsp == (unsigned short)FPR_COMMAND_NACK)
    {
        result = out.s.parameter;
        goto Cleanup;
    }
    if (parameterOut != 0)
    {
        *parameterOut = out.s.parameter;
    }

    // Send Data Packet
    if (info->dataIn > 0)
    {
        if((dataPktInSize != info->dataIn) || (dataPktIn == 0))
        {
            result = FPR_ERROR_NACK_INVALID_PARAM;
            goto Cleanup;
        }
        in.s.startCode1 = 0x5A;
        in.s.startCode2 = 0xA5;
        in.s.deviceId = 0x0001;
        in.s.checkSum = CheckSum(0, (unsigned char*)&in, sizeof(in.s.startCode1) + sizeof(in.s.startCode2) + sizeof(in.s.deviceId));
        in.s.checkSum = CheckSum(in.s.checkSum, dataPktIn, dataPktInSize);
        for (unsigned int n = 0; n < sizeof(in.s.startCode1) + sizeof(in.s.startCode2) + sizeof(in.s.deviceId); n++)
        {
            if (*timeout <= 0)
            {
                result = FPR_ERROR_NACK_TIMEOUT;
                goto Cleanup;
            }
            WriteCharFPR(in.raw[n], timeout);
        }
        for (unsigned int n = 0; n < dataPktInSize; n++)
        {
            if (*timeout <= 0)
            {
                result = FPR_ERROR_NACK_TIMEOUT;
                goto Cleanup;
            }
            WriteCharFPR(dataPktIn[n], timeout);
        }
        for (unsigned int n = sizeof(in) - sizeof(in.s.checkSum); n < sizeof(in); n++)
        {
            if (*timeout <= 0)
            {
                result = FPR_ERROR_NACK_TIMEOUT;
                goto Cleanup;
            }
            WriteCharFPR(in.raw[n], timeout);
        }

        // Retrive the data packet response
        for (unsigned int n = 0; n < sizeof(out); n++)
        {
            if (*timeout <= 0)
            {
                result = FPR_ERROR_NACK_TIMEOUT;
                goto Cleanup;
            }
            out.raw[n] = ReadCharFPR(timeout);
        }
        if ((out.s.startCode1 != 0x55) ||
            (out.s.startCode2 != 0xAA) ||
            (out.s.deviceId != 0x0001) ||
            (out.s.checkSum != CheckSum(0, (unsigned char*)&out, sizeof(out) - sizeof(out.s.checkSum))))
        {
            result = FPR_ERROR_NACK_COMM_ERR;
            goto Cleanup;
        }
        if (out.s.cmd_rsp == (unsigned short)FPR_COMMAND_NACK)
        {
            result = out.s.parameter;
            goto Cleanup;
        }
        if (parameterOut != 0)
        {
            *parameterOut = out.s.parameter;
        }
    }

    // Recive Data Packet
    if ((info->dataOut > 0) && (dataPktOut > 0) && (dataPktOut != 0))
    {
        for (unsigned int n = 0; n < sizeof(out.s.startCode1) + sizeof(out.s.startCode2) + sizeof(out.s.deviceId); n++)
        {
            if (*timeout <= 0)
            {
                result = FPR_ERROR_NACK_TIMEOUT;
                goto Cleanup;
            }
            out.raw[n] = ReadCharFPR(timeout);
        }
        for (unsigned int n = 0; n < dataPktOutSize; n++)
        {
            if (*timeout <= 0)
            {
                result = FPR_ERROR_NACK_TIMEOUT;
                goto Cleanup;
            }
            dataPktOut[n] = ReadCharFPR(timeout);

        }
        for (unsigned int n = sizeof(out) - sizeof(out.s.checkSum); n < sizeof(out); n++)
        {
            if (*timeout <= 0)
            {
                result = FPR_ERROR_NACK_TIMEOUT;
                goto Cleanup;
            }
            out.raw[n] = ReadCharFPR(timeout);
        }
        if ((out.s.startCode1 != 0x5A) ||
            (out.s.startCode2 != 0xA5) ||
            (out.s.deviceId != 0x0001) ||
            (out.s.checkSum != CheckSum(CheckSum(0, (unsigned char*)&out, sizeof(out.s.startCode1) + sizeof(out.s.startCode2) + sizeof(out.s.deviceId)), dataPktOut, dataPktOutSize)))
        {
            result = FPR_ERROR_NACK_COMM_ERR;
            goto Cleanup;
        }
    }

Cleanup:
    return result;
}

FPR_ERROR_CODE FPR_Open(DEV_INFO_FPR* info)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_OPEN, (info ? 1 : 0), 0, 0, 0, (unsigned char*)info, sizeof(DEV_INFO_FPR), &timeout);
}

FPR_ERROR_CODE FPR_Close(void)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_CLOSE, 0, 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_CMOSLEDControl(unsigned int on)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_CMOSLED, on, 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_ChangeBaudrate(unsigned int baudrate)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_CHANGEBAUDRATE, baudrate, 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_GetEnrollCount(unsigned int* count)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_GETENROLLCOUNT, 0, count, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_CheckEnrolled(unsigned int id)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_CHECKENROLLED, id, 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_EnrollStart(unsigned int id, unsigned char noDupChk, unsigned char noSave)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_ENROLLSTART, (noSave ? -1 : (noDupChk ? 0x80000000 : 0) | (0x0000ffff & id)), 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_Enroll(unsigned int no, unsigned char* fpTemplate)
{
    int timeout = 1000;
    return Execute((no == 1 ? FPR_COMMAND_ENROLL1 : (no == 2 ? FPR_COMMAND_ENROLL2 : FPR_COMMAND_ENROLL3)), 0, 0, 0, 0, fpTemplate, (fpTemplate ? FP_TEMPLATE_SIZE : 0), &timeout);
}

FPR_ERROR_CODE FPR_IsPressFinger(void)
{
    unsigned int parameter;
    int timeout = FP_DEFAULT_TIMEOUT;
    FPR_ERROR_CODE result = Execute(FPR_COMMAND_ISPRESSFINGER, 0, &parameter, 0, 0, 0, 0, &timeout);
    return (result == FPR_ERROR_ACK_SUCCESS) ? parameter : result;
}

FPR_ERROR_CODE FPR_DeleteId(unsigned int id)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_DELETEID, id, 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_DeleteAll(void)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_DELETEALL, 0, 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_Verify(unsigned int id)
{
    int timeout = 1000;
    return Execute(FPR_COMMAND_VERIFY, id, 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_Identify(unsigned int* id)
{
    int timeout = 1000;
    return Execute(FPR_COMMAND_IDENTIFY, 0, id, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_VerifyTemplate(unsigned int id, unsigned char* fpTemplate)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_VERIFYTEMPLATE, id, 0, fpTemplate, FP_TEMPLATE_SIZE, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_IdentifyTemplate(unsigned int* id, unsigned char* fpTemplate)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_IDENTIFYTEMPLATE, 0, id, fpTemplate, FP_TEMPLATE_SIZE, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_CaptureFinger(unsigned char bestImage)
{
    int timeout = 250;
    return Execute(FPR_COMMAND_CAPTUREFINGER, bestImage, 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_MakeTemplate(unsigned char* fpTemplate)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_MAKETEMPLATE, 0, 0, 0, 0, fpTemplate, FP_TEMPLATE_SIZE, &timeout);
}

FPR_ERROR_CODE FPR_GetImage(unsigned char* fpImage)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_GETIMAGE, 0, 0, 0, 0, fpImage, FP_IMAGE_SIZE, &timeout);
}

FPR_ERROR_CODE FPR_GetRawImage(unsigned char* fpRawImage)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_GETRAWIMAGE, 0, 0, 0, 0, fpRawImage, FP_RAW_IMAGE_SIZE, &timeout);
}

FPR_ERROR_CODE FPR_GetTemplate(unsigned int id, unsigned char* fpTemplate)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_GETTEMPLATE, id, 0, 0, 0, fpTemplate, FP_TEMPLATE_SIZE, &timeout);
}

FPR_ERROR_CODE FPR_SetTemplate(unsigned int id, unsigned char* fpTemplate)
{
    int timeout = 1000;
    return Execute(FPR_COMMAND_SETTEMPLATE, id, 0, fpTemplate, FP_TEMPLATE_SIZE, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_SetIAPMode(void)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_SETIAPMODE, 0, 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_SetSecurityLevel(unsigned int level)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_SETSECURITYLEVEL, level, 0, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_GetSecurityLevel(unsigned int* level)
{
    int timeout = FP_DEFAULT_TIMEOUT;
    return Execute(FPR_COMMAND_GETSECURITYLEVEL, 0, level, 0, 0, 0, 0, &timeout);
}

FPR_ERROR_CODE FPR_WaitForFinger(unsigned char pressed, unsigned int cycles)
{
    FPR_ERROR_CODE result = FPR_ERROR_ACK_SUCCESS;
    if ((result = FPR_CMOSLEDControl(1)) != FPR_ERROR_ACK_SUCCESS)
    {
        goto Cleanup;
    }
    do
    {
        if (cycles == 0)
        {
            result = FPR_ERROR_NACK_TIMEOUT;
            goto Cleanup;
        }
        result = FPR_IsPressFinger();
        if((result != FPR_ERROR_ACK_SUCCESS) && (result != FPR_ERROR_NACK_FINGER_IS_NOT_PRESSED))
        {
            goto Cleanup;
        }
        if(cycles < 0xffffffff) cycles--;
    } while (((result == FPR_ERROR_ACK_SUCCESS) && (!pressed)) ||
             ((result == FPR_ERROR_NACK_FINGER_IS_NOT_PRESSED) && (pressed)));
    result = FPR_ERROR_ACK_SUCCESS;
Cleanup:
    return result;
}

FPR_ERROR_CODE FPR_EnrollFinger(FPR_STATE_MACHINE* state, unsigned int id, unsigned char noDupChk, unsigned char noSave, unsigned char* fpTemplate)
{
    FPR_ERROR_CODE result = FPR_ERROR_ACK_SUCCESS;

    if (*state == FPR_STATE_MACHINE_START)
    {
        if ((result = FPR_EnrollStart(id, noDupChk, noSave)) != FPR_ERROR_ACK_SUCCESS)
        {
            goto Cleanup;
        }
        if ((result = FPR_CMOSLEDControl(1)) != FPR_ERROR_ACK_SUCCESS)
        {
            goto Cleanup;
        }
        *state = FPR_STATE_MACHINE_ENROLL_FIRST_PRESS;
    }

Restart:
    if ((*state == FPR_STATE_MACHINE_ENROLL_FIRST_PRESS) ||
        (*state == FPR_STATE_MACHINE_ENROLL_SECOND_PRESS) ||
        (*state == FPR_STATE_MACHINE_ENROLL_THIRD_PRESS))
    {
        result = FPR_IsPressFinger();
        if (result == FPR_ERROR_ACK_SUCCESS)
        {
            *state += 1;
        }
        else if (result == FPR_ERROR_NACK_FINGER_IS_NOT_PRESSED)
        {
            result = FPR_ERROR_ACK_SUCCESS;
            goto Cleanup;
        }
        else
        {
            goto Cleanup;
        }
    }

    if ((*state == FPR_STATE_MACHINE_ENROLL_FIRST_SCAN) ||
        (*state == FPR_STATE_MACHINE_ENROLL_SECOND_SCAN) ||
        (*state == FPR_STATE_MACHINE_ENROLL_THIRD_SCAN))
    {
        unsigned int no = (*state == FPR_STATE_MACHINE_ENROLL_FIRST_SCAN ? 1 : (*state == FPR_STATE_MACHINE_ENROLL_SECOND_SCAN ? 2 : 3));

        if ((result = FPR_CaptureFinger(1)) != FPR_ERROR_ACK_SUCCESS)
        {
            goto Cleanup;
        }
        if ((result = FPR_Enroll(no, (*state == FPR_STATE_MACHINE_ENROLL_THIRD_SCAN ? fpTemplate : 0))) != FPR_ERROR_ACK_SUCCESS)
        {
            goto Cleanup;
        }
        if ((result = FPR_CMOSLEDControl(0)) != FPR_ERROR_ACK_SUCCESS)
        {
            goto Cleanup;
        }
        if (*state == FPR_STATE_MACHINE_ENROLL_THIRD_SCAN)
        {
            *state = FPR_STATE_MACHINE_END;
            goto Cleanup;
        }
        else
        {
            *state += 1;
        }
        goto Cleanup;
    }

    if (((*state >= FPR_STATE_MACHINE_ENROLL_FIRST_LIGHT_OFF) && (*state < FPR_STATE_MACHINE_ENROLL_FIRST_LIGHT_ON)) ||
        ((*state >= FPR_STATE_MACHINE_ENROLL_SECOND_LIGHT_OFF) && (*state < FPR_STATE_MACHINE_ENROLL_SECOND_LIGHT_ON)))
    {
        *state += 1;
        goto Cleanup;
    }

    if ((*state == FPR_STATE_MACHINE_ENROLL_FIRST_LIGHT_ON) ||
        (*state == FPR_STATE_MACHINE_ENROLL_SECOND_LIGHT_ON))
    {
        if ((result = FPR_CMOSLEDControl(1)) != FPR_ERROR_ACK_SUCCESS)
        {
            goto Cleanup;
        }
        *state += 1;
    }

    if ((*state == FPR_STATE_MACHINE_ENROLL_FIRST_REMOVED) ||
        (*state == FPR_STATE_MACHINE_ENROLL_SECOND_REMOVED))
    {
        result = FPR_IsPressFinger();
        if (result == FPR_ERROR_NACK_FINGER_IS_NOT_PRESSED)
        {
            result = FPR_ERROR_ACK_SUCCESS;
            *state += 1;
            goto Restart;
        }
        else
        {
            goto Cleanup;
        }
    }

Cleanup:
    return result;
}

FPR_ERROR_CODE FPR_VerifyFinger(FPR_STATE_MACHINE* state, unsigned int id)
{
    FPR_ERROR_CODE result = FPR_ERROR_ACK_SUCCESS;

    if (*state == FPR_STATE_MACHINE_START)
    {
        if ((result = FPR_CMOSLEDControl(1)) != FPR_ERROR_ACK_SUCCESS)
        {
            goto Cleanup;
        }
        *state = FPR_STATE_MACHINE_VERIFY_PRESS;
    }

    if (*state == FPR_STATE_MACHINE_VERIFY_PRESS)
    {
        result = FPR_IsPressFinger();
        if (result == FPR_ERROR_ACK_SUCCESS)
        {
            *state += 1;
        }
        else if (result == FPR_ERROR_NACK_FINGER_IS_NOT_PRESSED)
        {
            result = FPR_ERROR_ACK_SUCCESS;
            goto Cleanup;
        }
        else
        {
            goto Cleanup;
        }
    }

    if (*state == FPR_STATE_MACHINE_VERIFY_SCAN)
    {
        if ((result = FPR_CaptureFinger(1)) != FPR_ERROR_ACK_SUCCESS)
        {
            goto Cleanup;
        }
        result = FPR_Verify(id);
        FPR_CMOSLEDControl(0);
        if (result != FPR_ERROR_ACK_SUCCESS)
        {
            goto Cleanup;
        }
        
        *state = FPR_STATE_MACHINE_END;
        goto Cleanup;
    }

Cleanup:
    return result;
}

FPR_ERROR_CODE FPR_IdentifyFinger(FPR_STATE_MACHINE* state, unsigned int* id)
{
    FPR_ERROR_CODE result = FPR_ERROR_ACK_SUCCESS;

    if (*state == FPR_STATE_MACHINE_START)
    {
        if ((result = FPR_CMOSLEDControl(1)) != FPR_ERROR_ACK_SUCCESS)
        {
            goto Cleanup;
        }
        *state = FPR_STATE_MACHINE_IDENTIFY_PRESS;
    }

    if (*state == FPR_STATE_MACHINE_IDENTIFY_PRESS)
    {
        result = FPR_IsPressFinger();
        if (result == FPR_ERROR_ACK_SUCCESS)
        {
            *state += 1;
        }
        else if (result == FPR_ERROR_NACK_FINGER_IS_NOT_PRESSED)
        {
            result = FPR_ERROR_ACK_SUCCESS;
            goto Cleanup;
        }
        else
        {
            goto Cleanup;
        }
    }

    if (*state == FPR_STATE_MACHINE_IDENTIFY_SCAN)
    {
        result = FPR_CaptureFinger(1);
        if (result == FPR_ERROR_ACK_SUCCESS)
        {
            result = FPR_Identify(id);
            FPR_CMOSLEDControl(0);
        }
        else if (result == FPR_ERROR_NACK_FINGER_IS_NOT_PRESSED)
        {
            *state = FPR_STATE_MACHINE_IDENTIFY_PRESS;
            result = FPR_ERROR_ACK_SUCCESS;
        }
        else
        {
            goto Cleanup;
        }

        *state = FPR_STATE_MACHINE_END;
        goto Cleanup;
    }

Cleanup:
    return result;
}

FPR_ERROR_CODE FPR_TPMWrite(UINT32 index, UINT8* data, UINT32 size, UINT32 offset)
{
    FPR_ERROR_CODE result = FPR_ERROR_NACK_INVALID_PARAM;
    FPR_STATE_MACHINE state = FPR_STATE_MACHINE_START;

    if ((result = InitializeFPR("COM5", 0)) != FPR_ERROR_ACK_SUCCESS)
    {
        goto Cleanup;
    }

    if (index < FP_AUTHORIZE_INDEX)
    {
        unsigned int fpSlot = index - NV_FPBASE_INDEX;
        if (size == sizeof(unsigned char))
        {
            if (data[0] == FP_SLOT_INITIALIZE_TEMPLATE)
            {
                // NOOP
                result = FPR_ERROR_ACK_SUCCESS;
            }
            else if (data[0] == FP_SLOT_DELETE_ALL_TEMPLATE)
            {
                // Delete All
                result = FPR_DeleteAll();
                if ((result != FPR_ERROR_ACK_SUCCESS) && (result != FPR_ERROR_NACK_DB_IS_EMPTY))
                {
                    //printf("FPR_DeleteAll() returned 0x%08x\n", result);
                    goto Cleanup;
                }
                else
                {
                    result = FPR_ERROR_ACK_SUCCESS;
                }
            }
            else if (data[0] == FP_SLOT_DELETE_TEMPLATE)
            {
                // Delete
                result = FPR_DeleteId(fpSlot);
                if ((result != FPR_ERROR_ACK_SUCCESS) && (result != FPR_ERROR_NACK_IS_NOT_USED))
                {
                    //printf("FPR_DeleteId() returned 0x%08x\n", result);
                    goto Cleanup;
                }
                else
                {
                    result = FPR_ERROR_ACK_SUCCESS;
                }
            }
            else if (data[0] == FP_SLOT_ENROLL_TEMPLATE)
            {
                unsigned int deadline = GetTickCount() + fpTimeout;
                unsigned int secsLeft = fpTimeout / 1000;
                if ((result = FPR_CheckEnrolled(fpSlot)) == FPR_ERROR_ACK_SUCCESS)
                {
                    result = FPR_DeleteId(fpSlot);
                }
                // Enroll
                state = FPR_STATE_MACHINE_START;
                do
                {
                    if ((result = FPR_EnrollFinger(&state, fpSlot, 0, 0, 0)) != FPR_ERROR_ACK_SUCCESS)
                    {
                        printf("FPR_EnrollFinger() in state %d returned 0x%08x\n", state, result);
                        goto Cleanup;
                    }
                    if (state != FPR_STATE_MACHINE_END)
                    {
                        Sleep(1);
                        if (secsLeft > ((deadline - GetTickCount()) / 1000))
                        {
                            secsLeft--;
                            printf("[TrustedDisplay]: %s (%d sec)                  \r", displayText, secsLeft);
                        }
                    }
                    if (deadline <= GetTickCount())
                    {
                        result = FPR_ERROR_NACK_TIMEOUT;
                        state = FPR_STATE_MACHINE_END;
                        //printf("FPR_EnrollFinger() timed out.\n");
                        goto Cleanup;
                    }
                    if (_plat__IsCanceled())
                    {
                        result = FPR_ERROR_NACK_CAPTURE_CANCELED;
                        state = FPR_STATE_MACHINE_END;
                        //printf("FPR_IdentifyFinger() canceled.\n");
                        goto Cleanup;
                    }
                } while (state != FPR_STATE_MACHINE_END);
                printf("[TrustedDisplay]: %s                  \r", displayText);
            }
        }
        else if (size == FP_TEMPLATE_SIZE)
        {
            if ((result = FPR_CheckEnrolled(fpSlot)) == FPR_ERROR_ACK_SUCCESS)
            {
                result = FPR_DeleteId(fpSlot);
            }
            // Template Write
            result = FPR_SetTemplate(fpSlot, data);
            if (result != FPR_ERROR_ACK_SUCCESS)
            {
                //printf("FPR_SetTemplate() returned 0x%08x\n", result);
                goto Cleanup;
            }
            else
            {
                result = FPR_ERROR_ACK_SUCCESS;
            }
        }
    }
    else if((index == FP_AUTHORIZE_INDEX) && (size == sizeof(unsigned int)))
    {
        if (data[3] == FP_AUTHORIZE_INITIALIZE)
        {
            // NOOP
            result = FPR_ERROR_ACK_SUCCESS;
        }
        else if (data[3] == FP_AUTHORIZE_TIMEOUT)
        {
            fpTimeout = (unsigned short)(*((unsigned int*)data) & 0x0000ffff);
        }
        else if (data[3] == FP_AUTHORIZE_VERIFY)
        {
            // Verify
            unsigned int verify = *((unsigned int*)data) & 0x00ffffff;
            unsigned int deadline = GetTickCount() + fpTimeout;
            unsigned int secsLeft = fpTimeout / 1000;
            state = FPR_STATE_MACHINE_START;
            do
            {
                result = FPR_VerifyFinger(&state, verify);
                if ((result != FPR_ERROR_ACK_SUCCESS) && (result != FPR_ERROR_NACK_VERIFY_FAILED))
                {
                    printf("\nFPR_VerifyFinger() in state %d returned 0x%08x\n", state, result);
                    goto Cleanup;
                }
                if (state != FPR_STATE_MACHINE_END)
                {
                    Sleep(1);
                    if (secsLeft > ((deadline - GetTickCount()) / 1000))
                    {
                        secsLeft--;
                        printf("[TrustedDisplay]: %s (%d sec)                  \r", displayText, secsLeft);
                    }
                }
                if (deadline <= GetTickCount())
                {
                    result = FPR_ERROR_NACK_TIMEOUT;
                    state = FPR_STATE_MACHINE_END;
                    //printf("FPR_VerifyFinger() timed out.\n");
                    goto Cleanup;
                }
                if (_plat__IsCanceled())
                {
                    result = FPR_ERROR_NACK_CAPTURE_CANCELED;
                    state = FPR_STATE_MACHINE_END;
                    //printf("FPR_IdentifyFinger() canceled.\n");
                    goto Cleanup;
                }
            } while (state != FPR_STATE_MACHINE_END);
            printf("[TrustedDisplay]: %s                  \r", displayText);
        }
    }

Cleanup:
    FPR_CMOSLEDControl(0);
    return result;
}

FPR_ERROR_CODE FPR_TPMRead(UINT32 index, UINT8* data, UINT32 size, UINT32 offset)
{
    FPR_ERROR_CODE result = FPR_ERROR_NACK_INVALID_PARAM;
    FPR_STATE_MACHINE state = FPR_STATE_MACHINE_START;

    if ((result = InitializeFPR("COM5", 0)) != FPR_ERROR_ACK_SUCCESS)
    {
        goto Cleanup;
    }

    if ((index < FP_AUTHORIZE_INDEX) && (size == FP_TEMPLATE_SIZE))
    {
        unsigned int fpSlot = index - NV_FPBASE_INDEX;
        // Template read
        result = FPR_GetTemplate(fpSlot, data);
        if ((result != FPR_ERROR_ACK_SUCCESS) &&
            (result != FPR_ERROR_NACK_IS_NOT_USED) &&
            (result != FPR_ERROR_NACK_DB_IS_EMPTY))
        {
            //printf("FPR_GetTemplate() returned 0x%08x\n", result);
            goto Cleanup;
        }
    }
    else if ((index == FP_AUTHORIZE_INDEX) && (size == sizeof(unsigned int)))
    {
        // Identify
        unsigned int* identify = (unsigned int*)data;
        unsigned int deadline = GetTickCount() + fpTimeout;
        unsigned int secsLeft = fpTimeout / 1000;
        state = FPR_STATE_MACHINE_START;
        do
        {
            result = FPR_IdentifyFinger(&state, identify);
            if (result == FPR_ERROR_NACK_IDENTIFY_FAILED)
            {
                *identify = (unsigned int)-1;
                result = FPR_ERROR_ACK_SUCCESS;
                goto Cleanup;
            }
            else if (result != FPR_ERROR_ACK_SUCCESS)
            {
                printf("FPR_IdentifyFinger() in state %d returned 0x%08x\n", state, result);
                goto Cleanup;
            }
            if (state != FPR_STATE_MACHINE_END)
            {
                Sleep(1);
                if (secsLeft > ((deadline - GetTickCount()) / 1000))
                {
                    secsLeft--;
                    printf("[TrustedDisplay]: %s (%d sec)                  \r", displayText, secsLeft);
                }
            }
            if (deadline <= GetTickCount())
            {
                result = FPR_ERROR_NACK_TIMEOUT;
                state = FPR_STATE_MACHINE_END;
                //printf("FPR_IdentifyFinger() timed out.\n");
                goto Cleanup;
            }
            if (_plat__IsCanceled())
            {
                result = FPR_ERROR_NACK_CAPTURE_CANCELED;
                state = FPR_STATE_MACHINE_END;
                //printf("FPR_IdentifyFinger() canceled.\n");
                goto Cleanup;
            }
        } while (state != FPR_STATE_MACHINE_END);
        printf("[TrustedDisplay]: %s                  \r", displayText);
    }

Cleanup:
    FPR_CMOSLEDControl(0);
    return result;
}

FPR_ERROR_CODE TDisp_Write(UINT8* data, UINT32 size, UINT32 offset)
{
    if ((offset + size) >= FP_DISPLAY_MAX_TEXT)
    {
        return FPR_ERROR_NACK_INVALID_PARAM;
    }
    if (size > 0)
    {
        memcpy(&displayText[offset], data, size);
        displayText[FP_DISPLAY_MAX_TEXT] = 0x00;
        printf("[TrustedDisplay]: %s                  \r", displayText);
    }
    else
    {
        memset(displayText, 0x00, sizeof(displayText));
        printf("\n");
    }

    return FPR_ERROR_ACK_SUCCESS;
}

FPR_ERROR_CODE TDisp_Read(UINT8* data, UINT32 size, UINT32 offset)
{
    if ((offset + size) >= FP_DISPLAY_MAX_TEXT)
    {
        return FPR_ERROR_NACK_INVALID_PARAM;
    }
    memcpy(data, &displayText[offset], size);
    return FPR_ERROR_ACK_SUCCESS;
}
