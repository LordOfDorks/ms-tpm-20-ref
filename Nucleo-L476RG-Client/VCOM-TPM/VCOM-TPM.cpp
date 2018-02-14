// VCOM-TPM.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#pragma pack(push, 1)
#define SIGNALMAGIC (0x326d7054)
#define MAX_TPM_COMMAND_SIZE (1024)
#define CMD_RSP_BUFFER_SIZE (sizeof(unsigned int) + MAX_TPM_COMMAND_SIZE)
typedef enum
{
    SignalNothing = 0,
    SignalReset,
    SignalSetClock,
    // IN {UINT32 time}
    SignalCancelOn,
    SignalCancelOff,
    SignalCommand,
    // IN {BYTE Locality, UINT32 InBufferSize, BYTE[InBufferSize] InBuffer}
    // OUT {UINT32 OutBufferSize, BYTE[OutBufferSize] OutBuffer}

} signalCode_t;

typedef struct
{
    unsigned int magic;
    signalCode_t signal;
    unsigned int dataSize;
} signalHdr_t;

typedef union
{
    struct
    {
        unsigned int time;
    } SignalSetClockPayload;
    struct
    {
        unsigned int locality;
        unsigned int cmdSize;
        unsigned char cmd[1];
    } SignalCommandPayload;
} signalPayload_t, *pSignalPayload_t;

typedef union
{
    signalHdr_t s;
    unsigned char b[sizeof(signalHdr_t)];
} signalWrapper_t, *pSignalWrapper_t;
#pragma pack(pop)

HANDLE hCom = INVALID_HANDLE_VALUE;

unsigned int GetTimeStamp(void)
{
    FILETIME now = { 0 };
    LARGE_INTEGER convert = { 0 };

    // Get the current timestamp
    GetSystemTimeAsFileTime(&now);
    convert.LowPart = now.dwLowDateTime;
    convert.HighPart = now.dwHighDateTime;
    convert.QuadPart = (convert.QuadPart - (UINT64)(11644473600000 * 10000)) / 10000000;
    return convert.LowPart;
}

void OpenTpmConnection(LPCTSTR comPort)
{
    DCB dcb = { 0 };
    if (hCom != INVALID_HANDLE_VALUE)
    {
        CloseHandle(hCom);
        hCom = INVALID_HANDLE_VALUE;
    }
    dcb.DCBlength = sizeof(DCB);
    dcb.BaudRate = CBR_115200;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    if (((hCom = CreateFile(comPort, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL)) == INVALID_HANDLE_VALUE) ||
        (!SetCommState(hCom, &dcb)))
    {
        throw GetLastError();
    }
}

void SetTpmResponseTimeout(unsigned int timeout)
{
    COMMTIMEOUTS to = { 0 };
    to.ReadIntervalTimeout = 0;
    to.ReadTotalTimeoutMultiplier = 0;
    to.ReadTotalTimeoutConstant = timeout;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 0;
    if (!SetCommTimeouts(hCom, &to))
    {
        throw GetLastError();
    }
}

std::vector<BYTE> SendTpmSignal(signalCode_t signal, unsigned int timeout, std::vector<BYTE> DataIn = std::vector<BYTE>())
{
    DWORD written = 0;
    std::vector<BYTE> signalBuf(sizeof(signalWrapper_t) + DataIn.size(), 0x00);
    pSignalWrapper_t sig = (pSignalWrapper_t)signalBuf.data();
    sig->s.magic = SIGNALMAGIC;
    sig->s.signal = signal;
    sig->s.dataSize = DataIn.size();
    if (DataIn.size() > 0)
    {
        memcpy(&signalBuf[sizeof(signalWrapper_t)], DataIn.data(), DataIn.size());
    }

    SetTpmResponseTimeout(timeout);
    if (!WriteFile(hCom, signalBuf.data(), signalBuf.size(), &written, NULL))
    {
        throw GetLastError();
    }

    std::vector<BYTE> rsp;
    if (signal == SignalCommand)
    {
        DWORD read = 0;
        rsp = std::vector<BYTE>(CMD_RSP_BUFFER_SIZE, 0x00);
        if (!ReadFile(hCom, rsp.data(), rsp.size(), (LPDWORD)&read, NULL))
        {
            throw GetLastError();
        }
        unsigned int rspSize = *((unsigned int*)rsp.data());
        memcpy(rsp.data(), &rsp.data()[sizeof(unsigned int)], rspSize);
        rsp.resize(rspSize);
    }
    
    return rsp;
}

std::vector<BYTE> GenerateTpmCommandPayload(unsigned int locality, std::vector<BYTE> cmd)
{
    pSignalPayload_t payload = NULL;
    std::vector<BYTE> dataIn(sizeof(payload->SignalCommandPayload) - sizeof(unsigned char) + cmd.size(), 0x00);
    payload = (pSignalPayload_t)dataIn.data();
    payload->SignalCommandPayload.locality = locality;
    payload->SignalCommandPayload.cmdSize = cmd.size();
    memcpy(payload->SignalCommandPayload.cmd, cmd.data(), cmd.size());
    return dataIn;
}

int main()
{
    std::vector<BYTE> dataIn;
    std::vector<BYTE> dataOut;

    try
    {
        OpenTpmConnection(TEXT("COM6"));

        SendTpmSignal(SignalNothing, 1000);

        dataIn = std::vector<BYTE>(sizeof(unsigned int), 0x00);
        *((unsigned int*)dataIn.data()) = GetTimeStamp();
        SendTpmSignal(SignalSetClock, 1000, dataIn);

        SendTpmSignal(SignalCancelOn, 1000);
        SendTpmSignal(SignalCancelOff, 1000);
        //    SendTpmSignal(SignalReset, 1000);

        std::vector<BYTE> tpmCmd(1024, 0xA5);
        dataIn = GenerateTpmCommandPayload(1, tpmCmd);
        dataOut = SendTpmSignal(SignalCommand, 1000, dataIn);
    }
    catch (std::exception e)
    {
        printf("Error.");
    }

    if (hCom != INVALID_HANDLE_VALUE)
    {
        CloseHandle(hCom);
        hCom = INVALID_HANDLE_VALUE;
    }
    return 0;
}

