#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include "StmUtil.h"
#include <wolfssl/wolfcrypt/sha512.h>
#undef INLINE
#include "Tpm.h"
#include "TpmDevice.h"

volatile tpmOperation_t tpmOp = { 0 };
extern char tpmUnique[WC_SHA512_DIGEST_SIZE];

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

static bool TpmGenerateUnique(void)
{
    wc_Sha512 hash;
    struct
    {
        uint16_t mcuType;
        uint16_t mcuRev;
        uint16_t flashSize;
        unsigned char serial[12];
    } mcuInfo;

    ReadMcuInfo(mcuInfo.serial, &mcuInfo.flashSize, &mcuInfo.mcuType, &mcuInfo.mcuRev);

    if((wc_InitSha512(&hash)) ||
       (wc_Sha512Update(&hash, (const byte*)&mcuInfo, sizeof(mcuInfo))) ||
       (wc_Sha512Final(&hash, (byte*)tpmUnique)))
    {
        logError("Sha512 failed\r\n");
        return false;
    }
    wc_Sha512Free(&hash);

#ifndef NDEBUG
    uint8_t unique[WC_SHA512_DIGEST_SIZE] = {0};
    _plat__GetUnique(0, sizeof(unique), unique);
    dbgPrint("Generated tpmUnique");
    for(uint32_t n = 0; n < sizeof(unique); n++)
    {
        if(!(n % 16)) dbgPrintAppend("\r\n    ");
        dbgPrintAppend("%02x", ((unsigned int)(unique[n])));
    }
    dbgPrintAppend("\r\n");
#endif
    return true;
}

bool TpmInitializeDevice(void)
{
    int retVal = 0;

    tpmOp.receivingCmd = -1;
    TpmGenerateUnique();

    SetDutyCycleIndicator(FALSE);

    // Factory reset requested?
    if(BlueButtonTransitionDetected())
    {
        dbgPrint("Factory reset requested.\r\n");
        if((retVal = _plat__NVEnable((void*)1)) < 0)
        {
            logError("_plat__NVEnable(1) failed unrecoverable.")
        }
        dbgPrint("Waiting for the button to be released...\r\n");
        while(BlueButtonTransitionDetected() == 0);
    }
    else
    {
        if((retVal = _plat__NVEnable((void*)0)) < 0)
        {
            logError("_plat__NVEnable(0) failed unrecoverable.")
        }
    }


    if(retVal > 0)
    {
        dbgPrint("TPM_Manufacture(1) requested.\r\n");
        if((retVal = TPM_Manufacture(1)) != 0)
        {
            logError("TPM_Manufacture(1) failed.\r\n");
        }
    }

    dbgPrint("_plat__SetNvAvail().\r\n");
    _plat__SetNvAvail();
    dbgPrint("_plat__Signal_PowerOn().\r\n");
    if((retVal =_plat__Signal_PowerOn()) != 0)
    {
        logError("_plat__Signal_PowerOn() failed.\r\n");
    }
    dbgPrint("_plat__Signal_Reset().\r\n");
    if((retVal =_plat__Signal_Reset()) != 0)
    {
        logError("_plat__Signal_Reset() failed.\r\n");
    }
    return (retVal == 0);
}

bool TpmOperationsLoop(void)
{
    // Device reset
    if(tpmOp.flags.resetRequested == 1)
    {
        tpmOp.flags.resetRequested = 0;

        HAL_Delay(1);
        dbgPrint("Executing _plat__Signal_PowerOff()\r\n");
        _plat__Signal_PowerOff();
        PerformSystemReset();
        return false;
    }

    if(tpmOp.flags.powerOffRequested == 1)
    {
        tpmOp.flags.powerOffRequested = 0;
        dbgPrint("Executing _plat__Signal_PowerOff()\r\n");
        _plat__Signal_PowerOff();
        KillUSBLink();
        return false;
    }

    // Physical presence button (blue button on the Nucleo)
    int ppButton = BlueButtonTransitionDetected();
    if(ppButton > 0)
    {
        dbgPrint("Executing _plat__Signal_PhysicalPresenceOn().\r\n");
        _plat__Signal_PhysicalPresenceOn();
    }
    else if (ppButton < 0)
    {
        dbgPrint("Executing _plat__Signal_PhysicalPresenceOff().\r\n");
        _plat__Signal_PhysicalPresenceOff();
    }

    // Command processing
    if(tpmOp.flags.executionRequested == 1)
    {
        tpmOp.flags.executionRequested = 0;
        unsigned int rspLenTPM = sizeof(tpmOp.msgBuf) - sizeof(rspLenTPM);
        unsigned char* rspTPM = (unsigned char*)&tpmOp.msgBuf[sizeof(rspLenTPM)];

        itmPrintAppend(ITMCMDRSP, "unsigned char CmdBuf[%d] = {", tpmOp.cmdSize);
        for(uint32_t n = 0; n < tpmOp.cmdSize; n++)
        {
            if(n > 0) itmPrintAppend(ITMCMDRSP, ", ");
            if(!(n % 16)) itmPrintAppend(ITMCMDRSP, "\r\n");
            itmPrintAppend(ITMCMDRSP, "0x%02x", tpmOp.msgBuf[n]);
        }
        itmPrintAppend(ITMCMDRSP, "\r\n};\r\n");

        SetDutyCycleIndicator(TRUE);
        dbgPrint("TPM_CC = 0x%01x%02x\r\n", tpmOp.msgBuf[8], tpmOp.msgBuf[9]);
        time_t execStart = time(NULL);
        _plat__RunCommand((unsigned int)tpmOp.cmdSize, (unsigned char*)tpmOp.msgBuf, &rspLenTPM, &rspTPM);
        *((unsigned int*)tpmOp.msgBuf) = rspLenTPM;
        time_t execEnd = time(NULL);
        dbgPrint("Completion time %u'%u\"\r\n", (unsigned int)(execEnd - execStart) / 60, (unsigned int)(execEnd - execStart) % 60);
        dbgPrint("TPM_RC = 0x%01x%02x\r\n", rspTPM[8], rspTPM[9]);
        SetDutyCycleIndicator(FALSE);

        itmPrintAppend(ITMCMDRSP, "unsigned char RspBuf[%d] = {", tpmOp.cmdSize);
        for(uint32_t n = 0; n < rspLenTPM; n++)
        {
            if(n > 0) itmPrintAppend(ITMCMDRSP, ", ");
            if(!(n % 16)) itmPrintAppend(ITMCMDRSP, "\r\n");
            itmPrintAppend(ITMCMDRSP, "0x%02x", rspTPM[n]);
        }
        itmPrintAppend(ITMCMDRSP, "\r\n};\r\n");

        tpmOp.rspSize = sizeof(rspLenTPM) + rspLenTPM;
        tpmOp.cmdSize = 0;
        tpmOp.flags.responseRequested = 1;
    }

    if(tpmOp.flags.responseRequested == 1)
    {
        tpmOp.flags.responseRequested = 0;
        if(tpmOp.rspSize > 0)
        {
            uint32_t chunk = 0;
            while(CDC_Transmit_FS((unsigned char*)&tpmOp.msgBuf, 0) != 0); // Wake up the link
            while(CDC_Transmit_FS((unsigned char*)&tpmOp.msgBuf, 14) != 0); // Send the header which is the minimum size
            for(uint32_t n = 14; n < tpmOp.rspSize; n += chunk) // Send the rest in 16 byte increments
            {
                chunk = MIN(16, tpmOp.rspSize - n);
                while(CDC_Transmit_FS((unsigned char*)&tpmOp.msgBuf[n], chunk) != 0);
//                dbgPrint("Sent(%u)\r\n", (unsigned int)(n + chunk));
            }
            itmPrintAppend(ITMSIGNAL, "Response(%d)\r\n", tpmOp.rspSize);
        }
    }

    return true;
}

void TpmConnectionReset(void)
{
    tpmOp.receivingCmd = -1;
    tpmOp.cmdSize = 0;
    tpmOp.rspSize = 0;
    memset((void*)tpmOp.msgBuf, 0x00, sizeof(tpmOp.msgBuf));
}

bool TpmSignalEvent(uint8_t* Buf, uint32_t *Len)
{
    // Pending inbound transfer
    if(tpmOp.receivingCmd > 0)
    {
        memcpy((void*)&tpmOp.msgBuf[tpmOp.cmdSize], (void*)Buf, *Len);
        tpmOp.cmdSize += *Len;
//        itmPrintAppend(ITMSIGNAL, "Received(%d)\r\n", tpmOp.cmdSize);
        if(tpmOp.cmdSize >= tpmOp.receivingCmd)
        {
            itmPrintAppend(ITMSIGNAL, "Received(%d)\r\n", tpmOp.cmdSize);
            tpmOp.receivingCmd = -1;
            tpmOp.flags.executionRequested = 1;
        }
    }
    else if(sizeof(signalWrapper_t) > *Len)
    {
        itmPrintAppend(ITMSIGNAL, "Invalid frame received.\r\n");
        return false;
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
                        itmPrintAppend(ITMSIGNAL, "Invalid data size %u for SignalNothing(%u).\r\n", (unsigned int)*Len, (unsigned int)sig->s.dataSize);
                        return false;
                    }
                    itmPrintAppend(ITMSIGNAL, "SignalNothing\r\n");
                    break;

                case SignalShutdown:
                    if((sig->s.dataSize != 0) || (*Len != sizeof(signalWrapper_t)))
                    {
                        itmPrintAppend(ITMSIGNAL, "Invalid data size %u for SignalShutdown(%u).\r\n", (unsigned int)*Len, (unsigned int)sig->s.dataSize);
                        return false;
                    }
                    itmPrintAppend(ITMSIGNAL, "SignalShutdown\r\n");
                    tpmOp.flags.powerOffRequested = 1;
                    break;

                case SignalReset:
                    if((sig->s.dataSize != 0) || (*Len != sizeof(signalWrapper_t)))
                    {
                        itmPrintAppend(ITMSIGNAL, "Invalid data size %u for SignalReset(%u).\r\n", (unsigned int)*Len, (unsigned int)sig->s.dataSize);
                        return false;
                    }
                    itmPrintAppend(ITMSIGNAL, "SignalReset\r\n");
                    tpmOp.flags.resetRequested = 1;
                    break;

                case SignalSetClock:
                    if((sig->s.dataSize != sizeof(unsigned int)) || (*Len != sizeof(signalWrapper_t) + sizeof(unsigned int)))
                    {
                        itmPrintAppend(ITMSIGNAL, "Invalid data size %u for SignalSetClock(%u).\r\n", (unsigned int)*Len, (unsigned int)sig->s.dataSize);
                        return false;
                    }
                    payload = (pSignalPayload_t)&Buf[sizeof(signalWrapper_t)];
                    SetRealTimeClock(payload->SignalSetClockPayload.time);
                    itmPrintAppend(ITMSIGNAL, "SignalSetClock(0x%08x)\r\n", payload->SignalSetClockPayload.time);
                    break;

                case SignalCancelOn:
                    if((sig->s.dataSize != 0) || (*Len != sizeof(signalWrapper_t)))
                    {
                        itmPrintAppend(ITMSIGNAL, "Invalid data size %u for SignalCancelOn(%u).\r\n", (unsigned int)*Len, (unsigned int)sig->s.dataSize);
                        return false;
                    }
                    itmPrintAppend(ITMSIGNAL, "SignalCancelOn\r\n");
                    _plat__SetCancel();
                    break;

                case SignalCancelOff:
                    if((sig->s.dataSize != 0) || (*Len != sizeof(signalWrapper_t)))
                    {
                        itmPrintAppend(ITMSIGNAL, "Invalid data size %u for SignalCancelOff(%u).\r\n", (unsigned int)*Len, (unsigned int)sig->s.dataSize);
                        return false;
                    }
                    itmPrintAppend(ITMSIGNAL, "SignalCancelOff\r\n");
                    _plat__ClearCancel();
                    break;

                case SignalCommand:
                    if((sig->s.dataSize == 0) ||
                       (*Len == sizeof(signalWrapper_t)))
                    {
                        itmPrintAppend(ITMSIGNAL, "Invalid data size %u for SignalCommand(%u).\r\n", (unsigned int)*Len, (unsigned int)sig->s.dataSize);
                        return false;
                    }
                    payload = (pSignalPayload_t)&Buf[sizeof(signalWrapper_t)];
                    unsigned int expected = sizeof(signalWrapper_t) + sizeof(unsigned int) * 2 + payload->SignalCommandPayload.cmdSize;
                    unsigned int maxAllowed = sizeof(tpmOp.msgBuf);
                    memset((unsigned char*)tpmOp.msgBuf, 0x00, sizeof(tpmOp.msgBuf));
                    tpmOp.rspSize = 0;
                    itmPrintAppend(ITMSIGNAL, "SignalCommand(%d)\r\n", payload->SignalCommandPayload.cmdSize);

                    // Set the locality for the command
                    if(_plat__LocalityGet() != payload->SignalCommandPayload.locality)
                    {
                        _plat__LocalitySet(payload->SignalCommandPayload.locality);
                        itmPrintAppend(ITMSIGNAL, "SetLocality(%d)\r\n", payload->SignalCommandPayload.locality);
                    }

                    if((*Len == expected) &&
                       (payload->SignalCommandPayload.cmdSize <= maxAllowed))
                    {
                        memcpy((void*)tpmOp.msgBuf, (void*)payload->SignalCommandPayload.cmd, payload->SignalCommandPayload.cmdSize);
                        tpmOp.cmdSize = payload->SignalCommandPayload.cmdSize;
                        tpmOp.flags.executionRequested = 1;
//                        itmPrintAppend(ITMSIGNAL, "Received(%d)\r\n", tpmOp.cmdSize);
                    }
                    else if((*Len < expected) &&
                            (payload->SignalCommandPayload.cmdSize <= maxAllowed))
                    {
                        unsigned int dataSnip = *Len - (sizeof(signalWrapper_t) + sizeof(unsigned int) * 2);
                        memcpy((void*)tpmOp.msgBuf, (void*)payload->SignalCommandPayload.cmd, dataSnip);
                        tpmOp.receivingCmd = payload->SignalCommandPayload.cmdSize;
                        tpmOp.cmdSize = dataSnip;
//                        itmPrintAppend(ITMSIGNAL, "Received(%d)\r\n", tpmOp.cmdSize);
                    }
                    else
                    {
                        logError("Invalid command size.\r\n");
                        return false;
                    }
                    break;

                case SignalResponse:
                    if((sig->s.dataSize != 0) || (*Len != sizeof(signalWrapper_t)))
                    {
                        itmPrintAppend(ITMSIGNAL, "Invalid data size %u for SignalResponse(%u).\r\n", (unsigned int)*Len, (unsigned int)sig->s.dataSize);
                        return false;
                    }
                    itmPrintAppend(ITMSIGNAL, "SignalResponse\r\n");
                    if(tpmOp.rspSize > 0)
                    {
                        tpmOp.flags.responseRequested = 1;
                    }
                    break;

                default:
                    itmPrintAppend(ITMSIGNAL, "Unknown Signal %u received.\r\n", sig->s.signal);
                    return false;
                    break;
            }
        }
    }
    return true;
}
