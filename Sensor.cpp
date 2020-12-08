/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 *
 * Contact: rdimitrov@nvidia.com
 *
 */

#include <iostream>
#include "Sensor.h"

#define LS_CMD_QUERY_VERSION      1
#define LS_CMD_STOP               3
#define LS_CMD_LIGHT_INPUT        5
#define LS_CMD_LATENCY           10         // wait for a click and then measure latency
#define LS_CMD_BOOT_LOADER       13
#define LS_CMD_LATENCY_AUTO_FIRE 14         // click the mouse and then measure latency
#define LS_CMD_LATENCY_IMMEDIATE 15         // start measuring latency right away
#define LS_CMD_SET_LUM_THRESHOLD 19
#define LS_CMD_LATENCY_AUTO_FIRE_HID 24     // returns a time from when it sent a click event
#define LS_CMD_READ_ADC          30

const float CALIBRATION_SCALER = 5000.0f;

#define FW_UPDATE_VERSION        14
#define HW_SUPPORTED_VERSION      4
#define SPM_PAGESIZE            128

Sensor::Sensor()
{
    hFile = INVALID_HANDLE_VALUE;
}

Sensor::~Sensor()
{
    Disconnect();
}

bool Sensor::ConnectSensor()
{
    // Try multiple com ports, looking for the Sensor HW.
    for (int i = 3; i < 15; i++)
    {
        if (Connect(i))
        {
            break;
        }
    }

    if (hFile == INVALID_HANDLE_VALUE)
    {
        return false;
    }

    SendCommand(LS_CMD_LIGHT_INPUT);

    return true;
}

void Sensor::StartLatencyMeasurement(ELatencyType type)
{
    switch (type)
    {
    case LatencyType_WaitForClick:  SendCommand(LS_CMD_LATENCY); break;
    case LatencyType_AutoClick:     SendCommand(LS_CMD_LATENCY_AUTO_FIRE); break;
    case LatencyType_Immediate:     SendCommand(LS_CMD_LATENCY_IMMEDIATE); break;
    }
}

float Sensor::ReadLatency()
{
    DWORD n = 0;
    int rawLat = 0;
    while (hFile != INVALID_HANDLE_VALUE)
    {
        if (!ReadFile(hFile, &rawLat, 1, &n, NULL))
        {
            puts("Sensor connection error.");
            Disconnect();
            return -2;
        }
        if (n)
        {
            n = 0;
            int msb = 0;
            Sleep(2);
            ReadFile(hFile, &msb, 1, &n, NULL);
            if (n)
            {
                n = 2;
                rawLat |= (msb << 8);
                break;
            }
        }
    }

    if (n != 2 || rawLat == 0x0000FFFF)
    {
        puts("Latency read error.");
        return -1;
    }

    double RAW_TO_SEC;
    if (hwVer >= 5)
        RAW_TO_SEC = 1.0 / 32768.0;
    else
        RAW_TO_SEC = 256.0 / 7.37E6;
    float lat = (float)(RAW_TO_SEC * rawLat);

    return lat;
}

float Sensor::ReadLuminance()
{
    if (hwVer < 5)
    {
        return 0;
    }

    SendCommand(LS_CMD_READ_ADC);

    int16_t v;
    DWORD n = 0;
    ReadFile(hFile, &v, 2, &n, NULL);

    return adcToNits * v;
}

void Sensor::SetActivationThreshold(float nits)
{
    uint16_t v;

    // convert from nits to raw adc values
    v = (int)(nits / adcToNits + 0.5f);

    if (hwVer < 5)
    {
        v /= 4;
    }

    SendCommand(LS_CMD_SET_LUM_THRESHOLD);

    DWORD n = 0;
    WriteFile(hFile, &v, 2, &n, NULL);
}

void Sensor::SendCommand(int cmd)
{
    DWORD n = 0;
    n = WriteFile(hFile, &cmd, 1, &n, NULL);

    FlushFileBuffers(hFile);
}

void Sensor::Disconnect()
{
    if (hFile != INVALID_HANDLE_VALUE)
    {
        SendCommand(LS_CMD_STOP);
        CloseHandle(hFile);
        hFile = INVALID_HANDLE_VALUE;
    }
}

bool Sensor::Connect(int com)
{
    char portName[16];

    sprintf_s(portName, com > 9 ? "\\\\.\\COM%d" : "COM%d", com);
    hFile = CreateFileA(static_cast<LPCSTR>(portName),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL);
    if (hFile == INVALID_HANDLE_VALUE)
    {
        DWORD error = GetLastError();
        if (error == ERROR_FILE_NOT_FOUND)
        {
            printf("COM%d: not available.\n", com);
        }
        else
        {
            printf("COM%d: error 0x%x.\n", com, error);
        }
        hFile = INVALID_HANDLE_VALUE;
        return false;
    }

    DCB dcb = { 0 };

    if (!GetCommState(hFile, &dcb))
    {
        printf("Failed to get the current serial parameters.\n");
        Disconnect();
        return false;
    }

    dcb.BaudRate = CBR_38400;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = ODDPARITY;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;

    if (!SetCommState(hFile, &dcb))
    {
        printf("COM%d: could not set parameters.\n", com);
        Disconnect();
        return false;
    }

    printf("COM%d: connected. ", com);

    SetTimeOuts(500);

    if (!QueryVersion())
    {
        Disconnect();
        return false;
    }

    SetTimeOuts(3);

    return true;
}

void Sensor::SetTimeOuts(int v)
{
    COMMTIMEOUTS timeouts;
    timeouts.ReadIntervalTimeout = v;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = v;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = v;
    if (!SetCommTimeouts(hFile, &timeouts))
    {
        puts("Could not set the COM timeouts.");
    }
}
#if 0
void Sensor::UpdateFirmware(int hwVer) // Expects no time outs
{
    char romName[64];
    sprintf_s(romName, "Firmware v%d.rom", hwVer);
    FILE *fp;
    fopen_s(&fp, romName, "rb");
    if (!fp)
    {
        puts("No update file found for this Sensor HW version.");
        exit(-1);
    }

    fseek(fp, 0, SEEK_END);
    size_t size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    size_t alloc = ((size + SPM_PAGESIZE - 1) / SPM_PAGESIZE) * SPM_PAGESIZE;
    unsigned char *buf = new unsigned char[alloc];
    fread(buf, size, 1, fp);

    fclose(fp);

    if (alloc != size)
    {
        memset(buf + size, 0, alloc - size);
    }

    int pages = alloc / SPM_PAGESIZE;

    puts("Your Sensor will now be updated to the latest version. Do not interrupt this process.");

    SendCommand(LS_CMD_STOP);
    SendCommand(LS_CMD_BOOT_LOADER);
    Sleep(50);
    PurgeComm(hFile, PURGE_RXCLEAR | PURGE_TXCLEAR);

    DWORD n;

    int retries = 0;
    bool success = true;

    for (int p = 0; p < pages; p++)
    {
        unsigned int offset = p * SPM_PAGESIZE;
        unsigned char* base = buf + offset;

        WriteFile(hFile, &offset, 2, &n, NULL);

        unsigned int sum = offset;

        for (int i = 0; i < SPM_PAGESIZE; i += 2)
        {
            sum = (sum + *(unsigned __int16*)(base + i)) & 0xFFFF;
        }

        Sleep(25);

        WriteFile(hFile, &sum, 2, &n, NULL);
        FlushFileBuffers(hFile);
        Sleep(25);

        for (int i = 0; i < SPM_PAGESIZE; i += 2)
        {
            WriteFile(hFile, base + i, 2, &n, NULL);
            FlushFileBuffers(hFile);
            Sleep(1);
        }

        success = 0;
        while (1)
        {
            ReadFile(hFile, &success, 1, &n, NULL);
            if (n != 0)
            {
                break;
            }
            // If we arrive here, Sensor didn't respond: a byte must have been dropped.
            // Fill the rest of the page with garbage until we fail the checksum.
            int c = 0xFF;
            WriteFile(hFile, &c, 1, &n, NULL);
        }

        Sleep(25);

        printf(" Page %2d: %s\n", p, success ? "OK" : "Failed");

        if (!success)
        {
            if (p == 0)
            {
                puts("Update failed on the first page, bailing out...");
                break;
            }
            Sleep(100);
            retries++;
            p--;
        }

        if (retries >= 10)
        {
            printf("Too many errors (%d), giving up...\n", retries);
            break;
        }
    }

    if (success)
    {
        puts("Sensor Firmware update complete.");
    }
    else
    {
        puts("Sensor Firmware update failed.");
    }

    int end = 0xFFFF;
    WriteFile(hFile, &end, 2, &n, NULL);

    delete[] buf;

    Sleep(100);
}
#endif

bool Sensor::QueryVersion()
{
    SendCommand(LS_CMD_QUERY_VERSION);

    uint32_t deviceVer;
    DWORD n = 0;
    ReadFile(hFile, &deviceVer, sizeof(deviceVer), &n, NULL);

    if (n == 4 && (deviceVer & 0xFF) == 0xF0)
    {
        bool hasCalibration = false;

        devId = (deviceVer >> 8) & 0xFF;
        hwVer = (deviceVer >> 16) & 0xFF;
        fwVer = deviceVer >> 24;

        printf("LDAT HW: %d, FW: %d, Dev ID: %d\n", hwVer, fwVer, devId);

        unsigned int cal = 0;
        int16_t offset = 0;

        ReadFile(hFile, &cal, 2, &n, NULL);
        if (cal > 0 && cal != 0xFFFF)
        {
            if (hwVer < 5)
            {
                const float REF_NITS = 335;
                adcToNits = REF_NITS * (16.0f / cal);
            }
            else
            {
                adcToNits = (float)cal / CALIBRATION_SCALER;
            }
        }
        else
        {
            adcToNits = 0.25f;
        }

        n = hwVer >= 5 ? 2 : 1;
        ReadFile(hFile, &offset, n, &n, NULL);


        return true;
    }
    else
    {
        puts("No Sensor HW.");
        return false;
    }
}
