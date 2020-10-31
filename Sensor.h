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

#pragma once

#include <Windows.h>

enum ELatencyType
{
    LatencyType_WaitForClick,
    LatencyType_AutoClick,
    LatencyType_Immediate
};

class Sensor
{
public:
    Sensor();
    ~Sensor();
    bool ConnectSensor();
    void StartLatencyMeasurement(ELatencyType type);
    float ReadLatency();
    float ReadLuminance();
    void SetActivationThreshold(float nits);
private:
    void SendCommand(int cmd);
    void Disconnect();
    bool Connect(int com);
    void SetTimeOuts(int v = 3);
//  void UpdateFirmware(int hwVer);
    bool QueryVersion();

    HANDLE hFile;
    int devId;
    int hwVer;
    int fwVer;
    float adcToNits;
};