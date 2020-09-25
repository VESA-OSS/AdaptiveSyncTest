//********************************************************* 
// 
// Copyright (c) Microsoft. All rights reserved. 
// This code is licensed under the MIT License (MIT). 
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF 
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY 
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR 
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT. 
// 
//*********************************************************

#include "pch.h"

#include "winioctl.h"
#include "ntddvdeo.h"

//#include "BasicMath.h"
#include "ColorSpaces.h"
#include "Game.h"

#include <winrt\Windows.Devices.Display.h>
#include <winrt\Windows.Devices.Enumeration.h>

#define BRIGHTNESS_SLIDER_FACTOR (m_rawOutDesc.MaxLuminance / m_outputDesc.MaxLuminance)

//using namespace concurrency;

using namespace winrt::Windows::Devices;
using namespace winrt::Windows::Devices::Display;
using namespace winrt::Windows::Devices::Enumeration;

void ACPipeline();

extern void ExitGame();

using namespace DirectX;

using Microsoft::WRL::ComPtr;

Game::Game(PWSTR appTitle)
{
    m_appTitle = appTitle;

    ConstructorInternal();
}

#define SQUARE_COUNT 50
void Game::ConstructorInternal()
{
    if (!QueryPerformanceFrequency(&m_qpcFrequency))            // initialize clock frequency (of this machine)
    {
        throw std::exception("QueryPerformanceFrequency");
    }

    m_mediaPresentDuration = 0; // Duration when using SwapChainMedia for hardware syncs.
                                // Units are in 0.1us or 100ns intervals.  Zero 0 = off.

    m_minFrameRate =  24;       // fps
    m_maxFrameRate = 120;
    m_color = 0.f;              // black by default

    m_currentTest = TestPattern::StartOfTest;
    m_flickerRateIndex = 0;		// select between 60, 90, 120, 180, 240Hz for Flicker test          2
    m_waveCounter = SQUARE_COUNT;
    m_waveEnum = WaveEnum::ZigZag; // default                                                       3
    m_waveUp = true;            // for use in zigzag wave                                           3

    m_latencyRateIndex = 0;		// select between 60, 90, 120, 180, 240Hz for Latency tests         4
    m_mediaRateIndex   = 0;		// select between 60, 90, 120, 180, 240Hz for Jitter                5

    m_latencyTestFrameRate = 60.;//                                                                 4
    m_sensorConnected = false;  //
    m_sensing = false;          // 
    m_flash = false;            // whether we are flashing the photocell this frame                 4
    ResetSensorStats();         // initialize the sensor tracking data                              4
    ResetFrameStats();          // initialize the frame timing data
    m_avgInputTime = 0.006;     // hard coded at 6ms until dongle can drive input

    m_autoG2G = false;          // if we are in automatic sequence mode                             5
    m_from = true;              // start with "From" color                                          5
    m_g2gFromIndex = 0;         // subtest for Gray To Gray test                                    5
    m_g2gToIndex = 0;           // subtest for Gray To Gray test                                    5
    m_frameDropRateIndex = 0;   // select subtest for frameDrop test                                6
    m_frameLockRateIndex = 0;   // select sutbtest for frameDrop test                               7
    m_judderTestFrameRate = 60.;//
    m_fAngle = 0;               // angle of object moving around screen                             8,9
    m_MotionBlurIndex = maxMotionBlurs;  // start with frame fraction 100%                          8
    m_tearingTestFrameRate = 60.;  //
    m_sweepPos = 0;             // position of bar in Tearing test                                  0

    m_targetFrameRate = 60.f;
    m_frameTime     = 0.016666;
    m_lastFrameTime = 0.016666;
    m_sleepDelay = 16.0;        // ms simulate workload of app (just used for some tests)
    m_frameCount = 0;
    m_frameCounter = 0;         // frames since start of app execution
    m_totalTimeSinceStart = 0;  // init clock since app start
    m_paused = 0;               // start out unpaused

    // map the storage into the pointer array
    for ( int i = 0; i < frameLogLength; i++)
    {
        m_frameEvents[i].clickCounts = 0;
        m_frameEvents[i].readCounts = 0;
        m_frameEvents[i].drawCounts = 0;
        m_frameEvents[i].presentCounts = 0;
        m_frameEvents[i].syncCounts = 0;
        m_frameEvents[i].photonCounts = 0;

        m_frameLog[i] = &m_frameEvents[i];
    }

    m_currentProfileTile = 0;	// which intensity profile tile we are on
	m_maxPQCode = 1;
	m_maxProfileTile = 1;

    m_testingTier = TestingTier::DisplayHDR400;
    m_outputDesc = { 0 };
	m_rawOutDesc.MaxLuminance = 0.f;
	m_rawOutDesc.MaxFullFrameLuminance = 0.f;
	m_rawOutDesc.MinLuminance = 0.f;

    m_showExplanatoryText = true;

	m_deviceResources = std::make_unique<DX::DeviceResources>();

    m_hideTextString = std::wstring(L"Press SPACE to hide this text.");

    m_deviceResources->RegisterDeviceNotify(this);
}

// Shuffle all the entries in the frame log down to make room for one more
void Game::RotateFrameLog()
{
    // swap the last pointer for after
    FrameEvents *temp = m_frameLog[frameLogLength - 1];

    // shuffle all the others down.
    for (int i = frameLogLength-1; i > 0; i--)
    {
        m_frameLog[i] = m_frameLog[i-1];
    }

    // move last entry to front
    m_frameLog[0] = temp;
    m_frameLog[0]->frameID = 0;         // clear for use
}

// Initialize the Direct3D resources required to run.
void Game::Initialize(HWND window, int width, int height)
{
    m_deviceResources->SetWindow(window, width, height);
    m_deviceResources->CreateDeviceResources();
    m_deviceResources->SetDpi(96.0f);     // TODO: using default 96 DPI for now
    m_deviceResources->CreateWindowSizeDependentResources();

    CreateDeviceIndependentResources();
    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();

}

void Game::TogglePause()
{
    m_paused = !m_paused;
}

void Game::ToggleSensing()
{
    if (m_currentTest == TestPattern::DisplayLatency)
    {
        m_sensing = !m_sensing;
    }
}

void Game::ToggleAutoG2G()
{
    if (m_currentTest == TestPattern::GrayToGray )
    {
        m_autoG2G = !m_autoG2G;            // automatic sequence mode                              5
        if (m_autoG2G == false)
        {
            m_g2gFromIndex = 0;             // reset these on return to manual control
            m_g2gToIndex = 0;
        }
    }
}
// clear all the stat tracking from the sensor
void Game::ResetSensorStats()
{
    m_sensorCount = 0;
    m_totalSensorTime  = 0.;
    m_totalSensorTime2 = 0.;
    m_minSensorTime = 0.999999;
    m_maxSensorTime = 0.;
}

void Game::ResetFrameStats()
{
    m_frameCount = 0;

    m_totalFrameTime  = 0.;             // clear accumulator
    m_totalFrameTime2 = 0.;            // square of above used for variance math
//  m_minFrameTime = 0.999999;
//  m_maxFrameTime = 0.;

    m_totalRenderTime  = 0.;
    m_totalRenderTime2 = 0.;

    m_totalPresentTime  = 0.;
    m_totalPresentTime2 = 0.;

}

// clears the accumulators for the current mode  -bound to 's' key
void Game::ResetCurrentStats()
{
    if ( m_sensing )
        ResetSensorStats();
    else
        ResetFrameStats();
}

// Returns whether the reported display metadata consists of
// default values generated by Windows.
bool Game::CheckForDefaults()
{
    return (
        // Default SDR display values (RS2)
        (270.0f == m_rawOutDesc.MaxLuminance &&
         270.0f == m_rawOutDesc.MaxFullFrameLuminance &&
         0.5f == m_rawOutDesc.MinLuminance) ||
        // Default HDR display values (RS2)
        (550.0f == m_rawOutDesc.MaxLuminance &&
         450.0f == m_rawOutDesc.MaxFullFrameLuminance &&
         0.5f == m_rawOutDesc.MinLuminance) ||
        // Default HDR display values (RS3)
        (1499.0f == m_rawOutDesc.MaxLuminance &&
         799.0f == m_rawOutDesc.MaxFullFrameLuminance &&
         0.01f == m_rawOutDesc.MinLuminance)
        );
}

// Default to tier based on what the EDID specified
Game::TestingTier Game::GetTestingTier()
{
#define SAFETY (0.970)
         if (m_rawOutDesc.MaxLuminance >= SAFETY * 10000.0f)
        return TestingTier::DisplayHDR10000;
    else if (m_rawOutDesc.MaxLuminance >= SAFETY * 6000.0f)
        return TestingTier::DisplayHDR6000;
    else if (m_rawOutDesc.MaxLuminance >= SAFETY * 4000.0f)
        return TestingTier::DisplayHDR4000;
	else if (m_rawOutDesc.MaxLuminance >= SAFETY * 3000.0f)
		return TestingTier::DisplayHDR3000;
	else if (m_rawOutDesc.MaxLuminance >= SAFETY * 2000.0f)
        return TestingTier::DisplayHDR2000;
    else if (m_rawOutDesc.MaxLuminance >= SAFETY * 1400.0f)
        return TestingTier::DisplayHDR1400;
    else if (m_rawOutDesc.MaxLuminance >= SAFETY * 1000.0f)
        return TestingTier::DisplayHDR1000;
    else if (m_rawOutDesc.MaxLuminance >= SAFETY * 600.0f)
        return TestingTier::DisplayHDR600;
	else if (m_rawOutDesc.MaxLuminance >= SAFETY * 500.0f)
			 return TestingTier::DisplayHDR500;
	else
        return TestingTier::DisplayHDR400;
}

WCHAR *Game::GetTierName(Game::TestingTier tier)
{
         if (tier == DisplayHDR400)   return L"DisplayHDR400";
	else if (tier == DisplayHDR500)   return L"DisplayHDR500";
	else if (tier == DisplayHDR600)   return L"DisplayHDR600";
	else if (tier == DisplayHDR1000)  return L"DisplayHDR1000";
	else if (tier == DisplayHDR1400)  return L"DisplayHDR1400";
    else if (tier == DisplayHDR2000)  return L"DisplayHDR2000";
	else if (tier == DisplayHDR3000)  return L"DisplayHDR3000";
	else if (tier == DisplayHDR4000)  return L"DisplayHDR4000";
    else if (tier == DisplayHDR6000)  return L"DisplayHDR6000";
    else if (tier == DisplayHDR10000) return L"DisplayHDR10000";
    else return L"Unsupported DisplayHDR Tier";
}

float Game::GetTierLuminance(Game::TestingTier tier)
{
	     if (tier == DisplayHDR400)	  return  400.f;
	else if (tier == DisplayHDR500)   return  500.f;
	else if (tier == DisplayHDR600)   return  600.f;
	else if (tier == DisplayHDR1000)  return  1015.27f;
	else if (tier == DisplayHDR1400)  return  1400.f;
	else if (tier == DisplayHDR2000)  return  2000.f;
	else if (tier == DisplayHDR3000)  return  3000.f;
	else if (tier == DisplayHDR4000)  return  4000.f;
	else if (tier == DisplayHDR6000)  return  6000.f;
	else if (tier == DisplayHDR10000) return 10000.f;
	else return -1.0f;
}

// Determines whether tearing support is available for fullscreen borderless windows.
void Game::CheckTearingSupport()
{
#ifndef PIXSUPPORT
    ComPtr<IDXGIFactory6> factory;
    HRESULT hr = CreateDXGIFactory1(IID_PPV_ARGS(&factory));
    BOOL allowTearing = FALSE;
    if (SUCCEEDED(hr))
    {
        hr = factory->CheckFeatureSupport(DXGI_FEATURE_PRESENT_ALLOW_TEARING, &allowTearing, sizeof(allowTearing));
    }

    m_tearingSupport = SUCCEEDED(hr) && allowTearing;
#else
    m_tearingSupport = TRUE;
#endif
}

#pragma region Frame Update
// Executes the basic game-style animation loop.

uint64_t myQPC()
{
    static const uint64_t TicksPerSecond = 10000000;    // microseconds
    LARGE_INTEGER cycles, qpcFrequency;

    QueryPerformanceFrequency(&qpcFrequency);

    QueryPerformanceCounter(&cycles);

    uint64_t r = cycles.QuadPart * TicksPerSecond;

    return r;
}


void mySleep(double mseconds)                // All routines named "sleep" must take milliseconds ms
{
    LARGE_INTEGER qpcFrequency, counts;                 // args to system calls
    uint64_t  time, timeLimit;                          // 64b ints for intermediate checking

    if (mseconds <= 0)                                  // validate input
    {
        Sleep(0);                                       // in case this is in a loop
        return;
    }

    QueryPerformanceFrequency(&qpcFrequency);           // get counts per second

    QueryPerformanceCounter(&counts);
    timeLimit = counts.QuadPart;
    timeLimit += (mseconds * qpcFrequency.QuadPart) / 1000; // convert time limit from ms into counts

    time = 0;
    while (time < timeLimit)
    {
        Sleep(0);                                       // cede control to OS for other processes
        QueryPerformanceCounter(&counts);
        time = counts.QuadPart;
    }
}

INT64 getPerfCounts(void)
{
    LARGE_INTEGER time;
    QueryPerformanceCounter(&time);
    return time.QuadPart;
}

bool Game::isMedia()
{
    switch (m_currentTest)
    {
        // cases where a media oriented fixed-frame rate is best
        case TestPattern::FlickerConstant:      //  2
        case TestPattern::FrameDrop:            //  6 
        case TestPattern::FrameLock:            //  7
        case TestPattern::MotionBlur:           //  8
            return true;

        // cases where a game-oriented adaptive frame rate is best
        case TestPattern::FlickerVariable:      //  3 
        case TestPattern::DisplayLatency:       //  4
        case TestPattern::GrayToGray:           //  5   
        case TestPattern::GameJudder:           //  9
        case TestPattern::Tearing:              //  0
            return false;

        default:
            return false;

    }
}

// Tick method for use with photocell sensor
void Game::Tick()
{
    double frameTime;                               // local variable for comparison
    Sensor sensor;                                  // external photocell devicep
    double dFrequency = m_qpcFrequency.QuadPart;    // counts per second of QPC

    // Update with data from last frame
    // get sync time of last frame from swapchain
    auto sc = m_deviceResources->GetSwapChain();
    DXGI_FRAME_STATISTICS frameStats;
    sc->GetFrameStatistics(&frameStats);
    m_frameLog[1]->syncCounts = frameStats.SyncQPCTime.QuadPart;
    // this is from 2 frames ago since we havent rotated yet

    // set photon time to be a bit after sync time until we have hardware event
    m_frameLog[0]->photonCounts = m_frameLog[0]->syncCounts + 100000;   // assume 10ms of GtG, etc.

    m_frameCounter++;                                       // new frame id

    if (!m_paused)
    {
        if (m_sensing)
        {
            m_flash = m_frameCounter & 1;                        // switch every other frame

            // if there should be a flash this frame, reconnect the sensor (should not be required every frame)
            if (m_flash)
            {
#if 1
                m_sensorConnected = sensor.ConnectSensor();
#else
                if  ( !m_sensorConnected )
                {
                    m_sensorConnected = sensor.ConnectSensor();
                }
//              Sleep(100);
#endif
            }

            // advance printout columns to current frame
            RotateFrameLog();                                    // make room to store this latest data

//          m_frameLog[0]->frameID = m_frameCounter;             // log the frame ID

            // log time when app starts a frame
            m_frameLog[0]->readCounts = getPerfCounts();

            // Don't bother to sleep during sensing
            // model app workload by sleeping that many ms
//          mySleep(sleepDelay);
            Update();                                       // actually do some CPU stuff

            // log when drawing starts on the GPU
//          m_frameLog[0]->drawCounts = getPerfCounts();

            // Draw
            Render();                                       // this is before we have all values for this frame

            // if there should be a flash this frame, start measuring.
            if (m_flash)
                sensor.StartLatencyMeasurement(LatencyType_AutoClick);

            // Show the new frame
            m_deviceResources->Present();

            // log time when app calls Present()
            m_frameLog[0]->presentCounts = getPerfCounts();

            // if this was a frame that included a flash, then read the photocell's measurement
            if (m_flash)
            {
                float sensorTime = 0.f;                             // time from sensor in seconds
                sensorTime = sensor.ReadLatency();                  // blocking call

                if ((sensorTime > 0.001) && (sensorTime < 0.100))     // if valid, run the stats on it
                {
                    // total it up for computing the average and variance
                    m_totalSensorTime += sensorTime;
                    m_totalSensorTime2 += (double) sensorTime * sensorTime;
                    m_sensorCount++;

                    // scan for min
                    if (sensorTime < m_minSensorTime)
                        m_minSensorTime = sensorTime;

                    // scan for max
                    if (sensorTime > m_maxSensorTime)
                        m_maxSensorTime = sensorTime;
                }
            }
        }
        else      // we are not using the sensor, just track in-PC timings
        {
        //  m_flash =  m_frameCounter & 1;                          // switch every other frame
        //  m_flash = (m_frameCounter >> 1) & 1;                    // switch every 2 frames
            m_flash = (m_frameCounter >> 2) & 1;                    // switch every 4 frames

            // advance to current frame
            RotateFrameLog();                                    // make room to store this latest data

            m_frameLog[0]->frameID = m_frameCounter;             // log the frame ID

            // Read input events()
            // log time when the button was clicked -extract the click time stamp
            // log time when photocell was hit -extract the photon time stamp
            // log time when the camera image flashed -extract the camera result time stamp

            // log time when app starts a frame
            m_frameLog[0]->readCounts = getPerfCounts();

            // simulate input device click that occurred before we read it.
            INT64 inputCounts = m_avgInputTime * dFrequency;    
            m_frameLog[0]->clickCounts = m_frameLog[0]->readCounts - inputCounts;

            if (m_mediaPresentDuration)
            {
                // rely on GPU refresh timer to control frame rate
            }
            else
            {
                // adjust delay to reach target frame rate
                double currentFrameTime = 0.5 * (m_frameTime + m_lastFrameTime);        // average of last two frame times
                double targetFrameTime = 1.0 / m_targetFrameRate;
                double deltaFrameTimeMs = (targetFrameTime - currentFrameTime) * 1000.0;   // also convert to ms
                m_sleepDelay += deltaFrameTimeMs * 0.5;                                       // 50% relaxation factor

                if (m_sleepDelay < 0.005) m_sleepDelay = 0.0;                               // limit to valid range
                if (m_sleepDelay > 50.0)  m_sleepDelay = 50.0;

                double sleepDelay = m_sleepDelay;

                // model app workload by sleeping that many ms
                mySleep(sleepDelay);
            }

            Update();                                       // actually do some CPU stuff

            // log when drawing starts on the GPU
            m_frameLog[0]->drawCounts = getPerfCounts();
            // Draw()
            Render();                                       // update screen (before we have all values for this frame)

            // log time when app calls Present()
            m_frameLog[0]->presentCounts = getPerfCounts();
            // Show the new frame
            m_deviceResources->Present();


            // log time when vsync happens on display
    //      m_frameLog[0]->syncCounts = presentCounts;      // clear this as we don't know until next frame

            // log time when photons arrive at photocell
    //      m_frameLog[0]->photonCounts = presentCounts;    // clear this as we don't know until next frame

// track frame time for frame rate
            m_lastFrameTime = m_frameTime;
            m_frameTime = (m_frameLog[0]->readCounts - m_lastReadCounts) / dFrequency;

            m_lastReadCounts = m_frameLog[0]->readCounts;

            // if valid data
            if ((m_frameTime > 0.001) && (m_frameTime < 0.100))     // run the stats on it
            {
                m_frameCount++;                                     // frames we average over

                // accumulate time for computing the averages
                m_totalFrameTime += m_frameTime;
                m_totalFrameTime2 += m_frameTime * m_frameTime;
#if 0
                // scan for min
                if (m_frameTime < m_minLatency)
                    m_minLatency = m_frameTime;

                // scan for max
                if (m_frameTime > m_maxLatency)
                    m_maxLatency = m_frameTime;
#endif
                // accumulate Render time for computing Average
                double renderTime = (m_frameLog[0]->presentCounts - m_frameLog[0]->drawCounts) / dFrequency;
                m_totalRenderTime += renderTime;
                m_totalRenderTime2 += renderTime * renderTime;

                // accumulate Present time for computing Average
                double presentTime = (m_frameLog[2]->syncCounts - m_frameLog[2]->presentCounts) / dFrequency;
                m_totalPresentTime += presentTime;
                m_totalPresentTime2 += presentTime * presentTime;


            }
        }
    }
    else
        Sleep( 15 );                                      // update at ~30Hz even when paused.
                 // save for use next frame

    // track data since app startup
    m_totalTimeSinceStart += m_frameTime;       // TODO totalTime should not be a double but a uint64 in microseconds
}

void Game::TickOld()
{
    double frameTime;                               // local variable for comparison

    // Update with data from last frame
    // get sync time of last frame from swapchain
    auto sc = m_deviceResources->GetSwapChain();
    DXGI_FRAME_STATISTICS frameStats;
    sc->GetFrameStatistics(&frameStats);
    m_frameLog[0]->syncCounts = frameStats.SyncQPCTime.QuadPart;

    // set photon time to be a bit after sync time until we have hardware event
    m_frameLog[0]->photonCounts = m_frameLog[0]->syncCounts + 100000;   // assume 10ms of GtG, etc.

    m_frameCounter++;                                       // new frame id

//  m_flash =  m_frameCounter & 1;                          // switch every other frame
//  m_flash = (m_frameCounter >> 1) & 1;                    // switch every 2 frames
    m_flash = (m_frameCounter >> 2) & 1;                    // switch every 4 frames

    if (!m_paused)
    {
        // advance to current frame
        RotateFrameLog();                                    // make room to store this latest data

        m_frameLog[0]->frameID = m_frameCounter;             // log the frame ID

        // Read input events()
        // log time when the button was clicked -extract the click time stamp
        // log time when photocell was hit -extract the photon time stamp
        // log time when the camera image flashed -extract the camera result time stamp

        // log time when app starts a frame
        m_frameLog[0]->readCounts = getPerfCounts();

        // simulate input device click occurred 10ms before we read it.
        m_frameLog[0]->clickCounts = m_frameLog[0]->readCounts - 100000; // assume 10ms of USB stack latency

        double sleepDelay = (1000.0 / m_targetFrameRate) - 1.3;    // assume 1.3ms of GPU workload

        // model app workload by sleeping that many ms
        mySleep(sleepDelay);
        Update();                                       // actually do some CPU stuff

        // log when drawing starts on the GPU
        m_frameLog[0]->drawCounts = getPerfCounts();
        Render();                                       // update screen (before we have all values for this frame)

        // Show the new frame.
        m_deviceResources->Present();

        // log time when app calls Present()
        m_frameLog[0]->presentCounts = getPerfCounts();

        // log time when vsync happens on display
//      m_frameLog[0]->syncCounts = presentCounts;      // clear this as we don't know until next frame

        // log time when photons arrive at photocell
//      m_frameLog[0]->photonCounts = presentCounts;    // clear this as we don't know until next frame
    }
    else
        Sleep( 15 );                                    // update at ~60Hz even when paused.

    double dFrequency = m_qpcFrequency.QuadPart;        // counts per second of QPC

    // Compute current frame time since last frame
    m_frameTime = (m_frameLog[0]->readCounts - m_lastReadCounts) / dFrequency;
    m_lastReadCounts = m_frameLog[0]->readCounts;       // save for use next frame

    // track data since app startup
    m_totalTimeSinceStart += m_frameTime;       // TODO totalTime should not be a double but a uint64 in microseconds
}

// Every test pattern could have an update routine to call in the main update routine in Tick()

// compute frame rate for this Flicker test with variable frame rate:                                       3
void Game::UpdateFlickerVariable()
{
    switch (m_waveEnum)
    {
        case WaveEnum::SquareWave:
        {
            m_waveCounter--;
            if (m_waveCounter > 0)
            {
                if (m_waveUp)
                    m_targetFrameRate = m_maxFrameRate;
                else
                    m_targetFrameRate = m_minFrameRate;
            }
            else
            {
                m_waveUp = !m_waveUp;           // toggle to wave down
                m_waveCounter = SQUARE_COUNT;   // reset counter
            }
        }
        break;

        case WaveEnum::ZigZag:    // zig-zag not square wave
        {
            if (m_waveUp)
            {
                m_targetFrameRate += 1;
                if (m_targetFrameRate > m_maxFrameRate)
                {
                    m_targetFrameRate = m_maxFrameRate;
                    m_waveUp = false;
                }
            }
            else
            {
                m_targetFrameRate -= 1;
                if (m_targetFrameRate < m_minFrameRate)
                {
                    m_targetFrameRate = m_minFrameRate;
                    m_waveUp = true;
                }
            }
        }
        break;

        case WaveEnum::Random:
        {
            double range = m_maxFrameRate - m_minFrameRate;
            m_targetFrameRate = m_minFrameRate + range * rand() / RAND_MAX;
            // TODO clamp max delta from current frame rate to some limit?s
        }
        break;

        case WaveEnum::Sine:
        {
            double amplitude = 0.5 * (m_maxFrameRate - m_minFrameRate);
            double baseline = 0.5 * (m_maxFrameRate + m_minFrameRate);
            double angle = m_totalTimeSinceStart;

            m_targetFrameRate = amplitude * sin(angle) + baseline;
        }
        break;
    }
}

// Set correct color for mid-screen test rect in G2G test section 10.2
float Game::GrayToGrayValue(INT32 index)
{
    float c, nits = 0;

    if (!CheckHDR_On())         // SDR case
    {
        double fDelta = 1.0f / (numGtGValues - 1);
        double d = fDelta * index;                  // proportion of the range
        c = pow(d, 2.2f);                           // apply gamma
//      c = d * m_outputDesc.MaxLuminance;          // scale to peak luminance
    }
    else                        // HDR case:
    {
        switch (index)
        {
        case 0: nits = 0; break;
        case 1: nits = 0.047366 * m_outputDesc.MaxLuminance; break;
        case 2: nits = 0.217638 * m_outputDesc.MaxLuminance; break;
        case 3: nits = 0.531049 * m_outputDesc.MaxLuminance; break;
        case 4: nits = m_outputDesc.MaxLuminance; break;
        }
        c = nitstoCCCS(nits);
    }
    return c;
}

// update routine for Gray2Gray Test pattern                                                               5
void Game::UpdateGrayToGray()
{
    if ( m_autoG2G )      // if we are in auto-sequence
    {
/*
        update the timer
        if it has run down, then
            Toggle the m_from boolean
            If we are back at true, then
            Increment the FromIndex
                if that overflows, increment the ToIndex
*/
        m_testTimeRemainingSec -= m_frameTime;
        if (m_testTimeRemainingSec < 0.f)           // timer has run out, so change something
        {
            if (m_from)
            {
                m_from = false;                     // switch to the To color
            }
            else  // we were on the To color
            {
                m_g2gFromIndex++;
                if (m_g2gFromIndex > numGtGValues - 1)
                {
                    m_g2gFromIndex = 0;

                    m_g2gToIndex++;
                    if (m_g2gToIndex > numGtGValues - 1)
                        m_g2gToIndex = 0;
                }
                m_from = true;
            }
            if ( m_g2gFromIndex != m_g2gToIndex )       // skip the diagonal where to and from are same
                m_testTimeRemainingSec = 0.5f; //  0.016666666f * 4;      // reset the timer to 4 frames
        }
    }
    else      // we are in manual state setting mode (not auto sequence)
    {
        m_from = (m_frameCounter >> 4) & 1;                           // switch every 16 frames
    }

    // compute color for test patch
    if ( m_from )
        m_color = GrayToGrayValue(m_g2gFromIndex);
    else
        m_color = GrayToGrayValue(m_g2gToIndex);
}

// Update any parameters used for animations.
void Game::Update()
{
    switch (m_currentTest)
    {
    case TestPattern::ConnectionProperties:                                                     //
        UpdateDxgiRefreshRatesInfo();
        break;

    case TestPattern::PanelCharacteristics:                                                     // 1
		UpdateDxgiColorimetryInfo();
		break;

    case TestPattern::FlickerVariable:                                                          // 3
        UpdateFlickerVariable();
        break;

    case TestPattern::DisplayLatency:                                                           // 4
        {

        }
        break;

    case TestPattern::GrayToGray:                                                               // 5
        {
            UpdateGrayToGray();
        }
        break;

    case TestPattern::WarmUp:                                                                   // W
        if (m_newTestSelected)
        {
            m_testTimeRemainingSec = 60.0f * 30.0f;	// 30 minutes
        }
        else
        {
            m_testTimeRemainingSec -= m_frameTime;
            m_testTimeRemainingSec = std::max(0.0, m_testTimeRemainingSec);
        }
        break;

    case TestPattern::Cooldown:                                                                 // C
        if (m_newTestSelected)
        {
            m_testTimeRemainingSec = 60.0f;		// One minute is 60 seconds
        }
        else
        {
            m_testTimeRemainingSec -= m_frameTime;
            m_testTimeRemainingSec = std::max(0.0, m_testTimeRemainingSec);
        }
        if (m_testTimeRemainingSec <= 0.0f)
            SetTestPattern(m_cachedTest);
        break;

    default:
        // Not all test patterns have animations, so not implemented is fine.
        break;
    }

    auto ctx = m_deviceResources->GetD2DDeviceContext();

    D2D1_BUFFER_PRECISION prec = D2D1_BUFFER_PRECISION_UNKNOWN;
    switch (m_deviceResources->GetBackBufferFormat())
    {
    case DXGI_FORMAT_B8G8R8A8_UNORM:
        prec = D2D1_BUFFER_PRECISION_8BPC_UNORM;
        break;

    case DXGI_FORMAT_R16G16B16A16_UNORM:
        prec = D2D1_BUFFER_PRECISION_16BPC_UNORM;
        break;

    case DXGI_FORMAT_R16G16B16A16_FLOAT:
        prec = D2D1_BUFFER_PRECISION_16BPC_FLOAT;
        break;

    default:
        DX::ThrowIfFailed(E_INVALIDARG);
        break;
    }

    if (m_dxgiColorInfoStale)
    {
        UpdateDxgiColorimetryInfo();
    }
    // TODO same for UpdateDxgiRefreshRatesInfo()

}

void Game::ChangeG2GFromIndex(bool increment)
{
    if (increment)
    {
        m_g2gFromIndex++;
        if (m_g2gFromIndex > numGtGValues - 1)
            m_g2gFromIndex = 0;
    }
    else
    {
        m_g2gFromIndex--;
        if (m_g2gFromIndex < 0)
            m_g2gFromIndex = numGtGValues - 1;
    }

    m_g2gFromIndex = m_g2gFromIndex % (numGtGValues + 1);

}

void Game::ChangeG2GToIndex(bool increment)
{
    if (increment)
    {
        m_g2gToIndex++;
        if (m_g2gToIndex > numGtGValues - 1)
            m_g2gToIndex = 0;
    }
    else
    {
        m_g2gToIndex--;
        if (m_g2gToIndex < 0)
            m_g2gToIndex = numGtGValues - 1;
    }

    m_g2gToIndex = m_g2gToIndex % (numGtGValues + 1);

}

// handle the up/down arrow key inputs
void Game::ChangeSubtest(bool increment)
{
    int testTier;
    switch (m_currentTest)
    {
    case TestPattern::PanelCharacteristics:
        testTier = (int)m_testingTier;
        testTier += (increment ? 1 : -1);
        testTier = clamp(testTier, (int)TestingTier::DisplayHDR400, (int)TestingTier::DisplayHDR10000);
        m_testingTier = (TestingTier)testTier;
        break;

    case TestPattern::FlickerConstant:                                                          // 2
        if (increment)
        {
            m_flickerRateIndex++;
            if (m_flickerRateIndex > numFlickerRates-1)
                m_flickerRateIndex = 0;
        }
        else
        {
            m_flickerRateIndex--;
            if (m_flickerRateIndex < 0)
                m_flickerRateIndex = numFlickerRates-1;
        }
        m_flickerRateIndex = m_flickerRateIndex % (numFlickerRates+1);
        break;

    case TestPattern::FlickerVariable:                                                          // 3
        if (increment)
        {
            unsigned int testInt = static_cast<unsigned int>(m_waveEnum) + 1;
            m_waveEnum = static_cast<WaveEnum>(testInt);
            if (m_waveEnum == WaveEnum::Max )
            {
                m_waveEnum = WaveEnum::ZigZag;      // wrap to the beginning
            }
        }
        else
        {
            unsigned int testInt = static_cast<unsigned int>(m_waveEnum) - 1;
            m_waveEnum = static_cast<WaveEnum>(testInt);
            if (m_waveEnum == WaveEnum::Nil )
            {
                m_waveEnum = WaveEnum::Sine;       // wrap to the last
            }
        }

/*
        if (increment)
        {
            if ( WaveEnum::Sine == m_waveEnum)
            {
                // Clamp to the end of the list.
            }
            else
            {
                unsigned int testInt = static_cast<unsigned int>(m_waveEnum) + 1;
                m_waveEnum = static_cast<WaveEnum>(testInt);
            }
        }
        else
        {
            if (WaveEnum::ZigZag == m_waveEnum)
            {
                // Clamp to the start of the list.
            }
            else
            {
                unsigned int testInt = static_cast<unsigned int>(m_waveEnum) - 1;
                m_waveEnum = static_cast<WaveEnum>(testInt);
            }
        }
*/
        break;

// TODO: these should probably all have their own state...  m_judderTestFrameRate, etc.
    case TestPattern::DisplayLatency:                                                           // 4
    {
        double delta = 1.0;
        if (m_latencyTestFrameRate >=  69.6) delta = 2.0;
        if (m_latencyTestFrameRate >= 149.6) delta = 5.0;
        if (m_latencyTestFrameRate >= 239.6) delta = 10.0;
        if (m_latencyTestFrameRate >= 479.6) delta = 30.0;

        if (increment)
        {
            m_latencyTestFrameRate -= delta;
        }
        else
        {
            m_latencyTestFrameRate += delta;
        }
        break;
    }

    case TestPattern::GrayToGray:                                                               // 5
        ChangeG2GToIndex(increment);
        break;

    case TestPattern::FrameDrop:                                                                // 6
        if (increment)
        {
            m_frameDropRateIndex++;
            if (m_frameDropRateIndex > numFrameDropRates-1)
                m_frameDropRateIndex = 0;
        }
        else
        {
            m_frameDropRateIndex--;
            if (m_frameDropRateIndex < 0)
                m_frameDropRateIndex = numFrameDropRates-1;
        }
        m_frameDropRateIndex = m_frameDropRateIndex % numFrameDropRates;
        break;

    case TestPattern::FrameLock:        // use media frame rates here                           // 7
        if (increment)
        {
            m_frameLockRateIndex++;
            if (m_frameLockRateIndex > numMediaRates-1)
                m_frameLockRateIndex = 0;
        }
        else
        {
            m_frameLockRateIndex--;
            if (m_frameLockRateIndex < 0)
                m_frameLockRateIndex = numMediaRates-1;
        }
        m_frameLockRateIndex = m_frameLockRateIndex % numMediaRates;
        break;

    case TestPattern::MotionBlur:                                                               // 8
        if (!increment)                     // make arrow keys run the right way
        {
            m_MotionBlurIndex++;
            if (m_MotionBlurIndex > maxMotionBlurs+3)
                m_MotionBlurIndex = 0;
        }
        else
        {
            m_MotionBlurIndex--;
            if (m_MotionBlurIndex < 0)
                m_MotionBlurIndex = maxMotionBlurs+3;
        }
//      m_MotionBlurIndex = m_MotionBlurIndex % maxMotionBlurs;
        break;

    case TestPattern::GameJudder:                                                               // 9
    {
        double delta = 1.0;
        if (m_judderTestFrameRate >= 69.6) delta = 2.0;
        if (m_judderTestFrameRate >= 149.6) delta = 5.0;
        if (m_judderTestFrameRate >= 239.6) delta = 10.0;
        if (m_judderTestFrameRate >= 479.6) delta = 30.0;

        if (increment)
        {
            m_judderTestFrameRate -= delta;
        }
        else
        {
            m_judderTestFrameRate += delta;
        }
        break;
    }

    case TestPattern::Tearing:                                                                  // 0
    {
        double delta = 1.0;
        if (m_tearingTestFrameRate >= 69.6) delta = 2.0;
        if (m_tearingTestFrameRate >= 149.6) delta = 5.0;
        if (m_tearingTestFrameRate >= 239.6) delta = 10.0;
        if (m_tearingTestFrameRate >= 479.6) delta = 30.0;

        if (increment)
        {
            m_tearingTestFrameRate -= delta;
        }
        else
        {
            m_tearingTestFrameRate += delta;
        }
        break;
    }

    }
}
void Game::UpdateDxgiRefreshRatesInfo()
{
    // TODO move code to find min/maxFrameRates into here from ConnectionProperties test pattern routine

}

void Game::UpdateDxgiColorimetryInfo()
{
    // Output information is cached on the DXGI Factory. If it is stale we need to create
    // a new factory and re-enumerate the displays.
    auto d3dDevice = m_deviceResources->GetD3DDevice();

    ComPtr<IDXGIDevice3> dxgiDevice;
    DX::ThrowIfFailed(d3dDevice->QueryInterface(IID_PPV_ARGS(&dxgiDevice)));

    ComPtr<IDXGIAdapter> dxgiAdapter;
    DX::ThrowIfFailed(dxgiDevice->GetAdapter(&dxgiAdapter));

    ComPtr<IDXGIFactory4> dxgiFactory;
    DX::ThrowIfFailed(dxgiAdapter->GetParent(IID_PPV_ARGS(&dxgiFactory)));

//  if (!dxgiFactory->IsCurrent())
    {
        DX::ThrowIfFailed(CreateDXGIFactory1(IID_PPV_ARGS(&dxgiFactory)));
    }

    // Get information about the display we are presenting to.
    ComPtr<IDXGIOutput> output;
    auto sc = m_deviceResources->GetSwapChain();
    DX::ThrowIfFailed(sc->GetContainingOutput(&output));

    ComPtr<IDXGIOutput6> output6;
    output.As(&output6);

    DX::ThrowIfFailed(output6->GetDesc1(&m_outputDesc));

	// Get raw (not OS-modified) luminance data:
	DISPLAY_DEVICE device = {};
	device.cb = sizeof(device);

	DisplayMonitor foundMonitor{ nullptr };
	for (UINT deviceIndex = 0; EnumDisplayDevices(m_outputDesc.DeviceName, deviceIndex, &device, EDD_GET_DEVICE_INTERFACE_NAME); deviceIndex++)
	{
		if (device.StateFlags & DISPLAY_DEVICE_ACTIVE)
		{
			foundMonitor = DisplayMonitor::FromInterfaceIdAsync(device.DeviceID).get();
			if (foundMonitor)
			{
				break;
			}
		}
	}

	if (!foundMonitor)
	{
	}
        
    winrt::Windows::Graphics::SizeInt32 dims;
    dims = foundMonitor.NativeResolutionInRawPixels();
    m_modeWidth  = dims.Width;
    m_modeHeight = dims.Height;

	// save the raw (not OS-modified) luminance data:
	m_rawOutDesc.MaxLuminance = foundMonitor.MaxLuminanceInNits();
	m_rawOutDesc.MaxFullFrameLuminance = foundMonitor.MaxAverageFullFrameLuminanceInNits();
	m_rawOutDesc.MinLuminance = foundMonitor.MinLuminanceInNits();
	// TODO: Should also get color primaries...

	// get PQ code at MaxLuminance
	m_maxPQCode = (int) roundf(1023.0f*Apply2084(m_rawOutDesc.MaxLuminance / 10000.f));

	m_dxgiColorInfoStale = false;

    //	ACPipeline();
}


void Game::GenerateTestPattern_StartOfTest(ID2D1DeviceContext2* ctx)
{
    // make sure we find the min/max resolutions even on startup
    auto format = m_deviceResources->GetBackBufferFormat();

    ComPtr<IDXGIOutput> output;
    auto sc = m_deviceResources->GetSwapChain();
    DX::ThrowIfFailed(sc->GetContainingOutput(&output));

    UINT num = 0;
    UINT flags = 0;

    // first call to get the number of modes in this format
    output->GetDisplayModeList(format, flags, &num, 0);

    // second call collects them into an array
    DXGI_MODE_DESC* pDescs = new DXGI_MODE_DESC[num];
    output->GetDisplayModeList(format, flags, &num, pDescs);

    m_minFrameRate = 1000000;       // 1 million
    m_maxFrameRate = 0;
    for (int i = 0; i < num; i++)
    {
        if (pDescs[i].Width == m_modeWidth && pDescs[i].Height == m_modeHeight)
        {
            double rate = (double)pDescs[i].RefreshRate.Numerator / (double)pDescs[i].RefreshRate.Denominator;
            if (rate > m_maxFrameRate) m_maxFrameRate = rate;                   // scan to find min/max
            if (rate < m_minFrameRate) m_minFrameRate = rate;
        }
    }

    std::wstringstream text;

    text << m_appTitle;
    text << L"\n\nVersion 0.70\n\n";
    //text << L"ALT-ENTER: Toggle fullscreen: all measurements should be made in fullscreen\n";
	text << L"->, PAGE DN:       Move to next test\n";
	text << L"<-, PAGE UP:        Move to previous test\n";
    text << L"NUMBER KEY:	Jump to test number\n";
    text << L"SPACE:		Hide text and target circle\n";
    text << L"P, Pause:               Pause\n";
    text << L"C:		Start 60s cool-down\n";
    text << L"W:		Start 30min warm-up\n";
    text << L"ALT-ENTER:	Toggle fullscreen\n";
    text << L"ESCAPE:		Exit fullscreen\n";
    text << L"ALT-F4:		Exit app\n";
    text << L"\nCopyright © VESA\nLast updated: " << __DATE__;

    RenderText(ctx, m_largeFormat.Get(), text.str(), m_largeTextRect);

    if (m_showExplanatoryText)
    {
        std::wstringstream title;
        title << L"Home.   Start Screen\n" << m_hideTextString;

        RenderText(ctx, m_largeFormat.Get(), title.str(), m_testTitleRect);
    }
    m_newTestSelected = false;

}

bool Game::CheckHDR_On()
{
	// TODO this should query an HDR bit if one is available
	bool HDR_On = false;
	switch (m_outputDesc.ColorSpace)
	{
	case DXGI_COLOR_SPACE_RGB_FULL_G22_NONE_P709: // sRGB
		break;

	case DXGI_COLOR_SPACE_RGB_FULL_G2084_NONE_P2020: // HDR10 PC
		HDR_On = true;
		break;

	default:
		break;
	}

	return HDR_On;
}

void Game::GenerateTestPattern_ConnectionProperties(ID2D1DeviceContext2* ctx)
{
    std::wstringstream text;

    text << L"\nConnection to: ";
    WCHAR* DisplayName = wcsrchr(m_outputDesc.DeviceName, '\\');
    text << ++DisplayName;

    text << L"\nConnection bit depth: ";
    text << std::to_wstring(m_outputDesc.BitsPerColor);

    text << L"\nConnection colorspace: [";
    text << std::to_wstring(m_outputDesc.ColorSpace);
    switch (m_outputDesc.ColorSpace)
    {
    case DXGI_COLOR_SPACE_RGB_FULL_G22_NONE_P709: // sRGB
        text << L"] sRGB/SDR";
        break;

    case DXGI_COLOR_SPACE_RGB_FULL_G2084_NONE_P2020: // HDR10 PC
        text << L"] HDR/advanced color on";
        break;
    default:
        text << L"] Unknown";
        break;
    }

    bool HDR_On = CheckHDR_On();

    text << "\n\nDisplay Modes supported at this resolution and bit depth:\n";
    text << m_modeWidth << " x " << m_modeHeight;

    DXGI_FORMAT format;
    format = DXGI_FORMAT_R10G10B10A2_UNORM;     // in case this is ever possible
    if (HDR_On)
    {
        format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        text << " @ 16b/channel\n";
    }
    else
    {
        format = DXGI_FORMAT_R8G8B8A8_UNORM;
        text << " @  8b/channel\n";
    }

    // Get information about the display we are presenting to.
    ComPtr<IDXGIOutput> output;
    auto sc = m_deviceResources->GetSwapChain();
    DX::ThrowIfFailed(sc->GetContainingOutput(&output));

    UINT num = 0;
    UINT flags = 0;

    // first call to get the number of modes in this format
    output->GetDisplayModeList(format, flags, &num, 0);

    // second call collects them into an array
    DXGI_MODE_DESC* pDescs = new DXGI_MODE_DESC[num];
    output->GetDisplayModeList(format, flags, &num, pDescs);

    m_minFrameRate = 1000000;       // 1 million
    m_maxFrameRate = 0;
    for (int i = 0; i < num; i++)
    {
        if (pDescs[i].Width == m_modeWidth && pDescs[i].Height == m_modeHeight)
        {
            double rate = (double)pDescs[i].RefreshRate.Numerator / (double)pDescs[i].RefreshRate.Denominator;
            text << fixed << setw(10) << setprecision(4) << rate << "Hz\n";

            if (rate > m_maxFrameRate) m_maxFrameRate = rate;                   // scan to find min/max
            if (rate < m_minFrameRate) m_minFrameRate = rate;
        }
    }
    m_FrameRateRatio = m_maxFrameRate / m_minFrameRate;

    // TODO tag which mode is current (defaults to highest for now)

    text << "Ratio: " << fixed << setw(8) << setprecision(3) << m_FrameRateRatio << "\n";

    // print the video frame rate range supported
    text << "\nRange of refresh rates for fixed rate media playback:\n";

    IDXGISwapChainMedia *scMedia;
    DX::ThrowIfFailed(sc->QueryInterface(IID_PPV_ARGS(&scMedia)));
    UINT desiredPresentDuration = 166667;       //  60Hz

    desiredPresentDuration = 166667;       //  60Hz
    desiredPresentDuration =  83333;       // 120Hz
    desiredPresentDuration =  83333;       // 120Hz
    desiredPresentDuration =  83333;       // 120Hz
    desiredPresentDuration =  55555;       // 180Hz
    desiredPresentDuration =  41667;       // 240Hz
    desiredPresentDuration =  33333;       // 300Hz
    desiredPresentDuration =  27777;       // 360Hz
    desiredPresentDuration =  69444;       // 144Hz

    UINT closestSmallerPresentDuration, closestLargerPresentDuration;
    DX::ThrowIfFailed(scMedia->CheckPresentDurationSupport(desiredPresentDuration,
        &closestSmallerPresentDuration, &closestLargerPresentDuration ));

    // m_mediaPresentDuration

//  closestSmallerPresentDuration = desiredPresentDuration;

    double rate, duration;
    text << "Min:  " << fixed;
    duration = closestSmallerPresentDuration/10000.0;        // scale units from hundreds of nanoseconds to ms
    rate = 0.f;
    if (closestSmallerPresentDuration > 0)
        rate = 1000.f / duration;
    text << setw(10) << setprecision(4) << rate << "Hz  ";
    text << setw(10) << setprecision(4) << duration << "ms\n";

    text << "Max:  ";
    duration = closestLargerPresentDuration/10000.0;         // scale units from hundreds of nanoseconds to ms
    rate = 0.f;
    if (closestLargerPresentDuration > 0)
        rate = 1000.f / duration;
    text << setw(10) << setprecision(4) << rate << "Hz  ";
    text << setw(10) << setprecision(4) << duration << "ms\n";


    RenderText(ctx, m_monospaceFormat.Get(), text.str(), m_largeTextRect);

    if (m_showExplanatoryText)
    {
        std::wstring title = L"Connection properties:\nPress SPACE to hide this text\n";
        RenderText(ctx, m_largeFormat.Get(), title, m_testTitleRect);
    }

    m_newTestSelected = false;
}

void Game::GenerateTestPattern_PanelCharacteristics(ID2D1DeviceContext2* ctx)           //                  1
{
    std::wstringstream text;
	text << fixed << setw(9) << setprecision(2);

	// TODO: ensure these values are up to date.
	if (CheckForDefaults())
	{
		text << L"***WARNING: These are OS-provided defaults. Display did not provide levels.***\n";
	}	
	
	//	text << m_outputDesc.DeviceName;
	text << L"\nBase Levels:" << fixed;
    text << L"\n  Max peak luminance:          " << setw(8);
    text << m_rawOutDesc.MaxLuminance;
    text << L"\n  Max frame-average luminance: " << setw(8);
    text << m_rawOutDesc.MaxFullFrameLuminance;
	text << L"\n  Min luminance:                  " << setw(8)<< setprecision(5);
	text << m_rawOutDesc.MinLuminance;
	text << L"\nAdjusted Levels:";
	text << L"\n  Max peak luminance:          " << setw(8) << setprecision(2);
	text << m_outputDesc.MaxLuminance;
	text << L"\n  Max frame-average luminance: " << setw(8);
	text << m_outputDesc.MaxFullFrameLuminance;
	text << L"\n  Min luminance:                  " << setw(8) << setprecision(5);
	text << m_outputDesc.MinLuminance;
	text << L"\nCurr. Brightness Slider Factor:   " << setw(8);
	text << BRIGHTNESS_SLIDER_FACTOR;

//  text << L"\nContrast ratio (peak/min):   ";
//  text << std::to_wstring(m_outputDesc.MaxLuminance / m_outputDesc.MinLuminance );
    text << L"\n";

    // Compute area of this device's gamut in uv coordinates

    // first, get primaries of display in 1931 xy coordinates
    float2 red_xy, grn_xy, blu_xy, wht_xy;
    red_xy = m_outputDesc.RedPrimary;
    grn_xy = m_outputDesc.GreenPrimary;
    blu_xy = m_outputDesc.BluePrimary;
    wht_xy = m_outputDesc.WhitePoint;

	text << L"\nCIE 1931          x         y";
    text << L"\nRed Primary  :  " << std::to_wstring(red_xy.x) << L"  " << std::to_wstring(red_xy.y);
    text << L"\nGreen Primary:  " << std::to_wstring(grn_xy.x) << L"  " << std::to_wstring(grn_xy.y);
    text << L"\nBlue Primary :  " << std::to_wstring(blu_xy.x) << L"  " << std::to_wstring(blu_xy.y);
    text << L"\nWhite Point  :  " << std::to_wstring(wht_xy.x) << L"  " << std::to_wstring(wht_xy.y);

    float gamutAreaDevice = ComputeGamutArea(red_xy, grn_xy, blu_xy);

    // rec.709/sRGB gamut area in uv coordinates
    float gamutAreasRGB = ComputeGamutArea(primaryR_709, primaryG_709, primaryB_709);

    // AdobeRGB gamut area in uv coordinates
    float GamutAreaAdobe = ComputeGamutArea(primaryR_Adobe, primaryG_Adobe, primaryB_Adobe);

    // DCI-P3 gamut area in uv coordinates
    float gamutAreaDCIP3 = ComputeGamutArea(primaryR_DCIP3, primaryG_DCIP3, primaryB_DCIP3);

    // BT.2020 gamut area in uv coordinates
    float gamutAreaBT2100 = ComputeGamutArea(primaryR_2020, primaryG_2020, primaryB_2020);

    // ACES gamut area in uv coordinates
    float gamutAreaACES = ComputeGamutArea(primaryR_ACES, primaryG_ACES, primaryB_ACES);

    const float gamutAreaHuman = 0.195f;	// TODO get this actual data!

    // Compute extent that this device gamut covers known popular ones:
    float coverageSRGB  = ComputeGamutCoverage(red_xy, grn_xy, blu_xy, primaryR_709,   primaryG_709,   primaryB_709);
    float coverageAdobe = ComputeGamutCoverage(red_xy, grn_xy, blu_xy, primaryR_Adobe, primaryG_Adobe, primaryB_Adobe);
    float coverageDCIP3 = ComputeGamutCoverage(red_xy, grn_xy, blu_xy, primaryR_DCIP3, primaryG_DCIP3, primaryB_DCIP3);
    float coverage2100  = ComputeGamutCoverage(red_xy, grn_xy, blu_xy, primaryR_2020,  primaryG_2020,  primaryB_2020);
    float coverageACES  = ComputeGamutCoverage(red_xy, grn_xy, blu_xy, primaryR_ACES,  primaryG_ACES,  primaryB_ACES);

    // display coverage values
    text << L"\n\nGamut Coverage based on reported primaries";
    text << L"\n        % sRGB : " << std::to_wstring(coverageSRGB*100.f);
    text << L"\n    % AdobeRGB : " << std::to_wstring(coverageAdobe*100.f);
    text << L"\n      % DCI-P3 : " << std::to_wstring(coverageDCIP3*100.f);
    text << L"\n     % BT.2100 : " << std::to_wstring(coverage2100*100.f);
    text << L"\n   % Human eye : " << std::to_wstring(gamutAreaDevice / gamutAreaHuman*100.f);

    // test code:
    float theory = gamutAreaDevice / gamutAreaBT2100;
    //	text << L"\n     %  Theory : " << std::to_wstring( theory*100.f);
    //	text << L"\n       % Error : " << std::to_wstring((coverage2100 - theory) / theory * 100.f);

    RenderText(ctx, m_monospaceFormat.Get(), text.str(), m_largeTextRect);

    if (m_showExplanatoryText)
    {
        std::wstring title = L"Reported Panel Characteristics\n" + m_hideTextString;
        RenderText(ctx, m_largeFormat.Get(), title, m_testTitleRect);
	}

    m_newTestSelected = false;

    // TODO should bail here if no valid data
}

void setBrightnessSliderPercent(UCHAR percent)
{
	HANDLE display = CreateFile(
		L"\\\\.\\LCD",
		(GENERIC_READ | GENERIC_WRITE),
		NULL,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);

	if (display == INVALID_HANDLE_VALUE)
	{
		throw new std::runtime_error("Failed to open handle to display for setting brightness");
	}
	else
	{
		DWORD ret;
		DISPLAY_BRIGHTNESS displayBrightness{};
		displayBrightness.ucACBrightness = percent;
		displayBrightness.ucDCBrightness = percent;
		displayBrightness.ucDisplayPolicy = DISPLAYPOLICY_BOTH;

		bool result = !DeviceIoControl(
			display,
			IOCTL_VIDEO_SET_DISPLAY_BRIGHTNESS,
			&displayBrightness,
			sizeof(displayBrightness),
			NULL,
			0,
			&ret,
			NULL);

		if (result)
		{
			throw new std::runtime_error("Failed to set brightness");
		}
	}
}

void Game::GenerateTestPattern_ResetInstructions(ID2D1DeviceContext2* ctx)
{
	if (m_newTestSelected)
	{

// This works but we don't know what to set it to.
// So it is commented out until we do
//		setBrightnessSliderPercent(33);
// EDID settings don't update afterwards anyway until next app run
	}

    if (m_showExplanatoryText)
    {
        std::wstring title = L"Start of performance tests\n" + m_hideTextString;
        RenderText(ctx, m_largeFormat.Get(), title, m_testTitleRect);
    }
    std::wstringstream text;
    text << L"For external displays, use their OSD to reset\n";
    text << L"all settings to the default or factory state.\n\n";

	text << L"For internal panels, set the OS \'Brightness\' slider to default.\n";
	text << L"Set the \'SDR color appearance\' slider to the leftmost point.\n";
	text << L"DO NOT CHANGE BRIGHTNESS SLIDERS AFTER APP START.\n\n";

	text << L"Be sure the \'Night light' setting is off.\n";

	text << L"Disable any Ambient Light Sensor capability.\n\n";

	text << L"The display should be at normal operating temperature.\n";
    text << L"   This can take as much as 30 minutes for some panels.\n";

    text << L"   Also note, ambient temperature may affect scores.\n\n";

    RenderText(ctx, m_largeFormat.Get(), text.str(), m_largeTextRect);
    m_newTestSelected = false;
}

// Flicker test -fixed frame rates:
// Just draws the screen at about 10nits luminance in either HDR or SDR mode
// Refresh rates should be 4 options:
//    min reported, max reported, 60Hz, 24 if possible, else 48.
void Game::GenerateTestPattern_FlickerConstant(ID2D1DeviceContext2* ctx)				// 2
{
    // compute background color
    float c; 
    if (CheckHDR_On())
    {
        float nits = 10.0f;
        c = nitstoCCCS( nits );
    }
    else
    {
        // code value to attain 10nits on an SDR monitor with 200nit page white
        float sRGBval = 65;
        c = RemoveSRGBCurve(sRGBval / 255.0f);
    }

	ComPtr<ID2D1SolidColorBrush> flickerBrush;
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &flickerBrush));

    // Fill screen with a box
    auto logSize = m_deviceResources->GetLogicalSize();
	ctx->FillRectangle(&logSize, flickerBrush.Get());

    // determine what rate to use based on the up/down arrow key setting
    double refreshRate = 60.0f;
    switch (m_flickerRateIndex)
    {
    case 0:
        refreshRate = m_minFrameRate;        // min reported by implementation
        break;

    case 1:
        refreshRate = m_maxFrameRate;        // max reported by implementation
        break;

    case 2:
        refreshRate =  60.0f;                // hard coded at 60Hz
        break;

    case 3:
        if (m_minFrameRate <= 24.0)
            refreshRate = 24.0f;             // 24 if supported, else 48.
        else
            refreshRate = 48.0f;
        break;

    case 4:
        refreshRate = 120.0f;                // hard coded at 120Hz
        break;

    }
    m_targetFrameRate = refreshRate;

	if ( m_showExplanatoryText )
	{
		std::wstringstream title;
        title << L"2 Flicker at Constant Refresh Rate: \n";
        title << fixed;
        title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
        title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms";
        switch (m_flickerRateIndex)
        {
        case 0: title << " min\n";   break;
        case 1: title << " max\n";   break;
        case 2: title << " p60\n";   break;
        case 3: title << " 24/48\n"; break;
        case 4: title << " 120\n";   break;
        }
        title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
        title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms\n";

        double avgFrameTime = m_totalFrameTime / m_frameCount;
        title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
        title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
        double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
        title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";
#if 0
        // display brightness level for this test
        title << setprecision(0);
        if (CheckHDR_On())
		{
			title << L"HDR10: ";
			title << sRGBval;
			title << L"  Nits: ";
			title << sRGBval;               // TODO: set correct intensity level even in SDR mode
		}
		else
		{
			title << L"sRGB: ";
			title << sRGBval;
			title << L"  Nits: ";
			title << sRGBval;
		}
#endif
        title << L"Adjust luminance to 10nits using UI slider or OSD\n";
		title << L"Select refresh rate using Up/Down arrows\n";
        title << m_hideTextString;

        RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	m_newTestSelected = false;

}

void Game::GenerateTestPattern_FlickerVariable(ID2D1DeviceContext2* ctx)				// 3
{
    // compute background color
    float c;
    if (CheckHDR_On())
    {
        float nits = 10.0f;
        c = nitstoCCCS(nits);
    }
    else
    {
        // code value to attain 10nits on an SDR monitor with 200nit page white
        float sRGBval = 65;
        c = RemoveSRGBCurve(sRGBval / 255.0f);
    }

	ComPtr<ID2D1SolidColorBrush> flickerBrush;
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &flickerBrush));

	auto logSize = m_deviceResources->GetLogicalSize();
	ctx->FillRectangle(&logSize, flickerBrush.Get());

    if (m_showExplanatoryText)
	{
		std::wstringstream title;
		title << L"3 Flicker at Varying Refresh Rate: ";
        switch (m_waveEnum)
        {
        case WaveEnum::ZigZag:
            title << L"-Zig Zag\n";
            break;
        case WaveEnum::SquareWave:
            title << L"-Square Wave\n";
            break;
        case WaveEnum::Random:
            title << L"-Random\n";
            break;
        case WaveEnum::Sine:
            title << L"-Sine Wave\n";
            break;
        }

        title << fixed;
        title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
        title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
        title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
        title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms\n";
#if 0
        // display brightness level for this test
        title << setprecision(0);
		if (CheckHDR_On())
		{
			title << L"HDR10: ";
			title << sRGBval;
			title << L"  Nits: ";
			title << sRGBval;
		}
		else
		{
			title << L"sRGB: ";
			title << sRGBval;
			title << L"  Nits: ";
			title << sRGBval;
		}
#endif
        title << L"Adjust luminance to 10nits using UI slider or OSD\n";
		title << L"Select zigzag vs square wave etc using Up/Down arrows\n";
        title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	m_newTestSelected = false;

}

void Game::GenerateTestPattern_DisplayLatency(ID2D1DeviceContext2 * ctx) // ************************************* 4
{
    if (m_newTestSelected)
    {
        ResetFrameStats();
        ResetSensorStats();
    }

    // compute background color
    float c;
    if (CheckHDR_On())
    {
        float nits = 100.0f;
        c = nitstoCCCS(nits);
    }
    else
    {
        // code value to attain 10nits on an SDR monitor with 200nit page white
        c = 0.5;
    }

    // fill the screen background with 100 nits white
    ComPtr<ID2D1SolidColorBrush> backgroundBrush;
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &backgroundBrush));
    auto logSize = m_deviceResources->GetLogicalSize();
    ctx->FillRectangle(&logSize, backgroundBrush.Get());

    m_targetFrameRate = m_latencyTestFrameRate;

    // draw a square 50mm on a side
    c = nitstoCCCS( 200 );
    ComPtr<ID2D1SolidColorBrush> whiteBrush;
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &whiteBrush));
    
    float dpi = m_deviceResources->GetDpi();
    float boxDim = 50./25.4 * dpi * 1.2;      // 50mm -> inches -> dips
	D2D1_RECT_F tenPercentRect =
	{
		(logSize.right - logSize.left) * 0.5f - boxDim / 2.0f,
		(logSize.bottom - logSize.top) * 0.5f - boxDim / 2.0f,
		(logSize.right - logSize.left) * 0.5f + boxDim / 2.0f,
		(logSize.bottom - logSize.top) * 0.5f + boxDim / 2.0f
    };

    // draw a circle 40mm in diameter
	float2 center = float2(logSize.right*0.5f, logSize.bottom*0.5f);
    float fRad = 0.5 * 40. / 25.4 * dpi * 1.2;      // radius of dia 40mm -> inches -> dips
    D2D1_ELLIPSE ellipse =
	{
		D2D1::Point2F(center.x, center.y),
		fRad, fRad
	};

    if ( m_flash )                                // if we want to flash the photocell
    {
        ctx->FillRectangle(&tenPercentRect, m_blackBrush.Get());
        ctx->FillEllipse(&ellipse, whiteBrush.Get());
    }
    else
    {
        ctx->FillRectangle(&tenPercentRect, whiteBrush.Get());
        ctx->FillEllipse(&ellipse, m_blackBrush.Get());
    }

    if (m_showExplanatoryText)
    {
        std::wstringstream title;

        title << L"4 Display Latency Measurement\n";

        if ( m_sensing )
        {
            // print frame rate we were targeting for this test (not sampling rate)
            title << fixed;
            title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
            title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
            title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
            title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms\n";

            int count = m_sensorCount;
            if (count <= 0) count = 1;
            double avgSensorTime = m_totalSensorTime / count;
            double varSensorTime = sqrt(count * m_totalSensorTime2 - m_totalSensorTime * m_totalSensorTime) / count;

            if (m_sensorConnected)
            {
                // print total end-to-end latency stats from sensor
                title << " S E N S I N G . . .\n";
                title << "Over last " << m_sensorCount << " samples\n";
                title << "Average:   " << setprecision(3) << setw(7);
                title << avgSensorTime * 1000.0;
                title << "ms\n";
                title << "Min:" << setprecision(3) << setw(7);
                title << m_minSensorTime * 1000.0;
                title << "  Max:" << setprecision(3) << setw(7);
                title << m_maxSensorTime * 1000.0;
                title << "  Var:" << setprecision(3) << setw(6);
                title << varSensorTime * 1000.0;
            }
            else
                title << "\n    Please Attach a photocell sensor.\n";
        }
        else
        {
            title << fixed;
            title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
            title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
            title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
            title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms";
            title << L"                          avg     var\n";

            int numCols = 6;
            assert(numCols <= frameLogLength);

            double dFrequency = m_qpcFrequency.QuadPart;    // counts per second of QPC

            // Display frame id counter
            title << "Frame ID:  ";
            for (int i = numCols - 1; i >= 0; i--)
            {
                title << setw(8) << m_frameLog[i]->frameID;
            }
            // show number of frames average is over
            title << setw(8) << m_frameCount;
//          title << "Average of " << m_frameCount << " frames: " << fixed << setprecision(3);

            // Print entire loop end-to-end intervals
            title << "\nFrame Time:" << setprecision(3);
            for (int i = numCols - 1; i > 0; i--)
            {
                double time = (m_frameLog[i - 1]->readCounts - m_frameLog[i]->readCounts) / dFrequency;
                title << setw(8) << time * 1000.0f;
            }
            title << "        ";
            double avgFrameTime = m_totalFrameTime / m_frameCount;
            double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
            title << setprecision(3) << setw(8) << avgFrameTime * 1000.;
            title << setprecision(3) << setw(8) << varFrameTime * 1000.;

            // Print time spent in Input Stack
            title << "\nInput lag: " << setprecision(3);
            for (int i = numCols - 1; i >= 0; i--)
            {
                double time = (m_frameLog[i]->readCounts - m_frameLog[i]->clickCounts) / dFrequency;
                title << setw(8) << time * 1000.0f;
            }

            // Print time spent in App CPU code
            title << "\nApp work:  " << setprecision(3);
            for (int i = numCols - 1; i >= 0; i--)
            {
                double time = (m_frameLog[i]->drawCounts - m_frameLog[i]->readCounts) / dFrequency;
                title << setw(8) << time * 1000.0f;
            }
            double avgAppTime = m_totalRenderTime / m_frameCount;
            double varAppTime = sqrt(m_frameCount * m_totalAppTime2 - m_totalAppTime * m_totalAppTime) / m_frameCount;
            title << setprecision(3) << setw(8) << avgAppTime * 1000.f;
            title << setprecision(3) << setw(8) << varAppTime * 1000.f;

            // Print time to render on GPU
            title << "\nRender:    " << setprecision(3);
            for (int i = numCols - 1; i >= 1; i--)
            {
                double time = (m_frameLog[i]->presentCounts - m_frameLog[i]->drawCounts) / dFrequency;
                title << setw(8) << time * 1000.0f;
            }
            title << "        ";
            double avgRenderTime = m_totalRenderTime / m_frameCount;
            double varRenderTime = sqrt(m_frameCount*m_totalRenderTime2 - m_totalRenderTime*m_totalRenderTime )/m_frameCount;
            title << setprecision(3) << setw(8) << avgRenderTime * 1000.f;
            title << setprecision(3) << setw(8) << varRenderTime * 1000.f;

            // Print time to finalize image and flip -time from Present() to Vsync
            title << "\nPresent:   " << setprecision(3);
            for (int i = numCols - 1; i >= 2; i--)
            {
                double time = (m_frameLog[i]->syncCounts - m_frameLog[i]->presentCounts) / dFrequency;
                title << setw(8) << time * 1000.0f;
            }
            title << "                ";
            double avgPresentTime = m_totalPresentTime / m_frameCount;
            double varPresentTime = sqrt(m_frameCount*m_totalPresentTime2 - m_totalPresentTime*m_totalPresentTime )/m_frameCount;
            title << setprecision(3) << setw(8) << avgPresentTime * 1000.f;
            title << setprecision(3) << setw(8) << varPresentTime * 1000.f;

            // Compute Display Latency
            double avgSensorTime = m_totalSensorTime / m_sensorCount;
            double avgDisplayTime = avgSensorTime - avgPresentTime;

            title << "\nDisplay:   " << setprecision(3);
            title << setw(8) << avgDisplayTime * 1000.0f;
            title << "ms or " << setprecision(3);
            title << setw(6) << avgDisplayTime / avgFrameTime;
            title << " frames";

            // print out total e2e experience latency
            double e2eTime = m_avgInputTime + avgFrameTime + avgPresentTime + avgDisplayTime;
            title << "\nTotal E2E: " << setprecision(3);
            title << setw(8) << e2eTime * 1000.0f;
            title << "ms or " << setprecision(3);
            title << setw(6) << e2eTime / avgFrameTime;
            title << " frames";

#if 0
            // print the running average of frame times
            title << "Average of last 9 frames: " << setprecision(3) << setw(9);
            uint count = 0;
            double avgTime = 0.0;
            for (int i = frameLogLength - 1; i >= 1; i--)
            {
                double time = (m_frameLog[i]->photonCounts - m_frameLog[i]->clickCounts) / dFrequency;
                if (time < 0.10)
                {
                    avgTime += time;
                    count++;
                }
            }
            title << avgTime / count * 1000.0f;
#endif

        }

        title << "\nPress S to toggle Sensor sampling";
        title << "\nPress R to Reset stat counters";
        title << "\n" << m_hideTextString;

        RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect, true);

    }
    m_newTestSelected = false;

}


void Game::GenerateTestPattern_GrayToGray(ID2D1DeviceContext2 * ctx) //**************** 5.
{
    // compute brush for surround - 
    float c;
    if (!CheckHDR_On())                         // SDR
    {
        // per CTS section 10.2
        float sRGBval = 187.f;       // in 8-bit encoding
        c = sRGBval / 255.0f;
    }
    else                                        // HDR
    {
        // should turn out to be 520/1023 in PQ/2084 code
        float nits = 100.0f;
        c = nitstoCCCS(nits);
    }
    ComPtr<ID2D1SolidColorBrush> surroundBrush;                 // background
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &surroundBrush));

    // draw background
    auto logSize = m_deviceResources->GetLogicalSize();
    ctx->FillRectangle(&logSize, surroundBrush.Get());

    // decide color for foreground patch
    c = m_color;    // computed in this pattern's Update method
    ComPtr<ID2D1SolidColorBrush> testBrush;                     // 10% area test square
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &testBrush));

    float refreshRate = 60.0f;                  // this test is hard-coded to 60Hz
    m_targetFrameRate = refreshRate;

//  float dpi = m_deviceResources->GetDpi();

    // draw test pattern
    float size = sqrt((logSize.right - logSize.left) * (logSize.bottom - logSize.top));
    size = size * sqrtf(0.10);              // dimension of a square of 10% screen area
    float2 center;
    center.x = (logSize.right - logSize.left) * 0.50f;
    center.y = (logSize.bottom - logSize.top) * 0.50f;
    D2D1_RECT_F tenPercentRect =
    {
        center.x - size * 0.50f,
        center.y - size * 0.50f,
        center.x + size * 0.50f,
        center.y + size * 0.50f
    };
    ctx->FillRectangle(&tenPercentRect, testBrush.Get());
    
	// Everything below this point should be hidden during actual measurements.
	if (m_showExplanatoryText)
	{
		std::wstringstream title;
		title << L"5 Gray To Gray:\n";
        title << fixed;
        title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
        title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
        title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
        title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms\n";
        if ( m_autoG2G )
        { 
            title << L"\nAutomatic sequence:\n";
        }
        else
        {
            title << L"Select 'From' brightness level using ,/. aka </> keys\n";
            title << L"Select 'To' brightness level using Up/Down arrows\n";
        }
        if (!CheckHDR_On())
        {
            title << "From:" << setw(9) << setprecision(4) << GrayToGrayValue(m_g2gFromIndex) * 100.f << "%";
            title << "  To:" << setw(9) << setprecision(4) << GrayToGrayValue(m_g2gToIndex) * 100.f << "%\n";
        }
        else
        {
            title << "From:" << setw(9) << setprecision(3) << GrayToGrayValue(m_g2gFromIndex)*80.f << "nits";
            title << "  To:" << setw(9) << setprecision(3) << GrayToGrayValue(m_g2gToIndex)*80.f << "nits\n";
        }
        title << "Press A to toggle Automatic sequence\n";
        title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect, true );
	}
	m_newTestSelected = false;

}
void Game::GenerateTestPattern_FrameDrop(ID2D1DeviceContext2* ctx)	//********************** 6
{
    D2D1_RECT_F logSize = m_deviceResources->GetLogicalSize();
    ComPtr<ID2D1SolidColorBrush> whiteBrush;			// brush for the "white" color

    float nits = m_outputDesc.MaxLuminance;             // for metadata

    float HDR10 = 150;
    nits = Remove2084(HDR10 / 1023.0f) * 10000.0f;		// "white" checker brightness

    float c = nitstoCCCS(nits) / BRIGHTNESS_SLIDER_FACTOR;
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &whiteBrush));

    float refreshRate = 60.0f;

    // decide shape of grid for test pattern
    int nCols = 10;
    int nRows = 6;
    switch (m_frameDropRateIndex)       // logic re frame rate should move into Update(); logic re tile dimensions could stay here.
    {
    case 0:
        refreshRate = 24.0f;
        nCols = 6;
        nRows = 4;
        break;

    case 1:
        refreshRate = 30.0f;
        nCols = 6;
        nRows = 5;
        break;

    case 2:
        refreshRate = 48.0f;
        nCols = 8;
        nRows = 6;
        break;

    case 3:
        refreshRate = 60.0f;
        nCols = 10;
        nRows = 6;
        break;

    case 4:
        refreshRate = 90.0f;
        nCols = 10;
        nRows =  9;
        break;

    case 5:
        refreshRate = 120.0f;
        nCols = 12;
        nRows = 10;
        break;

    case 6:
        refreshRate = 180.0f;
        nCols = 15;
        nRows = 12;
        break;

    case 7:
        refreshRate = 240.0f;
        nCols = 16;
        nRows = 15;
        break;

    default:
        refreshRate = 60.0f;
        nCols = 10;
        nRows = 6;
        break;
    }
    m_targetFrameRate = refreshRate;

    // draw a checkerboard background for reference
    float2 step;
    step.x = (logSize.right - logSize.left) / nCols;
    step.y = (logSize.bottom - logSize.top) / nRows;

#ifdef DRAW_CHECKER_BACKGROUND
    for (int jRow = 0; jRow < nRows; jRow++)
    {
        for (int iCol = 0; iCol < nCols; iCol++)
        {
            if ((iCol + jRow) & 0x01)
            {
                D2D1_RECT_F rect =
                {
                    iCol * step.x,
                    jRow * step.y,
                    (iCol + 1) * step.x,
                    (jRow + 1) * step.y
                };
                ctx->FillRectangle(&rect, whiteBrush.Get());
            }
        }
    }
#endif

    // now draw the current square:
    c = 1.0f;
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &whiteBrush));

    int iCol = m_frameCounter % nCols;
    int jRow = (m_frameCounter / nCols) % nRows;

    D2D1_RECT_F rect =
    {
        iCol * step.x,
        jRow * step.y,
        (iCol + 1) * step.x,
        (jRow + 1) * step.y
    };
    ctx->FillRectangle(&rect, whiteBrush.Get());

    // Everything below this point should be hidden during actual measurements.
    if (m_showExplanatoryText)
    {
        std::wstringstream title;
        title << L"6 Frame Drop Test\n";
        title << fixed;
        title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
        title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
        title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
        title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms\n";
        double avgFrameTime = m_totalFrameTime / m_frameCount;
        title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
        title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
        double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
        title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";
        title << "Grid " << nRows << " x " << nCols;
        title << L"\nSelect refresh rate using Up/Down arrows\n";
        title << m_hideTextString;

        RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
    }
    m_newTestSelected = false;

}

void Game::GenerateTestPattern_FrameLock(ID2D1DeviceContext2 * ctx)	//********************** 7
{
	D2D1_RECT_F logSize = m_deviceResources->GetLogicalSize();
	ComPtr<ID2D1SolidColorBrush> whiteBrush;			// brush for the "white" color

	float nits = m_outputDesc.MaxLuminance;             // for metadata

	float HDR10 = 150;
	nits = Remove2084( HDR10/1023.0f) * 10000.0f;		// "white" checker brightness

	float c = nitstoCCCS(nits)/BRIGHTNESS_SLIDER_FACTOR;
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &whiteBrush));

    float refreshRate = 60.0f;

    // decide shape of grid for test pattern
    int nCols = 10;
    int nRows = 6;
    switch (m_frameLockRateIndex)
    {
    case 0:
        refreshRate =  23.976f;
        nCols =  6;
        nRows =  4;
        break;

    case 1:
        refreshRate =  24.0f;
        nCols = 6;
        nRows = 4;
        break;

    case 2:
        refreshRate =  25.0f;
        nCols =  5;
        nRows =  5;
        break;

    case 3:
        refreshRate =  29.97f;
        nCols =  6;
        nRows =  5;
        break;

    case 4:
        refreshRate =  30.0f;
        nCols =  6;
        nRows =  5;
        break;

    case 5:
        refreshRate =  47.952f;
        nCols = 8;
        nRows = 6;
        break;

    case 6:
        refreshRate =  48.0f;
        nCols = 8;
        nRows = 6;
        break;

    case 7:
        refreshRate = 50.0f;
        nCols = 8;
        nRows = 6;
        break;

    case 8:
        refreshRate = 59.94f;
        nCols = 10;
        nRows = 6;
        break;

    case 9:
        refreshRate = 60.0f;
        nCols = 10;
        nRows = 6;
        break;
/*
    case 10:
        refreshRate = 120.0f;
        nCols = 12;
        nRows = 10;
        break;

    case 11:
        refreshRate = 180.0f;
        nCols = 15;
        nRows = 12;
        break;

    case 12:
        refreshRate = 240.0f;
        nCols = 16;
        nRows = 15;
        break;
*/
    default:
        refreshRate = 60.0f;
        nCols = 10;
        nRows = 6;
        break;
    }

    m_targetFrameRate = refreshRate;

	// draw a checkerboard background for reference
	float2 step;
	step.x = (logSize.right - logSize.left) / nCols;
	step.y = (logSize.bottom - logSize.top) / nRows;

#ifdef DRAW_CHECKER_BACKGROUND
    for (int jRow = 0; jRow < nRows; jRow++)
    {
        for (int iCol = 0; iCol < nCols; iCol++)
        {
            if ((iCol + jRow) & 0x01)
            {
                D2D1_RECT_F rect =
                {
                    iCol * step.x,
                    jRow * step.y,
                    (iCol + 1) * step.x,
                    (jRow + 1) * step.y
                };
                ctx->FillRectangle(&rect, whiteBrush.Get());
            }
        }
    }
#endif

    // now draw the current square:
    c = 1.0f;
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &whiteBrush));
    
    int iCol = m_frameCounter % nCols;
    int jRow = (m_frameCounter / nCols) % nRows;

    D2D1_RECT_F rect =
    {
        iCol * step.x,
        jRow * step.y,
        (iCol + 1) * step.x,
        (jRow + 1) * step.y
    };
    ctx->FillRectangle(&rect, whiteBrush.Get());

	// Everything below this point should be hidden during actual measurements.
	if (m_showExplanatoryText)
	{
		float fRad = sqrt((logSize.right - logSize.left) * (logSize.bottom - logSize.top)*0.04f);	// 4% screen area colorimeter box
		float2 center = float2(logSize.right*0.5f, logSize.bottom*0.5f);

		std::wstringstream title;
		title << L"7 Framerate Lock or Jitter Test\n";
        title << fixed;
        title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
        title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
        title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
        title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms\n";
        double avgFrameTime = m_totalFrameTime / m_frameCount;
        title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
        title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
        double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
        title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";
        title << "Grid " << nRows << " x " << nCols;

        title << L"\nSelect refresh rate using Up/Down arrows\n";
        title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

    m_newTestSelected = false;

}

void Game::GenerateTestPattern_EndOfMandatoryTests(ID2D1DeviceContext2* ctx)
{
    std::wstringstream text;
    text << L"This is the end of mandatory test content.\n";
    text << L"Optional/informative tests follow.";

    RenderText(ctx, m_largeFormat.Get(), text.str(), m_largeTextRect);
    m_newTestSelected = false;
}

void Game::GenerateTestPattern_MotionBlur(ID2D1DeviceContext2* ctx) // ********************** 8.
{
    float refreshRate = 60.0f;      // simulate 60Hz video on Netflix or Youtube
    m_targetFrameRate = refreshRate;

    // get window dimensions in pixels
    auto logSize = m_deviceResources->GetLogicalSize();
    float2 center = float2(logSize.right - logSize.left, logSize.bottom - logSize.top) * 0.5f;

    // fill background
    ctx->FillRectangle(&logSize, m_blackBrush.Get());

    // compute rotation angle of bar in degrees for D2D
    float fDeltaAngle = 360.0f * m_frameTime * 1.0f;// assume 2 revolution per second
    m_fAngle += fDeltaAngle;                        // predict next frame time from this one

#define NUM_BLUR_FRAMES 32
    int numBlurFrames = NUM_BLUR_FRAMES;

    float c = 1.0f;                                         // color for white
    float a = 1.0 / (float)numBlurFrames;                   // alpha channel for additive transparency
    ComPtr<ID2D1SolidColorBrush> spinBrush;                 // for the spinner
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c, a), &spinBrush));
    ctx->SetPrimitiveBlend(D2D1_PRIMITIVE_BLEND_ADD);

    float fWidth = min(center.x, center.y);     // make sure it fits on screen
    float fHeight = fWidth * 0.05f;             // aspect ratio of 20:1
    D2D1_RECT_F spinRect =
    {
        -fWidth, -fHeight, fWidth, fHeight
    };

    // what part of the frame time this panel is lit for
    float FrameFraction = (float)m_MotionBlurIndex/(float)maxMotionBlurs;

    // Draw the rectangle NumBlurFrames times at slightly different angles to blur it
    for ( int i = 0; i < numBlurFrames; i++)
    {
        float fDel = -fDeltaAngle * FrameFraction / (float)numBlurFrames;

        ctx->SetTransform(
            D2D1::Matrix3x2F::Rotation( m_fAngle + fDel*i ) * 
            D2D1::Matrix3x2F::Translation(center.x, center.y)
        );

        ctx->FillRectangle(&spinRect, spinBrush.Get());

        // clear transform after drawing
        ctx->SetTransform(D2D1::Matrix3x2F::Identity());
    }

    // draw the pivot
    float fRad = fHeight * 0.5;
    D2D1_ELLIPSE ellipse =
    {
        D2D1::Point2F(center.x, center.y),
        fRad, fRad
    };
    ctx->SetPrimitiveBlend(D2D1_PRIMITIVE_BLEND_COPY);
    ctx->FillEllipse(&ellipse, m_blackBrush.Get());

    if (m_showExplanatoryText)
    {
        std::wstringstream title;
        title << L"8 Motion Blur Tuning\n";
        title << fixed;
        title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
        title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
        title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
        title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms\n";

        title << "Frame Fraction: ";
        title << fixed << setw(6) << setprecision(2) << FrameFraction;  // in Percent?
        title << L"\nSelect Frame Fraction using Up/Down arrows\n";
        title << m_hideTextString;

        RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
    }

    m_newTestSelected = false;

}

void Game::GenerateTestPattern_GameJudder(ID2D1DeviceContext2* ctx) // ********************** 9.
{
    float nits = 100;

    bool bBFI = true;
    if (m_frameTime > 0.025 ||         // will flicker if too slow at under 50ms (40Hz)
        m_frameTime < 0.01666666)      // no time for a frame to fit if too fast (>1/2 refresh rate)
    {
        bBFI = false;
    }

    if ( bBFI ) nits *= 2.0f;       // increase it if we are drawing black frames
    // should scale this based on duration of black frame

    m_targetFrameRate = m_judderTestFrameRate;

    float c = nitstoCCCS(nits);
    ComPtr<ID2D1SolidColorBrush> spinnerBrush;
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &spinnerBrush));

    // get window dimensions in pixels
    auto logSize = m_deviceResources->GetLogicalSize();
    float2 center = float2(logSize.right - logSize.left, logSize.bottom - logSize.top) * 0.5f;

    // fill background
    ctx->FillRectangle(&logSize, m_blackBrush.Get());

    float fDeltaAngle = 360.0f * m_frameTime * 1.0f;// assume 2 revolutions per second
    m_fAngle += fDeltaAngle;                        // predict next frame time from this one

    // skip drawing it every other frame as a black frame
    if ( (m_frameCounter & 0x01) || (!bBFI ) )
    {
        ctx->SetTransform(
            D2D1::Matrix3x2F::Rotation(m_fAngle) *
            D2D1::Matrix3x2F::Translation(center.x, center.y)
        );

        float fWidth = min(center.x, center.y);     // make sure it fits on screen
        float fHeight = fWidth * 0.05;              // aspect ratio of 20:1

        D2D1_RECT_F spinRect =
        {
            -fWidth, -fHeight, fWidth, fHeight
        };
        ctx->FillRectangle(&spinRect, spinnerBrush.Get());

        // clear transform after drawing
        ctx->SetTransform(D2D1::Matrix3x2F::Identity());

        // draw the pivot
        float fRad = fHeight * 0.5;
        D2D1_ELLIPSE ellipse =
        {
            D2D1::Point2F(center.x, center.y),
            fRad, fRad
        };
        ctx->FillEllipse(&ellipse, m_blackBrush.Get());
    }

    if (m_showExplanatoryText)
    {
        double refreshRate = m_targetFrameRate;

        std::wstringstream title;
        title << L"9 Game Judder Removal";
        if (bBFI)
            title << "  BFI";
        title << "\nTarget:  ";
        title << fixed << setw(10) << setprecision(3);
        title << refreshRate << L"Hz   " << 1.0f / refreshRate * 1000.f << L"ms\n";
        title << "Current: ";
        title << fixed << setw(10) << setprecision(3);
        title << 1.0 / m_frameTime << L"Hz   " << m_frameTime * 1000.f << L"ms\n";

        title << L"\nSelect App Work time using Up/Down arrows\n";
        title << m_hideTextString;

        RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
    }
    m_newTestSelected = false;
}

void Game::GenerateTestPattern_Tearing(ID2D1DeviceContext2* ctx) // ********************** 0. T.
{
    // get window dimensions in pixels
    auto logSize = m_deviceResources->GetLogicalSize();

    // fill background
    ctx->FillRectangle(&logSize, m_blackBrush.Get());

    // compute rectangle size
    float fHeight = (logSize.bottom - logSize.top);
    float fWidth  = (logSize.right - logSize.left);

    m_targetFrameRate = m_tearingTestFrameRate;

    double duration = 0.250;                   // time to sweep L -> R across screen in seconds

    // how far to move the bar each refresh:
    double selectedFrameTime = 1.0 / m_targetFrameRate;
    double pixPerFrame = fWidth * selectedFrameTime / duration;
    int nCols = round(duration / selectedFrameTime);

    // move the bar over:
    m_sweepPos += pixPerFrame;
    if (m_sweepPos > logSize.right)
        m_sweepPos = 0.0;

    D2D1_RECT_F tearingRect =
    {
        m_sweepPos, 0., m_sweepPos + (float)pixPerFrame, fHeight
    };
    ctx->FillRectangle(&tearingRect, m_whiteBrush.Get());

    if (m_showExplanatoryText)
    {
        double refreshRate = m_targetFrameRate;

        std::wstringstream title;
        title << L"0 Tearing Check\n";
        title << fixed;
        title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
        title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
        title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
        title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms\n";
        double avgFrameTime = m_totalFrameTime / m_frameCount;
        title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
        title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
        double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
        title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";
        title << "Grid " << nCols;

        title << L"\nSelect frame duration using Up/Down arrows\n";
        title << m_hideTextString;

        RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
    }
    m_newTestSelected = false;
}

float3 roundf3(float3 in)
{
	float3 out;
	if (in.x < 0) in.x = 0;
	if (in.y < 0) in.y = 0;
	if (in.z < 0) in.z = 0;

	out.x = roundf(in.x);
	out.y = roundf(in.y);
	out.z = roundf(in.z);
	return out;
}

void Game::GenerateTestPattern_EndOfTest(ID2D1DeviceContext2* ctx)
{
    std::wstringstream text;
    text << L"This is the end of the test content.\n";
    text << L"Press ALT-F4 to quit.";

    RenderText(ctx, m_largeFormat.Get(), text.str(), m_largeTextRect);
    m_newTestSelected = false;

}

void Game::GenerateTestPattern_WarmUp(ID2D1DeviceContext2* ctx)
{
    float nits = 180.0f; // warm up level
    float c = nitstoCCCS(nits) / BRIGHTNESS_SLIDER_FACTOR;

    ComPtr<ID2D1SolidColorBrush> peakBrush;
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &peakBrush));

    auto logSize = m_deviceResources->GetLogicalSize();
    ctx->FillRectangle(&logSize, peakBrush.Get());

    if (m_showExplanatoryText)
    {
        std::wstringstream title;
        title << L"Warm-Up: ";
        title << fixed << setw(8) << setprecision(2);
        if (0.0f != m_testTimeRemainingSec)
        {
            title << m_testTimeRemainingSec;
            title << L" seconds remaining";
            title << L"\nNits: ";
            title << nits;
            title << L"  HDR10: ";
            title << setprecision(0);
            title << Apply2084(c * 80.f / 10000.f) * 1023.f;
            title << L"\n" << m_hideTextString;
        }
        else
        {
            title << L" done.";
        }

        RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect, true);
    }
    m_newTestSelected = false;
}


// this is invoked via the C key
// it just draws a black screen for 60seconds.
void Game::GenerateTestPattern_Cooldown(ID2D1DeviceContext2* ctx)       //              'C'
{
    if (m_showExplanatoryText)
    {
        std::wstringstream title;
        title << L"Cool-down: ";
        if (0.0f < m_testTimeRemainingSec)
        {
            //          title << static_cast<unsigned int>(m_testTimeRemainingSec);
            title << fixed << setw(12) << setprecision(3);
            title << m_testTimeRemainingSec << L" seconds remaining\n";
            title << fixed << setw(12) << setprecision(3);
            title << 1.0 / m_frameTime << L"FPS\n";
            title << fixed << setw(12) << setprecision(3);
            title << m_frameTime * 1000.f << L"ms\n";
            title << m_hideTextString;
        }
        else
        {
            title << L" done.";
        }

        RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
    }
    m_newTestSelected = false;

}

#pragma endregion

#pragma region Frame Render
// Draws the scene.
void Game::Render()
{
    m_deviceResources->PIXBeginEvent(L"Render");

    Clear();

    auto ctx = m_deviceResources->GetD2DDeviceContext();

    ctx->BeginDraw();

    // Do test pattern-specific rendering here.
    // RenderD2D() handles all operations that are common to all test patterns:
    // 1. BeginDraw/EndDraw
    switch (m_currentTest)
    {
    case TestPattern::StartOfTest:
        GenerateTestPattern_StartOfTest(ctx);
        break;
    case TestPattern::ConnectionProperties:
        GenerateTestPattern_ConnectionProperties(ctx);
        break;
    case TestPattern::PanelCharacteristics:								// 1
        GenerateTestPattern_PanelCharacteristics(ctx);
        break;
    case TestPattern::ResetInstructions:
        GenerateTestPattern_ResetInstructions(ctx);
        break;
    case TestPattern::FlickerConstant:									// 2
        GenerateTestPattern_FlickerConstant(ctx);
        break;
	case TestPattern::FlickerVariable:									// 3
		GenerateTestPattern_FlickerVariable(ctx);
		break;
	case TestPattern::DisplayLatency:									// 4
		GenerateTestPattern_DisplayLatency(ctx);
		break;
	case TestPattern::GrayToGray:										// 5
        GenerateTestPattern_GrayToGray(ctx);
        break;
    case TestPattern::FrameDrop:										// 6
        GenerateTestPattern_FrameDrop(ctx);
        break;
	case TestPattern::FrameLock:										// 7
		GenerateTestPattern_FrameLock(ctx);
		break;
    case TestPattern::EndOfMandatoryTests:								// 
        GenerateTestPattern_EndOfMandatoryTests(ctx);
        break;
	case TestPattern::MotionBlur:										// 8
		GenerateTestPattern_MotionBlur(ctx);
		break;
    case TestPattern::GameJudder:										// 9
        GenerateTestPattern_GameJudder(ctx);
        break;
    case TestPattern::Tearing:							    			// 0
        GenerateTestPattern_Tearing(ctx);
        break;
    case TestPattern::EndOfTest:
        GenerateTestPattern_EndOfTest(ctx);
        break;
    case TestPattern::WarmUp:                                           // W
        GenerateTestPattern_WarmUp(ctx);
        break;
    case TestPattern::Cooldown:                                         // C
        GenerateTestPattern_Cooldown(ctx);
        break;
    default:
        DX::ThrowIfFailed(E_NOTIMPL);
        break;
    }

    // Ignore D2DERR_RECREATE_TARGET here. This error indicates that the device
    // is lost. It will be handled during the next call to Present.
    HRESULT hr = ctx->EndDraw();
    if (hr != D2DERR_RECREATE_TARGET)
    {
        DX::ThrowIfFailed(hr);
    }

    m_deviceResources->PIXEndEvent();

    // moved Present() call out of Render() method so it can be 'timed' independently
}

// Helper method to clear the back buffers.
void Game::Clear()
{
    m_deviceResources->PIXBeginEvent(L"Clear");

    // Clear the views.
    auto context = m_deviceResources->GetD3DDeviceContext();
    auto renderTarget = m_deviceResources->GetRenderTargetView();
    auto depthStencil = m_deviceResources->GetDepthStencilView();

    context->ClearRenderTargetView(renderTarget, Colors::Black);
    context->ClearDepthStencilView(depthStencil, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

    context->OMSetRenderTargets(1, &renderTarget, depthStencil);

    // Set the viewport.
    auto viewport = m_deviceResources->GetScreenViewport();
    context->RSSetViewports(1, &viewport);

    m_deviceResources->PIXEndEvent();
}

// Note: textPos is interpreted as { Left, Top, ***Width***, ***Height*** } which is different from the
// definition of D2D1_RECT_F
void Game::RenderText(ID2D1DeviceContext2* ctx, IDWriteTextFormat* fmt, std::wstring text, D2D1_RECT_F textPos, bool useBlackText /* = false */)
{
    auto fact = m_deviceResources->GetDWriteFactory();
    ComPtr<IDWriteTextLayout> layout;
    DX::ThrowIfFailed(fact->CreateTextLayout(
        text.c_str(),
        (unsigned int)text.length(),
        fmt,
        textPos.right,
        textPos.bottom,
        &layout));

    if (useBlackText)
    {
        ComPtr<ID2D1SolidColorBrush> blackBrush;
        DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(0.0f, 0.0f, 0.0f), &blackBrush));
        ctx->DrawTextLayout(D2D1::Point2F(textPos.left, textPos.top), layout.Get(), blackBrush.Get());
    }
    else
    {
        ctx->DrawTextLayout(D2D1::Point2F(textPos.left, textPos.top), layout.Get(), m_whiteBrush.Get());
    }
}
#pragma endregion

#pragma region Message Handlers
// Message handlers
void Game::OnActivated()
{
    // TODO: Game is becoming active window.
}

void Game::OnDeactivated()
{
    // TODO: Game is becoming background window.
}

void Game::OnSuspending()
{
    // TODO: Game is being power-suspended (or minimized).
}

void Game::OnResuming()
{
//  m_timer.ResetElapsedTime();         // RESET QPC here!!!!

    // TODO: Game is being power-resumed (or returning from minimize).
}

void Game::OnWindowSizeChanged(int width, int height)
{
    // Window size changed also corresponds to switching monitors.
    m_dxgiColorInfoStale = true;

    if (!m_deviceResources->WindowSizeChanged(width, height))
        return;

    CreateWindowSizeDependentResources();
}

// Currently only updates DXGI_OUTPUT_DESC1 state.
void Game::OnDisplayChange()
{
    // Wait until the next Update call to refresh state.
    m_dxgiColorInfoStale = true;
}

// Properties
void Game::GetDefaultSize(int& width, int& height) const
{
    width = 1280;
    height = 720;
}
#pragma endregion

#pragma region Direct3D Resources
void Game::CreateDeviceIndependentResources()
{
    auto dwFactory = m_deviceResources->GetDWriteFactory();

    DX::ThrowIfFailed(dwFactory->CreateTextFormat(
        L"Segoe UI",
        nullptr,
        DWRITE_FONT_WEIGHT_NORMAL,
        DWRITE_FONT_STYLE_NORMAL,
        DWRITE_FONT_STRETCH_NORMAL,
        14.0f,
        L"en-US",
        &m_smallFormat));

    DX::ThrowIfFailed(dwFactory->CreateTextFormat(
        L"Consolas",
        nullptr,
        DWRITE_FONT_WEIGHT_NORMAL,
        DWRITE_FONT_STYLE_NORMAL,
        DWRITE_FONT_STRETCH_NORMAL,
        18.0f,
        L"en-US",
        &m_monospaceFormat));

    DX::ThrowIfFailed(dwFactory->CreateTextFormat(
        L"Segoe UI",
        nullptr,
        DWRITE_FONT_WEIGHT_NORMAL,
        DWRITE_FONT_STYLE_NORMAL,
        DWRITE_FONT_STRETCH_NORMAL,
        24.0f,
        L"en-US",
        &m_largeFormat));

}

// These are the resources that depend on the device.
void Game::CreateDeviceDependentResources()
{
    auto ctx = m_deviceResources->GetD2DDeviceContext();

    // these standard D2D values are really only valid for SDR mode!  TODO: make work in HDR
    DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::White, 1.f), &m_whiteBrush));
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Black, 1.f), &m_blackBrush));
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red,   1.f), &m_redBrush));

    for (auto it = m_testPatternResources.begin(); it != m_testPatternResources.end(); it++)
    {
        LoadTestPatternResources(&it->second);
    }

    UpdateDxgiColorimetryInfo();

	// Try to guess the testing tier that we are trying against
	m_testingTier = GetTestingTier();

}

// Allocate all memory resources that change on a window SizeChanged event.
void Game::CreateWindowSizeDependentResources()
{
    // Images are not scaled for window size - they are preserved at 1:1 pixel size.

    // D2D1_RECT_F is defined as: Left, Left, Right, Bottom
    // But we will interpret the struct members as: Left, Left, Width, Height
    // This lets us pack the size (for IDWriteTextLayout) and offset (for DrawText)
    // into a single struct.
    m_testTitleRect = { 0.0f, 0.0f, 800.0f, 100.0f };

	auto logicalSize = m_deviceResources->GetLogicalSize();

    m_largeTextRect =
    {
        (logicalSize.right - logicalSize.left) * 0.15f,
        (logicalSize.bottom - logicalSize.top) * 0.15f,
        (logicalSize.right - logicalSize.left) * 0.75f,
        (logicalSize.bottom - logicalSize.top) * 0.75f
    };

	m_MetadataTextRect =
	{
		logicalSize.right - 700.0f,
		logicalSize.bottom - 35.0f,
		logicalSize.right,
		logicalSize.bottom
	};
}

// This loads both device independent and dependent resources for the test pattern.
// Relies on wicSource and d2dSource being nullptr to (re)load resources.
void Game::LoadTestPatternResources(TestPatternResources* resources)
{
    LoadImageResources(resources);
    LoadEffectResources(resources);
}

void Game::LoadImageResources(TestPatternResources* resources)
{
    auto wicFactory = m_deviceResources->GetWicImagingFactory();
    auto ctx = m_deviceResources->GetD2DDeviceContext();

    // This test involves an image file.
    if (resources->imageFilename.compare(L"") != 0)
    {
        // First, ensure that there is a WIC source (device independent).
        if (resources->wicSource == nullptr)
        {
            ComPtr<IWICBitmapDecoder> decoder;
            HRESULT hr = wicFactory->CreateDecoderFromFilename(
                resources->imageFilename.c_str(),
                nullptr,
                GENERIC_READ,
                WICDecodeMetadataCacheOnDemand,
                &decoder);

            if FAILED(hr)
            {
                if (HRESULT_FROM_WIN32(ERROR_FILE_NOT_FOUND) == hr)
                {
                    resources->imageIsValid = false;
                    return;
                }
                else
                {
                    DX::ThrowIfFailed(hr);
                }
            }

            ComPtr<IWICBitmapFrameDecode> frame;
            DX::ThrowIfFailed(decoder->GetFrame(0, &frame));

            // Always convert to FP16 for JXR support. We ignore color profiles in this tool.
            WICPixelFormatGUID outFmt = GUID_WICPixelFormat64bppPRGBAHalf;

            ComPtr<IWICFormatConverter> converter;
            DX::ThrowIfFailed(wicFactory->CreateFormatConverter(&converter));
            DX::ThrowIfFailed(converter->Initialize(
                frame.Get(),
                outFmt,
                WICBitmapDitherTypeNone,
                nullptr,
                0.0f,
                WICBitmapPaletteTypeCustom));

            DX::ThrowIfFailed(converter.As(&resources->wicSource));
        }

        // Next, ensure that there is a D2D source (device dependent).
        if (resources->d2dSource == nullptr)
        {
            assert(resources->wicSource != nullptr);

            DX::ThrowIfFailed(ctx->CreateImageSourceFromWic(resources->wicSource.Get(), &resources->d2dSource));

            // The image is only valid if both WIC and D2D resources are ready.
            resources->imageIsValid = true;
        }
    }
}

void Game::LoadEffectResources(TestPatternResources* resources)
{
    auto ctx = m_deviceResources->GetD2DDeviceContext();

    // This test involves a shader file.
    if (resources->effectShaderFilename.compare(L"") != 0)
    {
        assert(resources->effectClsid != GUID{});

        try
        {
            DX::ThrowIfFailed(ctx->CreateEffect(resources->effectClsid, &resources->d2dEffect));
            resources->effectIsValid = true;
        }
        catch (std::exception)
        {
            // Most likely caused by a missing cso file. Continue on.
            resources->effectIsValid = false;
        }
    }
}

void Game::OnDeviceLost()
{
    m_gradientBrush.Reset();
    m_testTitleLayout.Reset();
    m_panelInfoTextLayout.Reset();
    m_panelInfoTextLayout.Reset();
    m_whiteBrush.Reset();

    for (auto it = m_testPatternResources.begin(); it != m_testPatternResources.end(); it++)
    {
        // Only invalidate the device dependent resources.
        it->second.d2dSource.Reset();
        it->second.imageIsValid = false;
        it->second.d2dEffect.Reset();
        it->second.effectIsValid = false;
    }
}

void Game::OnDeviceRestored()
{
    CreateDeviceDependentResources();

    CreateWindowSizeDependentResources();
}
#pragma endregion

#pragma region Test pattern control
// Set increment = true to go up, false to go down.
void Game::SetTestPattern(TestPattern testPattern)
{
    // save previous pattern in cache
    if (testPattern == TestPattern::Cooldown
        && m_currentTest != TestPattern::Cooldown)
        m_cachedTest = m_currentTest;

    if (TestPattern::StartOfTest <= testPattern)
    {

        if (TestPattern::Cooldown >= testPattern)
            {
            m_currentTest = testPattern;
        }
    }

    //	m_showExplanatoryText = true;
    m_newTestSelected = true;
}
// Set increment = true to go up, false to go down.
void Game::ChangeTestPattern(bool increment)
{
    if (TestPattern::Cooldown == m_currentTest)
    {
        m_currentTest = m_cachedTest;
        return;
    }

    if (increment)
    {
        if (TestPattern::EndOfTest == m_currentTest)
        {
            // Clamp to the end of the list.
        }
        else
        {
            unsigned int testInt = static_cast<unsigned int>(m_currentTest) + 1;
            m_currentTest = static_cast<TestPattern>(testInt);
        }
    }
    else
    {
        if (TestPattern::StartOfTest == m_currentTest)
        {
            // Clamp to the start of the list.
        }
        else
        {
            unsigned int testInt = static_cast<unsigned int>(m_currentTest) - 1;
            m_currentTest = static_cast<TestPattern>(testInt);
        }
    }

    //	m_showExplanatoryText = true;
    m_newTestSelected = true;
}

float clamp(float v, float min, float max)
{
	if (v > max)
		v = max; else
	if (v < min)
		v = min;
	return v;
}


void Game::StartTestPattern(void)
{
    m_currentTest = TestPattern::StartOfTest;
    // m_showExplanatoryText = true;
}

// TODO: Currently unused, but kept in case we want to emulate 8 bit behavior.
void Game::ChangeBackBufferFormat(DXGI_FORMAT fmt)
{
    // Just pass the new state to DeviceResources. It will use IDeviceNotify to ensure
    // that Game also recreates its resources.
    m_deviceResources->ChangeBackBufferFormat(fmt);
}

// Many test patterns can be affected by the presence of explanatory text.
// Allow user to toggle visibility on/off for each test pattern.
// Returns whether the visibility is true or false after the update.
bool Game::ToggleInfoTextVisible()
{
    m_showExplanatoryText = !m_showExplanatoryText;
    return m_showExplanatoryText;
}

// reset metadata to default state (same as panel properties) so it need do no tone mapping
// Note: OS does this on boot and on app exit.
void Game::SetMetadataNeutral()
{

    m_Metadata.MaxContentLightLevel = static_cast<UINT16>(m_rawOutDesc.MaxLuminance);
    m_Metadata.MaxFrameAverageLightLevel = static_cast<UINT16>(m_rawOutDesc.MaxFullFrameLuminance);
    m_Metadata.MaxMasteringLuminance = static_cast<UINT>(m_rawOutDesc.MaxLuminance);
    m_Metadata.MinMasteringLuminance = static_cast<UINT>(m_rawOutDesc.MinLuminance * 10000.0f);

    m_MetadataGamut = ColorGamut::GAMUT_Native;
    m_Metadata.RedPrimary[0] = static_cast<UINT16>(m_outputDesc.RedPrimary[0] * 50000.0f);
    m_Metadata.RedPrimary[1] = static_cast<UINT16>(m_outputDesc.RedPrimary[1] * 50000.0f);
    m_Metadata.GreenPrimary[0] = static_cast<UINT16>(m_outputDesc.GreenPrimary[0] * 50000.0f);
    m_Metadata.GreenPrimary[1] = static_cast<UINT16>(m_outputDesc.GreenPrimary[1] * 50000.0f);
    m_Metadata.BluePrimary[0] = static_cast<UINT16>(m_outputDesc.BluePrimary[0] * 50000.0f);
    m_Metadata.BluePrimary[1] = static_cast<UINT16>(m_outputDesc.BluePrimary[1] * 50000.0f);
    m_Metadata.WhitePoint[0] = static_cast<UINT16>(m_outputDesc.WhitePoint[0] * 50000.0f);
    m_Metadata.WhitePoint[1] = static_cast<UINT16>(m_outputDesc.WhitePoint[1] * 50000.0f);

    auto sc = m_deviceResources->GetSwapChain();
    DX::ThrowIfFailed(sc->SetHDRMetaData(DXGI_HDR_METADATA_TYPE_HDR10, sizeof(DXGI_HDR_METADATA_HDR10), &m_Metadata));
}

void Game::SetMetadata(float max, float avg, ColorGamut gamut)
{
    m_Metadata.MaxContentLightLevel = static_cast<UINT16>(max);
    m_Metadata.MaxFrameAverageLightLevel = static_cast<UINT16>(avg);
    m_Metadata.MaxMasteringLuminance = static_cast<UINT>(max);
    m_Metadata.MinMasteringLuminance = static_cast<UINT>(0.0 * 10000.0f);

    switch (gamut)
    {
    case GAMUT_Native:
        m_MetadataGamut = ColorGamut::GAMUT_Native;
        m_Metadata.RedPrimary[0] = static_cast<UINT16>(m_outputDesc.RedPrimary[0] * 50000.0f);
        m_Metadata.RedPrimary[1] = static_cast<UINT16>(m_outputDesc.RedPrimary[1] * 50000.0f);
        m_Metadata.GreenPrimary[0] = static_cast<UINT16>(m_outputDesc.GreenPrimary[0] * 50000.0f);
        m_Metadata.GreenPrimary[1] = static_cast<UINT16>(m_outputDesc.GreenPrimary[1] * 50000.0f);
        m_Metadata.BluePrimary[0] = static_cast<UINT16>(m_outputDesc.BluePrimary[0] * 50000.0f);
        m_Metadata.BluePrimary[1] = static_cast<UINT16>(m_outputDesc.BluePrimary[1] * 50000.0f);
        break;
    case GAMUT_sRGB:
        m_MetadataGamut = ColorGamut::GAMUT_sRGB;
        m_Metadata.RedPrimary[0] = static_cast<UINT16>(primaryR_709.x * 50000.0f);
        m_Metadata.RedPrimary[1] = static_cast<UINT16>(primaryR_709.y * 50000.0f);
        m_Metadata.GreenPrimary[0] = static_cast<UINT16>(primaryG_709.x * 50000.0f);
        m_Metadata.GreenPrimary[1] = static_cast<UINT16>(primaryG_709.y * 50000.0f);
        m_Metadata.BluePrimary[0] = static_cast<UINT16>(primaryB_709.x * 50000.0f);
        m_Metadata.BluePrimary[1] = static_cast<UINT16>(primaryB_709.y * 50000.0f);
        break;
    case GAMUT_Adobe:
        m_MetadataGamut = ColorGamut::GAMUT_Adobe;
        m_Metadata.RedPrimary[0] = static_cast<UINT16>(primaryR_Adobe.x * 50000.0f);
        m_Metadata.RedPrimary[1] = static_cast<UINT16>(primaryR_Adobe.y * 50000.0f);
        m_Metadata.GreenPrimary[0] = static_cast<UINT16>(primaryG_Adobe.x * 50000.0f);
        m_Metadata.GreenPrimary[1] = static_cast<UINT16>(primaryG_Adobe.y * 50000.0f);
        m_Metadata.BluePrimary[0] = static_cast<UINT16>(primaryB_Adobe.x * 50000.0f);
        m_Metadata.BluePrimary[1] = static_cast<UINT16>(primaryB_Adobe.y * 50000.0f);
        break;
    case GAMUT_DCIP3:
        m_MetadataGamut = ColorGamut::GAMUT_DCIP3;
        m_Metadata.RedPrimary[0] = static_cast<UINT16>(primaryR_DCIP3.x * 50000.0f);
        m_Metadata.RedPrimary[1] = static_cast<UINT16>(primaryR_DCIP3.y * 50000.0f);
        m_Metadata.GreenPrimary[0] = static_cast<UINT16>(primaryG_DCIP3.x * 50000.0f);
        m_Metadata.GreenPrimary[1] = static_cast<UINT16>(primaryG_DCIP3.y * 50000.0f);
        m_Metadata.BluePrimary[0] = static_cast<UINT16>(primaryB_DCIP3.x * 50000.0f);
        m_Metadata.BluePrimary[1] = static_cast<UINT16>(primaryB_DCIP3.y * 50000.0f);
        break;
    case GAMUT_BT2100:
        m_MetadataGamut = ColorGamut::GAMUT_BT2100;
        m_Metadata.RedPrimary[0] = static_cast<UINT16>(primaryR_2020.x * 50000.0f);
        m_Metadata.RedPrimary[1] = static_cast<UINT16>(primaryR_2020.y * 50000.0f);
        m_Metadata.GreenPrimary[0] = static_cast<UINT16>(primaryG_2020.x * 50000.0f);
        m_Metadata.GreenPrimary[1] = static_cast<UINT16>(primaryG_2020.y * 50000.0f);
        m_Metadata.BluePrimary[0] = static_cast<UINT16>(primaryB_2020.x * 50000.0f);
        m_Metadata.BluePrimary[1] = static_cast<UINT16>(primaryB_2020.y * 50000.0f);
        break;
    }

    m_Metadata.WhitePoint[0] = static_cast<UINT16>(D6500White.x * 50000.0f);
    m_Metadata.WhitePoint[1] = static_cast<UINT16>(D6500White.y * 50000.0f);

    auto sc = m_deviceResources->GetSwapChain();
    DX::ThrowIfFailed(sc->SetHDRMetaData(DXGI_HDR_METADATA_TYPE_HDR10, sizeof(DXGI_HDR_METADATA_HDR10), &m_Metadata));
}

// dump out the metadata to a string for display
void Game::PrintMetadata(ID2D1DeviceContext2* ctx, bool blackText /* = false */)
{
    std::wstringstream text;
    // print luminance levels
    text << "MaxCLL: ";
    text << std::to_wstring((int)m_Metadata.MaxContentLightLevel);
    text << "  MaxFALL: ";
    text << std::to_wstring((int)m_Metadata.MaxFrameAverageLightLevel);
    text << "  MinCLL: ";
    text << std::to_wstring(m_Metadata.MinMasteringLuminance / 10000.0f);

    // print gamut in use
    text << "  Gamut: ";
    switch (m_MetadataGamut)
    {
    case GAMUT_Native:
        text << "Native\n";
        break;
    case GAMUT_sRGB:
        text << "sRGB\n";
        break;
    case GAMUT_Adobe:
        text << "Adobe\n";
        break;
    case GAMUT_DCIP3:
        text << "DCI-P3\n";
        break;
    case GAMUT_BT2100:
        text << "bt.2100\n";
        break;
    }

    RenderText(ctx, m_largeFormat.Get(), text.str(), m_MetadataTextRect, blackText);
}


#pragma endregion

