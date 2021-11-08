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

#define versionString L"v0.960"

#include "pch.h"

#include "winioctl.h"
#include "ntddvdeo.h"
#include "dwmapi.h"

//#include "BasicMath.h"
#include "ColorSpaces.h"
#include "Game.h"

#define BRIGHTNESS_SLIDER_FACTOR (m_rawOutDesc.MaxLuminance / m_outputDesc.MaxLuminance)

//using namespace concurrency;

using namespace winrt::Windows::Devices;
using namespace winrt::Windows::Devices::Display;
using namespace winrt::Windows::Devices::Enumeration;

void ACPipeline();

extern void ExitGame();

using namespace DirectX;

using Microsoft::WRL::ComPtr;

Game::Game(const wchar_t* appTitle)
{
	m_appTitle = appTitle;

	ConstructorInternal();
}

#define SQUARE_COUNT 10

void Game::ConstructorInternal()
{
	if (!QueryPerformanceFrequency(&m_qpcFrequency))  // initialize clock frequency (of this machine)
	{
		throw std::exception("QueryPerformanceFrequency");
	}

	m_shiftKey = false;  // Whether the shift key is pressed
	m_pModeList = NULL;	  // ptr to list of display modes on this output.
	m_numModes = 0;
	m_logging = false;  // start out not writing to a log file

	m_mediaPresentDuration = 0;	 // Duration when using SwapChainMedia for hardware syncs.
				 // Units are in 0.1us or 100ns intervals.  Zero 0 = off.
	m_mediaVsyncCount = 1;	 // number of times to repeat the same frame for media playback

	m_vTotalFixedSupported = false;		 // assume we don't support fixed V-Total mode
	m_vTotalModeRequested = VTotalMode::Fixed;	 // default to try for PresentDuration mode
	m_vTotalFixedApproved = false;		 // default to assuming it's not working
	m_MPO = false;		 // assume no overlay plane initially

	m_minFrameRate = 10;  // fps      these will be overridden by the detection logic.
	m_maxFrameRate = 120;
	m_minFrameRateOverride = 0;	 // so user can override OS values
	m_maxFrameRateOverride = 0;	 // zero means do not override

	m_FrameRateRatio = 1.0;

	m_minDuration = 0;	// min frame time for Fixed V-Total mode -default to 0 for adaptive only
	m_maxDuration = 0;	// max frame time for Fixed V-Total mode -default to 0 for adaptive only
	m_autoResetAverageStatsCounts = MAXINT64;

	m_connectionDescriptorKind = DisplayMonitorDescriptorKind::DisplayId;

	m_color = 0.f;  // black by default

	m_currentTest = TestPattern::StartOfTest;

	m_flickerRateIndex = 0;  // select between min, max, etc for Flicker test                   2
	m_waveCounter = SQUARE_COUNT;
	m_waveEnum = WaveEnum::ZigZag;  // default                                          3
	m_waveUp = true;		    // for use in zigzag wave                           3
	m_waveAngle = 0.;		    // how far along we are in the sine wave            3
	m_waveInterval = 360;		    // duration of sine wave period in frames           3
	m_frameRateMargin = 0.0;		    // keeps from getting too close to the limits       3

	// parameters from DisplayID v2.1 to limit size of sudden changes in frame rate
	m_SuccessiveFrameDurationIncreaseInterval = 50;
	m_SuccessiveFrameDurationDecreaseInterval = 50;

	m_latencyRateIndex = 0;  // select between 60, 90, 120, 180, 240Hz for Latency tests        4
	m_mediaRateIndex = 0;  // select between 60, 90, 120, 180, 240Hz for Jitter               5

	m_latencyTestFrameRate = 1;	     // Flag for default to max                                 4
	m_sensorConnected = false;  //
	m_sensorNits = 27.0f;  // threshold for detection by sensor in nits               4
	m_sensing = false;  //
	m_flash = false;  // whether we are flashing the photocell this frame        4
	m_lastFlash = false;  // whether last frame was a flash                          4
	m_lastLastFlash = false;  // whether last frame was a flash                          4
	ResetSensorStats();		     // initialize the sensor tracking data                     4
	AutoResetAverageStats();	     // initialize the frame timing data
	m_avgInputTime = 0.006;	     // hard coded at 6ms until dongle can drive input
#define USB_TIME (0.002)	     // time for 1 round trip on USB wire       2ms?            4

	m_autoG2G = false;  // if we are in automatic sequence mode                            5
	m_g2gFrom = true;   // start with "From" color                                         5
	m_g2gFromIndex = 0;	     // subtest for Gray To Gray test                                   5
	m_g2gToIndex = 0;	     // subtest for Gray To Gray test                                   5
	m_g2gFrameRate = 1;	     // flag for default to max                                         5
	m_g2gCounter = 0;	     // counter for interval of gray periods                            5
	m_g2gInterval = 16;     // default interval for G2G switching                              5
	m_brightMode = false;	// if we are using a brighter warmup and G2G                       5

	m_frameDropRateEnum = DropRateEnum::Max;  // default to max                              6
	m_frameDropGamma = 1.;		      // used to balance brightness in square/random tests            6

	m_frameLockRateIndex = 0;		      // select sutbtest for frameDrop test          7
	m_MotionBlurIndex = maxMotionBlurs;  // start with frame fraction 100%              8
	m_motionBlurFrameRate = 60.;	      //                                             8
	m_judderTestFrameRate = 1.;	      // flag for default to max                     9
	m_fAngle = 0;		      // angle of object moving around screen       8,9
	m_tearingTestFrameRate = 1;		      // flag for default to max                     0
	m_sweepPos = 0;		      // position of bar in Tearing test             0

	m_targetFrameRate = 60.f;
	m_frameTime = 0.016667;
	m_lastFrameTime = 0.016667;
	m_sleepDelay = 16.0;  // ms simulate workload of app (just used for some tests)
	m_frameCount = 0;     // number of frames in current stats
	m_presentCount = 0;     // number of Presents in current stats
	m_frameCounter = 0;     // frames rendered since swapchain creation
	//  m_presentCounter = 0;       // frames presented since swapchain creation
	m_totalTimeSinceStart = 0;	// init clock since app start
	m_paused = 0;	// start out unpaused

	m_fileCounter = 0;	// next file name we can use

	// map the storage into the pointer array
	for (int i = 0; i < frameLogLength; i++)
	{
		m_frameEvents[i].clickCounts = 0;
		m_frameEvents[i].readCounts = 0;
		m_frameEvents[i].sleepCounts = 0;
		m_frameEvents[i].updateCounts = 0;
		m_frameEvents[i].drawCounts = 0;
		m_frameEvents[i].presentCounts = 0;
		m_frameEvents[i].syncCounts = 0;
		m_frameEvents[i].photonCounts = 0;

		m_frameLog[i] = &m_frameEvents[i];
	}

	m_currentProfileTile = 0;  // which intensity profile tile we are on
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
	FrameEvents* temp = m_frameLog[frameLogLength - 1];

	// shuffle all the others down.
	for (int i = frameLogLength - 1; i > 0; i--)
	{
		m_frameLog[i] = m_frameLog[i - 1];
	}

	// move last entry to front
	m_frameLog[0] = temp;

	// and clear it
	m_frameLog[0]->frameID = 0;
	m_frameLog[0]->presentID = 0;
	m_frameLog[0]->clickCounts;
	m_frameLog[0]->readCounts;
	m_frameLog[0]->sleepCounts;
	m_frameLog[0]->updateCounts;
	m_frameLog[0]->drawCounts;
	m_frameLog[0]->presentCounts;
	m_frameLog[0]->syncCounts;
	m_frameLog[0]->photonCounts;
}

// Initialize the Direct3D resources required to run.
void Game::Initialize(HWND window, int width, int height)
{
	m_deviceResources->SetWindow(window, width, height);
	m_deviceResources->CreateDeviceResources();
	m_deviceResources->SetDpi(96.0f);  // TODO: using default 96 DPI for now
	m_deviceResources->CreateWindowSizeDependentResources();

	CreateDeviceIndependentResources();
	CreateDeviceDependentResources();
	CreateWindowSizeDependentResources();
}

enum Game::VTotalMode Game::GetVTotalMode()
{
	return m_vTotalModeRequested;
}

// for UI to request a VTotal mode
void Game::ToggleVTotalMode()
{
	if (m_vTotalModeRequested == VTotalMode::Adaptive)
	{
		m_vTotalModeRequested = VTotalMode::Fixed;
		// tell deviceResources object so it can handle next swapchain resize properly  TODO it probably has to be recreated...
		m_deviceResources->SetVTotalMode(true);
	}
	else
	{
		m_vTotalModeRequested = VTotalMode::Adaptive;
		// tell deviceResources object so it can handle next swapchain resize properly  TODO it probably has to be recreated...
		m_deviceResources->SetVTotalMode(false);
	}

	// clear the stats counters since we are changing policy
	AutoResetAverageStats();
}

void Game::ToggleLogging()
{
	if (!m_logging)
	{
		m_fileCounter++;
		switch (m_currentTest)
		{
			// cases where a media oriented fixed-frame rate is best
		case TestPattern::FlickerConstant:  //  2
			sprintf_s(m_logFileName, 999, "AdaptSyncTest2_FlickerConst%03d.csv", m_fileCounter);
			break;
		case TestPattern::FlickerVariable:  //  3
			sprintf_s(m_logFileName, 999, "AdaptSyncTest3_FlickerVar%03d.csv", m_fileCounter);
			break;
		case TestPattern::DisplayLatency:  //  4
			sprintf_s(m_logFileName, 999, "AdaptSyncTest4_Latency%03d.csv", m_fileCounter);
			break;
		case TestPattern::GrayToGray:  //  5
			sprintf_s(m_logFileName, 999, "AdaptSyncTest5_GrayToGray%03d.csv", m_fileCounter);
			break;
		case TestPattern::FrameDrop:  //  6
			sprintf_s(m_logFileName, 999, "AdaptSyncTest6_Framedrop%03d.csv", m_fileCounter);
			break;
		case TestPattern::FrameLock:  //  7
			sprintf_s(m_logFileName, 999, "AdaptSyncTest7_Jitter%03d.csv", m_fileCounter);
			break;
		case TestPattern::MotionBlur:  //  8
			sprintf_s(m_logFileName, 999, "AdaptSyncTest8_Blur%03d.csv", m_fileCounter);
			break;
		case TestPattern::GameJudder:  //  9
			sprintf_s(m_logFileName, 999, "AdaptSyncTest9_Judder%03d.csv", m_fileCounter);
			break;
		case TestPattern::Tearing:  //  0
			sprintf_s(m_logFileName, 999, "AdaptSyncTest10_Tearing%03d.csv", m_fileCounter);
			break;

		default:
			break;
		}

		// open log file
		int err = 0;
		err = fopen_s(&m_logFile, m_logFileName, "w");
		fprintf_s(m_logFile, "Time, Mode, Target, Current, FrmStats, Brightness\n");
		m_logTime = 0;
		m_lastLogTime = 0;
		m_logging = true;
	}
	else
	{
		fclose(m_logFile);
		m_logging = false;
	}
}

void Game::ToggleMargin()
{
	if (m_frameRateMargin > 0.015625)  // 1/64
		m_frameRateMargin = 0.0;
	else if (m_frameRateMargin <= 0.015625)
		m_frameRateMargin = 2.0;
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

void Game::ToggleBrightMode()
{
	m_brightMode = !m_brightMode;
}

void Game::ToggleAutoG2G()
{
	if (m_currentTest == TestPattern::GrayToGray)
	{
		m_autoG2G = !m_autoG2G;	 // automatic sequence mode                              5
		if (m_autoG2G == false)
		{
			m_g2gFromIndex = 0;	 // reset these on return to manual control
			m_g2gToIndex = 0;
		}
	}
}

uint64_t myQPC()
{
	static const uint64_t TicksPerSecond = 10000000;  // microseconds
	LARGE_INTEGER	  cycles, qpcFrequency;

	QueryPerformanceFrequency(&qpcFrequency);

	QueryPerformanceCounter(&cycles);

	uint64_t r = cycles.QuadPart * TicksPerSecond;

	return r;
}

void mySleep(double mseconds)  // All routines named "sleep" must take milliseconds ms
{
	LARGE_INTEGER qpcFrequency, counts;	 // args to system calls
	uint64_t	  time, timeLimit;	 // 64b ints for intermediate checking

	if (mseconds <= 0)	// validate input
	{
		Sleep(0);  // in case this is in a loop
		return;
	}

	QueryPerformanceFrequency(&qpcFrequency);  // get counts per second

	QueryPerformanceCounter(&counts);
	timeLimit = counts.QuadPart;
	timeLimit += static_cast<uint64_t>((mseconds * static_cast<double>(qpcFrequency.QuadPart)) / 1000.0);  // convert time limit from ms into counts

	time = 0;
	while (time < timeLimit)
	{
		Sleep(0);  // cede control to OS for other processes
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

// clear all the stat tracking from the sensor
void Game::ResetSensorStats()
{
	m_sensorCount = 0;
	m_totalSensorTime = 0.;
	m_totalSensorTime2 = 0.;
	m_minSensorTime = 0.999999;
	m_maxSensorTime = 0.;
}

void Game::ResetAverageStats()
{
	m_frameCount = 1;  // for stats on frames

	m_totalFrameTime = m_lastFrameTime;		    // reset accumulator
	m_totalFrameTime2 = m_lastFrameTime * m_lastFrameTime;  // square of above used for variance math

	m_totalRunningTime = 0.;  // not sure we really ever want to reset this though...
	m_totalRunningTime2 = 0.;

	m_totalRenderTime = 0.;
	m_totalRenderTime2 = 0.;

	m_presentCount = 0;  // for stats on Presents
	m_totalPresentTime = 0.;
	m_totalPresentTime2 = 0.;
	m_minPresentTime = 1.;
	m_maxPresentTime = 0.;
}

// this version is deferred so things settle down a bit first.
void Game::AutoResetAverageStats(void)
{
	// start timer out at current QPC time
	m_autoResetAverageStatsCounts = getPerfCounts();

}

// clears the accumulators for the current mode  -bound to 's' key
// immediate, not deferrred.
void Game::ResetCurrentStats()
{
	if (m_sensing)
		ResetSensorStats();
	else
		ResetAverageStats();
}

// Returns whether the reported display metadata consists of
// default values generated by Windows.
bool Game::CheckForDefaults()
{
	return (
		// Default SDR display values (RS2)
		(270.0f == m_rawOutDesc.MaxLuminance && 270.0f == m_rawOutDesc.MaxFullFrameLuminance && 0.5f == m_rawOutDesc.MinLuminance) ||
		// Default HDR display values (RS2)
		(550.0f == m_rawOutDesc.MaxLuminance && 450.0f == m_rawOutDesc.MaxFullFrameLuminance && 0.5f == m_rawOutDesc.MinLuminance) ||
		// Default HDR display values (RS3)
		(1499.0f == m_rawOutDesc.MaxLuminance && 799.0f == m_rawOutDesc.MaxFullFrameLuminance && 0.01f == m_rawOutDesc.MinLuminance));
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

const wchar_t* Game::GetTierName(Game::TestingTier tier)
{
	if (tier == DisplayHDR400)
		return L"DisplayHDR400";
	else if (tier == DisplayHDR500)
		return L"DisplayHDR500";
	else if (tier == DisplayHDR600)
		return L"DisplayHDR600";
	else if (tier == DisplayHDR1000)
		return L"DisplayHDR1000";
	else if (tier == DisplayHDR1400)
		return L"DisplayHDR1400";
	else if (tier == DisplayHDR2000)
		return L"DisplayHDR2000";
	else if (tier == DisplayHDR3000)
		return L"DisplayHDR3000";
	else if (tier == DisplayHDR4000)
		return L"DisplayHDR4000";
	else if (tier == DisplayHDR6000)
		return L"DisplayHDR6000";
	else if (tier == DisplayHDR10000)
		return L"DisplayHDR10000";
	else
		return L"Unsupported DisplayHDR Tier";
}

float Game::GetTierLuminance(Game::TestingTier tier)
{
	if (tier == DisplayHDR400)
		return 400.f;
	else if (tier == DisplayHDR500)
		return 500.f;
	else if (tier == DisplayHDR600)
		return 600.f;
	else if (tier == DisplayHDR1000)
		return 1015.27f;
	else if (tier == DisplayHDR1400)
		return 1400.f;
	else if (tier == DisplayHDR2000)
		return 2000.f;
	else if (tier == DisplayHDR3000)
		return 3000.f;
	else if (tier == DisplayHDR4000)
		return 4000.f;
	else if (tier == DisplayHDR6000)
		return 6000.f;
	else if (tier == DisplayHDR10000)
		return 10000.f;
	else
		return -1.0f;
}

// Determines whether tearing support is available for fullscreen borderless windows.
void Game::CheckTearingSupport()
{
#ifndef PIXSUPPORT
	ComPtr<IDXGIFactory6> factory;
	HRESULT		  hr = CreateDXGIFactory1(IID_PPV_ARGS(&factory));
	BOOL		  allowTearing = FALSE;
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


bool Game::isMedia()
{
	switch (m_currentTest)
	{
		// cases where a media oriented fixed-frame rate is best
	case TestPattern::FlickerConstant:	//  2
	case TestPattern::FrameDrop:	//  6
	case TestPattern::FrameLock:	//  7
	case TestPattern::MotionBlur:	//  8
		return true;

		// cases where a game-oriented adaptive frame rate is best
	case TestPattern::FlickerVariable:	//  3
	case TestPattern::DisplayLatency:	//  4
	case TestPattern::GrayToGray:	//  5
	case TestPattern::GameJudder:	//  9
	case TestPattern::Tearing:		//  0
		return false;

	default:
		return false;
	}
}

HANDLE Game::GetFrameLatencyHandle()
{
	return m_deviceResources->GetFrameLatencyHandle();
}

void Game::LogFrameStats()
{
	// poll frame stats to see if we have newer data
	auto		  sc = m_deviceResources->GetSwapChain();
	DXGI_FRAME_STATISTICS frameStats;
	sc->GetFrameStatistics(&frameStats);

	// correct for case where framestats are from an older frame relative to current frameCounter
	m_frameStatsLag = m_frameCounter - static_cast<uint64_t>(frameStats.PresentCount);
	//  m_frameStatsLag = 2;
	if (m_frameStatsLag < 0)  // clamp positive
		m_frameStatsLag = 0;
	if (m_frameStatsLag >= frameLogLength - 1)	// clamp sane
		m_frameStatsLag = frameLogLength - 1;

	// update the log entry at the corresponding location
	m_frameLog[m_frameStatsLag]->syncCounts = frameStats.SyncQPCTime.QuadPart;
	m_frameLog[m_frameStatsLag]->presentID = frameStats.PresentCount;
}

// Tick method for use with photocell sensor
void Game::Tick()
{
	HRESULT hr = 0;
	//  double  frameTime;                                                  // local variable for comparison
	double dFrequency = static_cast<double>(m_qpcFrequency.QuadPart);  // counts per second of QPC

	// Update with data from last frame
	// get sync time of last frame from swapchain
	auto		  sc = m_deviceResources->GetSwapChain();
	DXGI_FRAME_STATISTICS frameStats;
	sc->GetFrameStatistics(&frameStats);
	m_frameLog[1]->syncCounts = frameStats.SyncQPCTime.QuadPart;
	// this is from 2 frames ago since we havent rotated yet

	// Compute a refresh rate for the actual monitor v-syncs
	INT64  monCounts = getPerfCounts();				    // get current time in QPC counts
	double monDeltaT = (monCounts - m_lastMonCounts) / dFrequency;  // see how long it's been
	if (monDeltaT > 0.256)					    // if it's been over 1/4 second
	{
		uint monSyncs = frameStats.SyncRefreshCount;		       // get current QPC count of syncs/refreshes
		uint monSyncDelta = monSyncs - static_cast<uint32_t>(m_lastMonSyncs);  // how many syncs since last time
		m_monitorSyncRate = (float)monSyncDelta / monDeltaT;		       // compute sync rate of panel
		m_lastMonCounts = monCounts;					       // save state for next time
		m_lastMonSyncs = monSyncs;
	}

	// also check the media stats that return an approved Present Duration
	IDXGISwapChainMedia* scMedia;
	DX::ThrowIfFailed(sc->QueryInterface(IID_PPV_ARGS(&scMedia)));
	DXGI_FRAME_STATISTICS_MEDIA mediaStats = { 0 };
	hr = scMedia->GetFrameStatisticsMedia(&mediaStats);

	if (hr == S_OK && mediaStats.ApprovedPresentDuration != 0)
		m_vTotalFixedApproved = true;
	else
		m_vTotalFixedApproved = false;

	// set photon time to be a bit after sync time until we have hardware event
	m_frameLog[0]->photonCounts = m_frameLog[0]->syncCounts + 100000;  // assume 10ms of GtG, etc.

	m_frameCounter++;  // new frame id

	if (!m_paused)
	{
		if (m_sensing)
		{
			m_lastLastFlash = m_lastFlash;		  // lagged from 2 frames ago
			m_lastFlash = m_flash;			  // lagged from previous frame
			m_flash = (m_frameCounter % 4) == 0;  // only flash 1 of every 4 frames

#if 0
			// if there should be a flash this frame, reconnect the sensor (should not be required every frame)
			if (m_flash)
			{
				// make sure we are still connected (should never happen in real life)
				if (!m_sensorConnected)
				{
					m_sensorConnected = m_sensor.ConnectSensor();
				}
			}
#endif
			// advance printout columns to current frame
			RotateFrameLog();  // make room to store this latest data

			//          m_frameLog[0]->frameID = m_frameCounter;             // log the frame ID

			// log time when app starts a frame
			m_frameLog[0]->readCounts = getPerfCounts();

			// Don't bother to sleep during sensing
			// model app workload by sleeping that many ms
			//          mySleep(sleepDelay);
			Update();  // actually do some CPU stuff

			// log when drawing starts on the GPU
			//          m_frameLog[0]->drawCounts = getPerfCounts();

			// Draw
			Render();  // this is before we have all values for this frame

			// if there should be a flash this frame, start measuring
			uint64_t sensorCountsStart;
			if (m_flash)
			{
				m_sensor.StartLatencyMeasurement(LatencyType_Immediate);
				sensorCountsStart = getPerfCounts();
			}
			UNREFERENCED_PARAMETER(sensorCountsStart);

			// log time when app calls Present()
			m_frameLog[0]->presentCounts = getPerfCounts();

			// Show the new frame
			//          UINT syncInterval = 0;
			//          UINT presentFlags;

			if (m_mediaPresentDuration != 0
				)
			{
				hr = m_deviceResources->Present(m_mediaVsyncCount, DXGI_PRESENT_USE_DURATION);
			}
			else
			{
				hr = m_deviceResources->Present(0, 0);	// default is V-Sync enabled, not tearing
			}

			// if this was a frame that included a flash, then read the photocell's measurement
			if (m_flash)
			{
				// time from start of latency measurement sensor in sec
				m_sensorTime = m_sensor.ReadLatency();	// blocking call

				// get own estimate of sensorTime based on QPC
				auto sensorCountsEnd = getPerfCounts();
				UNREFERENCED_PARAMETER(sensorCountsEnd);
				//              m_sensorTime = (sensorCountsEnd - sensorCountsStart) / dFrequency;

				if ((m_sensorTime > 0.001) && (m_sensorTime < 0.100))  // if valid, run the stats on it
				{
					// total it up for computing the average and variance
					m_totalSensorTime += m_sensorTime;
					m_totalSensorTime2 += (double)m_sensorTime * m_sensorTime;
					m_sensorCount++;

					// scan for min
					if (m_sensorTime < m_minSensorTime)
						m_minSensorTime = m_sensorTime;

					// scan for max
					if (m_sensorTime > m_maxSensorTime)
						m_maxSensorTime = m_sensorTime;
				}
			}
			//          else   // we are not flashing so just wait for the flip queue to update with the black frame

			// make sure each frame lasts long enough for the sensor to detect it.
			Sleep(2);

			m_lastFrameTime = m_frameTime;
			m_frameTime = (m_frameLog[0]->readCounts - m_lastReadCounts) / dFrequency;

			m_lastReadCounts = m_frameLog[0]->readCounts;
		}
		else  // we are not using the sensor, just track in-PC timings
		{
			//  m_flash =  m_frameCounter & 1;                          // switch every other frame
			//  m_flash = (m_frameCounter >> 1) & 1;                    // switch every 2 frames
			m_flash = (m_frameCounter >> 2) & 1;  // switch every 4 frames

			// advance to current frame
			RotateFrameLog();  // make room to store this latest data

			m_frameLog[0]->frameID = m_frameCounter;  // log the frame ID

			// Read input events()
			// log time when the button was clicked -extract the click time stamp
			// log time when photocell was hit -extract the photon time stamp
			// log time when the camera image flashed -extract the camera result time stamp

			// log time when app starts a frame
			m_frameLog[0]->readCounts = getPerfCounts();

			// simulate input device click that occurred before we read it.
			int64_t inputCounts = static_cast<int64_t>(m_avgInputTime * dFrequency);
			m_frameLog[0]->clickCounts = m_frameLog[0]->readCounts - inputCounts;

			// TODO:  FrameTime should not be a member var

			// default to no app frame-doubling
			m_mediaVsyncCount = 1;

			// use PresentDuration mode in some tests
			UINT closestSmallerDuration = 0, closestLargerDuration = 0;
			if (m_vTotalFixedSupported && (
				m_currentTest == TestPattern::FlickerConstant    // 2
				|| m_currentTest == TestPattern::DisplayLatency  // 4
				|| m_currentTest == TestPattern::FrameLock	     // 7
				|| m_currentTest == TestPattern::MotionBlur))     // 8
			{
				m_mediaVsyncCount = 1;

				// for fixed presentation time , find the number of frames to duplicate to get in the range
				if (m_targetFrameTime != 0)
				{
					INT64 tempDuration = (INT64)(10000000.0 * m_targetFrameTime);
					while (tempDuration > m_maxDuration)
					{
						m_mediaVsyncCount++;
						tempDuration = (INT64)(10000000.0 * m_targetFrameTime / m_mediaVsyncCount);
					}
				}

				m_mediaPresentDuration = (INT64)(10000000.0 * m_targetFrameTime / m_mediaVsyncCount);
				if (m_mediaPresentDuration > 0)
				{
						// confirm that this rate is actually supported
						hr = scMedia->CheckPresentDurationSupport(
							static_cast<uint32_t>(m_mediaPresentDuration), &closestSmallerDuration, &closestLargerDuration);

						// Check that one of the neighboring Durations matches our goal
						if (hr == S_OK && (closestLargerDuration != 0 || closestSmallerDuration != 0))
						{
							if ((m_mediaPresentDuration - closestSmallerDuration) < (closestLargerDuration - m_mediaPresentDuration))
							{
								m_mediaPresentDuration = closestSmallerDuration;
							}
							else
							{
								m_mediaPresentDuration = closestLargerDuration;
							}

							// then the rate is valid and we can set it
							hr = scMedia->SetPresentDuration(static_cast<uint32_t>(m_mediaPresentDuration));
							if (hr != S_OK)
							{
								m_mediaPresentDuration = 0;
							}
						}
				}
			}
			else
			{
				// PresentDuration is not to be used
				m_mediaPresentDuration = 0;

				if (m_vTotalFixedApproved)
				{
					scMedia->SetPresentDuration(0);
				}
			}

				hr = scMedia->GetFrameStatisticsMedia(&mediaStats);
				if (hr == S_OK && m_mediaPresentDuration > 0)
				{
					// we can use the wait model so indicate on UI
					m_frameLog[0]->sleepCounts = getPerfCounts();
				}
				else  // use a sleep timer
				{
					m_vTotalFixedApproved = false;

					// if we didnt get PresentDuration mode, maybe we got an MPO:
					m_MPO = false;
					if (mediaStats.CompositionMode == DXGI_FRAME_PRESENTATION_MODE_OVERLAY)
					{
						m_MPO = true;
					}

					double avgRunTime;		 // how long the app spends not sleeping
					m_mediaPresentDuration = 0;	 // indicate to not use PresentDuration model

					if (m_frameCount > 1)
						avgRunTime = m_totalRunningTime / m_frameCount;
					else
						avgRunTime = 0.0013;  // aka 1.3ms

						// apply frame doubling if frame rate is less than EDID reported minimum
					double targetFrameTime = m_targetFrameTime;
					double maxFrameTime = 1.001 / m_minFrameRate;  // as repotted by EDID
					if (targetFrameTime > maxFrameTime)
					{
						targetFrameTime /= 2.0;	 // halve the period to double the frame rate
						m_mediaVsyncCount = 2;
					}
					// if still to slow, then
					if (targetFrameTime > maxFrameTime)
					{
						targetFrameTime /= 1.5;	 // triple the frame rate
						m_mediaVsyncCount = 3;
					}
					// if still to slow, then
					if (targetFrameTime > maxFrameTime)
					{
						targetFrameTime /= 1.333333333;	 // quadruple the frame rate
						m_mediaVsyncCount = 4;
					}

					// don't worry too much about precision here as software loop is sloppy anyway.
					// compute how much of frame time to sleep by subtracting time running CPU/GPU
					m_sleepDelay = 1000.0 * (targetFrameTime - avgRunTime);

					// Hopefully set correct duration for this frame by sleeping enough
					m_frameLog[0]->sleepCounts = getPerfCounts();
					mySleep(m_sleepDelay);
				}

			// log when app logic starts on the GPU
			m_frameLog[0]->updateCounts = getPerfCounts();
			Update();  // actually do some CPU workload stuff

			// log when drawing starts on the GPU
			m_frameLog[0]->drawCounts = getPerfCounts();
			// Draw()
			Render();  // update screen (before we have all values for this frame)

			// log time when app calls Present()
			m_frameLog[0]->presentCounts = getPerfCounts();
			// Call Present() to show the new frame

			if (m_mediaPresentDuration != 0
				)
			{
				hr = m_deviceResources->Present(m_mediaVsyncCount, DXGI_PRESENT_USE_DURATION);
			}
			else
			{
				hr = m_deviceResources->Present(0, 0);                  // default is V-Sync enabled
			}

			// log time when vsync happens on display
			//      m_frameLog[0]->syncCounts = presentCounts;          // clear this as we don't know until next frame

			// log time when photons arrive at photocell
			//      m_frameLog[0]->photonCounts = presentCounts;        // clear this as we don't know until next frame

			// track frame time for frame rate
			m_lastFrameTime = m_frameTime;
			m_frameTime = (m_frameLog[0]->readCounts - m_lastReadCounts) / dFrequency;

			m_lastReadCounts = m_frameLog[0]->readCounts;

			// if valid data
			if ((m_frameTime > 0.001) && (m_frameTime < 0.100))	 // run the stats on it
			{
				m_frameCount++;	 // frames we average over

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
				// accumulate Running time for computing Average and Variance
				double sleepTime = (m_frameLog[0]->updateCounts - m_frameLog[0]->sleepCounts) / dFrequency;
				double runningTime = m_frameTime - sleepTime;  // all the time last frame not sleeping
				m_totalRunningTime += runningTime;
				m_totalRunningTime2 += runningTime * runningTime;

				// accumulate Render time for computing Average
				double renderTime = (m_frameLog[0]->presentCounts - m_frameLog[0]->drawCounts) / dFrequency;
				m_totalRenderTime += renderTime;
				m_totalRenderTime2 += renderTime * renderTime;

				// accumulate Present time for computing Average
				double presentTime = (m_frameLog[2]->syncCounts - m_frameLog[2]->presentCounts) / dFrequency;
				m_totalPresentTime += presentTime;
				m_totalPresentTime2 += presentTime * presentTime;
			}
			else
			{
				float x = 1.f;	// this line just for setting a breakpoint on
				UNREFERENCED_PARAMETER(x);
			}
		}
	}
	else
	{
		Sleep(15);  // update at ~60Hz even when paused.
				// save for use next frame
	}

	// track data since app startup
	m_totalTimeSinceStart += m_frameTime;  // TODO totalTime should not be a double but a uint64 in microseconds

	m_lastLogTime = m_logTime;	// save last value
	m_logTime += m_frameTime;	// time since logging started

	// run timer that delays average stats reset by 1/4 second after each (sub)test change:
	INT64  nowCounts = getPerfCounts();						  // get current time in QPC counts
	double deltaT = (nowCounts - m_autoResetAverageStatsCounts) / dFrequency;  // see how long it's been
	if (deltaT > 0.250)
	{
		ResetAverageStats();
		m_autoResetAverageStatsCounts = MAXINT64;
	}
}

#if 0 
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
		Sleep(15);                                    // update at ~60Hz even when paused.

	double dFrequency = m_qpcFrequency.QuadPart;        // counts per second of QPC

	// Compute current frame time since last frame
	m_frameTime = (m_frameLog[0]->readCounts - m_lastReadCounts) / dFrequency;
	m_lastReadCounts = m_frameLog[0]->readCounts;       // save for use next frame

	// track data since app startup
	m_totalTimeSinceStart += m_frameTime;       // TODO totalTime should not be a double but a uint64 in microseconds
}
#endif

// Every test pattern could have an update routine to call in the main update routine in Tick()
// Flicker test for fixed frame rate set                                                                    2
void Game::UpdateFlickerConstant()
{
	double maxFrameRate = m_maxFrameRate;
	double minFrameRate = m_minFrameRate;

	// check for overrides and apply
	if (m_maxFrameRateOverride != 0)
		maxFrameRate = m_maxFrameRateOverride;
	if (m_minFrameRateOverride != 0)
		minFrameRate = m_minFrameRateOverride;

	/* this is not reliable
		if (m_displayFrequency > 20)
		{
			if (maxFrameRate > m_displayFrequency)        // clamp to current mode
				maxFrameRate = m_displayFrequency;
		}
	*/

	// determine what rate to use based on the up/down arrow key setting
	switch (m_flickerRateIndex)
	{
	case 0:
		m_targetFrameRate = minFrameRate;  // min reported by implementation
		break;

	default:
		m_targetFrameRate = mediaRefreshRates[m_flickerRateIndex - 1];
		break;
	}

	m_targetFrameTime = 1.0 / m_targetFrameRate;
}

// compute frame rate for this Flicker test with variable frame rate:                                       3
void Game::UpdateFlickerVariable()
{
	double maxFrameRate = m_maxFrameRate;
	double minFrameRate = m_minFrameRate;

	// check for overrides and apply
	if (m_maxFrameRateOverride != 0)
		maxFrameRate = m_maxFrameRateOverride;
	if (m_minFrameRateOverride != 0)
		minFrameRate = m_minFrameRateOverride;


	/* this is not reliable
		if (m_displayFrequency > 20)
		{
			if (maxFrameRate > m_displayFrequency)        // clamp to current mode
				maxFrameRate = m_displayFrequency;
		}
	*/
	// vary frame rate based on current pattern
	switch (m_waveEnum)
	{
	case WaveEnum::ZigZag:  // zig-zag not square wave
	{
		if (m_waveUp)
		{
			m_targetFrameRate += 1;
			if (m_targetFrameRate > maxFrameRate)
			{
				m_targetFrameRate = maxFrameRate;
				m_waveUp = false;
			}
		}
		else
		{
			m_targetFrameRate -= 1;
			if (m_targetFrameRate < minFrameRate)
			{
				m_targetFrameRate = minFrameRate;
				m_waveUp = true;
			}
		}
	}
	break;

	case WaveEnum::SquareWave: {
		// set frame rate based on wave
		if (m_waveUp)
			m_targetFrameRate = maxFrameRate - 2 * m_frameRateMargin;
		else
			m_targetFrameRate = minFrameRate + m_frameRateMargin;

		// check to see if it is time to switch between Upside and downside of wave
		m_waveCounter--;
		if (m_waveCounter <= 0)
		{
			m_waveUp = !m_waveUp;	   // toggle to wave down
			m_waveCounter = SQUARE_COUNT;  // reset counter
		}

		// guarantee frame times are not outside the limits reported by DisplayID 2.1
		double nextTargetFrameTime = 1.0 / m_targetFrameRate;
		double deltaFrameTimeMS = (nextTargetFrameTime - m_targetFrameTime) * 1000.0;  // delta between next and current

		if (deltaFrameTimeMS >= 0)  // frame time just increased
		{
			if (deltaFrameTimeMS > m_SuccessiveFrameDurationIncreaseInterval)
				deltaFrameTimeMS = m_SuccessiveFrameDurationIncreaseInterval;
			nextTargetFrameTime = m_targetFrameTime + deltaFrameTimeMS / 1000.0;  // add clamped delta back to current
			m_targetFrameRate = 1.0 / nextTargetFrameTime;
		}
		else  // frame time just decreased
		{
			deltaFrameTimeMS *= -1.0;
			if (deltaFrameTimeMS > m_SuccessiveFrameDurationDecreaseInterval)
				deltaFrameTimeMS = m_SuccessiveFrameDurationDecreaseInterval;
			nextTargetFrameTime = m_targetFrameTime - deltaFrameTimeMS / 1000.0;  // ~add clamped delta back to current
			m_targetFrameRate = 1.0 / nextTargetFrameTime;
		}
	}
							 break;

	case WaveEnum::Random: {
		double base = minFrameRate + m_frameRateMargin;
		double range = maxFrameRate - 2 * m_frameRateMargin - base;
		m_targetFrameRate = base + range * rand() / RAND_MAX;

		// guarantee frame times are not outside the limits reported by DisplayID 2.1
		double nextTargetFrameTime = 1.0 / m_targetFrameRate;
		double deltaFrameTimeMS = (nextTargetFrameTime - m_targetFrameTime) * 1000.0;  // delta between next and current

		if (deltaFrameTimeMS >= 0)  // frame time just increased
		{
			if (deltaFrameTimeMS > m_SuccessiveFrameDurationIncreaseInterval)
				deltaFrameTimeMS = m_SuccessiveFrameDurationIncreaseInterval;
			nextTargetFrameTime = m_targetFrameTime + deltaFrameTimeMS / 1000.0;  // add clamped delta back to current
			m_targetFrameRate = 1.0 / nextTargetFrameTime;
		}
		else  // frame time just decreased
		{
			deltaFrameTimeMS *= -1.0;
			if (deltaFrameTimeMS > m_SuccessiveFrameDurationDecreaseInterval)
				deltaFrameTimeMS = m_SuccessiveFrameDurationDecreaseInterval;
			nextTargetFrameTime = m_targetFrameTime - deltaFrameTimeMS / 1000.0;  // ~add clamped delta back to current
			m_targetFrameRate = 1.0 / nextTargetFrameTime;
		}
	}
						 break;

	case WaveEnum::SineWave: {
		double minFR = minFrameRate + m_frameRateMargin;
		double maxFR = maxFrameRate - 2 * m_frameRateMargin;
		double base = 0.5 * (minFR + maxFR);
		double range = 0.5 * (maxFR - minFR);

		//      double angle = m_totalTimeSinceStart * M_PI/3.0;            // 60deg/sec = period of 6s
		// period is (waveinterval) 360 frames per cycle
		// frequency is 1/360th of a cycle per frame
		m_waveAngle += TAU / (double)m_waveInterval;
		m_waveAngle = fmod(m_waveAngle, TAU);  // wrap to be nice to sin()

		m_targetFrameRate = base + range * sin(m_waveAngle);
	}
						   break;

	case WaveEnum::Max:
		m_targetFrameRate = maxFrameRate;
		break;
	}

	m_targetFrameTime = 1.0 / m_targetFrameRate;
}

// Set correct color for mid-screen test rect in G2G test section 10.2
float Game::GrayToGrayValue(INT32 index)
{
	float c, nits = 0;

	if (!CheckHDR_On())	 // SDR case
	{
		float fDelta = 1.0f / (numGtGValues - 1);
		c = fDelta * index;	// proportion of the range
						//      c = d * m_outputDesc.MaxLuminance;          // scale to peak luminance
	}
	else  // HDR case:
	{
		switch (index)
		{
		case 0:
			nits = 0.000000f;
			break;
		case 1:
			nits = 0.047366f * m_outputDesc.MaxLuminance;
			break;
		case 2:
			nits = 0.217638f * m_outputDesc.MaxLuminance;
			break;
		case 3:
			nits = 0.531049f * m_outputDesc.MaxLuminance;
			break;
		case 4:
			nits = 1.000000f * m_outputDesc.MaxLuminance;
			break;
		}
		c = nitstoCCCS(nits);
	}
	return c;
}

// update routine for Gray2Gray Test pattern                                                               5
void Game::UpdateGrayToGray()
{
	double maxFrameRate = m_maxFrameRate;
	double minFrameRate = m_minFrameRate;

	// check for overrides and apply
	if (m_maxFrameRateOverride != 0)
		maxFrameRate = m_maxFrameRateOverride;
	if (m_minFrameRateOverride != 0)
		minFrameRate = m_minFrameRateOverride;

	if (m_autoG2G)  // if we are in auto-sequence
	{
		m_targetFrameRate = maxFrameRate;  // per CTS
		/*
					update the timer
					if it has run down, then
						Toggle the m_from boolean
						If we are back at true, then
						Increment the FromIndex
							if that overflows, increment the ToIndex
			*/
			//      m_testTimeRemainingSec -= m_frameTime;
		if (m_g2gCounter <= 0)	// counter ran down, so change something
		{
			if (m_g2gFrom)
			{
				m_g2gFrom = false;  // switch to the To color
			}
			else  // we were on the To color
			{
				m_g2gToIndex++;	 // varies fastest
				if (m_g2gToIndex > numGtGValues - 1)
				{
					m_g2gToIndex = 0;

					m_g2gFromIndex++;
					if (m_g2gFromIndex > numGtGValues - 1)
						m_g2gFromIndex = 0;
				}
				m_g2gFrom = true;
			}
			if ((m_g2gFromIndex == 4) || (m_g2gFromIndex != m_g2gToIndex))  // skip the diagonal where to and from are same
			{
				m_g2gCounter = static_cast<int32_t>(0.25 * m_targetFrameRate);	// reset counter to take 250ms at ANY frame rate
				if (m_g2gCounter & 0x01)					// if it is odd,
					m_g2gCounter++;						// add one to make it even
			}
			//              m_testTimeRemainingSec = 0.250;      //  0.006944444 * 5;
		}

	}
	else  // we are in manual state setting mode (not auto sequence)
	{
		//      m_g2gFrom = (m_frameCounter >> 4) & 1;                           // switch every 16 frames
		//      m_g2gFrom = (m_frameCounter >> 2) & 1;                           // switch every 8 frames
		//      m_g2gFrom = (m_frameCounter >> 2) & 1;                           // switch every 4 frames
		//      m_g2gFrom = (m_frameCounter >> 1) & 1;                           // switch every 2 frames
		//      m_g2gFrom = (m_frameCounter     ) & 1;                           // switch every other frame

		if (m_g2gCounter <= 0)
		{
			m_g2gFrom = !m_g2gFrom;	   // flip to showing the other color
			m_g2gCounter = m_g2gInterval;  // reset the interval timer
		}

		m_targetFrameRate = m_g2gFrameRate;  // per CTS
		if (m_targetFrameRate > maxFrameRate)
			m_targetFrameRate = maxFrameRate;

	}

	// define color for test patch
	if (m_g2gFrom)
		m_color = GrayToGrayValue(m_g2gFromIndex);
	else
		m_color = GrayToGrayValue(m_g2gToIndex);

	// set frame duration accordingly

#if 0
	// skip some frames during test#5 to keep panels from tuning overdrive to a fixed rate
	if (m_currentTest == TestPattern::GrayToGray)
	{
		//  m_targetFrameRate = 59.97;        // TODO delete this test code!!
		if (((float)rand() / RAND_MAX) > 0.50)                     // about half the time,
		{
			if (m_g2gCounter > 1)                                  // if there is room  left (2 intervals remaining: 1 and 0 )
			{
				m_targetFrameRate = m_targetFrameRate / 2.0;                // insert a double-length frame.
				m_g2gCounter--;                                             // do extra decrement to stay in sync
			}
		}
	}
#endif

	m_targetFrameTime = 1.0 / m_targetFrameRate;

	// run the timer
	m_g2gCounter--;
}

// update routine for Frame Drop test                                                                      6
void Game::UpdateFrameDrop()
{
	double maxFrameRate = m_maxFrameRate;
	double minFrameRate = m_minFrameRate;

	// check for overrides and apply
	if (m_maxFrameRateOverride != 0)
		maxFrameRate = m_maxFrameRateOverride;
	if (m_minFrameRateOverride != 0)
		minFrameRate = m_minFrameRateOverride;

	// determine what rate to use based on the up/down arrow key setting
	switch (m_frameDropRateEnum)
	{
	case DropRateEnum::Max:
		m_targetFrameRate = maxFrameRate;  // max reported by implementation
		break;
	case DropRateEnum::Random:			// pick a random frame duration within the valid range
		if (((float)rand() / RAND_MAX) > 0.50)	// randomly choose between Min and Max
			m_targetFrameRate = minFrameRate + m_frameRateMargin;
		else
			m_targetFrameRate = maxFrameRate - 2 * m_frameRateMargin;
		break;
	case DropRateEnum::SquareWave:  // stress test alternating between 24 and max
		if (m_frameCounter & 0x01)  // alternate between Min and Max
			//      if (m_frameCounter % 3 == 0 )
			m_targetFrameRate = minFrameRate + m_frameRateMargin;
		else
			m_targetFrameRate = maxFrameRate - 2 * m_frameRateMargin;
		break;
	case DropRateEnum::p48fps:
		m_targetFrameRate = 48;
		break;
	case DropRateEnum::p60fps:
		m_targetFrameRate = 60;
		break;
	case DropRateEnum::p72fps:
		m_targetFrameRate = 72;
		break;
	}

	m_targetFrameTime = 1.0 / m_targetFrameRate;
}

// Update any parameters used for animations, and also set the target frameTime;
void Game::Update()
{
	switch (m_currentTest)
	{
	case TestPattern::ConnectionProperties:  // 1
		UpdateDxgiRefreshRatesInfo();
		break;

	case TestPattern::PanelCharacteristics:  //
						 //      UpdateDxgiColorimetryInfo();
		break;

	case TestPattern::FlickerConstant:	// 2
		UpdateFlickerConstant();
		break;

	case TestPattern::FlickerVariable:	// 3
		UpdateFlickerVariable();
		break;

	case TestPattern::DisplayLatency:  // 4
	{
	}
	break;

	case TestPattern::GrayToGray:  // 5
	{
		UpdateGrayToGray();
	}
	break;

	case TestPattern::FrameDrop:  // 6
	{
		UpdateFrameDrop();
	}
	break;

	case TestPattern::FrameLock:  // 7
	{
		// This is handled in the draw routine
		m_targetFrameTime = 1.0 / m_targetFrameRate;
	}
	break;

	case TestPattern::MotionBlur:  // 8
	{
		m_targetFrameRate = m_motionBlurFrameRate;
		m_targetFrameTime = 1.0 / m_targetFrameRate;
	}
	break;

	case TestPattern::GameJudder:  // 9
	{
		m_targetFrameRate = m_judderTestFrameRate;
		m_targetFrameTime = 1.0 / m_targetFrameRate;
	}
	break;

	case TestPattern::Tearing:	// 10
	{
		m_targetFrameRate = m_tearingTestFrameRate;
		m_targetFrameTime = 1.0 / m_targetFrameRate;
	}
	break;

	case TestPattern::WarmUp:  // W
		if (m_newTestSelected)
		{
			m_testTimeRemainingSec =     60.0f * 60.0f;  // 60 minutes
			if ( m_brightMode )
				m_testTimeRemainingSec = 20.0f * 60.0f;  // 20 minutes
		}
		else
		{
			m_testTimeRemainingSec -= m_frameTime;
			m_testTimeRemainingSec = std::max(0.0, m_testTimeRemainingSec);
		}
		if (m_testTimeRemainingSec <= 0.0f)
			SetTestPattern(m_cachedTest);
		break;

	case TestPattern::Cooldown:	 // C
		if (m_newTestSelected)
		{
			m_testTimeRemainingSec = 60.0f;  // One minute is 60 seconds
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
	UNREFERENCED_PARAMETER(ctx);

	D2D1_BUFFER_PRECISION prec = D2D1_BUFFER_PRECISION_UNKNOWN;
	DXGI_FORMAT		  format = m_deviceResources->GetBackBufferFormat();
	switch (format)
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

// Keep value in range from min to max by clamping
float clamp(float x, float min, float max)  // inclusive
{
	if (x > max)
		return max;
	if (x < min)
		return min;
	return x;
}

// Keep value in from min to max by wrapping
int wrap(int x, int min, int max)  // inclusive
{
	if (max < min)
		return 0;
	int range = max - min + 1;
	x = (x - min) % range;
	if (x < 0)
		return max + 1 + x;
	else
		return min + x;
}

// routines to handle the 'from' and 'to' colors (currently on </> keys)
void Game::ProcessAngleBrackets(INT32 increment)
{
	switch (m_currentTest)
	{
	case TestPattern::ConnectionProperties:  // D
		if (m_shiftKey)
			increment *= 10;
		m_maxFrameRateOverride += increment;
		if (m_maxFrameRateOverride < 0.5)  // snap to zero so we can use == for no override
			m_maxFrameRateOverride = 0;
		break;

	case TestPattern::FlickerVariable:	// 3
		if (m_shiftKey)
			increment *= 8;
		m_SuccessiveFrameDurationIncreaseInterval += increment * 0.25;
		break;

	case TestPattern::GrayToGray:  // 5
		m_g2gFromIndex += increment;
		m_g2gFromIndex = wrap(m_g2gFromIndex, 0, numGtGValues - 1);
		break;
	}
}

void Game::ProcessSquareBrackets(INT32 increment)
{
	switch (m_currentTest)
	{
	case TestPattern::FlickerVariable:	// 3
		if (m_shiftKey)
			increment *= 8;
		m_SuccessiveFrameDurationDecreaseInterval += increment * 0.25;
		break;

	case TestPattern::GrayToGray:  // 5
		m_g2gToIndex += increment;
		m_g2gToIndex = wrap(m_g2gToIndex, 0, numGtGValues - 1);
		break;
	}
}

// routines to handle the +/- keys  -currently sineWave period, g2g frame skip factor, and motionblur level
void Game::ChangeG2GInterval(INT32 increment)
{
	switch (m_currentTest)
	{
	case TestPattern::FlickerVariable:	// 3
		if (m_shiftKey)
			increment *= 10;
		m_waveInterval += increment;
		m_waveInterval = clamp(m_waveInterval, 1, 3600);
		break;

	case TestPattern::GrayToGray:  // 5
		m_g2gInterval += increment;
		m_g2gInterval = clamp(m_g2gInterval, 1, 90);
		break;

	case TestPattern::FrameDrop:  // 6
		m_frameDropGamma += increment * 0.1;
		m_frameDropGamma = clamp(m_frameDropGamma, 0.1, 3.0);
		break;

	case TestPattern::MotionBlur:  // 8
		m_MotionBlurIndex += increment;
		if (m_MotionBlurIndex > maxMotionBlurs + 3)
			m_MotionBlurIndex = 0;
		if (m_MotionBlurIndex < 0)
			m_MotionBlurIndex = maxMotionBlurs + 3;
		break;
	}
}

// accessor for UI routine in main.cpp
void Game::SetShift(bool shift)
{
	m_shiftKey = shift;
}

// handle the up/down arrow key inputs (different for each test pattern)
void Game::ChangeSubtest(INT32 increment)
{
	int testInt;

	switch (m_currentTest)
	{
	case TestPattern::ConnectionProperties:  // D
		if (m_shiftKey)
			increment *= 10;
		m_minFrameRateOverride += increment;
		if (m_minFrameRateOverride < 0.5)  // snap to zero so we can use == for no override
			m_minFrameRateOverride = 0;
		break;

	case TestPattern::FlickerConstant:	// 2
		m_flickerRateIndex += increment;
		m_flickerRateIndex = wrap(m_flickerRateIndex, 0, numMediaRates);
		AutoResetAverageStats();
		break;

	case TestPattern::FlickerVariable:	// 3
		testInt = (int)m_waveEnum;
		testInt += increment;
		testInt = wrap(testInt, (int)WaveEnum::ZigZag, (int)WaveEnum::Max);
		m_waveEnum = static_cast<WaveEnum>(testInt);
		AutoResetAverageStats();
		break;

	case TestPattern::DisplayLatency:  // 4
		if (m_shiftKey)
			increment *= 10;
		if (m_sensing)
		{
			// adjust the sensor detection level
			m_sensorNits += increment;
			m_sensorNits = clamp(m_sensorNits, 0, 10000);
			m_sensor.SetActivationThreshold(m_sensorNits);
		}
		else
		{
			m_latencyTestFrameRate += increment;
			m_latencyTestFrameRate = std::clamp(m_latencyTestFrameRate, 20., 1000.);
			AutoResetAverageStats();
		}
		break;

	case TestPattern::GrayToGray:  // 5
		if (m_shiftKey)
			increment *= 10;
		m_g2gFrameRate += increment;
		m_g2gFrameRate = std::clamp(m_g2gFrameRate, 20.0, 1000.0);
		AutoResetAverageStats();
		break;

	case TestPattern::FrameDrop:  // 6
		testInt = (int)m_frameDropRateEnum;
		testInt += increment;
		testInt = wrap(testInt, (int)DropRateEnum::Max, (int)DropRateEnum::p72fps);
		m_frameDropRateEnum = static_cast<DropRateEnum>(testInt);
		AutoResetAverageStats();
		break;

	case TestPattern::FrameLock:  // use media frame rates here                             // 7
		m_frameLockRateIndex += increment;
		if (m_frameLockRateIndex > numMediaRates - 1)
			m_frameLockRateIndex = 0;
		if (m_frameLockRateIndex < 0)
			m_frameLockRateIndex = numMediaRates - 1;
		m_frameLockRateIndex = m_frameLockRateIndex % numMediaRates;
		AutoResetAverageStats();
		break;

	case TestPattern::MotionBlur:  // 8
		if (m_shiftKey)
			increment *= 10;
		m_motionBlurFrameRate += increment;
		m_motionBlurFrameRate = std::clamp(m_motionBlurFrameRate, 20., 1000.);
		AutoResetAverageStats();
		break;

	case TestPattern::GameJudder:  // 9
		if (m_shiftKey)
			increment *= 10;
		m_judderTestFrameRate += increment;
		m_judderTestFrameRate = std::clamp(m_judderTestFrameRate, 20., 1000.);
		AutoResetAverageStats();
		break;

	case TestPattern::Tearing:	// 0
		if (m_shiftKey)
			increment *= 10;
		m_tearingTestFrameRate += increment;
		m_tearingTestFrameRate = std::clamp(m_tearingTestFrameRate, 20., 1000.);
		AutoResetAverageStats();
		break;

	}
}
void Game::UpdateDxgiRefreshRatesInfo()
{
	// find min/maxFrameRates

	// Get information about the display we are presenting to.
	ComPtr<IDXGIOutput> output;
	auto		sc = m_deviceResources->GetSwapChain();
	DX::ThrowIfFailed(sc->GetContainingOutput(&output));

	// TODO: Get the current display refresh rate:
	DXGI_SWAP_CHAIN_FULLSCREEN_DESC pDescFS;
	sc->GetFullscreenDesc(&pDescFS);
	m_verticalSyncRate = pDescFS.RefreshRate;  // TODO: this only ever returns 0 even fullscreen.

	//  m_OSFrameRate = ??
	// this is from GDI
	HWND hwnd = ::GetDesktopWindow();
	HDC	 hdc = ::GetDC(hwnd);
	int	 refresh_rate = ::GetDeviceCaps(hdc, VREFRESH);
	UNREFERENCED_PARAMETER(refresh_rate);

	::ReleaseDC(hwnd, hdc);

	// Try GDI
	DEVMODE DevNode;
	EnumDisplaySettingsW(NULL, ENUM_CURRENT_SETTINGS, &DevNode);
	m_displayFrequency = DevNode.dmDisplayFrequency;  // This returns 0 on some configs

#if 0
	// Try DWM:
	DWM_TIMING_INFO DwmInfo;
	DwmInfo.cbSize = sizeof(DwmInfo);
	auto hr = DwmGetCompositionTimingInfo(NULL, &DwmInfo);
	double dwmRate = DwmInfo.rateRefresh.uiNumerator / DwmInfo.rateRefresh.uiDenominator;
	// how do I tell which monitor this is?
#endif

	//     DXGI_MODE_DESC dxgiMode;
	//  hr = output->FindClosestMatchingMode( NULL, &dxgiMode, NULL );

	DXGI_FORMAT format;
	if (CheckHDR_On())
	{
		format = DXGI_FORMAT_R16G16B16A16_FLOAT;
	}
	else
	{
		format = DXGI_FORMAT_R8G8B8A8_UNORM;
	}

	// first call to get the number of modes supported in this format
	UINT flags = 0;
	output->GetDisplayModeList(format, flags, &m_numModes, 0);

	// second call collects them into an array
	if (m_pModeList != NULL)
		free(m_pModeList);

	m_pModeList = new DXGI_MODE_DESC[m_numModes];
	output->GetDisplayModeList(format, flags, &m_numModes, m_pModeList);

	m_minFrameRate = 1000000;  // 1 million
	m_maxFrameRate = 0;
	for (uint32_t iMode = 0; iMode < m_numModes; iMode++)
	{
		if (m_pModeList[iMode].Width == m_modeWidth && m_pModeList[iMode].Height == m_modeHeight)
		{
			double rate = (double)m_pModeList[iMode].RefreshRate.Numerator / (double)m_pModeList[iMode].RefreshRate.Denominator;

			if (rate > m_maxFrameRate)
				m_maxFrameRate = rate;	// scan to find min/max
			if (rate < m_minFrameRate)
				m_minFrameRate = rate;
		}
	}
	// TODO tag which mode is current (defaults to highest for now)

	// print the fixed rate V-Total video frame rate range supported

	IDXGISwapChainMedia* scMedia;
	DX::ThrowIfFailed(sc->QueryInterface(IID_PPV_ARGS(&scMedia)));

	UINT closestSmallerPresentDuration, closestLargerPresentDuration;

	// find Min frame rate supported in PresentDuration mode by trying largest duration:
	DX::ThrowIfFailed(scMedia->CheckPresentDurationSupport(500000,  // 20Hz
		&closestSmallerPresentDuration,
		&closestLargerPresentDuration));
	m_maxDuration = closestSmallerPresentDuration;

	// find Max frame rate supported in PresentDuration mode by trying smallest duration:
	DX::ThrowIfFailed(scMedia->CheckPresentDurationSupport(1000,  // 10,000Hz
		&closestSmallerPresentDuration,
		&closestLargerPresentDuration));
	m_minDuration = closestLargerPresentDuration;

	// this may need to be adjusted based on the VESA criteria.  TODO
	if (m_minDuration > 0 && m_maxDuration > 0)  // if Durations are supported,
	{
		m_vTotalFixedSupported = true;

		m_minFrameRate = 10000000.f / m_maxDuration;  // then these should be non-zero
		m_maxFrameRate = 10000000.f / m_minDuration;
	}

	// initialize those scenes that should default to max frame rate
	// TODO: Keep this below actual frame rate of OS!

	//  m_maxFrameRate = 165;                           // TODO: test an odd number on occasion
	if (m_minFrameRate > 0)
		m_FrameRateRatio = m_maxFrameRate / m_minFrameRate;

	// set those rates that default to max (and apply override)

	if (m_latencyTestFrameRate < 5)  // 4
		m_latencyTestFrameRate = m_maxFrameRate;

	if (m_g2gFrameRate < 5)  // 5
		m_g2gFrameRate = m_maxFrameRate;

	// set up default switch interval based on max frame rate.
	m_g2gInterval = static_cast<int32_t>(0.25 * m_maxFrameRate);  // reset counter to take 250ms at ANY frame rate
	if (m_g2gInterval & 0x01)					  // if it is odd,
		m_g2gInterval++;					  // add one to make it even


	if (m_motionBlurFrameRate < 5)  // 8
		m_motionBlurFrameRate = m_maxFrameRate;

	if (m_judderTestFrameRate < 5)  // 9
		m_judderTestFrameRate = m_maxFrameRate;

	if (m_tearingTestFrameRate < 5)  // 0
		m_tearingTestFrameRate = m_maxFrameRate;
}

void Game::UpdateDxgiColorimetryInfo()
{
	// Output information is cached on the DXGI Factory. If it is stale we need to create
	// a new factory and re-enumerate the displays.
	auto d3dDevice = m_deviceResources->GetD3DDevice();

	// make sure we initialize the min/max modes even on startup
	DXGI_FORMAT format = m_deviceResources->GetBackBufferFormat();
	UNREFERENCED_PARAMETER(format);

	ComPtr<IDXGIDevice3> dxgiDevice;
	DX::ThrowIfFailed(d3dDevice->QueryInterface(IID_PPV_ARGS(&dxgiDevice)));

	ComPtr<IDXGIAdapter> dxgiAdapter;
	DX::ThrowIfFailed(dxgiDevice->GetAdapter(&dxgiAdapter));

	dxgiAdapter->GetDesc(&m_adapterDesc);

	ComPtr<IDXGIFactory4> dxgiFactory;
	DX::ThrowIfFailed(dxgiAdapter->GetParent(IID_PPV_ARGS(&dxgiFactory)));

	//  if (!dxgiFactory->IsCurrent())
	{
		DX::ThrowIfFailed(CreateDXGIFactory1(IID_PPV_ARGS(&dxgiFactory)));
	}

	// Get information about the display we are presenting to.
	ComPtr<IDXGIOutput> output;
	auto		sc = m_deviceResources->GetSwapChain();
	DX::ThrowIfFailed(sc->GetContainingOutput(&output));

	ComPtr<IDXGIOutput6> output6;
	output.As(&output6);

	DX::ThrowIfFailed(output6->GetDesc1(&m_outputDesc));

	// Get raw (not OS-modified) luminance data:
	DISPLAY_DEVICE device = {};
	device.cb = sizeof(device);

	bool found = false;

	for (UINT deviceIndex = 0; EnumDisplayDevices(m_outputDesc.DeviceName, deviceIndex, &device, EDD_GET_DEVICE_INTERFACE_NAME); deviceIndex++)
	{
		if (device.StateFlags & DISPLAY_DEVICE_ACTIVE)
		{
			found = true;

			auto displayMonitor = DisplayMonitor::FromInterfaceIdAsync(device.DeviceID).get();

			//          const auto displayMonitor = foundMonitor.get();
			//          DisplayMonitor displayMonitor(nullptr);

			auto dims = displayMonitor.NativeResolutionInRawPixels();
			m_modeWidth = dims.Width;
			m_modeHeight = dims.Height;

			// save the raw (not OS-modified) luminance data:
			m_rawOutDesc.MaxLuminance = displayMonitor.MaxLuminanceInNits();
			m_rawOutDesc.MaxFullFrameLuminance = displayMonitor.MaxAverageFullFrameLuminanceInNits();
			m_rawOutDesc.MinLuminance = displayMonitor.MinLuminanceInNits();
			// TODO: Should also get color primaries...

			// get PQ code at MaxLuminance
			m_maxPQCode = (int)roundf(1023.0f * Apply2084(m_rawOutDesc.MaxLuminance / 10000.f));

			m_monitorName = displayMonitor.DisplayName();

			m_connectionKind = displayMonitor.ConnectionKind();
			m_physicalConnectorKind = displayMonitor.PhysicalConnector();

			// get raw descriptor blob from this panel EDID or DisplayID
			auto buf = displayMonitor.GetDescriptor(DisplayMonitorDescriptorKind::Edid);
			if (buf.size() != 0)
			{
				m_connectionDescriptorKind = DisplayMonitorDescriptorKind::Edid;
			}
			else
			{
				buf = displayMonitor.GetDescriptor(DisplayMonitorDescriptorKind::DisplayId);
				if (buf.size() != 0)
				{
					m_connectionDescriptorKind = DisplayMonitorDescriptorKind::DisplayId;
				}
			}
			m_EDIDBlobSize = buf.size();
			break;
		}
	}

	if (!found)
	{
		// add error handling
	}

	m_dxgiColorInfoStale = false;

	// make sure refresh rates are also current
	UpdateDxgiRefreshRatesInfo();

	//  ACPipeline();
}

void Game::GenerateTestPattern_StartOfTest(ID2D1DeviceContext2* ctx)
{
	std::wstringstream text;

	text << m_appTitle;
	text << L" - " << versionString;
	text << L"\n\n";
	text << L"ALT-ENTER: Toggle fullscreen: all measurements should be made in fullscreen\n";
	text << L"->, PAGE DN:       Move to next test\n";
	text << L"<-, PAGE UP:        Move to previous test\n";
	text << L"NUMBER KEY:      Jump to test number\n";
	text << L"SPACE:                 Hide text and target circle\n";
	text << L"P, Pause:              Pause\n";
	text << L"R:           Reset sync timer\n";
	text << L"C:           Start 60s cool-down\n";
	text << L"W:          Start 60min warm-up\n";
	text << L"B:           Enable brighter warm-up & G2G\n";
	text << L"ALT-ENTER:    Toggle fullscreen\n";
	text << L"ESCAPE:          Exit fullscreen\n";
	text << L"ALT-F4:           Exit app\n";
	text << L"\nCopyright (C) VESA  - Last updated: " << __DATE__;

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
	case DXGI_COLOR_SPACE_RGB_FULL_G22_NONE_P709:  // sRGB
		break;

	case DXGI_COLOR_SPACE_RGB_FULL_G2084_NONE_P2020:  // HDR10 PC
		HDR_On = true;
		break;

	default:
		break;
	}

	return HDR_On;
}

void Game::GenerateTestPattern_ConnectionProperties(ID2D1DeviceContext2* ctx)  // ******************************* 1
{
	std::wstringstream text;

	text << L"Render GPU: ";
	text << m_adapterDesc.Description;

	text << L"\nMonitor: ";
	text << m_monitorName.c_str();

	text << L"\nDisplay: ";
	WCHAR* DisplayName = wcsrchr(m_outputDesc.DeviceName, '\\');
	text << ++DisplayName;

	text << L"\nConnector: ";

	switch (m_connectionKind)
	{
	case DisplayMonitorConnectionKind::Internal:
		text << "Internal Panel ";
		break;
	case DisplayMonitorConnectionKind::Wired:
		text << "Wired ";
		break;
	case DisplayMonitorConnectionKind::Wireless:
		text << "Wireless ";
		break;
	case DisplayMonitorConnectionKind::Virtual:
		text << "Virtual ";
		break;
	default:
		text << "Error";
		break;
	}

	// print the type of physical connector used by this display
	switch (m_physicalConnectorKind)
	{
	case DisplayMonitorPhysicalConnectorKind::Unknown:
		if (m_connectionKind != DisplayMonitorConnectionKind::Internal)
			text << "unknown";
		break;
	case DisplayMonitorPhysicalConnectorKind::HD15:
		text << "HD-15";
		break;
	case DisplayMonitorPhysicalConnectorKind::AnalogTV:
		text << "Analog TV";
		break;
	case DisplayMonitorPhysicalConnectorKind::Dvi:
		text << "DVI";
		break;
	case DisplayMonitorPhysicalConnectorKind::Hdmi:
		text << "HDMI";
		break;
	case DisplayMonitorPhysicalConnectorKind::Lvds:
		text << "LVDS";
		break;
	case DisplayMonitorPhysicalConnectorKind::Sdi:
		text << "SDI";
		break;
	case DisplayMonitorPhysicalConnectorKind::DisplayPort:
		text << "DisplayPort";
		break;
	default:
		text << "Error";
		break;
	}

	// Print the type of descriptor the display sent us
	switch (m_connectionDescriptorKind)
	{
	case DisplayMonitorDescriptorKind::Edid:
		text << " with EDID ";
		break;
	case DisplayMonitorDescriptorKind::DisplayId:
		text << " with DisplayID ";
		break;
	default:
		text << " Unknown";  // should never happen
		break;
	}

	// print number of blocks we found in the EDID
	text << m_EDIDBlobSize << "B " << m_EDIDBlobSize / 128 << "blocks";

	text << L"\nColorspace: [";
	text << std::to_wstring(m_outputDesc.ColorSpace);
	switch (m_outputDesc.ColorSpace)
	{
	case DXGI_COLOR_SPACE_RGB_FULL_G22_NONE_P709:  // sRGB
		text << L"] sRGB/SDR";
		break;

	case DXGI_COLOR_SPACE_RGB_FULL_G2084_NONE_P2020:  // HDR10 PC
		text << L"] HDR/advanced color on";
		break;
	default:
		text << L"] Unknown";
		break;
	}

	text << "\nResolution: " << m_modeWidth << " x " << m_modeHeight;
	text << " x " << std::to_wstring(m_outputDesc.BitsPerColor) << L"bits @ ";
	text << m_displayFrequency << L"Hz\n";

	text << "\nDesktop Display Modes supported at this resolution and bit depth:\n";

	// print out the modes in the current mode list that match current resolution:
	for (uint32_t iMode = 0; iMode < m_numModes; iMode++)
	{
		if (m_pModeList[iMode].Width == m_modeWidth && m_pModeList[iMode].Height == m_modeHeight)
		{
			UINT   num = m_pModeList[iMode].RefreshRate.Numerator;
			UINT   den = m_pModeList[iMode].RefreshRate.Denominator;
			double rate = (double)num / (double)den;
			text << L"      ";
			text << fixed << setw(10) << setprecision(3) << rate << "Hz  ";
			text << fixed << setw(10) << setprecision(4) << 1000. / rate << "ms";

			//          if (num == m_verticalSyncRate.Numerator
			//           && den == m_verticalSyncRate.Denominator)
			if (abs(rate - m_displayFrequency) < 0.1)
				text << L" <-- Current Windows maximum";

			text << L"\n";
		}
	}

	if (m_targetFrameRate > (double)(m_displayFrequency + 0.1))
		text << L"\nWARNING: **** Current Windows Settings may block testing of monitor peak rate ****\n";

	// print the video frame rate range supported
	text << "\nRange of refresh rates supported for fixed rate full-screen media playback:\n";

	text << "  From Min:  " << fixed;
	double minRate = m_minFrameRate;
	if (m_maxDuration > 0)
		minRate = 10000000.f / m_maxDuration;
	text << setw(7) << setprecision(3) << minRate << "Hz ";
	text << setw(8) << setprecision(4) << 1000.0 / minRate << "ms\n";  // scale units from hundreds of nanoseconds to ms

	text << "   To  Max:  ";
	double maxRate = m_maxFrameRate;
	if (m_minDuration > 0)
		maxRate = 10000000.f / m_minDuration;
	text << setw(7) << setprecision(3) << maxRate << "Hz ";
	text << setw(8) << setprecision(4) << 1000.0 / maxRate << "ms\n";  // scale units from hundreds of nanoseconds to ms

	text << "\n     Ratio: " << fixed << setw(8) << setprecision(3) << m_FrameRateRatio << "\n";

	// print the override values if present
	text << "\nOverrides of OS range: -set via up/down arrow keys\n";
	text << "  From Min:  " << fixed;
	text << setw(7) << setprecision(3) << m_minFrameRateOverride << "Hz ";
	text << setw(8) << setprecision(4) << 1000. / m_minFrameRateOverride << "ms\n";  // ms
	text << "   To  Max:  ";
	text << setw(7) << setprecision(3) << m_maxFrameRateOverride << "Hz ";
	text << setw(8) << setprecision(4) << 1000. / m_maxFrameRateOverride << "ms\n";  // ms


	RenderText(ctx, m_monospaceFormat.Get(), text.str(), m_largeTextRect);

	if (m_showExplanatoryText)
	{
		std::wstring title = L"Connection properties:\nPress SPACE to hide this text\n";
		RenderText(ctx, m_largeFormat.Get(), title, m_testTitleRect);
	}

	m_newTestSelected = false;
}

void Game::GenerateTestPattern_PanelCharacteristics(ID2D1DeviceContext2* ctx)  //         D
{
	std::wstringstream text;
	text << fixed << setw(9) << setprecision(2);

	// TODO: ensure these values are up to date.
	if (CheckForDefaults())
	{
		text << L"***WARNING: These are OS-provided defaults. Display did not provide levels.***\n";
	}

	//  text << m_outputDesc.DeviceName;
	text << L"\nBase Levels:" << fixed;
	text << L"\n  Max peak luminance:          " << setw(8);
	text << m_rawOutDesc.MaxLuminance;
	text << L"\n  Max frame-average luminance: " << setw(8);
	text << m_rawOutDesc.MaxFullFrameLuminance;
	text << L"\n  Min luminance:                  " << setw(8) << setprecision(5);
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
	UNREFERENCED_PARAMETER(gamutAreasRGB);

	// AdobeRGB gamut area in uv coordinates
	float GamutAreaAdobe = ComputeGamutArea(primaryR_Adobe, primaryG_Adobe, primaryB_Adobe);
	UNREFERENCED_PARAMETER(GamutAreaAdobe);

	// DCI-P3 gamut area in uv coordinates
	float gamutAreaDCIP3 = ComputeGamutArea(primaryR_DCIP3, primaryG_DCIP3, primaryB_DCIP3);
	UNREFERENCED_PARAMETER(gamutAreaDCIP3);

	// BT.2020 gamut area in uv coordinates
	float gamutAreaBT2100 = ComputeGamutArea(primaryR_2020, primaryG_2020, primaryB_2020);

	// ACES gamut area in uv coordinates
	float gamutAreaACES = ComputeGamutArea(primaryR_ACES, primaryG_ACES, primaryB_ACES);
	UNREFERENCED_PARAMETER(gamutAreaACES);

	const float gamutAreaHuman = 0.195f;  // TODO get this actual data!

	// Compute extent that this device gamut covers known popular ones:
	float coverageSRGB = ComputeGamutCoverage(red_xy, grn_xy, blu_xy, primaryR_709, primaryG_709, primaryB_709);
	float coverageAdobe = ComputeGamutCoverage(red_xy, grn_xy, blu_xy, primaryR_Adobe, primaryG_Adobe, primaryB_Adobe);
	float coverageDCIP3 = ComputeGamutCoverage(red_xy, grn_xy, blu_xy, primaryR_DCIP3, primaryG_DCIP3, primaryB_DCIP3);
	float coverage2100 = ComputeGamutCoverage(red_xy, grn_xy, blu_xy, primaryR_2020, primaryG_2020, primaryB_2020);
	float coverageACES = ComputeGamutCoverage(red_xy, grn_xy, blu_xy, primaryR_ACES, primaryG_ACES, primaryB_ACES);
	UNREFERENCED_PARAMETER(coverageACES);

	// display coverage values
	text << L"\n\nGamut Coverage based on reported primaries";
	text << L"\n        % sRGB : " << std::to_wstring(coverageSRGB * 100.f);
	text << L"\n    % AdobeRGB : " << std::to_wstring(coverageAdobe * 100.f);
	text << L"\n      % DCI-P3 : " << std::to_wstring(coverageDCIP3 * 100.f);
	text << L"\n     % BT.2100 : " << std::to_wstring(coverage2100 * 100.f);
	text << L"\n   % Human eye : " << std::to_wstring(gamutAreaDevice / gamutAreaHuman * 100.f);

	// test code:
	float theory = gamutAreaDevice / gamutAreaBT2100;
	UNREFERENCED_PARAMETER(theory);
	//  text << L"\n     %  Theory : " << std::to_wstring( theory*100.f);
	//  text << L"\n       % Error : " << std::to_wstring((coverage2100 - theory) / theory * 100.f);

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
	HANDLE display = CreateFile(L"\\\\.\\LCD", (GENERIC_READ | GENERIC_WRITE), NULL, NULL, OPEN_EXISTING, 0, NULL);

	if (display == INVALID_HANDLE_VALUE)
	{
		throw new std::runtime_error("Failed to open handle to display for setting brightness");
	}
	else
	{
		DWORD		   ret;
		DISPLAY_BRIGHTNESS displayBrightness{};
		displayBrightness.ucACBrightness = percent;
		displayBrightness.ucDCBrightness = percent;
		displayBrightness.ucDisplayPolicy = DISPLAYPOLICY_BOTH;

		bool result =
			!DeviceIoControl(display, IOCTL_VIDEO_SET_DISPLAY_BRIGHTNESS, &displayBrightness, sizeof(displayBrightness), NULL, 0, &ret, NULL);

		if (result)
		{
			throw new std::runtime_error("Failed to set brightness");
		}
	}
}

void Game::GenerateTestPattern_ResetInstructions(ID2D1DeviceContext2* ctx)
{
	if (m_showExplanatoryText)
	{
		std::wstring title = L"Start of performance tests\n" + m_hideTextString;
		RenderText(ctx, m_largeFormat.Get(), title, m_testTitleRect);
	}
	std::wstringstream text;
	text << L"For external displays, use their OSD to reset\n";
	text << L"all settings to the default or factory state.\n\n";

	text << L"For internal panels, set the OS \'Brightness\' slider to default.\n";
	//  text << L" in the Advanced color pane,";
	text << L"Set the \'SDR color appearance\' slider to the leftmost point.\n";
	//  text << L"DO NOT CHANGE BRIGHTNESS SLIDERS AFTER APP START.\n\n";

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
void Game::GenerateTestPattern_FlickerConstant(ID2D1DeviceContext2* ctx)  //************************************* 2
{
	if (m_newTestSelected)
		AutoResetAverageStats();

	// compute background color
	float c;
	if (CheckHDR_On())
	{
		float nits = 40.0f;
		c = nitstoCCCS(nits);
	}
	else
	{
		// code value to attain 10nits on an SDR monitor with 200nit page white
		float sRGBval = 127;
		c = sRGBval / 255.f;  // norm
	}

	ComPtr<ID2D1SolidColorBrush> flickerBrush;
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &flickerBrush));

	// Fill screen with a box
	auto logSize = m_deviceResources->GetLogicalSize();
	ctx->FillRectangle(&logSize, flickerBrush.Get());

	if (m_showExplanatoryText)
	{
		std::wstringstream title;
		title << versionString;
		title << L" Test 2 - Flicker at Constant Refresh Rate: ";

		//      title << m_flickerRateIndex;                    // debug printf
		switch (m_flickerRateIndex)
		{
		case 0:
			title << " Minimum";
			break;

		default:
			break;
		}
		if (m_targetFrameRate < (m_minFrameRate - 0.1))
			title << " Below Min - likely doubled.";
		title << "\n";

		if (m_targetFrameRate > (double)(m_displayFrequency + 0.1))
			title << L"\nWARNING: **** Windows Settings prevent operation over " << m_displayFrequency << L"Hz ****\n\n";

		title << fixed;
		title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
		title << setw(10) << setprecision(5) << m_targetFrameTime * 1000.f << L"ms\n";
		title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
		title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms  " << m_mediaVsyncCount << L"X\n";

		double avgFrameTime = m_totalFrameTime / m_frameCount;
		if (avgFrameTime < 0.0001)
		{
			bool x = true;
			UNREFERENCED_PARAMETER(x);
		}
		title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
		title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
		double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
		title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";

		title << "Monitor: " << setw(10) << setprecision(3) << m_monitorSyncRate << L"Hz";
		title << setw(9) << setprecision(1) << m_monitorSyncRate * avgFrameTime << "X\n";

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

		// display V-Total mode if requested
		if (m_vTotalModeRequested == VTotalMode::Fixed)
		{
			if (m_vTotalFixedApproved)
			{
				title << "V-Total: Fixed\n";  // green -got presentDuration
			}
			else if (m_MPO)
			{
				title << "MPO Overlay\n";  // blue  -just an overlay
			}
			else
			{
				title << "V-Total: Adaptive\n";	 // red   -none of the above
			}
		}
		else
			title << L"\n";

		if (m_logging)
			title << L"Logging" << setw(6) << setprecision(2) << m_logTime << L" sec\n";

		title << L"Adjust luminance to 40nits using UI slider or OSD\n";
		title << L"Select refresh rate using Up/Down arrows\n";
		title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	// dump data to log file
	if (m_logging)
	{
		fprintf_s(m_logFile,
			"%8.4f,%4.0f,%8.4f,%8.4f,%8.4f\n",
			m_lastLogTime * 1000.f,
			0.f,
			m_targetFrameTime * 1000.f,
			m_frameTime * 1000.,
			1000. / m_monitorSyncRate);
	}

	m_newTestSelected = false;
}

void Game::GenerateTestPattern_FlickerVariable(ID2D1DeviceContext2* ctx)  //************************************* 3
{
	if (m_newTestSelected)
		AutoResetAverageStats();

	// compute background color
	float c;
	if (CheckHDR_On())
	{
		float nits = 40.0f;
		c = nitstoCCCS(nits);
	}
	else
	{
		// code value to attain 10nits on an SDR monitor with 200nit page white
		float sRGBval = 127;
		c = sRGBval / 255.f;  // norm
	}

	ComPtr<ID2D1SolidColorBrush> flickerBrush;
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &flickerBrush));

	auto logSize = m_deviceResources->GetLogicalSize();
	ctx->FillRectangle(&logSize, flickerBrush.Get());

	if (m_showExplanatoryText)
	{
		std::wstringstream title;
		title << versionString;
		title << L" Test 3 - Flicker at Varying Refresh Rate: ";
		switch (m_waveEnum)
		{
		case WaveEnum::ZigZag:
			title << L"-Zig Zag  " << (m_waveUp ? " +\n" : "-\n");
			break;
		case WaveEnum::SquareWave:
			title << L"-Square Wave  " << (m_waveUp ? "+" : "-") << setw(m_waveCounter) << m_waveCounter << "\n";
			break;
		case WaveEnum::Random:
			title << L"-Random\n";
			break;
		case WaveEnum::SineWave:
			title << L"-Sine Wave " << m_waveInterval << "frames \n";
			break;
		case WaveEnum::Max:
			title << L"-Maximum\n";
			break;
		default:
			title << L"  E R R O R ! \n";
			break;
		}

		// would be nice to show the min max rates for square and sine wave,
		// but those values are not easy to get from here

		if (m_targetFrameRate > (double)(m_displayFrequency + 0.1))
			title << L"\nWARNING: **** Windows Settings prevent operation over " << m_displayFrequency << L"Hz ****\n\n";

		title << fixed;
		title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
		title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
		title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
		title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms  " << m_mediaVsyncCount << L"X\n";

		double avgFrameTime = m_totalFrameTime / m_frameCount;
		title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
		title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
		double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
		title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";

		title << "Monitor: " << setw(10) << setprecision(3) << m_monitorSyncRate << L"Hz";
		title << setw(9) << setprecision(1) << m_monitorSyncRate * avgFrameTime << "X\n";

		title << "m_SuccessiveFrameDurationIncreaseInterval";
		title << setw(6) << setprecision(2) << m_SuccessiveFrameDurationIncreaseInterval << " < >\n";
		title << "m_SuccessiveFrameDurationDecreaseInterval";
		title << setw(6) << setprecision(2) << m_SuccessiveFrameDurationDecreaseInterval << " [ ]\n";

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
		if (m_logging)
			title << L"Logging" << setw(6) << setprecision(2) << m_logTime << L" sec\n";

		title << L"Adjust luminance to 40nits using UI slider or OSD\n";
		title << L"Select zigzag vs square wave etc using Up/Down arrows\n";
		if (m_waveEnum == WaveEnum::SineWave)
			title << L"Adjust period of sine wave using +/- keys\n";
		title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	// dump data to log file
	if (m_logging)
	{
		fprintf_s(m_logFile,
			"%8.4f,%4.0f,%8.4f,%8.4f,%8.4f\n",
			m_lastLogTime * 1000.f,
			0.f,			       // mode
			m_targetFrameTime * 1000.f,  // requested frame ms
			m_frameTime * 1000.,	       // actual measured frame ms
			1000. / m_monitorSyncRate);  // frame duration from FrameStats
	}

	m_newTestSelected = false;
}

void Game::GenerateTestPattern_DisplayLatency(ID2D1DeviceContext2* ctx)	 // ************************************* 4
{
	if (m_newTestSelected)
	{
		// be sure to connect to sensor on entering this test
		if (!m_sensorConnected)
		{
			m_sensorConnected = m_sensor.ConnectSensor();
			m_sensor.SetActivationThreshold(m_sensorNits);
			ResetSensorStats();
		}

		AutoResetAverageStats();
	}

	// compute background color
	float c;
	if (CheckHDR_On())
	{
		float nits = 40.0f;
		c = nitstoCCCS(nits);
	}
	else
	{
		// TODO: change code value to attain 10nits on an SDR monitor with 200nit page white
		float sRGBval = 127;
		c = sRGBval / 255.f;  // norm
	}

	// fill the screen background with 100 nits white
	ComPtr<ID2D1SolidColorBrush> backgroundBrush;
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &backgroundBrush));
	auto logSize = m_deviceResources->GetLogicalSize();
	ctx->FillRectangle(&logSize, backgroundBrush.Get());

	m_targetFrameRate = m_latencyTestFrameRate;
	m_targetFrameTime = 1.0 / m_targetFrameRate;

	// draw a square 50mm on a side
	c = nitstoCCCS(200);
	ComPtr<ID2D1SolidColorBrush> whiteBrush;
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &whiteBrush));

	double	dpi = static_cast<double>(m_deviceResources->GetDpi());
	float	boxDim = static_cast<float>(50. / 25.4 * dpi * 1.2);  // 50mm -> inches -> dips
	D2D1_RECT_F tenPercentRect = { (logSize.right - logSize.left) * 0.5f - boxDim / 2.0f,
				  (logSize.bottom - logSize.top) * 0.5f - boxDim / 2.0f,
				  (logSize.right - logSize.left) * 0.5f + boxDim / 2.0f,
				  (logSize.bottom - logSize.top) * 0.5f + boxDim / 2.0f };

	// draw a circle 40mm in diameter
	float2	 center = float2(logSize.right * 0.5f, logSize.bottom * 0.5f);
	float	 fRad = static_cast<float>(0.5 * 40. / 25.4 * dpi * 1.2);  // radius of dia 40mm -> inches -> dips
	D2D1_ELLIPSE ellipse = { D2D1::Point2F(center.x, center.y), fRad, fRad };

	if (m_flash)  // if we want to flash the photocell
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
		title << versionString;
		title << L" Test 4 - Display Latency Measurement:  ";

		if (m_sensing)
		{
			title << m_frameCounter;
			// print frame rate we are sampling at:
			title << fixed;

#ifdef TESTING
			title << "\nTarget:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
			title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
			if (m_lastLastFlash)
				title << L"\n";
			title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
			title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms\n";
			if (!m_lastLastFlash)
				title << L"\n";
#else
			title << L"\n\n";
#endif

			int count = m_sensorCount;
			if (count <= 0)
				count = 1;
			double avgSensorTime = (m_totalSensorTime / m_sensorCount) - USB_TIME;
			double varSensorTime = sqrt(count * m_totalSensorTime2 - m_totalSensorTime * m_totalSensorTime) / count;

			if (m_sensorConnected)
			{
				// print total end-to-end latency stats from sensor
				title << " S E N S I N G  @ " << setprecision(0) << m_sensorNits << "Nits threshold\n";
				title << "\nSensor: " << setw(7) << setprecision(3) << m_sensorTime * 1000. << L"ms\n";	 // current value this frame

				title << "Over last " << m_sensorCount << " samples\n";
				title << "Avg: " << setprecision(3) << setw(7);
				title << avgSensorTime * 1000.0;
				title << "  Var: " << setprecision(3) << setw(7);
				title << varSensorTime * 1000.0;
				title << "\nMin: " << setprecision(3) << setw(7);
				title << m_minSensorTime * 1000.0;
				title << "  Max: " << setprecision(3) << setw(7);
				title << m_maxSensorTime * 1000.0;
				title << "\nUp/Down arrows adjust sensor nits threshold";
				title << "\nF1 key reconnects sensor";
			}
			else
				title << "\n    Please attach a photocell Sensor.\n";
		}
		else
		{
			title << L"\n";
			title << fixed;
			title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
			title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
			title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
			title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms  " << m_mediaVsyncCount << L"X\n";
			title << L"                                                          avg     var\n";

			int numCols = 10;
			assert(numCols <= frameLogLength);

			double dFrequency = static_cast<double>(m_qpcFrequency.QuadPart);  // counts per second of QPC

			// Display frame id counter
			title << "Frame ID:  ";
			for (int i = numCols - 1; i >= 0; i--)
			{
				title << setw(8) << m_frameLog[i]->frameID;
			}
			// show number of frames average is over
			title << setw(8) << m_frameCount;
			title << setw(8) << m_presentCount;
			//          title << "Average of " << m_frameCount << " frames: " << fixed << setprecision(3);

			// Display present id counter from DXGI_FRAME_STATISTICS
			title << "\nPresentID: ";
			for (int i = numCols - 1; i >= 2; i--)
			{
				title << setw(8) << m_frameLog[i]->presentID;
			}

			title << "\nLog Index: ";
			for (int i = numCols - 1; i >= 0; i--)
			{
				title << setw(8) << i;
			}

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

			// Print time spent running - not sleep()ing
			title << "\nRun Time:  " << setprecision(3);
			for (int i = numCols - 1; i > 0; i--)
			{
				double sleepTime = (m_frameLog[i]->updateCounts - m_frameLog[i]->sleepCounts) / dFrequency;
				double frameTime = (m_frameLog[i - 1]->readCounts - m_frameLog[i]->readCounts) / dFrequency;
				double time = frameTime - sleepTime;
				title << setw(8) << time * 1000.0f;
			}
			title << "        ";
			double avgRunTime = m_totalRunningTime / m_frameCount;
			double varRunTime = sqrt(m_frameCount * m_totalRunningTime2 - m_totalRunningTime * m_totalRunningTime) / m_frameCount;
			title << setprecision(3) << setw(8) << avgRunTime * 1000.f;
			title << setprecision(3) << setw(8) << varRunTime * 1000.f;

			// Print time to render on GPU
			title << "\nRender:    " << setprecision(3);
			for (int i = numCols - 1; i >= 1; i--)
			{
				double time = (m_frameLog[i]->presentCounts - m_frameLog[i]->drawCounts) / dFrequency;
				title << setw(8) << time * 1000.0f;
			}
			title << "        ";
			double avgRenderTime = m_totalRenderTime / m_frameCount;
			double varRenderTime = sqrt(m_frameCount * m_totalRenderTime2 - m_totalRenderTime * m_totalRenderTime) / m_frameCount;
			title << setprecision(3) << setw(8) << avgRenderTime * 1000.f;
			title << setprecision(3) << setw(8) << varRenderTime * 1000.f;

			// print how far out of sync the frame stats are vs our current frame counter
			title << setprecision(0) << setw(5) << m_frameStatsLag;

			// Print time to finalize image and flip -time from Present() to Vsync
			title << "\nPresent:   " << setprecision(3);
			for (int i = numCols - 1; i >= 2; i--)
			{
				double time = (m_frameLog[i]->syncCounts - m_frameLog[i]->presentCounts) / dFrequency;
				title << setw(8) << time * 1000.0f;
			}
			title << "                ";
			double avgPresentTime = m_totalPresentTime / m_frameCount;	// TODO: PresentCount Here?
			double varPresentTime = sqrt(m_frameCount * m_totalPresentTime2 - m_totalPresentTime * m_totalPresentTime) / m_frameCount;
			title << setprecision(3) << setw(8) << avgPresentTime * 1000.f;
			title << setprecision(3) << setw(8) << varPresentTime * 1000.f;

			// print range
			title << setprecision(3) << setw(8) << m_minPresentTime * 1000.f;
			title << setprecision(3) << setw(8) << m_maxPresentTime * 1000.f;

			// TODO REMOVE THIS HACK FOR TESTING to simulate a more consistent value from the presentStats API
			//         avgPresentTime = avgFrameTime / 2.0;

			title << "\nSensor: ";
			double avgSensorTime = (m_totalSensorTime / m_sensorCount) - USB_TIME;
			double varSensorTime = sqrt(m_sensorCount * m_totalSensorTime2 - m_totalSensorTime * m_totalSensorTime) / m_sensorCount;
			title << "Avg:" << setprecision(3) << setw(7);
			title << avgSensorTime * 1000.0;
			title << " Var:" << setprecision(3) << setw(7);
			title << varSensorTime * 1000.0;

			// Compute Display Latency
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
			// print out total LDAT E2E latency
			double LdatTime = m_avgInputTime + avgFrameTime + avgPresentTime + avgDisplayTime;
			title << "\nLDATs E2E: " << setprecision(3);
			title << setw(8) << LdatTime * 1000.0f;
			title << "ms or " << setprecision(3);
			title << setw(6) << LdatTime / avgFrameTime;
			title << " frames";
#endif

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
		title << "\n";

		if (m_logging)
			title << L"Logging" << setw(6) << setprecision(2) << m_logTime << L" sec\n";

		// there is some chance that reading the luminance interferes with timing...
#ifdef NOT_DEBUG
		float lum = m_sensor.ReadLuminance();
		if (m_flash)
			title << "       ";
		title << setprecision(0) << setw(5) << lum;
#endif
		title << "\nPress S to toggle Sensor sampling";
		title << "\nPress R to Reset stat counters";
		title << "\n" << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	// dump data to log file
	if (m_logging)
	{
		fprintf_s(m_logFile,
			"%8.4f,%4.0f,%8.4f,%8.4f,%8.4f\n",
			m_lastLogTime * 1000.f,
			0.f,
			m_targetFrameTime * 1000.f,
			m_frameTime * 1000.,
			1000. / m_monitorSyncRate);
	}

	m_newTestSelected = false;
}

void Game::GenerateTestPattern_GrayToGray(ID2D1DeviceContext2* ctx)  //********************************** 5.
{
	if (m_newTestSelected)
	{
		if (m_sensorConnected)
		{
			//          m_sensor.Disconnect();
			AutoResetAverageStats();
		}
	}

	// compute brush for surround -
	float c;
	if (CheckHDR_On())	// HDR
	{
		// should turn out to be 520/1023 in PQ/2084 code
		float nits = 40.0f;
		if (m_brightMode)
			nits = 80.0f;
		c = nitstoCCCS(nits);
	}
	else  // SDR
	{
		// per CTS section 10.2 - changed at meeting on 2021-06-22 to 40nits like other test background
		float sRGBval = 127.f;		  // in 8-bit encoding
		if (m_brightMode)
			sRGBval = 255.f;
		c = sRGBval / 255.f;  // norm
	}
	ComPtr<ID2D1SolidColorBrush> surroundBrush;	 // background
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &surroundBrush));

	// draw background
	auto logSize = m_deviceResources->GetLogicalSize();
	ctx->FillRectangle(&logSize, surroundBrush.Get());

	// decide color for foreground patch
	c = m_color;			     // computed in this pattern's Update method
	ComPtr<ID2D1SolidColorBrush> testBrush;  // 10% area test square
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &testBrush));

	// draw test pattern
	float size = sqrt((logSize.right - logSize.left) * (logSize.bottom - logSize.top));
	size = size * sqrtf(0.10);  // dimension of a square of 10% screen area
	float2 center;
	center.x = (logSize.right - logSize.left) * 0.50f;
	center.y = (logSize.bottom - logSize.top) * 0.50f;
	D2D1_RECT_F tenPercentRect = { center.x - size * 0.50f, center.y - size * 0.50f, center.x + size * 0.50f, center.y + size * 0.50f };
	ctx->FillRectangle(&tenPercentRect, testBrush.Get());

	// Everything below this point should be hidden during actual measurements.
	if (m_showExplanatoryText)
	{
		std::wstringstream title;
		title << versionString;
		title << L" Test 5 - Gray To Gray: ";
		title << L"Switching every " << m_g2gInterval << " frames. ";
		if (m_g2gFrom)
			title << L"From    ";
		else
			title << L"     To ";
		title << fixed << setw(m_g2gCounter + 1) << (m_g2gCounter + 1);

		if (m_targetFrameRate > (double)(m_displayFrequency + 0.1))
			title << L"\n\nWARNING: **** Windows Settings prevent operation over " << m_displayFrequency << L"Hz ****\n";

		title << "\nTarget:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
		title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
		title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
		title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms  " << m_mediaVsyncCount << L"X\n";

		double avgFrameTime = m_totalFrameTime / m_frameCount;
		title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
		title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
		double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
		title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";

		title << "Monitor: " << setw(10) << setprecision(3) << m_monitorSyncRate << L"Hz";
		title << setw(9) << setprecision(1) << m_monitorSyncRate * avgFrameTime << "X\n";

		if (m_autoG2G)
		{
			title << L"\nAutomatic sequence:\n";
		}
		else
		{
			title << L"Select 'From' brightness level using the '<' and '>' keys\n";
			title << L"Select  'To'  brightness level using the '[' and ']' keys\n";
			title << L"Select the b/w switch interval using the '+' and '-' keys\n";
		}
		if (!CheckHDR_On())
		{
			title << "From:" << setw(9) << setprecision(3) << 255 * m_g2gFromIndex / (numGtGValues - 1) << "/255";
			title << "  To:" << setw(9) << setprecision(3) << 255 * m_g2gToIndex / (numGtGValues - 1) << "/255\n";
		}
		else
		{
			title << "From:" << setw(9) << setprecision(3) << GrayToGrayValue(m_g2gFromIndex) * 80.f << "nits";
			title << "  To:" << setw(9) << setprecision(3) << GrayToGrayValue(m_g2gToIndex) * 80.f << "nits\n";
		}
		title << "Press A to toggle Automatic sequence\n";
		if (m_logging)
			title << L"Logging" << setw(6) << setprecision(2) << m_logTime << L" sec\n";
		title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	// dump data to log file
	if (m_logging)
	{
		fprintf_s(m_logFile,
			"%8.4f,%4.0f,%8.4f,%8.4f,%8.4f,%8.4f\n",
			m_lastLogTime * 1000.f,
			0.f,
			m_targetFrameTime * 1000.f,
			m_frameTime * 1000.,
			1000. / m_monitorSyncRate,
			c * 10. + 10.);
	}

	m_newTestSelected = false;
}

bool isPrime(int n)
{
	for (int i = 2; i <= n / 2; ++i)
	{
		if (n % i == 0)
		{
			return false;
		}
	}
	return true;
}

#define EXPOSURE_TIME (1.0)
#if defined(_DEBUG)
#define EXPOSURE_TIME (1.0)  // time camera aperture is open -in seconds
#endif
void Game::GenerateTestPattern_FrameDrop(ID2D1DeviceContext2* ctx)  //******************************************* 6
{
	if (m_newTestSelected)
		AutoResetAverageStats();

	D2D1_RECT_F			 logSize = m_deviceResources->GetLogicalSize();
	ComPtr<ID2D1SolidColorBrush> whiteBrush;  // brush for the "white" color

	float nits = m_outputDesc.MaxLuminance;  // for metadata

	float HDR10 = 150;
	nits = Remove2084(HDR10 / 1023.0f) * 10000.0f;  // "white" checker brightness

	float c = nitstoCCCS(nits);
	c = 0.50f;

	int	   nRows = 0, iCol, nCols = 0, jRow = 0, nCells;
	float2 step;

	switch (m_frameDropRateEnum)
	{
	case DropRateEnum::Max:
	case DropRateEnum::p48fps:
	case DropRateEnum::p60fps:
	case DropRateEnum::p72fps: {
		// figure out number of squares to draw
		double cells = m_targetFrameRate * EXPOSURE_TIME;  // works for 1/4 second exposure time on my phone

		// compute rows and columns for this many cells
		nCells = static_cast<int32_t>(round(cells));
		nRows = static_cast<int32_t>(floor(sqrt(cells))) + 1;
		nCols = nCells / nRows;
		// if it's not an integer match, then keep trying smaller values
		while (nRows * nCols != nCells)
		{
			nRows--;
			nCols = nCells / nRows;
		}
		if (nRows > nCols)
			swap(nRows, nCols);

		// compute size of rectangle to draw
		step.x = (logSize.right - logSize.left) / nCols;
		step.y = (logSize.bottom - logSize.top) / nRows;

		// compute position for current square
		iCol = m_frameCounter % nCols;
		jRow = (m_frameCounter / nCols) % nRows;

		if (iCol == 0)
			m_sweepPos = 0;

		break;
	}

	case DropRateEnum::Random: {
		// figure out number of squares to draw
		double avgFrameTime = (1.0 / m_maxFrameRate + 1.0 / m_minFrameRate) * 0.5;
		double avgFrameRate = 1.0 / avgFrameTime;
		double cells = avgFrameRate * EXPOSURE_TIME;

		// compute rows and columns for this many cells
		nCells = static_cast<int32_t>(round(cells));
		//      if (isPrime(nCells))
		//          nCells++;

		nRows = static_cast<int32_t>(floor(sqrt(cells))) + 1;
		nCols = nCells / nRows;
		// if it's not an integer match, then keep trying smaller values
		while (nRows * nCols != nCells)
		{
			nRows--;
			nCols = nCells / nRows;
		}
		if (nRows > nCols)
			swap(nRows, nCols);

		// compute size of square to draw
		step.x = (logSize.right - logSize.left) / nCols;
		step.y = (logSize.bottom - logSize.top) / nRows;

		step.x *= static_cast<float>(m_targetFrameTime / avgFrameTime);
		c = static_cast<float>(pow(0.2 + 0.5 * avgFrameTime / m_targetFrameTime, m_frameDropGamma));  // match brightness
		c += 0.15;  // make sure we always have some color to draw even for very long frames

		// compute position for current square
		iCol = m_frameCounter % nCols;
		jRow = (m_frameCounter / nCols) % nRows;

		if (iCol == (nCols - 1))
			step.x = logSize.right;

		if (iCol == 0)
			m_sweepPos = 0;

		break;
	}

	case DropRateEnum::SquareWave: {
		// figure out number of squares to draw
		double avgFrameTime = (1.0 / m_maxFrameRate + 1.0 / m_minFrameRate) * 0.5;
		double avgFrameRate = 1.0 / avgFrameTime;
		double cells = avgFrameRate * EXPOSURE_TIME;

		// compute rows and columns for this many cells
		nCells = static_cast<int32_t>(round(cells));
		if (isPrime(nCells))
			nCells++;
		nRows = static_cast<int32_t>(floor(sqrt(cells))) + 1;
		nCols = nCells / nRows;
		// if it's not an integer match, then keep trying smaller values
		while (nRows * nCols != nCells)
		{
			nRows--;
			nCols = nCells / nRows;
		}
		if (nRows > nCols)
			swap(nRows, nCols);

		// compute size of square to draw
		step.x = (logSize.right - logSize.left) / nCols;
		step.y = (logSize.bottom - logSize.top) / nRows;

		step.x *= static_cast<float>(m_targetFrameTime / avgFrameTime);
		c = static_cast<float>(pow(0.2 + 0.5 * avgFrameTime / m_targetFrameTime, m_frameDropGamma));  // match brightness
		c += 0.15;  // make sure we always have some color to draw even for very long frames

		// compute position for current square
		iCol = m_frameCounter % nCols;
		jRow = (m_frameCounter / nCols) % nRows;

		if (iCol == (nCols - 1))
			step.x = logSize.right;

		if (iCol == 0)
			m_sweepPos = 0;

		break;
	}

	default:
		// should assert here
		break;

	}  // switch

	// draw a checkerboard background for reference
//#define DRAW_CHECKER_BACKGROUND  // only works for regular spacing case, not random case
#ifdef DRAW_CHECKER_BACKGROUND
	for (int jRow = 0; jRow < nRows; jRow++)
	{
		for (int iCol = 0; iCol < nCols; iCol++)
		{
			if ((iCol + jRow) & 0x01)
			{
				D2D1_RECT_F rect = { iCol * step.x, jRow * step.y, (iCol + 1) * step.x, (jRow + 1) * step.y };
				ctx->FillRectangle(&rect, whiteBrush.Get());
			}
		}
	}
#endif

	// construct the rect now we know the dimensions
	D2D1_RECT_F rect = { m_sweepPos, jRow * step.y, m_sweepPos + step.x, (jRow + 1) * step.y };

	// now draw the current square:
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &whiteBrush));
	ctx->FillRectangle(&rect, whiteBrush.Get());
	m_sweepPos += step.x;

	// draw the gutters between rows as a reference black level
	float dy = step.y * 0.1f;
	for (int j = 0; j <= nRows; j++)
	{
		float y = j * step.y;
		rect = { logSize.left, y - dy, logSize.right, y + dy };
		ctx->FillRectangle(&rect, m_blackBrush.Get());
	}

	// Everything below this point should be hidden during actual measurements.
	if (m_showExplanatoryText)
	{
		std::wstringstream title;
		title << versionString;
		title << L" Test 6 - Frame Drop Test:  ";
		switch (m_frameDropRateEnum)
		{
		case DropRateEnum::Max:
			title << L"-Maximum\n";
			break;
		case DropRateEnum::Random:
			title << L"-Random " << fixed << setw(3) << setprecision(1) << m_frameDropGamma << "\n";
			break;
		case DropRateEnum::SquareWave:
			title << L"-Square Wave " << fixed << setw(3) << setprecision(1) << m_frameDropGamma << "\n";
			break;
		case DropRateEnum::p48fps:
			title << L"-fixed 48fps\n";
			break;
		case DropRateEnum::p60fps:
			title << L"-fixed 60fps\n";
			break;
		case DropRateEnum::p72fps:
			title << L"-fixed 72fps\n";
			break;
		default:
			title << L"  E R R O R ! \n";
			break;
		}

		if (m_targetFrameRate > (double)(m_displayFrequency + 0.1))
			title << L"\nWARNING: **** Windows Settings prevent operation over " << m_displayFrequency << L"Hz ****\n\n";

		title << fixed;
		title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
		title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
		title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
		title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms  " << m_mediaVsyncCount << L"X\n";

		double avgFrameTime = m_totalFrameTime / m_frameCount;
		title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
		title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
		double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
		title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";

		title << "Monitor: " << setw(10) << setprecision(3) << m_monitorSyncRate << L"Hz";
		title << setw(9) << setprecision(1) << m_monitorSyncRate * avgFrameTime << "X\n";

		title << "Grid " << nRows << " x " << nCols << L"\n";

		switch (m_frameDropRateEnum)
		{
		case DropRateEnum::SquareWave:
		case DropRateEnum::Random:
			title << L"Adjust brigthness of small vs large tiles via '+' and '-' keys\n";
			break;
		}

		title << L"Select refresh rate using Up/Down arrows\n";

		if (m_logging)
			title << L"Logging" << setw(6) << setprecision(2) << m_logTime << L" sec\n";
		title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	// dump data to log file
	if (m_logging)
	{
		fprintf_s(m_logFile,
			"%8.4f,%4.0f,%8.4f,%8.4f,%8.4f,%8.4f\n",
			m_lastLogTime * 1000.f,
			0.f,
			m_targetFrameTime * 1000.f,
			m_frameTime * 1000.,
			1000. / m_monitorSyncRate,
			c * 10.f);
	}

	m_newTestSelected = false;
}

void Game::GenerateTestPattern_FrameLock(ID2D1DeviceContext2* ctx)  //******************************************* 7
{
	if (m_newTestSelected)
		AutoResetAverageStats();

	D2D1_RECT_F			 logSize = m_deviceResources->GetLogicalSize();
	ComPtr<ID2D1SolidColorBrush> whiteBrush;  // brush for the "white" color

	float nits = m_outputDesc.MaxLuminance;  // for metadata

	float HDR10 = 150;
	nits = Remove2084(HDR10 / 1023.0f) * 10000.0f;  // "white" checker brightness

	float c = nitstoCCCS(nits);
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &whiteBrush));

	// decide shape of grid for test pattern
	int nCols = 10;
	int nRows = 6;
	switch (m_frameLockRateIndex)
	{
	case 0:
		m_targetFrameRate = 23.976f;
		nCols = 6;
		nRows = 4;
		break;

	case 1:
		m_targetFrameRate = 24.0f;
		nCols = 6;
		nRows = 4;
		break;

	case 2:
		m_targetFrameRate = 25.0f;
		nCols = 5;
		nRows = 5;
		break;

	case 3:
		m_targetFrameRate = 29.97f;
		nCols = 6;
		nRows = 5;
		break;

	case 4:
		m_targetFrameRate = 30.0f;
		nCols = 6;
		nRows = 5;
		break;

	case 5:
		m_targetFrameRate = 47.952f;
		nCols = 8;
		nRows = 6;
		break;

	case 6:
		m_targetFrameRate = 48.0f;
		nCols = 8;
		nRows = 6;
		break;

	case 7:
		m_targetFrameRate = 50.0f;
		nCols = 10;
		nRows = 5;
		break;

	case 8:
		m_targetFrameRate = 59.94f;
		nCols = 10;
		nRows = 6;
		break;

	case 9:
		m_targetFrameRate = 60.0f;
		nCols = 10;
		nRows = 6;
		break;

	default:
		m_targetFrameRate = 60.0f;
		nCols = 10;
		nRows = 6;
		break;
	}

	m_targetFrameTime = 1.0 / m_targetFrameRate;

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
				D2D1_RECT_F rect = { iCol * step.x, jRow * step.y, (iCol + 1) * step.x, (jRow + 1) * step.y };
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

	D2D1_RECT_F rect = { iCol * step.x, jRow * step.y, (iCol + 1) * step.x, (jRow + 1) * step.y };
	ctx->FillRectangle(&rect, whiteBrush.Get());

#if 0  // this needs exposure correction to align with the tiles drawn only 1 frame.
	// draw the gutters between rows as a reference white level
	float dy = step.y * 0.07f;
	for (int j = 0; j <= nRows; j++)
	{
		float y = j * step.y;
		rect = { logSize.left, y - dy, logSize.right, y + dy };
		ctx->FillRectangle(&rect, whiteBrush.Get());
	}
#endif

	// Everything below this point should be hidden during actual measurements.
	if (m_showExplanatoryText)
	{
		float fRad = sqrt((logSize.right - logSize.left) * (logSize.bottom - logSize.top) * 0.04f);  // 4% screen area colorimeter box
		UNREFERENCED_PARAMETER(fRad);

		float2 center = float2(logSize.right * 0.5f, logSize.bottom * 0.5f);

		std::wstringstream title;
		title << versionString;
		title << L" Test 7 - Framerate Lock or Jitter Test\n";

		if (m_targetFrameRate > (double)(m_displayFrequency + 0.1))
			title << L"\nWARNING: **** Windows Settings prevent operation over " << m_displayFrequency << L"Hz ****\n\n";

		title << fixed;
		title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
		title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";
		title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
		title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms  " << m_mediaVsyncCount << L"X\n";

		double avgFrameTime = m_totalFrameTime / m_frameCount;
		title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
		title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
		double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
		title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";

		title << "Monitor: " << setw(10) << setprecision(3) << m_monitorSyncRate << L"Hz";
		title << setw(9) << setprecision(1) << m_monitorSyncRate * avgFrameTime << "X\n";

		title << "Grid " << nRows << " x " << nCols << "\n";

		// display V-Total mode if requested
		if (m_vTotalModeRequested == VTotalMode::Fixed)
		{
			if (m_vTotalFixedApproved)
			{
				title << "V-Total: Fixed\n";  // green -got presentDuration
			}
			else if (m_MPO)
			{
				title << "MPO Overlay\n";  // blue  -just an overlay
			}
			else
			{
				title << "V-Total: Adaptive\n";	 // red   -none of the above
			}
		}
		else
			title << L"\n";

		title << L"Select refresh rate using Up/Down arrows\n";
		if (m_logging)
			title << L"Logging" << setw(6) << setprecision(2) << m_logTime << L" sec\n";
		title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	// dump data to log file
	if (m_logging)
	{
		fprintf_s(m_logFile,
			"%8.4f,%4.0f,%8.4f,%8.4f,%8.4f\n",
			m_lastLogTime * 1000.f,
			0.f,
			m_targetFrameTime * 1000.f,
			m_frameTime * 1000.,
			1000. / m_monitorSyncRate);
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

void Game::GenerateTestPattern_MotionBlur(ID2D1DeviceContext2* ctx)  // ***************************************** 8.
{
	if (m_newTestSelected)
		AutoResetAverageStats();

	// get window dimensions in pixels
	auto   logSize = m_deviceResources->GetLogicalSize();
	float2 center = float2(logSize.right - logSize.left, logSize.bottom - logSize.top) * 0.5f;

	// fill background
	ctx->FillRectangle(&logSize, m_blackBrush.Get());

	// compute rotation angle of bar in degrees for D2D
	float fDeltaAngle = static_cast<float>(360.0 * m_frameTime * 1.0);	// assume 2 revolution per second
	m_fAngle += fDeltaAngle;						// predict next frame time from this one

#define NUM_BLUR_FRAMES 32
	int numBlurFrames = NUM_BLUR_FRAMES;

	float			 c = 1.0f;			   // color for white
	float			 a = 1.0f / (float)numBlurFrames;  // alpha channel for additive transparency
	ComPtr<ID2D1SolidColorBrush> spinBrush;			   // for the spinner
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c, a), &spinBrush));
	ctx->SetPrimitiveBlend(D2D1_PRIMITIVE_BLEND_ADD);

	float	fWidth = min(center.x, center.y);  // make sure it fits on screen
	float	fHeight = fWidth * 0.05f;	     // aspect ratio of 20:1
	D2D1_RECT_F spinRect = { -fWidth, -fHeight, fWidth, fHeight };

	// what part of the frame time this panel is lit for
	float FrameFraction = (float)m_MotionBlurIndex / (float)maxMotionBlurs;

	// Draw the rectangle NumBlurFrames times at slightly different angles to blur it
	for (int i = 0; i < numBlurFrames; i++)
	{
		float fDel = -fDeltaAngle * FrameFraction / (float)numBlurFrames;

		ctx->SetTransform(D2D1::Matrix3x2F::Rotation(m_fAngle + fDel * i) * D2D1::Matrix3x2F::Translation(center.x, center.y));

		ctx->FillRectangle(&spinRect, spinBrush.Get());

		// clear transform after drawing
		ctx->SetTransform(D2D1::Matrix3x2F::Identity());
	}

	// draw the pivot
	float	 fRad = fHeight * 0.5f;
	D2D1_ELLIPSE ellipse = { D2D1::Point2F(center.x, center.y), fRad, fRad };
	ctx->SetPrimitiveBlend(D2D1_PRIMITIVE_BLEND_COPY);
	ctx->FillEllipse(&ellipse, m_blackBrush.Get());

	if (m_showExplanatoryText)
	{
		std::wstringstream title;
		title << versionString;
		title << L" Test 8 - Motion Blur Tuning\n";

		if (m_targetFrameRate > (double)(m_displayFrequency + 0.1))
			title << L"\nWARNING: **** Windows Settings prevent operation over " << m_displayFrequency << L"Hz ****\n\n";

		title << fixed;
		title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
		title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";

		title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
		title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms  " << m_mediaVsyncCount << L"X\n";

		double avgFrameTime = m_totalFrameTime / m_frameCount;
		title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
		title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
		double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
		title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";

		title << "Monitor: " << setw(10) << setprecision(3) << m_monitorSyncRate << L"Hz";
		title << setw(9) << setprecision(1) << m_monitorSyncRate * avgFrameTime << "X\n";

		title << "Frame Fraction: ";
		title << fixed << setw(6) << setprecision(2) << FrameFraction;	// in Percent?
		title << L"\nSelect frame rate using Up/Down arrows\n";
		title << L"Select Frame Fraction using +/- keys\n";

		// display V-Total mode if requested
		if (m_vTotalModeRequested == VTotalMode::Fixed)
		{
			if (m_vTotalFixedApproved)
			{
				title << "V-Total: Fixed\n";  // green -got presentDuration
			}
			else if (m_MPO)
			{
				title << "MPO Overlay\n";  // blue  -just an overlay
			}
			else
			{
				title << "V-Total: Adaptive\n";	 // red   -none of the above
			}
		}
		else
			title << L"\n";

		if (m_logging)
			title << L"Logging" << setw(6) << setprecision(2) << m_logTime << L" sec\n";
		title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	// dump data to log file
	if (m_logging)
	{
		fprintf_s(m_logFile,
			"%8.4f,%4.0f,%8.4f,%8.4f,%8.4f\n",
			m_lastLogTime * 1000.f,
			0.f,
			m_targetFrameTime * 1000.f,
			m_frameTime * 1000.,
			1000. / m_monitorSyncRate);
	}

	m_newTestSelected = false;
}

void Game::GenerateTestPattern_GameJudder(ID2D1DeviceContext2* ctx)  // **************************************** 9.
{
	float nits = 100;

	bool bBFI = true;
	if (m_frameTime > 0.025 ||		     // will flicker if too slow at under 50ms (40Hz)
		m_frameTime < 2.0 / m_maxFrameRate)  // no time for a frame to fit if too fast (>1/2 refresh rate)
	{
		bBFI = false;
	}

	if (bBFI)
		nits *= 2.0f;  // increase it if we are drawing black frames
		// should scale this based on duration of black frame

	float			 c = nitstoCCCS(nits);
	ComPtr<ID2D1SolidColorBrush> spinnerBrush;
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(c, c, c), &spinnerBrush));

	// get window dimensions in pixels
	auto   logSize = m_deviceResources->GetLogicalSize();
	float2 center = float2(logSize.right - logSize.left, logSize.bottom - logSize.top) * 0.5f;

	// fill background
	ctx->FillRectangle(&logSize, m_blackBrush.Get());

	float fDeltaAngle = static_cast<float>(360.0 * m_frameTime * 1.0);	// assume 2 revolutions per second
	m_fAngle += fDeltaAngle;						// predict next frame time from this one

	// skip drawing it every other frame as a black frame
	if ((m_frameCounter & 0x01) || (!bBFI))
	{
		ctx->SetTransform(D2D1::Matrix3x2F::Rotation(m_fAngle) * D2D1::Matrix3x2F::Translation(center.x, center.y));

		float fWidth = min(center.x, center.y);  // make sure it fits on screen
		float fHeight = fWidth * 0.05f;		  // aspect ratio of 20:1

		D2D1_RECT_F spinRect = { -fWidth, -fHeight, fWidth, fHeight };
		ctx->FillRectangle(&spinRect, spinnerBrush.Get());

		// clear transform after drawing
		ctx->SetTransform(D2D1::Matrix3x2F::Identity());

		// draw the pivot
		float	     fRad = fHeight * 0.5f;
		D2D1_ELLIPSE ellipse = { D2D1::Point2F(center.x, center.y), fRad, fRad };
		ctx->FillEllipse(&ellipse, m_blackBrush.Get());
	}

	if (m_showExplanatoryText)
	{
		std::wstringstream title;
		title << versionString;
		title << L" Test 9 - Game Judder Removal";
		if (bBFI)
			title << "  BFI";

		if (m_targetFrameRate > (double)(m_displayFrequency + 0.1))
			title << L"\n\nWARNING: **** Windows Settings prevent operation over " << m_displayFrequency << L"Hz ****\n";

		title << fixed;
		title << "\nTarget:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
		title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";

		title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
		title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms  " << m_mediaVsyncCount << L"X\n";

		double avgFrameTime = m_totalFrameTime / m_frameCount;
		title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
		title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
		double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
		title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";

		title << "Monitor: " << setw(10) << setprecision(3) << m_monitorSyncRate << L"Hz";
		title << setw(9) << setprecision(1) << m_monitorSyncRate * avgFrameTime << "X\n";

		title << L"\nSelect App Work time using Up/Down arrows\n";
		if (m_logging)
			title << L"Logging" << setw(6) << setprecision(2) << m_logTime << L" sec\n";
		title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	// dump data to log file
	if (m_logging)
	{
		fprintf_s(m_logFile,
			"%8.4f,%4.0f,%8.4f,%8.4f,%8.4f\n",
			m_lastLogTime * 1000.f,
			0.f,
			m_targetFrameTime * 1000.f,
			m_frameTime * 1000.,
			1000. / m_monitorSyncRate);
	}

	m_newTestSelected = false;
}

void Game::GenerateTestPattern_Tearing(ID2D1DeviceContext2* ctx)  // ****************************************** 0. T.
{
	if (m_newTestSelected)
		AutoResetAverageStats();

	// get window dimensions in pixels
	auto logSize = m_deviceResources->GetLogicalSize();

	// fill background
	ctx->FillRectangle(&logSize, m_blackBrush.Get());

	// compute rectangle size
	float fHeight = (logSize.bottom - logSize.top);
	float fWidth = (logSize.right - logSize.left);

	double duration = 0.250;  // time to sweep L -> R across screen in seconds

	// how far to move the bar each refresh:
	double selectedFrameTime = 1.0 / m_targetFrameRate;
	double pixPerFrame = fWidth * selectedFrameTime / duration;
	int	   nCols = static_cast<int>(round(duration / selectedFrameTime));

	// move the bar over:
	m_sweepPos += static_cast<float>(pixPerFrame);
	if (m_sweepPos > logSize.right - 5)	 // add tolerance in case of roundoff error
		m_sweepPos = 0.0;

	D2D1_RECT_F tearingRect = { m_sweepPos, 0., m_sweepPos + (float)pixPerFrame, fHeight };
	ctx->FillRectangle(&tearingRect, m_whiteBrush.Get());

	if (m_showExplanatoryText)
	{
		std::wstringstream title;
		title << versionString;
		title << L" Test 10 - Tearing Check\n";

		if (m_targetFrameRate > (double)(m_displayFrequency + 0.1))
			title << L"\nWARNING: **** Windows Settings prevent operation over " << m_displayFrequency << L"Hz ****\n\n";

		title << fixed;
		title << "Target:  " << setw(10) << setprecision(3) << m_targetFrameRate << L"fps  ";
		title << setw(10) << setprecision(5) << 1.0f / m_targetFrameRate * 1000.f << L"ms\n";

		title << "Current: " << setw(10) << setprecision(3) << 1.0 / m_frameTime << L"fps  ";
		title << setw(10) << setprecision(5) << m_frameTime * 1000.f << L"ms  " << m_mediaVsyncCount << L"X\n";

		double avgFrameTime = m_totalFrameTime / m_frameCount;
		title << "Average: " << setw(10) << setprecision(3) << 1.0 / avgFrameTime << L"fps  ";
		title << setw(10) << setprecision(5) << avgFrameTime * 1000.f << L"ms";
		double varFrameTime = sqrt(m_frameCount * m_totalFrameTime2 - m_totalFrameTime * m_totalFrameTime) / m_frameCount;
		title << setw(10) << setprecision(5) << varFrameTime * 1000.f << L"ms\n";

		title << "Monitor: " << setw(10) << setprecision(3) << m_monitorSyncRate << L"Hz";
		title << setw(9) << setprecision(1) << m_monitorSyncRate * avgFrameTime << "X\n";
		title << "Columns: " << nCols;

		title << L"\nSelect frame rate using Up/Down arrows\n";
		if (m_logging)
			title << L"Logging" << setw(6) << setprecision(2) << m_logTime << L" sec\n";
		title << m_hideTextString;

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}

	// dump data to log file
	if (m_logging)
	{
		fprintf_s(m_logFile,
			"%8.4f,%4.0f,%8.4f,%8.4f,%8.4f\n",
			m_lastLogTime * 1000.f,
			0.f,
			m_targetFrameTime * 1000.f,
			m_frameTime * 1000.,
			1000. / m_monitorSyncRate);
	}

	m_newTestSelected = false;
}

float3 roundf3(float3 in)
{
	float3 out;

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

void Game::GenerateTestPattern_WarmUp(ID2D1DeviceContext2* ctx)	 // **************************************** W
{
	// compute background color
	float c = 0.5;
	float nits = 200.;
	int	  sRGBval = 180;
	if (CheckHDR_On())
	{
		nits = 40.0f;
		if (m_brightMode)
			nits = 80.0f;
		c = nitstoCCCS(nits);
	}
	else
	{
		// code value to attain 40nits on an SDR monitor with 200nit page white
		sRGBval = 127;
		if (m_brightMode)
			sRGBval = 255;
		c = sRGBval / 255.f;  // norm
	}

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
			if (CheckHDR_On())
			{
				title << L"\nNits: " << nits;
				title << L"  HDR10: ";
				title << setprecision(0);
				title << Apply2084(c * 80.f / 10000.f) * 1023.f;
			}
			else
			{
				title << L"\nsRGBval: " << sRGBval << "  " << c * 100. << "%";
			}

			title << L"\n" << m_hideTextString;
		}
		else
		{
			title << L" done.";
		}

		RenderText(ctx, m_monospaceFormat.Get(), title.str(), m_testTitleRect);
	}
	m_newTestSelected = false;
}

// this is invoked via the C key
// it just draws a black screen for 60seconds.
void Game::GenerateTestPattern_Cooldown(ID2D1DeviceContext2* ctx)  // **************************************** C
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
	case TestPattern::ConnectionProperties:  // 1
		GenerateTestPattern_ConnectionProperties(ctx);
		break;
	case TestPattern::PanelCharacteristics:  // D
		GenerateTestPattern_PanelCharacteristics(ctx);
		break;
	case TestPattern::ResetInstructions:  // H
		GenerateTestPattern_ResetInstructions(ctx);
		break;
	case TestPattern::FlickerConstant:	// 2
		GenerateTestPattern_FlickerConstant(ctx);
		break;
	case TestPattern::FlickerVariable:	// 3
		GenerateTestPattern_FlickerVariable(ctx);
		break;
	case TestPattern::DisplayLatency:  // 4
		GenerateTestPattern_DisplayLatency(ctx);
		break;
	case TestPattern::GrayToGray:  // 5
		GenerateTestPattern_GrayToGray(ctx);
		break;
	case TestPattern::FrameDrop:  // 6
		GenerateTestPattern_FrameDrop(ctx);
		break;
	case TestPattern::FrameLock:  // 7
		GenerateTestPattern_FrameLock(ctx);
		break;
	case TestPattern::EndOfMandatoryTests:  //
		GenerateTestPattern_EndOfMandatoryTests(ctx);
		break;
	case TestPattern::MotionBlur:  // 8
		GenerateTestPattern_MotionBlur(ctx);
		break;
	case TestPattern::GameJudder:  // 9
		GenerateTestPattern_GameJudder(ctx);
		break;
	case TestPattern::Tearing:	// 0
		GenerateTestPattern_Tearing(ctx);
		break;
	case TestPattern::EndOfTest:
		GenerateTestPattern_EndOfTest(ctx);
		break;
	case TestPattern::WarmUp:  // W
		GenerateTestPattern_WarmUp(ctx);
		break;
	case TestPattern::Cooldown:	 // C
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
	auto		      fact = m_deviceResources->GetDWriteFactory();
	ComPtr<IDWriteTextLayout> layout;
	DX::ThrowIfFailed(fact->CreateTextLayout(text.c_str(), (unsigned int)text.length(), fmt, textPos.right, textPos.bottom, &layout));

	if (useBlackText)
	{
		// add highlight
		//      ctx->DrawTextLayout(D2D1::Point2F(textPos.left, textPos.top), layout.Get(), m_whiteBrush.Get());
		ctx->DrawTextLayout(D2D1::Point2F(textPos.left + 1, textPos.top + 1), layout.Get(), m_blackBrush.Get());
	}
	else
	{
		// add dropshadow
		ctx->DrawTextLayout(D2D1::Point2F(textPos.left + 1.0f, textPos.top + 1.0f), layout.Get(), m_blackBrush.Get());
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
void			Game::CreateDeviceIndependentResources()
{
	auto dwFactory = m_deviceResources->GetDWriteFactory();

	DX::ThrowIfFailed(dwFactory->CreateTextFormat(
		L"Segoe UI", nullptr, DWRITE_FONT_WEIGHT_NORMAL, DWRITE_FONT_STYLE_NORMAL, DWRITE_FONT_STRETCH_NORMAL, 14.0f, L"en-US", &m_smallFormat));

	DX::ThrowIfFailed(dwFactory->CreateTextFormat(
		L"Consolas", nullptr, DWRITE_FONT_WEIGHT_NORMAL, DWRITE_FONT_STYLE_NORMAL, DWRITE_FONT_STRETCH_NORMAL, 18.0f, L"en-US", &m_monospaceFormat));

	DX::ThrowIfFailed(dwFactory->CreateTextFormat(
		L"Segoe UI", nullptr, DWRITE_FONT_WEIGHT_NORMAL, DWRITE_FONT_STYLE_NORMAL, DWRITE_FONT_STRETCH_NORMAL, 24.0f, L"en-US", &m_largeFormat));
}

// These are the resources that depend on the device.
void Game::CreateDeviceDependentResources()
{
	auto ctx = m_deviceResources->GetD2DDeviceContext();

	// these standard D2D values are really only valid for SDR mode!  TODO: make work in HDR
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::White, 1.f), &m_whiteBrush));
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Black, 1.f), &m_blackBrush));
	DX::ThrowIfFailed(ctx->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 1.f), &m_redBrush));

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
	m_testTitleRect = { 0.0f, 0.0f, 1280.0f, 100.0f };

	auto logicalSize = m_deviceResources->GetLogicalSize();

	m_largeTextRect = { (logicalSize.right - logicalSize.left) * 0.15f,
			   (logicalSize.bottom - logicalSize.top) * 0.15f,
			   (logicalSize.right - logicalSize.left) * 0.75f,
			   (logicalSize.bottom - logicalSize.top) * 0.75f };

	m_MetadataTextRect = { logicalSize.right - 700.0f, logicalSize.bottom - 35.0f, logicalSize.right, logicalSize.bottom };
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
			HRESULT		      hr = wicFactory->CreateDecoderFromFilename(
				resources->imageFilename.c_str(), nullptr, GENERIC_READ, WICDecodeMetadataCacheOnDemand, &decoder);

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
			DX::ThrowIfFailed(converter->Initialize(frame.Get(), outFmt, WICBitmapDitherTypeNone, nullptr, 0.0f, WICBitmapPaletteTypeCustom));

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

// called on app exit to clean up
void Game::DisconnectSensor(void)
{
	m_sensor.~Sensor();
}

void Game::ReconnectSensor(void)
{
	if (m_sensing)
	{
		// disconnect
		//      m_sensor.~Sensor();
		m_sensor.Disconnect();

		// wait 1/2 second
		Sleep(500);

		// connect again
		m_sensorConnected = m_sensor.ConnectSensor();
		m_sensor.SetActivationThreshold(m_sensorNits);
		if (!m_sensorConnected)
		{
			bool x = true;
			UNREFERENCED_PARAMETER(x);
		}
		ResetSensorStats();
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
	if (testPattern == TestPattern::Cooldown && m_currentTest != TestPattern::Cooldown)
		m_cachedTest = m_currentTest;

	if (testPattern == TestPattern::WarmUp && m_currentTest != TestPattern::WarmUp)
		m_cachedTest = m_currentTest;

	if (TestPattern::StartOfTest <= testPattern)
	{
		if (TestPattern::Cooldown >= testPattern)
		{
			m_currentTest = testPattern;
		}
	}

	//  m_showExplanatoryText = true;
	m_newTestSelected = true;
}
// Set increment = true to go up, false to go down.
void Game::ChangeTestPattern(bool increment)
{
	if (TestPattern::Cooldown == m_currentTest || TestPattern::WarmUp == m_currentTest)
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
	else  //decrement
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

	// stop sensing if we switch tests
	m_sensing = false;

	// If logging, then stop when we switch tests
	if (m_logging)
		ToggleLogging();

	//  m_showExplanatoryText = true;
	m_newTestSelected = true;
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
