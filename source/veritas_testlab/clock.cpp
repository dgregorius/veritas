//--------------------------------------------------------------------------------------------------
// clock.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "clock.h"

// OpenGL
#include <glad.h>
#include <glfw3.h>

// STD
#include <chrono>
#include <thread>


//--------------------------------------------------------------------------------------------------
// VsClock
//--------------------------------------------------------------------------------------------------
void VsClock::Start()
	{
	mTime = 0.0;
	mElapsedTime = 0.0f;
	mPaused = false;
	glfwSetTime( mTime );
	}


//--------------------------------------------------------------------------------------------------
void VsClock::Stop()
	{
	mTime = 0.0;
	mElapsedTime = 0.0f;
	mPaused = false;
	glfwSetTime( mTime );
	}


//--------------------------------------------------------------------------------------------------
void VsClock::Advance()
	{
	if ( !mPaused )
		{
		double Time = glfwGetTime();
		double TargetTime = mTime + 1.0 / mFrequency;
		while ( Time < TargetTime )
			{
			std::this_thread::yield();
			Time = glfwGetTime();
			}

		mElapsedTime = static_cast< float >( Time - mTime );
		mTime = Time;
		}
	}


//--------------------------------------------------------------------------------------------------
float VsClock::GetFrequency()
	{
	return mFrequency;
	}


//--------------------------------------------------------------------------------------------------
void VsClock::SetFrequency( float Frequency )
	{
	mFrequency = Frequency;
	}


//--------------------------------------------------------------------------------------------------
void VsClock::Pause()
	{
	mPaused = true;
	}


//--------------------------------------------------------------------------------------------------
void VsClock::Unpause()
	{
	mPaused = false;
	}


//--------------------------------------------------------------------------------------------------
void VsClock::SetPaused( bool Paused )
	{
	mPaused = Paused;
	}


//--------------------------------------------------------------------------------------------------
bool VsClock::IsPaused()
	{
	return mPaused;
	}


//--------------------------------------------------------------------------------------------------
double VsClock::GetTime()
	{
	return mTime;
	}


//--------------------------------------------------------------------------------------------------
float VsClock::GetElapsedTime()
	{
	return !mPaused ? mElapsedTime : 0.0f;
	}


//--------------------------------------------------------------------------------------------------
// Timer
//--------------------------------------------------------------------------------------------------
uint64_t vsGetTicks()
	{
	using VSClock = std::chrono::high_resolution_clock;
	return VSClock::now().time_since_epoch().count();
	}


//--------------------------------------------------------------------------------------------------
double vsTicksToMilliSeconds( uint64_t Ticks )
	{
	using VSClock = std::chrono::high_resolution_clock;
	static const uint64_t Frequency = VSClock::period::den / VSClock::period::num;

	// 1. Separate whole seconds (Integer math)
	uint64_t WholeSeconds = Ticks / Frequency;

	// 2. Separate remaining ticks (Integer math)
	uint64_t RemainderTicks = Ticks % Frequency;

	// 3. Combine them into milliseconds
	return ( WholeSeconds * 1000.0 ) + ( RemainderTicks * 1000.0 / (double)Frequency );
	}

//--------------------------------------------------------------------------------------------------
double vsTicksToSeconds( uint64_t Ticks )
	{
	return vsTicksToMilliSeconds( Ticks ) / 1000.0;
	}