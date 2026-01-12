//--------------------------------------------------------------------------------------------------
// clock.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "clock.h"

// Windows
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// OpenGL
#include <glad.h>
#include <glfw3.h>


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static inline void rkYield()
	{
	SwitchToThread();
	}


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
			rkYield();
			Time = glfwGetTime();
			}
		mElapsedTime = static_cast< float >( Time - mTime );
		mTime = Time;
		}
	}


//--------------------------------------------------------------------------------------------------
double VsClock::GetFrequency()
	{
	return mFrequency;
	}


//--------------------------------------------------------------------------------------------------
void VsClock::SetFrequency( double Frequency )
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