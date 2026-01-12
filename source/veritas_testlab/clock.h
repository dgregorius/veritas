//--------------------------------------------------------------------------------------------------
/*
	@file		clock.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <stdint.h>


//--------------------------------------------------------------------------------------------------
// VsClock
//--------------------------------------------------------------------------------------------------
class VsClock
	{
	public:
		static void Start();
		static void Stop();
		static void Advance();

		static float GetFrequency();
		static void SetFrequency( float Frequency );

		static void Pause();
		static void Unpause();
		static void SetPaused( bool Paused );
		static bool IsPaused();

		static double GetTime();
		static float GetElapsedTime();
	
	private:
		static inline double mTime = 0.0f;
		static inline float mElapsedTime = 0.0f;
		static inline float mFrequency = 60.0f;
		static inline bool mPaused = true;
	};


//--------------------------------------------------------------------------------------------------
// Timer
//--------------------------------------------------------------------------------------------------
uint64_t vsGetTicks();
double vsTicksToMilliSeconds( uint64_t Ticks );
double vsTicksToSeconds( uint64_t Ticks );