//--------------------------------------------------------------------------------------------------
// timer.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "timer.h"
#include "assert.h"

#define VC_EXTRALEAN
#include <windows.h>

static uint64 gTimerOffset = rkGetTicks();


//--------------------------------------------------------------------------------------------------
// RnTimer
//--------------------------------------------------------------------------------------------------
double rkGetTime()
	{
	return static_cast< double >( rkGetTicks() - gTimerOffset ) / static_cast< double >( rkGetTicksPerSecond() );
	}


//--------------------------------------------------------------------------------------------------
void rkSetTime( double Time )
	{
	gTimerOffset = rkGetTicks() - static_cast< uint64 >( Time * rkGetTicksPerSecond() );
	}


//--------------------------------------------------------------------------------------------------
uint64 rkGetTicks()
	{
	LARGE_INTEGER Integer;
	QueryPerformanceCounter( &Integer );

	return Integer.QuadPart;
	}


//--------------------------------------------------------------------------------------------------
uint64 rkGetTicksPerSecond()
	{
	static uint64 Frequency = 0;
	if ( Frequency == 0 )
		{
		LARGE_INTEGER Integer;
		QueryPerformanceFrequency( &Integer );
		Frequency = Integer.QuadPart;
		}
	
	RK_ASSERT( Frequency );
	return Frequency;
	}


//--------------------------------------------------------------------------------------------------
float rkTicksToSeconds( uint64 Ticks )
	{
	return float( Ticks ) / float( rkGetTicksPerSecond() );
	}


//--------------------------------------------------------------------------------------------------
float rkTicksToMilliSeconds( uint64 Ticks )
	{
	return float( Ticks ) / float( rkGetTicksPerSecond() ) * 1000.0f;
	}


//--------------------------------------------------------------------------------------------------
void rkYield()
	{
	SwitchToThread();
	}