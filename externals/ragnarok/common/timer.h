//--------------------------------------------------------------------------------------------------
/*
	@file		timer.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/types.h"


//--------------------------------------------------------------------------------------------------
// RkTimer
//--------------------------------------------------------------------------------------------------
double rkGetTime();
void rkSetTime( double Time );

uint64 rkGetTicks(); 
uint64 rkGetTicksPerSecond();
float rkTicksToSeconds( uint64 Ticks );
float rkTicksToMilliSeconds( uint64 Ticks );
void rkYield();