//--------------------------------------------------------------------------------------------------
/*
	@file		log.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/signal.h"

#include <stdarg.h>
#include <stdio.h>


//--------------------------------------------------------------------------------------------------
// RkLog
//--------------------------------------------------------------------------------------------------
struct RkLog
	{
	static void DisplayMessage( const char* Format, ... );
	static void DisplayMessageV( const char* Format, va_list Args );
	static RkSignal< void ( const char* ) > MessagedDisplayed;

	static void DisplayWarning( const char* Format, ... );
	static void DisplayWarningV( const char* Format, va_list Args );
	static RkSignal< void ( const char* ) > WarningDisplayed;

	static void DisplayError( const char* Format, ... );
	static void DisplayErrorV( const char* Format, va_list Args );
	static RkSignal< void ( const char* ) > ErrorDisplayed;
	};