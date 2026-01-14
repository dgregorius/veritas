//--------------------------------------------------------------------------------------------------
// log.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "log.h"


//--------------------------------------------------------------------------------------------------
// RkLog
//--------------------------------------------------------------------------------------------------
RkSignal< void ( const char* ) > RkLog::MessagedDisplayed;
RkSignal< void ( const char* ) > RkLog::WarningDisplayed;
RkSignal< void ( const char* ) > RkLog::ErrorDisplayed;


//--------------------------------------------------------------------------------------------------
void RkLog::DisplayMessage( const char* Format, ... )
	{
	va_list Args;
	va_start( Args, Format );
	DisplayMessageV( Format, Args );
	va_end( Args );
	}


//--------------------------------------------------------------------------------------------------
void RkLog::DisplayMessageV( const char* Format, va_list Args )
	{
	char Message[ 512 ] = { 0 };
	vsprintf_s( Message, Format, Args );
	printf_s( Message );

	MessagedDisplayed.Emit( Message );
	}

//--------------------------------------------------------------------------------------------------
void RkLog::DisplayWarning( const char* Format, ... )
	{
	va_list Args;
	va_start( Args, Format );
	DisplayWarningV( Format, Args );
	va_end( Args );
	}


//--------------------------------------------------------------------------------------------------
void RkLog::DisplayWarningV( const char* Format, va_list Args )
	{
	char Warning[ 512 ] = { 0 };
	vsprintf_s( Warning, Format, Args );
	printf_s( Warning );

	WarningDisplayed.Emit( Warning );
	}

//--------------------------------------------------------------------------------------------------
void RkLog::DisplayError( const char* Format, ... )
	{
	va_list Args;
	va_start( Args, Format );
	DisplayErrorV( Format, Args );
	va_end( Args );
	}


//--------------------------------------------------------------------------------------------------
void RkLog::DisplayErrorV( const char* Format, va_list Args )
	{
	char Error[ 512 ] = { 0 };
	vsprintf_s( Error, Format, Args );
	printf_s( Error );

	ErrorDisplayed.Emit( Error );
	}

