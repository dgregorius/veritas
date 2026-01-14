//--------------------------------------------------------------------------------------------------
// tracy.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "tracy.h"

// Pull Tracy
#ifdef TRACY_ENABLE
#  include <TracyClient.cpp>
#endif

//--------------------------------------------------------------------------------------------------
// Tracy Profiler Thread naming
//--------------------------------------------------------------------------------------------------
TracyWorkerInterface::TracyWorkerInterface()
	{
#if defined( TRACY_ENABLE )
	tracy::SetThreadName( "Main" );
#endif
	}


//--------------------------------------------------------------------------------------------------
void TracyWorkerInterface::scheduler_prologue( tf::Worker& Worker )
	{
#if defined( TRACY_ENABLE )
	int GroupHint = static_cast< int >( Worker.id() ) + 1;
	std::string Name = "Worker (" + std::to_string( GroupHint ) + ")";
	tracy::SetThreadNameWithHint( Name.c_str(), GroupHint );
#endif
	}


//--------------------------------------------------------------------------------------------------
void TracyWorkerInterface::scheduler_epilogue( tf::Worker& Worker, std::exception_ptr )
	{

	}
