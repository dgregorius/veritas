//--------------------------------------------------------------------------------------------------
// qhMemory.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "qhMemory.h"
#include "qhTypes.h"

#include <malloc.h>


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static void* qhDefaultAlloc( size_t Bytes, size_t Alignment )
	{
	return _aligned_malloc( Bytes, Alignment );
	}


//--------------------------------------------------------------------------------------------------
static void qhDefaultFree( void* Address )
	{
	return _aligned_free( Address );
	}


//--------------------------------------------------------------------------------------------------
// qhMemory
//--------------------------------------------------------------------------------------------------
void* ( *qhAllocHook )( size_t, size_t ) = qhDefaultAlloc;
void ( *qhFreeHook )( void* ) = qhDefaultFree;


//--------------------------------------------------------------------------------------------------
void* qhAlloc( size_t Bytes, size_t Alignment )
	{
	QH_ASSERT( qhAllocHook != nullptr );
	return qhAllocHook( Bytes, Alignment );	
	}


//--------------------------------------------------------------------------------------------------
void qhFree( void* Address )
	{
	QH_ASSERT( qhFreeHook != nullptr );
	qhFreeHook( Address );
	}


