// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "constants.h"
#include "box3d/math_functions.h"

#if defined( B3_COMPILER_MSVC )
	#define _CRTDBG_MAP_ALLOC
	#include <crtdbg.h>
	#include <stdlib.h>
#else
	#include <stdlib.h>
#endif

#include <stdarg.h>
#include <string.h>

#ifdef BOX2D_PROFILE

	#include <tracy/TracyC.h>
	#define b3TracyCAlloc( ptr, size ) TracyCAlloc( ptr, size )
	#define b3TracyCFree( ptr ) TracyCFree( ptr )

#else

	#define b3TracyCAlloc( ptr, size )
	#define b3TracyCFree( ptr )

#endif

#include "platform.h"

#include <stdio.h>

// This allows the user to change the length units at runtime
float b3_lengthUnitsPerMeter = 1.0f;

void b3SetLengthUnitsPerMeter( float lengthUnits )
{
	B3_ASSERT( b3IsValidFloat( lengthUnits ) && lengthUnits > 0.0f );
	b3_lengthUnitsPerMeter = lengthUnits;
}

float b3GetLengthUnitsPerMeter( void )
{
	return b3_lengthUnitsPerMeter;
}

static int b3DefaultAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	printf( "BOX3D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );

	// return non-zero to break to debugger
	return 1;
}

b3AssertFcn* b3AssertHandler = b3DefaultAssertFcn;

void b3SetAssertFcn( b3AssertFcn* assertFcn )
{
	B3_ASSERT( assertFcn != nullptr );
	b3AssertHandler = assertFcn;
}

#if !defined( NDEBUG ) || defined( B3_ENABLE_ASSERT )
int b3InternalAssert( const char* condition, const char* fileName, int lineNumber )
{
	return b3AssertHandler( condition, fileName, lineNumber );
}
#endif

static void b3DefaultLogFcn( const char* message )
{
	printf( "Box3D: %s\n", message );
}

b3LogFcn* b3LogHandler = b3DefaultLogFcn;

void b3SetLogFcn( b3LogFcn* logFcn )
{
	B3_ASSERT( logFcn != nullptr );
	b3LogHandler = logFcn;
}

void b3Log( const char* format, ... )
{
	va_list args;
	va_start( args, format );
	char buffer[512];
	vsnprintf( buffer, sizeof( buffer ), format, args );
	b3LogHandler( buffer );
	va_end( args );
}

b3Version b3GetVersion( void )
{
	return b3Version{ 0, 2, 0 };
}

static b3AllocFcn* b3_allocFcn = nullptr;
static b3FreeFcn* b3_freeFcn = nullptr;

b3AtomicInt b3_byteCount;

void b3SetAllocator( b3AllocFcn* allocFcn, b3FreeFcn* freeFcn )
{
	b3_allocFcn = allocFcn;
	b3_freeFcn = freeFcn;
}

// Use 64 byte alignment for everything to align to 64 byte cache line and works with 256bit SIMD.
#define B3_ALIGNMENT 64

void* b3Alloc( size_t size )
{
	if ( size == 0 )
	{
		return nullptr;
	}

	// This could cause some sharing issues, however Box3D rarely calls b3Alloc.
	// todo this is not true, Box3D allocates a lot.
	b3AtomicFetchAddInt( &b3_byteCount, (int)size );

	// Allocation must be a multiple of 32 or risk a seg fault
	// https://en.cppreference.com/w/c/memory/aligned_alloc
	int size64 = ( ( (int)size - 1 ) | 0x3F ) + 1;

	if ( b3_allocFcn != nullptr )
	{
		void* ptr = b3_allocFcn( size64, B3_ALIGNMENT );
		b3TracyCAlloc( ptr, size );

		B3_ASSERT( ptr != nullptr );
		B3_ASSERT( ( (uintptr_t)ptr & 0x3F ) == 0 );

		return ptr;
	}

#ifdef B3_PLATFORM_WINDOWS
	void* ptr = _aligned_malloc( size64, B3_ALIGNMENT );
#elif defined( B3_PLATFORM_ANDROID )
	void* ptr = nullptr;
	if ( posix_memalign( &ptr, B3_ALIGNMENT, size64 ) != 0 )
	{
		// allocation failed, exit the application
		exit( EXIT_FAILURE );
	}
#else
	void* ptr = aligned_alloc( B3_ALIGNMENT, size64 );
#endif

	b3TracyCAlloc( ptr, size );

	B3_ASSERT( ptr != nullptr );
	B3_ASSERT( ( (uintptr_t)ptr & 0x3F ) == 0 );

	return ptr;
}

void b3Free( void* mem, size_t size )
{
	if ( mem == nullptr )
	{
		return;
	}

	b3TracyCFree( mem );

	if ( b3_freeFcn != nullptr )
	{
		b3_freeFcn( mem );
	}
	else
	{
#ifdef B3_PLATFORM_WINDOWS
		_aligned_free( mem );
#else
		free( mem );
#endif
	}

	b3AtomicFetchAddInt( &b3_byteCount, -(int)size );
}

void* b3GrowAlloc( void* oldMem, int oldSize, int newSize )
{
	// todo try _aligned_realloc

	B3_ASSERT( newSize > oldSize );
	void* newMem = b3Alloc( newSize );
	if ( oldSize > 0 )
	{
		memcpy( newMem, oldMem, oldSize );
		b3Free( oldMem, oldSize );
	}
	return newMem;
}

int b3GetByteCount( void )
{
	return b3AtomicLoadInt( &b3_byteCount );
}

void* b3AllocZeroed(size_t size)
{
	void* mem = b3Alloc( size );
	memset( mem, 0, size );
	return mem;
}
