// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "box3d/base.h"

#include <stddef.h>

#if defined( _WIN32 )

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

static double s_invFrequency = 0.0;

uint64_t b3GetTicks( void )
{
	LARGE_INTEGER counter;
	QueryPerformanceCounter( &counter );
	return (uint64_t)counter.QuadPart;
}

float b3GetMilliseconds( uint64_t ticks )
{
	if (s_invFrequency == 0.0)
	{
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency( &frequency );

		s_invFrequency = (double)frequency.QuadPart;
		if (s_invFrequency > 0.0)
		{
			s_invFrequency = 1000.0 / s_invFrequency;
		}
	}

	uint64_t ticksNow = b3GetTicks();
	return (float)( s_invFrequency * ( ticksNow - ticks ) );
}

float b3GetMillisecondsAndReset( uint64_t* ticks )
{
	if (s_invFrequency == 0.0)
	{
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency( &frequency );

		s_invFrequency = (double)frequency.QuadPart;
		if (s_invFrequency > 0.0)
		{
			s_invFrequency = 1000.0 / s_invFrequency;
		}
	}

	uint64_t ticksNow = b3GetTicks();
	float ms = (float)( s_invFrequency * ( ticksNow - *ticks ) );
	*ticks = ticksNow;
	return ms;
}

void b3Yield( void )
{
	SwitchToThread();
}

typedef struct b3Mutex
{
	CRITICAL_SECTION cs;
} b3Mutex;

b3Mutex* b3CreateMutex( void )
{
	b3Mutex* m = b3Alloc( sizeof( b3Mutex ) );
	InitializeCriticalSection( &m->cs );
	return m;
}

void b3DestroyMutex( b3Mutex* m )
{
	DeleteCriticalSection( &m->cs );
	*m = (b3Mutex){ 0 };
	b3Free( m, sizeof( b3Mutex ) );
}

void b3LockMutex( b3Mutex* m )
{
	EnterCriticalSection( &m->cs );
}

void b3UnlockMutex( b3Mutex* m )
{
	LeaveCriticalSection( &m->cs );
}

#elif defined( __linux__ ) || defined( __EMSCRIPTEN__ )

#include <sched.h>
#include <time.h>

uint64_t b3GetTicks( void )
{
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

float b3GetMilliseconds( uint64_t ticks )
{
	uint64_t ticksNow = b3GetTicks();
	return (float)( ( ticksNow - ticks ) / 1000000.0 );
}

float b3GetMillisecondsAndReset( uint64_t* ticks )
{
	uint64_t ticksNow = b3GetTicks();
	float ms = (float)( ( ticksNow - *ticks ) / 1000000.0 );
	*ticks = ticksNow;
	return ms;
}

void b3Yield( void )
{
	sched_yield();
}

#include <pthread.h>
typedef struct b3Mutex
{
	pthread_mutex_t mtx;
} b3Mutex;

b3Mutex* b3CreateMutex( void )
{
	b3Mutex* m = b3Alloc( sizeof( b3Mutex ) );
	pthread_mutex_init( &m->mtx, NULL );
	return m;
}

void b3DestroyMutex( b3Mutex* m )
{
	pthread_mutex_destroy( &m->mtx );
	*m = (b3Mutex){ 0 };
	b3Free( m, sizeof( b3Mutex ) );
}

void b3LockMutex( b3Mutex* m )
{
	pthread_mutex_lock( &m->mtx );
}

void b3UnlockMutex( b3Mutex* m )
{
	pthread_mutex_unlock( &m->mtx );
}

#elif defined( __APPLE__ )

#include <mach/mach_time.h>
#include <sched.h>
#include <sys/time.h>

static double s_invFrequency = 0.0;

uint64_t b3GetTicks( void )
{
	return mach_absolute_time();
}

float b3GetMilliseconds( uint64_t ticks )
{
	if (s_invFrequency == 0)
	{
		mach_timebase_info_data_t timebase;
		mach_timebase_info( &timebase );

		// convert to ns then to ms
		s_invFrequency = 1e-6 * (double)timebase.numer / (double)timebase.denom;
	}

	uint64_t ticksNow = b3GetTicks();
	return (float)( s_invFrequency * ( ticksNow - ticks ) );
}

float b3GetMillisecondsAndReset( uint64_t* ticks )
{
	if (s_invFrequency == 0)
	{
		mach_timebase_info_data_t timebase;
		mach_timebase_info( &timebase );

		// convert to ns then to ms
		s_invFrequency = 1e-6 * (double)timebase.numer / (double)timebase.denom;
	}

	uint64_t ticksNow = b3GetTicks();
	float ms = (float)( s_invFrequency * ( ticksNow - *ticks ) );
	*ticks = ticksNow;
	return ms;
}

void b3Yield( void )
{
	sched_yield();
}

#include <pthread.h>
typedef struct b3Mutex
{
	pthread_mutex_t mtx;
} b3Mutex;

b3Mutex* b3CreateMutex( void )
{
	b3Mutex* m = b3Alloc( sizeof( b3Mutex ) );
	pthread_mutex_init( &m->mtx, NULL );
	return m;
}

void b3DestroyMutex( b3Mutex* m )
{
	pthread_mutex_destroy( &m->mtx );
	*m = (b3Mutex){ 0 };
	b3Free( m, sizeof( b3Mutex ) );
}

void b3LockMutex( b3Mutex* m )
{
	pthread_mutex_lock( &m->mtx );
}

void b3UnlockMutex( b3Mutex* m )
{
	pthread_mutex_unlock( &m->mtx );
}

#else

uint64_t b3GetTicks( void )
{
	return 0;
}

float b3GetMilliseconds( uint64_t ticks )
{
	( (void)( ticks ) );
	return 0.0f;
}

float b3GetMillisecondsAndReset( uint64_t* ticks )
{
	( (void)( ticks ) );
	return 0.0f;
}

void b3Yield( void )
{
}

typedef struct b3Mutex
{
	int dummy;
} b3Mutex;

b3Mutex* b3CreateMutex( void )
{
	b3Mutex* m = b3Alloc( sizeof( b3Mutex ) );
	m->dummy = 42;
	return m;
}

void b3DestroyMutex( b3Mutex* m )
{
	*m = (b3Mutex){ 0 };
	b3Free( m, sizeof( b3Mutex ) );
}

void b3LockMutex( b3Mutex* m )
{
	(void)m;
}

void b3UnlockMutex( b3Mutex* m )
{
	(void)m;
}

#endif

// djb2 hash
// https://en.wikipedia.org/wiki/List_of_hash_functions
uint32_t b3Hash( uint32_t hash, const uint8_t* data, int count )
{
	uint32_t result = hash;
	for (int i = 0; i < count; i++)
	{
		result = ( result << 5 ) + result + data[i];
	}

	return result;
}
