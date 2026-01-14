// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include <stdint.h>

// Compare to SDL_AtomicInt

#if defined( _MSC_VER )
	#include <intrin.h>
#endif

static inline void b3AtomicStoreInt( b3AtomicInt* a, int value )
{
#if defined( _MSC_VER )
	(void)_InterlockedExchange( (long*)&a->value, value );
#elif defined( __GNUC__ ) || defined( __clang__ )
	__atomic_store_n( &a->value, value, __ATOMIC_SEQ_CST );
#else
	#error "Unsupported platform"
#endif
}

static inline int b3AtomicLoadInt( b3AtomicInt* a )
{
#if defined( _MSC_VER )
	return _InterlockedOr( (long*)&a->value, 0 );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
	#error "Unsupported platform"
#endif
}

static inline int b3AtomicFetchAddInt( b3AtomicInt* a, int increment )
{
#if defined( _MSC_VER )
	return _InterlockedExchangeAdd( (long*)&a->value, (long)increment );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_fetch_add( &a->value, increment, __ATOMIC_SEQ_CST );
#else
	#error "Unsupported platform"
#endif
}

static inline bool b3AtomicCompareExchangeInt( b3AtomicInt* a, int expected, int desired )
{
#if defined( _MSC_VER )
	return _InterlockedCompareExchange( (long*)&a->value, (long)desired, (long)expected ) == expected;
#elif defined( __GNUC__ ) || defined( __clang__ )
	// The value written to expected is ignored
	return __atomic_compare_exchange_n( &a->value, &expected, desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST );
#else
	#error "Unsupported platform"
#endif
}

static inline void b3AtomicStoreU32( b3AtomicU32* a, uint32_t value )
{
#if defined( _MSC_VER )
	(void)_InterlockedExchange( (long*)&a->value, value );
#elif defined( __GNUC__ ) || defined( __clang__ )
	__atomic_store_n( &a->value, value, __ATOMIC_SEQ_CST );
#else
	#error "Unsupported platform"
#endif
}

static inline uint32_t b3AtomicLoadU32( b3AtomicU32* a )
{
#if defined( _MSC_VER )
	return (uint32_t)_InterlockedOr( (long*)&a->value, 0 );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
	#error "Unsupported platform"
#endif
}
