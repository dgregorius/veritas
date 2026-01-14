//--------------------------------------------------------------------------------------------------
/*
	@file		qhTypes.h

	@author		Dirk Gregorius
	@version	0.1
	@date		30/11/2011

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <limits>


//--------------------------------------------------------------------------------------------------
// qhTypes
//--------------------------------------------------------------------------------------------------
typedef float					qhReal;
typedef float					qhReal32;
typedef double					qhReal64;

typedef int8_t					qhInt8;
typedef int16_t					qhInt16;
typedef int32_t					qhInt32;
typedef int64_t					qhInt64;

typedef uint8_t					qhUInt8;
typedef uint16_t				qhUInt16;
typedef uint32_t				qhUInt32;
typedef uint64_t				qhUInt64;

typedef intptr_t				qhIntPtr;
typedef uintptr_t				qhUIntPtr;

#define QH_INT_MIN				std::numeric_limits< int >::min()
#define QH_INT_MAX				std::numeric_limits< int >::max()
#define QH_REAL_MIN				std::numeric_limits< qhReal >::min()
#define QH_REAL_MAX				std::numeric_limits< qhReal >::max()
#define QH_REAL_EPSILON			std::numeric_limits< qhReal >::epsilon()

#define QH_GLOBAL_CONSTANT		extern __declspec( selectany )


//--------------------------------------------------------------------------------------------------
// qhAssert
//--------------------------------------------------------------------------------------------------
#if defined( DEBUG ) || defined( _DEBUG )
#  define QH_BREAK()			__debugbreak()
#  define QH_ASSERT( Cond )		do { if ( ( !( Cond ) ) ) QH_BREAK(); } while( 0 )
#else
#  define QH_BREAK()
#  define QH_ASSERT( Cond )		do { (void)sizeof( Cond ); } while( 0 )           
#endif


