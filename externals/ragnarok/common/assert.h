//--------------------------------------------------------------------------------------------------
/*
	@file		assert.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once


//--------------------------------------------------------------------------------------------------
// Assert macros
//--------------------------------------------------------------------------------------------------
#if defined( DEBUG ) || defined( _DEBUG )
	#define RK_BREAK							__debugbreak()
	#define RK_ASSERT( Cond )					do { if ( !( Cond ) ) RK_BREAK; } while( 0 )
#else
	#define RK_BREAK
	#define RK_ASSERT( Cond )					do { (void)sizeof( Cond ); } while( 0 )
#endif

