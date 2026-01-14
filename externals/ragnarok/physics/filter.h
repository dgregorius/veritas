//--------------------------------------------------------------------------------------------------
/*
	@file		filter.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/types.h"


//--------------------------------------------------------------------------------------------------
// RkFilter
//--------------------------------------------------------------------------------------------------
struct RkFilter
	{
	uint32 CategoryBits;
	uint32 MaskBits;
	};

bool operator==( const RkFilter& Filter1, const RkFilter& Filter2 );
bool operator!=( const RkFilter& Filter1, const RkFilter& Filter2 );

bool rkShouldCollide( const RkFilter& Filter1, const RkFilter& Filter2 );

// Constants
RK_GLOBAL_CONSTANT const RkFilter RK_DEFAULT_FILTER = { 0x00000001, 0xffffffff };


