//--------------------------------------------------------------------------------------------------
// filter.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "filter.h"


//--------------------------------------------------------------------------------------------------
// RkFilter
//--------------------------------------------------------------------------------------------------
bool operator==( const RkFilter& Filter1, const RkFilter& Filter2 )
	{
	return Filter1.CategoryBits == Filter2.CategoryBits && Filter1.MaskBits == Filter2.MaskBits;
	}


//--------------------------------------------------------------------------------------------------
bool operator!=( const RkFilter& Filter1, const RkFilter& Filter2 )
	{
	return Filter1.CategoryBits != Filter2.CategoryBits || Filter1.MaskBits != Filter2.MaskBits;
	}


//--------------------------------------------------------------------------------------------------
bool rkShouldCollide( const RkFilter& Filter1, const RkFilter& Filter2 )
	{
	return ( Filter1.MaskBits & Filter2.CategoryBits ) != 0 && ( Filter1.CategoryBits & Filter2.MaskBits ) != 0;
	}