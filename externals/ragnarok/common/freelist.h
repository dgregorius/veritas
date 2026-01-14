//--------------------------------------------------------------------------------------------------
/**
	@file		freelist.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/assert.h"
#include "ragnarok/common/types.h"


//--------------------------------------------------------------------------------------------------
// RkFreelist 
//--------------------------------------------------------------------------------------------------
class RkFreelist
	{
	public:
		bool Empty() const;
		void* Pop();
		void Push( void* Block );
		void Reset();
			
	private:
		RkFreelist* mNext = nullptr;
	};


#include "freelist.inl"