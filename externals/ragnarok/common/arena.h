//--------------------------------------------------------------------------------------------------
/**
	@file		arena.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/math.h"
#include "ragnarok/common/memory.h"


//--------------------------------------------------------------------------------------------------
// RkArena
//--------------------------------------------------------------------------------------------------
class RkArena
	{
	public:
		// Construction / Destruction
		explicit  RkArena( size_t Size );
		~RkArena();

		// Memory management
		void* Allocate( size_t Size, size_t Alignment = alignof( std::max_align_t ) );
		void Release();

	private:
		size_t mReservedSize = 0; 
		uint8* mReservedBase = nullptr;
		size_t mCommittedSize = 0;
		size_t mRemainingSize = 0;
	};