//--------------------------------------------------------------------------------------------------
/*
	@file		pool.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/assert.h"
#include "ragnarok/common/memory.h"


//--------------------------------------------------------------------------------------------------
// RkPoolBlock
//--------------------------------------------------------------------------------------------------
struct RkPoolBlock
	{
	RkPoolBlock* Next;
	};


//--------------------------------------------------------------------------------------------------
// RkPoolChunk
//--------------------------------------------------------------------------------------------------
struct RkPoolChunk
	{
	int BlockCount;
	RkPoolBlock* Blocks;
	};


//--------------------------------------------------------------------------------------------------
// RkPool
//--------------------------------------------------------------------------------------------------
class RkPool
	{
	public:
		// Construction / Destruction
		explicit RkPool( size_t BlockSize, size_t BlockAlignment = alignof( std::max_align_t ) );
		~RkPool();

		// Memory management
		void* Allocate();
		void Deallocate( void* Pointer );

	private:
		enum { MaxChunkCount = 20 };

		size_t mBlockSize = 0;
		size_t mBlockAlignment = 0;
		
		int mChunkCount = 0;
		RkPoolChunk mChunks[ MaxChunkCount ] = {};
		RkPoolBlock* mFreelist = nullptr;
		RkPoolBlock* mNextBlock = nullptr;
		size_t mRemainingBlocks = 0;
	};