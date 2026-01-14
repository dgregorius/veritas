//--------------------------------------------------------------------------------------------------
// pool.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "pool.h"


//--------------------------------------------------------------------------------------------------
// RkPool
//--------------------------------------------------------------------------------------------------
RkPool::RkPool( size_t BlockSize, size_t BlockAlignment )
	{
	RK_ASSERT( BlockSize >= sizeof( RkPoolBlock ) );
	RK_ASSERT( BlockAlignment >= alignof( RkPoolBlock* ) );

	mBlockSize = rkAlign( BlockSize, BlockAlignment );
	mBlockAlignment = static_cast< int >( BlockAlignment );
	}


//--------------------------------------------------------------------------------------------------
RkPool::~RkPool()
	{
	for ( int ChunkIndex = mChunkCount - 1; ChunkIndex >= 0; --ChunkIndex )
		{
		RkPoolChunk& Chunk = mChunks[ ChunkIndex ];
		rkAlignedFree( Chunk.Blocks );
		}
	}


//--------------------------------------------------------------------------------------------------
void* RkPool::Allocate()
	{
	// Try free list first
	if ( mFreelist )
		{
		RkPoolBlock* Block = mFreelist;
		mFreelist = Block->Next;
		
		return Block;
		}

	// Try to allocate from last chunk
	if ( !mRemainingBlocks )
		{
		if ( mChunkCount == MaxChunkCount )
			{
			return nullptr;
			}

		// Geometric growth
		const int MinBlockCount = 32;
		int BlockCount = MinBlockCount << mChunkCount;
		RkPoolBlock* Blocks = static_cast< RkPoolBlock* >( rkAlignedAlloc( BlockCount * mBlockSize, mBlockAlignment ) );
		RK_ASSERT( Blocks );

		mChunks[ mChunkCount ].BlockCount = BlockCount;
		mChunks[ mChunkCount ].Blocks = Blocks;
		mChunkCount++;

		mNextBlock = Blocks;
		mRemainingBlocks = BlockCount;
		}

	RK_ASSERT( mNextBlock );
	RkPoolBlock* Block = mNextBlock;
	mNextBlock = reinterpret_cast< RkPoolBlock* >( rkAddByteOffset( mNextBlock, mBlockSize ) );
	mRemainingBlocks--;

	return Block;
	}


//--------------------------------------------------------------------------------------------------
void RkPool::Deallocate( void* Pointer )
	{
	if ( !Pointer )
		{
		return;
		}

	RkPoolBlock* Block = static_cast< RkPoolBlock* >( Pointer );
	Block->Next = mFreelist;
	mFreelist = Block;
	}