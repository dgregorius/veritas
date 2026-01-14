// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "block_allocator.h"

#include "core.h"

#include "box3d/base.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

static constexpr int b3_chunkSize = 16 * 1024;
static constexpr int b3_maxBlockSize = 640;
static constexpr int b3_chunkArrayIncrement = 128;

// These are the supported object sizes. Actual allocations are rounded up the next size.
static const int b3_blockSizes[b3_blockSizeCount] = {
	16,	 // 0
	32,	 // 1
	64,	 // 2
	96,	 // 3
	128, // 4
	160, // 5
	192, // 6
	224, // 7
	256, // 8
	320, // 9
	384, // 10
	448, // 11
	512, // 12
	640, // 13
};

// This maps an arbitrary allocation size to a suitable slot in b3_blockSizes.
struct b3SizeMap
{
	b3SizeMap()
	{
		int j = 0;
		values[0] = 0;
		for ( int i = 1; i <= b3_maxBlockSize; ++i )
		{
			B3_ASSERT( j < b3_blockSizeCount );
			if ( i <= b3_blockSizes[j] )
			{
				values[i] = (uint8_t)j;
			}
			else
			{
				++j;
				values[i] = (uint8_t)j;
			}
		}
	}

	uint8_t values[b3_maxBlockSize + 1];
};

static const b3SizeMap b3_sizeMap;

struct b3Chunk
{
	int blockSize;
	b3Block* blocks;
};

struct b3Block
{
	b3Block* next;
};

void b3BlockAllocator::Create()
{
	static_assert( b3_blockSizeCount < UINT8_MAX, "too many blocks" );

	m_chunkSpace = b3_chunkArrayIncrement;
	m_chunkCount = 0;
	m_chunks = (b3Chunk*)b3Alloc( m_chunkSpace * sizeof( b3Chunk ) );

	memset( m_chunks, 0, m_chunkSpace * sizeof( b3Chunk ) );
	memset( m_freeLists, 0, sizeof( m_freeLists ) );
}

void b3BlockAllocator::Destroy()
{
	for ( int i = 0; i < m_chunkCount; ++i )
	{
		b3Free( m_chunks[i].blocks, b3_chunkSize );
	}

	b3Free( m_chunks, m_chunkSpace * sizeof( b3Chunk ) );
}

void* b3BlockAllocator::Allocate( int size )
{
	if ( size == 0 )
	{
		return nullptr;
	}

	B3_ASSERT( 0 < size );

	if ( size > b3_maxBlockSize )
	{
		return b3Alloc( size );
	}

	int index = b3_sizeMap.values[size];
	B3_ASSERT( 0 <= index && index < b3_blockSizeCount );

	if ( m_freeLists[index] )
	{
		b3Block* block = m_freeLists[index];
		m_freeLists[index] = block->next;
		return block;
	}
	else
	{
		if ( m_chunkCount == m_chunkSpace )
		{
			b3Chunk* oldChunks = m_chunks;
			int oldChunkSpace = m_chunkSpace;
			m_chunkSpace += b3_chunkArrayIncrement;
			m_chunks = (b3Chunk*)b3Alloc( m_chunkSpace * sizeof( b3Chunk ) );
			memcpy( m_chunks, oldChunks, m_chunkCount * sizeof( b3Chunk ) );
			memset( m_chunks + m_chunkCount, 0, b3_chunkArrayIncrement * sizeof( b3Chunk ) );
			b3Free(oldChunks, oldChunkSpace * sizeof( b3Chunk ) );
		}

		b3Chunk* chunk = m_chunks + m_chunkCount;
		chunk->blocks = (b3Block*)b3Alloc( b3_chunkSize );
#if defined( _DEBUG )
		memset( chunk->blocks, 0xcd, b3_chunkSize );
#endif
		int blockSize = b3_blockSizes[index];
		chunk->blockSize = blockSize;
		int blockCount = b3_chunkSize / blockSize;
		B3_ASSERT( blockCount * blockSize <= b3_chunkSize );
		for ( int i = 0; i < blockCount - 1; ++i )
		{
			b3Block* block = (b3Block*)( (int8_t*)chunk->blocks + blockSize * i );
			b3Block* next = (b3Block*)( (int8_t*)chunk->blocks + blockSize * ( i + 1 ) );
			block->next = next;
		}
		b3Block* last = (b3Block*)( (int8_t*)chunk->blocks + blockSize * ( blockCount - 1 ) );
		last->next = nullptr;

		m_freeLists[index] = chunk->blocks->next;
		++m_chunkCount;

		return chunk->blocks;
	}
}

void b3BlockAllocator::Free( void* p, int size )
{
	if ( size == 0 )
	{
		return;
	}

	B3_ASSERT( 0 < size );

	if ( size > b3_maxBlockSize )
	{
		b3Free( p, size );
		return;
	}

	int index = b3_sizeMap.values[size];
	B3_ASSERT( 0 <= index && index < b3_blockSizeCount );

#if !defined( NDEBUG )
	// Verify the memory address and size is valid.
	int blockSize = b3_blockSizes[index];
	bool found = false;
	for ( int i = 0; i < m_chunkCount; ++i )
	{
		b3Chunk* chunk = m_chunks + i;
		if ( chunk->blockSize != blockSize )
		{
			B3_ASSERT( (int8_t*)p + blockSize <= (int8_t*)chunk->blocks || (int8_t*)chunk->blocks + b3_chunkSize <= (int8_t*)p );
		}
		else
		{
			if ( (int8_t*)chunk->blocks <= (int8_t*)p && (int8_t*)p + blockSize <= (int8_t*)chunk->blocks + b3_chunkSize )
			{
				found = true;
			}
		}
	}

	B3_ASSERT( found );

	memset( p, 0xfd, blockSize );
#endif

	b3Block* block = (b3Block*)p;
	block->next = m_freeLists[index];
	m_freeLists[index] = block;
}

void b3BlockAllocator::Clear()
{
	for ( int i = 0; i < m_chunkCount; ++i )
	{
		b3Free( m_chunks[i].blocks, b3_chunkSize );
	}

	m_chunkCount = 0;
	memset( m_chunks, 0, m_chunkSpace * sizeof( b3Chunk ) );
	memset( m_freeLists, 0, sizeof( m_freeLists ) );
}
