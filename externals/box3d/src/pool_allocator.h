// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

// Pool allocator with block size doubling
// https://danielchasehooper.com/posts/segment_array/

// The first block can hold this many items. Each subsequent block has double capacity.
#define B3_BASE_BLOCK_ITEM_COUNT 32

// With 20 blocks the final block can hold 32 * 19^2 = 16,777,216 items
#define B3_MAX_POOL_BLOCKS 20

typedef struct b3PoolBlock
{
	void* data;
} b3PoolBlock;

typedef struct b3PoolAllocator
{
	struct b3PoolBlock blocks[B3_MAX_POOL_BLOCKS];
	int blockCount;

	struct b3PoolItem* freeList;
	int itemSize;

	// This is a bump allocator on the current block and avoids bulk free list population
	void* next;
	int remaining;

	int activeItems;
} b3PoolAllocator;

#ifdef __cplusplus
extern "C"
{
#endif

b3PoolAllocator b3CreatePool(int itemSize);
void b3DestroyPool(b3PoolAllocator* pool);
void* b3AllocatePoolItem( b3PoolAllocator* pool );
void b3FreePoolItem( b3PoolAllocator* pool, void* item );
int b3GetPoolByteCount( b3PoolAllocator* pool );

#ifdef __cplusplus
}
#endif
