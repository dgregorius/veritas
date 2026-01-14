// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "pool_allocator.h"

#include "core.h"

#include "box3d/base.h"

#include <stdbool.h>

typedef struct b3PoolItem
{
	struct b3PoolItem* next;
} b3PoolItem;

b3PoolAllocator b3CreatePool( int itemSize )
{
	B3_ASSERT( itemSize >= (int)sizeof( b3PoolItem ) );
	b3PoolAllocator pool = { 0 };
	pool.itemSize = itemSize;
	return pool;
}

void b3DestroyPool( b3PoolAllocator* pool )
{
	int itemCount = B3_BASE_BLOCK_ITEM_COUNT;
	for (int i = 0; i < pool->blockCount; ++i)
	{
		b3PoolBlock* block = pool->blocks + i;
		int size = itemCount * pool->itemSize;
		b3Free( block->data, size );
		itemCount *= 2;
	}
}

void* b3AllocatePoolItem( b3PoolAllocator* pool )
{
	// Use free list
	if (pool->freeList != NULL)
	{
		void* item = pool->freeList;
		pool->freeList = pool->freeList->next;
		pool->activeItems += 1;
		return item;
	}

	// Consume last allocated block using bump allocation
	if (pool->remaining > 0)
	{
		void* item = pool->next;
		pool->next = (char*)pool->next + pool->itemSize;
		pool->remaining -= 1;
		pool->activeItems += 1;
		return item;
	}

	int blockCount = pool->blockCount;
	if (blockCount == B3_MAX_POOL_BLOCKS)
	{
		B3_ASSERT( false );
		return NULL;
	}

	// Allocate new block
	b3PoolBlock* block = pool->blocks + blockCount;
	int itemCount = B3_BASE_BLOCK_ITEM_COUNT << blockCount;
	block->data = b3Alloc( itemCount * pool->itemSize );
	pool->blockCount += 1;

	// Setup the bump allocator
	pool->next = block->data;
	pool->remaining = itemCount;

	// Bump allocate
	void* item = pool->next;
	pool->next = (char*)pool->next + pool->itemSize;
	pool->remaining -= 1;
	pool->activeItems += 1;
	return item;
}

void b3FreePoolItem( b3PoolAllocator* pool, void* item )
{
	if (item == NULL)
	{
		return;
	}

	B3_ASSERT( pool->activeItems > 0 );

	b3PoolItem* poolItem = item;
	poolItem->next = pool->freeList;
	pool->freeList = poolItem;
	pool->activeItems -= 1;
}

int b3GetPoolByteCount( b3PoolAllocator* pool )
{
	int itemCount = B3_BASE_BLOCK_ITEM_COUNT;
	int byteCount = 0;
	for (int i = 0; i < pool->blockCount; ++i)
	{
		byteCount += itemCount * pool->itemSize;
		itemCount *= 2;
	}
	return byteCount;
}
