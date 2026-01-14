// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once
#include <stdbool.h>

#define B3_MAX_ARENA_ENTRIES 32

typedef struct b3ArenaEntry
{
	char* data;
	const char* name;
	int size;
	bool usedMalloc;
} b3ArenaEntry;

// This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will B3_ASSERT
// if you try to interleave multiple allocate/free pairs.
// This allocator uses the heap if space is insufficient.
// I could remove the need to free entries individually.
typedef struct b3ArenaAllocator
{
	char* data;
	int capacity;
	int index;

	int allocation;
	int maxAllocation;

	b3ArenaEntry entries[B3_MAX_ARENA_ENTRIES];
	int entryCount;
} b3ArenaAllocator;

#ifdef __cplusplus
extern "C"
{
#endif

b3ArenaAllocator b3CreateArenaAllocator( int capacity );
void b3DestroyArenaAllocator( b3ArenaAllocator* allocator );

void* b3AllocateArenaItem( b3ArenaAllocator* alloc, int size, const char* name );
void b3FreeArenaItem( b3ArenaAllocator* alloc, void* mem );

// Grow the arena based on usage
void b3GrowArena( b3ArenaAllocator* alloc );

int b3GetArenaCapacity( b3ArenaAllocator* alloc );
int b3GetArenaAllocation( b3ArenaAllocator* alloc );
int b3GetMaxArenaAllocation( b3ArenaAllocator* alloc );

#ifdef __cplusplus
}
#endif
