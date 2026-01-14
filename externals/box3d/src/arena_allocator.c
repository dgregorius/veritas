// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "arena_allocator.h"

#include "core.h"

#include <stdbool.h>
#include <stddef.h>

b3ArenaAllocator b3CreateArenaAllocator( int capacity )
{
	B3_ASSERT( capacity >= 0 );
	b3ArenaAllocator allocator = { 0 };
	allocator.capacity = capacity;
	allocator.data = (char*)b3Alloc( capacity );
	return allocator;
}

void b3DestroyArenaAllocator( b3ArenaAllocator* allocator )
{
	b3Free( allocator->data, allocator->capacity );
}

void* b3AllocateArenaItem( b3ArenaAllocator* alloc, int size, const char* name )
{
	if ( alloc->entryCount == B3_MAX_ARENA_ENTRIES )
	{
		B3_ASSERT( false );
		return NULL;
	}

	// ensure allocation is 32 byte aligned to support 256-bit SIMD
	int size32 = ( ( size - 1 ) | 0x1F ) + 1;

	b3ArenaEntry entry;
	entry.size = size32;
	entry.name = name;
	if ( alloc->index + size32 > alloc->capacity )
	{
		// fall back to the heap (undesirable)
		entry.data = (char*)b3Alloc( size32 );
		entry.usedMalloc = true;

		B3_ASSERT( ( (uintptr_t)entry.data & 0x1F ) == 0 );
	}
	else
	{
		entry.data = alloc->data + alloc->index;
		entry.usedMalloc = false;
		alloc->index += size32;

		B3_ASSERT( ( (uintptr_t)entry.data & 0x1F ) == 0 );
	}

	alloc->allocation += size32;
	if ( alloc->allocation > alloc->maxAllocation )
	{
		alloc->maxAllocation = alloc->allocation;
	}

	alloc->entries[alloc->entryCount] = entry;
	alloc->entryCount += 1;
	return entry.data;
}

void b3FreeArenaItem( b3ArenaAllocator* alloc, void* mem )
{
	int entryCount = alloc->entryCount;
	B3_ASSERT( entryCount > 0 );
	b3ArenaEntry* entry = alloc->entries + ( entryCount - 1 );
	B3_ASSERT( mem == entry->data );
	if ( entry->usedMalloc )
	{
		b3Free( mem, entry->size );
	}
	else
	{
		alloc->index -= entry->size;
	}
	alloc->allocation -= entry->size;
	alloc->entryCount -= 1;
}

void b3GrowArena( b3ArenaAllocator* alloc )
{
	// Stack must not be in use
	B3_ASSERT( alloc->allocation == 0 );

	if ( alloc->maxAllocation > alloc->capacity )
	{
		b3Free( alloc->data, alloc->capacity );
		alloc->capacity = alloc->maxAllocation + alloc->maxAllocation / 2;
		alloc->data = (char*)b3Alloc( alloc->capacity );
	}
}

int b3GetArenaCapacity( b3ArenaAllocator* alloc )
{
	return alloc->capacity;
}

int b3GetArenaAllocation( b3ArenaAllocator* alloc )
{
	return alloc->allocation;
}

int b3GetMaxArenaAllocation( b3ArenaAllocator* alloc )
{
	return alloc->maxAllocation;
}
