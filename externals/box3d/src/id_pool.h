// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

struct b3IdPool
{
	b3Array<int> freeArray;
	int nextIndex;
};

b3IdPool b3CreateIdPool();
void b3DestroyIdPool( b3IdPool* pool );

int b3AllocId( b3IdPool* pool );
void b3FreeId( b3IdPool* pool, int id );
void b3ValidateFreeId(const b3IdPool* pool, int id );

inline int b3GetIdCount( const b3IdPool* pool )
{
	return pool->nextIndex - pool->freeArray.count;
}

inline int b3GetIdCapacity( const b3IdPool* pool )
{
	return pool->nextIndex;
}

inline int b3GetIdBytes( const b3IdPool* pool )
{
	return pool->freeArray.GetByteCount();
}
