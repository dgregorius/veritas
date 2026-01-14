// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

constexpr int b3_blockSizeCount = 14;

struct b3Block;
struct b3Chunk;

// todo this is not used yet
// This is a small object allocator used for allocating small
// objects that persist for more than one time step.
// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
struct b3BlockAllocator
{
	void Create();
	void Destroy();

	/// Allocate memory. This will use b3Alloc if the size is larger than b3_maxBlockSize.
	void* Allocate(int size);

	/// Free memory. This will use b3Free if the size is larger than b3_maxBlockSize.
	void Free(void* p, int size);

	void Clear();

	b3Chunk* m_chunks;
	int m_chunkCount;
	int m_chunkSpace;

	b3Block* m_freeLists[b3_blockSizeCount];
};
