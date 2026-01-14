// SPDX-FileCopyrightText: 2025 Dirk Gregorius
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

struct b3HeapNode
{
	double key;
	int index;
};

struct b3Heap
{
	int Size() const
	{
		return mHeap.count;
	}

	bool Empty() const
	{
		return mHeap.count == 0;
	}

	void Clear()
	{
		mHeap.Clear();
	}

	void Push( b3HeapNode* node);
	void Update( b3HeapNode* node);
	void Remove( b3HeapNode* node);
	b3HeapNode* Pop();

	b3HeapNode* Top();
	const b3HeapNode* Top() const;

	bool IsConsistent() const;

	void BubbleUp( int index);
	void BubbleDown( int index);

	b3Array<b3HeapNode*> mHeap;
};
