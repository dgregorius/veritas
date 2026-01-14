// SPDX-FileCopyrightText: 2025 Dirk Gregorius
// SPDX-License-Identifier: MIT

#include "heap.h"

#include <float.h>

static double b3MinDouble( double a, double b )
{
	return a <= b ? a : b;
}

void b3Heap::Push( b3HeapNode* node)
{
	B3_ASSERT(node && node->index < 0 );

	mHeap.PushBack(node);
	node->index = mHeap.count - 1;

	BubbleUp(node->index);

	B3_ASSERT( IsConsistent() );
}

//--------------------------------------------------------------------------------------------------
void b3Heap::Update( b3HeapNode* node)
{
	B3_ASSERT(node && mHeap[node->index] == node);

	int index = node->index;
	int parent = (index - 1 ) / 2;

	if (index > 0 && mHeap[parent]->key > mHeap[index]->key )
	{
		BubbleUp(index);
	}
	else
	{
		BubbleDown(index);
	}

	B3_ASSERT( IsConsistent() );
}

//--------------------------------------------------------------------------------------------------
void b3Heap::Remove( b3HeapNode* node)
{
	B3_ASSERT(node && mHeap[node->index] == node);

	int index = node->index;
	node->index = -1;

	mHeap[index] = mHeap.Back();
	mHeap[index]->index = index;
	mHeap.PopBack();

	if (index < mHeap.count )
	{
		if ( mHeap[index]->key > node->key )
		{
			BubbleDown(index);
		}
		else
		{
			BubbleUp(index);
		}
	}

	B3_ASSERT( IsConsistent() );
}

//--------------------------------------------------------------------------------------------------
b3HeapNode* b3Heap::Pop()
{
	B3_ASSERT( mHeap[0]->index == 0 );

	b3HeapNode* root = mHeap[0];
	root->index = -1;

	mHeap[0] = mHeap.Back();
	mHeap[0]->index = 0;
	mHeap.PopBack();

	BubbleDown( 0 );

	B3_ASSERT( IsConsistent() );

	return root;
}

//--------------------------------------------------------------------------------------------------
b3HeapNode* b3Heap::Top()
{
	B3_ASSERT( mHeap[0]->index == 0 );
	return mHeap[0];
}

//--------------------------------------------------------------------------------------------------
const b3HeapNode* b3Heap::Top() const
{
	B3_ASSERT( mHeap[0]->index == 0 );
	return mHeap[0];
}

//--------------------------------------------------------------------------------------------------
bool b3Heap::IsConsistent() const
{
	double minKey = DBL_MAX;
	for ( int index = 0; index < mHeap.count; ++index)
	{
		minKey = b3MinDouble(minKey, mHeap[index]->key );

		if ( mHeap[index]->index != index)
		{
			return false;
		}

		if ( mHeap[index]->key < mHeap[0]->key )
		{
			return false;
		}

		int leftChild = 2 * index + 1;
		if ( leftChild < mHeap.count && mHeap[index]->key > mHeap[leftChild]->key )
		{
			return false;
		}

		int rightChild = leftChild + 1;
		if ( rightChild < mHeap.count && mHeap[index]->key > mHeap[rightChild]->key )
		{
			return false;
		}
	}

	return mHeap.IsEmpty() || mHeap[0]->key == minKey;
}

//--------------------------------------------------------------------------------------------------
inline void b3Heap::BubbleUp( int index)
{
	b3HeapNode* node = mHeap[index];
	B3_ASSERT(node->index == index);

	int parent = (index - 1 ) / 2;
	while (index > 0 && mHeap[parent]->key > node->key )
	{
		// Move parent down
		mHeap[index] = mHeap[parent];
		mHeap[index]->index = index;

		// Move node index up
		index = parent;
		parent = (parent - 1 ) / 2;
	}

	// Save node at its new location
	mHeap[index] = node;
	node->index = index;
}

//--------------------------------------------------------------------------------------------------
inline void b3Heap::BubbleDown( int index)
{
	b3HeapNode* node = mHeap[index];
	B3_ASSERT(node->index == index);

	while (index < mHeap.GetCount() / 2 )
	{
		int leftChild = 2 * index + 1;
		int rightChild = leftChild + 1;

		int smallerChild = leftChild;
		if ( rightChild < mHeap.GetCount() && mHeap[rightChild]->key < mHeap[leftChild]->key )
		{
			smallerChild = rightChild;
		}

		if (node->key <= mHeap[smallerChild]->key )
		{
			break;
		}

		// Move larger child up
		mHeap[index] = mHeap[smallerChild];
		mHeap[index]->index = index;

		// Move node index down
		index = smallerChild;
	}

	// Save node at its new location
	mHeap[index] = node;
	node->index = index;
}
