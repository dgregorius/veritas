//--------------------------------------------------------------------------------------------------
// qhList.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// qhListNode
//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhInsert( T* Node, T* Where )
	{
	QH_ASSERT( !qhInList( Node ) && qhInList( Where ) );

	Node->Prev = Where->Prev;
	Node->Next = Where;

	Node->Prev->Next = Node; 
	Node->Next->Prev = Node;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhRemove( T* Node )
	{
	QH_ASSERT( qhInList( Node ) );

	Node->Prev->Next = Node->Next; 
	Node->Next->Prev = Node->Prev;

	Node->Prev = nullptr;
	Node->Next = nullptr;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
bool qhInList( T* Node )
	{
	return Node->Prev != nullptr && Node->Next != nullptr;
	}


//--------------------------------------------------------------------------------------------------
// qhList
//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhList< T >::qhList() 
	{
	mHead.Prev = &mHead;
	mHead.Next = &mHead;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
int qhList< T >::Size() const
	{
	int Count = 0;
	for ( const T* Node = Begin(); Node != End(); Node = Node->Next )
		{
		Count++;
		}

	return Count;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
bool qhList< T >::Empty() const
	{
	return mHead.Next == &mHead;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhList< T >::Clear() 
	{
	mHead.Prev = &mHead;
	mHead.Next = &mHead;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhList< T >::PushFront( T* Node )
	{
	qhInsert( Node, mHead.Next );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T* qhList< T >::PopFront()
	{
	QH_ASSERT( !Empty() );
	T* Node = mHead.Next;
	qhRemove( Node );

	return Node;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhList< T >::PushBack( T* Node )
	{
	qhInsert( Node, mHead.Prev );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T* qhList< T >::PopBack()
	{
	QH_ASSERT( !Empty() );
	T* Node = mHead.Prev;
	qhRemove( Node );

	return Node;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhList< T >::Insert( T* Node, T* Where )
	{
	qhInsert( Node, Where );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhList< T >::Remove( T* Node )
	{
	qhRemove( Node );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
int qhList< T >::IndexOf( const T* Node ) const
	{
	int Index = 0;
	for ( const T* First = Begin(); First != End(); First = First->Next )
		{
		if ( First == Node )
			{
			return Index;
			}

		Index++;
		}

	return -1;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T* qhList< T >::Front()
	{
	QH_ASSERT( !Empty() );
	return mHead.Next;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
const T* qhList< T >::Front() const
	{
	QH_ASSERT( !Empty() );
	return mHead.Next;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T* qhList< T >::Back()
	{
	QH_ASSERT( !Empty() );
	return mHead.Prev;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
const T* qhList< T >::Back() const
	{
	QH_ASSERT( !Empty() );
	return mHead.Prev;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T* qhList< T >::Begin()
	{
	return mHead.Next;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
const T* qhList< T >::Begin() const 
	{
	return mHead.Next;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T* qhList< T >::End()
	{
	return &mHead;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
const T* qhList< T >::End() const
	{
	return &mHead;
	}










