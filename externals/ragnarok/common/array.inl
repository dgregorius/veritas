//--------------------------------------------------------------------------------------------------
// array.inl	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkArray
//--------------------------------------------------------------------------------------------------
template < typename T >
inline RkArray< T >::RkArray( RkArray&& Other )
	{
	if ( Other.mAllocated )
		{
		// Other owns memory - swap
		mFirst = Other.mFirst;
		mLast = Other.mLast;
		mEnd = Other.mEnd;

		Other.mFirst = nullptr;
		Other.mLast = nullptr;
		Other.mEnd = nullptr;
		}
	else
		{
		// Other does not own memory - copy
		Reserve( Other.Size() );

		rkUninitializedCopy( Other.Begin(), Other.End(), Begin() );
		mLast = mFirst + Other.Size();
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline RkArray< T >::RkArray( const RkArray& Other )
	{
	Reserve( Other.Size() );
	
	rkUninitializedCopy( Other.Begin(), Other.End(), Begin() );
	mLast = mFirst + Other.Size();
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
RkArray< T >::RkArray( std::initializer_list< T > List )
	{
	Resize( static_cast< int >( List.size() ) );
	rkCopy( List.begin(), List.end(), mFirst );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline RkArray< T >::RkArray( const T* First, const T* Last )
	{
	if ( Last > First )
		{
		Resize( static_cast< int >( Last - First ) );
		rkCopy( First, Last, mFirst );
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline RkArray< T >::RkArray( int Count )
	{
	if ( Count > 0 )
		{
		Resize( Count );
		}
	}



//--------------------------------------------------------------------------------------------------
template < typename T >
inline RkArray< T >::RkArray( int Count, const T& Value )
	{
	if ( Count > 0 )
		{
		Resize( Count, Value );
		}
	}



//--------------------------------------------------------------------------------------------------
template < typename T >
inline RkArray< T >::RkArray( T* First, T* Last, T* End )
	{
	RK_ASSERT( First <= Last && Last <= End );
	mFirst = First;
	mLast = Last;
	mEnd = End;

	// We do *not* own this memory
	mAllocated = false;
	}



//--------------------------------------------------------------------------------------------------
template < typename T >
inline RkArray< T >::~RkArray()
	{
	// Deallocate
	rkDestroy( mFirst, mLast );
	if ( mAllocated )
		{
		rkAlignedFree( mFirst );
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline RkArray< T >& RkArray< T >::operator=( RkArray&& Other )
	{
	if ( this != &Other )
		{
		if ( Other.mAllocated )
			{
			// Destroy
			rkDestroy( mFirst, mLast );
			if ( mAllocated )
				{
				rkAlignedFree( mFirst );
				}

			// Move
			mFirst = Other.mFirst;
			mLast = Other.mLast;
			mEnd = Other.mEnd;

			Other.mFirst = nullptr;
			Other.mLast = nullptr;
			Other.mEnd = nullptr;
			}
		else
			{
			// Copy
			Resize( Other.Size() );
			rkCopy( Other.Begin(), Other.End(), Begin() );
			}
		}

	return *this;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline RkArray< T >& RkArray< T >::operator=( const RkArray& Other )
	{
	if ( this != &Other )
		{
		Resize( Other.Size() );
		rkCopy( Other.Begin(), Other.End(), Begin() );
		}

	return *this;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline RkArray< T >& RkArray<T>::operator=( std::initializer_list< T > List )
	{
	Resize( static_cast< int >( List.size() ) );
	rkCopy( List.begin(), List.end(), Begin() );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline int RkArray< T >::Size() const
	{
	return static_cast< int >( mLast - mFirst );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline int RkArray< T >::Capacity() const
	{
	return static_cast< int >( mEnd - mFirst );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline bool RkArray< T >::Empty() const
	{
	return mLast == mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline void RkArray< T >::Clear()
	{
	rkDestroy( mFirst, mLast );
	mLast = mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
RK_NOINLINE void RkArray< T >::Reserve( int Count )
	{
	RK_ASSERT( Count >= 0 );
	if ( Count > Capacity() )
		{
		// Allocate new buffers
		static_assert( alignof( T ) <= 16 );
		T* First = static_cast< T* >( rkAlignedAlloc( Count * sizeof( T ), 16 ) );
		T* Last = First + Size();
		T* End = First + Count;

		// Move old buffer
		if ( !Empty() )
			{
			rkUninitializedMove( mFirst, mLast, First );
			}

		rkDestroy( mFirst, mLast );
		if ( mAllocated )
			{
			rkAlignedFree( mFirst );
			}
		
		// Save new buffers
		mFirst = First;
		mLast = Last;
		mEnd = End;

		// We now own the memory
		mAllocated = true;
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline void RkArray< T >::Resize( int Count )
	{
	Reserve( Count );

	rkDestroy( Size() - Count, mFirst + Count );
	rkConstruct( Count - Size(), mLast );
	mLast = mFirst + Count;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline void RkArray< T >::Resize( int Count, const T& Value )
	{
	Reserve( Count );

	rkDestroy( Size() - Count, mFirst + Count );
	rkCopyConstruct( Count - Size(), mLast, Value );
	mLast = mFirst + Count;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
RK_FORCEINLINE int RkArray< T >::PushBack()
	{
	if ( mLast == mEnd )
		{
		// Try to grow 50%
		Grow();
		}
	rkConstruct( mLast++ );

	return Size() - 1;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
RK_FORCEINLINE int RkArray< T >::PushBack( const T& Value )
	{
	if ( mFirst <= std::addressof( Value ) && std::addressof( Value ) < mLast )
		{
		// The object is part of the array. Growing the
		// array might destroy it and invalidate the reference
		int Index = int( std::addressof( Value ) - mFirst );
		if ( mLast == mEnd )
			{
			// Try to grow 50%
			Grow();
			}

		rkCopyConstruct( mLast++, mFirst[ Index ] );
		}
	else
		{
		if ( mLast == mEnd )
			{
			// Try to grow 50%
			Grow();
			}

		rkCopyConstruct( mLast++, Value );
		}

	return Size() - 1;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
RK_FORCEINLINE void RkArray< T >::PushBack( int Count, const T* List )
	{
	PushBack( List, List + Count );
	}


//--------------------------------------------------------------------------------------------------
template <typename T>
RK_FORCEINLINE void RkArray< T >::PushBack( const T* First, const T* Last )
	{
	int Count = int( Last - First );
	if ( Count > 0 )
		{
		// DIRK_TODO: Handle self append!
		RK_ASSERT( Last < mFirst || First >= mEnd );

		// DIRK_TODO: Use reserve
		Resize( Size() + Count );
		rkCopy( First, Last, mLast - Count );
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
RK_FORCEINLINE void RkArray< T >::PushBack( const RkArray< T >& Other )
	{
	PushBack( Other.Begin(), Other.End() );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline void RkArray< T >::Insert( int Where, const T& Value )
	{
	// DIRK_TODO: Handle insertion of elements inside array...
	RK_ASSERT( std::addressof( Value ) < mFirst || mLast < std::addressof( Value ) );

	// As in vector::insert() the insertion index can be one larger 
	// than the vector size indicating insertion at the end.
	RK_ASSERT( 0 <= Where && Where <= Size() );
	if ( 0 <= Where && Where <= Size() )
		{
		// Add new object
		PushBack();

		// Shift down
		for ( int Index = Size() - 1; Index > Where; --Index )
			{
			mFirst[ Index ] = std::move( mFirst[ Index - 1 ] );
			}

		// Insert 
		mFirst[ Where ] = Value;
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline void RkArray< T >::PopBack()
	{
	RK_ASSERT( !Empty() );
	rkDestroy( --mLast );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline void RkArray<T>::Remove( int Where )
	{
	if ( 0 <= Where && Where < Size() )
		{
		T* First = mFirst + Where;
		for ( T* Iterator = First; ++Iterator != mLast; )
			{
			*First++ = std::move( *Iterator );
			}

		rkDestroy( First, mLast );
		mLast = First;
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline void RkArray< T >::Remove( const T& Value )
	{
	T* First = std::remove( mFirst, mLast, Value );

	rkDestroy( First, mLast );
	mLast = First;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline bool RkArray< T >::Contains( const T& Value ) const
	{
	return std::find( mFirst, mLast, Value ) != mLast;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > 
inline int RkArray< T >::IndexOf( const T& Value ) const
	{
	int Index = static_cast< int >( std::distance( mFirst, std::find( mFirst, mLast, Value ) ) );
	if ( Index == Size() )
		{
		Index = -1;
		}

	return Index;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline T& RkArray< T >::Front()
	{
	RK_ASSERT( !Empty() );
	return *mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline const T& RkArray< T >::Front() const
	{
	RK_ASSERT( !Empty() );
	return *mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline T& RkArray< T >::Back()
	{
	RK_ASSERT( !Empty() );
	return *( mLast - 1 );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline const T& RkArray< T >::Back() const
	{
	RK_ASSERT( !Empty() );
	return *( mLast - 1 );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline T* RkArray< T >::Begin()
	{
	return mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline const T* RkArray< T >::Begin() const
	{
	return mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline T* RkArray< T >::End()
	{
	return mLast;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline const T* RkArray< T >::End() const
	{
	return mLast;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline T* RkArray< T >::Data()
	{
	return mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline const T* RkArray< T >::Data() const
	{
	return mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline T& RkArray< T >::operator[]( int Index )
	{
	RK_ASSERT( 0 <= Index && Index < Size() );
	return *( mFirst + Index );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline const T& RkArray< T >::operator[]( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < Size() );
	return *( mFirst + Index );
	}



//--------------------------------------------------------------------------------------------------
template < typename T >
inline bool RkArray< T >::operator==( const RkArray& Other ) const
	{
	return Size() == Other.Size() && std::equal( mFirst, mLast, Other.mFirst );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline bool RkArray< T >::operator!=( const RkArray& Other ) const
	{
	return !( *this == Other );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > RK_NOINLINE
void RkArray<T>::Grow()
	{
	int CurrentCapacity = Capacity();
	int NewCapacity = CurrentCapacity > 0 ? CurrentCapacity + CurrentCapacity / 2 : InitialCapacity;
	Reserve( NewCapacity );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline void rkDeleteAll( T** First, T** Last )
	{
	// Iterate backwards
	for ( T** Iterator = Last; Iterator-- != First; )
		{
		delete *Iterator;
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
inline void rkDeleteAll( RkArray< T* >& Array )
	{
	rkDeleteAll( Array.Begin(), Array.End() );
	}


//--------------------------------------------------------------------------------------------------
// RkStackArray 
//--------------------------------------------------------------------------------------------------
template < typename T, int N > 
RkStackArray< T, N >::RkStackArray()
	: RkArray< T >( reinterpret_cast< T* >( mStorage ), reinterpret_cast< T* >( mStorage ), reinterpret_cast< T* >( mStorage ) + N )
	{
	RK_ASSERT( !this->mAllocated );
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int N >
RkStackArray< T, N >::RkStackArray( int Count )
	: RkStackArray< T, N >()
	{
	if ( Count > 0 )
		{
		this->Resize( Count );
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int N >
RkStackArray< T, N >::RkStackArray( int Count, const T& Value )
	: RkStackArray< T, N >()
	{
	if ( Count > 0 )
		{
		this->Resize( Count, Value );
		}
	}


//--------------------------------------------------------------------------------------------------
// RkIndexedArray
//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex >
inline int RkIndexedArray< T, MemberIndex >::Size() const
	{
	return mArray.Size();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex >
inline int RkIndexedArray< T, MemberIndex >::Capacity() const
	{
	return mArray.Capacity();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex >
inline bool RkIndexedArray< T, MemberIndex >::Empty() const
	{
	return mArray.Empty();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex >
inline int RkIndexedArray< T, MemberIndex >::PushBack( T* Object )
	{
	RK_ASSERT( Object && Object->*MemberIndex == -1 );
	
	int Index = mArray.PushBack( Object );
	Object->*MemberIndex = Index;

	return Index;
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex >
inline void RkIndexedArray<T, MemberIndex>::PopBack()
	{
	mArray.PopBack();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex  >
inline void RkIndexedArray< T, MemberIndex >::Remove( T* Object )
	{
	RK_ASSERT( Object && Object->*MemberIndex != -1 );
	
	int Index = Object->*MemberIndex;
	RK_ASSERT( mArray[ Index ] == Object );
	Object->*MemberIndex = -1;

	if ( Index < mArray.Size() - 1 )
		{
		mArray[ Index ] = mArray.Back();
		mArray[ Index ]->*MemberIndex = Index;
		}

	mArray.PopBack();
	RK_ASSERT( Object->*MemberIndex == -1 );
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::* MemberIndex >
inline bool RkIndexedArray< T, MemberIndex >::Contains( T* Value ) const
	{
	return mArray.Contains( Value );
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex  >
inline T* RkIndexedArray<T, MemberIndex>::Front()
	{
	return mArray.Front();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex  >
const T* RkIndexedArray<T, MemberIndex>::Front() const
	{
	return mArray.Front();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex  >
inline T* RkIndexedArray<T, MemberIndex>::Back()
	{
	return mArray.Back();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex  >
inline const T* RkIndexedArray<T, MemberIndex>::Back() const
	{
	return mArray.Back();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex  >
inline T** RkIndexedArray<T, MemberIndex>::Begin()
	{
	return mArray.Begin();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex  >
inline T* const* RkIndexedArray<T, MemberIndex>::Begin() const
	{
	return mArray.Begin();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex  >
inline T** RkIndexedArray<T, MemberIndex>::End()
	{
	return mArray.End();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex >
inline T* const* RkIndexedArray<T, MemberIndex>::End() const
	{
	return mArray.End();
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex >
inline const T* RkIndexedArray< T, MemberIndex >::operator[]( int Index ) const
	{
	return mArray[ Index ];
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex >
inline T* RkIndexedArray< T, MemberIndex >::operator[]( int Index )
	{
	return mArray[ Index ];
	}


//--------------------------------------------------------------------------------------------------
template < typename T, int T::* MemberIndex >
inline void rkDeleteAll( RkIndexedArray< T, MemberIndex >& Array )
	{
	rkDeleteAll( Array.Begin(), Array.End() );
	}