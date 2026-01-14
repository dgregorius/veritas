//--------------------------------------------------------------------------------------------------
// qhArray.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// qhArray
//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhArray< T >::qhArray()
	: mFirst( nullptr )
	, mLast( nullptr )
	, mEnd( nullptr )
	{

	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhArray< T >::qhArray( std::initializer_list< T > List )
	: qhArray( List.begin(), List.end() )
	{

	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhArray< T >::qhArray( const T* First, const T* Last )
	: mFirst( nullptr )
	, mLast( nullptr )
	, mEnd( nullptr )
	{
	if ( Last > First )
		{
		Resize( int( Last - First ) );
		qhCopy( First, Last, mFirst );
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhArray< T >::qhArray( const qhArray& Other )
	: mFirst( nullptr )
	, mLast( nullptr )
	, mEnd( nullptr )
	{
	*this = Other;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhArray< T >::qhArray( qhArray&& Other )
	{
	mFirst = Other.mFirst;
	mLast = Other.mLast;
	mEnd = Other.mEnd;

	Other.mFirst = nullptr;
	Other.mLast = nullptr;
	Other.mEnd = nullptr;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhArray< T >::~qhArray()
	{
	qhDestroy( mFirst, mLast );
	qhFree( mFirst );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >
qhArray< T >& qhArray<T>::operator=( std::initializer_list< T > List )
	{
	Resize( List.size() );
	qhCopy( List.begin(), List.end(), Begin() );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhArray< T >& qhArray< T >::operator=( const qhArray& Other )
	{
	if ( this != &Other )
		{
		Resize( Other.Size() );
		qhCopy( Other.Begin(), Other.End(), Begin() );
		}

	return *this;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhArray< T >& qhArray< T >::operator=( qhArray&& Other )
	{
	if ( this != &Other )
		{
		qhDestroy( mFirst, mLast );
		qhFree( mFirst );

		mFirst = Other.mFirst;
		mLast = Other.mLast;
		mEnd = Other.mEnd;

		Other.mFirst = nullptr;
		Other.mLast = nullptr;
		Other.mEnd = nullptr;
		}

	return *this;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
int qhArray< T >::Size() const
	{
	return int( mLast - mFirst );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
int qhArray< T >::Capacity() const
	{
	return int( mEnd - mFirst );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
bool qhArray< T >::Empty() const
	{
	return mLast == mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhArray< T >::Clear()
	{
	qhDestroy( mFirst, mLast );
	mLast = mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhArray< T >::Reserve( int Count )
	{
	QH_ASSERT( Count >= 0 );
	if ( Count > Capacity() )
		{
		// Allocate new buffers
		T* First = static_cast< T* >( qhAlloc( Count * sizeof( T ), __alignof( T ) ) );
		T* Last = First + Size();
		T* End = First + Count;

		// Move old buffer and free it
		if ( !Empty() )
			{
			qhUninitializedMove( mFirst, mLast, First );
			}
		qhDestroy( mFirst, mLast );
		qhFree( mFirst );

		// Save new buffers
		mFirst = First;
		mLast = Last;
		mEnd = End;
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhArray< T >::Resize( int Count )
	{
	QH_ASSERT( Count >= 0 );
	Reserve( Count );

	qhDestroy( Size() - Count, mFirst + Count );
	qhConstruct( Count - Size(), mLast );
	mLast = mFirst + Count;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
int qhArray< T >::PushBack()
	{
	if ( mLast == mEnd )
		{
		// Try to grow 50%
		Reserve( 3 * Capacity() / 2 + 1 );
		}
	qhConstruct( mLast++ );

	return Size() - 1;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhArray< T >::PushBack( const T& Other )
	{
	if ( Inside( std::addressof( Other ) ) )
		{
		// The object is part of the array. Growing the
		// array might destroy it and invalidate the reference
		int Index = int( std::addressof( Other ) - mFirst );
		if ( mLast == mEnd )
			{
			// Try to grow 50%
			Reserve( 3 * Capacity() / 2 + 1 );
			}

		new ( static_cast< void* >( mLast++ ) ) T( mFirst[ Index ] );
		}
	else
		{
		if ( mLast == mEnd )
			{
			// Try to grow 50%
			Reserve( 3 * Capacity() / 2 + 1 );
			}

		new ( static_cast< void* >( mLast++ ) ) T( Other );
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhArray< T >::PushBack( T&& Other )
	{
	if ( Inside( std::addressof( Other ) ) )
		{
		// The object is part of the vector. Growing the
		// array might destroy it and invalidate the reference
		int Index = int( std::addressof( Other ) - mFirst );
		if ( mLast == mEnd )
			{
			// Try to grow 50%
			Reserve( 3 * Capacity() / 2 + 1 );
			}

		new ( static_cast< void* >( mLast++ ) ) T( std::move( mFirst[ Index ] ) );
		}
	else
		{
		if ( mLast == mEnd )
			{
			// Try to grow 50%
			Reserve( 3 * Capacity() / 2 + 1 );
			}

		new ( static_cast< void* >( mLast++ ) ) T( std::move( Other ) );
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhArray< T >::PopBack()
	{
	QH_ASSERT( !Empty() );
	qhDestroy( --mLast );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhArray< T >::Append( const qhArray< T >& Other )
	{
	Append( Other.mFirst, Other.mLast );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhArray< T >::Append( const T* First, const T* Last )
	{
	int Count = int( Last - First );
	if ( Count > 0 )
		{
		// DIRK_TODO: Handle self append!
		QH_ASSERT( Last < mFirst || First >= mEnd );

		Resize( Size() + Count );
		rnCopy( First, Last, mLast - Count );
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
int qhArray<T>::IndexOf( const T& Object ) const
	{
	for ( int Index = 0; Index < Size(); ++Index )
		{
		if ( *( mFirst + Index ) == Object )
			{
			return Index;
			}
		}

	return -1;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T& qhArray< T >::operator[]( int Index )
	{
	QH_ASSERT( 0 <= Index && Index < Size() );
	return *( mFirst + Index );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
const T& qhArray< T >::operator[]( int Index ) const
	{
	QH_ASSERT( 0 <= Index && Index < Size() );
	return *( mFirst + Index );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T& qhArray< T >::Front()
	{
	QH_ASSERT( !Empty() );
	return *mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
const T& qhArray< T >::Front() const
	{
	QH_ASSERT( !Empty() );
	return *mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T& qhArray< T >::Back()
	{
	QH_ASSERT( !Empty() );
	return *( mLast - 1 );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
const T& qhArray< T >::Back() const
	{
	QH_ASSERT( !Empty() );
	return *( mLast - 1 );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T* qhArray< T >::Begin()
	{
	return mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
const T* qhArray< T >::Begin() const
	{
	return mFirst;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T* qhArray< T >::End()
	{
	return mLast;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
const T* qhArray< T >::End() const
	{
	return mLast;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhArray<T>::Swap( qhArray< T >& Other )
	{
	std::swap( mFirst, Other.mFirst );
	std::swap( mLast, Other.mLast );
	std::swap( mEnd, Other.mEnd );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
bool qhArray< T >::operator==( const qhArray& Other ) const
	{
	return Size() == Other.Size() && std::equal( mFirst, mLast, Other.mFirst );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
bool qhArray< T >::operator!=( const qhArray& Other ) const
	{
	return !( *this == Other );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
bool qhArray< T >::Inside( const T* Object ) const
	{
	return mFirst <= Object && Object < mLast;
	}