//--------------------------------------------------------------------------------------------------
// qhMemory.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// qhMemory
//--------------------------------------------------------------------------------------------------
inline void* qhAddByteOffset( void* Address, size_t Offset )
	{
	return reinterpret_cast< qhUInt8* >( Address ) + Offset;
	}


//--------------------------------------------------------------------------------------------------
inline const void* qhAddByteOffset( const void* Address, size_t Offset )
	{
	return reinterpret_cast< const qhUInt8* >( Address ) + Offset;
	}


//--------------------------------------------------------------------------------------------------
inline void* qhAlign( void* Address, size_t Alignment )
	{
	qhIntPtr Offset = reinterpret_cast< qhIntPtr >( Address );
	Offset = ( Offset + Alignment - 1 ) & ~( Alignment - 1 );
	QH_ASSERT( ( Offset & ( Alignment - 1 ) ) == 0 );

	return reinterpret_cast< void* >( Offset );
	}


//--------------------------------------------------------------------------------------------------
inline const void* qhAlign( const void* Address, size_t Alignment )
	{
	qhIntPtr Offset = reinterpret_cast< qhIntPtr >( Address );
	Offset = ( Offset + Alignment - 1 ) & ~( Alignment - 1 );
	QH_ASSERT( ( Offset & ( Alignment - 1 ) ) == 0 );

	return reinterpret_cast< const void* >( Offset );
	}


//--------------------------------------------------------------------------------------------------
inline bool qhIsAligned( const void* Address, size_t Alignment )
	{
	return ( reinterpret_cast< qhIntPtr >( Address ) & ( Alignment - 1 ) ) == 0;
	}


//--------------------------------------------------------------------------------------------------
// qhPool
//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhPool< T >::qhPool()
	: mSize( 0 )
	, mPool( nullptr )
	, mFree( -1 )
	{

	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
qhPool< T >::~qhPool()
	{
	qhFree( mPool );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhPool< T >::Clear()
	{
	mSize = 0;
	qhFree( mPool );
	mPool = nullptr;
	mFree = -1;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhPool< T >::Resize( int Size )
	{
	QH_ASSERT( mSize == 0 && mPool == nullptr );

	mSize = Size;
	mPool = (T*)qhAlloc( Size * sizeof( T ), alignof( T ) );
	mFree = 0;

	for ( int i = 0; i < mSize - 1; ++i )
		{
		int* Next = (int*)( mPool + i );
		*Next = i + 1;
		}

	int* Next = (int*)( mPool + mSize - 1 );
	*Next = -1;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T* qhPool< T >::Allocate()
	{
	QH_ASSERT( mFree >= 0 );
	int Next = mFree;
	mFree = *(int*)( mPool + mFree );

#ifdef QH_DEBUG
	memset( mPool + Next, 0xcd, sizeof( T ) );
#endif

	return mPool + Next;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhPool< T >::Free( T* Address )
	{
#ifdef QH_DEBUG
	QH_ASSERT( Address != nullptr );
	memset( Address, 0xdb, sizeof( T ) );
#endif

	QH_ASSERT( mPool <= Address && Address < mPool + mSize );
	int* Next = (int*)Address;
	*Next = mFree;
	
	mFree = int( Address - mPool );
	QH_ASSERT( 0 <= mFree && mFree < mSize );
	}


//--------------------------------------------------------------------------------------------------
// Memory utilities
//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhConstruct( T* Object )
	{
	QH_ASSERT( Object );
	new ( static_cast< void* >( Object ) ) T();
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhConstruct( int Count, T* Objects )
	{
	for ( int Index = 0; Index < Count; ++Index )
		{
		new ( static_cast< void* >( Objects + Index ) ) T();
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
void qhConstruct( T* First, T* Last )
	{
	QH_ASSERT( Last >= First );
	while ( First != Last )
		{
		new ( static_cast< void* >( First++ ) ) T();
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
void qhDestroy( T*, std::true_type )
	{
	// Intentionally empty!
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
void qhDestroy( T* Object, std::false_type )
	{
	if ( Object )
		{
		Object->~T();
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
void qhDestroy( T* Object )
	{
	qhDestroy( Object, std::is_trivially_destructible< T >() );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
void qhDestroy( int, T*, std::true_type )
	{
	// Intentionally empty!
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
void qhDestroy( int Count, T* Objects, std::false_type )
	{
	for ( int Index = 0; Index < Count; ++Index )
		{
		( Objects + Index )->~T();
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
void qhDestroy( int Count, T* Objects )
	{
	qhDestroy( Count, Objects, std::is_trivially_destructible< T >() );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
void qhDestroy( T*, T*, std::true_type )
	{
	// Intentionally empty!
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
void qhDestroy( T* First, T* Last, std::false_type )
	{
	while ( First != Last )
		{
		( First++ )->~T();
		}
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
void qhDestroy( T* First, T* Last )
	{
	QH_ASSERT( Last >= First );
	qhDestroy( First, Last, std::is_trivially_destructible< T >() );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhCopyConstruct( T* Object, const T& Value )
	{
	QH_ASSERT( Object );
	new ( static_cast< void* >( Object ) ) T( Value );
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
void qhMoveConstruct( T* Object, T&& Value )
	{
	QH_ASSERT( Object );
	new ( static_cast< void* >( Object ) ) T( std::move( Value ) );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhCopy( const T* First, const T* Last, T* Dest, std::true_type )
	{
	memcpy( Dest, First, ( Last - First ) * sizeof( T ) );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhCopy( const T* First, const T* Last, T* Dest, std::false_type )
	{
	while ( First != Last )
		{
		*Dest++ = *First++;
		}
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhCopy( const T* First, const T* Last, T* Dest )
	{
	QH_ASSERT( Last >= First );
	qhCopy( First, Last, Dest, std::is_trivially_copyable< T >() );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhMove( T* First, T* Last, T* Dest, std::true_type )
	{
	memmove( Dest, First, ( Last - First ) * sizeof( T ) );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhMove( T* First, T* Last, T* Dest, std::false_type )
	{
	while ( First != Last )
		{
		*Dest++ = std::move( *First++ );
		}
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhMove( T* First, T* Last, T* Dest )
	{
	QH_ASSERT( Last >= First );
	qhMove( First, Last, Dest, std::is_trivially_move_assignable< T >() );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhFill( T* First, T* Last, const T& Value )
	{
	QH_ASSERT( Last >= First );
	while ( First != Last )
		{
		*First++ = Value;
		}
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhUninitializedCopy( const T* First, const T* Last, T* Dest, std::true_type )
	{
	rnMemCpy( Dest, First, ( Last - First ) * sizeof( T ) );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhUninitializedCopy( const T* First, const T* Last, T* Dest, std::false_type )
	{
	while ( First != Last )
		{
		new ( static_cast< void* >( std::addressof( *Dest++ ) ) ) T( *First++ );
		}
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhUninitializedCopy( const T* First, const T* Last, T* Dest )
	{
	QH_ASSERT( Last >= First );
	qhUninitializedCopy( First, Last, Dest, std::is_trivially_copy_constructible< T >() );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhUninitializedMove( T* First, T* Last, T* Dest, std::true_type )
	{
	memmove( Dest, First, ( Last - First ) * sizeof( T ) );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhUninitializedMove( T* First, T* Last, T* Dest, std::false_type )
	{
	for ( ; First != Last; ++First, ++Dest )
		{
		new ( static_cast< void* >( Dest ) ) T( std::move( *First ) );
		}
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhUninitializedMove( T* First, T* Last, T* Dest )
	{
	QH_ASSERT( Last >= First );
	qhUninitializedMove( First, Last, Dest, std::is_trivially_copy_constructible< T >() );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline
void qhUninitializedFill( T* First, T* Last, const T& Value )
	{
	QH_ASSERT( Last >= First );
	while ( First != Last )
		{
		new ( static_cast< void* >( First++ ) ) T( Value );
		}
	}
