//--------------------------------------------------------------------------------------------------
// resource.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkResourcePointer
//--------------------------------------------------------------------------------------------------
template < typename T >  inline
RkResourcePointer< T >::RkResourcePointer( const T* Pointer )
	{
	mOffset = Pointer ? int32( intp( Pointer ) - intp( this ) ) : 0;
	RK_ASSERT( mOffset >= 0 );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
RkResourcePointer< T >::RkResourcePointer( const T& Reference )
	{
	mOffset = int32( intp( &Reference ) - intp( this ) );
	RK_ASSERT( mOffset >= 0 );
	}


//--------------------------------------------------------------------------------------------------
template < typename T >  inline
RkResourcePointer< T >& RkResourcePointer< T >::operator=( const T* Pointer )
	{
	mOffset = Pointer ? int32( intp( Pointer ) - intp( this ) ) : 0;
	RK_ASSERT( mOffset >= 0 );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
RkResourcePointer< T >& RkResourcePointer< T >::operator=( const T& Reference )
	{
	mOffset = int32( intp( &Reference ) - intp( this ) );
	RK_ASSERT( mOffset >= 0 );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
T* RkResourcePointer< T >::operator->()
	{
	RK_ASSERT( mOffset >= 0 );
	T* Pointer = mOffset ? reinterpret_cast<T*>( intp( this ) + mOffset ) : nullptr;
	RK_ASSERT( rkIsAligned( Pointer ) );

	return Pointer;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
const T* RkResourcePointer< T >::operator->() const
	{
	RK_ASSERT( mOffset >= 0 );
	const T* Pointer = mOffset ? reinterpret_cast<const T*>( intp( this ) + mOffset ) : nullptr;
	RK_ASSERT( rkIsAligned( Pointer ) );

	return Pointer;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
RkResourcePointer< T >::operator T* ( )
	{
	RK_ASSERT( mOffset >= 0 );
	T* Pointer = mOffset ? reinterpret_cast<T*>( intp( this ) + mOffset ) : nullptr;
	RK_ASSERT( rkIsAligned( Pointer ) );

	return Pointer;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
RkResourcePointer< T >::operator const T* ( ) const
	{
	RK_ASSERT( mOffset >= 0 );
	const T* Pointer = mOffset ? reinterpret_cast<const T*>( intp( this ) + mOffset ) : nullptr;
	RK_ASSERT( rkIsAligned( Pointer ) );

	return Pointer;
	}


//--------------------------------------------------------------------------------------------------
template < typename T > inline
bool RkResourcePointer< T >::IsAligned( int Alignment ) const
	{
	return ( ( intp( this ) + mOffset ) & ( Alignment - 1 ) ) == 0;
	}