//--------------------------------------------------------------------------------------------------
// gjk.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// GJK proxy
//--------------------------------------------------------------------------------------------------
inline int RkGJKProxy::GetVertexCount() const
	{
	return mVertexCount;
	}


//--------------------------------------------------------------------------------------------------
inline const RkVector3* RkGJKProxy::GetVertexBuffer() const
	{
	return mVertexBuffer;
	}


//--------------------------------------------------------------------------------------------------
inline const RkVector3& RkGJKProxy::GetVertex( int Index ) const
	{
	RK_ASSERT( mVertexCount > 0 );
	RK_ASSERT( mVertexBuffer );
	RK_ASSERT( 0 <= Index && Index < mVertexCount );

	return mVertexBuffer[ Index ];
	}




