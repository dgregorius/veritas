//--------------------------------------------------------------------------------------------------
// hull.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkHull
//--------------------------------------------------------------------------------------------------
inline RkVector3 RkHull::GetPosition( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < VertexCount );
	return PositionList[ Index ];
	}


//--------------------------------------------------------------------------------------------------
inline const RkVertex* RkHull::GetVertex( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < VertexCount );
	return VertexList + Index;
	}


//--------------------------------------------------------------------------------------------------
inline int RkHull::GetVertexIndex( const RkVertex* Vertex ) const
	{
	const RkVertex* VertexBegin = VertexList;
	const RkVertex* VertexEnd = VertexBegin + VertexCount;

	if ( VertexBegin <= Vertex && Vertex < VertexEnd )
		{
		return static_cast< int >(Vertex - VertexBegin);
		}

	return -1;
	}


//--------------------------------------------------------------------------------------------------
inline const RkHalfEdge* RkHull::GetEdge( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < EdgeCount );
	return EdgeList + Index;
	}


//--------------------------------------------------------------------------------------------------
inline int RkHull::GetEdgeIndex( const RkHalfEdge* Edge ) const
	{
	const RkHalfEdge* EdgeBegin = EdgeList;
	const RkHalfEdge* EdgeEnd = EdgeBegin + EdgeCount;

	if ( EdgeBegin <= Edge && Edge < EdgeEnd )
		{
		return static_cast< int >(Edge - EdgeBegin);
		}

	return -1;
	}


//--------------------------------------------------------------------------------------------------
inline RkPlane3 RkHull::GetPlane( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < FaceCount );
	return PlaneList[ Index ];
	}


//--------------------------------------------------------------------------------------------------
inline const RkFace* RkHull::GetFace( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < FaceCount );
	return FaceList + Index;
	}


//--------------------------------------------------------------------------------------------------
inline int RkHull::GetFaceIndex( const RkFace* Face ) const
	{
	const RkFace* FaceBegin = FaceList;
	const RkFace* FaceEnd = FaceBegin + FaceCount;

	if ( FaceBegin <= Face && Face < FaceEnd )
		{
		return static_cast< int >(Face - FaceBegin);
		}

	return -1;
	}







