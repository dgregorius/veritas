//--------------------------------------------------------------------------------------------------
// qhConvex.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// qhConvex
//--------------------------------------------------------------------------------------------------
inline int qhConvex::GetVertexCount() const
	{
	return mVertexList.Size();
	}


//--------------------------------------------------------------------------------------------------
inline int qhConvex::GetHalfEdgeCount() const
	{
	int Count = 0;
	for ( const qhFace* Face = mFaceList.Begin(); Face != mFaceList.End(); Face = Face->Next )
		{
		qhHalfEdge* Edge = Face->Edge;
		QH_ASSERT( Edge != nullptr );

		do 
			{
			Count += 1;
			Edge = Edge->Next;
			} 
		while ( Edge != Face->Edge );
		}

	return Count;
	}


//--------------------------------------------------------------------------------------------------
inline const qhList< qhVertex >& qhConvex::GetVertexList() const
	{
	return mVertexList;
	}


//--------------------------------------------------------------------------------------------------
inline const qhList< qhFace >& qhConvex::GetFaceList() const
	{
	return mFaceList;
	}


//--------------------------------------------------------------------------------------------------
inline int qhConvex::GetFaceCount() const
	{
	return mFaceList.Size();
	}


