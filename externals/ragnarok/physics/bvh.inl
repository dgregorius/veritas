//--------------------------------------------------------------------------------------------------
// bvh.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkBVHTriangle
//--------------------------------------------------------------------------------------------------
inline RkBVHTriangle rkMakeTriangle( int Index1, int Index2, int Index3 )
	{
	RkBVHTriangle Out;
	Out.Index1 = Index1;
	Out.Index2 = Index2;
	Out.Index3 = Index3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
// RkBVHNode
//--------------------------------------------------------------------------------------------------
inline bool RkBVHNode::IsLeaf() const
	{
	return Type == kLeaf;
	}


//--------------------------------------------------------------------------------------------------
inline RkBVHNode* RkBVHNode::GetLeftChild()
	{
	// The left child follows its parent.
	RK_ASSERT( !IsLeaf() );
	return this + 1;
	}


//--------------------------------------------------------------------------------------------------
inline const RkBVHNode* RkBVHNode::GetLeftChild() const
	{
	// The left child follows its parent.
	RK_ASSERT( !IsLeaf() );
	return this + 1;
	}


//--------------------------------------------------------------------------------------------------
inline RkBVHNode* RkBVHNode::GetRightChild()
	{
	// We store the offset of the right child relative to its parent
	RK_ASSERT( !IsLeaf() );
	return this + ChildOffset;
	}


//--------------------------------------------------------------------------------------------------
inline const RkBVHNode* RkBVHNode::GetRightChild() const
	{
	// We store the offset of the right child relative to its parent
	RK_ASSERT( !IsLeaf() );
	return this + ChildOffset;
	}


//--------------------------------------------------------------------------------------------------
inline RkBounds3 RkBVHNode::GetBounds() const
	{
	return RkBounds3( RkVector3( BoundsMin ), RkVector3( BoundsMax ) );
	}


//--------------------------------------------------------------------------------------------------
// RkBVH
//--------------------------------------------------------------------------------------------------
inline RkBVHNode* RkBVH::GetRoot()
	{
	// The first node is the root
	return NodeList;
	}


//--------------------------------------------------------------------------------------------------
inline const RkBVHNode* RkBVH::GetRoot() const
	{
	// The first node is the root
	return NodeList;
	}


//--------------------------------------------------------------------------------------------------
inline const RkBVHTriangle& RkBVH::GetTriangle( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < TriangleCount );
	return TriangleList[ Index ];
	}


//--------------------------------------------------------------------------------------------------
inline const RkVector3& RkBVH::GetVertex( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < VertexCount );
	return VertexList[ Index ];
	}


//--------------------------------------------------------------------------------------------------
inline int RkBVH::GetHeight() const
	{
	const RkBVHNode* Root = GetRoot();
	if ( !Root )
		{
		return 0;
		}

	return Root->GetHeight();
	}





