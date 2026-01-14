//--------------------------------------------------------------------------------------------------
// dynamictree.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkDynamicTree
//--------------------------------------------------------------------------------------------------
inline int RkDynamicTree::GetProxyCount() const
	{
	return mProxyCount;
	}


//--------------------------------------------------------------------------------------------------
inline RkBounds3 RkDynamicTree::GetProxyBounds( int Proxy ) const
	{
	RK_ASSERT( mNodes[ Proxy ].IsLeaf() );
	return mNodes[ Proxy ].GetBounds();
	}


//--------------------------------------------------------------------------------------------------
inline void* RkDynamicTree::GetProxyUserData( int Proxy ) const
	{
	RK_ASSERT( mNodes[ Proxy ].IsLeaf() );
	return mNodes[ Proxy ].UserData;
	}


//--------------------------------------------------------------------------------------------------
template< typename T > 
float RkDynamicTree::CastRay( const RkVector3& RayStartInput, const RkVector3& RayEndInput, float MaxAlpha, T&& Callback ) const
	{
	if ( mRoot < 0 )
		{
		RK_ASSERT( mRoot == RK_NULL_NODE );
		return MaxAlpha;
		}

	RK_ASSERT( 0.0f < MaxAlpha && MaxAlpha <= 1.0f );
	float BestAlpha = MaxAlpha;
	v32 Lambda = vmSplat( BestAlpha );

	v32 RayStart = vmLoad3( RayStartInput );
	v32 RayDelta = vmLoad3( RayEndInput - RayStartInput );
	v32 RayEnd = RayStart + Lambda * RayDelta;
	
	v32 RayMin = vmMin( RayStart, RayEnd );
	v32 RayMax = vmMax( RayStart, RayEnd );

	int Count = 0;
	int Stack[ RK_STACK_SIZE ];
	int Node = mRoot;

	while ( true )
		{
		// Test node/ray overlap using SAT
		v32 NodeMin = vmLoad3A( mNodes[ Node ].BoundsMin );
		v32 NodeMax = vmLoad3A( mNodes[ Node ].BoundsMax );
		if ( vmTestBoundsOverlap( NodeMin, NodeMax, RayMin, RayMax ) && vmTestBoundsRayOverlap( NodeMin, NodeMax, RayStart, RayDelta ) )
			{
		    // SAT: The node and ray overlap - process leaf node or recurse
			if ( mNodes[ Node ].IsLeaf() )
				{
				// The callback mechanism allows us to implement any-, closest-, and
				// all hit(s) queries in one function. We expect from the client to 
				// return the following for this to work:
				// Any: alpha = 0 (if hit anything)
				// Closest: 0 < alpha < 1
				// All: alpha = 1 (always)
				float Alpha = Callback( mNodes[ Node ].UserData, RayStartInput, RayEndInput, BestAlpha );
				RK_ASSERT( 0 <= Alpha && Alpha <= 1.0f );
				
				if ( Alpha == 0.0f )
					{
					// The client has terminated the ray cast.
					return 0.0f;
					}

				if ( Alpha < BestAlpha )
					{
					BestAlpha = Alpha;

					// Update ray bounds
					Lambda = vmSplat( BestAlpha );
					RayEnd = RayStart + Lambda * RayDelta;
					RayMin = vmMin( RayStart, RayEnd );
					RayMax = vmMax( RayStart, RayEnd );
					}
				}
			else
				{
				// Recurse
				RK_ASSERT( Count <= RK_STACK_SIZE - 1 );
				Stack[ Count++ ] = mNodes[ Node ].Child2;
				Node = mNodes[ Node ].Child1;

				continue;
				}
			}

		if ( Count == 0 )
			{
			break;
			}
		Node = Stack[ --Count ];
		}

	return BestAlpha;
	}


//--------------------------------------------------------------------------------------------------
// template< typename T >
// float RkDynamicTree::CastShape( const RkTransform& ShapeStart, const RkVector3& ShapeEnd, const RkGeometry& ShapeGeometry, const RkBounds3& ShapeBounds, float MaxAlpha, T&& Callback ) const
// 	{
// 	if ( mRoot < 0 )
// 		{
// 		RK_ASSERT( mRoot == RK_NULL_NODE );
// 		return MaxAlpha;
// 		}
// 
// 	RK_ASSERT( 0.0f < MaxAlpha && MaxAlpha <= 1.0f );
// 	float BestAlpha = MaxAlpha;
// 	v32 Lambda = vmSplat( BestAlpha );
// 
// 	RkVector3 BoundsCenter = 0.5f * ( ShapeBounds.Max + ShapeBounds.Min );
// 	RkVector3 BoundsExtent = ShapeBounds.Max - BoundsCenter;
// 
// 	v32 RayStart = vmLoad3( BoundsCenter );
// 	v32 RayDelta = vmLoad3( ShapeEnd - ShapeStart.Translation );
// 	v32 RayEnd = RayStart + Lambda * RayDelta;
// 	v32 RayExtent = vmLoad3( BoundsExtent );
// 
// 	v32 RayMin = vmMin( RayStart, RayEnd );
// 	v32 RayMax = vmMax( RayStart, RayEnd );
// 
// 	int Count = 0;
// 	int Stack[ RK_STACK_SIZE ];
// 	int Node = mRoot;
// 
// 	while ( true )
// 		{
// 		// Test node/ray overlap using SAT
// 		v32 NodeMin = vmLoad3A( mNodes[ Node ].BoundsMin ) - RayExtent;
// 		v32 NodeMax = vmLoad3A( mNodes[ Node ].BoundsMax ) + RayExtent;
// 		if ( vmTestBoundsOverlap( NodeMin, NodeMax, RayMin, RayMax ) && vmTestBoundsRayOverlap( NodeMin, NodeMax, RayStart, RayDelta ) )
// 			{
// 		    // SAT: The node and ray overlap - process leaf node or recurse
// 			if ( mNodes[ Node ].IsLeaf() )
// 				{
// 				// The callback mechanism allows us to implement any-, closest-, and
// 				// all hit(s) queries in one function. We expect from the client to 
// 				// return the following for this to work:
// 				// Any: alpha = 0 (if hit anything)
// 				// Closest: 0 < alpha < 1
// 				// All: alpha = 1 (always)
// 				float Alpha = Callback( mNodes[ Node ].UserData, ShapeStart, ShapeEnd, ShapeGeometry, ShapeBounds, BestAlpha );
// 				RK_ASSERT( 0 <= Alpha && Alpha <= 1.0f );
// 
// 				if ( Alpha == 0.0f )
// 					{
// 					// The client has terminated the sphere cast.
// 					return 0.0f;
// 					}
// 
// 				if ( Alpha < BestAlpha )
// 					{
// 					BestAlpha = Alpha;
// 
// 					// Update ray bounds
// 					Lambda = vmSplat( BestAlpha );
// 					RayEnd = RayStart + Lambda * RayDelta;
// 					RayMin = vmMin( RayStart, RayEnd );
// 					RayMax = vmMax( RayStart, RayEnd );
// 					}
// 				}
// 			else
// 				{
// 				// Recurse
// 				RK_ASSERT( Count <= RK_STACK_SIZE - 1 );
// 				Stack[ Count++ ] = mNodes[ Node ].Child2;
// 				Node = mNodes[ Node ].Child1;
// 
// 				continue;
// 				}
// 			}
// 
// 		if ( Count == 0 )
// 			{
// 			break;
// 			}
// 		Node = Stack[ --Count ];
// 		}
// 
// 	return BestAlpha;
// 	}


//--------------------------------------------------------------------------------------------------
template< typename T >
void RkDynamicTree::Query( const RkBounds3& Bounds, T&& Callback ) const
	{
	if ( mRoot < 0 )
		{
		RK_ASSERT( mRoot == RK_NULL_NODE );
		return;
		}

	v32 BoundsMin = vmLoad3( Bounds.Min );
	v32 BoundsMax = vmLoad3( Bounds.Max );

	int Count = 0;
	int Stack[ RK_STACK_SIZE ];
	int Node = mRoot;

	while ( true )
		{
		v32 NodeMin = vmLoad3A( mNodes[ Node ].BoundsMin );
		v32 NodeMax = vmLoad3A( mNodes[ Node ].BoundsMax );
		if ( vmTestBoundsOverlap( NodeMin, NodeMax, BoundsMin, BoundsMax ) )
			{
			if ( mNodes[ Node ].IsLeaf() )
				{
				Callback( mNodes[ Node ].UserData );
				}
			else
				{
				// Recurse
				RK_ASSERT( Count <= RK_STACK_SIZE - 1 );
				Stack[ Count++ ] = mNodes[ Node ].Child2;
				Node = mNodes[ Node ].Child1;

				continue;
				}
			}

		if ( Count == 0 )
			{
			break;
			}
		Node = Stack[ --Count ];
		}
	}


//--------------------------------------------------------------------------------------------------
inline void RkDynamicTree::Query( RkArray< int >& Out, const RkBounds3& Bounds ) const
	{
	if ( mRoot < 0 )
		{
		RK_ASSERT( mRoot == RK_NULL_NODE );
		return;
		}

	v32 BoundsMin = vmLoad3( Bounds.Min );
	v32 BoundsMax = vmLoad3( Bounds.Max );

	int Count = 0;
	int Stack[ RK_STACK_SIZE ];
	int Node = mRoot;

	while ( true )
		{
		v32 NodeMin = vmLoad3A( mNodes[ Node ].BoundsMin );
		v32 NodeMax = vmLoad3A( mNodes[ Node ].BoundsMax );
		if ( vmTestBoundsOverlap( NodeMin, NodeMax, BoundsMin, BoundsMax ) )
			{
			if ( mNodes[ Node ].IsLeaf() )
				{
				Out.PushBack( Node );
				}
			else
				{
				// Recurse
				RK_ASSERT( Count <= RK_STACK_SIZE - 1 );
				Stack[ Count++ ] = mNodes[ Node ].Child2;
				Node = mNodes[ Node ].Child1;

				continue;
				}
			}

		if ( Count == 0 )
			{
			break;
			}
		Node = Stack[ --Count ];
		}
	}