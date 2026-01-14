//--------------------------------------------------------------------------------------------------
// broadphase.inl	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkBroadphase
//--------------------------------------------------------------------------------------------------
template< typename T >
void RkBroadphase::CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, T&& Callback ) const
	{
	float MaxAlpha = 1.0f;
	for ( int ProxyType = 0; ProxyType < RK_BODY_TYPE_COUNT; ++ProxyType )
		{
		MaxAlpha = mTrees[ ProxyType ].CastRay( RayStart, RayEnd, MaxAlpha, std::forward< T >( Callback ) );
		if ( MaxAlpha == 0.0f )
			{
			break;
			}
		}
	}


//--------------------------------------------------------------------------------------------------
template< typename T >
void RkBroadphase::Query( const RkBounds3& Bounds, T&& Callback ) const
	{
	for ( int ProxyType = 0; ProxyType < RK_BODY_TYPE_COUNT; ++ProxyType )
		{
		mTrees[ ProxyType ].Query( Bounds, std::forward< T >( Callback ) );
		}
	}


//--------------------------------------------------------------------------------------------------
template< typename T >
void RkBroadphase::Update( RkArray< RkProxyPair >& PairBuffer, T&& Callback )
	{
	// Perform tree queries for all moving objects
	if ( !mMoveBuffer.Empty() )
		{
		RkStackArray< RkArray< RkProxyPair >, 32 > LocalPairs( static_cast< int >( mExecutor->num_workers() ) );

		tf::Taskflow Taskflow;
		Taskflow.for_each( mMoveBuffer.Begin(), mMoveBuffer.End(), [ & ]( RkProxy QueryProxy )
			{
			RK_TRACY_ZONE_LVL3( "Update Pairs Task", Tracy_UpdatePairsTask );

			// Proxies might have been deleted
			if ( QueryProxy == RK_NULL_PROXY )
				{
				return;
				}

			int WorkerIndex = mExecutor->this_worker_id();
			RkArray < RkProxyPair >& LocalBuffer = LocalPairs[ WorkerIndex ];
			RkBounds3 QueryBounds = mTrees[ QueryProxy.Type ].GetProxyBounds( QueryProxy.Index );
			for ( int ProxyType = 0; ProxyType < RK_BODY_TYPE_COUNT; ++ProxyType )
				{
				RkArray< int > TreeProxies;
				mTrees[ ProxyType ].Query( TreeProxies, QueryBounds );

				for ( int ProxyIndex : TreeProxies )
					{
					RkProxy TreeProxy;
					TreeProxy.Type = ProxyType;
					TreeProxy.Index = ProxyIndex;

					// A proxy cannot build a pair with itself
					if ( TreeProxy == QueryProxy )
						{
						continue;
						}

					// Avoid duplicates
					if ( mMoveSet.Contains( TreeProxy ) && TreeProxy > QueryProxy )
						{
						continue;
						}

					// Filter
					if ( !Callback( QueryProxy, TreeProxy ) )
						{
						continue;
						}

					// Create a new pair
					int Index = LocalBuffer.PushBack();
					LocalBuffer[ Index ].Proxy1 = QueryProxy;
					LocalBuffer[ Index ].Proxy2 = TreeProxy;
					}
				}
			} );

		mExecutor->run( Taskflow ).wait();
	
		// Consolidate local results
		for ( const RkArray< RkProxyPair >& LocalBuffer : LocalPairs )
			{
			PairBuffer.PushBack( LocalBuffer.Begin(), LocalBuffer.End() );
			}
		}

	// Clear move buffer
	mMoveBuffer.Clear();
	mMoveSet.Clear();
	}