//--------------------------------------------------------------------------------------------------
// broadphase.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "broadphase.h"
#include "body.h"
#include "shape.h"


//--------------------------------------------------------------------------------------------------
// RkBroadphase
//--------------------------------------------------------------------------------------------------
RkBroadphase::RkBroadphase( RkWorld* World, tf::Executor* Executor )
	{
	RK_ASSERT( World );
	mWorld = World;
	RK_ASSERT( Executor );
	mExecutor = Executor;
	}


//--------------------------------------------------------------------------------------------------
RkWorld* RkBroadphase::GetWorld() const
	{
	return mWorld;
	}


//--------------------------------------------------------------------------------------------------
int RkBroadphase::GetTreeCount() const 
	{
	return RK_COUNT_OF( mTrees );
	}


//--------------------------------------------------------------------------------------------------
const RkDynamicTree* RkBroadphase::GetTree( int TreeIndex ) const
	{
	return 0 <= TreeIndex && TreeIndex < RK_COUNT_OF( mTrees ) ? mTrees + TreeIndex : nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkProxy RkBroadphase::CreateProxy( const RkBounds3& Bounds, RkShape* Shape )
	{
	if ( !Shape )
		{
		return RK_NULL_PROXY;
		}

	RkBody* Body = Shape->GetBody();
	RkBodyType ProxyType = Body->GetType();

	RkProxy Proxy;
	Proxy.Type = ProxyType;
	Proxy.Index = mTrees[ ProxyType ].CreateProxy( Bounds, Shape );

	// DIRK_TODO: We *potentially* want to skip static bodies here since the makes the initial scene load expensive
	BufferMove( Proxy );

	return Proxy;
	}


//--------------------------------------------------------------------------------------------------
void RkBroadphase::MoveProxy( RkProxy Proxy, const RkBounds3& Bounds )
	{
	RK_ASSERT( Proxy != RK_NULL_PROXY );
	mTrees[ Proxy.Type ].MoveProxy( Proxy.Index, Bounds );
	BufferMove( Proxy );
	}


//--------------------------------------------------------------------------------------------------
void RkBroadphase::RefitProxy( RkProxy Proxy, const RkBounds3& Bounds )
	{
	RK_ASSERT( Proxy != RK_NULL_PROXY );
	mTrees[ Proxy.Type ].RefitProxy( Proxy.Index, Bounds );
	BufferMove( Proxy );
	}


//--------------------------------------------------------------------------------------------------
void RkBroadphase::TouchProxy( RkProxy Proxy )
	{
	if ( Proxy == RK_NULL_PROXY )
		{
		return;
		}

	BufferMove( Proxy );
	}


//--------------------------------------------------------------------------------------------------
RkShape* RkBroadphase::DestroyProxy( RkProxy Proxy )
	{
	if ( Proxy == RK_NULL_PROXY )
		{
		return nullptr;
		}

	UnbufferMove( Proxy );

	return static_cast< RkShape* >( mTrees[ Proxy.Type ].DestroyProxy( Proxy.Index ) );
	}


//--------------------------------------------------------------------------------------------------
int RkBroadphase::GetProxyCount() const
	{
	int ProxyCount = 0;
	ProxyCount += mTrees[ RK_STATIC_BODY ].GetProxyCount();
	ProxyCount += mTrees[ RK_KEYFRAMED_BODY ].GetProxyCount();
	ProxyCount += mTrees[ RK_DYNAMIC_BODY ].GetProxyCount();

	return ProxyCount;
	}


//--------------------------------------------------------------------------------------------------
RkShape* RkBroadphase::GetProxyShape( RkProxy Proxy ) const
	{
	if ( Proxy == RK_NULL_PROXY )
		{
		return nullptr;
		}

	return static_cast< RkShape* >( mTrees[ Proxy.Type ].GetProxyUserData( Proxy.Index ) );
	}


//--------------------------------------------------------------------------------------------------
RkBounds3 RkBroadphase::GetProxyBounds( RkProxy Proxy ) const
	{
	if ( Proxy == RK_NULL_PROXY )
		{
		return RK_BOUNDS3_EMPTY;
		}

	return mTrees[ Proxy.Type ].GetProxyBounds( Proxy.Index );
	}


//--------------------------------------------------------------------------------------------------
bool RkBroadphase::IsDirty() const
	{
	RK_ASSERT( !mTrees[ RK_STATIC_BODY ].IsDirty() );
	return mTrees[ RK_KEYFRAMED_BODY ].IsDirty() || mTrees[ RK_DYNAMIC_BODY ].IsDirty();
	}


//--------------------------------------------------------------------------------------------------
void RkBroadphase::Rebuild()
	{
	RK_ASSERT( !mTrees[ RK_STATIC_BODY ].IsDirty() );
	mTrees[ RK_KEYFRAMED_BODY ].Rebuild();
	mTrees[ RK_DYNAMIC_BODY ].Rebuild();
	}


//--------------------------------------------------------------------------------------------------
int RkBroadphase::GetMoveCount() const
	{
	return mMoveBuffer.Size();
	}


//--------------------------------------------------------------------------------------------------
const RkProxyBuffer& RkBroadphase::GetMoveBuffer() const
	{
	return mMoveBuffer;
	}


//--------------------------------------------------------------------------------------------------
void RkBroadphase::BufferMove( RkProxy Proxy )
	{
	RK_ASSERT( Proxy != RK_NULL_PROXY );

	if ( !mMoveSet.Contains( Proxy ) )
		{
		mMoveSet.Insert( Proxy );
		RK_ASSERT( !mMoveBuffer.Contains( Proxy ) );
		mMoveBuffer.PushBack( Proxy );
		}
	}


//--------------------------------------------------------------------------------------------------
void RkBroadphase::UnbufferMove( RkProxy Proxy )
	{
	RK_ASSERT( Proxy != RK_NULL_PROXY );
	
	if ( mMoveSet.Remove( Proxy ) )
		{
		RK_ASSERT( mMoveBuffer.Contains( Proxy ) );
		for ( int Index = 0; Index < mMoveBuffer.Size(); ++Index )
			{
			if ( mMoveBuffer[ Index ] == Proxy )
				{
				mMoveBuffer[ Index ] = RK_NULL_PROXY;
				}
			}
		}
	}