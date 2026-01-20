//--------------------------------------------------------------------------------------------------
// shape.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "shape.h"
#include "body.h"
#include "broadphase.h"
#include "constants.h"


//--------------------------------------------------------------------------------------------------
// RkShape
//--------------------------------------------------------------------------------------------------
RkShape::RkShape( RkShapeType Type, RkBody* Body )
	{
	mType = Type;

	RK_ASSERT( Body );
	mBody = Body;
	
	mFilter = RK_DEFAULT_FILTER;
	mDensity = 1000.0f;
	mFriction = 0.6f;
	mRestitution = 0.0f;

	mProxy = RK_NULL_PROXY;
	mProxyBounds = RK_BOUNDS3_EMPTY;
	mDirty = false;

	mUserColor = RK_COLOR_TRANSPARENT;
	mUserData = nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkShapeType RkShape::GetType() const
	{
	return mType;
	}


//--------------------------------------------------------------------------------------------------
RkBody* RkShape::GetBody() const
	{
	return mBody;
	}


//--------------------------------------------------------------------------------------------------
RkFilter RkShape::GetFilter() const
	{
	return mFilter;
	}


//--------------------------------------------------------------------------------------------------
void RkShape::SetFilter( RkFilter Filter )
	{
	if ( mFilter != Filter )
		{
		mFilter = Filter;
		
		// DIRK_TODO: Re-filter contact
		//RK_ASSERT( 0 );
		}
	}


//--------------------------------------------------------------------------------------------------
float RkShape::GetDensity() const
	{
	return mDensity;
	}


//--------------------------------------------------------------------------------------------------
void RkShape::SetDensity( float Density )
	{
	if ( mDensity != Density )
		{
		mDensity = Density;
		mBody->RebuildMass();
		}
	}


//--------------------------------------------------------------------------------------------------
float RkShape::GetFriction() const
	{
	return mFriction;
	}


//--------------------------------------------------------------------------------------------------
void RkShape::SetFriction( float Friction )
	{
	RK_ASSERT( Friction >= 0.0f );
	mFriction = Friction;
	}


//--------------------------------------------------------------------------------------------------
float RkShape::GetRestitution() const
	{
	return mRestitution;
	}


//--------------------------------------------------------------------------------------------------
void RkShape::SetRestitution( float Restitution )
	{
	RK_ASSERT( Restitution >= 0.0f );
	mRestitution = Restitution;
	}


//--------------------------------------------------------------------------------------------------
RkColor RkShape::GetUserColor() const
	{
	return mUserColor;
	}


//--------------------------------------------------------------------------------------------------
void RkShape::SetUserColor( const RkColor& UserColor ) const
	{
	mUserColor = UserColor;
	}


//--------------------------------------------------------------------------------------------------
void* RkShape::GetUserData() const
	{
	return mUserData;
	}


//--------------------------------------------------------------------------------------------------
void RkShape::SetUserData( void* UserData ) const
	{
	mUserData = UserData;
	}


//--------------------------------------------------------------------------------------------------
RkProxy RkShape::GetProxy() const
	{
	return mProxy;
	}


//--------------------------------------------------------------------------------------------------
RkBounds3 RkShape::GetProxyBounds() const
	{
	return mProxyBounds;
	}


//--------------------------------------------------------------------------------------------------
void RkShape::Insert( RkBroadphase* Broadphase, const RkTransform& Transform )
	{
	RK_ASSERT( mProxy == RK_NULL_PROXY );

	mProxyBounds = ComputeBounds( Transform );
	if ( mBody->GetType() != RK_STATIC_BODY )
		{
		mProxyBounds = rkInflate( mProxyBounds, RK_BOUNDS_EXTENSION );
		}

	mProxy = Broadphase->CreateProxy( mProxyBounds, this );
	}


//--------------------------------------------------------------------------------------------------
void RkShape::Move( RkBroadphase* Broadphase, const RkTransform& Transform )
	{
	RK_ASSERT( mProxy.Type == static_cast< uint32 >( mBody->GetType() ) );

	RkBounds3 Bounds = ComputeBounds( Transform );
	if ( ( mProxyBounds.Contains( Bounds ) ) )
		{
		return;
		}

	mProxyBounds = Bounds;
	if ( mBody->GetType() != RK_STATIC_BODY )
		{
		mProxyBounds = rkInflate( mProxyBounds, RK_BOUNDS_EXTENSION );
		}

	Broadphase->MoveProxy( mProxy, mProxyBounds );
	}


//--------------------------------------------------------------------------------------------------
void RkShape::Remove( RkBroadphase* Broadphase )
	{
	RK_ASSERT( mProxy.Type == static_cast< uint32 >( mBody->GetType() ) );

	Broadphase->DestroyProxy( mProxy );

	mProxy = RK_NULL_PROXY;
	mProxyBounds = RK_BOUNDS3_EMPTY;
	}


//--------------------------------------------------------------------------------------------------
bool RkShape::Update( const RkTransform& Transform )
	{
	RK_ASSERT( mProxy.Type == static_cast< uint32 >( mBody->GetType() ) );
	
	RK_ASSERT( !mDirty );
	RkBounds3 Bounds = ComputeBounds( Transform );
	if ( ( mProxyBounds.Contains( Bounds ) ) )
		{
		return false;
		}

	mDirty = true;
	mProxyBounds = Bounds;
	if ( mBody->GetType() != RK_STATIC_BODY )
		{
		mProxyBounds = rkInflate( mProxyBounds, RK_BOUNDS_EXTENSION );
		}
	RK_ASSERT( !rkContains( mBody->GetWorld()->GetBroadphase()->GetProxyBounds( mProxy ), mProxyBounds ) );
	
	return true;
	}


//--------------------------------------------------------------------------------------------------
bool RkShape::IsDirty() const
	{
	return mDirty;
	}


//--------------------------------------------------------------------------------------------------
void RkShape::SetDirty( bool Dirty )
	{
	mDirty = Dirty;
	}


//--------------------------------------------------------------------------------------------------
// RkShapePair
//--------------------------------------------------------------------------------------------------
static inline uint64_t rkHashInteger( uint64_t Key )
	{
	Key ^= Key >> 23;
	Key *= 0x2127599bf4325c37ull;
	Key ^= Key >> 47;
	return Key;
	}


//--------------------------------------------------------------------------------------------------
static inline void rkHashMul( uint64_t* Hash1, uint64_t* Hash2 )
	{
	*Hash1 = _umul128( *Hash1, *Hash2, Hash2 );
	}


//--------------------------------------------------------------------------------------------------
static inline uint64_t rkHashMix( uint64_t Hash1, uint64_t Hash2 )
	{
	rkHashMul( &Hash1, &Hash2 );
	return Hash1 ^ Hash2;
	}


//--------------------------------------------------------------------------------------------------
uint64_t rkHashPair( const RkShapePair& ShapePair )
	{
	RK_ASSERT( ShapePair.Shape1 );
	uint64 Hash1 = rkHashInteger( (uint64_t)ShapePair.Shape1 );
	RK_ASSERT( ShapePair.Shape2 );
	uint64 Hash2 = rkHashInteger( (uint64_t)ShapePair.Shape2 );

	return rkHashMix( Hash1, Hash2 );
	}


//--------------------------------------------------------------------------------------------------
bool rkCmprPair( const RkShapePair& Lhs, const RkShapePair& Rhs )
	{
	return Lhs.Shape1 == Rhs.Shape1 && Lhs.Shape2 == Rhs.Shape2;
	}