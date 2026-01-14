//--------------------------------------------------------------------------------------------------
// contact.inl	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkContactCache
//--------------------------------------------------------------------------------------------------
inline void rkClearCache( RkContactCache& Cache )
	{
	rkClearCache( Cache.SAT );
	rkClearCache( Cache.GJK );
	}


//--------------------------------------------------------------------------------------------------
// RkContact
//--------------------------------------------------------------------------------------------------
inline RkContactType RkContact::GetType() const
	{
	return mType;
	}


//--------------------------------------------------------------------------------------------------
inline RkWorld* RkContact::GetWorld() const
	{
	return mWorld;
	}


//--------------------------------------------------------------------------------------------------
inline RkBody* RkContact::GetBody1() const
	{
	return mBody1;
	}


//--------------------------------------------------------------------------------------------------
inline RkBody* RkContact::GetBody2() const
	{
	return mBody2;
	}


//--------------------------------------------------------------------------------------------------
inline RkBody* RkContact::GetOtherBody( RkBody* Body ) const
	{
	if ( Body == mBody1 )
		{
		return mBody2;
		}

	if ( Body == mBody2 )
		{
		return mBody1;
		}

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
inline RkShape* RkContact::GetShape1() const
	{
	return mShape1;
	}


//--------------------------------------------------------------------------------------------------
inline RkShape* RkContact::GetShape2() const
	{
	return mShape2;
	}


//--------------------------------------------------------------------------------------------------
inline RkShape* RkContact::GetOtherShape( RkShape* Shape ) const
	{
	if ( Shape == mShape1 )
		{
		return mShape2;
		}

	if ( Shape == mShape2 )
		{
		return mShape1;
		}

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
inline int RkContact::GetManifoldCount() const
	{
	return mManifoldCount;
	}


//--------------------------------------------------------------------------------------------------
inline RkManifold& RkContact::GetManifold( int ManifoldIndex )
	{
	RK_ASSERT( 0 <= ManifoldIndex && ManifoldIndex < mManifoldCount );
	return mManifolds[ ManifoldIndex ];
	}


//--------------------------------------------------------------------------------------------------
inline const RkManifold& RkContact::GetManifold( int ManifoldIndex ) const
	{
	RK_ASSERT( 0 <= ManifoldIndex && ManifoldIndex < mManifoldCount );
	return mManifolds[ ManifoldIndex ];
	}