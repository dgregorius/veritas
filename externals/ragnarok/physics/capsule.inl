//--------------------------------------------------------------------------------------------------
// capsule.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkCapsule
//--------------------------------------------------------------------------------------------------
inline RkCapsule::RkCapsule( const RkVector3& Center1, const RkVector3& Center2, float Radius )
	: Center1( Center1 )
	, Center2( Center2 )
	, Radius( Radius )
	{

	}


//--------------------------------------------------------------------------------------------------
inline RkBounds3 RkCapsule::ComputeBounds() const
	{
	RkVector3 Extent( Radius, Radius, Radius );
	RkBounds3 Bounds1( Center1 - Extent, Center1 + Extent );
	RkBounds3 Bounds2( Center2 - Extent, Center2 + Extent );

	return Bounds1 + Bounds2;
	}


//--------------------------------------------------------------------------------------------------
inline RkBounds3 RkCapsule::ComputeBounds( const RkTransform& Transform ) const
	{
	RkVector3 AbsoluteCenter1 = Transform * Center1;
	RkVector3 AbsoluteCenter2 = Transform * Center2;
	RkVector3 Extent( Radius, Radius, Radius );

	RkBounds3 Bounds1( AbsoluteCenter1 - Extent, AbsoluteCenter1 + Extent );
	RkBounds3 Bounds2( AbsoluteCenter2 - Extent, AbsoluteCenter2 + Extent );

	return Bounds1 + Bounds2;
	}


//--------------------------------------------------------------------------------------------------
inline int RkCapsule::FindSupportVertex( const RkVector3& Direction ) const
	{
	return rkDot( Center1, Direction ) > rkDot( Center2, Direction ) ? 0 : 1;
	}


//--------------------------------------------------------------------------------------------------
inline RkVector3 RkCapsule::GetCenter( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < 2 );
	return Index == 0 ? Center1 : Center2;
	}