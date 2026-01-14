//--------------------------------------------------------------------------------------------------
// sphere.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkSphere
//--------------------------------------------------------------------------------------------------
inline RkSphere::RkSphere( const RkVector3& Center, float Radius )
	: Center( Center )
	, Radius( Radius )
	{

	}


//--------------------------------------------------------------------------------------------------
inline RkBounds3 RkSphere::ComputeBounds() const
	{
	RkVector3 Extent( Radius, Radius, Radius );
	return RkBounds3( Center - Extent, Center + Extent );
	}


//--------------------------------------------------------------------------------------------------
inline RkBounds3 RkSphere::ComputeBounds( const RkTransform& Transform ) const
	{
	RkVector3 AbsoluteCenter = Transform * Center;
	RkVector3 Extent( Radius, Radius, Radius );

	return RkBounds3( AbsoluteCenter - Extent, AbsoluteCenter + Extent );
	}