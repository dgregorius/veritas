//--------------------------------------------------------------------------------------------------
// sphereshape.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "sphereshape.h"
#include "mass.h"


//--------------------------------------------------------------------------------------------------
// RkSphereShape
//--------------------------------------------------------------------------------------------------
RkSphereShape::RkSphereShape( RkBody* Body, const RkSphere& Sphere )
	: RkShape( RK_SPHERE_SHAPE, Body )
	, mSphere( Sphere )
	{

	}


//--------------------------------------------------------------------------------------------------
RkSphere RkSphereShape::GetSphere() const
	{
	return mSphere;
	}


//--------------------------------------------------------------------------------------------------
RkShapeCastResult RkSphereShape::CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, float MaxAlpha ) const
	{
	RkShapeCastResult Result;

	float Alpha = rkIntersectSegmentSphere( RayStart, RayEnd, mSphere.Center, mSphere.Radius );
	if ( 0.0f <= Alpha && Alpha < MaxAlpha )
		{
		RkVector3 HitPoint = rkLerp( RayStart, RayEnd, Alpha );

		Result.HitTime = Alpha;
		Result.HitPoint = HitPoint;
		Result.HitNormal = Alpha > 0.0f ? ( HitPoint - mSphere.Center ) / mSphere.Radius : RK_VEC3_ZERO;
		}

	return Result;
	}


//--------------------------------------------------------------------------------------------------
RkBounds3 RkSphereShape::ComputeBounds( const RkTransform& Transform ) const
	{
	RkVector3 Center = Transform * mSphere.Center;
	RkVector3 Extent = RkVector3( mSphere.Radius );

	return RkBounds3( Center - Extent, Center + Extent );
	}


//--------------------------------------------------------------------------------------------------
RkMassProperties RkSphereShape::ComputeMassProperties() const
	{
	RkVector3 Center = mSphere.Center;
	float Radius = mSphere.Radius;
	
	float Volume = 4.0f / 3.0f * RK_PI * Radius * Radius * Radius;
	float Mass = Volume * mDensity;
	float Ixx = 0.4f * Mass * Radius * Radius;

	RkMassProperties Out;
	Out.Mass = Mass;
	Out.Center = Center;
	Out.Inertia = RkMatrix3( Ixx, Ixx, Ixx ) + rkSteiner( Mass, Center );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
float RkSphereShape::GetMinMotionRadius() const
	{
	return mSphere.Radius;
	}


//--------------------------------------------------------------------------------------------------
float RkSphereShape::GetMaxMotionRadius( const RkVector3& Center ) const
	{
	return rkDistance( Center, mSphere.Center ) + mSphere.Radius;
	}
