//--------------------------------------------------------------------------------------------------
// capsuleshape.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "capsuleshape.h"
#include "mass.h"


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static inline RkShapeCastResult rkCastRayAgainstCap( const RkVector3& P, const RkVector3& Q, const RkVector3& C, float R, float MaxAlpha )
	{
	RkShapeCastResult Result;

	float Alpha = rkIntersectSegmentSphere( P, Q, C, R );
	if ( 0.0f <= Alpha && Alpha < MaxAlpha )
		{
		Result.HitTime = Alpha;
		Result.HitPoint = rkLerp( P, Q, Alpha );
		Result.HitNormal = ( Result.HitPoint - C ) / R;
		}

	return Result;
	}


//--------------------------------------------------------------------------------------------------
static inline RkShapeCastResult rkTransformResult( const RkShapeCastResult& Result, const RkQuaternion& Rotation, const RkVector3& Translation )
	{
	RkShapeCastResult Out;
	Out.HitTime = Result.HitTime;
	Out.HitPoint = Rotation * Result.HitPoint + Translation;
	Out.HitNormal = Rotation * Result.HitNormal;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
// RkCapsuleShape
//--------------------------------------------------------------------------------------------------
RkCapsuleShape::RkCapsuleShape( RkBody* Body, const RkCapsule& Capsule )
	: RkShape( RK_CAPSULE_SHAPE, Body )
	, mCapsule( Capsule )
	{

	}


//--------------------------------------------------------------------------------------------------
RkCapsule RkCapsuleShape::GetCapsule() const
	{
	return mCapsule;
	}


//--------------------------------------------------------------------------------------------------
RkShapeCastResult RkCapsuleShape::CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, float MaxAlpha ) const
	{
	// Initialize result structure
	RkShapeCastResult Result;

	// Compute height and handle degenerate capsules
	float Height = rkDistance( mCapsule.Center1, mCapsule.Center2 );
	if ( Height < 1000.0f * RK_F32_MIN )
		{
		RkVector3 Center = 0.5f * ( mCapsule.Center1 + mCapsule.Center2 );
		
		// DIRK_TODO: Handle start in solid properly
		float Alpha = rkIntersectSegmentSphere( RayStart, RayEnd, Center, mCapsule.Radius);
		if ( 0.0f <= Alpha && Alpha < MaxAlpha )
			{
			RkVector3 HitPoint = ( 1.0f - Alpha ) * RayStart + Alpha * RayEnd;

			Result.HitTime = Alpha;
			Result.HitPoint = HitPoint;
			Result.HitNormal = Alpha > 0.0f ? ( HitPoint - Center ) / mCapsule.Radius : RK_VEC3_ZERO;
			}

		return Result;
		}

	// Transform ray and capsule into local space capsule space
	RkQuaternion Rotation = rkShortestArc( RK_VEC3_AXIS_Y, ( mCapsule.Center2 - mCapsule.Center1 ) / Height );
	RkVector3 Translation = mCapsule.Center1;

	RkVector3 P = rkTMul( Rotation, RayStart - Translation );
	RkVector3 Q = rkTMul( Rotation, RayEnd - Translation );
	RkVector3 PQ = Q - P;

	RkVector3 A( 0.0f, 0.0f, 0.0f );
	RkVector3 B( 0.0f, Height, 0.0f );
	RkVector3 AB = B - A;

	float K1 = PQ.X * PQ.X + PQ.Z * PQ.Z;
	float K3 = P.X * P.X + P.Z * P.Z - rkSquare( mCapsule.Radius );

	// Parallel case
	if ( K1 < 1000.0f * RK_F32_MIN )
		{
		if ( K3 > 0.0f )
			{
			// Parallel and outside
			Result.HitTime = 1.0f;
			return Result;
			}

		if ( 0.0f <= P.Y && P.Y <= Height )
			{
			// Parallel and inside
			Result.HitTime = 0.0f;
			Result.HitPoint = RayStart;
			return Result;
			}

		// Below cylinder and casting upwards
		if ( P.Y < 0.0f && PQ.Y > 0.0f )
			{
			Result = rkCastRayAgainstCap( P, Q, A, mCapsule.Radius, MaxAlpha );
			return rkTransformResult( Result, Rotation, Translation );
			}

		// Above cylinder and casting downwards
		if ( P.Y > Height && PQ.Y < 0.0f )
			{
			Result = rkCastRayAgainstCap( P, Q, B, mCapsule.Radius, MaxAlpha );
			return rkTransformResult( Result, Rotation, Translation );
			}

		// Above or below and casting away from cylinder
		Result.HitTime = 1.0f;
		return Result;
		}

	// Non-parallel case
	float K2 = PQ.X * P.X + PQ.Z * P.Z;

	float Discriminant = K2 * K2 - K1 * K3;
	if ( Discriminant < 0.0f )
		{
		// No real roots - no intersection
		Result.HitTime = 1.0f;
		return Result;
		}

	float T = ( -K2 - rkSqrt( Discriminant ) ) / K1;
	if ( T > MaxAlpha )
		{
		// Segment approaching cylinder, but not quite getting there.
		Result.HitTime = 1.0f;
		return Result;
		}

	// Don't skip t < 0. This means that we start in the *infinite* cylinder and still might hit a cap
	RkVector3 C = P + T * PQ;
	RkVector3 AC = C - A;

	float S = rkDot( AC, AB ) / rkSquare( Height );
	if ( S < 0.0f )
		{
		// X projects outside A, run test through sphere at A
		Result = rkCastRayAgainstCap( P, Q, A, mCapsule.Radius, MaxAlpha );
		return rkTransformResult( Result, Rotation, Translation );
		}

	if ( S > 1.0f )
		{
		// X projects outside B, run test through sphere at B
		Result = rkCastRayAgainstCap( P, Q, B, mCapsule.Radius, MaxAlpha );
		return rkTransformResult( Result, Rotation, Translation );
		}

	// Now handle t < 0
	if ( T < 0 )
		{
		// Rays starts inside
		Result.HitTime = 0.0f;
		Result.HitPoint = RayStart;
		return Result;
		}

	// Ray hits cylinder inside segment AB
	Result.HitTime = T;
	Result.HitPoint = C;
	Result.HitNormal = RkVector3( C.X, 0.0f, C.Z ) / mCapsule.Radius;

	return rkTransformResult( Result, Rotation, Translation );
	
	}


//--------------------------------------------------------------------------------------------------
RkBounds3 RkCapsuleShape::ComputeBounds( const RkTransform& Transform ) const
	{
	RkVector3 Center1 = Transform * mCapsule.Center1;
	RkVector3 Center2 = Transform * mCapsule.Center2;
	RkVector3 Extent( mCapsule.Radius );

	return RkBounds3( Center1 - Extent, Center1 + Extent ) + RkBounds3( Center2 - Extent, Center2 + Extent );
	}


//--------------------------------------------------------------------------------------------------
RkMassProperties RkCapsuleShape::ComputeMassProperties() const
	{
	RkVector3 Center1 = mCapsule.Center1;
	RkVector3 Center2 = mCapsule.Center2;
	float Radius = mCapsule.Radius;

	// Cylinder
	float CylinderHeight = rkDistance( Center1, Center2 );
	float CylinderVolume = RK_PI * Radius * Radius * CylinderHeight;
	float CylinderMass = CylinderVolume * mDensity;

	// Sphere
	float SphereVolume = ( 4.0f / 3.0f ) * RK_PI * Radius * Radius * Radius;
	float SphereMass = SphereVolume * mDensity;

	// Local accumulated inertia
	RkMatrix3 Inertia = rkCylinderInertia( CylinderMass, Radius, CylinderHeight ) + rkSphereInertia( SphereMass, Radius );

	float Steiner = 0.125f * SphereMass * ( 3.0f * Radius + 2.0f * CylinderHeight ) * CylinderHeight;
	Inertia.A11 += Steiner;
	Inertia.A33 += Steiner;

	// Align capsule axis with chosen up-axis
	RkMatrix3 R = RK_MAT3_IDENTITY;
	if ( CylinderHeight * CylinderHeight > 1000.0f * RK_F32_MIN )
		{
		RkVector3 Direction = rkNormalize( Center2 - Center1 );
		RkQuaternion Q = rkShortestArc( RK_VEC3_AXIS_Y, Direction );
		R = RkMatrix3( Q );
		}

	// Parallel Axis Theorem
	float Mass = SphereMass + CylinderMass;
	RkVector3 Center = 0.5f * ( Center1 + Center2 );

	RkMassProperties Out;
	Out.Mass = Mass;
	Out.Center = Center;
	Out.Inertia = R * Inertia * rkTranspose( R ) + rkSteiner( Mass, Center );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
float RkCapsuleShape::GetMinMotionRadius() const
	{
	return mCapsule.Radius;
	}


//--------------------------------------------------------------------------------------------------
float RkCapsuleShape::GetMaxMotionRadius( const RkVector3& Center ) const
	{
	float DistanceSq1 = rkDistanceSq( Center, mCapsule.Center1 );
	float DistanceSq2 = rkDistanceSq( Center, mCapsule.Center2 );

	return rkSqrt( rkMax( DistanceSq1, DistanceSq2 ) ) + mCapsule.Radius;
	}
