//--------------------------------------------------------------------------------------------------
// hullshape.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "hullshape.h"
#include "constants.h"
#include "mass.h"


//--------------------------------------------------------------------------------------------------
// RkHullShape
//--------------------------------------------------------------------------------------------------
RkHullShape::RkHullShape( RkBody* Body, RkHull* Hull )
	: RkShape( RK_HULL_SHAPE, Body )
	{
	RK_ASSERT( Hull );
	mHull = Hull;
	}


//--------------------------------------------------------------------------------------------------
RkHull* RkHullShape::GetHull() const
	{
	return mHull;
	}


//--------------------------------------------------------------------------------------------------
RkShapeCastResult RkHullShape::CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, float MaxAlpha ) const
	{
	RkShapeCastResult Result;

	float MinT = 0.0f;
	float MaxT = MaxAlpha;
	int BestFace = -1;

	RkVector3 RayDelta = RayEnd - RayStart;
	for ( int Face = 0; Face < mHull->FaceCount; ++Face )
		{
		RkPlane3 Plane = mHull->GetPlane( Face );

		float Distance = Plane.Offset - rkDot( Plane.Normal, RayStart );
		float Denominator = rkDot( Plane.Normal, RayDelta );

		if ( Denominator == 0.0f )
			{
			if ( Distance < 0 )
				{
				// Parallel and runs outside plane
				return Result;
				}
			}
		else
			{
			float T = Distance / Denominator;

			if ( Denominator < 0.0f )
				{
				if ( T > MinT )
					{
					BestFace = Face;
					MinT = T;
					}
				}
			else
				{
				if ( T < MaxT )
					{
					MaxT = T;
					}
				}

			if ( MinT > MaxT )
				{
				return Result;
				}
			}
		}

	if ( BestFace >= 0 )
		{
		Result.HitPoint = RayStart + MinT * RayDelta;
		Result.HitNormal = mHull->PlaneList[ BestFace ].Normal;
		Result.HitTime = MinT;
		}
	else
		{
		Result.HitPoint = RayStart;
		Result.HitTime = 0.0f;
		}

	return Result;
	}


//--------------------------------------------------------------------------------------------------
RkBounds3 RkHullShape::ComputeBounds( const RkTransform& Transform ) const
	{
	// The shape bounds must contain the convex radius in the broadphase for contact generation!
	RkBounds3 Bounds = rkInflate( mHull->Bounds, RK_CONVEX_RADIUS );
	return Transform * Bounds;
	}


//--------------------------------------------------------------------------------------------------
RkMassProperties RkHullShape::ComputeMassProperties() const
	{
	RkMassProperties Out;
	Out.Mass = mDensity * mHull->Mass; 
	Out.Center = mHull->Center;
	Out.Inertia = mDensity * mHull->Inertia + rkSteiner( Out.Mass, Out.Center );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
float RkHullShape::GetMinMotionRadius() const
	{
	RK_ASSERT( mHull );

	float MinMotionRadius = RK_F32_MAX;
	for ( int Index = 0; Index < mHull->FaceCount; ++Index )
		{
		RkPlane3 Plane = mHull->GetPlane( Index );
		float Distance = rkDistance( Plane, mHull->Center );
		RK_ASSERT( Distance < 0.0f );

		MinMotionRadius = rkMin( MinMotionRadius, -Distance );
		}

	// DIRK_TODO: Add convex radius?
	return MinMotionRadius;
	}


//--------------------------------------------------------------------------------------------------
float RkHullShape::GetMaxMotionRadius( const RkVector3& Center ) const
	{
	RK_ASSERT( mHull );

	float MaxMotionRadius = 0.0f;
	for ( int Index = 0; Index < mHull->VertexCount; ++Index )
		{
		RkVector3 Vertex = mHull->GetPosition( Index );
		MaxMotionRadius = rkMax( MaxMotionRadius, rkDistance( Vertex, Center ) );
		}

	return MaxMotionRadius;
	}
