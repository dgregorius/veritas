//--------------------------------------------------------------------------------------------------
// maifold.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "manifold.h"


//--------------------------------------------------------------------------------------------------
// RkManifold
//--------------------------------------------------------------------------------------------------
float rkFindImpulse( const RkManifold& Manifold, uint32 Key )
	{
	RK_ASSERT( Key != RK_INVALID_FEATURE_PAIR );

	for ( int Index = 0; Index < Manifold.PointCount; ++Index )
		{
		if ( Manifold.Points[ Index ].Key == Key )
			{
			return Manifold.Points[ Index ].Impulse;
			}
		}

	return -1.0f;
	}


//--------------------------------------------------------------------------------------------------
void rkProjectFriction( RkManifold& Out, const RkManifold& Manifold )
	{
	RkVector3 Friction = Manifold.LinearFrictionImpulse.X * Manifold.Tangent1 + Manifold.LinearFrictionImpulse.Y * Manifold.Tangent2;
	Out.LinearFrictionImpulse.X = rkDot( Friction, Out.Tangent1 );
	Out.LinearFrictionImpulse.Y = rkDot( Friction, Out.Tangent2 );

	RkVector3 Twist = Manifold.AngularFrictionImpulse * Manifold.Normal;
	Out.AngularFrictionImpulse = rkDot( Twist, Out.Normal );
	}