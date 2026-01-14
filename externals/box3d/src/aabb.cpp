// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"

#include "box3d/math_functions.h"

#include <float.h>

// From Real-time Collision Detection, p179.
b3CastOutput b3RayCastAABB( b3AABB a, b3Vec3 p1, b3Vec3 p2 )
{
	// Radius not handled
	b3CastOutput output = { };

	float tmin = -FLT_MAX;
	float tmax = FLT_MAX;

	b3Vec3 p = p1;
	b3Vec3 d = b3Sub( p2, p1 );
	b3Vec3 absD = b3Abs( d );

	b3Vec3 normal = b3Vec3_zero;

	// x-coordinate
	if ( absD.x < FLT_EPSILON )
	{
		// parallel
		if ( p.x < a.lowerBound.x || a.upperBound.x < p.x )
		{
			return output;
		}
	}
	else
	{
		float inv_d = 1.0f / d.x;
		float t1 = ( a.lowerBound.x - p.x ) * inv_d;
		float t2 = ( a.upperBound.x - p.x ) * inv_d;

		// Sign of the normal vector.
		float s = -1.0f;

		if ( t1 > t2 )
		{
			float tmp = t1;
			t1 = t2;
			t2 = tmp;
			s = 1.0f;
		}

		// Push the min up
		if ( t1 > tmin )
		{
			normal.y = 0.0f;
			normal.x = s;
			tmin = t1;
		}

		// Pull the max down
		tmax = b3MinFloat( tmax, t2 );

		if ( tmin > tmax )
		{
			return output;
		}
	}

	// y-coordinate
	if ( absD.y < FLT_EPSILON )
	{
		// parallel
		if ( p.y < a.lowerBound.y || a.upperBound.y < p.y )
		{
			return output;
		}
	}
	else
	{
		float inv_d = 1.0f / d.y;
		float t1 = ( a.lowerBound.y - p.y ) * inv_d;
		float t2 = ( a.upperBound.y - p.y ) * inv_d;

		// Sign of the normal vector.
		float s = -1.0f;

		if ( t1 > t2 )
		{
			float tmp = t1;
			t1 = t2;
			t2 = tmp;
			s = 1.0f;
		}

		// Push the min up
		if ( t1 > tmin )
		{
			normal.x = 0.0f;
			normal.y = s;
			tmin = t1;
		}

		// Pull the max down
		tmax = b3MinFloat( tmax, t2 );

		if ( tmin > tmax )
		{
			return output;
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if ( tmin < 0.0f || 1.0f < tmin )
	{
		return output;
	}

	// Intersection.
	output.fraction = tmin;
	output.normal = normal;
	output.point = b3Lerp( p1, p2, tmin );
	output.hit = true;
	return output;
}
