// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"

// Ray cast an AABB
b3CastOutput b3RayCastAABB( b3AABB a, b3Vec3 p1, b3Vec3 p2 );

// Get the surface area (perimeter)
inline float b3Perimeter( b3AABB a )
{
	float wx = a.upperBound.x - a.lowerBound.x;
	float wy = a.upperBound.y - a.lowerBound.y;
	float wz = a.upperBound.z - a.lowerBound.z;
	return 2.0f * ( wx * wz + wy * wx + wz * wy );
}

/// Enlarge a to contain b
/// @return true if the AABB grew
inline bool b3EnlargeAABB( b3AABB* a, b3AABB b )
{
	bool changed = false;
	if ( b.lowerBound.x < a->lowerBound.x )
	{
		a->lowerBound.x = b.lowerBound.x;
		changed = true;
	}

	if ( b.lowerBound.y < a->lowerBound.y )
	{
		a->lowerBound.y = b.lowerBound.y;
		changed = true;
	}

	if ( b.lowerBound.z < a->lowerBound.z )
	{
		a->lowerBound.z = b.lowerBound.z;
		changed = true;
	}

	if ( a->upperBound.x < b.upperBound.x )
	{
		a->upperBound.x = b.upperBound.x;
		changed = true;
	}

	if ( a->upperBound.y < b.upperBound.y )
	{
		a->upperBound.y = b.upperBound.y;
		changed = true;
	}

	if ( a->upperBound.z < b.upperBound.z )
	{
		a->upperBound.z = b.upperBound.z;
		changed = true;
	}

	return changed;
}

/// Do a and b overlap
inline bool b3OverlapAABBs( b3AABB a, b3AABB b )
{
	return !( b.lowerBound.x > a.upperBound.x || b.lowerBound.y > a.upperBound.y || b.lowerBound.z > a.upperBound.z ||
			  a.lowerBound.x > b.upperBound.x || a.lowerBound.y > b.upperBound.y || a.lowerBound.z > b.upperBound.z );
}
