// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"

typedef struct b3ChildShape
{
	union
	{
		b3Capsule capsule;
		const b3Hull* hull;
		b3Mesh mesh;
		b3Sphere sphere;
	};

	b3Transform transform;

	// Index 0 is used for convex shapes.
	int materialIndices[B3_MAX_COMPOUND_MESH_MATERIALS];
	b3ShapeType type;
} b3ChildShape;

typedef bool b3CompoundQueryFcn( const b3Compound* compound, int childIndex, void* context );

#ifdef __cplusplus
extern "C"
{
#endif

b3ChildShape b3GetCompoundChild( const b3Compound* compound, int childIndex );
void b3QueryCompound( const b3Compound* compound, b3AABB aabb, b3CompoundQueryFcn* fcn, void* context );

b3TOIOutput b3CompoundTimeOfImpact( const b3Compound* compound, b3Transform transform, const b3ShapeProxy* proxy,
									const b3Sweep* sweep, float maxFraction );

// Transforms a sweep for a compound child shape
b3Sweep b3MakeCompoundChildSweep( b3Transform compoundTransform, b3Transform childTransform );

int b3CollideMoverAndCompound( b3PlaneResult* planes, int capacity, const b3Compound* shape, const b3Capsule* mover );

#ifdef __cplusplus
}
#endif
