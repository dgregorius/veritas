// SPDX-FileCopyrightText: 2025 Dirk Gregorius and Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "stack_array.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

struct b3Capsule;
struct b3Hull;

struct b3FaceQuery
{
	float separation;
	int faceIndex;
	int vertexIndex;
};

float b3Project( const b3Plane& plane, const b3Hull* hull );

// The relative transform is the transform of the second shape in the frame of the first shape
b3FaceQuery b3QueryFaceDirectionHullAndCapsule( const b3Hull* hull, const b3Capsule* capsule, b3Transform capsuleTransform );
b3FaceQuery b3QueryFaceDirections( const b3Hull* hullA, const b3Hull* hullB, const b3Transform& relativeTransform );

struct b3EdgeQuery
{
	float separation;
	int indexA;
	int indexB;
};

bool b3IsMinkowskiFace( b3Vec3 a, b3Vec3 b, b3Vec3 n );
bool b3IsMinkowskiFace( b3Vec3 a, b3Vec3 b, b3Vec3 bxa, b3Vec3 c, b3Vec3 d, b3Vec3 dxc );

float b3EdgeEdgeSeparation( b3Vec3 p1, b3Vec3 e1, b3Vec3 c1, b3Vec3 p2, b3Vec3 e2, b3Vec3 c2 );

// The relative transform is the transform of the second shape in the frame of the first shape
b3EdgeQuery b3QueryEdgeDirectionHullAndCapsule( const b3Hull* hull, const b3Capsule* capsule, b3Transform capsuleTransform );
b3EdgeQuery b3QueryEdgeDirections( const b3Hull* hullA, const b3Hull* hullB, const b3Transform& relativeTransform );

int b3FindIncidentFace( const b3Hull* hull, b3Vec3 refNormal, int vertexIndex );

#define B3_MAX_CLIP_POINTS 32

enum b3FeatureOwner
{
	b3_featureShapeA = 0,
	b3_featureShapeB = 1
};

struct b3FeaturePair
{
	// Incoming type (either edge on shape A or shape B)
	uint8_t owner1;
	// Incoming edge index (into associated shape array)
	uint8_t index1;
	// Outgoing type (either edge on shape A or shape B)
	uint8_t owner2;
	// Outgoing edge index (into associated shape array)
	uint8_t index2;
};

b3FeaturePair b3MakeFeaturePair( b3FeatureOwner owner1, int index1, b3FeatureOwner owner2, int index2 );
b3FeaturePair b3FlipPair( b3FeaturePair pair );

B3_INLINE uint32_t b3MakeFeatureKey( b3FeaturePair pair )
{
	return ( uint32_t( pair.owner1 ) << 24 ) | ( uint32_t( pair.index1 ) << 16 ) | ( uint32_t( pair.owner2 ) << 8 ) |
		   uint32_t( pair.index2 );
}

B3_INLINE uint32_t b3MakeFeatureKeyFull( b3FeatureOwner owner1, int index1, b3FeatureOwner owner2, int index2 )
{
	b3FeaturePair pair = b3MakeFeaturePair( owner1, index1, owner2, index2 );
	return ( uint32_t( pair.owner1 ) << 24 ) | ( uint32_t( pair.index1 ) << 16 ) | ( uint32_t( pair.owner2 ) << 8 ) |
		   uint32_t( pair.index2 );
}

struct b3ClipVertex
{
	b3Vec3 position;
	float separation;
	b3FeaturePair pair;
};

bool b3ClipSegment( b3ClipVertex segment[2], b3Plane plane );

void b3BuildCapsuleSegment( b3ClipVertex out[2], const b3Capsule* capsule, b3Transform capsuleTransform );

bool b3ClipSegmentToHullFace( b3ClipVertex segment[2], const b3Hull* hull, b3Transform hullTransform, int refFace );

[[nodiscard]]
int b3BuildPolygon( b3ClipVertex* out, const b3Transform& transform, const b3Hull* hull, int incFace, b3Plane refPlane );

[[nodiscard]]
int b3ClipPolygon( b3ClipVertex* out, b3ClipVertex* polygon, int count, b3Plane clipPlane, int edge, b3Plane refPlane );

void b3ReduceManifoldPoints( b3Manifold* manifold, b3ManifoldPoint* points, int count );
