//--------------------------------------------------------------------------------------------------
/*
	@file		clipping.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/constants.h"

#include "ragnarok/common/array.h"
#include "ragnarok/common/math.h"

struct RkCapsule;
struct RkHull;
    

//--------------------------------------------------------------------------------------------------
// RkFeatureType
//--------------------------------------------------------------------------------------------------
enum RkFeatureType
	{
	RK_FEATURE_SHAPE_1 = 0,
	RK_FEATURE_SHAPE_2 = 1
	};


//--------------------------------------------------------------------------------------------------
// RkFeaturePair
//--------------------------------------------------------------------------------------------------
struct RkFeaturePair
	{
	// Attributes 
	uint8 IncomingType;		// Incoming type (either edge on shape 1 or shape 2)
	uint8 IncomingIndex;	// Incoming edge index (into associated shape array)
	uint8 OutgoingType;		// Outgoing type (either edge on shape 1 or shape 2)
	uint8 OutgoingIndex;	// Outgoing edge index (into associated shape array)
	
	operator uint32() const
		{
		return ( OutgoingIndex << 24 ) | ( OutgoingType << 16 ) | ( IncomingIndex << 8 ) | IncomingType;
		}
	};

RkFeaturePair rkMakePair( RkFeatureType Type1, int Index1, RkFeatureType Type2, int Index2 );
RkFeaturePair rkFlipPair( RkFeaturePair Pair );


//--------------------------------------------------------------------------------------------------
// RkClipVertex
//--------------------------------------------------------------------------------------------------
struct RkClipVertex
	{
	RkVector3 Position;
	float Separation;

	RkFeaturePair Pair;
	};


//--------------------------------------------------------------------------------------------------
// General clipping
//--------------------------------------------------------------------------------------------------
bool rkClipSegment( RkClipVertex Segment[ 2 ], const RkPlane3& Plane );


//--------------------------------------------------------------------------------------------------
// Capsule clipping
//--------------------------------------------------------------------------------------------------
void rkBuildSegment( RkClipVertex Out[ 2 ], const RkTransform& Transform, const RkCapsule& Capsule );
bool rkClipSegment( RkClipVertex Segment[ 2 ], const RkTransform& Transform, const RkHull* Hull, int RefFace );


//--------------------------------------------------------------------------------------------------
// Hull clipping
//--------------------------------------------------------------------------------------------------
void rkBuildPolygon( RkArray< RkClipVertex >& Out, const RkTransform& Transform, const RkHull* Hull, int IncFace, const RkPlane3& RefPlane );
void rkClipPolygon( RkArray< RkClipVertex >& Out, const RkArray< RkClipVertex >& Polygon, const RkPlane3& Plane, int Edge, const RkPlane3& RefPlane );
void rkCullPolygon( RkArray< RkClipVertex >& Polygon, const RkPlane3& RefPlane, float MaxSeparation = 2.0f * RK_CONVEX_RADIUS );