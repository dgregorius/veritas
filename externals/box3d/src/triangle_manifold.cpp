// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "algorithm.h"
#include "constants.h"
#include "contact.h"
#include "core.h"
#include "platform.h"
#include "sat.h"
#include "shape.h"

#include "box3d/base.h"
#include "box3d/collision.h"

#include <stddef.h>

struct b3TriangleData
{
	b3Vec3 v1, v2, v3;
	b3Vec3 e1, e2, e3;
	b3Vec3 center;
	b3Plane plane;
	int flags;
};

b3TriangleManifold b3CollideTriangleAndSphere( const b3Vec3* triangleVertices, int flags, const b3Sphere* sphere )
{
	B3_UNUSED( flags );

	b3TriangleManifold manifold = {};

	const float speculativeDistance = B3_SPECULATIVE_DISTANCE;

	b3Vec3 center = sphere->center;

	b3Vec3 v1 = triangleVertices[0], v2 = triangleVertices[1], v3 = triangleVertices[2];

	// Closest point on triangle to sphere center
	b3TrianglePoint closest = b3ClosestPointOnTriangle( v1, v2, v3, center );

	// Test separating axis
	float squaredDistance = b3DistanceSquared( closest.point, center );
	float maxDistance = sphere->radius + speculativeDistance;
	if ( squaredDistance > maxDistance * maxDistance )
	{
		return manifold;
	}

	float distance = sqrtf( squaredDistance );
	b3Vec3 normal;
	if ( distance * distance > 1000.0f * FLT_MIN )
	{
		normal = ( 1.0f / distance ) * ( center - closest.point );
	}
	else
	{
		normal = b3Normalize( b3Cross( v2 - v1, v3 - v1 ) );
	}

	// contact point mid-way
	b3Vec3 contactPoint = 0.5f * ( center - sphere->radius * normal + closest.point );

	manifold.base.normal = normal;
	manifold.base.pointCount = 1;

	b3ManifoldPoint* mp = manifold.base.points + 0;
	mp->anchorA = contactPoint;
	mp->anchorB = contactPoint;
	mp->point = contactPoint;
	mp->separation = distance - sphere->radius;
	mp->id = b3MakeFeatureKeyFull( b3_featureShapeA, 0, b3_featureShapeA, 0 );

	manifold.feature = closest.feature;
	manifold.squaredDistance = squaredDistance;

	return manifold;
}

b3Manifold b3CollideTriangleAndCapsule( const b3Vec3* triangleVertices, int flags, const b3Capsule* capsule,
										b3SimplexCache* cache )
{
	B3_UNUSED( flags );
	b3TriangleHull triangleHull = b3MakeTriangleHull( triangleVertices[0], triangleVertices[1], triangleVertices[2] );
	return b3CollideHullAndCapsule( &triangleHull.base, b3Transform_identity, capsule, b3Transform_identity, cache );
}

static inline int b3GetTriangleSupport( b3Vec3* points, b3Vec3 direction )
{
	int index = 0;
	float distance = b3Dot( points[0], direction );

	float d = b3Dot( points[1], direction );
	if ( d > distance )
	{
		distance = d;
		index = 1;
	}

	d = b3Dot( points[2], direction );
	if ( d > distance )
	{
		return 2;
	}

	return index;
}

static b3FaceQuery b3QueryTriangleFace( const b3TriangleData* triangle, const b3Hull* hull )
{
	const b3Vec3* hullPoints = b3GetHullPoints( hull );
	b3Plane plane = triangle->plane;
	int vertexIndex = b3FindHullSupportVertex( hull, b3Neg( plane.normal ) );
	b3Vec3 support = hullPoints[vertexIndex];
	float separation = b3PlaneSeparation( plane, support );

	return b3FaceQuery{
		.separation = separation,
		.faceIndex = 0,
		.vertexIndex = (uint8_t)vertexIndex,
	};
}

static b3FaceQuery b3QueryHullFace( const b3TriangleData* triangle, const b3Hull* hull )
{
	const b3Plane* hullPlanes = b3GetHullPlanes( hull );
	int faceCount = hull->faceCount;

	b3Vec3 trianglePoints[] = { triangle->v1, triangle->v2, triangle->v3 };

	int maxFaceIndex = -1;
	int maxVertexIndex = -1;
	float maxFaceSeparation = -FLT_MAX;

	for ( int faceIndex = 0; faceIndex < faceCount; ++faceIndex )
	{
		b3Plane plane = hullPlanes[faceIndex];

		int vertexIndex = b3GetTriangleSupport( trianglePoints, -plane.normal );
		b3Vec3 support = trianglePoints[vertexIndex];
		float separation = b3PlaneSeparation( plane, support );
		if ( separation > maxFaceSeparation )
		{
			maxFaceIndex = faceIndex;
			maxVertexIndex = vertexIndex;
			maxFaceSeparation = separation;
		}
	}

	return b3FaceQuery{
		.separation = maxFaceSeparation,
		.faceIndex = maxFaceIndex,
		.vertexIndex = maxVertexIndex,
	};
}

static b3EdgeQuery b3TestEdgePairs( const b3TriangleData* triangle, const b3Hull* hull )
{
	b3EdgeQuery result = { .separation = -FLT_MAX, .indexA = B3_NULL_INDEX, .indexB = B3_NULL_INDEX };

	b3Vec3 trianglePoints[] = { triangle->v1, triangle->v2, triangle->v3 };
	b3Vec3 triangleEdges[] = { triangle->e1, triangle->e2, triangle->e3 };
	float speculativeDistance = B3_SPECULATIVE_DISTANCE;
	int edgeFlags[] = { b3_concaveEdge1, b3_concaveEdge1, b3_concaveEdge3 };

#if B3_FORCE_GHOST_COLLISIONS
	int triangleFlags = 0xFF;
#else
	int triangleFlags = triangle->flags;
#endif

	b3Vec3 triNormal = triangle->plane.normal;

	const b3HullHalfEdge* hullEdges = b3GetHullEdges( hull );
	const b3Vec3* hullPoints = b3GetHullPoints( hull );
	const b3Plane* hullPlanes = b3GetHullPlanes( hull );
	int edgeCount = hull->edgeCount;

	for ( int i = 0; i < edgeCount; i += 2 )
	{
		const b3HullHalfEdge* edge = hullEdges + i;
		const b3HullHalfEdge* twin = hullEdges + i + 1;
		B3_ASSERT( edge->twin == i + 1 && twin->twin == i );

		b3Vec3 hullPoint = hullPoints[edge->origin];
		b3Vec3 hullEdge = hullPoints[twin->origin] - hullPoint;

		b3Vec3 hullNormal1 = hullPlanes[edge->face].normal;
		b3Vec3 hullNormal2 = hullPlanes[twin->face].normal;

		for ( int j = 0; j < 3; ++j )
		{
			b3Vec3 triEdge = triangleEdges[j];

			float cab = b3Dot( hullNormal1, triEdge );
			float dab = b3Dot( hullNormal2, triEdge );
			float bcd = b3Dot( triNormal, hullEdge );
			if ( cab * dab >= 0.0f || cab * bcd <= 0.0f )
			{
				continue;
			}

			b3Vec3 triPoint = trianglePoints[j];
			float separation = b3EdgeEdgeSeparation( triPoint, triEdge, triangle->center, hullPoint, hullEdge, hull->center );

			if ( separation > speculativeDistance )
			{
				result.separation = separation;
				result.indexA = j;
				result.indexB = i;
				return result;
			}

			if ( separation > result.separation && ( edgeFlags[j] & triangleFlags ) == 0 )
			{
				// Note: We don't exit early if we find a separating axis here since we want to
				// find the best one for caching and account for the convex radius later.
				result.separation = separation;
				result.indexA = j;
				result.indexB = i;
			}
		}
	}

	return result;
}

// This is called when the hull face is the separating axis. It is optimized for triangle versus hull collision.
// This performs clipping and contact point reduction and populates the manifold. This is done local to the hull,
// so the output must be converted by the caller.
static void b3CollideHullFace( b3Manifold* manifold, const b3TriangleData* triangle, const b3Hull* hull, b3FaceQuery query,
							   b3SATCache* cache )
{
	const b3HullFace* hullFaces = b3GetHullFaces( hull );
	const b3HullHalfEdge* hullEdges = b3GetHullEdges( hull );
	const b3Plane* hullPlanes = b3GetHullPlanes( hull );
	const b3Vec3* hullPoints = b3GetHullPoints( hull );

	// Reference hull face
	int refFace = query.faceIndex;
	b3Plane refPlane = hullPlanes[refFace];

	// Build clip polygon from triangle face (the incident face)
	b3ClipVertex buffer1[2 * B3_MAX_CLIP_POINTS], buffer2[2 * B3_MAX_CLIP_POINTS];

	b3Vec3 trianglePoints[] = { triangle->v1, triangle->v2, triangle->v3 };
	for ( int i = 0; i < 3; ++i )
	{
		buffer1[i].position = trianglePoints[i];
		buffer1[i].separation = b3PlaneSeparation( refPlane, trianglePoints[i] );
		buffer1[i].pair = b3MakeFeaturePair( b3_featureShapeB, i, b3_featureShapeB, ( i + 1 ) % 3 );
	}
	int pointCount = 3;

	// Clip triangle face against side planes of reference face
	b3ClipVertex* input = buffer1;
	b3ClipVertex* output = buffer2;

	const b3HullFace* face = hullFaces + refFace;
	int edgeIndex = face->edge;

	do
	{
		const b3HullHalfEdge* edge = hullEdges + edgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = hullEdges + nextEdgeIndex;
		b3Vec3 vertex1 = hullPoints[edge->origin];
		b3Vec3 vertex2 = hullPoints[next->origin];
		b3Vec3 tangent = b3Normalize( vertex2 - vertex1 );
		b3Vec3 binormal = b3Cross( tangent, refPlane.normal );

		b3Plane clipPlane = b3MakePlaneFromNormalAndPoint( binormal, vertex1 );

		pointCount = b3ClipPolygon( output, input, pointCount, clipPlane, edgeIndex, refPlane );
		B3_ASSERT( pointCount <= 2 * B3_MAX_CLIP_POINTS );

		if ( pointCount < 3 )
		{
			manifold->pointCount = 0;
			*cache = {};
			return;
		}

		// Swap buffers, output becomes input for the next clipping plane
		B3_SWAP( output, input );
		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != face->edge );

	pointCount = b3MinInt( pointCount, B3_MAX_CLIP_POINTS );

	b3ManifoldPoint points[B3_MAX_CLIP_POINTS];
	float minSeparation = FLT_MAX;

	for ( int i = 0; i < pointCount; ++i )
	{
		b3ClipVertex* clipPoint = input + i;

		// Move point onto reference plane for improved culling
		b3Vec3 point = b3MulSub( clipPoint->position, clipPoint->separation, refPlane.normal );

		b3ManifoldPoint* mp = points + i;
		*mp = {};
		mp->point = point;
		mp->anchorA = point;
		mp->anchorB = point;
		mp->separation = clipPoint->separation;
		b3FeaturePair pair = b3FlipPair( clipPoint->pair );
		mp->id = b3MakeFeatureKey( pair );

		minSeparation = b3MinFloat( minSeparation, clipPoint->separation );
	}

	if ( minSeparation >= B3_SPECULATIVE_DISTANCE )
	{
		manifold->pointCount = 0;
		*cache = {};
		return;
	}

	manifold->normal = b3Neg( refPlane.normal );

	b3ReduceManifoldPoints( manifold, points, pointCount );

	// Save cache
	cache->separation = query.separation;
	cache->type = b3_faceNormalB;
	cache->indexA = (uint8_t)query.vertexIndex;
	cache->indexB = (uint8_t)query.faceIndex;
}

static void b3CollideTriangleFace( b3Manifold* manifold, const b3TriangleData* triangle, const b3Hull* hull, b3FaceQuery query,
								   b3SATCache* cache, int triangleIndex )
{
	B3_UNUSED( triangleIndex );

	const b3HullFace* hullFaces = b3GetHullFaces( hull );
	const b3HullHalfEdge* hullEdges = b3GetHullEdges( hull );
	const b3Vec3* hullPoints = b3GetHullPoints( hull );

	// Find incident face
	B3_ASSERT( query.faceIndex == 0 );
	b3Plane refPlane = triangle->plane;

	int incFace = b3FindIncidentFace( hull, refPlane.normal, query.vertexIndex );

	// Build clip polygon from incident face
	b3ClipVertex buffer1[2 * B3_MAX_CLIP_POINTS], buffer2[2 * B3_MAX_CLIP_POINTS];
	int pointCount = 0;
	const b3HullFace* face = hullFaces + incFace;
	int hullEdgeIndex = face->edge;

	do
	{
		const b3HullHalfEdge* edge = hullEdges + hullEdgeIndex;

		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = hullEdges + nextEdgeIndex;

		b3Vec3 hullPoint = hullPoints[next->origin];
		buffer1[pointCount].position = hullPoint;
		buffer1[pointCount].separation = b3PlaneSeparation( refPlane, hullPoint );
		buffer1[pointCount].pair = b3MakeFeaturePair( b3_featureShapeB, hullEdgeIndex, b3_featureShapeB, nextEdgeIndex );

		pointCount += 1;

		hullEdgeIndex = nextEdgeIndex;
	}
	while ( hullEdgeIndex != face->edge && pointCount < 2 * B3_MAX_CLIP_POINTS );

	B3_ASSERT( pointCount >= 3 );

	// Clip incident face against side planes of reference face (triangle)
	b3ClipVertex* input = buffer1;
	b3ClipVertex* output = buffer2;

	b3Vec3 trianglePoints[] = { triangle->v1, triangle->v2, triangle->v3 };
	b3Vec3 triangleEdges[] = { triangle->e1, triangle->e2, triangle->e3 };

	for ( int i = 0; i < 3 && pointCount > 0; ++i )
	{
		b3Vec3 sideNormal = b3Cross( triangleEdges[i], refPlane.normal );
		sideNormal = b3Normalize( sideNormal );

		b3Plane clipPlane = b3MakePlaneFromNormalAndPoint( sideNormal, trianglePoints[i] );

		pointCount = b3ClipPolygon( output, input, pointCount, clipPlane, i, refPlane );
		B3_ASSERT( pointCount <= 2 * B3_MAX_CLIP_POINTS );

		B3_SWAP( output, input );
	}

	pointCount = b3MinInt( pointCount, B3_MAX_CLIP_POINTS );

	b3ManifoldPoint points[B3_MAX_CLIP_POINTS];
	float minSeparation = FLT_MAX;

	for ( int i = 0; i < pointCount; ++i )
	{
		b3ClipVertex* clipPoint = input + i;

		// Move point onto reference plane for improved culling
		b3Vec3 point = b3MulSub(clipPoint->position, clipPoint->separation, refPlane.normal);

		b3ManifoldPoint* mp = points + i;
		*mp = {};
		mp->point = point;
		mp->anchorA = point;
		mp->anchorB = point;
		mp->separation = clipPoint->separation;
		b3FeaturePair pair = clipPoint->pair;
		mp->id = b3MakeFeatureKey( pair );

		minSeparation = b3MinFloat(minSeparation, clipPoint->separation );
	}

	if (minSeparation >= B3_SPECULATIVE_DISTANCE)
	{
		manifold->pointCount = 0;
		*cache = {};
		return;
	}

	manifold->normal = refPlane.normal;

	b3ReduceManifoldPoints( manifold, points, pointCount );

	// Save cache
	cache->separation = query.separation;
	cache->type = b3_faceNormalA;
	cache->indexA = (uint8_t)query.faceIndex;
	cache->indexB = (uint8_t)query.vertexIndex;
}

static void b3CollideEdges( b3Manifold* manifold, b3Vec3 trianglePoint, b3Vec3 triangleEdge, b3Vec3 triangleCenter,
							const b3Hull* hull, b3EdgeQuery query, b3SATCache* cache )
{
	manifold->pointCount = 0;

	B3_VALIDATE( query.separation <= 2.0f * B3_SPECULATIVE_DISTANCE );

	b3Vec3 cA = triangleCenter;
	b3Vec3 pA = trianglePoint;
	b3Vec3 eA = triangleEdge;

	const b3HullHalfEdge* edgesB = b3GetHullEdges( hull );
	const b3Vec3* pointsB = b3GetHullPoints( hull );
	const b3HullHalfEdge* edgeB = edgesB + query.indexB;
	const b3HullHalfEdge* twinB = edgesB + edgeB->twin;
	b3Vec3 pB = pointsB[edgeB->origin];
	b3Vec3 qB = pointsB[twinB->origin];
	b3Vec3 eB = qB - pB;

	b3Vec3 normal = b3Cross( eA, eB );
	normal = b3Normalize( normal );

	// Ensure normal points outward from triangle center
	if ( b3Dot( normal, pA - cA ) < 0.0f )
	{
		normal = -normal;
	}

	// Get the closest points between the infinite edge lines
	b3ClosestApproachResult result = b3ClosestApproachLines( pA, eA, pB, eB );

	// Is one of the closest points outside of the associated edge segment?
	if ( result.lambda1 < 0.0f || 1.0f < result.lambda1 || result.lambda2 < 0.0f || 1.0f < result.lambda2 )
	{
		*cache = {};
		return;
	}

	// This can slide off the end from caching
	float separation = b3Dot( normal, result.point2 - result.point1 );
	B3_VALIDATE( b3AbsFloat( separation - query.separation ) < B3_LINEAR_SLOP );

	b3Vec3 point = 0.5f * ( result.point1 + result.point2 );

	manifold->normal = normal;

	b3ManifoldPoint* mp = manifold->points + 0;
	mp->point = point;
	mp->anchorA = point;
	mp->anchorB = point;
	mp->separation = separation;
	mp->id = b3MakeFeatureKeyFull( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );

	manifold->pointCount = 1;

	// Save cache
	cache->separation = separation;
	cache->type = b3_edgePairAxis;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;
}

// See "Collision Detection of Convex Polyhedra Based on Duality Transformation"
// Simplified for triangle versus hull
static inline bool b3IsTriangleMinkowskiFace( b3Vec3 triNormal, b3Vec3 triEdge, b3Vec3 hullNormal1, b3Vec3 hullNormal2,
											  b3Vec3 hullEdge )
{
	float cab = b3Dot( hullNormal1, triEdge );
	float dab = b3Dot( hullNormal2, triEdge );
	float bcd = b3Dot( triNormal, hullEdge );
	return cab * dab < 0.0f && cab * bcd > 0.0f;
}

b3AtomicInt b3_triangleConvexCalls;
b3AtomicInt b3_triangleCacheHits;

b3Manifold b3CollideTriangleAndHull( const b3Vec3* triangleVertices, int flags, const b3Hull* hull, b3SATCache* cache,
									 b3TriangleFeature* feature, int triangleIndex )
{
	// todo testing
	//cache->type = b3_unknownFeature;

	*feature = b3_featureNone;
	b3Manifold manifold = {};

	b3Vec3 v1 = triangleVertices[0], v2 = triangleVertices[1], v3 = triangleVertices[2];
	b3Plane trianglePlane = b3MakePlaneFromPoints( v1, v2, v3 );
	b3Vec3 triangleCenter = b3MulSV( 1.0f / 3.0f, b3Add( v1, b3Add( v2, v3 ) ) );
	b3Vec3 trianglePoints[] = { v1, v2, v3 };
	b3Vec3 triangleEdges[] = { b3Sub( v2, v1 ), b3Sub( v3, v2 ), b3Sub( v1, v3 ) };

	b3TriangleData triangle = {
		.v1 = v1,
		.v2 = v2,
		.v3 = v3,
		.e1 = triangleEdges[0],
		.e2 = triangleEdges[1],
		.e3 = triangleEdges[2],
		.center = triangleCenter,
		.plane = trianglePlane,
		.flags = flags,
	};

	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Plane* hullPlanes = b3GetHullPlanes( hull );
	const b3Vec3* hullPoints = b3GetHullPoints( hull );

	float speculativeDistance = B3_SPECULATIVE_DISTANCE;
	float linearSlop = B3_LINEAR_SLOP;

	cache->hit = 1;

	// Attempt to use the cache to speed up collision
	switch ( cache->type )
	{
		case b3_faceNormalA:
		{
			B3_ASSERT( cache->indexA == 0 );

			// We perform all computations in local space of the second hull
			int vertexIndex = b3FindHullSupportVertex( hull, b3Neg( trianglePlane.normal ) );
			b3Vec3 support = hullPoints[vertexIndex];
			float separation = b3PlaneSeparation( trianglePlane, support );

			if ( separation >= speculativeDistance )
			{
				// We found a separating axis so update cache.
				cache->separation = separation;
				cache->indexB = (uint8_t)vertexIndex;
				return manifold;
			}

			if ( cache->separation >= speculativeDistance )
			{
				// Objects were separated and are now potentially penetrating - cache is invalidated
				*cache = {};

				// Fall through
			}
			else if ( b3AbsFloat( cache->separation - separation ) < linearSlop )
			{
				// Try to rebuild contact from last separating plane and new deepest point.
				// The deepest point can change when we roll over the surface (e.g. cylinder).
				b3FaceQuery faceQuery;
				faceQuery.separation = separation;
				faceQuery.faceIndex = cache->indexA;
				faceQuery.vertexIndex = vertexIndex;

				// Attempt to reuse cache
				b3CollideTriangleFace( &manifold, &triangle, hull, faceQuery, cache, triangleIndex );

				// Cache reuse is only successful if it creates contact points
				if ( manifold.pointCount > 0 )
				{
					*feature = b3_featureTriangleFace;
					return manifold;
				}

				// Fall through
			}
		}
		break;

		case b3_faceNormalB:
		{
			b3Plane plane = hullPlanes[cache->indexB];

			// Get triangle support point
			int vertexIndex = 0;
			float distance = -b3Dot( v1, plane.normal );
			for ( int i = 1; i < 3; ++i )
			{
				float d = -b3Dot( trianglePoints[i], plane.normal );
				if ( d > distance )
				{
					distance = d;
					vertexIndex = i;
				}
			}

			b3Vec3 support = trianglePoints[vertexIndex];

			// Separation of triangle support point with hull plane
			float separation = b3PlaneSeparation( plane, support );

			if ( separation >= speculativeDistance )
			{
				// We found a separating axis so update cache
				cache->separation = separation;
				cache->indexB = (uint8_t)vertexIndex;
				return manifold;
			}

			if ( cache->separation >= speculativeDistance )
			{
				// Objects were separated and are now potentially penetrating - cache is invalidated
				*cache = {};

				// Fall through
			}
			else if ( b3AbsFloat( cache->separation - separation ) < linearSlop )
			{
				// Try to rebuild contact from last features
				b3FaceQuery faceQuery;
				faceQuery.separation = separation;
				faceQuery.faceIndex = cache->indexB;
				faceQuery.vertexIndex = vertexIndex;

				// Attempt to reuse cache
				b3CollideHullFace( &manifold, &triangle, hull, faceQuery, cache );

				// Cache reuse is only successful if it creates contact points
				if ( manifold.pointCount > 0 )
				{
					*feature = b3_featureHullFace;
					return manifold;
				}

				// Fall through
			}
		}
		break;

		case b3_edgePairAxis:
		{
			B3_ASSERT( cache->indexA < 3 );
			int indexA = cache->indexA;

			b3Vec3 triPoint = trianglePoints[indexA];
			b3Vec3 triEdge = triangleEdges[indexA];

			B3_ASSERT( cache->indexB < hull->edgeCount - 1 );
			int indexB = cache->indexB;

			const b3HullHalfEdge* edge2 = edges + indexB;
			const b3HullHalfEdge* twin2 = edges + indexB + 1;
			B3_ASSERT( edge2->twin == indexB + 1 && twin2->twin == indexB );

			b3Vec3 hullPoint = hullPoints[edge2->origin];
			b3Vec3 hullEdge = hullPoints[twin2->origin] - hullPoint;
			b3Vec3 hullNormal1 = hullPlanes[edge2->face].normal;
			b3Vec3 hullNormal2 = hullPlanes[twin2->face].normal;

			if ( b3IsTriangleMinkowskiFace( trianglePlane.normal, triEdge, hullNormal1, hullNormal2, hullEdge ) )
			{
				// Transform reference center of the first hull into local space of the second hull
				float separation = b3EdgeEdgeSeparation( triPoint, triEdge, triangleCenter, hullPoint, hullEdge, hull->center );
				if ( separation >= speculativeDistance )
				{
					// We found a separating axis
					cache->separation = separation;
					return manifold;
				}

				if ( cache->separation >= speculativeDistance )
				{
					// Objects were separated substantially and are now potentially penetrating.
					*cache = {};

					// Fall through
				}
				else if ( b3AbsFloat( cache->separation - separation ) < linearSlop )
				{
					// Try to rebuild contact from last features
					b3EdgeQuery edgeQuery;
					edgeQuery.indexA = cache->indexA;
					edgeQuery.indexB = cache->indexB;
					edgeQuery.separation = separation;

					// Attempt to reuse cache
					b3CollideEdges( &manifold, triPoint, triEdge, triangleCenter, hull, edgeQuery, cache );

					// Cache reuse is only successful if it creates contact points
					if ( manifold.pointCount > 0 )
					{
						B3_ASSERT( edgeQuery.indexA < 3 );
						b3TriangleFeature edgesFeatures[] = { b3_featureEdge1, b3_featureEdge2, b3_featureEdge3 };
						*feature = edgesFeatures[edgeQuery.indexA];
						return manifold;
					}

					// Fall through
				}
			}
		}
		break;

		default:
			B3_ASSERT( cache->type == b3_unknownFeature );
			break;
	}

	// Cache miss
	cache->hit = 0;

	// Find axis of minimum penetration
	b3FaceQuery faceQueryA = b3QueryTriangleFace( &triangle, hull );
	if ( faceQueryA.separation > speculativeDistance )
	{
		// We found a separating axis
		cache->separation = faceQueryA.separation;
		cache->type = b3_faceNormalA;
		cache->indexA = (uint8_t)faceQueryA.faceIndex;
		cache->indexB = (uint8_t)faceQueryA.vertexIndex;
		return manifold;
	}

	b3FaceQuery faceQueryB = b3QueryHullFace( &triangle, hull );
	if ( faceQueryB.separation > speculativeDistance )
	{
		// We found a separating axis
		cache->separation = faceQueryB.separation;
		cache->type = b3_faceNormalB;
		cache->indexA = (uint8_t)faceQueryB.faceIndex;
		cache->indexB = (uint8_t)faceQueryB.vertexIndex;
		return manifold;
	}

	b3EdgeQuery edgeQuery = b3TestEdgePairs( &triangle, hull );
	if ( edgeQuery.separation > speculativeDistance )
	{
		// We found a separating axis
		cache->separation = edgeQuery.separation;
		cache->type = b3_edgePairAxis;
		cache->indexA = (uint8_t)edgeQuery.indexA;
		cache->indexB = (uint8_t)edgeQuery.indexB;
		return manifold;
	}

	if (edgeQuery.indexA != B3_NULL_INDEX &&
		edgeQuery.separation > 0.9f * faceQueryA.separation + linearSlop &&
		edgeQuery.separation > 0.9f * faceQueryB.separation + linearSlop)
	{
		B3_ASSERT( 0 <= edgeQuery.indexA && edgeQuery.indexA < 3 );
		b3Vec3 trianglePoint = trianglePoints[edgeQuery.indexA];
		b3Vec3 triangleEdge = triangleEdges[edgeQuery.indexA];

		b3CollideEdges( &manifold, trianglePoint, triangleEdge, triangleCenter, hull, edgeQuery, cache );
		b3TriangleFeature edgesFeatures[] = { b3_featureEdge1, b3_featureEdge2, b3_featureEdge3 };
		*feature = edgesFeatures[edgeQuery.indexA];
	}
	else if (faceQueryB.separation > faceQueryA.separation + 0.5f * linearSlop)
	{
		b3CollideHullFace( &manifold, &triangle, hull, faceQueryB, cache );
		*feature = b3_featureHullFace;
	}
	else
	{
		b3CollideTriangleFace( &manifold, &triangle, hull, faceQueryA, cache, triangleIndex );
		*feature = b3_featureTriangleFace;
	}

	return manifold;
}
