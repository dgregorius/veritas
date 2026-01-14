// SPDX-FileCopyrightText: 2025 Dirk Gregorius and Erin Catto
// SPDX-License-Identifier: MIT

#include "sat.h"

#include "algorithm.h"
#include "constants.h"
#include "shape.h"

#include "box3d/math_functions.h"

float b3Project( const b3Plane& plane, const b3Hull* hull )
{
	b3Vec3 support = b3FindHullSupportPoint( hull, -plane.normal );
	return b3PlaneSeparation( plane, support );
}

b3FaceQuery b3QueryFaceDirectionHullAndCapsule( const b3Hull* hull, const b3Capsule* capsule, b3Transform capsuleTransform )
{
	int maxFaceIndex = -1;
	int maxVertexIndex = -1;
	float maxFaceSeparation = -FLT_MAX;
	const b3Plane* planes = b3GetHullPlanes( hull );

	b3Vec3 capsulePoints[2] = {
		b3TransformPoint( capsuleTransform, capsule->center1 ),
		b3TransformPoint( capsuleTransform, capsule->center2 ),
	};

	for ( int faceIndex = 0; faceIndex < hull->faceCount; ++faceIndex )
	{
		b3Plane plane = planes[faceIndex];

		int vertexIndex = b3GetPointSupport( capsulePoints, 2, -plane.normal );
		b3Vec3 support = capsulePoints[vertexIndex];
		float separation = b3PlaneSeparation( plane, support );
		if ( separation > maxFaceSeparation )
		{
			maxVertexIndex = vertexIndex;
			maxFaceIndex = faceIndex;
			maxFaceSeparation = separation;
		}
	}

	return b3FaceQuery{
		.separation = maxFaceSeparation,
		.faceIndex = (uint8_t)maxFaceIndex,
		.vertexIndex = (uint8_t)maxVertexIndex,
	};
}

b3FaceQuery b3QueryFaceDirections( const b3Hull* hullA, const b3Hull* hullB, const b3Transform& relativeTransform )
{
	// We perform all computations in local space of the second hull
	b3Transform transform = b3InvertTransform( relativeTransform );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	int maxFaceIndex = -1;
	int maxVertexIndex = -1;
	float maxFaceSeparation = -FLT_MAX;

	for ( int faceIndex = 0; faceIndex < hullA->faceCount; ++faceIndex )
	{
		b3Plane plane = b3TransformPlane( transform, planesA[faceIndex] );

		int vertexIndex = b3FindHullSupportVertex( hullB, -plane.normal );
		b3Vec3 support = pointsB[vertexIndex];
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
		.faceIndex = (uint8_t)maxFaceIndex,
		.vertexIndex = (uint8_t)maxVertexIndex,
	};
}

//--------------------------------------------------------------------------------------------------
// Edge queries
//--------------------------------------------------------------------------------------------------
bool b3IsMinkowskiFace( b3Vec3 a, b3Vec3 b, b3Vec3 n )
{
	// An isolated edge (e.g. like in a capsule) defines a circle through the
	// origin on the Gauss map. So testing for overlap between this circle and
	// the arc AB simplifies to a simple plane test.
	float an = b3Dot( a, n );
	float bn = b3Dot( b, n );

	return an * bn <= 0.0f;
}

// bxa = cross(b, a) and dxc = cross(d, c)
// but in practice we use the edge vector between the faces for robustness
bool b3IsMinkowskiFace( b3Vec3 a, b3Vec3 b, b3Vec3 bxa, b3Vec3 c, b3Vec3 d, b3Vec3 dxc )
{
	// Two edges build a face on the Minkowski sum if the associated arcs ab and cd intersect on the Gauss map.
	// The associated arcs are defined by the adjacent face normals of each edge.
	float cba = b3Dot( c, bxa );
	float dba = b3Dot( d, bxa );
	float adc = b3Dot( a, dxc );
	float bdc = b3Dot( b, dxc );

	return cba * dba < 0.0f && adc * bdc < 0.0f && cba * bdc > 0.0f;
}

float b3EdgeEdgeSeparation( b3Vec3 p1, b3Vec3 e1, b3Vec3 c1, b3Vec3 p2, b3Vec3 e2, b3Vec3 c2 )
{
	// Build search direction
	b3Vec3 u = b3Cross( e1, e2 );
	float length = b3Length( u );

	// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
	const float kTolerance = 0.005f;
	if ( length < kTolerance * sqrtf( b3LengthSquared( e1 ) * b3LengthSquared( e2 ) ) )
	{
		return -FLT_MAX;
	}

	if ( length * length < 1000.0f * FLT_MIN )
	{
		return -FLT_MAX;
	}

	b3Vec3 n = ( 1.0f / length ) * u;

	// Make sure normal points away from the first shape
	// For a triangle, it is possible that N is aligned with the triangle normal and the Sign
	// value can be close to zero and flicker between small negative and positive values, leading to
	// an incorrect separation value. So we assume the other hull has some volume and pick the most
	// significant sign value to orient N.
	float sign1 = b3Dot( n, p1 - c1 );
	float sign2 = b3Dot( n, p2 - c2 );
	if ( b3AbsFloat( sign1 ) > b3AbsFloat( sign2 ) )
	{
		if ( sign1 < 0.0f )
		{
			n = -n;
		}
	}
	else
	{
		if ( sign2 > 0.0f )
		{
			n = -n;
		}
	}

	// s = Dot(n, p2) - d = Dot(n, p2) - Dot(n, p1) = Dot(n, p2 - p1)
	return b3Dot( n, p2 - p1 );
}

b3EdgeQuery b3QueryEdgeDirectionHullAndCapsule( const b3Hull* hull, const b3Capsule* capsule, b3Transform capsuleTransform )
{
	// Find axis of minimum penetration
	float maxSeparation = -FLT_MAX;
	int maxIndex1 = -1;
	int maxIndex2 = -1;

	// We perform all computations in local space of the hull
	b3Vec3 p1 = b3TransformPoint( capsuleTransform, capsule->center1 );
	b3Vec3 q1 = b3TransformPoint( capsuleTransform, capsule->center2 );
	b3Vec3 e1 = q1 - p1;

	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );

	for ( int index = 0; index < hull->edgeCount; index += 2 )
	{
		const b3HullHalfEdge* edge = edges + index;
		const b3HullHalfEdge* twin = edges + index + 1;
		B3_ASSERT( edge->twin == index + 1 && twin->twin == index );

		b3Vec3 p2 = points[edge->origin];
		b3Vec3 q2 = points[twin->origin];
		b3Vec3 e2 = q2 - p2;

		b3Vec3 u2 = planes[edge->face].normal;
		b3Vec3 v2 = planes[twin->face].normal;

		if ( b3IsMinkowskiFace( u2, v2, e1 ) )
		{
			// We can pass any point on the edge and choose
			// the edge centers for better numerical precision.
			b3Vec3 c1 = 0.5f * ( q1 + p1 );
			b3Vec3 c2 = hull->center;
			float separation = b3EdgeEdgeSeparation( q1, e1, c1, q2, e2, c2 );
			if ( separation > maxSeparation )
			{
				// Note: We don't exit early if we find a separating axis here since we want to
				// find the best one for caching and account for the convex radius later.
				maxSeparation = separation;
				maxIndex1 = 0;
				maxIndex2 = index;
			}
		}
	}

	// Save result
	return b3EdgeQuery{
		.separation = maxSeparation,
		.indexA = (uint8_t)maxIndex1,
		.indexB = (uint8_t)maxIndex2,
	};
}

b3EdgeQuery b3QueryEdgeDirections( const b3Hull* hullA, const b3Hull* hullB, const b3Transform& relativeTransform )
{
	// Find axis of minimum penetration
	float maxSeparation = -FLT_MAX;
	int maxIndex1 = B3_NULL_INDEX;
	int maxIndex2 = B3_NULL_INDEX;

	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );
	const b3Plane* planesB = b3GetHullPlanes( hullB );

	// We perform all computations in local space of the second hull
	b3Transform transform = b3InvertTransform( relativeTransform );
	b3Matrix3 matrix = b3MakeMatrixFromQuat( transform.q );

	for ( int index1 = 0; index1 < hullA->edgeCount; index1 += 2 )
	{
		const b3HullHalfEdge* edge1 = edgesA + index1;
		const b3HullHalfEdge* twin1 = edgesA + index1 + 1;
		B3_ASSERT( edge1->twin == index1 + 1 && twin1->twin == index1 );

		b3Vec3 q1 = pointsA[twin1->origin];
		b3Vec3 e1 = b3MulMV( matrix, q1 - pointsA[edge1->origin] );
		q1 = b3MulMV( matrix, q1 ) + transform.p;

		b3Vec3 u1 = b3MulMV( matrix, planesA[edge1->face].normal );
		b3Vec3 v1 = b3MulMV( matrix, planesA[twin1->face].normal );

		for ( int index2 = 0; index2 < hullB->edgeCount; index2 += 2 )
		{
			const b3HullHalfEdge* edge2 = edgesB + index2;
			const b3HullHalfEdge* twin2 = edgesB + index2 + 1;
			B3_ASSERT( edge2->twin == index2 + 1 && twin2->twin == index2 );

			b3Vec3 q2 = pointsB[twin2->origin];
			b3Vec3 e2 = q2 - pointsB[edge2->origin];

			b3Vec3 u2 = planesB[edge2->face].normal;
			b3Vec3 v2 = planesB[twin2->face].normal;

			bool isMinkowski;
			// bool b3IsMinkowskiFace( b3Vec3 a, b3Vec3 b, b3Vec3 bXA, b3Vec3 c, b3Vec3 d, b3Vec3 dXC )
			{
				// Two edges build a face on the Minkowski sum if the associated arcs AB and CD intersect on the Gauss map.
				// The associated arcs are defined by the adjacent face normals of each edge.
				float cba = b3Dot( u2, e1 );
				float dba = b3Dot( v2, e1 );
				float adc = -b3Dot( u1, e2 );
				float bdc = -b3Dot( v1, e2 );

				isMinkowski = cba * dba < 0.0f && adc * bdc < 0.0f && cba * bdc > 0.0f;
			}

			// if ( b3IsMinkowskiFace( u1, v1, -e1, -u2, -v2, -e2 ) )
			if ( isMinkowski )
			{
				// float Separation = b3Project(Q2, E2, Q1, E1, Hull2->Center);

				b3Vec3 c1 = b3TransformPoint( transform, hullA->center );
				b3Vec3 c2 = hullB->center;
				float separation = b3EdgeEdgeSeparation( q1, e1, c1, q2, e2, c2 );

				if ( separation > maxSeparation )
				{
					// Note: We don't exit early if we find a separating axis here since we want to
					// find the best one for caching and account for the convex radius later.
					maxSeparation = separation;
					maxIndex1 = index1;
					maxIndex2 = index2;
				}
			}
		}
	}

	return b3EdgeQuery{
		.separation = maxSeparation,
		.indexA = maxIndex1,
		.indexB = maxIndex2,
	};
}

// This was extended to make the wedge shape get the correct incident face.
// Instead of looking directly for the most anti-parallel face, we first find the closest vertex (passed in).
// Then we look for all edges coming out of that vertex and look for the edge that is
// most perpendicular to the reference normal.
// Then from that edge, we select the adjacent face that is most anti-parallel to the reference normal.
int b3FindIncidentFace( const b3Hull* hull, b3Vec3 refNormal, int vertexIndex )
{
	const b3HullVertex* vertices = b3GetHullVertices( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	int minEdgeIndex = -1;
	float minEdgeProjection = FLT_MAX;

	const b3HullVertex* vertex = vertices + vertexIndex;
	B3_ASSERT( vertex );

	int edgeIndex = vertex->edge;
	const b3HullHalfEdge* edge = edges + edgeIndex;
	b3Vec3 edgeOrigin = points[edge->origin];
	B3_ASSERT( edge->origin == vertexIndex );

	do
	{
		const b3HullHalfEdge* twin = edges + edge->twin;
		b3Vec3 twinOrigin = points[twin->origin];

		b3Vec3 axis = b3Normalize( twinOrigin - edgeOrigin );
		float edgeProjection = b3AbsFloat( b3Dot( axis, refNormal ) );
		if ( edgeProjection < minEdgeProjection )
		{
			minEdgeIndex = edgeIndex;
			minEdgeProjection = edgeProjection;
		}

		edgeIndex = twin->next;
		edge = edges + edgeIndex;
		B3_ASSERT( edge->origin == vertexIndex );
	}
	while ( edge != edges + vertex->edge );
	B3_ASSERT( minEdgeIndex >= 0 );

	const b3HullHalfEdge* minEdge = edges + minEdgeIndex;
	int minFaceIndex1 = minEdge->face;
	b3Plane minPlane1 = planes[minFaceIndex1];

	const b3HullHalfEdge* minTwin = edges + minEdge->twin;
	int minFaceIndex2 = minTwin->face;
	b3Plane minPlane2 = planes[minFaceIndex2];

	return b3Dot( minPlane1.normal, refNormal ) < b3Dot( minPlane2.normal, refNormal ) ? minFaceIndex1 : minFaceIndex2;
}

static void b3SwapUint8( uint8_t& a, uint8_t& b )
{
	uint8_t c = a;
	a = b;
	b = c;
}

b3FeaturePair b3MakeFeaturePair( b3FeatureOwner owner1, int index1, b3FeatureOwner owner2, int index2 )
{
	B3_ASSERT( 0 <= index1 && index1 <= UINT8_MAX );
	B3_ASSERT( 0 <= index2 && index2 <= UINT8_MAX );

	b3FeaturePair pair;
	pair.index1 = uint8_t( index1 );
	pair.owner1 = uint8_t( owner1 );
	pair.index2 = uint8_t( index2 );
	pair.owner2 = uint8_t( owner2 );

	B3_ASSERT( b3MakeFeatureKey( pair ) != B3_INVALID_FEATURE_PAIR );
	return pair;
}

b3FeaturePair b3FlipPair( b3FeaturePair pair )
{
	B3_ASSERT( pair.owner1 == 0 || pair.owner1 == 1 );
	B3_ASSERT( pair.owner2 == 0 || pair.owner2 == 1 );
	b3SwapUint8( pair.owner1, pair.owner2 );
	pair.owner1 = 1 - pair.owner1;
	pair.owner2 = 1 - pair.owner2;

	b3SwapUint8( pair.index1, pair.index2 );

	B3_ASSERT( b3MakeFeatureKey( pair ) != B3_INVALID_FEATURE_PAIR );
	return pair;
}

bool b3ClipSegment( b3ClipVertex segment[2], b3Plane plane )
{
	int vertexCount = 0;
	b3ClipVertex vertex1 = segment[0];
	b3ClipVertex vertex2 = segment[1];

	float distance1 = b3PlaneSeparation( plane, vertex1.position );
	float distance2 = b3PlaneSeparation( plane, vertex2.position );

	// If the points are behind the plane
	if ( distance1 <= 0.0f )
	{
		segment[vertexCount++] = vertex1;
	}
	if ( distance2 <= 0.0f )
	{
		segment[vertexCount++] = vertex2;
	}

	// If the points are on different sides of the plane
	if ( distance1 * distance2 < 0.0f )
	{
		// Find intersection point of edge and plane
		float t = distance1 / ( distance1 - distance2 );
		segment[vertexCount].position = ( 1.0f - t ) * vertex1.position + t * vertex2.position;
		segment[vertexCount].pair = distance1 > 0.0f ? vertex1.pair : vertex2.pair;
		vertexCount++;
	}

	return vertexCount == 2;
}

void b3BuildCapsuleSegment( b3ClipVertex out[2], const b3Capsule* capsule, b3Transform capsuleTransform )
{
	out[0].position = b3TransformPoint( capsuleTransform, capsule->center1 );
	out[0].separation = 0.0f;
	out[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
	out[1].position = b3TransformPoint( capsuleTransform, capsule->center2 );
	out[1].separation = 0.0f;
	out[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );
}

bool b3ClipSegmentToHullFace( b3ClipVertex segment[2], const b3Hull* hull, b3Transform hullTransform, int refFace )
{
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	b3Plane refPlane = b3TransformPlane( hullTransform, planes[refFace] );

	const b3HullFace* face = faces + refFace;

	int edgeIndex = face->edge;

	do
	{
		const b3HullHalfEdge* edge = edges + edgeIndex;

		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edges + nextEdgeIndex;

		b3Vec3 vertex1 = b3TransformPoint( hullTransform, points[edge->origin] );
		b3Vec3 vertex2 = b3TransformPoint( hullTransform, points[next->origin] );
		b3Vec3 tangent = b3Normalize( vertex2 - vertex1 );
		b3Vec3 binormal = b3Cross( tangent, refPlane.normal );

		if ( !b3ClipSegment( segment, b3MakePlaneFromNormalAndPoint( binormal, vertex1 ) ) )
		{
			return false;
		}

		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != face->edge );

	return true;
}

#if B3_ENABLE_VALIDATION
static bool b3ValidatePolygon( b3ClipVertex* polygon, int count )
{
	// Empty polygons are valid (we can clip away all points when re-constructing manifolds from cache)
	if ( count == 0 )
	{
		return true;
	}

	// Validate that incoming and outgoing edges match
	b3ClipVertex vertex1 = polygon[count - 1];
	for ( int i = 0; i < count; ++i )
	{
		b3ClipVertex vertex2 = polygon[i];

		if ( vertex1.pair.owner2 != vertex2.pair.owner1 )
		{
			return false;
		}

		if ( vertex1.pair.index2 != vertex2.pair.index1 )
		{
			return false;
		}

		vertex1 = vertex2;
	}

	return true;
}
#endif

int b3BuildPolygon( b3ClipVertex* out, const b3Transform& transform, const b3Hull* hull, int incFace, b3Plane refPlane )
{
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	const b3HullFace* face = faces + incFace;
	int edgeIndex = face->edge;
	B3_ASSERT( edges[edgeIndex].face == incFace );

	int outCount = 0;

	do
	{
		const b3HullHalfEdge* edge = edges + edgeIndex;

		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edges + nextEdgeIndex;

		b3ClipVertex vertex;
		vertex.position = b3TransformPoint( transform, points[next->origin] );
		vertex.separation = b3PlaneSeparation( refPlane, vertex.position );
		vertex.pair = b3MakeFeaturePair( b3_featureShapeB, edgeIndex, b3_featureShapeB, nextEdgeIndex );

		out[outCount] = vertex;
		outCount += 1;

		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != face->edge && outCount < B3_MAX_CLIP_POINTS );

	B3_VALIDATE( b3ValidatePolygon( out, outCount ) );

	return outCount;
}

int b3ClipPolygon( b3ClipVertex* out, b3ClipVertex* polygon, int count, b3Plane clipPlane, int edge, b3Plane refPlane )
{
	B3_ASSERT( count >= 3 );

	b3ClipVertex vertex1 = polygon[count - 1];
	float distance1 = b3PlaneSeparation( clipPlane, vertex1.position );
	int outCount = 0;

	for ( int index = 0; index < count; ++index )
	{
		b3ClipVertex vertex2 = polygon[index];
		float distance2 = b3PlaneSeparation( clipPlane, vertex2.position );

		// Clip edge against plane (Sutherland-Hodgman clipping)
		if ( distance1 <= 0.0f && distance2 <= 0.0f )
		{
			// Both vertices are behind the plane - keep vertex2
			out[outCount] = vertex2;
			outCount += 1;
		}
		else if ( distance1 <= 0.0f && distance2 > 0.0f )
		{
			// Vertex1 is behind of the plane, vertex2 is in front -> intersection point
			float fraction = distance1 / ( distance1 - distance2 );
			b3Vec3 position = vertex1.position + fraction * ( vertex2.position - vertex1.position );

			// Keep intersection point and adjust outgoing edge
			b3ClipVertex vertex;
			vertex.position = position;
			vertex.separation = b3PlaneSeparation( refPlane, position );
			vertex.pair = vertex2.pair;
			vertex.pair.owner2 = b3_featureShapeA;
			vertex.pair.index2 = static_cast<uint8_t>( edge );
			out[outCount] = vertex;
			outCount += 1;
		}
		else if ( distance2 <= 0.0f && distance1 > 0.0f )
		{
			// Vertex1 is in front, vertex2 is behind of the plane, -> intersection point
			float fraction = distance1 / ( distance1 - distance2 );
			b3Vec3 position = vertex1.position + fraction * ( vertex2.position - vertex1.position );

			// Keep intersection point and adjust incoming edge
			b3ClipVertex vertex;
			vertex.position = position;
			vertex.separation = b3PlaneSeparation( refPlane, position );
			vertex.pair = vertex1.pair;
			vertex.pair.owner1 = b3_featureShapeA;
			vertex.pair.index1 = static_cast<uint8_t>( edge );
			out[outCount] = vertex;
			outCount += 1;

			// And also keep vertex2
			out[outCount] = vertex2;
			outCount += 1;
		}

		// Keep vertex2 as starting vertex for next edge
		vertex1 = vertex2;
		distance1 = distance2;
	}

	B3_VALIDATE( b3ValidatePolygon( out, outCount ) );

	return outCount;
}

// Reduce the manifold points to a maximum of 4 points.
// Note: this modifies the input point array to improve performance
void b3ReduceManifoldPoints( b3Manifold* manifold, b3ManifoldPoint* points, int count )
{
	if ( count <= 4 )
	{
		for ( int i = 0; i < count; ++i )
		{
			manifold->points[i] = points[i];
		}

		manifold->pointCount = count;
		return;
	}

	b3Vec3 normal = manifold->normal;
	float linearSlop = B3_LINEAR_SLOP;

	// Step 1: find extreme point that is touching
	int bestIndex = B3_NULL_INDEX;
	float bestScore = -FLT_MAX;

#if 0
	// Use deepest point -> manifold jitter
	for ( int index = 0; index < count; ++index )
	{
		b3ManifoldPoint* mp = points + index;

		float score = -mp->separation;
		if ( 0.99f * score - 0.1f * linearSlop > bestScore )
		{
			bestIndex = index;
			bestScore = score;
		}
	}
#else
	// Arbitrary tangent direction
	b3Vec3 perp1 = b3Perp( normal );
	b3Vec3 perp2 = b3Cross( perp1, normal );
	b3Vec3 searchDirection = -0.4535961214255773f * perp1 + 0.8912073600614354f * perp2;
	float speculativeDistance = B3_SPECULATIVE_DISTANCE;
	for ( int index = 0; index < count; ++index )
	{
		b3ManifoldPoint* mp = points + index;

		if ( mp->separation > speculativeDistance )
		{
			continue;
		}

		// The deeper the better
		float score = -mp->separation + b3Dot( searchDirection, mp->anchorB );
		if ( 0.99f * score > bestScore )
		{
			bestIndex = index;
			bestScore = score;
		}
	}
#endif

	B3_ASSERT( 0 <= bestIndex && bestIndex < count );
	manifold->points[0] = points[bestIndex];
	manifold->pointCount = 1;

	// Remove best point from array
	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 a = manifold->points[0].anchorB;

	// Step 2: Find farthest point in 2D
	bestScore = 0.0f;
	bestIndex = B3_NULL_INDEX;
	float maxDistanceSquared = 0.0f;

	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].anchorB;
		b3Vec3 d = b3Sub( p, a );
		b3Vec3 v = b3MulSub( d, b3Dot( d, normal ), normal );
		float distanceSquared = b3LengthSquared( v );
		maxDistanceSquared = b3MaxFloat( maxDistanceSquared, distanceSquared );
		float separation = b3MaxFloat( 0.0f, -points[index].separation );
		float score = distanceSquared + 4.0f * separation * separation;
		if ( 0.999f * score > bestScore )
		{
			bestScore = score;
			bestIndex = index;
		}
	}

	if ( bestScore < linearSlop * linearSlop )
	{
		return;
	}

	B3_ASSERT( 0 <= bestIndex && bestIndex < count );
	manifold->points[1] = points[bestIndex];
	manifold->pointCount = 2;

	// Remove best point from array
	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 b = manifold->points[1].anchorB;

	// Step 3: Find the point with the maximum triangular area
	bestScore = 0.0f;
	bestIndex = B3_NULL_INDEX;
	float bestSignedArea = 0.0f;
	b3Vec3 ba = b3Sub( b, a );
	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].anchorB;
		float signedArea = b3Dot( normal, b3Cross( ba, p - a ) );
		float score = b3AbsFloat( signedArea );
		if ( score >= 1.01f * bestScore )
		{
			bestScore = score;
			bestIndex = index;
			bestSignedArea = signedArea;
		}
	}

	if ( bestScore < linearSlop * linearSlop )
	{
		return;
	}

	B3_ASSERT( bestIndex != B3_NULL_INDEX );

	manifold->points[2] = points[bestIndex];
	manifold->pointCount = 3;
	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 c = manifold->points[2].anchorB;

	// Step 4: get the point that adds the most area outside the current triangle
#if 0
	// Get deepest point remaining -> manifold jitter
	bestScore = -FLT_MAX;
	bestIndex = B3_NULL_INDEX;
	for ( int index = 0; index < count; ++index )
	{
		float score = -points[index].separation;
		if ( 0.99f * score - 0.1f * linearSlop >= bestScore )
		{
			bestScore = score;
			bestIndex = index;
		}
	}
#else
	bestScore = 0.0f;
	bestIndex = B3_NULL_INDEX;
	float invSignedArea = 1.0f / bestSignedArea;
	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].anchorB;
		float u1 = invSignedArea * b3Dot( normal, b3Cross( p - a, ba ) );
		float u2 = invSignedArea * b3Dot( normal, b3Cross( p - b, c - b ) );
		float u3 = invSignedArea * b3Dot( normal, b3Cross( p - c, a - c ) );
		float score = b3MaxFloat( u1, b3MaxFloat( u2, u3 ) );

		if ( 0.99f * score >= bestScore )
		{
			bestScore = score;
			bestIndex = index;
		}
	}
#endif

	if ( bestIndex != B3_NULL_INDEX )
	{
		manifold->points[manifold->pointCount] = points[bestIndex];
		manifold->pointCount += 1;
	}
}
