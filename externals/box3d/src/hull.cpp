// SPDX-FileCopyrightText: 2025 Dirk Gregorius
// SPDX-License-Identifier: MIT

#include "algorithm.h"
#include "constants.h"
#include "math_internal.h"
#include "qhull.h"
#include "shape.h"

#include "box3d/collision.h"

#include <stdlib.h>

static b3Vec3* b3GetHullPointsWrite( b3Hull* hull )
{
	if ( hull->pointOffset == 0 )
	{
		return nullptr;
	}

	return (b3Vec3*)( (intptr_t)hull + hull->pointOffset );
}

static b3Plane* b3GetHullPlanesWrite( b3Hull* hull )
{
	if ( hull->planeOffset == 0 )
	{
		return nullptr;
	}

	return (b3Plane*)( (intptr_t)hull + hull->planeOffset );
}

static b3HullVertex* b3GetHullVerticesWrite( b3Hull* hull )
{
	if ( hull->vertexOffset == 0 )
	{
		return nullptr;
	}

	return (b3HullVertex*)( (intptr_t)hull + hull->vertexOffset );
}

static b3HullHalfEdge* b3GetHullEdgesWrite( b3Hull* hull )
{
	if ( hull->edgeOffset == 0 )
	{
		return nullptr;
	}

	return (b3HullHalfEdge*)( (intptr_t)hull + hull->edgeOffset );
}

static b3HullFace* b3GetHullFacesWrite( b3Hull* hull )
{
	if ( hull->faceOffset == 0 )
	{
		return nullptr;
	}

	return (b3HullFace*)( (intptr_t)hull + hull->faceOffset );
}

// Sort the edges so that twins are grouped.
// todo_erin this is O(n^2)
static void b3SortEdges( const b3QHHalfEdge** edges, int count )
{
	for ( int edgeIndex = 0; edgeIndex < count; edgeIndex += 2 )
	{
		const b3QHHalfEdge* pEdge = edges[edgeIndex];
		for ( int twinIndex = edgeIndex + 1; twinIndex < count; twinIndex += 1 )
		{
			const b3QHHalfEdge* pTwin = edges[twinIndex];
			if ( pEdge->twin == pTwin )
			{
				B3_ASSERT( pTwin->twin == pEdge );
				const b3QHHalfEdge* temp = edges[edgeIndex + 1];
				edges[edgeIndex + 1] = edges[twinIndex];
				edges[twinIndex] = temp;
			}
		}
	}
}

#if B3_ENABLE_VALIDATION
static bool b3ValidateEdges( const b3QHHalfEdge** edges, int count )
{
	for ( int i = 0; i < count; i += 2 )
	{
		const b3QHHalfEdge* edge = edges[i];
		const b3QHHalfEdge* twin = edges[i + 1];

		if ( edge->twin != twin || twin->twin != edge )
		{
			return false;
		}
	}

	return true;
}
#endif

int b3FindHullSupportVertex( const b3Hull* hull, b3Vec3 direction )
{
	int bestIndex = -1;
	float bestDot = -FLT_MAX;

	int vertexCount = hull->vertexCount;
	const b3Vec3* points = b3GetHullPoints( hull );

	for ( int index = 0; index < vertexCount; ++index )
	{
		float dot = b3Dot( direction, points[index] );
		if ( dot > bestDot )
		{
			bestIndex = index;
			bestDot = dot;
		}
	}
	B3_ASSERT( bestIndex >= 0 );

	return bestIndex;
}

b3Vec3 b3FindHullSupportPoint( const b3Hull* hull, b3Vec3 direction )
{
	int vertexCount = hull->vertexCount;
	B3_ASSERT( vertexCount > 0 );

	const b3Vec3* points = b3GetHullPoints( hull );

	b3Vec3 bestPoint = points[0];
	float bestDot = b3Dot( direction, bestPoint );

	for ( int index = 1; index < vertexCount; ++index )
	{
		float dot = b3Dot( direction, points[index] );
		if ( dot > bestDot )
		{
			bestPoint = points[index];
			bestDot = dot;
		}
	}

	return bestPoint;
}

int b3FindHullSupportFace( const b3Hull* hull, b3Vec3 direction )
{
	int bestIndex = -1;
	float bestDot = -FLT_MAX;

	int faceCount = hull->faceCount;
	const b3Plane* planes = b3GetHullPlanes( hull );

	for ( int index = 0; index < faceCount; ++index )
	{
		float dot = b3Dot( planes[index].normal, direction );
		if ( dot > bestDot )
		{
			bestDot = dot;
			bestIndex = index;
		}
	}
	B3_ASSERT( bestIndex >= 0 );

	return bestIndex;
}

#if B3_ENABLE_VALIDATION

bool b3IsValidHull( const b3Hull* hull )
{
	if ( hull->version != B3_HULL_VERSION )
	{
		return false;
	}

	// Euler's identity for convex polyhedron
	int v = hull->vertexCount;
	int e = hull->edgeCount / 2;
	int f = hull->faceCount;

	if ( v - e + f != 2 )
	{
		return false;
	}

	// Vertex invariants
	const b3HullVertex* vertices = b3GetHullVertices( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	for ( int index = 0; index < hull->vertexCount; ++index )
	{
		const b3HullVertex* vertex = vertices + index;
		const b3HullHalfEdge* edge = edges + vertex->edge;

		// Connectivity tests
		if ( edge->origin != index )
		{
			return false;
		}
	}

	// Edge invariants
	for ( int index = 0; index < hull->edgeCount; index += 2 )
	{
		const b3HullHalfEdge* edge = edges + index + 0;
		const b3HullHalfEdge* twin = edges + index + 1;

		// Connectivity tests
		if ( edge->twin != index + 1 )
		{
			return false;
		}

		if ( twin->twin != index + 0 )
		{
			return false;
		}
	}

	// Face invariants
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	for ( int faceIndex = 0; faceIndex < hull->faceCount; ++faceIndex )
	{
		const b3HullFace* face = faces + faceIndex;

		int baseEdgeIndex = face->edge;
		const b3HullHalfEdge* edge = edges + baseEdgeIndex;

		// Geometry tests
		b3Plane plane = planes[faceIndex];
		if ( b3PlaneSeparation( plane, hull->center ) >= 0.0f )
		{
			return false;
		}

		int edgeIndex = baseEdgeIndex;
		do
		{
			// Connectivity tests
			edge = edges + edgeIndex;
			const b3HullHalfEdge* next = edges + edge->next;
			const b3HullHalfEdge* twin = edges + edge->twin;

			if ( edge->face != faceIndex )
			{
				return false;
			}

			if ( twin->twin != edgeIndex )
			{
				return false;
			}

			if ( next->origin != twin->origin )
			{
				return false;
			}

			edgeIndex = edge->next;
		}
		while ( edgeIndex != baseEdgeIndex );
	}

	return true;
}

#else

bool b3IsValidHull( const b3Hull* hull )
{
	B3_UNUSED( hull );
	return true;
}

#endif

b3Hull* b3CreateCylinder( float height, float radius, float yOffset, int sides )
{
	B3_ASSERT( height > 0.0f );
	B3_ASSERT( radius > 0.0f );
	B3_ASSERT( 3 <= sides && sides <= 32 );

	int pointCount = 2 * sides;
	b3Vec3* points = (b3Vec3*)b3Alloc( pointCount * sizeof( b3Vec3 ) );
	B3_ASSERT( points != nullptr );

	float alpha = 0.0f;
	float deltaAlpha = 2.0f * B3_PI / sides;

	for ( int index = 0; index < sides; ++index )
	{
		float sinAlpha = b3Sin( alpha );
		float cosAlpha = b3Cos( alpha );

		points[2 * index + 0] = { radius * cosAlpha, yOffset, radius * sinAlpha };
		points[2 * index + 1] = { radius * cosAlpha, yOffset + height, radius * sinAlpha };

		alpha += deltaAlpha;
	}

	b3Hull* hull = b3CreateHull( points, pointCount );
	B3_ASSERT( hull->vertexCount == pointCount );
	B3_ASSERT( hull->edgeCount == 6 * sides );
	B3_ASSERT( hull->faceCount == sides + 2 );

	b3Free( points, pointCount * sizeof( b3Vec3 ) );

	return hull;
}

b3Hull* b3CreateCone( float height, float radius1, float radius2, int slices )
{
	B3_ASSERT( height > 0.0f );
	B3_ASSERT( radius1 > 0.0f );
	B3_ASSERT( radius2 > 0.0f );
	B3_ASSERT( 4 <= slices && slices <= 32 );

	int pointCount = 2 * slices;
	b3Vec3* points = (b3Vec3*)b3Alloc( pointCount * sizeof( b3Vec3 ) );
	B3_ASSERT( points != nullptr );

	float alpha = 0.0f;
	float deltaAlpha = 2.0f * B3_PI / slices;

	for ( int index = 0; index < slices; ++index )
	{
		float sinAlpha = b3Sin( alpha );
		float cosAlpha = b3Cos( alpha );

		points[2 * index + 0] = { radius1 * cosAlpha, 0.0f, radius1 * sinAlpha };
		points[2 * index + 1] = { radius2 * cosAlpha, height, radius2 * sinAlpha };

		alpha += deltaAlpha;
	}

	b3Hull* hull = b3CreateHull( points, pointCount );
	B3_ASSERT( hull->vertexCount == pointCount );
	B3_ASSERT( hull->edgeCount == 6 * slices );
	B3_ASSERT( hull->faceCount == slices + 2 );

	b3Free( points, pointCount * sizeof( b3Vec3 ) );

	return hull;
}

template <typename T> static inline int b3FindIndexSlow( const T** array, int count, const T* value )
{
	for ( int i = 0; i < count; ++i )
	{
		if ( array[i] == value )
		{
			return i;
		}
	}

	return B3_NULL_INDEX;
}

static void b3UpdateHullBounds( b3Hull* hull )
{
	const b3Vec3* points = b3GetHullPoints( hull );
	int vertexCount = hull->vertexCount;

	B3_ASSERT( vertexCount > 0 );
	b3AABB bounds;
	bounds.lowerBound = points[0];
	bounds.upperBound = points[0];

	for ( int i = 1; i < vertexCount; ++i )
	{
		b3Vec3 p = points[i];
		bounds.lowerBound = b3Min( bounds.lowerBound, p );
		bounds.upperBound = b3Max( bounds.upperBound, p );
	}

	hull->aabb = bounds;
}

// M. Kallay - "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
// todo shift origin like Box2D does
static void b3UpdateHullBulkProperties( b3Hull* hull )
{
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );

	float area = 0.0f;
	float volume = 0.0f;
	b3Vec3 center = b3Vec3_zero;

	float xx = 0.0f;
	float xy = 0.0f;
	float yy = 0.0f;
	float xz = 0.0f;
	float zz = 0.0f;
	float yz = 0.0f;

	int faceCount = hull->faceCount;

	for ( int faceIndex = 0; faceIndex < faceCount; ++faceIndex )
	{
		const b3HullFace* face = faces + faceIndex;
		const b3HullHalfEdge* edge1 = edges + face->edge;
		const b3HullHalfEdge* edge2 = edges + edge1->next;
		const b3HullHalfEdge* edge3 = edges + edge2->next;

		B3_ASSERT( edge1 != edge3 );
		B3_ASSERT( edge1->origin < hull->vertexCount );

		b3Vec3 v1 = points[edge1->origin];

		do
		{
			B3_ASSERT( edge2->origin < hull->vertexCount );
			B3_ASSERT( edge3->origin < hull->vertexCount );

			b3Vec3 v2 = points[edge2->origin];
			b3Vec3 v3 = points[edge3->origin];

			// Surface area
			area += b3Length( b3Cross( v2 - v1, v3 - v1 ) );

			// Signed volume of this tetrahedron
			float det = b3ScalarTripleProduct( v1, v2, v3 );

			// Contribution to mass
			volume += det;

			// Contribution to centroid
			b3Vec3 v4 = v1 + v2 + v3;
			center += det * v4;

			// Contribution to inertia monomials
			xx += det * ( v1.x * v1.x + v2.x * v2.x + v3.x * v3.x + v4.x * v4.x );
			yy += det * ( v1.y * v1.y + v2.y * v2.y + v3.y * v3.y + v4.y * v4.y );
			zz += det * ( v1.z * v1.z + v2.z * v2.z + v3.z * v3.z + v4.z * v4.z );
			xy += det * ( v1.x * v1.y + v2.x * v2.y + v3.x * v3.y + v4.x * v4.y );
			xz += det * ( v1.x * v1.z + v2.x * v2.z + v3.x * v3.z + v4.x * v4.z );
			yz += det * ( v1.y * v1.z + v2.y * v2.z + v3.y * v3.z + v4.y * v4.z );

			edge2 = edge3;
			edge3 = edges + edge3->next;
		}
		while ( edge1 != edge3 );

	}

	B3_ASSERT( volume > 0.0f );

	center = ( 0.25f / volume ) * center;

	// Compute the inner radius using the face planes and the centroid
	float radius = FLT_MAX;
	for ( int faceIndex = 0; faceIndex < faceCount; ++faceIndex )
	{
		b3Plane plane = planes[faceIndex];

		float distance = b3PlaneSeparation( plane, center );
		B3_ASSERT( distance < 0.0f );

		radius = b3MinFloat( radius, -distance );
	}

	B3_ASSERT( 0.0f < radius && radius < FLT_MAX );

	b3Matrix3 inertia;
	inertia.cx.x = yy + zz;
	inertia.cy.x = -xy;
	inertia.cz.x = -xz;
	inertia.cx.y = -xy;
	inertia.cy.y = xx + zz;
	inertia.cz.y = -yz;
	inertia.cx.z = -xz;
	inertia.cy.z = -yz;
	inertia.cz.z = xx + yy;

	float mass = volume / 6.0f;
	b3Matrix3 centralInertia = ( 1.0f / 120.0f ) * inertia - b3Steiner( mass, center );

	hull->mass = mass;
	hull->center = center;
	hull->centralInertia = centralInertia;
	hull->volume = volume;
	hull->surfaceArea = area;
	hull->innerRadius = radius;
}

b3Hull* b3CreateHull( const b3Vec3* points, int pointCount )
{
	b3HullBuilder convex = {};
	convex.Construct( points, pointCount );
	if ( !convex.IsConsistent() )
	{
		return nullptr;
	}

	constexpr int maxCount = UINT8_MAX;

	if ( pointCount > maxCount )
	{
		return nullptr;
	}

	// Copy vertices, edges and faces into array to transform pointers into indices
	const b3QHVertex* tempVertices[maxCount];
	int vertexIndex = 0;

	const b3QHVertex& vertexList = convex.GetVertexList();
	for ( const b3QHVertex* vertex = b3Begin( vertexList ); vertex != b3End( vertexList ); vertex = vertex->next )
	{
		B3_ASSERT( vertexIndex < maxCount );
		tempVertices[vertexIndex] = vertex;
		vertexIndex += 1;
	}

	B3_ASSERT( vertexIndex <= pointCount );
	int vertexCount = vertexIndex;

	const b3QHFace* tempFaces[maxCount];
	const b3QHHalfEdge* tempEdges[maxCount];
	int faceCount = 0;
	int edgeCount = 0;

	b3QHFace& faceList = convex.m_faceList;
	for ( const b3QHFace* face = b3Begin( faceList ); face != b3End( faceList ); face = face->next )
	{
		if ( faceCount == maxCount )
		{
			return nullptr;
		}

		tempFaces[faceCount] = face;
		faceCount += 1;

		b3QHHalfEdge* edge = face->edge;

		do
		{
			if ( edgeCount == maxCount )
			{
				return nullptr;
			}

			tempEdges[edgeCount] = edge;
			edgeCount += 1;

			edge = edge->next;
		}
		while ( edge != face->edge );
	}

	// Sort the edges so that twins are paired.
	// todo_erin slow O(n^2)?
	b3SortEdges( tempEdges, edgeCount );

#if B3_ENABLE_VALIDATION
	B3_ASSERT( b3ValidateEdges( tempEdges, edgeCount ) );
#endif

	// Allocate the hull. Arrays hang off the end.
	// No special alignment needed because this doesn't use SIMD.
	int byteCount = sizeof( b3Hull );
	int vertexOffset = byteCount;
	byteCount += vertexCount * sizeof( b3HullVertex );
	int pointOffset = byteCount;
	byteCount += vertexCount * sizeof( b3Vec3 );
	int edgeOffset = byteCount;
	byteCount += edgeCount * sizeof( b3HullHalfEdge );
	int faceOffset = byteCount;
	byteCount += faceCount * sizeof( b3HullFace );
	int planeOffset = byteCount;
	byteCount += faceCount * sizeof( b3Plane );

	// Fill hull
	b3Hull* hull = (b3Hull*)b3Alloc( byteCount );

	// Clear memory to ensure resource determinism
	memset( hull, 0, byteCount );

	hull->version = B3_HULL_VERSION;
	hull->vertexOffset = vertexOffset;
	hull->pointOffset = pointOffset;
	hull->edgeOffset = edgeOffset;
	hull->faceOffset = faceOffset;
	hull->planeOffset = planeOffset;

	hull->vertexCount = vertexCount;
	hull->edgeCount = edgeCount;
	hull->faceCount = faceCount;

	hull->byteCount = byteCount;

	b3HullVertex* vertices = b3GetHullVerticesWrite( hull );
	b3HullHalfEdge* edges = b3GetHullEdgesWrite( hull );
	b3HullFace* faces = b3GetHullFacesWrite( hull );
	b3Vec3* finalPoints = b3GetHullPointsWrite( hull );
	b3Plane* planes = b3GetHullPlanesWrite( hull );

	for ( int index = 0; index < vertexCount; ++index )
	{
		vertices[index].edge = 0;

		const b3QHVertex* vertex = tempVertices[index];
		finalPoints[index] = vertex->position;
	}

	// todo_erin this is essentially O(n^2) with searches
	for ( int index = 0; index < edgeCount; ++index )
	{
		const b3QHHalfEdge* edge = tempEdges[index];

		// todo_erin iterate these things and install the indices to avoid the slow search

		int next = b3FindIndexSlow( tempEdges, edgeCount, edge->next );
		B3_ASSERT( 0 <= next && next <= UINT8_MAX );
		edges[index].next = uint8_t( next );

		int twin = b3FindIndexSlow( tempEdges, edgeCount, edge->twin );
		B3_ASSERT( 0 <= twin && twin <= UINT8_MAX );
		edges[index].twin = uint8_t( twin );

		int face = b3FindIndexSlow( tempFaces, faceCount, edge->face );
		B3_ASSERT( 0 <= face && face <= UINT8_MAX );
		edges[index].face = uint8_t( face );

		int origin = b3FindIndexSlow( tempVertices, vertexCount, edge->origin );
		B3_ASSERT( 0 <= origin && origin <= UINT8_MAX );
		edges[index].origin = uint8_t( origin );

		vertices[origin].edge = uint8_t( index );
	}

	for ( int index = 0; index < faceCount; ++index )
	{
		const b3QHFace* face = tempFaces[index];

		int edge = b3FindIndexSlow( tempEdges, edgeCount, face->edge );
		B3_ASSERT( 0 <= edge && edge <= UINT8_MAX );
		faces[index].edge = uint8_t( edge );
		planes[index] = face->plane;
	}

	b3UpdateHullBounds( hull );
	b3UpdateHullBulkProperties( hull );

	if ( b3IsValidHull( hull ) == false )
	{
		b3DestroyHull( hull );
		return nullptr;
	}

	return hull;
}

b3Hull* b3CloneHull( const b3Hull* hull )
{
	if ( hull == nullptr || b3IsValidHull( hull ) == false )
	{
		return nullptr;
	}

	b3Hull* clone = static_cast<b3Hull*>( b3Alloc( hull->byteCount ) );
	memcpy( clone, hull, hull->byteCount );

	return clone;
}

b3Hull* b3CloneAndTransformHull( const b3Hull* original, b3Transform transform, b3Vec3 scale )
{
	if ( original == nullptr || b3IsValidHull( original ) == false )
	{
		return nullptr;
	}

	// Step 1: clone the hull memory
	b3Hull* hull = static_cast<b3Hull*>( b3Alloc( original->byteCount ) );
	memcpy( hull, original, original->byteCount );

	// Step 2: get a safe scale
	b3Vec3 safeScale = b3SafeScale( scale );

	// Step 3: reverse face winding if necessary
	b3HullHalfEdge* edges = b3GetHullEdgesWrite( hull );
	const b3HullFace* faces = b3GetHullFaces( hull );
	int faceCount = hull->faceCount;
	int vertexCount = hull->vertexCount;

	if ( safeScale.x * safeScale.y * safeScale.z < 0.0f )
	{
		// Hull is reflected so reverse edge winding for each face

		for ( int i = 0; i < faceCount; ++i )
		{
			const b3HullFace* face = faces + i;

			// Pass 1: acquire the previous index of the start index
			uint8_t startEdgeIndex = face->edge;
			uint8_t currentEdgeIndex = startEdgeIndex;
			uint8_t prevEdgeIndex = UINT8_MAX;

			do
			{
				b3HullHalfEdge* edge = edges + currentEdgeIndex;

				if ( edge->next == startEdgeIndex )
				{
					prevEdgeIndex = currentEdgeIndex;
					break;
				}

				currentEdgeIndex = edge->next;
			}
			while ( currentEdgeIndex != startEdgeIndex );

			B3_ASSERT( prevEdgeIndex != UINT8_MAX );

			// Pass 2: change all the next indices to be the previous indices
			currentEdgeIndex = startEdgeIndex;

			do
			{
				b3HullHalfEdge* edge = edges + currentEdgeIndex;
				uint8_t nextIndex = edge->next;
				edge->next = prevEdgeIndex;

				// Swap origin with twin, but only once
				if (currentEdgeIndex < edge->twin)
				{
					b3HullHalfEdge* twin = edges + edge->twin;
					B3_SWAP( edge->origin, twin->origin );
				}

				prevEdgeIndex = currentEdgeIndex;
				currentEdgeIndex = nextIndex;
			}
			while ( currentEdgeIndex != startEdgeIndex );
		}

		// Pass 3: fix vertex edges
		b3HullVertex* vertices = b3GetHullVerticesWrite( hull );

		for (int i = 0; i < vertexCount; ++i)
		{
			b3HullVertex* vertex = vertices + i;
			const b3HullHalfEdge* edge = edges + vertex->edge;
			vertex->edge = edge->twin;
		}
	}

	// Step 4: transform vertices
	b3Matrix3 matrix = b3MakeMatrixFromQuat( transform.q );
	b3Vec3* points = b3GetHullPointsWrite( hull );

	for ( int i = 0; i < vertexCount; ++i )
	{
		points[i] = b3Add( b3MulMV( matrix, b3Mul( safeScale, points[i] ) ), transform.p );
	}

	// Step 5: recompute planes
	b3Plane* planes = b3GetHullPlanesWrite( hull );

	for ( int i = 0; i < faceCount; ++i )
	{
		int count = 0;
		b3Vec3 centroid = b3Vec3_zero;
		b3Vec3 normal = b3Vec3_zero;

		const b3HullFace* face = faces + i;
		uint8_t startEdgeIndex = face->edge;
		uint8_t currentEdgeIndex = startEdgeIndex;

		const b3HullHalfEdge* startEdge = edges + currentEdgeIndex;
		B3_ASSERT( startEdge->face == i );

		B3_ASSERT( startEdge->origin < vertexCount );

		// Use the first vertex as the origin to reduce round-off
		b3Vec3 origin = points[startEdge->origin];

		do
		{
			b3HullHalfEdge* edge = edges + currentEdgeIndex;
			b3HullHalfEdge* twin = edges + edge->twin;
			B3_ASSERT( twin->twin == currentEdgeIndex );

			b3Vec3 v1 = points[edge->origin] - origin;
			b3Vec3 v2 = points[twin->origin] - origin;

			count++;
			centroid += v1;
			normal.x += ( v1.y - v2.y ) * ( v1.z + v2.z );
			normal.y += ( v1.z - v2.z ) * ( v1.x + v2.x );
			normal.z += ( v1.x - v2.x ) * ( v1.y + v2.y );

			currentEdgeIndex = edge->next;
		}
		while ( currentEdgeIndex != startEdgeIndex );

		B3_ASSERT( count > 0 );
		centroid *= 1.0f / float( count );
		centroid += origin;

		float area = b3Length( normal );
		B3_ASSERT( area > float( 0 ) );
		normal *= 1.0f / area;

		planes[i] = b3MakePlaneFromNormalAndPoint( normal, centroid );
	}

	b3UpdateHullBounds( hull );
	b3UpdateHullBulkProperties( hull );

	B3_VALIDATE(b3IsValidHull( hull ));

	return hull;
}

void b3DestroyHull( b3Hull* hull )
{
	b3Free( hull, hull->byteCount );
}

b3MassData b3ComputeHullMass( const b3Hull* shape, float density )
{
	b3MassData out;
	out.mass = density * shape->mass;
	out.center = shape->center;

	// Inertia about the hull origin
	out.inertia = density * shape->centralInertia + b3Steiner( out.mass, out.center );
	return out;
}

b3AABB b3ComputeHullAABB( const b3Hull* shape, b3Transform transform )
{
	return b3AABB_Transform( transform, shape->aabb );
}

b3AABB b3ComputeSweptHullAABB( const b3Hull* shape, b3Transform xf1, b3Transform xf2 )
{
	b3AABB aabb1 = b3AABB_Transform( xf1, shape->aabb );
	b3AABB aabb2 = b3AABB_Transform( xf2, shape->aabb );
	return b3AABB_Union( aabb1, aabb2 );
}

bool b3OverlapHull( const b3Hull* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy )
{
	const b3Vec3* points = b3GetHullPoints( shape );

	b3DistanceInput input;
	input.proxyA = b3ShapeProxy{ points, shape->vertexCount, 0.0f };
	input.proxyB = *proxy;
	input.transformA = shapeTransform;
	input.transformB = b3Transform_identity;
	input.useRadii = true;

	b3SimplexCache cache = {};
	b3DistanceOutput output = b3ShapeDistance( &input, &cache, nullptr, 0 );
	return output.distance < B3_OVERLAP_SLOP;
}

b3CastOutput b3RayCastHull( const b3Hull* shape, const b3RayCastInput* input )
{
	B3_ASSERT( b3IsValidRay( input ) );
	b3CastOutput output = {};

	float lower = 0.0f;
	float upper = input->maxFraction;
	int bestFace = -1;

	const b3Plane* planes = b3GetHullPlanes( shape );

	for ( int faceIndex = 0; faceIndex < shape->faceCount; ++faceIndex )
	{
		b3Plane plane = planes[faceIndex];

		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		float distance = plane.offset - b3Dot( plane.normal, input->origin );
		float denominator = b3Dot( plane.normal, input->translation );

		if ( denominator == 0.0f )
		{
			if ( distance < 0.0f )
			{
				// Parallel and runs outside plane
				return output;
			}
		}
		else
		{
			// todo
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.

			float fraction = distance / denominator;

			if ( denominator < 0.0f )
			{
				if ( fraction > lower )
				{
					bestFace = faceIndex;
					lower = fraction;
				}
			}
			else
			{
				if ( fraction < upper )
				{
					upper = fraction;
				}
			}

			if ( upper < lower )
			{
				// Ray misses
				return output;
			}
		}
	}

	if ( bestFace >= 0 )
	{
		output.point = input->origin + lower * input->translation;
		output.normal = planes[bestFace].normal;
		output.fraction = lower;
		output.hit = true;
	}
	else
	{
		// initial overlap
		output.point = input->origin;
		output.hit = true;
	}

	return output;
}

b3CastOutput b3ShapeCastHull( const b3Hull* shape, const b3ShapeCastInput* input )
{
	const b3Vec3* points = b3GetHullPoints( shape );

	b3ShapeCastPairInput pairInput;
	pairInput.proxyA = { points, shape->vertexCount, 0.0f };
	pairInput.proxyB = input->proxy;
	pairInput.transformA = b3Transform_identity;
	pairInput.transformB = b3Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.maxFraction = input->maxFraction;
	pairInput.canEncroach = input->canEncroach;

	b3CastOutput output = b3ShapeCast( &pairInput );
	return output;
}

int b3CollideMoverAndHull( b3PlaneResult* result, const b3Hull* shape, const b3Capsule* mover )
{
	const b3Vec3* points = b3GetHullPoints( shape );
	b3DistanceInput distanceInput;
	distanceInput.proxyA = { points, shape->vertexCount, 0.0f };
	distanceInput.proxyB = { &mover->center1, 2, mover->radius };
	distanceInput.transformA = b3Transform_identity;
	distanceInput.transformB = b3Transform_identity;
	distanceInput.useRadii = false;

	float totalRadius = mover->radius;

	b3SimplexCache cache = {};
	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, &cache, nullptr, 0 );

	if ( distanceOutput.distance <= totalRadius )
	{
		b3Plane plane = { distanceOutput.normal, totalRadius - distanceOutput.distance };
		*result = b3PlaneResult{ plane, distanceOutput.pointA };
		return 1;
	}

	return 0;
}

b3ShapeExtent b3ComputeHullExtent( const b3Hull* hull, b3Vec3 origin )
{
	const b3Vec3* points = b3GetHullPoints( hull );

	float maxExtentSquared = 0.0f;
	for ( int index = 0; index < hull->vertexCount; ++index )
	{
		b3Vec3 point = points[index];
		maxExtentSquared = b3MaxFloat( maxExtentSquared, b3DistanceSquared( point, origin ) );
	}

	b3ShapeExtent extent = { hull->innerRadius, sqrtf( maxExtentSquared ) };
	return extent;
}

float b3ComputeHullProjectedArea( const b3Hull* hull, b3Vec3 direction )
{
	float area = 0.0f;

	int faceCount = hull->faceCount;
	const b3HullFace* hullFaces = b3GetHullFaces( hull );
	const b3HullHalfEdge* hullEdges = b3GetHullEdges( hull );
	const b3Vec3* hullPoints = b3GetHullPoints( hull );

	for ( int i = 0; i < faceCount; ++i )
	{
		const b3HullFace* face = hullFaces + i;

		int baseEdge = face->edge;
		const b3HullHalfEdge* edge = hullEdges + baseEdge;
		b3Vec3 p1 = hullPoints[edge->origin];

		int edgeIndex = edge->next;
		edge = hullEdges + edgeIndex;
		b3Vec3 p2 = hullPoints[edge->origin];

		edgeIndex = edge->next;

		do
		{
			edge = hullEdges + edgeIndex;
			b3Vec3 p3 = hullPoints[edge->origin];

			b3Vec3 e1 = p2 - p1;
			b3Vec3 e2 = p3 - p1;
			b3Vec3 n = b3Cross( e1, e2 );
			float a = b3Dot( n, direction );
			area += b3MaxFloat( a, 0.0f );

			p2 = p3;
			edgeIndex = edge->next;
		}
		while ( edgeIndex != baseEdge );
	}

	return 0.5f * area;
}

// todo this can be done statically and memcpy and change the dynamic parts
b3TriangleHull b3MakeTriangleHull( b3Vec3 a, b3Vec3 b, b3Vec3 c )
{
	b3TriangleHull triangle;

	// Bounding volume
	b3Vec3 lower = b3Min( a, b3Min( b, c ) );
	b3Vec3 upper = b3Max( a, b3Max( b, c ) );
	triangle.base.aabb = { lower, upper };

	// Mass properties
	triangle.base.mass = 0.0f;
	triangle.base.center = ( 1.0f / 3.0f ) * ( a + b + c );
	triangle.base.centralInertia = {};

	// Vertices
	triangle.base.vertexCount = 3;
	triangle.triangleVertices[0].edge = 0;
	triangle.triangleVertices[1].edge = 2;
	triangle.triangleVertices[2].edge = 4;
	triangle.base.vertexOffset = offsetof( b3TriangleHull, triangleVertices );

	triangle.trianglePositions[0] = a;
	triangle.trianglePositions[1] = b;
	triangle.trianglePositions[2] = c;
	triangle.base.pointOffset = offsetof( b3TriangleHull, trianglePositions );

	// Edges (remember that each edge *must* be followed by its twin!)
	triangle.base.edgeCount = 6;
	triangle.triangleEdges[0] = { 2, 1, 0, 0 }; // face 0 - edge 0
	triangle.triangleEdges[2] = { 4, 3, 1, 0 }; // face 0 - edge 1
	triangle.triangleEdges[4] = { 0, 5, 2, 0 }; // face 0 - edge 2
	triangle.triangleEdges[1] = { 5, 0, 1, 1 }; // face 1 - edge 0
	triangle.triangleEdges[3] = { 1, 2, 2, 1 }; // face 1 - edge 1
	triangle.triangleEdges[5] = { 3, 4, 0, 1 }; // face 1 - edge 2
	triangle.base.edgeOffset = offsetof( b3TriangleHull, triangleEdges );

	// Faces
	b3Vec3 normal = b3Cross( b - a, c - a );
	normal = b3Normalize( normal );
	B3_ASSERT( b3IsNormalized( normal ) );

	triangle.base.faceCount = 2;
	triangle.triangleFaces[0].edge = 0;
	triangle.triangleFaces[1].edge = 1;
	triangle.base.faceOffset = offsetof( b3TriangleHull, triangleFaces );

	triangle.trianglePlanes[0] = b3MakePlaneFromNormalAndPoint( normal, triangle.base.center );
	triangle.trianglePlanes[1] = b3MakePlaneFromNormalAndPoint( -normal, triangle.base.center );
	triangle.base.planeOffset = offsetof( b3TriangleHull, trianglePlanes );

	triangle.base.version = B3_HULL_VERSION;
	triangle.base.byteCount = sizeof( b3TriangleHull );
	triangle.base.surfaceArea = 0.0f;

	return triangle;
}

// Used to make all the static data for box hull
static b3BoxHull b3MakeStaticBoxHull()
{
	b3BoxHull box = {};

	// Vertices
	box.base.vertexCount = 8;
	box.boxVertices[0].edge = 8;
	box.boxVertices[1].edge = 1;
	box.boxVertices[2].edge = 0;
	box.boxVertices[3].edge = 9;
	box.boxVertices[4].edge = 13;
	box.boxVertices[5].edge = 3;
	box.boxVertices[6].edge = 5;
	box.boxVertices[7].edge = 11;
	box.base.vertexOffset = offsetof( b3BoxHull, boxVertices );

	// Edges (remember that each edge *must* be followed by its twin!)
	box.base.edgeCount = 24;
	box.boxEdges[0] = { 2, 1, 2, 0 };
	box.boxEdges[1] = { 17, 0, 1, 5 };
	box.boxEdges[2] = { 4, 3, 1, 0 };
	box.boxEdges[3] = { 20, 2, 5, 3 };
	box.boxEdges[4] = { 6, 5, 5, 0 };
	box.boxEdges[5] = { 23, 4, 6, 4 };
	box.boxEdges[6] = { 0, 7, 6, 0 };
	box.boxEdges[7] = { 18, 6, 2, 2 };
	box.boxEdges[8] = { 10, 9, 0, 1 };
	box.boxEdges[9] = { 21, 8, 3, 5 };
	box.boxEdges[10] = { 12, 11, 3, 1 };
	box.boxEdges[11] = { 16, 10, 7, 2 };
	box.boxEdges[12] = { 14, 13, 7, 1 };
	box.boxEdges[13] = { 19, 12, 4, 4 };
	box.boxEdges[14] = { 8, 15, 4, 1 };
	box.boxEdges[15] = { 22, 14, 0, 3 };
	box.boxEdges[16] = { 7, 17, 3, 2 };
	box.boxEdges[17] = { 9, 16, 2, 5 };
	box.boxEdges[18] = { 11, 19, 6, 2 };
	box.boxEdges[19] = { 5, 18, 7, 4 };
	box.boxEdges[20] = { 15, 21, 1, 3 };
	box.boxEdges[21] = { 1, 20, 0, 5 };
	box.boxEdges[22] = { 3, 23, 4, 3 };
	box.boxEdges[23] = { 13, 22, 5, 4 };
	box.base.edgeOffset = offsetof( b3BoxHull, boxEdges );

	// Faces
	box.base.faceCount = 6;
	box.boxFaces[0].edge = 0;
	box.boxFaces[1].edge = 8;
	box.boxFaces[2].edge = 16;
	box.boxFaces[3].edge = 20;
	box.boxFaces[4].edge = 19;
	box.boxFaces[5].edge = 21;
	box.base.faceOffset = offsetof( b3BoxHull, boxFaces );

	// Planes and point offsets
	box.base.planeOffset = offsetof( b3BoxHull, boxPlanes );
	box.base.pointOffset = offsetof( b3BoxHull, boxPoints );

	box.base.version = B3_HULL_VERSION;
	box.base.byteCount = sizeof( b3BoxHull );

	return box;
}

static b3BoxHull s_boxHull = b3MakeStaticBoxHull();

b3BoxHull b3MakeTransformedBoxHull( b3Vec3 halfWidths, b3Transform transform )
{
	// rep movs
	b3BoxHull boxHull = s_boxHull;

	float minH = 0.2f * B3_LINEAR_SLOP;
	b3Vec3 h = b3Max( { minH, minH, minH }, halfWidths );

	// Bounding volume
	boxHull.base.aabb = b3AABB_Transform( transform, { -h, h } );
	boxHull.base.surfaceArea = 8.0f * ( h.x * h.y + h.x * h.z + h.y * h.z );
	boxHull.base.volume = 8.0f * h.x * h.y * h.z;
	boxHull.base.innerRadius = b3MinFloat( h.x, b3MinFloat( h.y, h.z ) );

	// Mass properties
	boxHull.base.mass = 8.0f * h.x * h.y * h.z;
	boxHull.base.center = transform.p;

	b3Matrix3 boxInertia = b3BoxInertia( boxHull.base.mass, -h, h );
	boxHull.base.centralInertia = b3TransformInertia( transform, boxInertia, boxHull.base.mass );

	// Planes
	b3Vec3 lower = -h;
	b3Vec3 upper = h;

	boxHull.boxPlanes[0] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( -b3Vec3_axisX, lower ) );
	boxHull.boxPlanes[1] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( b3Vec3_axisX, upper ) );
	boxHull.boxPlanes[2] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( -b3Vec3_axisY, lower ) );
	boxHull.boxPlanes[3] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( b3Vec3_axisY, upper ) );
	boxHull.boxPlanes[4] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( -b3Vec3_axisZ, lower ) );
	boxHull.boxPlanes[5] = b3TransformPlane( transform, b3MakePlaneFromNormalAndPoint( b3Vec3_axisZ, upper ) );

	boxHull.boxPoints[0] = b3TransformPoint( transform, { h.x, h.y, h.z } );
	boxHull.boxPoints[1] = b3TransformPoint( transform, { -h.x, h.y, h.z } );
	boxHull.boxPoints[2] = b3TransformPoint( transform, { -h.x, -h.y, h.z } );
	boxHull.boxPoints[3] = b3TransformPoint( transform, { h.x, -h.y, h.z } );
	boxHull.boxPoints[4] = b3TransformPoint( transform, { h.x, h.y, -h.z } );
	boxHull.boxPoints[5] = b3TransformPoint( transform, { -h.x, h.y, -h.z } );
	boxHull.boxPoints[6] = b3TransformPoint( transform, { -h.x, -h.y, -h.z } );
	boxHull.boxPoints[7] = b3TransformPoint( transform, { h.x, -h.y, -h.z } );

	return boxHull;
}

b3BoxHull b3MakeBoxHull( b3Vec3 halfWidths )
{
	return b3MakeTransformedBoxHull( halfWidths, b3Transform_identity );
}

void b3ScaleBox( b3Vec3* halfWidths, b3Transform* transform, b3Vec3 postScale, float minHalfWidth )
{
	B3_ASSERT( b3IsValidFloat( minHalfWidth ) && minHalfWidth > 0.0f );

	b3Quat q = transform->q;

	// Adjust rotation to handle reflection
	if ( postScale.x < 0.0f || postScale.y < 0.0f || postScale.z < 0.0f )
	{
		// todo this might be unnecessary if rotation is identity
		// todo compare with polar decomposition (much more expensive)
		// https://github.com/martinbis11/polar-decomposition-3x3
		b3Matrix3 m = b3MakeMatrixFromQuat( q );
		m.cx.x *= postScale.x;
		m.cy.x *= postScale.x;
		m.cz.x *= postScale.x;
		m.cx.y *= postScale.y;
		m.cy.y *= postScale.y;
		m.cz.y *= postScale.y;
		m.cx.z *= postScale.z;
		m.cy.z *= postScale.z;
		m.cz.z *= postScale.z;
		m.cx = b3Normalize( m.cx );
		m.cy = b3Normalize( m.cy );
		m.cz = b3Normalize( m.cz );
		m.cx = postScale.x < 0.0f ? -m.cx : m.cx;
		m.cy = postScale.y < 0.0f ? -m.cy : m.cy;
		m.cz = postScale.z < 0.0f ? -m.cz : m.cz;
		q = b3MakeQuatFromMatrix( &m );
	}

	b3Vec3 absScale = b3Abs( postScale );

	b3Vec3 h = *halfWidths;
	b3Vec3 p1 = absScale * b3RotateVector( q, -h );
	b3Vec3 p2 = absScale * b3RotateVector( q, h );

	// Convert points back to local space
	b3Vec3 localP1 = b3InvRotateVector( q, p1 );
	b3Vec3 localP2 = b3InvRotateVector( q, p2 );

	// Compute upper and lower bounds in local space
	b3Vec3 lower = b3Min( localP1, localP2 );
	b3Vec3 upper = b3Max( localP1, localP2 );

	// Now get the scaled half width in local space
	b3Vec3 scaledHalfWidth = 0.5f * ( upper - lower );

	b3Vec3 m = { minHalfWidth, minHalfWidth, minHalfWidth };
	*halfWidths = b3Max( scaledHalfWidth, m );
	*transform = { postScale * transform->p, q };
}

// todo use new hull scaling technique
b3BoxHull b3MakeScaledBoxHull( b3Vec3 halfWidths, b3Transform transform, b3Vec3 postScale )
{
	b3Vec3 h = halfWidths;
	b3Transform xf = transform;
	b3ScaleBox( &h, &xf, postScale, 4.0f * B3_LINEAR_SLOP );
	return b3MakeTransformedBoxHull( h, xf );
}
