// SPDX-FileCopyrightText: 2025 Dirk Gregorius
// SPDX-License-Identifier: MIT

#include "qhull.h"

#include "math_internal.h"
#include "stack_array.h"

#define B3_AXIS_X 0
#define B3_AXIS_Y 1
#define B3_AXIS_Z 2

#define B3_MARK_VISIBLE 0
#define B3_MARK_DELETE 1
#define B3_MARK_CONFIRM 2
#define B3_SQRT3 1.732050808f

template <typename T> int b3Size( const T& head )
{
	int count = 0;
	for ( const T* node = b3Begin( head ); node != b3End( head ); node = node->next )
	{
		count++;
	}

	return count;
}

template <typename T> bool b3Empty( const T& head )
{
	return head.next == &head;
}

template <typename T> void b3Insert( T* node, T* where )
{
	B3_ASSERT( !b3Contains( node ) && b3Contains( where ) );

	node->prev = where->prev;
	node->next = where;

	node->prev->next = node;
	node->next->prev = node;
}

template <typename T> void b3Remove( T* node )
{
	B3_ASSERT( b3Contains( node ) );

	node->prev->next = node->next;
	node->next->prev = node->prev;

	node->prev = nullptr;
	node->next = nullptr;
}

template <typename T> bool b3Contains( T* node )
{
	return node->prev != nullptr && node->next != nullptr;
}

template <typename T> void b3PushFront( T& head, T* node )
{
	b3Insert( node, head.next );
}

template <typename T> T* b3PopFront( T& head )
{
	B3_ASSERT( !b3Empty( head ) );
	T* node = head.next;
	b3Remove( node );

	return node;
}

template <typename T> void b3PushBack( T& head, T* node )
{
	b3Insert( node, head.prev );
}

template <typename T> T* b3PopBack( T& head )
{
	B3_ASSERT( !b3Empty( head ) );
	T* node = head.prev;
	b3Remove( node );

	return node;
}

int b3HullBuilder::GetVertexCount() const
{
	return b3Size( m_vertexList );
}

int b3HullBuilder::GetHalfEdgeCount() const
{
	int count = 0;
	for ( const b3QHFace* face = b3Begin( m_faceList ); face != b3End( m_faceList ); face = face->next )
	{
		b3QHHalfEdge* edge = face->edge;
		B3_ASSERT( edge != nullptr );

		do
		{
			count += 1;
			edge = edge->next;
		}
		while ( edge != face->edge );
	}

	return count;
}

int b3HullBuilder::GetFaceCount() const
{
	return b3Size( m_faceList );
}

const b3QHVertex& b3HullBuilder::GetVertexList() const
{
	return m_vertexList;
}

const b3QHFace& b3HullBuilder::GetFaceList() const
{
	return m_faceList;
}

static b3AABB b3BuildBounds( int vertexCount, const b3Vec3* vertices )
{
	b3AABB bounds = B3_BOUNDS3_EMPTY;
	for ( int i = 0; i < vertexCount; ++i )
	{
		bounds.lowerBound = b3Min( bounds.lowerBound, vertices[i] );
		bounds.upperBound = b3Max( bounds.upperBound, vertices[i] );
	}

	return bounds;
}

static void b3FindFarthestPointsAlongCardinalAxes( int& index1, int& index2, float tolerance, int vertexCount,
												   const b3Vec3* vertexBase )
{
	index1 = index2 = -1;

	b3Vec3 v0 = vertexBase[0];
	b3Vec3 min[3] = { v0, v0, v0 };
	b3Vec3 max[3] = { v0, v0, v0 };

	int minIndex[3] = { 0, 0, 0 };
	int maxIndex[3] = { 0, 0, 0 };

	for ( int i = 1; i < vertexCount; ++i )
	{
		b3Vec3 v = vertexBase[i];

		// X-Axis
		if ( v.x < min[B3_AXIS_X].x )
		{
			min[B3_AXIS_X] = v;
			minIndex[B3_AXIS_X] = i;
		}
		else if ( v.x > max[B3_AXIS_X].x )
		{
			max[B3_AXIS_X] = v;
			maxIndex[B3_AXIS_X] = i;
		}

		// Y-Axis
		if ( v.y < min[B3_AXIS_Y].y )
		{
			min[B3_AXIS_Y] = v;
			minIndex[B3_AXIS_Y] = i;
		}
		else if ( v.y > max[B3_AXIS_Y].y )
		{
			max[B3_AXIS_Y] = v;
			maxIndex[B3_AXIS_Y] = i;
		}

		// Z-Axis
		if ( v.z < min[B3_AXIS_Z].z )
		{
			min[B3_AXIS_Z] = v;
			minIndex[B3_AXIS_Z] = i;
		}
		else if ( v.z > max[B3_AXIS_Z].z )
		{
			max[B3_AXIS_Z] = v;
			maxIndex[B3_AXIS_Z] = i;
		}
	}

	b3Vec3 distance;
	distance.x = max[B3_AXIS_X].x - min[B3_AXIS_X].x;
	distance.y = max[B3_AXIS_Y].y - min[B3_AXIS_Y].y;
	distance.z = max[B3_AXIS_Z].z - min[B3_AXIS_Z].z;

	float distanceArray[3] = { distance.x, distance.y, distance.z };
	int maxElement = b3MaxElementIndex( distance );

	if ( distanceArray[maxElement] > float( 2 ) * tolerance )
	{
		index1 = minIndex[maxElement];
		index2 = maxIndex[maxElement];
	}
}

static int b3FindFarthestPointFromLine( int index1, int index2, float tolerance, int vertexCount, const b3Vec3* vertexBase )
{
	b3Vec3 a = vertexBase[index1];
	b3Vec3 b = vertexBase[index2];

	b3Vec3 ab = b - a;
	float maxDistance = float( 2 ) * tolerance;
	int maxIndex = -1;

	for ( int i = 0; i < vertexCount; ++i )
	{
		if ( i == index1 || i == index2 )
		{
			continue;
		}

		const b3Vec3& p = vertexBase[i];

		b3Vec3 ap = p - a;
		float s = b3Dot( ap, ab ) / b3Dot( ab, ab );
		b3Vec3 q = a + s * ab;

		float distance = b3Distance( p, q );
		if ( distance > maxDistance )
		{
			maxDistance = distance;
			maxIndex = i;
		}
	}

	return maxIndex;
}

static int b3FindFarthestPointFromPlane( int index1, int index2, int index3, float tolerance, int vertexCount,
										 const b3Vec3* vertexBase )
{
	b3Vec3 a = vertexBase[index1];
	b3Vec3 b = vertexBase[index2];
	b3Vec3 c = vertexBase[index3];

	b3Plane plane = b3MakePlaneFromPoints( a, b, c );

	float maxDistance = float( 2 ) * tolerance;
	int maxIndex = -1;

	for ( int i = 0; i < vertexCount; ++i )
	{
		if ( i == index1 || i == index2 || i == index3 )
		{
			continue;
		}

		float distance = b3AbsFloat( b3PlaneSeparation( plane, vertexBase[i] ) );
		if ( distance > maxDistance )
		{
			maxDistance = distance;
			maxIndex = i;
		}
	}

	return maxIndex;
}

bool b3IsEdgeConvex( const b3QHHalfEdge* edge, float tolerance )
{
	float distance = b3PlaneSeparation( edge->face->plane, edge->twin->face->centroid );
	return distance < -tolerance;
}

bool b3IsEdgeConcave( const b3QHHalfEdge* edge, float tolerance )
{
	float distance = b3PlaneSeparation( edge->face->plane, edge->twin->face->centroid );
	return distance > tolerance;
}

static int b3VertexCount( const b3QHFace* face )
{
	int vertexCount = 0;

	const b3QHHalfEdge* edge = face->edge;
	do
	{
		vertexCount++;
		edge = edge->next;
	}
	while ( edge != face->edge );

	return vertexCount;
}

static void b3LinkFace( b3QHFace* face, int index, b3QHHalfEdge* twin )
{
	B3_ASSERT( face != twin->face );

	b3QHHalfEdge* edge = face->edge;
	while ( index-- > 0 )
	{
		B3_ASSERT( edge->face == face );
		edge = edge->next;
	}

	B3_ASSERT( edge != twin );
	edge->twin = twin;
	twin->twin = edge;
}

static void b3LinkFaces( b3QHFace* face1, int index1, b3QHFace* face2, int index2 )
{
	B3_ASSERT( face1 != face2 );

	b3QHHalfEdge* edge1 = face1->edge;
	while ( index1-- > 0 )
	{
		edge1 = edge1->next;
	}

	b3QHHalfEdge* edge2 = face2->edge;
	while ( index2-- > 0 )
	{
		edge2 = edge2->next;
	}

	B3_ASSERT( edge1 != edge2 );
	edge1->twin = edge2;
	edge2->twin = edge1;
}

static void b3NewellPlane( b3QHFace* face )
{
	int count = 0;
	b3Vec3 centroid = b3Vec3_zero;
	b3Vec3 normal = b3Vec3_zero;

	b3QHHalfEdge* edge = face->edge;
	B3_ASSERT( edge->face == face );

	// Use the first vertex as the origin to reduce round-off
	b3Vec3 origin = edge->origin->position;

	do
	{
		b3QHHalfEdge* twin = edge->twin;
		B3_ASSERT( twin->twin == edge );

		b3Vec3 v1 = edge->origin->position - origin;
		b3Vec3 v2 = twin->origin->position - origin;

		count++;
		centroid += v1;
		normal.x += ( v1.y - v2.y ) * ( v1.z + v2.z );
		normal.y += ( v1.z - v2.z ) * ( v1.x + v2.x );
		normal.z += ( v1.x - v2.x ) * ( v1.y + v2.y );

		edge = edge->next;
	}
	while ( edge != face->edge );

	B3_ASSERT( count > 0 );
	centroid *= 1.0f / float( count );
	centroid += origin;

	float area = b3Length( normal );
	B3_ASSERT( area > float( 0 ) );
	normal *= 1.0f / area;

	face->centroid = centroid;
	face->plane = b3MakePlaneFromNormalAndPoint( normal, centroid );
	face->area = area;
}

static bool b3CheckConsistency( const b3QHFace* face )
{
	if ( face->mark == B3_MARK_DELETE )
	{
		// Face is not on the hull
		return false;
	}

	if ( b3VertexCount( face ) < 3 )
	{
		// Invalid geometry
		return false;
	}

	const b3QHHalfEdge* edge = face->edge;

	do
	{
		const b3QHHalfEdge* twin = edge->twin;

		if ( twin == nullptr )
		{
			// Unreflected edge
			return false;
		}
		if ( twin->face == nullptr )
		{
			// Missing face
			return false;
		}
		if ( twin->face == face )
		{
			// Edge is connecting the same face
			return false;
		}
		if ( twin->face->mark == B3_MARK_DELETE )
		{
			// Face is not on hull
			return false;
		}
		if ( twin->twin != edge )
		{
			// Edge reflected incorrectly
			return false;
		}
		if ( edge->next->origin != twin->origin )
		{
			// Topology error
			return false;
		}
		if ( edge->origin != twin->next->origin )
		{
			// Topology error
			return false;
		}
		if ( edge->face != face )
		{
			// Topology error
			return false;
		}

		edge = edge->next;
	}
	while ( edge != face->edge );

	return true;
}

bool b3IsTriangle( const b3QHFace* face )
{
	return b3VertexCount( face ) == 3;
}

b3HullBuilder::b3HullBuilder()
	: m_tolerance( 0 )
	, m_minRadius( 0 )
	, m_minOutside( 0 )
	, m_interiorPoint( b3Vec3_zero )
{
	// List sentinels
	m_orphanedList.prev = &m_orphanedList;
	m_orphanedList.next = &m_orphanedList;

	m_vertexList.prev = &m_vertexList;
	m_vertexList.next = &m_vertexList;

	m_faceList.prev = &m_faceList;
	m_faceList.next = &m_faceList;
}

b3HullBuilder::~b3HullBuilder()
{
	// Destroy faces
	b3QHFace* face = b3Begin( m_faceList );
	while ( face != b3End( m_faceList ) )
	{
		b3QHFace* nuke = face;
		face = face->next;

		b3Remove( nuke );
		DestroyFace( nuke );
	}

	// Destroy vertices
	b3QHVertex* vertex = b3Begin( m_vertexList );
	while ( vertex != b3End( m_vertexList ) )
	{
		b3QHVertex* nuke = vertex;
		vertex = vertex->next;

		b3Remove( nuke );
		DestroyVertex( nuke );
	}
}

void b3HullBuilder::Construct( const b3Vec3* points, int pointCount )
{
	// Validate passed arguments
	if ( pointCount < 4 )
	{
		return;
	}

	b3Vec3 origin = points[0];
	b3StackArray<b3Vec3, 64> shiftedPoints;
	shiftedPoints.Resize( pointCount );
	for (int i = 0; i < pointCount; ++i)
	{
		shiftedPoints[i] = points[i] - origin;
	}

	// Try to build an initial hull
	AllocateMemory( pointCount );
	ComputeTolerance( pointCount, shiftedPoints.m_data );
	if ( !BuildInitialHull( pointCount, shiftedPoints.m_data ) )
	{
		return;
	}

	// Construct hull
	b3QHVertex* vertex = NextConflictVertex();
	while ( vertex )
	{
		AddVertexToHull( vertex );
		vertex = NextConflictVertex();
	}

	CleanHull(origin);
}

bool b3HullBuilder::IsConsistent() const
{
	// Convex polyhedron invariants
	int v = GetVertexCount();
	int e = GetHalfEdgeCount() / 2;
	int f = GetFaceCount();

	// Euler's identity
	if ( v - e + f != 2 )
	{
		return false;
	}

	// Edge and face invariants
	for ( const b3QHFace* face = b3Begin( m_faceList ); face != b3End( m_faceList ); face = face->next )
	{
		// Face invariants (Topology)
		if ( face->edge->face != face )
		{
			return false;
		}

		// Face invariants (Geometry)
		if ( !b3CheckConsistency( face ) )
		{
			return false;
		}

		// 		if ( !qhIsConvex( Face, mMinRadius ) )
		// 			{
		// 			return false;
		// 			}

		if ( b3PlaneSeparation( face->plane, m_interiorPoint ) > 0 )
		{
			return false;
		}

		if ( face->mark != B3_MARK_VISIBLE )
		{
			return false;
		}

		const b3QHHalfEdge* edge = face->edge;

		do
		{
			// DIRK_TODO: Vertex invariant -> each vertex must be connected to > 2 edges...

			// Edge invariants (Topology)
			if ( edge->next->origin != edge->twin->origin )
			{
				return false;
			}
			if ( edge->prev->next != edge )
			{
				return false;
			}

			if ( edge->next->prev != edge )
			{
				return false;
			}

			if ( edge->twin->twin != edge )
			{
				return false;
			}

			if ( edge->face != face )
			{
				return false;
			}

			// Edge invariants (Geometry)
			// 			if ( !Edge->IsConvex( mMinRadius ) )
			// 				{
			// 				return false;
			// 				}

			if ( b3DistanceSquared( edge->origin->position, edge->twin->origin->position ) < 1000.0f * FLT_MIN )
			{
				return false;
			}

			edge = edge->next;
		}
		while ( edge != face->edge );
	}

	return true;
}



void b3HullBuilder::AllocateMemory( int vertexCount )
{
	// Compute maximum feature counts
	int maxVertexCount = vertexCount;
	int maxEdgeCount = 3 * vertexCount - 6;
	int maxFaceCount = 2 * vertexCount - 4;

	// During construction we can temporarily allocate additional features.
	// Also remember we are actually allocating half-edges where H = 2 * E.
	m_vertexPool.Reserve( 2 * maxVertexCount );
	m_edgePool.Reserve( 4 * maxEdgeCount );
	m_facePool.Reserve( 2 * maxFaceCount );
}

b3QHVertex* b3HullBuilder::CreateVertex( const b3Vec3& position, int index )
{
	// 	b3QHVertex* Vertex = (b3QHVertex*)b3Alloc( sizeof( b3QHVertex ), __alignof( b3QHVertex ) );
	// 	new ( Vertex ) b3QHVertex;

	b3QHVertex* vertex = m_vertexPool.Allocate();
	*vertex = {};

	vertex->prev = nullptr;
	vertex->next = nullptr;
	vertex->mark = B3_MARK_CONFIRM;
	vertex->position = position;
	vertex->conflictFace = nullptr;
	vertex->userIndex = index;

	return vertex;
}

void b3HullBuilder::DestroyVertex( b3QHVertex* vertex )
{
	B3_ASSERT( !b3Contains( vertex ) );

	// 	Vertex->~b3QHVertex();
	// 	b3Free( Vertex );

	vertex->~b3QHVertex();
	m_vertexPool.Free( vertex );
}

b3QHFace* b3HullBuilder::CreateFace( b3QHVertex* vertex1, b3QHVertex* vertex2, b3QHVertex* vertex3 )
{
	// 	b3QHFace* Face = (b3QHFace*)b3Alloc( sizeof( b3QHFace ), __alignof( b3QHFace ) );
	// 	new ( Face ) b3QHFace;
	//
	// 	b3QHHalfEdge* Edge1 = (b3QHHalfEdge*)b3Alloc( sizeof( b3QHHalfEdge ), __alignof( b3QHHalfEdge ) );
	// 	b3QHHalfEdge* Edge2 = (b3QHHalfEdge*)b3Alloc( sizeof( b3QHHalfEdge ), __alignof( b3QHHalfEdge ) );
	// 	b3QHHalfEdge* Edge3 = (b3QHHalfEdge*)b3Alloc( sizeof( b3QHHalfEdge ), __alignof( b3QHHalfEdge ) );

	b3QHFace* face = m_facePool.Allocate();
	*face = {};

	b3QHHalfEdge* edge1 = m_edgePool.Allocate();
	b3QHHalfEdge* edge2 = m_edgePool.Allocate();
	b3QHHalfEdge* edge3 = m_edgePool.Allocate();

	b3Vec3 p1 = vertex1->position;
	b3Vec3 p2 = vertex2->position;
	b3Vec3 p3 = vertex3->position;

	b3Plane plane;
	plane.normal = b3Cross( p2 - p1, p3 - p1 );
	float length;
	plane.normal = b3GetLengthAndNormalize( &length, plane.normal );
	plane.offset = b3Dot( plane.normal, p1 );

	float area = 0.5f * length;

	// Initialize face
	face->prev = nullptr;
	face->next = nullptr;

	face->edge = edge1;

	face->mark = B3_MARK_VISIBLE;
	face->area = area;
	face->centroid = ( 1.0f / 3.0f ) * ( vertex1->position + vertex2->position + vertex3->position );
	face->plane = plane;
	face->flipped = b3PlaneSeparation( plane, m_interiorPoint ) > 0;

	face->conflictList.prev = &face->conflictList;
	face->conflictList.next = &face->conflictList;

	// Initialize edges
	edge1->prev = edge3;
	edge1->next = edge2;
	edge1->origin = vertex1;
	edge1->face = face;
	edge1->twin = nullptr;

	edge2->prev = edge1;
	edge2->next = edge3;
	edge2->origin = vertex2;
	edge2->face = face;
	edge2->twin = nullptr;

	edge3->prev = edge2;
	edge3->next = edge1;
	edge3->origin = vertex3;
	edge3->face = face;
	edge3->twin = nullptr;

	return face;
}

void b3HullBuilder::DestroyFace( b3QHFace* face )
{
	B3_ASSERT( !b3Contains( face ) );

	// Edge can be null if face was merged
	b3QHHalfEdge* edge = face->edge;
	if ( edge != nullptr )
	{
		do
		{
			b3QHHalfEdge* nuke = edge;
			edge = edge->next;

			// b3Free( Nuke );
			m_edgePool.Free( nuke );
		}
		while ( edge != face->edge );
	}

	// 	Face->~b3QHFace();
	// 	b3Free( Face );

	face->~b3QHFace();
	m_facePool.Free( face );
}

void b3HullBuilder::ComputeTolerance( int pointCount, const b3Vec3* points )
{
	// Compute the tolerance relative to the object size
	b3AABB bounds = b3BuildBounds( pointCount, points );
	b3Vec3 max = b3Max( b3Abs( bounds.lowerBound ), b3Abs( bounds.upperBound ) );

	float maxSum = max.x + max.y + max.z;
	float maxCoord = b3MaxFloat( max.x, b3MaxFloat( max.y, max.z ) );
	float maxDistance = b3MinFloat( B3_SQRT3 * maxCoord, maxSum );

	float tolerance = ( 3.0f * maxDistance * 1.01f + maxCoord ) * FLT_EPSILON;

	m_tolerance = tolerance;
	m_minRadius = 2.0f * m_tolerance;
	m_minOutside = 2.0f * m_minRadius;
	B3_ASSERT( m_minRadius < m_minOutside + 3.0f * FLT_EPSILON );
}

bool b3HullBuilder::BuildInitialHull( int pointCount, const b3Vec3* points )
{
	int index1, index2;
	b3FindFarthestPointsAlongCardinalAxes( index1, index2, m_tolerance, pointCount, points );
	if ( index1 < 0 || index2 < 0 )
	{
		return false;
	}

	int index3 = b3FindFarthestPointFromLine( index1, index2, m_tolerance, pointCount, points );
	if ( index3 < 0 )
	{
		return false;
	}

	int index4 = b3FindFarthestPointFromPlane( index1, index2, index3, m_tolerance, pointCount, points );
	if ( index4 < 0 )
	{
		return false;
	}

	// Check winding order
	b3Vec3 v1 = points[index1] - points[index4];
	b3Vec3 v2 = points[index2] - points[index4];
	b3Vec3 v3 = points[index3] - points[index4];

	if ( b3ScalarTripleProduct( v1, v2, v3 ) < float( 0 ) )
	{
		int temp = index2;
		index2 = index3;
		index3 = temp;
	}

	// Compute an interior point to detect flipped faces
	m_interiorPoint = b3Vec3_zero;
	m_interiorPoint += points[index1];
	m_interiorPoint += points[index2];
	m_interiorPoint += points[index3];
	m_interiorPoint += points[index4];
	m_interiorPoint *= 0.25f;

	// Allocate initial vertices and save them in the vertex list
	b3QHVertex* vertex1 = CreateVertex( points[index1], index1 );
	b3PushBack( m_vertexList, vertex1 );
	b3QHVertex* vertex2 = CreateVertex( points[index2], index2 );
	b3PushBack( m_vertexList, vertex2 );
	b3QHVertex* vertex3 = CreateVertex( points[index3], index3 );
	b3PushBack( m_vertexList, vertex3 );
	b3QHVertex* vertex4 = CreateVertex( points[index4], index4 );
	b3PushBack( m_vertexList, vertex4 );

	// Allocate initial faces and save them in the face list
	b3QHFace* face1 = CreateFace( vertex1, vertex2, vertex3 );
	b3PushBack( m_faceList, face1 );
	b3QHFace* face2 = CreateFace( vertex4, vertex2, vertex1 );
	b3PushBack( m_faceList, face2 );
	b3QHFace* face3 = CreateFace( vertex4, vertex3, vertex2 );
	b3PushBack( m_faceList, face3 );
	b3QHFace* face4 = CreateFace( vertex4, vertex1, vertex3 );
	b3PushBack( m_faceList, face4 );

	// Link faces
	b3LinkFaces( face1, 0, face2, 1 );
	b3LinkFaces( face1, 1, face3, 1 );
	b3LinkFaces( face1, 2, face4, 1 );

	b3LinkFaces( face2, 0, face3, 2 );
	b3LinkFaces( face3, 0, face4, 2 );
	b3LinkFaces( face4, 0, face2, 2 );

	B3_ASSERT( b3CheckConsistency( face1 ) );
	B3_ASSERT( b3CheckConsistency( face2 ) );
	B3_ASSERT( b3CheckConsistency( face3 ) );
	B3_ASSERT( b3CheckConsistency( face4 ) );

	// Partition vertices into conflict lists
	for ( int index = 0; index < pointCount; ++index )
	{
		if ( index == index1 || index == index2 || index == index3 || index == index4 )
		{
			continue;
		}

		const b3Vec3& point = points[index];

		float maxDistance = m_minOutside;
		b3QHFace* maxFace = nullptr;

		for ( b3QHFace* face = b3Begin( m_faceList ); face != b3End( m_faceList ); face = face->next )
		{
			float distance = b3PlaneSeparation( face->plane, point );
			if ( distance > maxDistance )
			{
				maxDistance = distance;
				maxFace = face;
			}
		}

		if ( maxFace != nullptr )
		{
			b3QHVertex* vertex = CreateVertex( point, index );

			vertex->conflictFace = maxFace;
			b3PushBack( maxFace->conflictList, vertex );
		}
	}

	return true;
}

b3QHVertex* b3HullBuilder::NextConflictVertex()
{
	// Choose the farthest point to avoid adding a lot of nearly coplanar vertices
	b3QHVertex* maxVertex = nullptr;
	float maxDistance = m_minOutside;

	// DIRK_TODO: This can be optimized easily by storing the farthest vertex of a face at the end/beginning of the conflict list
	for ( b3QHFace* face = b3Begin( m_faceList ); face != b3End( m_faceList ); face = face->next )
	{
		if ( !b3Empty( face->conflictList ) )
		{
			for ( b3QHVertex* vertex = b3Begin( face->conflictList ); vertex != b3End( face->conflictList );
				  vertex = vertex->next )
			{
				B3_ASSERT( vertex->conflictFace == face );
				float distance = b3PlaneSeparation( face->plane, vertex->position );

				if ( distance > maxDistance )
				{
					maxDistance = distance;
					maxVertex = vertex;
				}
			}
		}
	}

	return maxVertex;
}

void b3HullBuilder::AddVertexToHull( b3QHVertex* vertex )
{
	// Remove vertex from conflict face
	b3QHFace* face = vertex->conflictFace;
	vertex->conflictFace = nullptr;
	b3Remove( vertex );
	b3PushBack( m_vertexList, vertex );

	// Find the horizon edges
	b3StackArray<b3QHHalfEdge*, 32> horizon;
	BuildHorizon( horizon, vertex, face );
	B3_ASSERT( horizon.m_count >= 3 );

	// Create new cone faces
	b3StackArray<b3QHFace*, 32> cone;
	BuildCone( cone, horizon, vertex );
	B3_ASSERT( cone.m_count >= 3 );

#ifdef QH_DEBUG
	// Push iteration before merging faces
	AddIteration( Vertex, Horizon, Cone, mFaceList );
	int Iteration = mIterations.Size() - 1;
#endif

	// Merge coplanar faces
	MergeFaces( cone );

	// Resolve orphaned vertices
	ResolveVertices( cone );

	// Remove hidden faces and add new ones
	ResolveFaces( cone );
}

void b3HullBuilder::CleanHull(b3Vec3 origin)
{
	// Mark all vertices on the hull as visible
	for ( b3QHFace* face = b3Begin( m_faceList ); face != b3End( m_faceList ); face = face->next )
	{
		b3QHHalfEdge* edge = face->edge;

		do
		{
			edge->origin->mark = B3_MARK_VISIBLE;
			edge = edge->next;
		}
		while ( edge != face->edge );

		face->plane.offset += b3Dot( face->plane.normal, origin );
		face->centroid += origin;
	}

	// Remove unconfirmed vertices
	b3QHVertex* vertex = b3Begin( m_vertexList );
	while ( vertex != b3End( m_vertexList ) )
	{
		b3QHVertex* next = vertex->next;
		if ( vertex->mark != B3_MARK_VISIBLE )
		{
			b3Remove( vertex );
			DestroyVertex( vertex );
		}
		else
		{
			vertex->position += origin;	
		}
		vertex = next;
	}

	m_interiorPoint += origin;
}

void b3HullBuilder::BuildHorizon( b3StackArray<b3QHHalfEdge*, 32>& horizon, b3QHVertex* apex, b3QHFace* seed,
								  b3QHHalfEdge* edge1 )
{
	// Move vertices to orphaned list
	seed->mark = B3_MARK_DELETE;

	b3QHVertex* vertex = b3Begin( seed->conflictList );
	while ( vertex != b3End( seed->conflictList ) )
	{
		b3QHVertex* orphan = vertex;
		vertex = vertex->next;

		orphan->conflictFace = nullptr;
		b3Remove( orphan );

		b3PushBack( m_orphanedList, orphan );
	}
	B3_ASSERT( b3Empty( seed->conflictList ) );

	b3QHHalfEdge* edge;
	if ( edge1 != nullptr )
	{
		edge = edge1->next;
	}
	else
	{
		edge1 = seed->edge;
		edge = edge1;
	}

	do
	{
		b3QHHalfEdge* twin = edge->twin;
		if ( twin->face->mark == B3_MARK_VISIBLE )
		{
			float distance = b3PlaneSeparation( twin->face->plane, apex->position );
			if ( distance > m_minRadius )
			{
				BuildHorizon( horizon, apex, twin->face, twin );
			}
			else
			{
				horizon.PushBack( edge );
			}
		}

		edge = edge->next;
	}
	while ( edge != edge1 );
}

void b3HullBuilder::BuildCone( b3StackArray<b3QHFace*, 32>& cone, const b3StackArray<b3QHHalfEdge*, 32>& horizon,
							   b3QHVertex* apex )
{
	// Create cone faces and link bottom edges to horizon
	for ( int i = 0; i < horizon.m_count; ++i )
	{
		b3QHHalfEdge* edge = horizon[i];
		B3_ASSERT( edge->twin->twin == edge );

		b3QHFace* face = CreateFace( apex, edge->origin, edge->twin->origin );
		cone.PushBack( face );

		// Link face to bottom edge
		b3LinkFace( face, 1, edge->twin );
	}

	// Link new cone faces with each other
	b3QHFace* face1 = cone[cone.m_count - 1];
	for ( int i = 0; i < cone.m_count; ++i )
	{
		b3QHFace* face2 = cone[i];
		b3LinkFaces( face1, 2, face2, 0 );
		face1 = face2;
	}
}

void b3HullBuilder::MergeFaces( const b3StackArray<b3QHFace*, 32>& cone )
{
	// Merge flipped faces
	for ( int i = 0; i < cone.m_count; ++i )
	{
		b3QHFace* face = cone[i];
		if ( face->mark == B3_MARK_VISIBLE )
		{
			if ( face->flipped )
			{
				face->flipped = false;

				float bestArea = 0;
				b3QHHalfEdge* bestEdge = nullptr;

				b3QHHalfEdge* edge = face->edge;

				do
				{
					b3QHHalfEdge* twin = edge->twin;

					float area = twin->face->area;
					if ( area > bestArea )
					{
						bestArea = area;
						bestEdge = edge;
					}

					edge = edge->next;
				}
				while ( edge != face->edge );

				B3_ASSERT( bestEdge != nullptr );
				ConnectFaces( bestEdge );
			}
		}
	}

	// Merge all concave
	for ( int i = 0; i < cone.m_count; ++i )
	{
		b3QHFace* face = cone[i];
		if ( face->mark == B3_MARK_VISIBLE )
		{
			// Merge concave face
			while ( MergeConcave( face ) )
			{
			}
		}
	}

	// Merge all coplanar
	for ( int i = 0; i < cone.m_count; ++i )
	{
		b3QHFace* face = cone[i];
		if ( face->mark == B3_MARK_VISIBLE )
		{
			// Merge coplanar faces
			while ( MergeCoplanar( face ) )
			{
			}
		}
	}
}

void b3HullBuilder::ResolveVertices( const b3StackArray<b3QHFace*, 32>& cone )
{
	// Resolve orphaned vertices
	b3QHVertex* vertex = b3Begin( m_orphanedList );
	while ( vertex != b3End( m_orphanedList ) )
	{
		b3QHVertex* next = vertex->next;
		b3Remove( vertex );

		float maxDistance = m_minOutside;
		b3QHFace* maxFace = nullptr;

		for ( int i = 0; i < cone.m_count; ++i )
		{
			// Skip faces that got merged
			if ( cone[i]->mark == B3_MARK_VISIBLE )
			{
				float distance = b3PlaneSeparation( cone[i]->plane, vertex->position );
				if ( distance > maxDistance )
				{
					maxDistance = distance;
					maxFace = cone[i];
				}
			}
		}

		if ( maxFace != nullptr )
		{
			B3_ASSERT( maxFace->mark == B3_MARK_VISIBLE );
			b3PushBack( maxFace->conflictList, vertex );
			vertex->conflictFace = maxFace;
		}
		else
		{
			// Vertex has been already removed from the orphaned list
			// and can be destroyed
			DestroyVertex( vertex );
			vertex = nullptr;
		}

		vertex = next;
	}

	B3_ASSERT( b3Empty( m_orphanedList ) );
}

void b3HullBuilder::ResolveFaces( const b3StackArray<b3QHFace*, 32>& cone )
{
	// Delete hidden faces
	{
		b3QHFace* face = b3Begin( m_faceList );
		while ( face != b3End( m_faceList ) )
		{
			b3QHFace* nuke = face;
			face = face->next;

			if ( nuke->mark == B3_MARK_DELETE )
			{
				B3_ASSERT( b3Empty( nuke->conflictList ) );

				b3Remove( nuke );
				DestroyFace( nuke );
			}
		}
	}

	// Add new faces
	{
		for ( int i = 0; i < cone.m_count; ++i )
		{
			b3QHFace* face = cone[i];

			// Assert that all concave faces were merged!
			if ( face->mark == B3_MARK_DELETE )
			{
				DestroyFace( face );
				continue;
			}

			b3PushBack( m_faceList, cone[i] );
		}
	}
}

bool b3HullBuilder::MergeConcave( b3QHFace* face )
{
	b3QHHalfEdge* edge = face->edge;

	do
	{
		b3QHHalfEdge* twin = edge->twin;

		if ( b3IsEdgeConcave( edge, m_minRadius ) || b3IsEdgeConcave( twin, m_minRadius ) )
		{
			// Merge
			ConnectFaces( edge );
			return true;
		}

		edge = edge->next;
	}
	while ( edge != face->edge );

	return false;
}

bool b3HullBuilder::MergeCoplanar( b3QHFace* face )
{
	b3QHHalfEdge* edge = face->edge;

	do
	{
		b3QHHalfEdge* twin = edge->twin;

		if ( !b3IsEdgeConvex( edge, m_minRadius ) || !b3IsEdgeConvex( twin, m_minRadius ) )
		{
			ConnectFaces( edge );
			return true;
		}

		edge = edge->next;
	}
	while ( edge != face->edge );

	return false;
}

void b3HullBuilder::ConnectFaces( b3QHHalfEdge* edge )
{
	// The absorbing face
	b3QHFace* face = edge->face;
	B3_ASSERT( b3CheckConsistency( face ) );

	// Find the strip of shared edges
	b3QHHalfEdge* twin = edge->twin;
	B3_ASSERT( b3CheckConsistency( twin->face ) );

	b3QHHalfEdge* edgePrev = edge->prev;
	b3QHHalfEdge* edgeNext = edge->next;
	b3QHHalfEdge* twinPrev = twin->prev;
	b3QHHalfEdge* twinNext = twin->next;

	while ( edgePrev->twin->face == twin->face )
	{
		B3_ASSERT( edgePrev->twin == twinNext );
		B3_ASSERT( twinNext->twin == edgePrev );

		edgePrev = edgePrev->prev;
		twinNext = twinNext->next;
	}
	B3_ASSERT( edgePrev->face != twinNext->face );

	while ( edgeNext->twin->face == twin->face )
	{
		B3_ASSERT( edgeNext->twin == twinPrev );
		B3_ASSERT( twinPrev->twin == edgeNext );

		edgeNext = edgeNext->next;
		twinPrev = twinPrev->prev;
	}
	B3_ASSERT( edgeNext->face != twinPrev->face );

	// Make sure we don't reference a shared edge
	face->edge = edgePrev;

	// Discard opposing face and absorb non-shared edges
	b3StackArray<b3QHFace*, 32> mergedFaces;
	mergedFaces.PushBack( twin->face );
	twin->face->mark = B3_MARK_DELETE;
	twin->face->edge = nullptr;

	for ( b3QHHalfEdge* absorbed = twinNext; absorbed != twinPrev->next; absorbed = absorbed->next )
	{
		absorbed->face = face;
	}

	// Delete shared edges (before connection)
	DestroyEdges( edgePrev->next, edgeNext );
	DestroyEdges( twinPrev->next, twinNext );

	// Connect half edges (this can have side effects)
	ConnectEdges( edgePrev, twinNext, mergedFaces );
	ConnectEdges( twinPrev, edgeNext, mergedFaces );

	// Rebuild geometry for the merges face
	b3NewellPlane( face );
	B3_ASSERT( b3CheckConsistency( face ) );

	// Absorb conflict vertices
	AbsorbFaces( face, mergedFaces );
}

void b3HullBuilder::ConnectEdges( b3QHHalfEdge* prev, b3QHHalfEdge* next, b3StackArray<b3QHFace*, 32>& mergedFaces )
{
	B3_ASSERT( prev != next );
	B3_ASSERT( prev->face == next->face );

	// Check for redundant edges (this has side effects)
	// If this condition holds true both faces are in the same
	// plane since they share three vertices.
	if ( prev->twin->face == next->twin->face )
	{
		// next is redundant and will be removed.
		// It should not be referenced by its associated face!
		if ( next->face->edge == next )
		{
			next->face->edge = prev;
		}

		b3QHHalfEdge* twin;
		if ( b3VertexCount( prev->twin->face ) == 3 )
		{
			twin = next->twin->prev->twin;
			B3_ASSERT( twin->face->mark != B3_MARK_DELETE );

			// If the opposing face is a triangle. We will
			// get rid of it *and* its associated edges
			// (Don't set OpposingFace->Edge = nullptr!)
			b3QHFace* opposingFace = prev->twin->face;
			opposingFace->mark = B3_MARK_DELETE;
			mergedFaces.PushBack( opposingFace );
		}
		else
		{
			twin = next->twin;

			// prev->Twin is redundant and will be removed.
			// It should not be referenced by its associated face!
			if ( twin->face->edge == prev->twin )
			{
				twin->face->edge = twin;
			}

			twin->next = prev->twin->next;
			twin->next->prev = twin;

			// b3Free( prev->Twin );
			m_edgePool.Free( prev->twin );
		}

		prev->next = next->next;
		prev->next->prev = prev;

		prev->twin = twin;
		twin->twin = prev;

		// Destroy the redundant edge and its associated vertex
		b3Remove( next->origin );
		DestroyVertex( next->origin );

		// b3Free( next );
		m_edgePool.Free( next );

		// Twin->Face was modified, so recompute its plane
		b3NewellPlane( twin->face );
	}
	else
	{
		prev->next = next;
		next->prev = prev;
	}
}

void b3HullBuilder::DestroyEdges( b3QHHalfEdge* begin, b3QHHalfEdge* end )
{
	b3QHHalfEdge* edge = begin;
	while ( edge != end )
	{
		b3QHHalfEdge* nuke = edge;
		edge = edge->next;

		// Delete vertex if there is more than one shared edge
		// DIRK_TODO: Since we run over the twin edges as well this would delete the vertex twice!
		// 		if ( Nuke != Begin )
		// 			{
		// 			mVertexList.Remove( Nuke->Origin );
		// 			DestroyVertex( Nuke->Origin );
		// 			}

		// b3Free( Nuke );
		m_edgePool.Free( nuke );
	}
}

void b3HullBuilder::AbsorbFaces( b3QHFace* face, b3StackArray<b3QHFace*, 32>& mergedFaces )
{
	for ( int i = 0; i < mergedFaces.m_count; ++i )
	{
		B3_ASSERT( mergedFaces[i]->mark == B3_MARK_DELETE );
		b3QHVertex& conflictList = mergedFaces[i]->conflictList;

		b3QHVertex* vertex = b3Begin( conflictList );
		while ( vertex != b3End( conflictList ) )
		{
			b3QHVertex* next = vertex->next;
			b3Remove( vertex );

			if ( b3PlaneSeparation( face->plane, vertex->position ) > m_minOutside )
			{
				b3PushBack( face->conflictList, vertex );
				vertex->conflictFace = face;
			}
			else
			{
				b3PushBack( m_orphanedList, vertex );
			}

			vertex = next;
		}

		B3_ASSERT( b3Empty( conflictList ) );
	}
}
