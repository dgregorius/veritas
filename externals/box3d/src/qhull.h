// SPDX-FileCopyrightText: 2025 Dirk Gregorius
// SPDX-License-Identifier: MIT

#pragma once

#include "pool.h"
#include "stack_array.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

struct b3QHVertex;
struct b3QHHalfEdge;
struct b3QHFace;

struct b3QHVertex
{
	b3QHVertex* prev;
	b3QHVertex* next;

	int mark;
	b3Vec3 position;
	b3QHFace* conflictFace;

	// Index into original vertex array used to construct hull
	int userIndex;
};

struct b3QHHalfEdge
{
	b3QHHalfEdge* prev;
	b3QHHalfEdge* next;

	b3QHVertex* origin;
	b3QHFace* face;
	b3QHHalfEdge* twin;
};

bool b3IsEdgeConvex( const b3QHHalfEdge* edge, float tolerance );
bool b3IsEdgeConcave( const b3QHHalfEdge* edge, float tolerance );

struct b3QHFace
{
	b3QHFace* prev;
	b3QHFace* next;
	b3QHHalfEdge* edge;

	int mark;
	float area;
	b3Vec3 centroid;
	b3Plane plane;
	bool flipped;

	b3QHVertex conflictList;
};

bool b3IsTriangle( const b3QHFace* face );

class b3HullBuilder
{
public:
	// Construction / Destruction
	b3HullBuilder();
	b3HullBuilder( const b3HullBuilder& ) = delete;
	b3HullBuilder( b3HullBuilder&& ) = delete;
	~b3HullBuilder();

	b3HullBuilder& operator=( const b3HullBuilder& ) = delete;
	b3HullBuilder& operator=( b3HullBuilder&& ) = delete;

	// todo pass in arena allocator
	void Construct( const b3Vec3* points, int pointCount );
	bool IsConsistent() const;

	// Accessors / Mutators
	int GetVertexCount() const;
	int GetHalfEdgeCount() const;
	int GetFaceCount() const;

	const b3QHVertex& GetVertexList() const;
	const b3QHFace& GetFaceList() const;

	// Memory management
	void AllocateMemory( int vertexCount );
	b3QHVertex* CreateVertex( const b3Vec3& position, int index );
	void DestroyVertex( b3QHVertex* vertex );
	b3QHFace* CreateFace( b3QHVertex* vertex1, b3QHVertex* vertex2, b3QHVertex* vertex3 );
	void DestroyFace( b3QHFace* face );

	// Implementation
	void ComputeTolerance( int pointCount, const b3Vec3* points );
	bool BuildInitialHull( int pointCount, const b3Vec3* points );
	b3QHVertex* NextConflictVertex();
	void AddVertexToHull( b3QHVertex* vertex );
	void CleanHull(b3Vec3 origin);

	void BuildHorizon( b3StackArray<b3QHHalfEdge*, 32>& horizon, b3QHVertex* apex, b3QHFace* seed,
					   b3QHHalfEdge* edge1 = nullptr );
	void BuildCone( b3StackArray<b3QHFace*, 32>& cone, const b3StackArray<b3QHHalfEdge*, 32>& horizon, b3QHVertex* apex );
	void MergeFaces( const b3StackArray<b3QHFace*, 32>& cone );
	void ResolveVertices( const b3StackArray<b3QHFace*, 32>& cone );
	void ResolveFaces( const b3StackArray<b3QHFace*, 32>& cone );

	bool MergeConcave( b3QHFace* face );
	bool MergeCoplanar( b3QHFace* face );
	void ConnectFaces( b3QHHalfEdge* edge );
	void ConnectEdges( b3QHHalfEdge* prev, b3QHHalfEdge* next, b3StackArray<b3QHFace*, 32>& mergedFaces );
	void DestroyEdges( b3QHHalfEdge* begin, b3QHHalfEdge* end );
	void AbsorbFaces( b3QHFace* face, b3StackArray<b3QHFace*, 32>& mergedFaces );

	// Data members
	float m_tolerance;
	float m_minRadius;
	float m_minOutside;

	b3Vec3 m_interiorPoint;
	b3QHVertex m_orphanedList;
	b3QHVertex m_vertexList;
	b3QHFace m_faceList;

	// todo instead of using a pool use an arena allocator passed by the user
	b3Pool<b3QHVertex> m_vertexPool;
	b3Pool<b3QHHalfEdge> m_edgePool;
	b3Pool<b3QHFace> m_facePool;
};

template <typename T> T* b3Begin( T& head )
{
	return head.next;
}

template <typename T> const T* b3Begin( const T& head )
{
	return head.next;
}

template <typename T> T* b3End( T& head )
{
	return &head;
}

template <typename T> const T* b3End( const T& head )
{
	return &head;
}
