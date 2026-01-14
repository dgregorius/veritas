//--------------------------------------------------------------------------------------------------
/*
	@file		hull.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/array.h"
#include "ragnarok/common/math.h"
#include "ragnarok/common/resource.h"


//--------------------------------------------------------------------------------------------------
// RkVertex
//--------------------------------------------------------------------------------------------------
struct RkVertex
	{
	uint8 Edge;
	};


//--------------------------------------------------------------------------------------------------
// RkHalfEdge
//--------------------------------------------------------------------------------------------------
struct RkHalfEdge
	{
	uint8 Next;
	uint8 Twin;
	uint8 Origin;
	uint8 Face;
	};


//--------------------------------------------------------------------------------------------------
// RkFace
//--------------------------------------------------------------------------------------------------
struct RkFace
	{
	uint8 Edge;
	};


//--------------------------------------------------------------------------------------------------
// RkHull
//--------------------------------------------------------------------------------------------------
struct RkHull
	{
	// Bounding volume
	RkBounds3 Bounds;

	// Mass properties (normalized => density = 1)
	float Mass;
	RkVector3 Center;
	RkMatrix3 Inertia;
	
	// Geometry
	int32 VertexCount;
	RkResourcePointer< RkVertex > VertexList;
	RkResourcePointer< RkVector3 > PositionList;
	int32 EdgeCount;
	RkResourcePointer< RkHalfEdge > EdgeList;
	int32 FaceCount;
	RkResourcePointer< RkFace > FaceList;
	RkResourcePointer< RkPlane3 > PlaneList;

	RkVector3 GetPosition( int Index ) const;
	const RkVertex* GetVertex( int Index ) const;
	int GetVertexIndex( const RkVertex* Vertex ) const;
	const RkHalfEdge* GetEdge( int Index ) const;
	int GetEdgeIndex( const RkHalfEdge* Edge ) const;
	RkPlane3 GetPlane( int Index ) const;
	const RkFace* GetFace( int Index ) const;
	int GetFaceIndex( const RkFace* Face ) const;

	int FindSupportFace( const RkVector3& Direction ) const;
	int FindSupportVertex( const RkVector3& Direction ) const;

	size_t GetSize() const;
	bool IsConsistent() const;
	};

// Hull utilities
RkHull* rkCreateBox( const RkVector3& Extent );
RkHull* rkCreateBox( const RkVector3& Center, const RkVector3& Extent );
RkHull* rkCreateCylinder( float Height, float Radius, int Slices = 16 );
RkHull* rkCreateCone( float Height, float Radius1, float Radius2 = 0.0f, int Slices = 16 );
RkHull* rkCreateConvex( float Radius, int PointCount = 8 );
RkHull* rkCreateHull( int VertexCount, const RkVector3* VertexBase );
RkHull* rkCloneHull( const RkHull* Hull );
void rkDestroyHull( RkHull*& Hull );


//--------------------------------------------------------------------------------------------------
// RkTriangleHull
//--------------------------------------------------------------------------------------------------
struct RkTriangleHull : public RkHull
	{
	RkTriangleHull( const RkVector3& A, const RkVector3& B, const RkVector3& C );
		
	RkVertex TriangleVertices[ 3 ];
	RkVector3 TrianglePositions[ 3 ];
	RkHalfEdge TriangleEdges[ 6 ];
	RkFace TriangleFaces[ 2 ];
	RkPlane3 TrianglePlanes[ 2 ];
	};


#include "hull.inl"