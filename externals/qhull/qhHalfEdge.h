//--------------------------------------------------------------------------------------------------
/*
	@file		qhHalfEdge.h

	@author		Dirk Gregorius
	@version	0.1
	@date		30/11/2011

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "qhTypes.h"
#include "qhMath.h"
#include "qhList.h"

struct qhVertex;
struct qhHalfEdge;
struct qhFace;

#define QH_MARK_VISIBLE		0
#define QH_MARK_DELETE		1
#define QH_MARK_CONFIRM		2


//--------------------------------------------------------------------------------------------------
// qhVertex
//--------------------------------------------------------------------------------------------------
struct qhVertex 
	{
	qhVertex* Prev;
	qhVertex* Next;

	int Mark;
	qhVector3 Position;
	qhFace* ConflictFace;

	int UserIndex; // Index into original vertex array used to construct hull. (DIRK_TODO: Remove)

	mutable void* UserData = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// qhHalfEdge
//--------------------------------------------------------------------------------------------------
struct qhHalfEdge
	{
	qhHalfEdge* Prev;
	qhHalfEdge* Next;

	qhVertex* Origin;
	qhFace* Face;
	qhHalfEdge* Twin;

	mutable void* UserData = nullptr;

	bool IsConvex( qhReal Tolerance ) const;
	bool IsConcave( qhReal Tolerance ) const;
	};


//--------------------------------------------------------------------------------------------------
// qhFace
//--------------------------------------------------------------------------------------------------
struct qhFace
	{
	qhFace* Prev;
	qhFace* Next;

	qhHalfEdge* Edge;

	int Mark;
	qhReal Area;
	qhVector3 Centroid;
	qhPlane Plane;
	bool Flipped;

	qhList< qhVertex > ConflictList;

	mutable void* UserData = nullptr;

	bool IsTriangle() const;
	};


//--------------------------------------------------------------------------------------------------
// Utilities
//--------------------------------------------------------------------------------------------------
void qhLinkFace( qhFace* Face, int Index, qhHalfEdge* Twin );
void qhLinkFaces( qhFace* Face1, int Index1, qhFace* Face2, int Index2 );
void qhNewellPlane( qhFace* Face );

int qhVertexCount( const qhFace* Face );
bool qhCheckConsistency( const qhFace* Face );
bool qhIsConvex( const qhFace* Face, qhReal Tolerance );




