//--------------------------------------------------------------------------------------------------
/*
	@file		bvh.h

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
// RkBVHTriangle
//--------------------------------------------------------------------------------------------------
struct RkBVHTriangle
	{
	int32 Index1;
	int32 Index2;
	int32 Index3;
	};

RkBVHTriangle rkMakeTriangle( int Index1, int Index2, int Index3 );


//--------------------------------------------------------------------------------------------------
// RkBVHNode
//--------------------------------------------------------------------------------------------------
struct RK_ALIGN16 RkBVHNode
	{
	float BoundsMin[ 3 ];
	union 
		{
		// Internal node
		struct 
			{
			uint32 Axis : 2;
			uint32 ChildOffset : 30;	
			};

		// Leaf node
		struct 
			{
			uint32 Type : 2;
			uint32 TriangleCount : 30;
			};
		};

	float BoundsMax[ 3 ];
	uint32 TriangleOffset;

	enum { kLeaf = 3 };

	bool IsLeaf() const;
	RkBVHNode* GetLeftChild();
	const RkBVHNode* GetLeftChild() const;
	RkBVHNode* GetRightChild();
	const RkBVHNode* GetRightChild() const;
	RkBounds3 GetBounds() const;
	
	int GetHeight() const;
	};


//--------------------------------------------------------------------------------------------------
// RkBVH
//--------------------------------------------------------------------------------------------------
struct RK_ALIGN16 RkBVH
	{
	RkBounds3 Bounds;

	int32 NodeCount;
	RkResourcePointer< RkBVHNode > NodeList;
	int32 TriangleCount;
	RkResourcePointer< RkBVHTriangle > TriangleList;
	int32 VertexCount;
	RkResourcePointer< RkVector3 > VertexList;
	
	RkBVHNode* GetRoot();
	const RkBVHNode* GetRoot() const;
	const RkBVHTriangle& GetTriangle( int Index ) const;
	const RkVector3& GetVertex( int Index ) const;
	int GetHeight() const;

	size_t GetSize() const;
	bool IsConsistent() const;
	};

// Mesh utilities
RkBVH* rkCreateGrid( int Size, float Scale = 1.0f );
RkBVH* rkCreateTorus( int RadialResolution, int TubularResolution, float Radius, float Thickness );
RkBVH* rkCreateBVH( int VertexCount, const RkVector3* VertexBase, int TriangleCount, const int* IndexBase );
RkBVH* rkCloneBVH( const RkBVH* BVH );
void rkDestroyBVH( RkBVH*& BVH );


#include "bvh.inl"