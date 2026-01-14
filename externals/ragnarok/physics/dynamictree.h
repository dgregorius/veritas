//--------------------------------------------------------------------------------------------------
/*
	@file		dynamictree.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/array.h"
#include "ragnarok/common/math.h"
#include "ragnarok/common/pool.h"

// SIMD math
#include <simd.h>


//--------------------------------------------------------------------------------------------------
// Constants
//--------------------------------------------------------------------------------------------------
#define	RK_NULL_NODE -1


//--------------------------------------------------------------------------------------------------
// RkDynamicTreeNode
//--------------------------------------------------------------------------------------------------
struct RK_ALIGN16 RkDynamicTreeNode
	{
	RkVector3 BoundsMin;
	int32 Child1;
	RkVector3 BoundsMax;
	int32 Child2;
	int32 Parent;
	int32 Height;
	bool Dirty;
	void* UserData;
	uint32 Padding[ 2 ];

	inline bool IsLeaf() const
		{
		RK_ASSERT( Child1 != RK_NULL_NODE || Child2 == RK_NULL_NODE );
		return Child1 == RK_NULL_NODE;
		}

	inline RkBounds3 GetBounds() const
		{
		return RkBounds3( BoundsMin, BoundsMax );
		}
	};

static_assert( std::is_pod< RkDynamicTreeNode >::value && sizeof( RkDynamicTreeNode ) == 64 );


//--------------------------------------------------------------------------------------------------
// RkDynamicTreeNode
//--------------------------------------------------------------------------------------------------
class RkDynamicTreeNodePool
	{
	public:
		// Construction / Destruction
		RkDynamicTreeNodePool() = default;
		~RkDynamicTreeNodePool();

		// Memory
		int Size() const;
		int Capactity() const;
		bool Empty() const;

		void Reserve( int Capacity );

		int Alloc();
		void Free( int Index );

		// Accessors
		RkDynamicTreeNode& operator[]( int Index );
		const RkDynamicTreeNode& operator[]( int Index ) const;

	private:
		int mSize = 0;
		int mCapacity = 0;
		RkDynamicTreeNode* mNodes = nullptr;
		int mNext = -1;
	};


//--------------------------------------------------------------------------------------------------
// RkDynamicTree
//--------------------------------------------------------------------------------------------------
class RkDynamicTree
	{
	public:
		// Construction 
		RkDynamicTree();

		// Proxies
		int CreateProxy( const RkBounds3& Bounds, void* UserData = nullptr );
		void MoveProxy( int Proxy, const RkBounds3& Bounds );
		void* DestroyProxy( int Proxy );

		int GetProxyCount() const;
		RkBounds3 GetProxyBounds( int Proxy ) const;
		void* GetProxyUserData( int Proxy ) const;

		// Casting
		template< typename T >
		float CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, float MaxAlpha, T&& Callback ) const;

		// Queries
		template< typename T >
		void Query( const RkBounds3& Bounds, T&& Callback ) const;
		void Query( RkArray< int >& Out, const RkBounds3& Bounds ) const;

		// Statistics
		int GetHeight() const;
		int GetMaxBalance() const;
		float GetArea() const;
		float GetAreaRatio() const;
		RkBounds3 GetBounds() const;

	ragnarok:
		bool IsDirty() const;
		void RefitProxy( int Proxy, const RkBounds3& Bounds );
		void CollectLeaves( RkArray< int >& Leaves );
		void Rebuild();
		int RebuildRecursive( int* First, int* Last, int Parent = -1 );
		RkBounds3 BuildBounds( const int* First, const int* Last ) const;
		
	private:
		// Implementation
		enum { RK_STACK_SIZE = 1024 };
		
		int FindBestSibling( const RkBounds3& Bounds ) const;
		void InsertLeaf( int Leaf, bool ShouldRotate );
		void RemoveLeaf( int Leaf );

		void Rotate( int Node );
		void AdjustAncestors( int Node, bool ShouldRotate );

		void Validate( int Index ) const;

		int mRoot;
		int mProxyCount;
		RkDynamicTreeNodePool mNodes;
		bool mDirty;
	};


#include "dynamictree.inl"