//--------------------------------------------------------------------------------------------------
/*
	@file		broadphase.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/bodytype.h"
#include "ragnarok/physics/dynamictree.h"
#include "ragnarok/physics/proxy.h"
#include "ragnarok/physics/world.h"

using RkProxyBuffer = RkArray< RkProxy >;

class RkShape;
class RkWorld;


//--------------------------------------------------------------------------------------------------
// RkBroadphase
//--------------------------------------------------------------------------------------------------
class RkBroadphase
	{
	public:
		// Construction /
		RkBroadphase( RkWorld* World, tf::Executor* Executor );
		
		// World
		RkWorld* GetWorld() const;

		// Tree
		int GetTreeCount() const;
		const RkDynamicTree* GetTree( int TreeIndex ) const;

		// Proxy interfaces
		RkProxy CreateProxy( const RkBounds3& Bounds, RkShape* Shape );
		void MoveProxy( RkProxy Proxy, const RkBounds3& Bounds );
		void RefitProxy( RkProxy Proxy, const RkBounds3& Bounds );
		void TouchProxy( RkProxy Proxy );
		RkShape* DestroyProxy( RkProxy Proxy );
		
		int GetProxyCount() const;
		RkShape* GetProxyShape( RkProxy Proxy ) const;
		RkBounds3 GetProxyBounds( RkProxy Proxy ) const;

		// Queries
		template< typename T >
		void CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, T&& Callback ) const;
		template< typename T >
		void Query( const RkBounds3& Bounds, T&& Callback ) const;

		// Simulation
		template< typename T >
		void Update( RkArray< RkProxyPair >& PairBuffer, T&& Callback );
		
		bool IsDirty() const;
		void Rebuild();

	ragnarok:
		int GetMoveCount() const;
		const RkProxyBuffer& GetMoveBuffer() const;

	private:
		void BufferMove( RkProxy Proxy );
		void UnbufferMove( RkProxy Proxy );

		RkWorld* mWorld;
		tf::Executor* mExecutor;

		RkDynamicTree mTrees[ RK_BODY_TYPE_COUNT ];
		RkProxyBuffer mMoveBuffer;
		RkProxySet mMoveSet;
	};


#include "broadphase.inl"

