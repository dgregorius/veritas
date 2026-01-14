  //--------------------------------------------------------------------------------------------------
// convexcontact.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "convexcontact.h"
#include "body.h"
#include "manifold.h"

#include "shape.h"
#include "sphereshape.h"
#include "capsuleshape.h"
#include "hullshape.h"


//--------------------------------------------------------------------------------------------------
// Collision matrix
//--------------------------------------------------------------------------------------------------
static void rkCollideSpheres( RkManifold& Manifold, const RkTransform& Transform1, const RkShape* Shape1, const RkTransform& Transform2, const RkShape* Shape2, RkContactCache& Cache )
	{
	// DIRK_TODO: ...
	RK_ASSERT( 0 );
	}


//--------------------------------------------------------------------------------------------------
static void rkCollideSphereCapsule( RkManifold& Manifold, const RkTransform& Transform1, const RkShape* Shape1, const RkTransform& Transform2, const RkShape* Shape2, RkContactCache& Cache )
	{
	// DIRK_TODO: ...
	RK_ASSERT( 0 );
	}


//--------------------------------------------------------------------------------------------------
static void rkCollideCapsules( RkManifold& Manifold, const RkTransform& Transform1, const RkShape* Shape1, const RkTransform& Transform2, const RkShape* Shape2, RkContactCache& Cache )
	{
	// DIRK_TODO: ...
	RK_ASSERT( 0 );
	}


//--------------------------------------------------------------------------------------------------
static void rkCollideSphereHull( RkManifold& Manifold, const RkTransform& Transform1, const RkShape* Shape1, const RkTransform& Transform2, const RkShape* Shape2, RkContactCache& Cache )
	{
	// DIRK_TODO: ...
	RK_ASSERT( 0 );
	}


//--------------------------------------------------------------------------------------------------
static void rkCollideCapsuleHull( RkManifold& Manifold, const RkTransform& Transform1, const RkShape* Shape1, const RkTransform& Transform2, const RkShape* Shape2, RkContactCache& Cache )
	{
	// DIRK_TODO: ...
	RK_ASSERT( 0 );
	}


//--------------------------------------------------------------------------------------------------
static void rkCollideHulls( RkManifold& Manifold, const RkTransform& Transform1, const RkShape* Shape1, const RkTransform& Transform2, const RkShape* Shape2, RkContactCache& Cache )
	{
	RK_ASSERT( Shape1->GetType() == RK_HULL_SHAPE );
	const RkHullShape* HullShape1 = static_cast< const RkHullShape* >( Shape1 );
	RK_ASSERT( Shape2->GetType() == RK_HULL_SHAPE );
	const RkHullShape* HullShape2 = static_cast< const RkHullShape* >( Shape2 );

	RkHull* Hull1 = HullShape1->GetHull();
	RK_ASSERT( Hull1 );
	RkHull* Hull2 = HullShape2->GetHull();
	RK_ASSERT( Hull2 );

	rkCollide( Manifold, Transform1, Hull1, Transform2, Hull2, Cache.SAT );
	}


//--------------------------------------------------------------------------------------------------
static void rkCollide( RkManifold& Manifold, const RkTransform& Transform1, const RkShape* Shape1, const RkTransform& Transform2, const RkShape* Shape2, RkContactCache& Cache )
	{
	// Dynamic dispatch
	typedef void ( *RkCollideFunc )( RkManifold&, const RkTransform&, const RkShape*, const RkTransform&, const RkShape*, RkContactCache& );
	static const RkCollideFunc CollisionMatrix[ RK_SHAPE_TYPE_COUNT ][ RK_SHAPE_TYPE_COUNT ] =
		{
			{ rkCollideSpheres, rkCollideSphereCapsule,  rkCollideSphereHull,	nullptr },
			{ nullptr,          rkCollideCapsules,       rkCollideCapsuleHull,	nullptr },
			{ nullptr,          nullptr,                 rkCollideHulls,		nullptr },
			{ nullptr,			nullptr,			     nullptr,				nullptr }
		};

	RK_ASSERT( Shape1->GetType() <= Shape2->GetType() );
	RkCollideFunc Collide = CollisionMatrix[ Shape1->GetType() ][ Shape2->GetType() ];
	Collide( Manifold, Transform1, Shape1, Transform2, Shape2, Cache );
	}


//--------------------------------------------------------------------------------------------------
// RkConvexContact
//--------------------------------------------------------------------------------------------------
RkConvexContact::RkConvexContact( RkWorld* World, RkShape* Shape1, RkShape* Shape2 )
	: RkContact( RK_CONVEX_CONTACT, World, Shape1, Shape2 )
	{
	RK_ASSERT( Shape1->GetType() != RK_MESH_SHAPE );
	RK_ASSERT( Shape2->GetType() != RK_MESH_SHAPE );
	}


//--------------------------------------------------------------------------------------------------
void RkConvexContact::Collide()
	{
	RK_ASSERT( mBody1->GetType() == RK_DYNAMIC_BODY || mBody2->GetType() == RK_DYNAMIC_BODY );
	
	RkTransform Transform1 = mBody1->GetTransform();
	RkTransform Transform2 = mBody2->GetTransform();

	RK_ASSERT( std::all_of( mManifolds, mManifolds + mManifoldCount, []( const RkManifold& Manifold ) { return Manifold.PointCount > 0; } ) );
	rkCollide( mManifolds[ 0 ], Transform1, mShape1, Transform2, mShape2, mCache );
	mManifoldCount = mManifolds[ 0 ].Empty() ? 0 : 1;
 	}
