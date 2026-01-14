//--------------------------------------------------------------------------------------------------
// meshcontact.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "meshcontact.h"
#include "manifold.h"
#include "shape.h"


//--------------------------------------------------------------------------------------------------
// RkMeshContact
//--------------------------------------------------------------------------------------------------
RkMeshContact::RkMeshContact( RkWorld* World, RkShape* Shape1, RkShape* Shape2 )
	: RkContact( RK_MESH_CONTACT, World, Shape1, Shape2 )
	{
	RK_ASSERT( Shape1->GetType() != RK_MESH_SHAPE );
	RK_ASSERT( Shape2->GetType() == RK_MESH_SHAPE );
	}


//--------------------------------------------------------------------------------------------------
void RkMeshContact::Collide()
	{
	// DIRK_TODO: We do not support empty manifolds anymore and expect the following invariant 'bool Touching = mManifoldCount > 0' to hold true
	RK_ASSERT( 0 );
	}