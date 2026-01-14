//--------------------------------------------------------------------------------------------------
/*
	@file		constraintgraph.h

	@author		Dirk Gregorius
	@version	02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/arena.h"
#include "ragnarok/common/array.h"
#include "ragnarok/common/bitset.h"
#include "ragnarok/common/math.h"
#include "ragnarok/common/tracy.h"

#include "ragnarok/physics/body.h"
#include "ragnarok/physics/constants.h"
#include "ragnarok/physics/contact.h"
#include "ragnarok/physics/joint.h"


//--------------------------------------------------------------------------------------------------
// RkGraphColor
//--------------------------------------------------------------------------------------------------
struct RkGraphColor
	{
	RkIndexedArray< RkJoint, &RkJoint::ColorIndex > Joints;
	RkIndexedArray< RkContact, &RkContact::ColorIndex > Contacts;
	
	int Size() const
		{
		return Joints.Size() + Contacts.Size();
		}

	bool Empty() const
		{
		return Joints.Empty() && Contacts.Empty();
		}

	int GetJointCount() const
		{
		return Joints.Size();
		}

	int GetContactCount() const
		{
		return Contacts.Size();
		}

	int GetManifoldCount() const
		{ 
		// DIRK_TODO: Only true for now until I bring back mesh contacts 
		RK_ASSERT( std::all_of( Contacts.Begin(), Contacts.End(), []( const RkContact* Contact ) { return Contact->GetManifoldCount() == 1; } ) );
		return Contacts.Size();
		}
	};


//--------------------------------------------------------------------------------------------------
// RkConstraintGraph
//--------------------------------------------------------------------------------------------------
struct RkConstraintGraph
	{
	RkGraphColor Colors[ RK_MAX_GRAPH_COLORS ];
	RkIndexedArray< RkJoint, &RkJoint::ColorIndex > mOverflowJoints;
	RkIndexedArray< RkContact, &RkContact::ColorIndex > mOverflowContacts;

	int GetJointCount() const;
	void AddJoint( RkJoint* Joint );
	void RemoveJoint( RkJoint* Joint );

	int GetContactCount() const;
	void AddContact( RkContact* Contact );
	void RemoveContact( RkContact* Contact );
	};


//--------------------------------------------------------------------------------------------------
// RkConstraintBuffer
//--------------------------------------------------------------------------------------------------
struct RkConstraintBuffer
	{
	uint8* Memory = nullptr;
	RkJointConstraint* JointConstraints[ RK_MAX_GRAPH_COLORS ] {};
	RkContactConstraint* ContactConstraints[ RK_MAX_GRAPH_COLORS ] {};
	int* ContactMappings[ RK_MAX_GRAPH_COLORS ] {};

	bool IsValid() const
		{
		return Memory != nullptr;
		}
	};

void rkAllocConstraintBuffer( RkArena* MainArena, RkConstraintBuffer& ConstraintBuffer, const RkConstraintGraph& ConstraintGraph );
void rkFreeConstraintBuffer( RkArena* MainArena, RkConstraintBuffer& ConstraintBuffer );


