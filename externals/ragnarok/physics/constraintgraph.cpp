//--------------------------------------------------------------------------------------------------
// constraintgraph.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "constraintgraph.h"

// Internal
#include "contactsolver.h"
#include "jointsolver.h"

// C++ 20 bit operations
#include <bit>


//--------------------------------------------------------------------------------------------------
// RkConstraintGraph
//--------------------------------------------------------------------------------------------------
int RkConstraintGraph::GetJointCount() const
	{
	int JointCount = 0;
	for ( const RkGraphColor& Color : Colors )
		{
		JointCount += Color.Joints.Size();
		}

	return JointCount;
	}


//--------------------------------------------------------------------------------------------------
void RkConstraintGraph::AddJoint( RkJoint* Joint )
	{
	RK_ASSERT( Joint );
	RK_ASSERT( Joint->GraphIndex < 0 );

	RkBody* Body1 = Joint->GetBody1();
	RkBodyType BodyType1 = Body1->GetType();
	RkBody* Body2 = Joint->GetBody2();
	RkBodyType BodyType2 = Body2->GetType();
	RK_ASSERT( BodyType1 == RK_DYNAMIC_BODY || BodyType2 == RK_DYNAMIC_BODY );
	RK_ASSERT( BodyType1 == RK_DYNAMIC_BODY || Body1->Colors == 0 );
	RK_ASSERT( BodyType2 == RK_DYNAMIC_BODY || Body2->Colors == 0 );

	uint32 UsedColors = Body1->Colors | Body2->Colors;
	int GraphIndex = std::countr_zero( ~UsedColors );
	
	if ( GraphIndex < RK_MAX_GRAPH_COLORS )
		{
		Colors[ GraphIndex ].Joints.PushBack( Joint );
		Joint->GraphIndex = GraphIndex;

		if ( BodyType1 == RK_DYNAMIC_BODY )
			{
			Body1->Colors |= 1 << GraphIndex;
			}
		if ( BodyType2 == RK_DYNAMIC_BODY )
			{
			Body2->Colors |= 1 << GraphIndex;
			}
		}
	else
		{
		// Overflow
		RK_ASSERT( Joint->GraphIndex < 0 );
		mOverflowJoints.PushBack( Joint );
		}
	}


//--------------------------------------------------------------------------------------------------
void RkConstraintGraph::RemoveJoint( RkJoint* Joint )
	{
	RK_ASSERT( Joint );
	RK_ASSERT( Joint->ColorIndex >= 0 );
	
	if ( Joint->GraphIndex >= 0 )
		{
		RkBody* Body1 = Joint->GetBody1();
		RkBodyType BodyType1 = Body1->GetType();
		RkBody* Body2 = Joint->GetBody2();
		RkBodyType BodyType2 = Body2->GetType();

		if ( BodyType2 == RK_DYNAMIC_BODY )
			{
			Body2->Colors &= ~( 1 << Joint->GraphIndex );
			}
		if ( BodyType1 == RK_DYNAMIC_BODY )
			{
			Body1->Colors &= ~( 1 << Joint->GraphIndex );
			}

		RK_ASSERT( Joint->GraphIndex < RK_MAX_GRAPH_COLORS );
		RkGraphColor& Color = Colors[ Joint->GraphIndex ];
		Color.Joints.Remove( Joint );
		Joint->GraphIndex = -1;
		}
	else
		{
		mOverflowJoints.Remove( Joint );
		}
	}


//--------------------------------------------------------------------------------------------------
int RkConstraintGraph::GetContactCount() const
	{
	int ContactCount = 0;
	for ( const RkGraphColor& Color : Colors )
		{
		ContactCount += Color.Contacts.Size();
		}

	return ContactCount;
	}


//--------------------------------------------------------------------------------------------------
void RkConstraintGraph::AddContact( RkContact* Contact )
	{
	RK_ASSERT( Contact );
	RK_ASSERT( Contact->GraphIndex < 0 );

	RkBody* Body1 = Contact->GetBody1();
	RkBodyType BodyType1 = Body1->GetType();
	RkBody* Body2 = Contact->GetBody2();
	RkBodyType BodyType2 = Body2->GetType();
	RK_ASSERT( BodyType1 == RK_DYNAMIC_BODY || BodyType2 == RK_DYNAMIC_BODY );
	RK_ASSERT( BodyType1 == RK_DYNAMIC_BODY || Body1->Colors == 0 );
	RK_ASSERT( BodyType2 == RK_DYNAMIC_BODY || Body2->Colors == 0 );

	if ( BodyType1 == RK_DYNAMIC_BODY && BodyType2 == RK_DYNAMIC_BODY )
		{
		// If both bodies are dynamic find lowest available color
		uint32 AvailableColors = ~( Body1->Colors | Body2->Colors );
		int GraphIndex = std::countr_zero( AvailableColors ); 

		if ( GraphIndex < RK_DYNAMIC_GRAPH_COLORS )
			{
			Colors[ GraphIndex ].Contacts.PushBack( Contact );
			Contact->GraphIndex = GraphIndex;
			Body1->Colors |= 1 << GraphIndex;
			Body2->Colors |= 1 << GraphIndex;

			return;
			}
		}
	else if ( BodyType1 == RK_DYNAMIC_BODY )
		{
		// Otherwise use highest available color
		uint32 AvailableColors = ~Body1->Colors;
		int GraphIndex = std::countl_zero( AvailableColors );
		if ( GraphIndex < RK_STATIC_GRAPH_COLORS )
			{
			// Shift graph index
			GraphIndex = ( RK_MAX_GRAPH_COLORS - 1 ) - GraphIndex;
			Colors[ GraphIndex ].Contacts.PushBack( Contact );
			Contact->GraphIndex = GraphIndex;
			Body1->Colors |= 1 << GraphIndex;

			return;
			}
		}
	else if ( BodyType2 == RK_DYNAMIC_BODY )
		{
		// Otherwise use highest available color
		uint32 AvailableColors = ~Body2->Colors;
		int GraphIndex = std::countl_zero( AvailableColors );
		if ( GraphIndex < RK_STATIC_GRAPH_COLORS )
			{
			// Shift graph index
			GraphIndex = ( RK_MAX_GRAPH_COLORS - 1 ) - GraphIndex;
			Colors[ GraphIndex ].Contacts.PushBack( Contact );
			Contact->GraphIndex = GraphIndex;
			Body2->Colors |= 1 << GraphIndex;

			return;
			}
		}

	// Overflow
	RK_ASSERT( Contact->GraphIndex < 0 );
	mOverflowContacts.PushBack( Contact );
	}


//--------------------------------------------------------------------------------------------------
void RkConstraintGraph::RemoveContact( RkContact* Contact )
	{
	RK_ASSERT( Contact );
	RK_ASSERT( Contact->ColorIndex >= 0 );

	if ( Contact->GraphIndex >= 0 )
		{
		RkBody* Body1 = Contact->GetBody1();
		RkBodyType BodyType1 = Body1->GetType();
		RkBody* Body2 = Contact->GetBody2();
		RkBodyType BodyType2 = Body2->GetType();

		if ( BodyType2 == RK_DYNAMIC_BODY )
			{
			Body2->Colors &= ~( 1 << Contact->GraphIndex );
			}
		if ( BodyType1 == RK_DYNAMIC_BODY )
			{
			Body1->Colors &= ~( 1 << Contact->GraphIndex );
			}

		RK_ASSERT( Contact->GraphIndex < RK_MAX_GRAPH_COLORS );
		RkGraphColor& Color = Colors[ Contact->GraphIndex ];
		Color.Contacts.Remove( Contact );
		Contact->GraphIndex = -1;
		}
	else
		{
		mOverflowContacts.Remove( Contact );
		}
	}


//--------------------------------------------------------------------------------------------------
// RkConstraintBuffer
//--------------------------------------------------------------------------------------------------
void rkAllocConstraintBuffer( RkArena* MainArena, RkConstraintBuffer& ConstraintBuffer, const RkConstraintGraph& ConstraintGraph )
	{
	// Allocate memory
	int JointCount = 0;
	int ContactCount = 0;
	int ManifoldCount = 0;
	
	for ( int ColorIndex = 0; ColorIndex < RK_MAX_GRAPH_COLORS; ++ColorIndex )
		{
		const RkGraphColor& Color = ConstraintGraph.Colors[ ColorIndex ];
		if ( Color.Empty() )
			{
			continue;
			}

		JointCount += Color.GetJointCount();
		ContactCount += Color.GetContactCount();
		ManifoldCount += Color.GetManifoldCount();
		}

	size_t Bytes = 0;
	size_t JointOffset = Bytes = rkAlign( Bytes, alignof( RkJointConstraint ) );
	Bytes += JointCount * sizeof( RkJointConstraint );
	size_t ContactOffset = Bytes = rkAlign( Bytes, alignof( RkContactConstraint ) );
	Bytes += ManifoldCount * sizeof( RkContactConstraint );
	size_t MappingsOffset = Bytes = rkAlign( Bytes, alignof( int ) );
	Bytes += ContactCount * sizeof( int );

	if ( Bytes == 0 )
		{
		return;
		}

	// Fill buffer
	uint8* Buffer = static_cast< uint8* >( MainArena->Allocate( Bytes ) );
	RkJointConstraint* JointIterator = (RkJointConstraint*)rkAddByteOffset( Buffer, JointOffset );
	RK_ASSERT( rkIsAligned( JointIterator, alignof( RkJointConstraint ) ) );
	RkContactConstraint* ContactIterator = (RkContactConstraint*)rkAddByteOffset( Buffer, ContactOffset );
	RK_ASSERT( rkIsAligned( ContactIterator, alignof( RkContactConstraint ) ) );
	int* MappingsIterator = (int*)rkAddByteOffset( Buffer, MappingsOffset );
	RK_ASSERT( rkIsAligned( MappingsIterator, alignof( int ) ) );

	ConstraintBuffer.Memory = Buffer;
	for ( int ColorIndex = 0; ColorIndex < RK_MAX_GRAPH_COLORS; ++ColorIndex )
		{
		const RkGraphColor& Color = ConstraintGraph.Colors[ ColorIndex ];
		if ( Color.Empty() )
			{
			continue;
			}

		int Mapping = 0;
		ConstraintBuffer.JointConstraints[ ColorIndex ] = JointIterator;
		JointIterator += Color.GetJointCount();
		ConstraintBuffer.ContactConstraints[ ColorIndex ] = ContactIterator;
		ContactIterator += Color.GetManifoldCount();
		ConstraintBuffer.ContactMappings[ ColorIndex ] = MappingsIterator;
		for ( const RkContact* Contact : Color.Contacts )
			{
			*MappingsIterator++ = Mapping;
			Mapping += Contact->GetManifoldCount();
			}
		RK_ASSERT( MappingsIterator == ConstraintBuffer.ContactMappings[ ColorIndex ] + Color.Contacts.Size() );
		}
	RK_ASSERT( JointIterator == (RkJointConstraint*)rkAddByteOffset( Buffer, JointOffset ) + JointCount );
	RK_ASSERT( ContactIterator == (RkContactConstraint*)rkAddByteOffset( Buffer, ContactOffset ) + ManifoldCount );
	RK_ASSERT( MappingsIterator == (int*)rkAddByteOffset( Buffer, MappingsOffset ) + ContactCount );
	}


//--------------------------------------------------------------------------------------------------
void rkFreeConstraintBuffer( RkArena* MainArena, RkConstraintBuffer& ConstraintBuffer )
	{
	ConstraintBuffer.Memory = nullptr;
	MainArena->Release();
	}
