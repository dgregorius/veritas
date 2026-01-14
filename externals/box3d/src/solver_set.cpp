// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver_set.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"

#include <string.h>

void b3DestroySolverSet( b3World* world, int setIndex )
{
	b3SolverSet* set = world->solverSets.Get( setIndex );

	int contactSimCount = set->contactSims.count;
	for (int i = 0; i < contactSimCount; ++i)
	{
		set->contactSims[i].triangleCache.Destroy();
	}

	set->bodySims.Destroy();
	set->bodyStates.Destroy();
	set->contactSims.Destroy();
	set->jointSims.Destroy();
	set->islandSims.Destroy();

	b3FreeId( &world->solverSetIdPool, setIndex );
	*set = {};
	set->setIndex = B3_NULL_INDEX;
}

// Wake a solver set. Does not merge islands.
// Contacts can be in several places:
// 1. non-touching contacts in the disabled set
// 2. non-touching contacts already in the awake set
// 3. touching contacts in the sleeping set
// This handles contact types 1 and 3. Type 2 doesn't need any action.
void b3WakeSolverSet( b3World* world, int setIndex )
{
	B3_ASSERT( setIndex >= b3_firstSleepingSet );
	b3SolverSet* set = world->solverSets.Get( setIndex );
	b3SolverSet* awakeSet = world->solverSets.Get( b3_awakeSet );
	b3SolverSet* disabledSet = world->solverSets.Get( b3_disabledSet );

	b3Body* bodies = world->bodies.data;

	int bodyCount = set->bodySims.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b3BodySim* simSrc = set->bodySims.data + i;

		b3Body* body = bodies + simSrc->bodyId;
		B3_ASSERT( body->setIndex == setIndex );
		body->setIndex = b3_awakeSet;
		body->localIndex = awakeSet->bodySims.count;

		// Reset sleep timer
		body->sleepTime = 0.0f;

		b3BodySim* simDst = awakeSet->bodySims.Add( );
		memcpy( simDst, simSrc, sizeof( b3BodySim ) );

		b3BodyState* state = awakeSet->bodyStates.Add();
		*state = b3_identityBodyState;
		state->flags = body->flags;

		// move non-touching contacts from disabled set to awake set
		int contactKey = body->headContactKey;
		while ( contactKey != B3_NULL_INDEX )
		{
			int edgeIndex = contactKey & 1;
			int contactId = contactKey >> 1;

			b3Contact* contact = world->contacts.Get( contactId );

			contactKey = contact->edges[edgeIndex].nextKey;

			if ( contact->setIndex != b3_disabledSet )
			{
				B3_ASSERT( contact->setIndex == b3_awakeSet || contact->setIndex == setIndex );
				continue;
			}

			int localIndex = contact->localIndex;
			b3ContactSim* contactSim = disabledSet->contactSims.Get(localIndex );

			B3_ASSERT( ( contact->flags & b3_contactTouchingFlag ) == 0 && contactSim->manifoldCount == 0 );

			contact->setIndex = b3_awakeSet;
			contact->localIndex = awakeSet->contactSims.count;
			b3ContactSim* awakeContactSim = awakeSet->contactSims.Add();
			memcpy( awakeContactSim, contactSim, sizeof( b3ContactSim ) );

			int movedLocalIndex = disabledSet->contactSims.RemoveSwap( localIndex );
			if ( movedLocalIndex != B3_NULL_INDEX )
			{
				// fix moved element
				b3ContactSim* movedContactSim = disabledSet->contactSims.data + localIndex;
				b3Contact* movedContact = world->contacts.Get( movedContactSim->contactId );
				B3_ASSERT( movedContact->localIndex == movedLocalIndex );
				movedContact->localIndex = localIndex;
			}
		}
	}

	// transfer touching contacts from sleeping set to contact graph
	{
		int contactCount = set->contactSims.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b3ContactSim* contactSim = set->contactSims.data + i;
			b3Contact* contact = world->contacts.Get( contactSim->contactId );
			B3_ASSERT( contact->flags & b3_contactTouchingFlag );
			B3_ASSERT( contactSim->simFlags & b3_simTouchingFlag );
			B3_ASSERT( contactSim->manifoldCount > 0 );
			B3_ASSERT( contact->setIndex == setIndex );
			b3AddContactToGraph( world, contactSim, contact );

			// Triangle cache got moved, avoid double destroy
			contactSim->triangleCache = {};

			contact->setIndex = b3_awakeSet;
		}
	}

	// transfer joints from sleeping set to awake set
	{
		int jointCount = set->jointSims.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b3JointSim* jointSim = set->jointSims.data + i;
			b3Joint* joint = world->joints.Get(jointSim->jointId );
			B3_ASSERT( joint->setIndex == setIndex );
			b3AddJointToGraph( world, jointSim, joint );
			joint->setIndex = b3_awakeSet;
		}
	}

	// transfer island from sleeping set to awake set
	// Usually a sleeping set has only one island, but it is possible
	// that joints are created between sleeping islands and they
	// are moved to the same sleeping set.
	{
		int islandCount = set->islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b3IslandSim* islandSrc = set->islandSims.data + i;
			b3Island* island = world->islands.Get( islandSrc->islandId );
			island->setIndex = b3_awakeSet;
			island->localIndex = awakeSet->islandSims.count;
			b3IslandSim* islandDst = awakeSet->islandSims.Add( );
			memcpy( islandDst, islandSrc, sizeof( b3IslandSim ) );
		}
	}

	// destroy the sleeping set
	b3DestroySolverSet( world, setIndex );
}

void b3TrySleepIsland( b3World* world, int islandId )
{
	b3Island* island = world->islands.Get( islandId );
	B3_ASSERT( island->setIndex == b3_awakeSet );

	// cannot put an island to sleep while it has a pending split
	if ( island->constraintRemoveCount > 0 )
	{
		return;
	}

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island
	int sleepSetId = b3AllocId( &world->solverSetIdPool );
	if ( sleepSetId == world->solverSets.count )
	{
		b3SolverSet set = {};
		set.setIndex = B3_NULL_INDEX;
		world->solverSets.PushBack(set );
	}

	b3SolverSet* sleepSet = world->solverSets.Get( sleepSetId );
	*sleepSet = {};

	// grab awake set after creating the sleep set because the solver set array may have been resized
	b3SolverSet* awakeSet = world->solverSets.Get( b3_awakeSet );
	B3_ASSERT( 0 <= island->localIndex && island->localIndex < awakeSet->islandSims.count );

	sleepSet->setIndex = sleepSetId;
	sleepSet->bodySims.Create( island->bodyCount );
	sleepSet->contactSims.Create( island->contactCount );
	sleepSet->jointSims.Create( island->jointCount );

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		b3SolverSet* disabledSet = world->solverSets.Get( b3_disabledSet );
		int bodyId = island->headBody;
		while ( bodyId != B3_NULL_INDEX )
		{
			b3Body* body = world->bodies.Get( bodyId );
			B3_ASSERT( body->setIndex == b3_awakeSet );
			B3_ASSERT( body->islandId == islandId );

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			if ( body->bodyMoveIndex != B3_NULL_INDEX )
			{
				b3BodyMoveEvent* moveEvent = world->bodyMoveEvents.Get( body->bodyMoveIndex );
				B3_ASSERT( moveEvent->bodyId.index1 - 1 == bodyId );
				B3_ASSERT( moveEvent->bodyId.generation == body->generation );
				moveEvent->fellAsleep = true;
				body->bodyMoveIndex = B3_NULL_INDEX;
			}

			int awakeBodyIndex = body->localIndex;
			b3BodySim* awakeSim = awakeSet->bodySims.Get( awakeBodyIndex );

			// move body sim to sleep set
			int sleepBodyIndex = sleepSet->bodySims.count;
			b3BodySim* sleepBodySim = sleepSet->bodySims.Add( );
			memcpy( sleepBodySim, awakeSim, sizeof( b3BodySim ) );

			int movedIndex = awakeSet->bodySims.RemoveSwap( awakeBodyIndex );
			if ( movedIndex != B3_NULL_INDEX )
			{
				// fix local index on moved element
				b3BodySim* movedSim = awakeSet->bodySims.data + awakeBodyIndex;
				int movedId = movedSim->bodyId;
				b3Body* movedBody = world->bodies.Get( movedId );
				B3_ASSERT( movedBody->localIndex == movedIndex );
				movedBody->localIndex = awakeBodyIndex;
			}

			// destroy state, no need to clone
			awakeSet->bodyStates.RemoveSwap( awakeBodyIndex );

			body->setIndex = sleepSetId;
			body->localIndex = sleepBodyIndex;

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			int contactKey = body->headContactKey;
			while ( contactKey != B3_NULL_INDEX )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				b3Contact* contact = world->contacts.Get( contactId );

				B3_ASSERT( contact->setIndex == b3_awakeSet || contact->setIndex == b3_disabledSet );
				contactKey = contact->edges[edgeIndex].nextKey;

				if ( contact->setIndex == b3_disabledSet )
				{
					// already moved to disabled set by another body in the island
					continue;
				}

				if ( contact->colorIndex != B3_NULL_INDEX )
				{
					// contact is touching and will be moved separately
					B3_ASSERT( ( contact->flags & b3_contactTouchingFlag ) != 0 );
					continue;
				}

				// the other body may still be awake, it still may go to sleep and then it will be responsible
				// for moving this contact to the disabled set.
				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = contact->edges[otherEdgeIndex].bodyId;
				b3Body* otherBody = world->bodies.Get( otherBodyId );
				if ( otherBody->setIndex == b3_awakeSet )
				{
					continue;
				}

				int localIndex = contact->localIndex;
				b3ContactSim* contactSim = awakeSet->contactSims.Get( localIndex );

				B3_ASSERT( contactSim->manifoldCount == 0 );
				B3_ASSERT( ( contact->flags & b3_contactTouchingFlag ) == 0 );

				// move the non-touching contact to the disabled set
				contact->setIndex = b3_disabledSet;
				contact->localIndex = disabledSet->contactSims.count;
				b3ContactSim* disabledContactSim = disabledSet->contactSims.Add( );
				memcpy( disabledContactSim, contactSim, sizeof( b3ContactSim ) );

				int movedLocalIndex = awakeSet->contactSims.RemoveSwap( localIndex );
				if ( movedLocalIndex != B3_NULL_INDEX )
				{
					// fix moved element
					b3ContactSim* movedContactSim = awakeSet->contactSims.data + localIndex;
					b3Contact* movedContact = world->contacts.Get( movedContactSim->contactId );
					B3_ASSERT( movedContact->localIndex == movedLocalIndex );
					movedContact->localIndex = localIndex;
				}
			}

			bodyId = body->islandNext;
		}
	}

	// move touching contacts
	// this shuffles contacts in the awake set
	{
		int contactId = island->headContact;
		while ( contactId != B3_NULL_INDEX )
		{
			b3Contact* contact = world->contacts.Get( contactId );
			B3_ASSERT( contact->setIndex == b3_awakeSet );
			B3_ASSERT( contact->islandId == islandId );
			int colorIndex = contact->colorIndex;
			B3_ASSERT( 0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT );

			b3GraphColor* color = world->constraintGraph.colors + colorIndex;

			// Remove bodies from graph coloring associated with this constraint
			if ( colorIndex != B3_OVERFLOW_INDEX )
			{
				// might clear a bit for a static body, but this has no effect
				b3ClearBit( &color->bodySet, contact->edges[0].bodyId );
				b3ClearBit( &color->bodySet, contact->edges[1].bodyId );
			}

			int localIndex = contact->localIndex;
			b3ContactSim* awakeContactSim = color->contactSims.Get( localIndex );

			int sleepContactIndex = sleepSet->contactSims.count;
			b3ContactSim* sleepContactSim = sleepSet->contactSims.Add( );
			memcpy( sleepContactSim, awakeContactSim, sizeof( b3ContactSim ) );

			int movedLocalIndex = color->contactSims.RemoveSwap( localIndex );
			if ( movedLocalIndex != B3_NULL_INDEX )
			{
				// fix moved element
				b3ContactSim* movedContactSim = color->contactSims.data + localIndex;
				b3Contact* movedContact = world->contacts.Get( movedContactSim->contactId );
				B3_ASSERT( movedContact->localIndex == movedLocalIndex );
				movedContact->localIndex = localIndex;
			}

			contact->setIndex = sleepSetId;
			contact->colorIndex = B3_NULL_INDEX;
			contact->localIndex = sleepContactIndex;

			contactId = contact->islandNext;
		}
	}

	// move joints
	// this shuffles joints in the awake set
	{
		int jointId = island->headJoint;
		while ( jointId != B3_NULL_INDEX )
		{
			b3Joint* joint = world->joints.Get( jointId );
			B3_ASSERT( joint->setIndex == b3_awakeSet );
			B3_ASSERT( joint->islandId == islandId );
			int colorIndex = joint->colorIndex;
			int localIndex = joint->localIndex;

			B3_ASSERT( 0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT );

			b3GraphColor* color = world->constraintGraph.colors + colorIndex;

			b3JointSim* awakeJointSim = color->jointSims.Get( localIndex );

			if ( colorIndex != B3_OVERFLOW_INDEX )
			{
				// might clear a bit for a static body, but this has no effect
				b3ClearBit( &color->bodySet, joint->edges[0].bodyId );
				b3ClearBit( &color->bodySet, joint->edges[1].bodyId );
			}

			int sleepJointIndex = sleepSet->jointSims.count;
			b3JointSim* sleepJointSim = sleepSet->jointSims.Add( );
			memcpy( sleepJointSim, awakeJointSim, sizeof( b3JointSim ) );

			int movedIndex = color->jointSims.RemoveSwap( localIndex );
			if ( movedIndex != B3_NULL_INDEX )
			{
				// fix moved element
				b3JointSim* movedJointSim = color->jointSims.data + localIndex;
				int movedId = movedJointSim->jointId;
				b3Joint* movedJoint = world->joints.Get( movedId );
				B3_ASSERT( movedJoint->localIndex == movedIndex );
				movedJoint->localIndex = localIndex;
			}

			joint->setIndex = sleepSetId;
			joint->colorIndex = B3_NULL_INDEX;
			joint->localIndex = sleepJointIndex;

			jointId = joint->islandNext;
		}
	}

	// move island struct
	{
		B3_ASSERT( island->setIndex == b3_awakeSet );

		int islandIndex = island->localIndex;
		b3IslandSim* sleepIsland = sleepSet->islandSims.Add( );
		sleepIsland->islandId = islandId;

		int movedIslandIndex = awakeSet->islandSims.RemoveSwap( islandIndex );
		if ( movedIslandIndex != B3_NULL_INDEX )
		{
			// fix index on moved element
			b3IslandSim* movedIslandSim = awakeSet->islandSims.data + islandIndex;
			int movedIslandId = movedIslandSim->islandId;
			b3Island* movedIsland = world->islands.Get( movedIslandId );
			B3_ASSERT( movedIsland->localIndex == movedIslandIndex );
			movedIsland->localIndex = islandIndex;
		}

		island->setIndex = sleepSetId;
		island->localIndex = 0;
	}

	b3ValidateSolverSets( world );
}

// This is called when joints are created between sets. I want to allow the sets
// to continue sleeping if both are asleep. Otherwise one set is waked.
// Islands will get merge when the set is waked.
void b3MergeSolverSets( b3World* world, int setId1, int setId2 )
{
	B3_ASSERT( setId1 >= b3_firstSleepingSet );
	B3_ASSERT( setId2 >= b3_firstSleepingSet );
	b3SolverSet* set1 = world->solverSets.Get( setId1 );
	b3SolverSet* set2 = world->solverSets.Get( setId2 );

	// Move the fewest number of bodies
	if ( set1->bodySims.count < set2->bodySims.count )
	{
		b3SolverSet* tempSet = set1;
		set1 = set2;
		set2 = tempSet;

		int tempId = setId1;
		setId1 = setId2;
		setId2 = tempId;
	}

	// transfer bodies
	{
		b3Body* bodies = world->bodies.data;
		int bodyCount = set2->bodySims.count;
		for ( int i = 0; i < bodyCount; ++i )
		{
			b3BodySim* simSrc = set2->bodySims.data + i;

			b3Body* body = bodies + simSrc->bodyId;
			B3_ASSERT( body->setIndex == setId2 );
			body->setIndex = setId1;
			body->localIndex = set1->bodySims.count;

			b3BodySim* simDst = set1->bodySims.Add( );
			memcpy( simDst, simSrc, sizeof( b3BodySim ) );
		}
	}

	// transfer contacts
	{
		int contactCount = set2->contactSims.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b3ContactSim* contactSrc = set2->contactSims.data + i;

			b3Contact* contact = world->contacts.Get( contactSrc->contactId );
			B3_ASSERT( contact->setIndex == setId2 );
			contact->setIndex = setId1;
			contact->localIndex = set1->contactSims.count;

			b3ContactSim* contactDst = set1->contactSims.Add( );
			memcpy( contactDst, contactSrc, sizeof( b3ContactSim ) );

			// Triangle cache got moved, avoid double destroy
			contactSrc->triangleCache = {};
		}
	}

	// transfer joints
	{
		int jointCount = set2->jointSims.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b3JointSim* jointSrc = set2->jointSims.data + i;

			b3Joint* joint = world->joints.Get( jointSrc->jointId );
			B3_ASSERT( joint->setIndex == setId2 );
			joint->setIndex = setId1;
			joint->localIndex = set1->jointSims.count;

			b3JointSim* jointDst = set1->jointSims.Add();
			memcpy( jointDst, jointSrc, sizeof( b3JointSim ) );
		}
	}

	// transfer islands
	{
		int islandCount = set2->islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b3IslandSim* islandSrc = set2->islandSims.data + i;
			int islandId = islandSrc->islandId;

			b3Island* island = world->islands.Get( islandId );
			island->setIndex = setId1;
			island->localIndex = set1->islandSims.count;

			b3IslandSim* islandDst = set1->islandSims.Add();
			memcpy( islandDst, islandSrc, sizeof( b3IslandSim ) );
		}
	}

	// destroy the merged set
	// Warning: need to be careful not to destroy things that got transferred, like triangle caches.
	b3DestroySolverSet( world, setId2 );

	b3ValidateSolverSets( world );
}

void b3TransferBody( b3World* world, b3SolverSet* targetSet, b3SolverSet* sourceSet, b3Body* body )
{
	if ( targetSet == sourceSet )
	{
		return;
	}

	int sourceIndex = body->localIndex;
	b3BodySim* sourceSim = sourceSet->bodySims.Get( sourceIndex );

	int targetIndex = targetSet->bodySims.count;
	b3BodySim* targetSim = targetSet->bodySims.Add();
	memcpy( targetSim, sourceSim, sizeof( b3BodySim ) );

	// Clear transient body flags
	targetSim->flags &= ~( b3_isFast | b3_isSpeedCapped | b3_hadTimeOfImpact );

	// Remove body sim from solver set that owns it
	int movedIndex = sourceSet->bodySims.RemoveSwap( sourceIndex );
	if ( movedIndex != B3_NULL_INDEX )
	{
		// Fix moved body index
		b3BodySim* movedSim = sourceSet->bodySims.data + sourceIndex;
		int movedId = movedSim->bodyId;
		b3Body* movedBody = world->bodies.Get( movedId );
		B3_ASSERT( movedBody->localIndex == movedIndex );
		movedBody->localIndex = sourceIndex;
	}

	if ( sourceSet->setIndex == b3_awakeSet )
	{
		sourceSet->bodyStates.RemoveSwap( sourceIndex );
	}
	else if ( targetSet->setIndex == b3_awakeSet )
	{
		b3BodyState* state = targetSet->bodyStates.Add();
		*state = b3_identityBodyState;
		state->flags = body->flags;
	}

	body->setIndex = targetSet->setIndex;
	body->localIndex = targetIndex;
}

void b3TransferJoint( b3World* world, b3SolverSet* targetSet, b3SolverSet* sourceSet, b3Joint* joint )
{
	if ( targetSet == sourceSet )
	{
		return;
	}

	int localIndex = joint->localIndex;
	int colorIndex = joint->colorIndex;

	// Retrieve source.
	b3JointSim* sourceSim;
	if ( sourceSet->setIndex == b3_awakeSet )
	{
		B3_ASSERT( 0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT );
		b3GraphColor* color = world->constraintGraph.colors + colorIndex;

		sourceSim = color->jointSims.Get( localIndex );
	}
	else
	{
		B3_ASSERT( colorIndex == B3_NULL_INDEX );
		sourceSim = sourceSet->jointSims.Get(localIndex );
	}

	// Create target and copy. Fix joint.
	if ( targetSet->setIndex == b3_awakeSet )
	{
		b3AddJointToGraph( world, sourceSim, joint );
		joint->setIndex = b3_awakeSet;
	}
	else
	{
		joint->setIndex = targetSet->setIndex;
		joint->localIndex = targetSet->jointSims.count;
		joint->colorIndex = B3_NULL_INDEX;

		b3JointSim* targetSim = targetSet->jointSims.Add();
		memcpy( targetSim, sourceSim, sizeof( b3JointSim ) );
	}

	// Destroy source.
	if ( sourceSet->setIndex == b3_awakeSet )
	{
		b3RemoveJointFromGraph( world, joint->edges[0].bodyId, joint->edges[1].bodyId, colorIndex, localIndex );
	}
	else
	{
		int movedIndex = sourceSet->jointSims.RemoveSwap( localIndex );
		if ( movedIndex != B3_NULL_INDEX )
		{
			// fix swapped element
			b3JointSim* movedJointSim = sourceSet->jointSims.data + localIndex;
			int movedId = movedJointSim->jointId;
			b3Joint* movedJoint = world->joints.Get( movedId );
			movedJoint->localIndex = localIndex;
		}
	}
}
