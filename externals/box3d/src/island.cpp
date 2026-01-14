// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "island.h"

#include "body.h"
#include "contact.h"
#include "core.h"
#include "joint.h"
#include "physics_world.h"
#include "solver_set.h"

#include <stddef.h>

b3Island* b3CreateIsland( b3World* world, int setIndex )
{
	B3_ASSERT( setIndex == b3_awakeSet || setIndex >= b3_firstSleepingSet );

	int islandId = b3AllocId( &world->islandIdPool );

	if ( islandId == world->islands.count )
	{
		b3Island emptyIsland = {};
		world->islands.PushBack( emptyIsland );
	}
	else
	{
		B3_ASSERT( world->islands.data[islandId].setIndex == B3_NULL_INDEX );
	}

	b3SolverSet* set = world->solverSets.Get( setIndex );

	b3Island* island = world->islands.Get( islandId );
	island->setIndex = setIndex;
	island->localIndex = set->islandSims.count;
	island->islandId = islandId;
	island->headBody = B3_NULL_INDEX;
	island->tailBody = B3_NULL_INDEX;
	island->bodyCount = 0;
	island->headContact = B3_NULL_INDEX;
	island->tailContact = B3_NULL_INDEX;
	island->contactCount = 0;
	island->headJoint = B3_NULL_INDEX;
	island->tailJoint = B3_NULL_INDEX;
	island->jointCount = 0;
	island->constraintRemoveCount = 0;

	b3IslandSim* islandSim = set->islandSims.Add();
	islandSim->islandId = islandId;

	return island;
}

void b3DestroyIsland( b3World* world, int islandId )
{
	if ( world->splitIslandId == islandId )
	{
		world->splitIslandId = B3_NULL_INDEX;
	}

	// assume island is empty
	b3Island* island = world->islands.Get( islandId );
	b3SolverSet* set = world->solverSets.Get( island->setIndex );
	int movedIndex = set->islandSims.RemoveSwap( island->localIndex );
	if ( movedIndex != B3_NULL_INDEX )
	{
		// Fix index on moved element
		b3IslandSim* movedElement = set->islandSims.data + island->localIndex;
		int movedId = movedElement->islandId;
		b3Island* movedIsland = world->islands.Get( movedId );
		B3_ASSERT( movedIsland->localIndex == movedIndex );
		movedIsland->localIndex = island->localIndex;
	}

	// Free island and id (preserve island generation)
	island->islandId = B3_NULL_INDEX;
	island->setIndex = B3_NULL_INDEX;
	island->localIndex = B3_NULL_INDEX;
	b3FreeId( &world->islandIdPool, islandId );
}

static int b3MergeIslands( b3World* world, int islandIdA, int islandIdB )
{
	if ( islandIdA == islandIdB )
	{
		return islandIdA;
	}

	if ( islandIdA == B3_NULL_INDEX )
	{
		B3_ASSERT( islandIdB != B3_NULL_INDEX );
		return islandIdB;
	}

	if ( islandIdB == B3_NULL_INDEX )
	{
		B3_ASSERT( islandIdA != B3_NULL_INDEX );
		return islandIdA;
	}

	b3Island* islandA = world->islands.Get( islandIdA );
	b3Island* islandB = world->islands.Get( islandIdB );

	// Keep the biggest island to reduce cache misses
	b3Island* big;
	b3Island* small;
	if ( islandA->bodyCount >= islandB->bodyCount )
	{
		big = islandA;
		small = islandB;
	}
	else
	{
		big = islandB;
		small = islandA;
	}

	int bigId = big->islandId;

	// remap island indices (cache misses)
	int bodyId = small->headBody;
	while ( bodyId != B3_NULL_INDEX )
	{
		b3Body* body = world->bodies.Get( bodyId );
		body->islandId = bigId;
		bodyId = body->islandNext;
	}

	int contactId = small->headContact;
	while ( contactId != B3_NULL_INDEX )
	{
		b3Contact* contact = world->contacts.Get( contactId );
		contact->islandId = bigId;
		contactId = contact->islandNext;
	}

	int jointId = small->headJoint;
	while ( jointId != B3_NULL_INDEX )
	{
		b3Joint* joint = world->joints.Get( jointId );
		joint->islandId = bigId;
		jointId = joint->islandNext;
	}

	// connect body lists
	B3_ASSERT( big->tailBody != B3_NULL_INDEX );
	b3Body* tailBody = world->bodies.Get( big->tailBody );
	B3_ASSERT( tailBody->islandNext == B3_NULL_INDEX );
	tailBody->islandNext = small->headBody;

	B3_ASSERT( small->headBody != B3_NULL_INDEX );
	b3Body* headBody = world->bodies.Get( small->headBody );
	B3_ASSERT( headBody->islandPrev == B3_NULL_INDEX );
	headBody->islandPrev = big->tailBody;

	big->tailBody = small->tailBody;
	big->bodyCount += small->bodyCount;

	// connect contact lists
	if ( big->headContact == B3_NULL_INDEX )
	{
		// Big island has no contacts
		B3_ASSERT( big->tailContact == B3_NULL_INDEX && big->contactCount == 0 );
		big->headContact = small->headContact;
		big->tailContact = small->tailContact;
		big->contactCount = small->contactCount;
	}
	else if ( small->headContact != B3_NULL_INDEX )
	{
		// Both islands have contacts
		B3_ASSERT( small->tailContact != B3_NULL_INDEX && small->contactCount > 0 );
		B3_ASSERT( big->tailContact != B3_NULL_INDEX && big->contactCount > 0 );

		b3Contact* tailContact = world->contacts.Get( big->tailContact );
		B3_ASSERT( tailContact->islandNext == B3_NULL_INDEX );
		tailContact->islandNext = small->headContact;

		b3Contact* headContact = world->contacts.Get( small->headContact );
		B3_ASSERT( headContact->islandPrev == B3_NULL_INDEX );
		headContact->islandPrev = big->tailContact;

		big->tailContact = small->tailContact;
		big->contactCount += small->contactCount;
	}

	if ( big->headJoint == B3_NULL_INDEX )
	{
		// Root island has no joints
		B3_ASSERT( big->tailJoint == B3_NULL_INDEX && big->jointCount == 0 );
		big->headJoint = small->headJoint;
		big->tailJoint = small->tailJoint;
		big->jointCount = small->jointCount;
	}
	else if ( small->headJoint != B3_NULL_INDEX )
	{
		// Both islands have joints
		B3_ASSERT( small->tailJoint != B3_NULL_INDEX && small->jointCount > 0 );
		B3_ASSERT( big->tailJoint != B3_NULL_INDEX && big->jointCount > 0 );

		b3Joint* tailJoint = world->joints.Get( big->tailJoint );
		B3_ASSERT( tailJoint->islandNext == B3_NULL_INDEX );
		tailJoint->islandNext = small->headJoint;

		b3Joint* headJoint = world->joints.Get( small->headJoint );
		B3_ASSERT( headJoint->islandPrev == B3_NULL_INDEX );
		headJoint->islandPrev = big->tailJoint;

		big->tailJoint = small->tailJoint;
		big->jointCount += small->jointCount;
	}

	// Track removed constraints
	big->constraintRemoveCount += small->constraintRemoveCount;

	small->bodyCount = 0;
	small->contactCount = 0;
	small->jointCount = 0;
	small->headBody = B3_NULL_INDEX;
	small->headContact = B3_NULL_INDEX;
	small->headJoint = B3_NULL_INDEX;
	small->tailBody = B3_NULL_INDEX;
	small->tailContact = B3_NULL_INDEX;
	small->tailJoint = B3_NULL_INDEX;
	small->constraintRemoveCount = 0;

	b3DestroyIsland( world, small->islandId );

	b3ValidateIsland( world, bigId );

	return bigId;
}

static void b3AddContactToIsland( b3World* world, int islandId, b3Contact* contact )
{
	B3_ASSERT( contact->islandId == B3_NULL_INDEX );
	B3_ASSERT( contact->islandPrev == B3_NULL_INDEX );
	B3_ASSERT( contact->islandNext == B3_NULL_INDEX );

	b3Island* island = world->islands.Get( islandId );

	if ( island->headContact != B3_NULL_INDEX )
	{
		contact->islandNext = island->headContact;
		b3Contact* headContact = world->contacts.Get( island->headContact );
		headContact->islandPrev = contact->contactId;
	}

	island->headContact = contact->contactId;
	if ( island->tailContact == B3_NULL_INDEX )
	{
		island->tailContact = island->headContact;
	}

	island->contactCount += 1;
	contact->islandId = islandId;

	b3ValidateIsland( world, islandId );
}

// Link a contact into an island.
void b3LinkContact( b3World* world, b3Contact* contact )
{
	B3_ASSERT( ( contact->flags & b3_contactTouchingFlag ) != 0 );

	int bodyIdA = contact->edges[0].bodyId;
	int bodyIdB = contact->edges[1].bodyId;

	b3Body* bodyA = world->bodies.Get( bodyIdA );
	b3Body* bodyB = world->bodies.Get( bodyIdB );

	B3_ASSERT( bodyA->setIndex != b3_disabledSet && bodyB->setIndex != b3_disabledSet );
	B3_ASSERT( bodyA->setIndex != b3_staticSet || bodyB->setIndex != b3_staticSet );

	// Wake bodyB if bodyA is awake and bodyB is sleeping
	if ( bodyA->setIndex == b3_awakeSet && bodyB->setIndex >= b3_firstSleepingSet )
	{
		b3WakeSolverSet( world, bodyB->setIndex );
	}

	// Wake bodyA if bodyB is awake and bodyA is sleeping
	if ( bodyB->setIndex == b3_awakeSet && bodyA->setIndex >= b3_firstSleepingSet )
	{
		b3WakeSolverSet( world, bodyA->setIndex );
	}

	int islandIdA = bodyA->islandId;
	int islandIdB = bodyB->islandId;

	// Static bodies have null island indices.
	B3_ASSERT( bodyA->setIndex != b3_staticSet || islandIdA == B3_NULL_INDEX );
	B3_ASSERT( bodyB->setIndex != b3_staticSet || islandIdB == B3_NULL_INDEX );
	B3_ASSERT( islandIdA != B3_NULL_INDEX || islandIdB != B3_NULL_INDEX );

	// Merge islands. This will destroy one of the islands.
	int finalIslandId = b3MergeIslands( world, islandIdA, islandIdB );

	// Add contact to the island that survived
	b3AddContactToIsland( world, finalIslandId, contact );
}

// This is called when a contact no longer has contact points or when a contact is destroyed.
void b3UnlinkContact( b3World* world, b3Contact* contact )
{
	B3_ASSERT( contact->islandId != B3_NULL_INDEX );

	// remove from island
	int islandId = contact->islandId;
	b3Island* island = world->islands.Get( islandId );

	if ( contact->islandPrev != B3_NULL_INDEX )
	{
		b3Contact* prevContact = world->contacts.Get( contact->islandPrev );
		B3_ASSERT( prevContact->islandNext == contact->contactId );
		prevContact->islandNext = contact->islandNext;
	}

	if ( contact->islandNext != B3_NULL_INDEX )
	{
		b3Contact* nextContact = world->contacts.Get( contact->islandNext );
		B3_ASSERT( nextContact->islandPrev == contact->contactId );
		nextContact->islandPrev = contact->islandPrev;
	}

	if ( island->headContact == contact->contactId )
	{
		island->headContact = contact->islandNext;
	}

	if ( island->tailContact == contact->contactId )
	{
		island->tailContact = contact->islandPrev;
	}

	B3_ASSERT( island->contactCount > 0 );
	island->contactCount -= 1;
	island->constraintRemoveCount += 1;

	contact->islandId = B3_NULL_INDEX;
	contact->islandPrev = B3_NULL_INDEX;
	contact->islandNext = B3_NULL_INDEX;

	b3ValidateIsland( world, islandId );
}

static void b3AddJointToIsland( b3World* world, int islandId, b3Joint* joint )
{
	B3_ASSERT( joint->islandId == B3_NULL_INDEX );
	B3_ASSERT( joint->islandPrev == B3_NULL_INDEX );
	B3_ASSERT( joint->islandNext == B3_NULL_INDEX );

	b3Island* island = world->islands.Get( islandId );

	if ( island->headJoint != B3_NULL_INDEX )
	{
		joint->islandNext = island->headJoint;
		b3Joint* headJoint = world->joints.Get( island->headJoint );
		headJoint->islandPrev = joint->jointId;
	}

	island->headJoint = joint->jointId;
	if ( island->tailJoint == B3_NULL_INDEX )
	{
		island->tailJoint = island->headJoint;
	}

	island->jointCount += 1;
	joint->islandId = islandId;

	b3ValidateIsland( world, islandId );
}

void b3LinkJoint( b3World* world, b3Joint* joint )
{
	b3Body* bodyA = world->bodies.Get( joint->edges[0].bodyId );
	b3Body* bodyB = world->bodies.Get( joint->edges[1].bodyId );

	B3_ASSERT( bodyA->type == b3_dynamicBody || bodyB->type == b3_dynamicBody );

	if ( bodyA->setIndex == b3_awakeSet && bodyB->setIndex >= b3_firstSleepingSet )
	{
		b3WakeSolverSet( world, bodyB->setIndex );
	}
	else if ( bodyB->setIndex == b3_awakeSet && bodyA->setIndex >= b3_firstSleepingSet )
	{
		b3WakeSolverSet( world, bodyA->setIndex );
	}

	int islandIdA = bodyA->islandId;
	int islandIdB = bodyB->islandId;

	B3_ASSERT( islandIdA != B3_NULL_INDEX || islandIdB != B3_NULL_INDEX );

	// Merge islands. This will destroy one of the islands.
	int finalIslandId = b3MergeIslands( world, islandIdA, islandIdB );

	// Add joint the island that survived
	b3AddJointToIsland( world, finalIslandId, joint );
}

void b3UnlinkJoint( b3World* world, b3Joint* joint )
{
	if ( joint->islandId == B3_NULL_INDEX )
	{
		return;
	}

	// remove from island
	int islandId = joint->islandId;
	b3Island* island = world->islands.Get( islandId );

	if ( joint->islandPrev != B3_NULL_INDEX )
	{
		b3Joint* prevJoint = world->joints.Get( joint->islandPrev );
		B3_ASSERT( prevJoint->islandNext == joint->jointId );
		prevJoint->islandNext = joint->islandNext;
	}

	if ( joint->islandNext != B3_NULL_INDEX )
	{
		b3Joint* nextJoint = world->joints.Get( joint->islandNext );
		B3_ASSERT( nextJoint->islandPrev == joint->jointId );
		nextJoint->islandPrev = joint->islandPrev;
	}

	if ( island->headJoint == joint->jointId )
	{
		island->headJoint = joint->islandNext;
	}

	if ( island->tailJoint == joint->jointId )
	{
		island->tailJoint = joint->islandPrev;
	}

	B3_ASSERT( island->jointCount > 0 );
	island->jointCount -= 1;
	island->constraintRemoveCount += 1;

	joint->islandId = B3_NULL_INDEX;
	joint->islandPrev = B3_NULL_INDEX;
	joint->islandNext = B3_NULL_INDEX;

	b3ValidateIsland( world, islandId );
}

#define B3_CONTACT_REMOVE_THRESHOLD 1

void b3SplitIsland( b3World* world, int baseId )
{
	b3Island* baseIsland = world->islands.Get( baseId );
	int setIndex = baseIsland->setIndex;

	if ( setIndex != b3_awakeSet )
	{
		// can only split awake island
		return;
	}

	if ( baseIsland->constraintRemoveCount == 0 )
	{
		// this island doesn't need to be split
		return;
	}

	b3ValidateIsland( world, baseId );

	int bodyCount = baseIsland->bodyCount;

	b3Body* bodies = world->bodies.data;
	b3ArenaAllocator* alloc = &world->arena;

	// No lock is needed because I ensure the allocator is not used while this task is active.
	int* stack = (int*)b3AllocateArenaItem( alloc, bodyCount * sizeof( int ), "island stack" );
	int* bodyIds = (int*)b3AllocateArenaItem( alloc, bodyCount * sizeof( int ), "body ids" );

	// Build array containing all body indices from base island. These
	// serve as seed bodies for the depth first search (DFS).
	int index = 0;
	int nextBody = baseIsland->headBody;
	while ( nextBody != B3_NULL_INDEX )
	{
		bodyIds[index++] = nextBody;
		b3Body* body = bodies + nextBody;

		nextBody = body->islandNext;
	}
	B3_ASSERT( index == bodyCount );

	// Each island is found as a depth first search starting from a seed body
	for ( int i = 0; i < bodyCount; ++i )
	{
		int seedIndex = bodyIds[i];
		b3Body* seed = bodies + seedIndex;
		B3_ASSERT( seed->setIndex == setIndex );

		if ( seed->islandId != baseId )
		{
			// The body has already been visited
			continue;
		}

		int stackCount = 0;
		stack[stackCount++] = seedIndex;

		// Create new island
		// No lock needed because only a single island can split per time step. No islands are being used during the constraint
		// solve. However, islands are touched during body finalization.
		b3Island* island = b3CreateIsland( world, setIndex );

		int islandId = island->islandId;
		seed->islandId = islandId;

		// Perform a depth first search (DFS) on the constraint graph.
		while ( stackCount > 0 )
		{
			// Grab the next body off the stack and add it to the island.
			int bodyId = stack[--stackCount];
			b3Body* body = bodies + bodyId;
			B3_ASSERT( body->setIndex == b3_awakeSet );
			B3_ASSERT( body->islandId == islandId );

			// Add body to island
			if ( island->tailBody != B3_NULL_INDEX )
			{
				bodies[island->tailBody].islandNext = bodyId;
			}
			body->islandPrev = island->tailBody;
			body->islandNext = B3_NULL_INDEX;
			island->tailBody = bodyId;

			if ( island->headBody == B3_NULL_INDEX )
			{
				island->headBody = bodyId;
			}

			island->bodyCount += 1;

			// Search all contacts connected to this body.
			int contactKey = body->headContactKey;
			while ( contactKey != B3_NULL_INDEX )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				b3Contact* contact = world->contacts.Get( contactId );
				B3_ASSERT( contact->contactId == contactId );

				// Next key
				contactKey = contact->edges[edgeIndex].nextKey;

				// Has this contact already been added to this island?
				if ( contact->islandId == islandId )
				{
					continue;
				}

				// Is this contact enabled and touching?
				if ( ( contact->flags & b3_contactTouchingFlag ) == 0 )
				{
					continue;
				}

				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = contact->edges[otherEdgeIndex].bodyId;
				b3Body* otherBody = bodies + otherBodyId;

				// Maybe add other body to stack
				if ( otherBody->islandId != islandId && otherBody->setIndex != b3_staticSet )
				{
					B3_ASSERT( stackCount < bodyCount );
					stack[stackCount++] = otherBodyId;

					// Need to update the body's island id immediately so it is not traversed again
					otherBody->islandId = islandId;
				}

				// Add contact to island
				contact->islandId = islandId;
				if ( island->tailContact != B3_NULL_INDEX )
				{
					b3Contact* tailContact = world->contacts.Get( island->tailContact );
					tailContact->islandNext = contactId;
				}
				contact->islandPrev = island->tailContact;
				contact->islandNext = B3_NULL_INDEX;
				island->tailContact = contactId;

				if ( island->headContact == B3_NULL_INDEX )
				{
					island->headContact = contactId;
				}

				island->contactCount += 1;
			}

			// Search all joints connect to this body.
			int jointKey = body->headJointKey;
			while ( jointKey != B3_NULL_INDEX )
			{
				int jointId = jointKey >> 1;
				int edgeIndex = jointKey & 1;

				b3Joint* joint = world->joints.Get( jointId );
				B3_ASSERT( joint->jointId == jointId );

				// Next key
				jointKey = joint->edges[edgeIndex].nextKey;

				// Has this joint already been added to this island?
				if ( joint->islandId == islandId )
				{
					continue;
				}

				if ( joint->setIndex == b3_disabledSet )
				{
					continue;
				}

				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = joint->edges[otherEdgeIndex].bodyId;
				b3Body* otherBody = bodies + otherBodyId;

				// Don't simulate joints connected to disabled bodies.
				if ( otherBody->setIndex == b3_disabledSet )
				{
					continue;
				}

				// At least one body must be dynamic
				if ( body->type != b3_dynamicBody && otherBody->type != b3_dynamicBody )
				{
					continue;
				}

				// Maybe add other body to stack
				if ( otherBody->islandId != islandId && otherBody->setIndex == b3_awakeSet )
				{
					B3_ASSERT( stackCount < bodyCount );
					stack[stackCount++] = otherBodyId;

					// Need to update the body's island id immediately so it is not traversed again
					otherBody->islandId = islandId;
				}

				// Add joint to island
				joint->islandId = islandId;
				if ( island->tailJoint != B3_NULL_INDEX )
				{
					b3Joint* tailJoint = world->joints.Get( island->tailJoint );
					tailJoint->islandNext = jointId;
				}
				joint->islandPrev = island->tailJoint;
				joint->islandNext = B3_NULL_INDEX;
				island->tailJoint = jointId;

				if ( island->headJoint == B3_NULL_INDEX )
				{
					island->headJoint = jointId;
				}

				island->jointCount += 1;
			}
		}

		b3ValidateIsland( world, islandId );
	}

	// Done with the base split island. This is delayed because the baseId is used as a marker and it
	// should not be recycled in while splitting.
	b3DestroyIsland( world, baseId );

	b3FreeArenaItem( alloc, bodyIds );
	b3FreeArenaItem( alloc, stack );
}

// Split an island because some contacts and/or joints have been removed.
// This is called during the constraint solve while islands are not being touched. This uses DFS and touches a lot of memory,
// so it can be quite slow.
// Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity
// Note: static bodies are never in an island
// Note: this task interacts with some allocators without locks under the assumption that no other tasks
// are interacting with these data structures.
void b3SplitIslandTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b3TracyCZoneNC( split, "Split Island", b3_colorOlive, true );

	B3_UNUSED( startIndex );
	B3_UNUSED( endIndex );
	B3_UNUSED( threadIndex );

	uint64_t ticks = b3GetTicks();
	b3World* world = (b3World*)context;

	B3_ASSERT( world->splitIslandId != B3_NULL_INDEX );

	b3SplitIsland( world, world->splitIslandId );

	world->profile.splitIslands += b3GetMilliseconds( ticks );
	b3TracyCZoneEnd( split );
}

#if B3_ENABLE_VALIDATION
void b3ValidateIsland( b3World* world, int islandId )
{
	if ( islandId == B3_NULL_INDEX )
	{
		return;
	}

	b3Island* island = world->islands.Get( islandId );
	B3_ASSERT( island->islandId == islandId );
	B3_ASSERT( island->setIndex != B3_NULL_INDEX );
	B3_ASSERT( island->headBody != B3_NULL_INDEX );

	{
		B3_ASSERT( island->tailBody != B3_NULL_INDEX );
		B3_ASSERT( island->bodyCount > 0 );
		if ( island->bodyCount > 1 )
		{
			B3_ASSERT( island->tailBody != island->headBody );
		}
		B3_ASSERT( island->bodyCount <= b3GetIdCount( &world->bodyIdPool ) );

		int count = 0;
		int bodyId = island->headBody;
		while ( bodyId != B3_NULL_INDEX )
		{
			b3Body* body = world->bodies.Get( bodyId );
			B3_ASSERT( body->islandId == islandId );
			B3_ASSERT( body->setIndex == island->setIndex );
			count += 1;

			if ( count == island->bodyCount )
			{
				B3_ASSERT( bodyId == island->tailBody );
			}

			bodyId = body->islandNext;
		}
		B3_ASSERT( count == island->bodyCount );
	}

	if ( island->headContact != B3_NULL_INDEX )
	{
		B3_ASSERT( island->tailContact != B3_NULL_INDEX );
		B3_ASSERT( island->contactCount > 0 );
		if ( island->contactCount > 1 )
		{
			B3_ASSERT( island->tailContact != island->headContact );
		}
		B3_ASSERT( island->contactCount <= b3GetIdCount( &world->contactIdPool ) );

		int count = 0;
		int contactId = island->headContact;
		while ( contactId != B3_NULL_INDEX )
		{
			b3Contact* contact = world->contacts.Get( contactId );
			B3_ASSERT( contact->setIndex == island->setIndex );
			B3_ASSERT( contact->islandId == islandId );
			count += 1;

			if ( count == island->contactCount )
			{
				B3_ASSERT( contactId == island->tailContact );
			}

			contactId = contact->islandNext;
		}
		B3_ASSERT( count == island->contactCount );
	}
	else
	{
		B3_ASSERT( island->tailContact == B3_NULL_INDEX );
		B3_ASSERT( island->contactCount == 0 );
	}

	if ( island->headJoint != B3_NULL_INDEX )
	{
		B3_ASSERT( island->tailJoint != B3_NULL_INDEX );
		B3_ASSERT( island->jointCount > 0 );
		if ( island->jointCount > 1 )
		{
			B3_ASSERT( island->tailJoint != island->headJoint );
		}
		B3_ASSERT( island->jointCount <= b3GetIdCount( &world->jointIdPool ) );

		int count = 0;
		int jointId = island->headJoint;
		while ( jointId != B3_NULL_INDEX )
		{
			b3Joint* joint = world->joints.Get( jointId );
			B3_ASSERT( joint->setIndex == island->setIndex );
			count += 1;

			if ( count == island->jointCount )
			{
				B3_ASSERT( jointId == island->tailJoint );
			}

			jointId = joint->islandNext;
		}
		B3_ASSERT( count == island->jointCount );
	}
	else
	{
		B3_ASSERT( island->tailJoint == B3_NULL_INDEX );
		B3_ASSERT( island->jointCount == 0 );
	}
}

#else

void b3ValidateIsland( b3World* world, int islandId )
{
	B3_UNUSED( world );
	B3_UNUSED( islandId );
}
#endif
