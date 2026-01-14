// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "physics_world.h"

#include "arena_allocator.h"
#include "array.h"
#include "bitset.h"
#include "body.h"
#include "broad_phase.h"
#include "compound.h"
#include "constants.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "ctz.h"
#include "island.h"
#include "joint.h"
#include "platform.h"
#include "sensor.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"

#include "box3d/box3d.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

static_assert( B3_MAX_WORLDS > 0, "must be 1 or more" );
static_assert( B3_MAX_WORLDS < UINT16_MAX, "B3_MAX_WORLDS limit exceeded" );
b3World b3_worlds[B3_MAX_WORLDS];
b3AtomicInt b3_worldCount;
int b3_maxWorldCount;

b3World* b3GetUnlockedWorldFromId( b3WorldId id )
{
	B3_ASSERT( 1 <= id.index1 && id.index1 <= B3_MAX_WORLDS );
	b3World* world = b3_worlds + ( id.index1 - 1 );
	B3_ASSERT( id.index1 == world->worldId + 1 );
	B3_ASSERT( id.generation == world->generation );

	// A world accessed from an id should not be locked
	if ( world->locked )
	{
		B3_ASSERT( false );
		return nullptr;
	}
	return world;
}

b3World* b3GetWorldFromId( b3WorldId id )
{
	B3_ASSERT( 1 <= id.index1 && id.index1 <= B3_MAX_WORLDS );
	b3World* world = b3_worlds + ( id.index1 - 1 );
	B3_ASSERT( id.index1 == world->worldId + 1 );
	B3_ASSERT( id.generation == world->generation );
	return world;
}

b3World* b3GetWorld( int index )
{
	B3_ASSERT( 0 <= index && index < B3_MAX_WORLDS );
	b3World* world = b3_worlds + index;
	B3_ASSERT( world->worldId == index );
	return world;
}

b3World* b3GetUnlockedWorld( int index )
{
	B3_ASSERT( 0 <= index && index < B3_MAX_WORLDS );
	b3World* world = b3_worlds + index;
	B3_ASSERT( world->worldId == index );
	if ( world->locked )
	{
		B3_ASSERT( false );
		return nullptr;
	}

	return world;
}

static void* b3DefaultAddTaskFcn( b3TaskCallback* task, int count, int minRange, void* taskContext, void* userContext )
{
	B3_UNUSED( minRange );
	B3_UNUSED( userContext );
	task( 0, count, 0, taskContext );
	return nullptr;
}

static void b3DefaultFinishTaskFcn( void* userTask, void* userContext )
{
	B3_UNUSED( userTask );
	B3_UNUSED( userContext );
}

static float b3DefaultFrictionCallback( float frictionA, uint64_t materialA, float frictionB, uint64_t materialB )
{
	B3_UNUSED( materialA, materialB );
	return sqrtf( frictionA * frictionB );
}

static float b3DefaultRestitutionCallback( float restitutionA, uint64_t materialA, float restitutionB, uint64_t materialB )
{
	B3_UNUSED( materialA, materialB );
	return b3MaxFloat( restitutionA, restitutionB );
}

static void b3CreateWorkerContexts( b3World* world )
{
	world->taskContexts.Create( world->workerCount );
	world->taskContexts.Resize( world->workerCount );
	world->taskContexts.MemZero();

	world->sensorTaskContexts.Create( world->workerCount );
	world->sensorTaskContexts.Resize( world->workerCount );
	world->sensorTaskContexts.MemZero();

	for ( int i = 0; i < world->workerCount; ++i )
	{
		world->taskContexts.data[i].arena = b3CreateArenaAllocator( 2048 );
		world->taskContexts.data[i].sensorHits.Create( 8 );
		world->taskContexts.data[i].contactStateBitSet = b3CreateBitSet( 1024 );
		world->taskContexts.data[i].jointStateBitSet = b3CreateBitSet( 1024 );
		world->taskContexts.data[i].enlargedSimBitSet = b3CreateBitSet( 256 );
		world->taskContexts.data[i].awakeIslandBitSet = b3CreateBitSet( 256 );
		world->taskContexts.data[i].splitIslandId = B3_NULL_INDEX;

		world->sensorTaskContexts.data[i].eventBits = b3CreateBitSet( 128 );
	}
}

static void b3DestroyWorkerContexts( b3World* world )
{
	for ( int i = 0; i < world->workerCount; ++i )
	{
		b3DestroyArenaAllocator( &world->taskContexts.data[i].arena );
		world->taskContexts.data[i].sensorHits.Destroy();
		b3DestroyBitSet( &world->taskContexts.data[i].contactStateBitSet );
		b3DestroyBitSet( &world->taskContexts.data[i].jointStateBitSet );
		b3DestroyBitSet( &world->taskContexts.data[i].enlargedSimBitSet );
		b3DestroyBitSet( &world->taskContexts.data[i].awakeIslandBitSet );

		b3DestroyBitSet( &world->sensorTaskContexts.data[i].eventBits );
	}

	world->taskContexts.Destroy();
	world->sensorTaskContexts.Destroy();
}

b3WorldId b3CreateWorld( const b3WorldDef* def )
{
	B3_CHECK_DEF( def );

	int worldId = B3_NULL_INDEX;
	for ( int i = 0; i < B3_MAX_WORLDS; ++i )
	{
		if ( b3_worlds[i].inUse == false )
		{
			worldId = i;
			break;
		}
	}

	if ( worldId == B3_NULL_INDEX )
	{
		b3Log( "B3_MAX_WORLDS of %d exceeded!!!", B3_MAX_WORLDS );
		B3_ASSERT( worldId != B3_NULL_INDEX );
		return b3WorldId{};
	}

	// b3Log( "b3_lengthUnitsPerMeter = %g", b3_lengthUnitsPerMeter );

	int oldCount = b3AtomicFetchAddInt( &b3_worldCount, 1 );
	b3_maxWorldCount = b3MaxInt( b3_maxWorldCount, oldCount + 1 );

	// b3Log( "Created world %d", worldId );

	b3InitializeContactRegisters();

	b3World* world = b3_worlds + worldId;
	uint16_t revision = world->generation;

	memset( world, 0, sizeof( b3World ) );

	world->worldId = (uint16_t)worldId;
	world->generation = revision;
	world->inUse = true;

	world->arena = b3CreateArenaAllocator( 2048 );
	world->manifoldPools[0] = b3CreatePool( 1 * sizeof( b3Manifold ) );
	world->manifoldMutex[0] = b3CreateMutex();
	world->manifoldPools[1] = b3CreatePool( 2 * sizeof( b3Manifold ) );
	world->manifoldMutex[1] = b3CreateMutex();
	world->manifoldPools[2] = b3CreatePool( 3 * sizeof( b3Manifold ) );
	world->manifoldMutex[2] = b3CreateMutex();

	b3CreateBroadPhase( &world->broadPhase, &def->capacity );
	b3CreateGraph( &world->constraintGraph, 16 );

	// pools
	world->bodyIdPool = b3CreateIdPool();

	int bodyCapacity = b3MaxInt( 16, def->capacity.staticBodyCount + def->capacity.dynamicBodyCount );
	world->bodies.Create( bodyCapacity );
	world->solverSets.Create( 8 );

	// add empty static, active, and disabled body sets
	world->solverSetIdPool = b3CreateIdPool();
	b3SolverSet set = {};

	// static set
	set.setIndex = b3AllocId( &world->solverSetIdPool );
	world->solverSets.PushBack( set );
	world->solverSets.data[b3_staticSet].bodySims.Create( b3MaxInt( 16, def->capacity.staticBodyCount ) );
	B3_ASSERT( world->solverSets.data[b3_staticSet].setIndex == b3_staticSet );

	// disabled set
	set.setIndex = b3AllocId( &world->solverSetIdPool );
	world->solverSets.PushBack( set );
	B3_ASSERT( world->solverSets.data[b3_disabledSet].setIndex == b3_disabledSet );

	// awake set
	set.setIndex = b3AllocId( &world->solverSetIdPool );
	world->solverSets.PushBack( set );
	world->solverSets.data[b3_awakeSet].bodySims.Create( b3MaxInt( 16, def->capacity.dynamicBodyCount ) );
	world->solverSets.data[b3_awakeSet].bodyStates.Create( b3MaxInt( 16, def->capacity.dynamicBodyCount ) );
	world->solverSets.data[b3_awakeSet].contactSims.Create( b3MaxInt( 16, def->capacity.contactCount ) );
	B3_ASSERT( world->solverSets.data[b3_awakeSet].setIndex == b3_awakeSet );

	world->shapeIdPool = b3CreateIdPool();

	int shapeCapacity = b3MaxInt( 16, def->capacity.staticShapeCount + def->capacity.dynamicShapeCount );
	world->shapes.Create( shapeCapacity );

	world->contactIdPool = b3CreateIdPool();
	world->contacts.Create( b3MaxInt( 16, def->capacity.contactCount ) );

	world->jointIdPool = b3CreateIdPool();
	world->joints.Create( 16 );

	world->islandIdPool = b3CreateIdPool();
	world->islands.Create( b3MaxInt( 16, def->capacity.dynamicBodyCount ) );

	world->sensors.Create( 4 );

	world->bodyMoveEvents.Create( 4 );
	world->sensorBeginEvents.Create( 4 );
	world->sensorEndEvents[0].Create( 4 );
	world->sensorEndEvents[1].Create( 4 );
	world->contactBeginEvents.Create( 4 );
	world->contactEndEvents[0].Create( 4 );
	world->contactEndEvents[1].Create( 4 );
	world->contactHitEvents.Create( 4 );
	world->jointEvents.Create( 4 );
	world->endEventArrayIndex = 0;

	world->stepIndex = 0;
	world->splitIslandId = B3_NULL_INDEX;
	world->activeTaskCount = 0;
	world->taskCount = 0;
	world->gravity = def->gravity;
	world->hitEventThreshold = def->hitEventThreshold;
	world->restitutionThreshold = def->restitutionThreshold;
	world->maxLinearSpeed = def->maximumLinearSpeed;
	world->contactSpeed = def->contactSpeed;
	world->contactHertz = def->contactHertz;
	world->contactDampingRatio = def->contactDampingRatio;

	if ( def->frictionCallback == nullptr )
	{
		world->frictionCallback = b3DefaultFrictionCallback;
	}
	else
	{
		world->frictionCallback = def->frictionCallback;
	}

	if ( def->restitutionCallback == nullptr )
	{
		world->restitutionCallback = b3DefaultRestitutionCallback;
	}
	else
	{
		world->restitutionCallback = def->restitutionCallback;
	}

	world->enableSleep = def->enableSleep;
	world->locked = false;
	world->enableWarmStarting = true;
	world->enableContinuous = def->enableContinuous;
	world->enableSpeculative = true;
	world->userTreeTask = nullptr;
	world->userData = def->userData;

	if ( def->workerCount > 0 && def->enqueueTask != nullptr && def->finishTask != nullptr )
	{
		world->workerCount = b3ClampInt( def->workerCount, 1, B3_MAX_WORKERS );
		world->enqueueTaskFcn = def->enqueueTask;
		world->finishTaskFcn = def->finishTask;
		world->userTaskContext = def->userTaskContext;
	}
	else
	{
		world->workerCount = 1;
		world->enqueueTaskFcn = b3DefaultAddTaskFcn;
		world->finishTaskFcn = b3DefaultFinishTaskFcn;
		world->userTaskContext = nullptr;
	}

	b3CreateWorkerContexts( world );

	world->debugBodySet = b3CreateBitSet( 256 );
	world->debugJointSet = b3CreateBitSet( 256 );
	world->debugContactSet = b3CreateBitSet( 256 );
	world->debugIslandSet = b3CreateBitSet( 256 );
	world->createDebugShape = def->createDebugShape;
	world->destroyDebugShape = def->destroyDebugShape;
	world->userDebugShapeContext = def->userDebugShapeContext;

	// add one to worldId so that 0 represents a null b3WorldId
	return b3WorldId{ (uint16_t)( worldId + 1 ), world->generation };
}

void b3DestroyWorld( b3WorldId worldId )
{
	b3AtomicFetchAddInt( &b3_worldCount, -1 );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	world->locked = true;

	b3DestroyBitSet( &world->debugBodySet );
	b3DestroyBitSet( &world->debugJointSet );
	b3DestroyBitSet( &world->debugContactSet );
	b3DestroyBitSet( &world->debugIslandSet );

	b3DestroyWorkerContexts( world );

	world->bodyMoveEvents.Destroy();
	world->sensorBeginEvents.Destroy();
	world->sensorEndEvents[0].Destroy();
	world->sensorEndEvents[1].Destroy();
	world->contactBeginEvents.Destroy();
	world->contactEndEvents[0].Destroy();
	world->contactEndEvents[1].Destroy();
	world->contactHitEvents.Destroy();
	world->jointEvents.Destroy();

	int sensorCount = world->sensors.count;
	for ( int i = 0; i < sensorCount; ++i )
	{
		world->sensors.data[i].hits.Destroy();
		world->sensors.data[i].overlaps1.Destroy();
		world->sensors.data[i].overlaps2.Destroy();
	}

	world->sensors.Destroy();
	world->bodies.Destroy();

	int shapeCapacity = world->shapes.count;
	b3Shape* shapes = world->shapes.data;
	for ( int i = 0; i < shapeCapacity; ++i )
	{
		b3Shape* shape = shapes + i;
		if ( shape->id != B3_NULL_INDEX )
		{
			b3DestroyShapeAllocations( world, shapes + i );
		}
	}

	world->shapes.Destroy();
	world->contacts.Destroy();
	world->joints.Destroy();
	world->islands.Destroy();

	// Destroy solver sets
	int setCapacity = world->solverSets.count;
	for ( int i = 0; i < setCapacity; ++i )
	{
		b3SolverSet* set = world->solverSets.data + i;
		if ( set->setIndex != B3_NULL_INDEX )
		{
			b3DestroySolverSet( world, i );
		}
	}

	world->solverSets.Destroy();

	b3DestroyGraph( &world->constraintGraph );
	b3DestroyBroadPhase( &world->broadPhase );

	b3DestroyIdPool( &world->bodyIdPool );
	b3DestroyIdPool( &world->shapeIdPool );
	b3DestroyIdPool( &world->contactIdPool );
	b3DestroyIdPool( &world->jointIdPool );
	b3DestroyIdPool( &world->islandIdPool );
	b3DestroyIdPool( &world->solverSetIdPool );

	b3DestroyPool( &world->manifoldPools[0] );
	b3DestroyMutex( world->manifoldMutex[0] );
	b3DestroyPool( &world->manifoldPools[1] );
	b3DestroyMutex( world->manifoldMutex[1] );
	b3DestroyPool( &world->manifoldPools[2] );
	b3DestroyMutex( world->manifoldMutex[2] );

	b3DestroyArenaAllocator( &world->arena );

	// Wipe world but preserve generation
	uint16_t generation = world->generation;
	memset( world, 0, sizeof( b3World ) );
	world->generation = generation + 1;

	// b3Log( "Destroyed world %d", worldId.index1 - 1 );
}

int b3GetWorldCount()
{
	return b3AtomicLoadInt( &b3_worldCount );
}

int b3GetMaxWorldCount()
{
	return b3_maxWorldCount;
}

static void b3CollideTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b3TracyCZoneNC( collide_task, "Collide Task", b3_colorDodgerBlue, true );

	b3StepContext* stepContext = (b3StepContext*)context;
	b3World* world = stepContext->world;
	B3_ASSERT( (int)threadIndex < world->workerCount );
	b3TaskContext* taskContext = world->taskContexts.data + threadIndex;
	b3ContactSim** contactSims = stepContext->contactSims;
	b3Shape* shapes = world->shapes.data;
	b3Body* bodies = world->bodies.data;

	B3_ASSERT( startIndex < endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3ContactSim* contactSim = contactSims[i];

		int contactId = contactSim->contactId;

		b3Shape* shapeA = shapes + contactSim->shapeIdA;
		b3Shape* shapeB = shapes + contactSim->shapeIdB;

		// Do proxies still overlap?
		bool overlap = b3AABB_Overlaps( shapeA->fatAABB, shapeB->fatAABB );
		if ( overlap == false )
		{
			contactSim->simFlags |= b3_simDisjoint;
			contactSim->simFlags &= ~b3_simTouchingFlag;
			b3SetBit( &taskContext->contactStateBitSet, contactId );
		}
		else
		{
			bool wasTouching = ( contactSim->simFlags & b3_simTouchingFlag );

			// Update contact respecting shape/body order (A,B)
			b3Body* bodyA = bodies + shapeA->bodyId;
			b3Body* bodyB = bodies + shapeB->bodyId;
			b3BodySim* bodySimA = b3GetBodySim( world, bodyA );
			b3BodySim* bodySimB = b3GetBodySim( world, bodyB );

			// avoid cache misses in b3PrepareContactsTask
			contactSim->bodySimIndexA = bodyA->setIndex == b3_awakeSet ? bodyA->localIndex : B3_NULL_INDEX;
			contactSim->invMassA = bodySimA->invMass;
			contactSim->invIA = bodySimA->invInertiaWorld;

			contactSim->bodySimIndexB = bodyB->setIndex == b3_awakeSet ? bodyB->localIndex : B3_NULL_INDEX;
			contactSim->invMassB = bodySimB->invMass;
			contactSim->invIB = bodySimB->invInertiaWorld;

			b3Transform transformA = bodySimA->transform;
			b3Transform transformB = bodySimB->transform;

			b3Vec3 centerOffsetA = b3RotateVector( transformA.q, bodySimA->localCenter );
			b3Vec3 centerOffsetB = b3RotateVector( transformB.q, bodySimB->localCenter );

			// This updates solid contacts
			bool touching = b3UpdateContact( world, threadIndex, contactSim, shapeA, centerOffsetA, transformA, shapeB,
											 centerOffsetB, transformB );

			// State changes that affect island connectivity. Also affects contact events.
			if ( touching == true && wasTouching == false )
			{
				contactSim->simFlags |= b3_simStartedTouching;
				b3SetBit( &taskContext->contactStateBitSet, contactId );
			}
			else if ( touching == false && wasTouching == true )
			{
				contactSim->simFlags |= b3_simStoppedTouching;
				b3SetBit( &taskContext->contactStateBitSet, contactId );
			}
		}
	}

	b3TracyCZoneEnd( collide_task );
}

static void b3UpdateTreesTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	B3_UNUSED( startIndex );
	B3_UNUSED( endIndex );
	B3_UNUSED( threadIndex );

	b3TracyCZoneNC( tree_task, "Rebuild Trees", b3_colorFireBrick, true );

	b3World* world = (b3World*)context;
	b3BroadPhase_RebuildTrees( &world->broadPhase );

	b3TracyCZoneEnd( tree_task );
}

static void b3AddNonTouchingContact( b3World* world, b3Contact* contact, b3ContactSim* contactSim )
{
	B3_ASSERT( contact->setIndex == b3_awakeSet );
	b3SolverSet* set = world->solverSets.Get( b3_awakeSet );
	contact->colorIndex = B3_NULL_INDEX;
	contact->localIndex = set->contactSims.count;

	b3ContactSim* newContactSim = set->contactSims.Add();
	memcpy( newContactSim, contactSim, sizeof( b3ContactSim ) );
}

static void b3RemoveNonTouchingContact( b3World* world, int setIndex, int localIndex )
{
	b3SolverSet* set = world->solverSets.Get( setIndex );
	int movedIndex = set->contactSims.RemoveSwap( localIndex );
	if ( movedIndex != B3_NULL_INDEX )
	{
		b3ContactSim* movedContactSim = set->contactSims.data + localIndex;
		b3Contact* movedContact = world->contacts.Get( movedContactSim->contactId );
		B3_ASSERT( movedContact->setIndex == setIndex );
		B3_ASSERT( movedContact->localIndex == movedIndex );
		B3_ASSERT( movedContact->colorIndex == B3_NULL_INDEX );
		movedContact->localIndex = localIndex;
	}
}

// Narrow-phase collision
static void b3Collide( b3StepContext* context )
{
	b3World* world = context->world;

	B3_ASSERT( world->workerCount > 0 );

	b3TracyCZoneNC( collide, "Collide", b3_colorDarkOrchid, true );

	// Task that can be done in parallel with the narrow-phase
	// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
	// todo_erin move this to start when contacts are being created
	world->userTreeTask = world->enqueueTaskFcn( &b3UpdateTreesTask, 1, 1, world, world->userTaskContext );
	world->taskCount += 1;
	world->activeTaskCount += world->userTreeTask == nullptr ? 0 : 1;

	// Gather contacts from all the graph colors into a single array for easier parallel-for
	int contactCount = 0;
	b3GraphColor* graphColors = world->constraintGraph.colors;
	for ( int i = 0; i < B3_GRAPH_COLOR_COUNT; ++i )
	{
		contactCount += graphColors[i].contactSims.count;
	}

	int nonTouchingCount = world->solverSets.data[b3_awakeSet].contactSims.count;
	contactCount += nonTouchingCount;

	if ( contactCount == 0 )
	{
		b3TracyCZoneEnd( collide );
		return;
	}

	b3ContactSim** contactSims =
		(b3ContactSim**)b3AllocateArenaItem( &world->arena, contactCount * sizeof( b3ContactSim* ), "contacts" );

	int contactIndex = 0;
	for ( int i = 0; i < B3_GRAPH_COLOR_COUNT; ++i )
	{
		b3GraphColor* color = graphColors + i;
		int count = color->contactSims.count;
		b3ContactSim* base = color->contactSims.data;
		for ( int j = 0; j < count; ++j )
		{
			contactSims[contactIndex] = base + j;
			contactIndex += 1;
		}
	}

	{
		b3ContactSim* base = world->solverSets.data[b3_awakeSet].contactSims.data;
		for ( int i = 0; i < nonTouchingCount; ++i )
		{
			contactSims[contactIndex] = base + i;
			contactIndex += 1;
		}
	}

	B3_ASSERT( contactIndex == contactCount );

	context->contactSims = contactSims;

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	int contactIdCapacity = b3GetIdCapacity( &world->contactIdPool );
	for ( int i = 0; i < world->workerCount; ++i )
	{
		b3SetBitCountAndClear( &world->taskContexts.data[i].contactStateBitSet, contactIdCapacity );
		world->taskContexts.data[i].satCallCount = 0;
		world->taskContexts.data[i].satCacheHitCount = 0;
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	int minRange = 20;
	void* userCollideTask = world->enqueueTaskFcn( &b3CollideTask, contactCount, minRange, context, world->userTaskContext );
	world->taskCount += 1;
	if ( userCollideTask != nullptr )
	{
		world->finishTaskFcn( userCollideTask, world->userTaskContext );
	}

	b3FreeArenaItem( &world->arena, contactSims );
	context->contactSims = nullptr;
	contactSims = nullptr;

	// Serially update contact state
	// todo_erin bring this zone together with island merge
	b3TracyCZoneNC( contact_state, "Contact State", b3_colorLightSlateGray, true );

	int satMultiplier = context->dt > 0.0f ? 1 : 0;

	// Bitwise OR all contact bits
	b3BitSet* bitSet = &world->taskContexts.data[0].contactStateBitSet;
	world->satCallCount += satMultiplier * world->taskContexts.data[0].satCallCount;
	world->satCacheHitCount += satMultiplier * world->taskContexts.data[0].satCacheHitCount;
	for ( int i = 1; i < world->workerCount; ++i )
	{
		b3InPlaceUnion( bitSet, &world->taskContexts.data[i].contactStateBitSet );
		world->satCallCount += satMultiplier * world->taskContexts.data[i].satCallCount;
		world->satCacheHitCount += satMultiplier * world->taskContexts.data[i].satCacheHitCount;
	}

	b3SolverSet* awakeSet = world->solverSets.Get( b3_awakeSet );

	int endEventArrayIndex = world->endEventArrayIndex;

	const b3Shape* shapes = world->shapes.data;
	uint16_t worldId = world->worldId;

	// Process contact state changes. Iterate over set bits
	for ( uint32_t k = 0; k < bitSet->blockCount; ++k )
	{
		uint64_t bits = bitSet->bits[k];
		while ( bits != 0 )
		{
			uint32_t ctz = b3CTZ64( bits );
			int contactId = (int)( 64 * k + ctz );

			b3Contact* contact = world->contacts.Get( contactId );
			B3_ASSERT( contact->setIndex == b3_awakeSet );

			int colorIndex = contact->colorIndex;
			int localIndex = contact->localIndex;

			b3ContactSim* contactSim = nullptr;
			if ( colorIndex != B3_NULL_INDEX )
			{
				// contact lives in constraint graph
				B3_ASSERT( 0 <= colorIndex && colorIndex < B3_GRAPH_COLOR_COUNT );
				b3GraphColor* color = graphColors + colorIndex;
				contactSim = color->contactSims.Get( localIndex );
			}
			else
			{
				contactSim = awakeSet->contactSims.Get( localIndex );
			}

			const b3Shape* shapeA = shapes + contact->shapeIdA;
			const b3Shape* shapeB = shapes + contact->shapeIdB;
			b3ShapeId shapeIdA = { shapeA->id + 1, worldId, shapeA->generation };
			b3ShapeId shapeIdB = { shapeB->id + 1, worldId, shapeB->generation };
			b3ContactId contactFullId = {
				.index1 = contactId + 1,
				.world0 = worldId,
				.padding = 0,
				.generation = contact->generation,
			};
			uint32_t flags = contact->flags;
			uint32_t simFlags = contactSim->simFlags;

			if ( simFlags & b3_simDisjoint )
			{
				// Bounding boxes no longer overlap
				b3DestroyContact( world, contact, false );
				contact = nullptr;
				contactSim = nullptr;
			}
			else if ( simFlags & b3_simStartedTouching )
			{
				B3_ASSERT( contact->islandId == B3_NULL_INDEX );

				if ( flags & b3_contactEnableContactEvents )
				{
					b3ContactBeginTouchEvent event = { shapeIdA, shapeIdB, contactFullId };
					world->contactBeginEvents.PushBack( event );
				}

				B3_ASSERT( contactSim->manifoldCount > 0 );
				B3_ASSERT( contact->setIndex == b3_awakeSet );

				// Link first because this wakes colliding bodies and ensures the body sims
				// are in the correct place.
				contact->flags |= b3_contactTouchingFlag;
				b3LinkContact( world, contact );

				// Make sure these didn't change
				B3_ASSERT( contact->colorIndex == B3_NULL_INDEX );
				B3_ASSERT( contact->localIndex == localIndex );

				// Contact sim pointer may have become orphaned due to awake set growth,
				// so I just need to refresh it.
				contactSim = awakeSet->contactSims.Get( localIndex );

				contactSim->simFlags &= ~b3_simStartedTouching;

				b3AddContactToGraph( world, contactSim, contact );
				b3RemoveNonTouchingContact( world, b3_awakeSet, localIndex );
				contactSim = nullptr;
			}
			else if ( simFlags & b3_simStoppedTouching )
			{
				contactSim->simFlags &= ~b3_simStoppedTouching;
				contact->flags &= ~b3_contactTouchingFlag;

				if ( contact->flags & b3_contactEnableContactEvents )
				{
					b3ContactEndTouchEvent event = { shapeIdA, shapeIdB, contactFullId };
					world->contactEndEvents[endEventArrayIndex].PushBack( event );
				}

				B3_ASSERT( contactSim->manifoldCount == 0 );

				b3UnlinkContact( world, contact );
				int bodyIdA = contact->edges[0].bodyId;
				int bodyIdB = contact->edges[1].bodyId;

				b3AddNonTouchingContact( world, contact, contactSim );

				// Don't destroy triangle cache
				bool destroy = false;
				b3RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex, destroy );
				contact = nullptr;
				contactSim = nullptr;
			}

			// Clear the smallest set bit
			bits = bits & ( bits - 1 );
		}
	}

	b3ValidateSolverSets( world );
	b3ValidateContacts( world );

	b3TracyCZoneEnd( contact_state );
	b3TracyCZoneEnd( collide );
}

void b3World_Step( b3WorldId worldId, float timeStep, int subStepCount )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	world->locked = true;

	for ( int i = 0; i < world->workerCount; ++i )
	{
		world->taskContexts[i].pointCount = 0;
		world->taskContexts[i].lineCount = 0;
	}

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	world->bodyMoveEvents.Clear();
	world->sensorBeginEvents.Clear();
	world->contactBeginEvents.Clear();
	world->contactHitEvents.Clear();
	world->jointEvents.Clear();

	world->profile = {};

	if ( timeStep == 0.0f )
	{
		// Swap end event array buffers
		world->endEventArrayIndex = 1 - world->endEventArrayIndex;
		world->sensorEndEvents[world->endEventArrayIndex].Clear();
		world->contactEndEvents[world->endEventArrayIndex].Clear();
	}

	b3TracyCZoneNC( world_step, "Step", b3_colorBox2DGreen, true );

	world->activeTaskCount = 0;
	world->taskCount = 0;

	uint64_t stepTicks = b3GetTicks();

	{
		b3Capacity* c = &world->maxCapacity;
		c->staticShapeCount = b3MaxInt( c->staticShapeCount, world->broadPhase.trees[b3_staticBody].proxyCount );
		c->dynamicShapeCount = b3MaxInt( c->dynamicShapeCount, world->broadPhase.trees[b3_dynamicBody].proxyCount );

		int staticBodyCount = world->solverSets.data[b3_staticSet].bodySims.count;
		c->staticBodyCount = b3MaxInt( c->staticBodyCount, staticBodyCount );

		// this includes kinematic bodies
		int totalBodyCount = b3GetIdCount( &world->bodyIdPool );
		c->dynamicBodyCount = b3MaxInt( c->dynamicBodyCount, totalBodyCount - staticBodyCount );

		int totalContactCount = b3GetIdCount( &world->contactIdPool );
		c->contactCount = b3MaxInt( c->contactCount, totalContactCount );
	}

	// Update collision pairs and create contacts
	{
		uint64_t pairTicks = b3GetTicks();
		b3UpdateBroadPhasePairs( world );
		world->profile.pairs = b3GetMilliseconds( pairTicks );
	}

	b3StepContext context = {};
	context.world = world;
	context.dt = timeStep;
	context.subStepCount = b3MaxInt( 1, subStepCount );

	if ( timeStep > 0.0f )
	{
		context.inv_dt = 1.0f / timeStep;
		context.h = timeStep / context.subStepCount;
		context.inv_h = context.subStepCount * context.inv_dt;
	}
	else
	{
		context.inv_dt = 0.0f;
		context.h = 0.0f;
		context.inv_h = 0.0f;
	}

	world->inv_h = context.inv_h;
	world->inv_dt = context.inv_dt;

	// Hertz values get reduced for large time steps
	float contactHertz = b3MinFloat( world->contactHertz, 0.125f * context.inv_h );
	context.contactSoftness = b3MakeSoft( contactHertz, world->contactDampingRatio, context.h );
	context.staticSoftness = b3MakeSoft( 2.0f * contactHertz, world->contactDampingRatio, context.h );

	context.restitutionThreshold = world->restitutionThreshold;
	context.maxLinearVelocity = world->maxLinearSpeed;
	context.enableWarmStarting = world->enableWarmStarting;

	// Update contacts
	{
		uint64_t collideTicks = b3GetTicks();
		b3Collide( &context );
		world->profile.collide = b3GetMilliseconds( collideTicks );
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	b3SolverSet* awakeSet = world->solverSets.Get( b3_awakeSet );
	int awakeBodyCount = awakeSet->bodySims.count;
	if ( context.dt > 0.0f && awakeBodyCount > 0 )
	{
		uint64_t solveTicks = b3GetTicks();
		b3Solve( world, &context );
		world->profile.solve = b3GetMilliseconds( solveTicks );
	}
	else
	{
		// Nothing to simulate, however I must still finish the broad-phase rebuild.
		if ( world->userTreeTask != nullptr )
		{
			world->finishTaskFcn( world->userTreeTask, world->userTaskContext );
			world->userTreeTask = nullptr;
			world->activeTaskCount -= 1;
		}

		b3ValidateNoEnlarged( &world->broadPhase );
	}

	// Update sensors
	{
		uint64_t sensorTicks = b3GetTicks();
		b3OverlapSensors( world );
		world->profile.sensors = b3GetMilliseconds( sensorTicks );
	}

	world->profile.step = b3GetMilliseconds( stepTicks );

	B3_ASSERT( b3GetArenaAllocation( &world->arena ) == 0 );

	// Ensure stack is large enough
	b3GrowArena( &world->arena );

	for ( int i = 0; i < world->workerCount; ++i )
	{
		b3ArenaAllocator* arena = &world->taskContexts[i].arena;
		B3_ASSERT( b3GetArenaAllocation( arena ) == 0 );
		b3GrowArena( arena );
	}

	// Make sure all tasks that were started were also finished
	B3_ASSERT( world->activeTaskCount == 0 );

	b3TracyCZoneEnd( world_step );

	// Swap end event array buffers
	world->endEventArrayIndex = 1 - world->endEventArrayIndex;
	world->sensorEndEvents[world->endEventArrayIndex].Clear();
	world->contactEndEvents[world->endEventArrayIndex].Clear();

	world->locked = false;
}

struct DrawContext
{
	b3World* world;
	b3DebugDraw* draw;
};

static bool DrawQueryCallback( int proxyId, uint64_t userData, void* context )
{
	int shapeId = int( userData );

	B3_UNUSED( proxyId );

	struct DrawContext* drawContext = (DrawContext*)context;
	b3World* world = drawContext->world;
	b3DebugDraw* draw = drawContext->draw;

	b3Shape* shape = world->shapes.Get( shapeId );
	B3_ASSERT( shape->id == shapeId );

	b3SetBit( &world->debugBodySet, shape->bodyId );

	if ( draw->drawShapes )
	{
		b3Body* body = world->bodies.Get( shape->bodyId );
		b3BodySim* bodySim = b3GetBodySim( world, body );

		b3HexColor color;

		if ( shape->materials[0].customColor != 0 )
		{
			color = b3HexColor( shape->materials[0].customColor );
		}
		else if ( body->type == b3_dynamicBody && body->mass == 0.0f )
		{
			// Bad body
			color = b3_colorRed;
		}
		else if ( body->setIndex == b3_disabledSet )
		{
			color = b3_colorSlateGray;
		}
		else if ( shape->sensorIndex != B3_NULL_INDEX )
		{
			color = b3_colorWheat;
		}
		else if ( body->flags & b3_hadTimeOfImpact )
		{
			color = b3_colorLime;
		}
		else if ( ( bodySim->flags & b3_isBullet ) && body->setIndex == b3_awakeSet )
		{
			color = b3_colorTurquoise;
		}
		else if ( body->flags & b3_isSpeedCapped )
		{
			color = b3_colorYellow;
		}
		else if ( bodySim->flags & b3_isFast )
		{
			color = b3_colorSalmon;
		}
		else if ( body->type == b3_staticBody )
		{
			color = b3_colorPaleGreen;
		}
		else if ( body->type == b3_kinematicBody )
		{
			if ( body->setIndex == b3_awakeSet )
			{
				color = b3_colorRoyalBlue;
			}
			else
			{
				color = b3_colorDarkBlue;
			}
		}
		else if ( body->setIndex == b3_awakeSet )
		{
			color = b3_colorPink;
		}
		else
		{
			color = b3_colorGray;
		}

		if ( shape->userShape == nullptr && world->createDebugShape != nullptr )
		{
			b3DebugShape debugShape = {};
			debugShape.type = shape->type;

			switch ( shape->type )
			{
				case b3_capsuleShape:
					debugShape.capsule = &shape->capsule;
					shape->userShape = world->createDebugShape( &debugShape, world->userDebugShapeContext );
					break;
				case b3_compoundShape:
					debugShape.compound = shape->compound;
					shape->userShape = world->createDebugShape( &debugShape, world->userDebugShapeContext );
					break;
				case b3_heightShape:
					debugShape.heightField = shape->heightField;
					shape->userShape = world->createDebugShape( &debugShape, world->userDebugShapeContext );
					break;
				case b3_hullShape:
					debugShape.hull = shape->hull;
					shape->userShape = world->createDebugShape( &debugShape, world->userDebugShapeContext );
					break;
				case b3_meshShape:
					debugShape.mesh = &shape->mesh;
					shape->userShape = world->createDebugShape( &debugShape, world->userDebugShapeContext );
					break;
				case b3_sphereShape:
					debugShape.sphere = &shape->sphere;
					shape->userShape = world->createDebugShape( &debugShape, world->userDebugShapeContext );
					break;
				default:
					B3_ASSERT( false );
					break;
			}
		}

		if ( shape->userShape != nullptr )
		{
			draw->DrawShapeFcn( shape->userShape, bodySim->transform, color, draw->context );
		}
	}

	if ( draw->drawBounds )
	{
		b3AABB aabb = shape->fatAABB;
		draw->DrawBoundsFcn( aabb, b3_colorGold, draw->context );
	}

	return true;
}

void b3World_Draw( b3WorldId worldId, b3DebugDraw* draw, uint64_t maskBits )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	B3_ASSERT( b3IsValidAABB( draw->drawingBounds ) );

	const float axisScale = 0.3f * b3_lengthUnitsPerMeter;
	b3HexColor speculativeColor = b3_colorGainsboro;
	b3HexColor addColor = b3_colorGreen;
	b3HexColor persistColor = b3_colorBlue;
	b3HexColor normalColor = b3_colorDimGray;
	b3HexColor impulseColor = b3_colorMagenta;
	// b3HexColor frictionColor = b3_colorYellow;

	b3HexColor graphColors[B3_GRAPH_COLOR_COUNT] = {
		b3_colorRed,	b3_colorOrange, b3_colorYellow,	   b3_colorGreen,	  b3_colorCyan,		b3_colorBlue,
		b3_colorViolet, b3_colorPink,	b3_colorChocolate, b3_colorGoldenRod, b3_colorCoral,	b3_colorRosyBrown,
		b3_colorAqua,	b3_colorPeru,	b3_colorLime,	   b3_colorGold,	  b3_colorPlum,		b3_colorSnow,
		b3_colorTeal,	b3_colorKhaki,	b3_colorSalmon,	   b3_colorPeachPuff, b3_colorHoneyDew, b3_colorBlack,
	};

	int bodyCapacity = b3GetIdCapacity( &world->bodyIdPool );
	b3SetBitCountAndClear( &world->debugBodySet, bodyCapacity );

	int jointCapacity = b3GetIdCapacity( &world->jointIdPool );
	b3SetBitCountAndClear( &world->debugJointSet, jointCapacity );

	int contactCapacity = b3GetIdCapacity( &world->contactIdPool );
	b3SetBitCountAndClear( &world->debugContactSet, contactCapacity );

	int islandCapacity = b3GetIdCapacity( &world->islandIdPool );
	b3SetBitCountAndClear( &world->debugIslandSet, islandCapacity );

	struct DrawContext drawContext = { world, draw };

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3DynamicTree_Query( world->broadPhase.trees + i, draw->drawingBounds, maskBits, false, DrawQueryCallback, &drawContext );
	}

	uint32_t wordCount = world->debugBodySet.blockCount;
	uint64_t* bits = world->debugBodySet.bits;
	for ( uint32_t wordIndex = 0; wordIndex < wordCount; ++wordIndex )
	{
		uint64_t word = bits[wordIndex];
		while ( word != 0 )
		{
			uint32_t ctz = b3CTZ64( word );
			uint32_t bodyId = 64 * wordIndex + ctz;

			b3Body* body = world->bodies.Get( bodyId );

			if ( draw->drawBodyNames && body->name[0] != 0 )
			{
				b3Vec3 offset = { 0.05f, 0.05f, 0.05f };
				b3BodySim* bodySim = b3GetBodySim( world, body );
				b3Transform transform = { bodySim->center, bodySim->transform.q };
				b3Vec3 p = b3TransformPoint( transform, offset );
				draw->DrawStringFcn( p, body->name, b3_colorOrange, draw->context );
			}

			if ( draw->drawMass && body->type == b3_dynamicBody )
			{
				b3Vec3 offset = { 0.1f, 0.1f, 0.1f };
				b3BodySim* bodySim = b3GetBodySim( world, body );

				b3Transform transform = { bodySim->center, bodySim->transform.q };
				draw->DrawTransformFcn( transform, draw->context );

				b3Vec3 p = b3TransformPoint( transform, offset );

				char buffer[32];
				snprintf( buffer, 32, "  %.2f", body->mass );
				draw->DrawStringFcn( p, buffer, b3_colorWhite, draw->context );
			}

			if ( draw->drawJoints )
			{
				int jointKey = body->headJointKey;
				while ( jointKey != B3_NULL_INDEX )
				{
					int jointId = jointKey >> 1;
					int edgeIndex = jointKey & 1;
					b3Joint* joint = world->joints.Get( jointId );

					// avoid double draw
					if ( b3GetBit( &world->debugJointSet, jointId ) == false )
					{
						b3DrawJoint( draw, world, joint );
						b3SetBit( &world->debugJointSet, jointId );
					}

					jointKey = joint->edges[edgeIndex].nextKey;
				}
			}

			const float linearSlop = B3_LINEAR_SLOP;
			if ( draw->drawContacts && body->type == b3_dynamicBody && body->setIndex == b3_awakeSet )
			{
				int contactKey = body->headContactKey;
				while ( contactKey != B3_NULL_INDEX )
				{
					int contactId = contactKey >> 1;
					int edgeIndex = contactKey & 1;
					b3Contact* contact = world->contacts.Get( contactId );
					contactKey = contact->edges[edgeIndex].nextKey;

					if ( contact->setIndex != b3_awakeSet || contact->colorIndex == B3_NULL_INDEX )
					{
						continue;
					}

					// avoid double draw
					if ( b3GetBit( &world->debugContactSet, contactId ) == false )
					{
						B3_ASSERT( 0 <= contact->colorIndex && contact->colorIndex < B3_GRAPH_COLOR_COUNT );

						b3GraphColor* gc = world->constraintGraph.colors + contact->colorIndex;
						b3ContactSim* contactSim = gc->contactSims.Get( contact->localIndex );

						for ( int manifoldIndex = 0; manifoldIndex < contactSim->manifoldCount; ++manifoldIndex )
						{
							const b3Manifold* manifold = contactSim->manifolds + manifoldIndex;
							int pointCount = manifold->pointCount;
							b3Vec3 normal = manifold->normal;
							char buffer[32];

							b3Vec3 center = b3Vec3_zero;

							for ( int j = 0; j < pointCount; ++j )
							{
								const b3ManifoldPoint* point = manifold->points + j;
								center += point->point;

								if ( draw->drawGraphColors )
								{
									// graph color
									float pointSize = contact->colorIndex == B3_OVERFLOW_INDEX ? 15.0f : 10.0f;
									draw->DrawPointFcn( point->point, pointSize, graphColors[contact->colorIndex],
														draw->context );
									// g_draw.DrawString(point->position, "%d", point->color);
								}
								else if ( point->separation > linearSlop )
								{
									// Speculative
									draw->DrawPointFcn( point->point, 10.0f, speculativeColor, draw->context );
								}
								else if ( point->persisted == false )
								{
									// Add
									draw->DrawPointFcn( point->point, 15.0f, addColor, draw->context );
								}
								else if ( point->persisted == true )
								{
									// Persist
									draw->DrawPointFcn( point->point, 10.0f, persistColor, draw->context );
								}

								if ( draw->drawContactNormals )
								{
									b3Vec3 p1 = point->point;
									b3Vec3 p2 = b3MulAdd( p1, axisScale, normal );
									draw->DrawSegmentFcn( p1, p2, normalColor, draw->context );
								}
								else if ( draw->drawContactForces )
								{
									// multiply by one-half due to relax iteration
									float force = 0.5f * point->totalNormalImpulse * world->inv_dt;
									b3Vec3 p1 = point->point;
									b3Vec3 p2 = b3MulAdd( p1, draw->forceScale * force, normal );
									draw->DrawSegmentFcn( p1, p2, impulseColor, draw->context );
									snprintf( buffer, B3_ARRAY_COUNT( buffer ), "%.1f", force );
									draw->DrawStringFcn( p1, buffer, b3_colorWhite, draw->context );
								}
								else if ( draw->drawContactFeatures )
								{
									snprintf( buffer, B3_ARRAY_COUNT( buffer ), "%d", point->id );
									draw->DrawStringFcn( point->point, buffer, b3_colorWhite, draw->context );
								}
							}

							if ( pointCount > 0 && draw->drawFrictionForces )
							{
								b3Vec3 force = b3MulSV( 0.5f * world->inv_h, manifold->frictionImpulse );
								b3Vec3 p1 = ( 1.0f / pointCount ) * center;
								b3Vec3 p2 = b3MulAdd( p1, draw->forceScale, force );
								draw->DrawSegmentFcn( p1, p2, b3_colorCrimson, draw->context );
								draw->DrawPointFcn( p1, 10.0f, b3_colorCrimson, draw->context );
								snprintf( buffer, B3_ARRAY_COUNT( buffer ), "%.2f", b3Length( force ) );
								draw->DrawStringFcn( p1, buffer, b3_colorWhite, draw->context );
							}
						}

						b3SetBit( &world->debugContactSet, contactId );
					}

					contactKey = contact->edges[edgeIndex].nextKey;
				}
			}

			if ( draw->drawIslands )
			{
				int islandId = body->islandId;
				if ( islandId != B3_NULL_INDEX && b3GetBit( &world->debugIslandSet, islandId ) == false )
				{
					b3Island* island = world->islands.data + islandId;
					if ( island->setIndex == B3_NULL_INDEX )
					{
						continue;
					}

					int shapeCount = 0;
					b3AABB aabb = {
						.lowerBound = { FLT_MAX, FLT_MAX, FLT_MAX },
						.upperBound = { -FLT_MAX, -FLT_MAX, -FLT_MAX },
					};

					int islandBodyId = island->headBody;
					while ( islandBodyId != B3_NULL_INDEX )
					{
						b3Body* islandBody = world->bodies.Get( islandBodyId );
						int shapeId = islandBody->headShapeId;
						while ( shapeId != B3_NULL_INDEX )
						{
							b3Shape* shape = world->shapes.Get( shapeId );
							aabb = b3AABB_Union( aabb, shape->fatAABB );
							shapeCount += 1;
							shapeId = shape->nextShapeId;
						}

						islandBodyId = islandBody->islandNext;
					}

					if ( shapeCount > 0 )
					{
						draw->DrawBoundsFcn( aabb, b3_colorOrangeRed, draw->context );
					}

					b3SetBit( &world->debugIslandSet, islandId );
				}
			}

			// Clear the smallest set bit
			word = word & ( word - 1 );
		}
	}

	char buffer[32] = {};
	b3Vec3 offset = { 0.002f, 0.002f, 0.002f };
	for ( int i = 0; i < world->workerCount; ++i )
	{
		int pointCount = world->taskContexts[i].pointCount;
		b3DebugPoint* points = world->taskContexts[i].points;
		for ( int j = 0; j < pointCount; ++j )
		{
			b3DebugPoint* point = points + j;
			draw->DrawPointFcn( point->p, 10.0f, point->color, draw->context );
			snprintf( buffer, 32, "%d", point->label );
			b3Vec3 ps = b3Add( point->p, offset );
			draw->DrawStringFcn( ps, buffer, point->color, draw->context );
		}

		int lineCount = world->taskContexts[i].lineCount;
		b3DebugLine* lines = world->taskContexts[i].lines;
		for ( int j = 0; j < lineCount; ++j )
		{
			b3DebugLine* line = lines + j;
			draw->DrawSegmentFcn( line->p1, line->p2, line->color, draw->context );
			draw->DrawPointFcn( line->p1, 10.0f, line->color, draw->context );
			snprintf( buffer, 32, "%d", line->label );
			b3Vec3 ps = b3Add( line->p1, offset );
			draw->DrawStringFcn( ps, buffer, line->color, draw->context );
		}
	}
}

b3BodyEvents b3World_GetBodyEvents( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return b3BodyEvents{};
	}

	int count = world->bodyMoveEvents.count;
	b3BodyEvents events = { world->bodyMoveEvents.data, count };
	return events;
}

b3SensorEvents b3World_GetSensorEvents( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return b3SensorEvents{};
	}

	// Careful to use previous buffer
	int endEventArrayIndex = 1 - world->endEventArrayIndex;

	int beginCount = world->sensorBeginEvents.count;
	int endCount = world->sensorEndEvents[endEventArrayIndex].count;

	b3SensorEvents events = {
		.beginEvents = world->sensorBeginEvents.data,
		.endEvents = world->sensorEndEvents[endEventArrayIndex].data,
		.beginCount = beginCount,
		.endCount = endCount,
	};
	return events;
}

b3ContactEvents b3World_GetContactEvents( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return b3ContactEvents{};
	}

	// Careful to use previous buffer
	int endEventArrayIndex = 1 - world->endEventArrayIndex;

	int beginCount = world->contactBeginEvents.count;
	int endCount = world->contactEndEvents[endEventArrayIndex].count;
	int hitCount = world->contactHitEvents.count;

	b3ContactEvents events = {
		.beginEvents = world->contactBeginEvents.data,
		.endEvents = world->contactEndEvents[endEventArrayIndex].data,
		.hitEvents = world->contactHitEvents.data,
		.beginCount = beginCount,
		.endCount = endCount,
		.hitCount = hitCount,
	};

	return events;
}

b3JointEvents b3World_GetJointEvents( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return {};
	}

	int count = world->jointEvents.count;
	b3JointEvents events = { world->jointEvents.data, count };
	return events;
}

bool b3World_IsValid( b3WorldId id )
{
	if ( id.index1 < 1 || B3_MAX_WORLDS < id.index1 )
	{
		return false;
	}

	b3World* world = b3_worlds + ( id.index1 - 1 );

	if ( world->worldId != id.index1 - 1 )
	{
		// world is not allocated
		return false;
	}

	return id.generation == world->generation;
}

bool b3Body_IsValid( b3BodyId id )
{
	if ( B3_MAX_WORLDS <= id.world0 )
	{
		// invalid world
		return false;
	}

	b3World* world = b3_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		// world is free
		return false;
	}

	if ( id.index1 < 1 || world->bodies.count < id.index1 )
	{
		// invalid index
		return false;
	}

	b3Body* body = world->bodies.data + ( id.index1 - 1 );
	if ( body->setIndex == B3_NULL_INDEX )
	{
		// this was freed
		return false;
	}

	B3_ASSERT( body->localIndex != B3_NULL_INDEX );

	if ( body->generation != id.generation )
	{
		// this id is orphaned
		return false;
	}

	return true;
}

bool b3Shape_IsValid( b3ShapeId id )
{
	if ( B3_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b3World* world = b3_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int shapeId = id.index1 - 1;
	if ( shapeId < 0 || world->shapes.count <= shapeId )
	{
		return false;
	}

	b3Shape* shape = world->shapes.data + shapeId;
	if ( shape->id == B3_NULL_INDEX )
	{
		// shape is free
		return false;
	}

	B3_ASSERT( shape->id == shapeId );

	return id.generation == shape->generation;
}

bool b3Joint_IsValid( b3JointId id )
{
	if ( B3_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b3World* world = b3_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int jointId = id.index1 - 1;
	if ( jointId < 0 || world->joints.count <= jointId )
	{
		return false;
	}

	b3Joint* joint = world->joints.data + jointId;
	if ( joint->jointId == B3_NULL_INDEX )
	{
		// joint is free
		return false;
	}

	B3_ASSERT( joint->jointId == jointId );

	return id.generation == joint->generation;
}

bool b3Contact_IsValid( b3ContactId id )
{
	if ( B3_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b3World* world = b3_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int contactId = id.index1 - 1;
	if ( contactId < 0 || world->contacts.count <= contactId )
	{
		return false;
	}

	b3Contact* contact = world->contacts.data + contactId;
	if ( contact->contactId == B3_NULL_INDEX )
	{
		// contact is free
		return false;
	}

	B3_ASSERT( contact->contactId == contactId );

	return id.generation == contact->generation;
}

void b3World_EnableSleeping( b3WorldId worldId, bool flag )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	if ( flag == world->enableSleep )
	{
		return;
	}

	world->enableSleep = flag;

	if ( flag == false )
	{
		int setCount = world->solverSets.count;
		for ( int i = b3_firstSleepingSet; i < setCount; ++i )
		{
			b3SolverSet* set = world->solverSets.Get( i );
			if ( set->bodySims.count > 0 )
			{
				b3WakeSolverSet( world, i );
			}
		}
	}
}

bool b3World_IsSleepingEnabled( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->enableSleep;
}

void b3World_EnableWarmStarting( b3WorldId worldId, bool flag )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	world->enableWarmStarting = flag;
}

bool b3World_IsWarmStartingEnabled( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->enableWarmStarting;
}

int b3World_GetAwakeBodyCount( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return 0;
	}
	b3SolverSet* awakeSet = world->solverSets.Get( b3_awakeSet );
	return awakeSet->bodySims.count;
}

void b3World_EnableContinuous( b3WorldId worldId, bool flag )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	world->enableContinuous = flag;
}

bool b3World_IsContinuousEnabled( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->enableContinuous;
}

void b3World_SetRestitutionThreshold( b3WorldId worldId, float value )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	world->restitutionThreshold = b3ClampFloat( value, 0.0f, FLT_MAX );
}

float b3World_GetRestitutionThreshold( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->restitutionThreshold;
}

void b3World_SetHitEventThreshold( b3WorldId worldId, float value )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	world->hitEventThreshold = b3ClampFloat( value, 0.0f, FLT_MAX );
}

float b3World_GetHitEventThreshold( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->hitEventThreshold;
}

void b3World_SetContactTuning( b3WorldId worldId, float hertz, float dampingRatio, float contactSpeed )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	world->contactHertz = b3ClampFloat( hertz, 0.0f, FLT_MAX );
	world->contactDampingRatio = b3ClampFloat( dampingRatio, 0.0f, FLT_MAX );
	world->contactSpeed = b3ClampFloat( contactSpeed, 0.0f, FLT_MAX );
}

void b3World_SetMaximumLinearSpeed( b3WorldId worldId, float maximumLinearSpeed )
{
	B3_ASSERT( b3IsValidFloat( maximumLinearSpeed ) && maximumLinearSpeed > 0.0f );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	world->maxLinearSpeed = maximumLinearSpeed;
}

float b3World_GetMaximumLinearSpeed( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->maxLinearSpeed;
}

b3Profile b3World_GetProfile( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return b3Profile{};
	}
	return world->profile;
}

b3Counters b3World_GetCounters( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return b3Counters{};
	}
	b3Counters s = {};
	s.bodyCount = b3GetIdCount( &world->bodyIdPool );
	s.shapeCount = b3GetIdCount( &world->shapeIdPool );
	s.contactCount = b3GetIdCount( &world->contactIdPool );
	s.manifolds[0] = world->manifoldPools[0].activeItems;
	s.manifolds[1] = world->manifoldPools[1].activeItems;
	s.manifolds[2] = world->manifoldPools[2].activeItems;
	s.jointCount = b3GetIdCount( &world->jointIdPool );
	s.islandCount = b3GetIdCount( &world->islandIdPool );

	b3DynamicTree* staticTree = world->broadPhase.trees + b3_staticBody;
	s.staticTreeHeight = b3DynamicTree_GetHeight( staticTree );

	b3DynamicTree* dynamicTree = world->broadPhase.trees + b3_dynamicBody;
	b3DynamicTree* kinematicTree = world->broadPhase.trees + b3_kinematicBody;
	s.treeHeight = b3MaxInt( b3DynamicTree_GetHeight( dynamicTree ), b3DynamicTree_GetHeight( kinematicTree ) );

	s.satCallCount = world->satCallCount;
	s.satCacheHitCount = world->satCacheHitCount;

	s.stackUsed = b3GetMaxArenaAllocation( &world->arena );
	s.byteCount = b3GetByteCount();
	s.taskCount = world->taskCount;

	static_assert( B3_GRAPH_COLOR_COUNT == sizeof( s.colorCounts ) / sizeof( s.colorCounts[0] ) );

	for ( int i = 0; i < B3_GRAPH_COLOR_COUNT; ++i )
	{
		s.colorCounts[i] = world->constraintGraph.colors[i].contactSims.count + world->constraintGraph.colors[i].jointSims.count;
	}
	return s;
}

b3Capacity b3World_GetMaxCapacity( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return b3Capacity{};
	}
	return world->maxCapacity;
}

void b3World_SetUserData( b3WorldId worldId, void* userData )
{
	b3World* world = b3GetWorldFromId( worldId );
	world->userData = userData;
}

void* b3World_GetUserData( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->userData;
}

void b3World_SetWorkerCount( b3WorldId worldId, int count )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	if ( count == world->workerCount )
	{
		return;
	}

	b3DestroyWorkerContexts( world );
	world->workerCount = b3ClampInt( count, 1, B3_MAX_WORKERS );
	b3CreateWorkerContexts( world );
}

static FILE* b3OpenFile( const char* fileName )
{
	FILE* file = nullptr;

#if defined( _MSC_VER )
	errno_t e = fopen_s( &file, fileName, "w" );
	if ( e != 0 )
	{
		return nullptr;
	}
#else
	file = fopen( fileName, "w" );
#endif

	return file;
}

void b3World_DumpMemoryStats( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	FILE* file = b3OpenFile( "box3d_memory.txt" );
	if ( file == nullptr )
	{
		return;
	}

	// id pools
	fprintf( file, "id pools\n" );
	fprintf( file, "body ids: %d\n", b3GetIdBytes( &world->bodyIdPool ) );
	fprintf( file, "solver set ids: %d\n", b3GetIdBytes( &world->solverSetIdPool ) );
	fprintf( file, "joint ids: %d\n", b3GetIdBytes( &world->jointIdPool ) );
	fprintf( file, "contact ids: %d\n", b3GetIdBytes( &world->contactIdPool ) );
	fprintf( file, "island ids: %d\n", b3GetIdBytes( &world->islandIdPool ) );
	fprintf( file, "shape ids: %d\n", b3GetIdBytes( &world->shapeIdPool ) );
	fprintf( file, "\n" );

	// world arrays
	fprintf( file, "world arrays\n" );
	fprintf( file, "bodies: %d\n", world->bodies.GetByteCount() );
	fprintf( file, "solver sets: %d\n", world->solverSets.GetByteCount() );
	fprintf( file, "joints: %d\n", world->joints.GetByteCount() );
	fprintf( file, "contacts: %d\n", world->contacts.GetByteCount() );
	fprintf( file, "islands: %d\n", world->islands.GetByteCount() );
	fprintf( file, "shapes: %d\n", world->shapes.GetByteCount() );
	fprintf( file, "\n" );

	// broad-phase
	fprintf( file, "broad-phase\n" );
	fprintf( file, "static tree: %d\n", b3DynamicTree_GetByteCount( world->broadPhase.trees + b3_staticBody ) );
	fprintf( file, "kinematic tree: %d\n", b3DynamicTree_GetByteCount( world->broadPhase.trees + b3_kinematicBody ) );
	fprintf( file, "dynamic tree: %d\n", b3DynamicTree_GetByteCount( world->broadPhase.trees + b3_dynamicBody ) );
	b3HashSet* moveSet = &world->broadPhase.moveSet;
	fprintf( file, "moveSet: %d (%d, %d)\n", b3GetHashSetBytes( moveSet ), moveSet->count, moveSet->capacity );
	fprintf( file, "moveArray: %d\n", world->broadPhase.moveArray.GetByteCount() );
	b3HashSet* pairSet = &world->broadPhase.pairSet;
	fprintf( file, "pairSet: %d (%d, %d)\n", b3GetHashSetBytes( pairSet ), pairSet->count, pairSet->capacity );
	fprintf( file, "\n" );

	// solver sets
	int bodySimCapacity = 0;
	int bodyStateCapacity = 0;
	int jointSimCapacity = 0;
	int contactSimCapacity = 0;
	int islandSimCapacity = 0;
	int solverSetCapacity = world->solverSets.count;
	for ( int i = 0; i < solverSetCapacity; ++i )
	{
		b3SolverSet* set = world->solverSets.data + i;
		if ( set->setIndex == B3_NULL_INDEX )
		{
			continue;
		}

		bodySimCapacity += set->bodySims.capacity;
		bodyStateCapacity += set->bodyStates.capacity;
		jointSimCapacity += set->jointSims.capacity;
		contactSimCapacity += set->contactSims.capacity;
		islandSimCapacity += set->islandSims.capacity;
	}

	fprintf( file, "solver sets\n" );
	fprintf( file, "body sim: %d\n", bodySimCapacity * (int)sizeof( b3BodySim ) );
	fprintf( file, "body state: %d\n", bodyStateCapacity * (int)sizeof( b3BodyState ) );
	fprintf( file, "joint sim: %d\n", jointSimCapacity * (int)sizeof( b3JointSim ) );
	fprintf( file, "contact sim: %d\n", contactSimCapacity * (int)sizeof( b3ContactSim ) );
	fprintf( file, "island sim: %d\n", islandSimCapacity * (int)sizeof( islandSimCapacity ) );
	fprintf( file, "\n" );

	// constraint graph
	int bodyBitSetBytes = 0;
	contactSimCapacity = 0;
	jointSimCapacity = 0;
	for ( int i = 0; i < B3_GRAPH_COLOR_COUNT; ++i )
	{
		b3GraphColor* c = world->constraintGraph.colors + i;
		bodyBitSetBytes += b3GetBitSetBytes( &c->bodySet );
		contactSimCapacity += c->contactSims.capacity;
		jointSimCapacity += c->jointSims.capacity;
	}

	fprintf( file, "constraint graph\n" );
	fprintf( file, "body bit sets: %d\n", bodyBitSetBytes );
	fprintf( file, "joint sim: %d\n", jointSimCapacity * (int)sizeof( b3JointSim ) );
	fprintf( file, "contact sim: %d\n", contactSimCapacity * (int)sizeof( b3ContactSim ) );
	fprintf( file, "\n" );

	// stack allocator
	fprintf( file, "stack allocator: %d\n\n", world->arena.capacity );

	fclose( file );
}

void b3World_DumpShapeBounds( b3WorldId worldId, b3BodyType type )
{
	B3_ASSERT( b3_staticBody <= type && type <= b3_dynamicBody );
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	FILE* file = b3OpenFile( "box3d_bounds.txt" );
	if ( file == nullptr )
	{
		return;
	}

	b3DynamicTree* tree = world->broadPhase.trees + type;
	b3TreeNode* nodes = tree->nodes;

	uint16_t requiredFlags = b3_allocatedNode | b3_leafNode;
	int capacity = tree->nodeCapacity;
	for ( int i = 0; i < capacity; ++i )
	{
		b3TreeNode* node = nodes + i;
		if ( ( node->flags & requiredFlags ) != requiredFlags )
		{
			// skip internal and free nodes
			continue;
		}

		b3Vec3 a = node->aabb.lowerBound;
		b3Vec3 b = node->aabb.upperBound;
		fprintf( file, "%.9f %.9f %.9f %.9f %.9f %.9f\n", a.x, a.y, a.z, b.x, b.y, b.z );
	}

	fclose( file );
}

struct WorldQueryContext
{
	b3World* world;
	b3OverlapResultFcn* fcn;
	b3QueryFilter filter;
	void* userContext;
};

static bool TreeQueryCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = int( userData );
	WorldQueryContext* worldContext = (WorldQueryContext*)context;
	b3World* world = worldContext->world;

	b3Shape* shape = world->shapes.Get( shapeId );

	b3Filter shapeFilter = shape->filter;
	b3QueryFilter queryFilter = worldContext->filter;

	if ( b3ShouldQueryCollide( &shapeFilter, &queryFilter ) == false )
	{
		return true;
	}

	b3ShapeId id = { shapeId + 1, world->worldId, shape->generation };
	bool result = worldContext->fcn( id, worldContext->userContext );
	return result;
}

b3TreeStats b3World_OverlapAABB( b3WorldId worldId, b3AABB aabb, b3QueryFilter filter, b3OverlapResultFcn* fcn, void* context )
{
	b3TreeStats treeStats = {};

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return treeStats;
	}

	B3_ASSERT( b3IsValidAABB( aabb ) );

	WorldQueryContext worldContext = { world, fcn, filter, context };

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3TreeStats treeResult =
			b3DynamicTree_Query( world->broadPhase.trees + i, aabb, filter.maskBits, false, TreeQueryCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

struct WorldOverlapContext
{
	b3World* world;
	b3OverlapResultFcn* fcn;
	b3QueryFilter filter;
	b3ShapeProxy proxy;
	void* userContext;
};

static bool b3TreeOverlapCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = int( userData );
	WorldOverlapContext* worldContext = (WorldOverlapContext*)context;
	b3World* world = worldContext->world;

	b3Shape* shape = world->shapes.Get( shapeId );

	b3Filter shapeFilter = shape->filter;
	b3QueryFilter queryFilter = worldContext->filter;

	if ( b3ShouldQueryCollide( &shapeFilter, &queryFilter ) == false )
	{
		return true;
	}

	b3Body* body = world->bodies.Get( shape->bodyId );
	b3Transform transform = b3GetBodyTransformQuick( world, body );

	bool overlapping = b3OverlapShape( shape, transform, &worldContext->proxy );
	if ( overlapping == false )
	{
		return true;
	}

	b3ShapeId id = { shape->id + 1, world->worldId, shape->generation };
	bool result = worldContext->fcn( id, worldContext->userContext );
	return result;
}

b3TreeStats b3World_OverlapShape( b3WorldId worldId, const b3ShapeProxy* proxy, b3QueryFilter filter, b3OverlapResultFcn* fcn,
								  void* context )
{
	b3TreeStats treeStats = {};

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return treeStats;
	}

	b3AABB aabb = b3MakeAABB( proxy->points, proxy->count, proxy->radius );
	WorldOverlapContext worldContext = {
		world, fcn, filter, *proxy, context,
	};

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3TreeStats treeResult = b3DynamicTree_Query( world->broadPhase.trees + i, aabb, filter.maskBits, false,
													  b3TreeOverlapCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

struct WorldMoverContext
{
	b3World* world;
	b3PlaneResultFcn* fcn;
	b3QueryFilter filter;
	b3Capsule mover;
	void* userContext;
};

static bool TreeCollideCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = int( userData );
	WorldMoverContext* worldContext = (WorldMoverContext*)context;
	b3World* world = worldContext->world;

	b3Shape* shape = world->shapes.Get( shapeId );

	b3Filter shapeFilter = shape->filter;
	b3QueryFilter queryFilter = worldContext->filter;

	if ( b3ShouldQueryCollide( &shapeFilter, &queryFilter ) == false )
	{
		return true;
	}

	b3Body* body = world->bodies.Get( shape->bodyId );
	b3Transform transform = b3GetBodyTransformQuick( world, body );

	b3PlaneResult buffer[64];
	int count = b3CollideMover( buffer, 64, shape, transform, &worldContext->mover );

	if ( count > 0 )
	{
		b3ShapeId id = { shape->id + 1, world->worldId, shape->generation };
		return worldContext->fcn( id, buffer, count, worldContext->userContext );
	}

	return true;
}

// It is tempting to use a shape proxy for the mover, but this makes handling deep overlap difficult and the generality may
// not be worth it.
void b3World_CollideMover( b3WorldId worldId, const b3Capsule* mover, b3QueryFilter filter, b3PlaneResultFcn* fcn, void* context )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	b3Vec3 r = { mover->radius, mover->radius, mover->radius };

	b3AABB aabb;
	aabb.lowerBound = b3Min( mover->center1, mover->center2 ) - r;
	aabb.upperBound = b3Max( mover->center1, mover->center2 ) + r;

	WorldMoverContext worldContext = {
		world, fcn, filter, *mover, context,
	};

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3DynamicTree_Query( world->broadPhase.trees + i, aabb, filter.maskBits, false, TreeCollideCallback, &worldContext );
	}
}

struct WorldRayCastContext
{
	b3World* world;
	b3CastResultFcn* fcn;
	b3QueryFilter filter;
	float fraction;
	void* userContext;
};

static float RayCastCallback( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = int( userData );
	WorldRayCastContext* worldContext = (WorldRayCastContext*)context;
	b3World* world = worldContext->world;

	b3Shape* shape = world->shapes.Get( shapeId );
	b3Filter shapeFilter = shape->filter;
	b3QueryFilter queryFilter = worldContext->filter;

	if ( b3ShouldQueryCollide( &shapeFilter, &queryFilter ) == false )
	{
		return input->maxFraction;
	}

	b3Body* body = world->bodies.Get( shape->bodyId );
	b3Transform transform = b3GetBodyTransformQuick( world, body );
	b3CastOutput output = b3RayCastShape( shape, transform, input );

	if ( output.hit )
	{
		B3_ASSERT( output.fraction <= input->maxFraction );

		b3ShapeId id = { shapeId + 1, world->worldId, shape->generation };

		int materialIndex = b3ClampInt( output.materialIndex, 0, shape->materialCount - 1 );
		uint64_t userMaterialId = shape->materials[materialIndex].userMaterialId;

		int triangleIndex = output.triangleIndex;
		int childIndex = output.childIndex;
		float fraction = worldContext->fcn( id, output.point, output.normal, output.fraction, userMaterialId, triangleIndex,
											childIndex, worldContext->userContext );

		// The user may return -1 to skip this shape
		if ( 0.0f <= fraction && fraction <= 1.0f )
		{
			worldContext->fraction = fraction;
		}

		return fraction;
	}

	return input->maxFraction;
}

b3TreeStats b3World_CastRay( b3WorldId worldId, b3Vec3 origin, b3Vec3 translation, b3QueryFilter filter, b3CastResultFcn* fcn,
							 void* context )
{
	b3TreeStats treeStats = {};

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return treeStats;
	}

	B3_ASSERT( b3IsValidVec3( origin ) );
	B3_ASSERT( b3IsValidVec3( translation ) );

	b3RayCastInput input = { origin, translation, 1.0f };

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3TreeStats treeResult =
			b3DynamicTree_RayCast( world->broadPhase.trees + i, &input, filter.maskBits, false, RayCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

// This callback finds the closest hit. This is the most common callback used in games.
static float b3RayCastClosestFcn( b3ShapeId shapeId, b3Vec3 point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
								  int triangleIndex, int childIndex, void* context )
{
	// Ignore initial overlap
	if ( fraction == 0.0f )
	{
		return -1.0f;
	}

	b3RayResult* rayResult = (b3RayResult*)context;
	rayResult->shapeId = shapeId;
	rayResult->point = point;
	rayResult->normal = normal;
	rayResult->fraction = fraction;
	rayResult->userMaterialId = userMaterialId;
	rayResult->triangleIndex = triangleIndex;
	rayResult->childIndex = childIndex;
	rayResult->hit = true;
	return fraction;
}

b3RayResult b3World_CastRayClosest( b3WorldId worldId, b3Vec3 origin, b3Vec3 translation, b3QueryFilter filter )
{
	b3RayResult result = {};

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return result;
	}

	B3_ASSERT( b3IsValidVec3( origin ) );
	B3_ASSERT( b3IsValidVec3( translation ) );

	b3RayCastInput input = { origin, translation, 1.0f };
	WorldRayCastContext worldContext = { world, b3RayCastClosestFcn, filter, 1.0f, &result };

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3TreeStats treeResult =
			b3DynamicTree_RayCast( world->broadPhase.trees + i, &input, filter.maskBits, false, RayCastCallback, &worldContext );
		result.nodeVisits += treeResult.nodeVisits;
		result.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return result;
		}

		input.maxFraction = worldContext.fraction;
	}

	return result;
}

static float b3ShapeCastCallback( const b3ShapeCastInput* input, int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = int( userData );
	WorldRayCastContext* worldContext = (WorldRayCastContext*)context;
	b3World* world = worldContext->world;

	b3Shape* shape = world->shapes.Get( shapeId );
	b3Filter shapeFilter = shape->filter;
	b3QueryFilter queryFilter = worldContext->filter;

	if ( b3ShouldQueryCollide( &shapeFilter, &queryFilter ) == false )
	{
		return input->maxFraction;
	}

	b3Body* body = world->bodies.Get( shape->bodyId );
	b3Transform transform = b3GetBodyTransformQuick( world, body );

	b3CastOutput output = b3ShapeCastShape( shape, transform, input );

	if ( output.hit )
	{
		b3ShapeId id = { shapeId + 1, world->worldId, shape->generation };
		int materialIndex = b3ClampInt( output.materialIndex, 0, shape->materialCount - 1 );
		uint64_t userMaterialId = shape->materials[materialIndex].userMaterialId;

		int triangleIndex = output.triangleIndex;
		int childIndex = output.childIndex;
		float fraction = worldContext->fcn( id, output.point, output.normal, output.fraction, userMaterialId, triangleIndex,
											childIndex, worldContext->userContext );

		// The user may return -1 to skip this shape
		if ( 0.0f <= fraction && fraction <= 1.0f )
		{
			worldContext->fraction = fraction;
		}

		return fraction;
	}

	return input->maxFraction;
}

b3TreeStats b3World_CastShape( b3WorldId worldId, const b3ShapeProxy* proxy, b3Vec3 translation, b3QueryFilter filter,
							   b3CastResultFcn* fcn, void* context )
{
	b3TreeStats treeStats = {};

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return treeStats;
	}

	B3_ASSERT( b3IsValidVec3( translation ) );

	b3ShapeCastInput input;
	input.proxy = *proxy;
	input.translation = translation;
	input.maxFraction = 1.0f;
	input.canEncroach = false;

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3TreeStats treeResult = b3DynamicTree_ShapeCast( world->broadPhase.trees + i, &input, filter.maskBits, false,
														  b3ShapeCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

struct WorldMoverCastContext
{
	b3World* world;
	b3QueryFilter filter;
	float fraction;
};

static float MoverCastCallback( const b3ShapeCastInput* input, int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = int( userData );
	WorldMoverCastContext* worldContext = (WorldMoverCastContext*)context;
	b3World* world = worldContext->world;

	b3Shape* shape = world->shapes.Get( shapeId );
	b3Filter shapeFilter = shape->filter;
	b3QueryFilter queryFilter = worldContext->filter;

	if ( b3ShouldQueryCollide( &shapeFilter, &queryFilter ) == false )
	{
		return worldContext->fraction;
	}

	b3Body* body = world->bodies.Get( shape->bodyId );
	b3Transform transform = b3GetBodyTransformQuick( world, body );

	b3CastOutput output = b3ShapeCastShape( shape, transform, input );
	if ( output.fraction == 0.0f )
	{
		// Ignore overlapping shapes
		return worldContext->fraction;
	}

	worldContext->fraction = output.fraction;
	return output.fraction;
}

float b3World_CastMover( b3WorldId worldId, const b3Capsule* mover, b3Vec3 translation, b3QueryFilter filter )
{
	B3_ASSERT( b3IsValidVec3( translation ) );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return 1.0f;
	}

	b3ShapeCastInput input;
	input.proxy = { &mover->center1, 2, mover->radius };
	input.translation = translation;
	input.maxFraction = 1.0f;
	input.canEncroach = mover->radius > 0.0f;

	WorldMoverCastContext worldContext = { world, filter, 1.0f };

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3DynamicTree_ShapeCast( world->broadPhase.trees + i, &input, filter.maskBits, false, MoverCastCallback, &worldContext );

		if ( worldContext.fraction == 0.0f )
		{
			return 0.0f;
		}

		input.maxFraction = worldContext.fraction;
	}

	return worldContext.fraction;
}

void b3World_SetCustomFilterCallback( b3WorldId worldId, b3CustomFilterFcn* fcn, void* context )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}
	world->customFilterFcn = fcn;
	world->customFilterContext = context;
}

void b3World_SetPreSolveCallback( b3WorldId worldId, b3PreSolveFcn* fcn, void* context )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}
	world->preSolveFcn = fcn;
	world->preSolveContext = context;
}

void b3World_SetGravity( b3WorldId worldId, b3Vec3 gravity )
{
	b3World* world = b3GetWorldFromId( worldId );
	world->gravity = gravity;
}

b3Vec3 b3World_GetGravity( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->gravity;
}

struct ExplosionContext
{
	b3World* world;
	b3Vec3 position;
	float radius;
	float falloff;
	float impulsePerArea;
};

static bool ExplosionCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = int( userData );
	ExplosionContext* explosionContext = (ExplosionContext*)context;
	b3World* world = explosionContext->world;

	b3Shape* shape = world->shapes.Get( shapeId );

	b3Body* body = world->bodies.Get( shape->bodyId );
	B3_ASSERT( body->type == b3_dynamicBody );

	b3Transform transform = b3GetBodyTransformQuick( world, body );

	b3DistanceInput input;
	input.proxyA = b3MakeShapeProxy( shape );
	input.proxyB = { &explosionContext->position, 1, 0.0f };
	input.transformA = transform;
	input.transformB = b3Transform_identity;
	input.useRadii = true;

	b3SimplexCache cache = {};
	b3DistanceOutput output = b3ShapeDistance( &input, &cache, nullptr, 0 );

	float radius = explosionContext->radius;
	float falloff = explosionContext->falloff;
	if ( output.distance > radius + falloff )
	{
		return true;
	}

	b3WakeBody( world, body );

	if ( body->setIndex != b3_awakeSet )
	{
		return true;
	}

	b3Vec3 closestPoint = output.pointA;
	if ( output.distance == 0.0f )
	{
		b3Vec3 localCentroid = b3GetShapeCentroid( shape );
		closestPoint = b3TransformPoint( transform, localCentroid );
	}

	b3Vec3 direction = b3Sub( closestPoint, explosionContext->position );
	if ( b3LengthSquared( direction ) > 100.0f * FLT_EPSILON * FLT_EPSILON )
	{
		direction = b3Normalize( direction );
	}
	else
	{
		direction = b3Vec3{ 1.0f, 0.0f, 0.0f };
	}

	float area = b3GetShapeProjectedArea( shape, direction );
	float scale = 1.0f;
	if ( output.distance > radius && falloff > 0.0f )
	{
		scale = b3ClampFloat( ( radius + falloff - output.distance ) / falloff, 0.0f, 1.0f );
	}

	float magnitude = explosionContext->impulsePerArea * area * scale;
	b3Vec3 impulse = b3MulSV( magnitude, direction );

	int localIndex = body->localIndex;
	b3SolverSet* set = world->solverSets.Get( b3_awakeSet );
	b3BodyState* state = set->bodyStates.Get( localIndex );
	b3BodySim* bodySim = set->bodySims.Get( localIndex );
	state->linearVelocity = b3MulAdd( state->linearVelocity, bodySim->invMass, impulse );
	state->angularVelocity += bodySim->invInertiaWorld * b3Cross( b3Sub( closestPoint, bodySim->center ), impulse );

	return true;
}

void b3World_Explode( b3WorldId worldId, const b3ExplosionDef* explosionDef )
{
	uint64_t maskBits = explosionDef->maskBits;
	b3Vec3 position = explosionDef->position;
	float radius = explosionDef->radius;
	float falloff = explosionDef->falloff;
	float impulsePerArea = explosionDef->impulsePerArea;

	B3_ASSERT( b3IsValidVec3( position ) );
	B3_ASSERT( b3IsValidFloat( radius ) && radius >= 0.0f );
	B3_ASSERT( b3IsValidFloat( falloff ) && falloff >= 0.0f );
	B3_ASSERT( b3IsValidFloat( impulsePerArea ) );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	// Locked due to waking
	world->locked = true;

	struct ExplosionContext explosionContext = { world, position, radius, falloff, impulsePerArea };

	float totalRadius = radius + falloff;
	b3Vec3 r = { totalRadius, totalRadius, totalRadius };
	b3AABB aabb = { position - r, position + r };

	b3DynamicTree_Query( world->broadPhase.trees + b3_dynamicBody, aabb, maskBits, false, ExplosionCallback, &explosionContext );

	world->locked = false;
}

void b3World_RebuildStaticTree( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}

	b3DynamicTree* staticTree = world->broadPhase.trees + b3_staticBody;
	b3DynamicTree_Rebuild( staticTree, true );
}

void b3World_EnableSpeculative( b3WorldId worldId, bool flag )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == nullptr )
	{
		return;
	}
	world->enableSpeculative = flag;
}

#if B3_ENABLE_VALIDATION
// This validates island graph connectivity for each body
void b3ValidateConnectivity( b3World* world )
{
	b3Body* bodies = world->bodies.data;
	int bodyCapacity = world->bodies.count;

	for ( int bodyIndex = 0; bodyIndex < bodyCapacity; ++bodyIndex )
	{
		b3Body* body = bodies + bodyIndex;
		if ( body->id == B3_NULL_INDEX )
		{
			b3ValidateFreeId( &world->bodyIdPool, bodyIndex );
			continue;
		}

		B3_ASSERT( bodyIndex == body->id );

		// Need to get the root island because islands are not merged until the next time step
		int bodyIslandId = body->islandId;
		int bodySetIndex = body->setIndex;

		int contactKey = body->headContactKey;
		while ( contactKey != B3_NULL_INDEX )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			b3Contact* contact = world->contacts.Get( contactId );

			bool touching = ( contact->flags & b3_contactTouchingFlag ) != 0;
			if ( touching )
			{
				if ( bodySetIndex != b3_staticSet )
				{
					int contactIslandId = contact->islandId;
					B3_ASSERT( contactIslandId == bodyIslandId );
				}
			}
			else
			{
				B3_ASSERT( contact->islandId == B3_NULL_INDEX );
			}

			contactKey = contact->edges[edgeIndex].nextKey;
		}

		int jointKey = body->headJointKey;
		while ( jointKey != B3_NULL_INDEX )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			b3Joint* joint = world->joints.Get( jointId );

			int otherEdgeIndex = edgeIndex ^ 1;

			b3Body* otherBody = world->bodies.Get( joint->edges[otherEdgeIndex].bodyId );

			if ( bodySetIndex == b3_disabledSet || otherBody->setIndex == b3_disabledSet )
			{
				B3_ASSERT( joint->islandId == B3_NULL_INDEX );
			}
			else if ( bodySetIndex == b3_staticSet )
			{
				// Intentional nesting
				if ( otherBody->setIndex == b3_staticSet )
				{
					B3_ASSERT( joint->islandId == B3_NULL_INDEX );
				}
			}
			else if ( body->type != b3_dynamicBody && otherBody->type != b3_dynamicBody )
			{
				B3_ASSERT( joint->islandId == B3_NULL_INDEX );
			}
			else
			{
				int jointIslandId = joint->islandId;
				B3_ASSERT( jointIslandId == bodyIslandId );
			}

			jointKey = joint->edges[edgeIndex].nextKey;
		}
	}
}

// Validates solver sets, but not island connectivity
void b3ValidateSolverSets( b3World* world )
{
	B3_ASSERT( b3GetIdCapacity( &world->bodyIdPool ) == world->bodies.count );
	B3_ASSERT( b3GetIdCapacity( &world->contactIdPool ) == world->contacts.count );
	B3_ASSERT( b3GetIdCapacity( &world->jointIdPool ) == world->joints.count );
	B3_ASSERT( b3GetIdCapacity( &world->islandIdPool ) == world->islands.count );
	B3_ASSERT( b3GetIdCapacity( &world->solverSetIdPool ) == world->solverSets.count );

	int activeSetCount = 0;
	int totalBodyCount = 0;
	int totalJointCount = 0;
	int totalContactCount = 0;
	int totalIslandCount = 0;

	// Validate all solver sets
	int setCount = world->solverSets.count;
	for ( int setIndex = 0; setIndex < setCount; ++setIndex )
	{
		b3SolverSet* set = world->solverSets.data + setIndex;
		if ( set->setIndex != B3_NULL_INDEX )
		{
			activeSetCount += 1;

			if ( setIndex == b3_staticSet )
			{
				B3_ASSERT( set->contactSims.count == 0 );
				B3_ASSERT( set->islandSims.count == 0 );
				B3_ASSERT( set->bodyStates.count == 0 );
			}
			else if ( setIndex == b3_disabledSet )
			{
				B3_ASSERT( set->islandSims.count == 0 );
				B3_ASSERT( set->bodyStates.count == 0 );
			}
			else if ( setIndex == b3_awakeSet )
			{
				B3_ASSERT( set->bodySims.count == set->bodyStates.count );
				B3_ASSERT( set->jointSims.count == 0 );
			}
			else
			{
				B3_ASSERT( set->bodyStates.count == 0 );
			}

			// Validate bodies
			{
				b3Body* bodies = world->bodies.data;
				B3_ASSERT( set->bodySims.count >= 0 );
				totalBodyCount += set->bodySims.count;
				for ( int i = 0; i < set->bodySims.count; ++i )
				{
					b3BodySim* bodySim = set->bodySims.data + i;

					int bodyId = bodySim->bodyId;
					B3_ASSERT( 0 <= bodyId && bodyId < world->bodies.count );
					b3Body* body = bodies + bodyId;
					B3_ASSERT( body->setIndex == setIndex );
					B3_ASSERT( body->localIndex == i );
					B3_ASSERT( body->generation == body->generation );

					if ( body->type == b3_dynamicBody )
					{
						B3_ASSERT( body->flags & b3_dynamicFlag );
					}

					if ( setIndex == b3_disabledSet )
					{
						B3_ASSERT( body->headContactKey == B3_NULL_INDEX );
					}

					// Validate body shapes
					int prevShapeId = B3_NULL_INDEX;
					int shapeId = body->headShapeId;
					while ( shapeId != B3_NULL_INDEX )
					{
						b3Shape* shape = world->shapes.Get( shapeId );
						B3_ASSERT( shape->id == shapeId );
						B3_ASSERT( shape->prevShapeId == prevShapeId );

						if ( setIndex == b3_disabledSet )
						{
							B3_ASSERT( shape->proxyKey == B3_NULL_INDEX );
						}
						else if ( setIndex == b3_staticSet )
						{
							B3_ASSERT( B3_PROXY_TYPE( shape->proxyKey ) == b3_staticBody );
						}
						else
						{
							b3BodyType proxyType = B3_PROXY_TYPE( shape->proxyKey );
							B3_ASSERT( proxyType == b3_kinematicBody || proxyType == b3_dynamicBody );
						}

						prevShapeId = shapeId;
						shapeId = shape->nextShapeId;
					}

					// Validate body contacts
					int contactKey = body->headContactKey;
					while ( contactKey != B3_NULL_INDEX )
					{
						int contactId = contactKey >> 1;
						int edgeIndex = contactKey & 1;

						b3Contact* contact = world->contacts.Get( contactId );
						B3_ASSERT( contact->setIndex != b3_staticSet );
						B3_ASSERT( contact->edges[0].bodyId == bodyId || contact->edges[1].bodyId == bodyId );
						contactKey = contact->edges[edgeIndex].nextKey;
					}

					// Validate body joints
					int jointKey = body->headJointKey;
					while ( jointKey != B3_NULL_INDEX )
					{
						int jointId = jointKey >> 1;
						int edgeIndex = jointKey & 1;

						b3Joint* joint = world->joints.Get( jointId );

						int otherEdgeIndex = edgeIndex ^ 1;

						b3Body* otherBody = world->bodies.Get( joint->edges[otherEdgeIndex].bodyId );

						if ( setIndex == b3_disabledSet || otherBody->setIndex == b3_disabledSet )
						{
							B3_ASSERT( joint->setIndex == b3_disabledSet );
						}
						else if ( setIndex == b3_staticSet && otherBody->setIndex == b3_staticSet )
						{
							B3_ASSERT( joint->setIndex == b3_staticSet );
						}
						else if ( body->type != b3_dynamicBody && otherBody->type != b3_dynamicBody )
						{
							B3_ASSERT( joint->setIndex == b3_staticSet );
						}
						else if ( setIndex == b3_awakeSet )
						{
							B3_ASSERT( joint->setIndex == b3_awakeSet );
						}
						else if ( setIndex >= b3_firstSleepingSet )
						{
							B3_ASSERT( joint->setIndex == setIndex );
						}

						b3JointSim* jointSim = b3GetJointSim( world, joint );
						B3_ASSERT( jointSim->jointId == jointId );
						B3_ASSERT( jointSim->bodyIdA == joint->edges[0].bodyId );
						B3_ASSERT( jointSim->bodyIdB == joint->edges[1].bodyId );

						jointKey = joint->edges[edgeIndex].nextKey;
					}
				}
			}

			// Validate contacts
			{
				B3_ASSERT( set->contactSims.count >= 0 );
				totalContactCount += set->contactSims.count;
				for ( int i = 0; i < set->contactSims.count; ++i )
				{
					b3ContactSim* contactSim = set->contactSims.data + i;
					b3Contact* contact = world->contacts.Get( contactSim->contactId );
					if ( setIndex == b3_awakeSet )
					{
						// contact should be non-touching if awake
						// or it could be this contact hasn't been transferred yet
						B3_ASSERT( contactSim->manifoldCount == 0 || ( contactSim->simFlags & b3_simStartedTouching ) != 0 );
					}
					B3_ASSERT( contact->setIndex == setIndex );
					B3_ASSERT( contact->colorIndex == B3_NULL_INDEX );
					B3_ASSERT( contact->localIndex == i );
				}
			}

			// Validate joints
			{
				B3_ASSERT( set->jointSims.count >= 0 );
				totalJointCount += set->jointSims.count;
				for ( int i = 0; i < set->jointSims.count; ++i )
				{
					b3JointSim* jointSim = set->jointSims.data + i;
					b3Joint* joint = world->joints.Get( jointSim->jointId );
					B3_ASSERT( joint->setIndex == setIndex );
					B3_ASSERT( joint->colorIndex == B3_NULL_INDEX );
					B3_ASSERT( joint->localIndex == i );
				}
			}

			// Validate islands
			{
				B3_ASSERT( set->islandSims.count >= 0 );
				totalIslandCount += set->islandSims.count;
				for ( int i = 0; i < set->islandSims.count; ++i )
				{
					b3IslandSim* islandSim = set->islandSims.data + i;
					b3Island* island = world->islands.Get( islandSim->islandId );
					B3_ASSERT( island->setIndex == setIndex );
					B3_ASSERT( island->localIndex == i );
				}
			}
		}
		else
		{
			B3_ASSERT( set->bodySims.count == 0 );
			B3_ASSERT( set->contactSims.count == 0 );
			B3_ASSERT( set->jointSims.count == 0 );
			B3_ASSERT( set->islandSims.count == 0 );
			B3_ASSERT( set->bodyStates.count == 0 );
		}
	}

	int setIdCount = b3GetIdCount( &world->solverSetIdPool );
	B3_ASSERT( activeSetCount == setIdCount );

	int bodyIdCount = b3GetIdCount( &world->bodyIdPool );
	B3_ASSERT( totalBodyCount == bodyIdCount );

	int islandIdCount = b3GetIdCount( &world->islandIdPool );
	B3_ASSERT( totalIslandCount == islandIdCount );

	// Validate constraint graph
	for ( int colorIndex = 0; colorIndex < B3_GRAPH_COLOR_COUNT; ++colorIndex )
	{
		b3GraphColor* color = world->constraintGraph.colors + colorIndex;
		int bitCount = 0;

		B3_ASSERT( color->contactSims.count >= 0 );
		totalContactCount += color->contactSims.count;
		for ( int i = 0; i < color->contactSims.count; ++i )
		{
			b3ContactSim* contactSim = color->contactSims.data + i;
			b3Contact* contact = world->contacts.Get( contactSim->contactId );
			// contact should be touching in the constraint graph or awaiting transfer to non-touching
			B3_ASSERT( contactSim->manifoldCount > 0 ||
					   ( contactSim->simFlags & ( b3_simStoppedTouching | b3_simDisjoint ) ) != 0 );
			B3_ASSERT( contact->setIndex == b3_awakeSet );
			B3_ASSERT( contact->colorIndex == colorIndex );
			B3_ASSERT( contact->localIndex == i );

			int bodyIdA = contact->edges[0].bodyId;
			int bodyIdB = contact->edges[1].bodyId;

			if ( colorIndex < B3_OVERFLOW_INDEX )
			{
				b3Body* bodyA = world->bodies.Get( bodyIdA );
				b3Body* bodyB = world->bodies.Get( bodyIdB );
				B3_ASSERT( b3GetBit( &color->bodySet, bodyIdA ) == ( bodyA->type == b3_dynamicBody ) );
				B3_ASSERT( b3GetBit( &color->bodySet, bodyIdB ) == ( bodyB->type == b3_dynamicBody ) );

				bitCount += bodyA->type == b3_dynamicBody ? 1 : 0;
				bitCount += bodyB->type == b3_dynamicBody ? 1 : 0;
			}
		}

		B3_ASSERT( color->jointSims.count >= 0 );
		totalJointCount += color->jointSims.count;
		for ( int i = 0; i < color->jointSims.count; ++i )
		{
			b3JointSim* jointSim = color->jointSims.data + i;
			b3Joint* joint = world->joints.Get( jointSim->jointId );
			B3_ASSERT( joint->setIndex == b3_awakeSet );
			B3_ASSERT( joint->colorIndex == colorIndex );
			B3_ASSERT( joint->localIndex == i );

			int bodyIdA = joint->edges[0].bodyId;
			int bodyIdB = joint->edges[1].bodyId;

			if ( colorIndex < B3_OVERFLOW_INDEX )
			{
				b3Body* bodyA = world->bodies.Get( bodyIdA );
				b3Body* bodyB = world->bodies.Get( bodyIdB );
				B3_ASSERT( b3GetBit( &color->bodySet, bodyIdA ) == ( bodyA->type == b3_dynamicBody ) );
				B3_ASSERT( b3GetBit( &color->bodySet, bodyIdB ) == ( bodyB->type == b3_dynamicBody ) );

				bitCount += bodyA->type == b3_dynamicBody ? 1 : 0;
				bitCount += bodyB->type == b3_dynamicBody ? 1 : 0;
			}
		}

		// Validate the bit population for this graph color
		B3_ASSERT( bitCount == b3CountSetBits( &color->bodySet ) );
	}

	int contactIdCount = b3GetIdCount( &world->contactIdPool );
	B3_ASSERT( totalContactCount == contactIdCount );
	B3_ASSERT( totalContactCount == (int)world->broadPhase.pairSet.count );

	int jointIdCount = b3GetIdCount( &world->jointIdPool );
	B3_ASSERT( totalJointCount == jointIdCount );

// Validate shapes
// This is very slow on compounds
#if 0
	int shapeCapacity = b3Array(world->shapeArray).count;
	for (int shapeIndex = 0; shapeIndex < shapeCapacity; shapeIndex += 1)
	{
		b3Shape* shape = world->shapeArray + shapeIndex;
		if (shape->id != shapeIndex)
		{
			continue;
		}

		B3_ASSERT(0 <= shape->bodyId && shape->bodyId < b3Array(world->bodyArray).count);

		b3Body* body = world->bodyArray + shape->bodyId;
		B3_ASSERT(0 <= body->setIndex && body->setIndex < b3Array(world->solverSetArray).count);

		b3SolverSet* set = world->solverSetArray + body->setIndex;
		B3_ASSERT(0 <= body->localIndex && body->localIndex < set->sims.count);

		b3BodySim* bodySim = set->sims.mData + body->localIndex;
		B3_ASSERT(bodySim->bodyId == shape->bodyId);

		bool found = false;
		int shapeCount = 0;
		int index = body->headShapeId;
		while (index != B3_NULL_INDEX)
		{
			b3CheckId(world->shapeArray, index);
			b3Shape* s = world->shapeArray + index;
			if (index == shapeIndex)
			{
				found = true;
			}

			index = s->nextShapeId;
			shapeCount += 1;
		}

		B3_ASSERT(found);
		B3_ASSERT(shapeCount == body->shapeCount);
	}
#endif
}

// Validate contact touching status.
void b3ValidateContacts( b3World* world )
{
	int contactCount = world->contacts.count;
	B3_ASSERT( contactCount == b3GetIdCapacity( &world->contactIdPool ) );
	int allocatedContactCount = 0;

	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		b3Contact* contact = world->contacts.Get( contactIndex );
		if ( contact->contactId == B3_NULL_INDEX )
		{
			continue;
		}

		B3_ASSERT( contact->contactId == contactIndex );

		allocatedContactCount += 1;

		bool touching = ( contact->flags & b3_contactTouchingFlag ) != 0;

		int setId = contact->setIndex;

		if ( setId == b3_awakeSet )
		{
			if ( touching )
			{
				B3_ASSERT( 0 <= contact->colorIndex && contact->colorIndex < B3_GRAPH_COLOR_COUNT );
			}
			else
			{
				B3_ASSERT( contact->colorIndex == B3_NULL_INDEX );
			}
		}
		else if ( setId >= b3_firstSleepingSet )
		{
			// Only touching contacts allowed in a sleeping set
			B3_ASSERT( touching == true );
		}
		else
		{
			// Sleeping and non-touching contacts belong in the disabled set
			B3_ASSERT( touching == false && setId == b3_disabledSet );
		}

		b3ContactSim* contactSim = b3GetContactSim( world, contact );
		B3_ASSERT( contactSim->contactId == contactIndex );
		B3_ASSERT( contactSim->bodyIdA == contact->edges[0].bodyId );
		B3_ASSERT( contactSim->bodyIdB == contact->edges[1].bodyId );

		bool simTouching = ( contactSim->simFlags & b3_simTouchingFlag ) != 0;
		B3_ASSERT( touching == simTouching );

		for ( int i = 0; i < contactSim->manifoldCount; ++i )
		{
			const b3Manifold* manifold = contactSim->manifolds + i;
			B3_ASSERT( 0 <= manifold->pointCount && manifold->pointCount <= 4 );
		}

		int cacheCount = contactSim->triangleCache.count;
		if ( cacheCount > 0 )
		{
			B3_ASSERT( contactSim->triangleCache.data != nullptr );
			B3_ASSERT( contactSim->triangleCache.capacity >= cacheCount );

			b3Shape* shapeA = world->shapes.Get( contactSim->shapeIdA );
			if ( shapeA->type == b3_meshShape )
			{
				int triangleCount = shapeA->mesh.data->triangleCount;
				for ( int i = 0; i < cacheCount; ++i )
				{
					int triangleIndex = contactSim->triangleCache.data[i].triangleIndex;
					B3_ASSERT( 0 <= triangleIndex && triangleIndex < triangleCount );
				}
			}
			else if ( shapeA->type == b3_heightShape )
			{
				int triangleCount = b3GetHeightFieldTriangleCount( shapeA->heightField );
				for ( int i = 0; i < cacheCount; ++i )
				{
					int triangleIndex = contactSim->triangleCache.data[i].triangleIndex;
					B3_ASSERT( 0 <= triangleIndex && triangleIndex < triangleCount );
				}
			}
			else
			{
				B3_ASSERT( shapeA->type == b3_compoundShape );
				b3ChildShape child = b3GetCompoundChild( shapeA->compound, contactSim->childIndex );
				B3_ASSERT( child.type == b3_meshShape );

				int triangleCount = child.mesh.data->triangleCount;
				for ( int i = 0; i < cacheCount; ++i )
				{
					int triangleIndex = contactSim->triangleCache.data[i].triangleIndex;
					B3_ASSERT( 0 <= triangleIndex && triangleIndex < triangleCount );
				}
			}
		}
	}

	int contactIdCount = b3GetIdCount( &world->contactIdPool );
	B3_ASSERT( allocatedContactCount == contactIdCount );
}

#else

void b3ValidateConnectivity( b3World* world )
{
	B3_UNUSED( world );
}

void b3ValidateSolverSets( b3World* world )
{
	B3_UNUSED( world );
}

void b3ValidateContacts( b3World* world )
{
	B3_UNUSED( world );
}

#endif
