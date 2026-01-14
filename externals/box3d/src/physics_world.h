// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "arena_allocator.h"
#include "array.h"
#include "bitset.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "id_pool.h"
#include "pool_allocator.h"

#include "box3d/types.h"

#define B3_DEBUG_POINT_CAPACITY 64
#define B3_DEBUG_LINE_CAPACITY 64

struct b3Body;
struct b3Contact;
struct b3Island;
struct b3Joint;
struct b3Sensor;
struct b3SensorTaskContext;
struct b3SensorHit;
struct b3Shape;
struct b3SolverSet;

enum b3SetType
{
	b3_staticSet = 0,
	b3_disabledSet = 1,
	b3_awakeSet = 2,
	b3_firstSleepingSet = 3,
};

struct b3DebugPoint
{
	b3Vec3 p;
	int label;
	b3HexColor color;
};

struct b3DebugLine
{
	b3Vec3 p1, p2;
	int label;
	b3HexColor color;
};

// Per thread task storage
struct b3TaskContext
{
	b3ArenaAllocator arena;

	// Collect per thread sensor continuous hit events.
	b3Array<b3SensorHit> sensorHits;

	// These bits align with the b3ConstraintGraph::contactBlocks and signal a change in contact status
	b3BitSet contactStateBitSet;

	// These bits align with the joint id capacity and signal a change in contact status
	b3BitSet jointStateBitSet;

	// Used to track bodies with shapes that have enlarged AABBs. This avoids having a bit array
	// that is very large when there are many static shapes.
	b3BitSet enlargedSimBitSet;

	// Used to put islands to sleep
	b3BitSet awakeIslandBitSet;

	// Per worker split island candidate
	float splitSleepTime;
	int splitIslandId;

	// Profiling
	int satCallCount;
	int satCacheHitCount;

	b3DebugPoint points[B3_DEBUG_POINT_CAPACITY];
	int pointCount;

	b3DebugLine lines[B3_DEBUG_LINE_CAPACITY];
	int lineCount;

	// Prevent false sharing
	char cacheLine[64];
};

// The world struct manages all physics entities, dynamic simulation,  and asynchronous queries.
// The world also contains efficient memory management facilities.
struct b3World
{
	b3ArenaAllocator arena;

	// Allocates one manifold at a time
	b3PoolAllocator manifoldPools[B3_MAX_MANIFOLDS];
	b3Mutex* manifoldMutex[B3_MAX_MANIFOLDS];

	b3BroadPhase broadPhase;
	b3ConstraintGraph constraintGraph;

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b3Body.
	b3IdPool bodyIdPool;

	// This is a sparse array that maps body ids to the body data
	// stored in solver sets. As sims move within a set or across set.
	// Indices come from id pool.
	b3Array<b3Body> bodies;

	// Provides free list for solver sets.
	b3IdPool solverSetIdPool;

	// Solvers sets allow sims to be stored in contiguous arrays. The first
	// set is all static sims. The second set is active sims. The third set is disabled
	// sims. The remaining sets are sleeping islands.
	b3Array<b3SolverSet> solverSets;

	// Used to create stable ids for joints
	b3IdPool jointIdPool;

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the solver sets.
	b3Array<b3Joint> joints;

	// Used to create stable ids for contacts
	b3IdPool contactIdPool;

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the solver sets.
	b3Array<b3Contact> contacts;

	// Used to create stable ids for islands
	b3IdPool islandIdPool;

	// This is a sparse array that maps island ids to the island data stored in the solver sets.
	b3Array<b3Island> islands;

	b3IdPool shapeIdPool;

	// These are sparse arrays that point into the pools above
	b3Array<b3Shape> shapes;

	// This is a dense array of sensor data.
	b3Array<b3Sensor> sensors;

	// Per thread storage
	b3Array<b3TaskContext> taskContexts;
	b3Array<b3SensorTaskContext> sensorTaskContexts;

	b3Array<b3BodyMoveEvent> bodyMoveEvents;
	b3Array<b3SensorBeginTouchEvent> sensorBeginEvents;
	b3Array<b3ContactBeginTouchEvent> contactBeginEvents;

	// End events are double buffered so that the user doesn't need to flush events
	b3Array<b3SensorEndTouchEvent> sensorEndEvents[2];
	b3Array<b3ContactEndTouchEvent> contactEndEvents[2];
	int endEventArrayIndex;

	b3Array<b3ContactHitEvent> contactHitEvents;
	b3Array<b3JointEvent> jointEvents;

	// Used to track debug draw
	b3BitSet debugBodySet;
	b3BitSet debugJointSet;
	b3BitSet debugContactSet;
	b3BitSet debugIslandSet;
	b3CreateDebugShapeCallback* createDebugShape;
	b3DestroyDebugShapeCallback* destroyDebugShape;
	void* userDebugShapeContext;

	// Id that is incremented every time step
	uint64_t stepIndex;

	// Identify islands for splitting as follows:
	// - I want to split islands so smaller islands can sleep
	// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
	//   if it has removed constraints
	// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
	// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
	// - if no bodies want to sleep then there is no reason to perform island splitting
	int splitIslandId;

	b3Vec3 gravity;
	float hitEventThreshold;
	float restitutionThreshold;
	float maxLinearSpeed;
	float contactSpeed;
	float contactHertz;
	float contactDampingRatio;

	b3FrictionCallback* frictionCallback;
	b3RestitutionCallback* restitutionCallback;

	uint16_t generation;

	b3Profile profile;
	int satCallCount;
	int satCacheHitCount;

	b3Capacity maxCapacity;

	b3PreSolveFcn* preSolveFcn;
	void* preSolveContext;

	b3CustomFilterFcn* customFilterFcn;
	void* customFilterContext;

	int workerCount;
	b3EnqueueTaskCallback* enqueueTaskFcn;
	b3FinishTaskCallback* finishTaskFcn;
	void* userTaskContext;
	void* userTreeTask;

	void* userData;

	// latest inverse sub-step
	float inv_h;

	// latest inverse full-step
	float inv_dt;

	int activeTaskCount;
	int taskCount;

	uint16_t worldId;

	bool enableSleep;

	// This indicates there is a world write operation in progress. This is for debugging and
	// not a real mutex. This should have minimal performance impact.
	bool locked;

	bool enableWarmStarting;
	bool enableContinuous;
	bool enableSpeculative;
	bool inUse;
};

b3World* b3GetUnlockedWorldFromId( b3WorldId id );
b3World* b3GetWorldFromId( b3WorldId id );

b3World* b3GetUnlockedWorld( int index );
b3World* b3GetWorld( int index );

static inline b3Manifold* b3AllocateManifolds( b3World* world, int count )
{
	if (count < 0 || B3_MAX_MANIFOLDS < count)
	{
		B3_ASSERT( false );
		return nullptr;
	}

	// Need lock because this is called from the parallel narrow phase
	int index = count - 1;
	b3LockMutex( world->manifoldMutex[index] );
	b3Manifold* manifolds = (b3Manifold*)b3AllocatePoolItem( &world->manifoldPools[index] );
	b3UnlockMutex( world->manifoldMutex[index] );
	return manifolds;
}

static inline void b3FreeManifolds( b3World* world, b3Manifold* manifolds, int count )
{
	if (count < 0 || B3_MAX_MANIFOLDS < count)
	{
		B3_ASSERT( false );
		return;
	}

	int index = count - 1;
	b3LockMutex( world->manifoldMutex[index] );
	b3FreePoolItem( &world->manifoldPools[index], manifolds );
	b3UnlockMutex( world->manifoldMutex[index] );
}

void b3ValidateConnectivity( b3World* world );
void b3ValidateSolverSets( b3World* world );
void b3ValidateContacts( b3World* world );
