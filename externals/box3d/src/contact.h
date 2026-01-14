// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

#include "box3d/collision.h"
#include "box3d/types.h"

#define B3_FORCE_GHOST_COLLISIONS 0

struct b3Shape;
struct b3World;

union b3ContactCache
{
	b3SATCache satCache;
	b3SimplexCache simplexCache;
};

struct b3TriangleCache
{
	int triangleIndex;
	b3ContactCache cache;
};

enum b3ContactFlags
{
	// Set when the solid shapes are touching.
	b3_contactTouchingFlag = 0x00000001,

	// Contact has a hit event
	b3_contactHitEventFlag = 0x00000002,

	// This contact wants contact events
	b3_contactEnableContactEvents = 0x00000004,
};

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
struct b3ContactEdge
{
	int bodyId;
	int prevKey;
	int nextKey;
};

// Cold contact data. Used as a persistent handle and for persistent island
// connectivity.
struct b3Contact
{
	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	int setIndex;

	// index into the constraint graph color array
	// B3_NULL_INDEX for non-touching or sleeping contacts
	// B3_NULL_INDEX when slot is free
	int colorIndex;

	// contact index within set or graph color
	// B3_NULL_INDEX when slot is free
	int localIndex;

	b3ContactEdge edges[2];
	int shapeIdA;
	int shapeIdB;
	int childIndex;

	// A contact only belongs to an island if touching, otherwise B3_NULL_INDEX.
	int islandPrev;
	int islandNext;
	int islandId;

	int contactId;

	// b3ContactFlags
	uint32_t flags;

	// This is monotonically advanced when a contact is allocated in this slot
	// Used to check for invalid b3ContactId
	uint32_t generation;
};

// Shifted to be distinct from b3ContactFlags
enum b3ContactSimFlags
{
	// Set when the shapes are touching
	b3_simTouchingFlag = 0x00010000,

	// This contact no longer has overlapping AABBs
	b3_simDisjoint = 0x00020000,

	// This contact started touching
	b3_simStartedTouching = 0x00040000,

	// This contact stopped touching
	b3_simStoppedTouching = 0x00080000,

	// This contact has a hit event
	b3_simEnableHitEvent = 0x00100000,

	// This contact wants pre-solve events
	b3_simEnablePreSolveEvents = 0x00200000,
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
struct b3ContactSim
{
	int contactId;

#if B3_ENABLE_VALIDATION
	int bodyIdA;
	int bodyIdB;
#endif

	int bodySimIndexA;
	int bodySimIndexB;

	int shapeIdA;
	int shapeIdB;
	int childIndex;

	float invMassA;

	// todo ensure this is updated in narrow phase
	b3Matrix3 invIA;

	float invMassB;
	b3Matrix3 invIB;

	int manifoldCount;
	b3Manifold* manifolds;

	b3ContactCache cache;

	// Mesh contact data

	// todo use block allocator for this for faster shutdown
	b3Array<b3TriangleCache> triangleCache;
	b3AABB meshQueryBounds;

	// Mixed friction and restitution
	float friction;
	float restitution;
	float rollingResistance;
	b3Vec3 tangentVelocity;

	// b3ContactSimFlags
	uint32_t simFlags;
};

void b3InitializeContactRegisters( void );

void b3CreateContact( b3World* world, b3Shape* shapeA, b3Shape* shapeB, int childIndex );
void b3DestroyContact( b3World* world, b3Contact* contact, bool wakeBodies );

b3ContactSim* b3GetContactSim( b3World* world, b3Contact* contact );

bool b3ShouldShapesCollide( b3Filter filterA, b3Filter filterB );

bool b3UpdateContact( b3World* world, int workerIndex, b3ContactSim* contactSim, b3Shape* shapeA, b3Vec3 centerOffsetA, b3Transform xfA,
					  b3Shape* shapeB, b3Vec3 centerOffsetB, b3Transform xfB );

bool b3ComputeMeshManifolds( b3World* world, int workerIndex, b3ContactSim* sim, const b3Shape* shapeA, b3Transform xfA, const int* materialMap, const b3Shape* shapeB,
							 b3Transform xfB );

