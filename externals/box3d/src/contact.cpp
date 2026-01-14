// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact.h"

#include "algorithm.h"
#include "array.h"
#include "body.h"
#include "compound.h"
#include "island.h"
#include "physics_world.h"
#include "shape.h"
#include "solver_set.h"
#include "table.h"

#include "box3d/box3d.h"

// #include <float.h>
// #include <math.h>

// Contacts and determinism
// A deterministic simulation requires contacts to exist in the same order in b3Island no matter the thread count.
// The order must reproduce from run to run. This is necessary because the Gauss-Seidel constraint solver is order dependent.
//
// Creation:
// - Contacts are created using results from b3UpdateBroadPhasePairs
// - These results are ordered according to the order of the broad-phase move array
// - The move array is ordered according to the shape creation order using a bitset.
// - The island/shape/body order is determined by creation order
// - Logically contacts are only created for awake bodies, so they are immediately added to the awake contact array (serially)
//
// Island linking:
// - The awake contact array is built from the body-contact graph for all awake bodies in awake islands.
// - Awake contacts are solved in parallel and they generate contact state changes.
// - These state changes may link islands together using union find.
// - The state changes are ordered using a bit array that encompasses all contacts
// - As long as contacts are created in deterministic order, island link order is deterministic.
// - This keeps the order of contacts in islands deterministic

// Manifold functions should compute important results in local space to improve precision. However, this
// interface function takes two world transforms instead of a relative transform for these reasons:
//
// First:
// The anchors need to be computed relative to the shape origin in world space. This is necessary so the
// solver does not need to access static body transforms. Not even in constraint preparation. This approach
// has world space vectors yet retains precision.
//
// Second:
// b3ManifoldPoint::point is very useful for debugging and it is in world space.
//
// Third:
// The user may call the manifold functions directly and they should be easy to use and have easy to use
// results.
// typedef b3Manifold b3ManifoldFcn( const b3Shape* shapeA, b3Transform xfA, const b3Shape* shapeB, b3Transform xfB,
//								  b3ContactCache* cache );

static b3Contact* b3GetContactFullId( b3World* world, b3ContactId contactId )
{
	int id = contactId.index1 - 1;
	b3Contact* contact = world->contacts.Get( id );
	B3_ASSERT( contact->contactId == id && contact->generation == contactId.generation );
	return contact;
}

b3ContactData b3Contact_GetData( b3ContactId contactId )
{
	b3World* world = b3GetWorld( contactId.world0 );
	b3Contact* contact = b3GetContactFullId( world, contactId );
	b3ContactSim* contactSim = b3GetContactSim( world, contact );

	const b3Shape* shapeA = world->shapes.Get( contactSim->shapeIdA );
	const b3Shape* shapeB = world->shapes.Get( contactSim->shapeIdB );

	b3ContactData data;
	data.contactId = contactId;
	data.shapeIdA = {
		.index1 = shapeA->id + 1,
		.world0 = (uint16_t)contactId.world0,
		.generation = shapeA->generation,
	};
	data.shapeIdB = {
		.index1 = shapeB->id + 1,
		.world0 = (uint16_t)contactId.world0,
		.generation = shapeB->generation,
	};

	if ( contactSim->manifoldCount > 0 )
	{
		data.manifolds = contactSim->manifolds;
		data.manifoldCount = contactSim->manifoldCount;
	}
	else
	{
		data.manifolds = nullptr;
		data.manifoldCount = 0;
	}

	return data;
}

struct b3ContactRegister
{
	// b3ManifoldFcn* fcn;
	bool supported;
	bool primary;
};

static struct b3ContactRegister s_registers[b3_shapeTypeCount][b3_shapeTypeCount];
static bool s_initialized = false;

#if 0
static b3Manifold b3SphereManifold( const b3Shape* shapeA, b3Transform xfA, const b3Shape* shapeB, b3Transform xfB,
									b3ContactCache* )
{
	return b3CollideSpheres( &shapeA->sphere, xfA, &shapeB->sphere, xfB );
}

static b3Manifold b3CapsuleAndSphereManifold( const b3Shape* shapeA, b3Transform xfA, const b3Shape* shapeB, b3Transform xfB,
											  b3ContactCache* )
{
	return b3CollideCapsuleAndSphere( &shapeA->capsule, xfA, &shapeB->sphere, xfB );
}

static b3Manifold b3CapsuleManifold( const b3Shape* shapeA, b3Transform xfA, const b3Shape* shapeB, b3Transform xfB,
									 b3ContactCache* )
{
	return b3CollideCapsules( &shapeA->capsule, xfA, &shapeB->capsule, xfB );
}

static b3Manifold b3HullAndSphereManifold( const b3Shape* shapeA, b3Transform xfA, const b3Shape* shapeB, b3Transform xfB,
										   b3ContactCache* cache )
{
	return b3CollideHullAndSphere( shapeA->hull, xfA, &shapeB->sphere, xfB, &cache->simplexCache );
}

static b3Manifold b3HullAndCapsuleManifold( const b3Shape* shapeA, b3Transform xfA, const b3Shape* shapeB, b3Transform xfB,
											b3ContactCache* cache )
{
	return b3CollideHullAndCapsule( shapeA->hull, xfA, &shapeB->capsule, xfB, &cache->simplexCache );
}

static b3Manifold b3HullManifold( const b3Shape* shapeA, b3Transform xfA, const b3Shape* shapeB, b3Transform xfB,
								  b3ContactCache* cache )
{
	return b3CollideHulls( shapeA->hull, xfA, shapeB->hull, xfB, &cache->satCache );
}
#endif

static void b3AddType( b3ShapeType type1, b3ShapeType type2 )
{
	B3_ASSERT( 0 <= type1 && type1 < b3_shapeTypeCount );
	B3_ASSERT( 0 <= type2 && type2 < b3_shapeTypeCount );

	s_registers[type1][type2].supported = true;
	s_registers[type1][type2].primary = true;

	if ( type1 != type2 )
	{
		s_registers[type2][type1].supported = true;
		s_registers[type2][type1].primary = false;
	}
}

void b3InitializeContactRegisters( void )
{
	if ( s_initialized == false )
	{
		b3AddType( b3_sphereShape, b3_sphereShape );
		b3AddType( b3_capsuleShape, b3_sphereShape );
		b3AddType( b3_capsuleShape, b3_capsuleShape );
		b3AddType( b3_compoundShape, b3_sphereShape );
		b3AddType( b3_compoundShape, b3_capsuleShape );
		b3AddType( b3_compoundShape, b3_hullShape );
		b3AddType( b3_hullShape, b3_sphereShape );
		b3AddType( b3_hullShape, b3_capsuleShape );
		b3AddType( b3_hullShape, b3_hullShape );
		b3AddType( b3_meshShape, b3_sphereShape );
		b3AddType( b3_meshShape, b3_capsuleShape );
		b3AddType( b3_meshShape, b3_hullShape );
		b3AddType( b3_heightShape, b3_sphereShape );
		b3AddType( b3_heightShape, b3_capsuleShape );
		b3AddType( b3_heightShape, b3_hullShape );
		s_initialized = true;
	}
}

void b3CreateContact( b3World* world, b3Shape* shapeA, b3Shape* shapeB, int childIndex )
{
	b3ShapeType typeA = shapeA->type;
	b3ShapeType typeB = shapeB->type;

	B3_ASSERT( 0 <= typeA && typeA < b3_shapeTypeCount );
	B3_ASSERT( 0 <= typeB && typeB < b3_shapeTypeCount );

	if ( s_registers[typeA][typeB].supported == false )
	{
		// For example, no mesh vs mesh collision
		return;
	}

	if ( s_registers[typeA][typeB].primary == false )
	{
		// flip order
		b3CreateContact( world, shapeB, shapeA, childIndex );
		return;
	}

	b3Body* bodyA = world->bodies.Get( shapeA->bodyId );
	b3Body* bodyB = world->bodies.Get( shapeB->bodyId );

	B3_ASSERT( bodyA->setIndex != b3_disabledSet && bodyB->setIndex != b3_disabledSet );
	B3_ASSERT( bodyA->setIndex != b3_staticSet || bodyB->setIndex != b3_staticSet );

	int setIndex;
	if ( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet )
	{
		setIndex = b3_awakeSet;
	}
	else
	{
		// sleeping and non-touching contacts live in the disabled set
		// later if this set is found to be touching then the sleeping
		// islands will be linked and the contact moved to the merged island

		// This is possible if a shape moves slightly then falls asleep
		setIndex = b3_disabledSet;
	}

	b3SolverSet* set = world->solverSets.Get( setIndex );

	// Create contact key and contact
	int contactId = b3AllocId( &world->contactIdPool );
	if ( contactId == world->contacts.count )
	{
		world->contacts.PushBack( b3Contact{} );
	}

	int shapeIdA = shapeA->id;
	int shapeIdB = shapeB->id;

	b3Contact* contact = world->contacts.Get( contactId );
	contact->contactId = contactId;
	contact->generation += 1;
	contact->setIndex = setIndex;
	contact->colorIndex = B3_NULL_INDEX;
	contact->localIndex = set->contactSims.count;
	contact->islandId = B3_NULL_INDEX;
	contact->islandPrev = B3_NULL_INDEX;
	contact->islandNext = B3_NULL_INDEX;
	contact->shapeIdA = shapeIdA;
	contact->shapeIdB = shapeIdB;
	contact->childIndex = childIndex;
	contact->flags = 0;

	B3_ASSERT( shapeA->sensorIndex == B3_NULL_INDEX && shapeB->sensorIndex == B3_NULL_INDEX );

	if ( shapeA->enableContactEvents || shapeB->enableContactEvents )
	{
		contact->flags |= b3_contactEnableContactEvents;
	}

	// Connect to body A
	{
		contact->edges[0].bodyId = shapeA->bodyId;
		contact->edges[0].prevKey = B3_NULL_INDEX;
		contact->edges[0].nextKey = bodyA->headContactKey;

		int keyA = ( contactId << 1 ) | 0;
		int headContactKey = bodyA->headContactKey;
		if ( headContactKey != B3_NULL_INDEX )
		{
			b3Contact* headContact = world->contacts.Get( headContactKey >> 1 );
			headContact->edges[headContactKey & 1].prevKey = keyA;
		}
		bodyA->headContactKey = keyA;
		bodyA->contactCount += 1;
	}

	// Connect to body B
	{
		contact->edges[1].bodyId = shapeB->bodyId;
		contact->edges[1].prevKey = B3_NULL_INDEX;
		contact->edges[1].nextKey = bodyB->headContactKey;

		int keyB = ( contactId << 1 ) | 1;
		int headContactKey = bodyB->headContactKey;
		if ( bodyB->headContactKey != B3_NULL_INDEX )
		{
			b3Contact* headContact = world->contacts.Get( headContactKey >> 1 );
			headContact->edges[headContactKey & 1].prevKey = keyB;
		}
		bodyB->headContactKey = keyB;
		bodyB->contactCount += 1;
	}

	// Add to pair set for fast lookup
	uint64_t pairKey = b3ShapePairKey( shapeIdA, shapeIdB, childIndex );
	b3AddKey( &world->broadPhase.pairSet, pairKey );

	// Contacts are created as non-touching. Later if they are found to be touching
	// they will link islands and be moved into the constraint graph.
	b3ContactSim* contactSim = set->contactSims.Add();
	contactSim->contactId = contactId;

#if B3_ENABLE_VALIDATION
	contactSim->bodyIdA = shapeA->bodyId;
	contactSim->bodyIdB = shapeB->bodyId;
#endif

	contactSim->bodySimIndexA = B3_NULL_INDEX;
	contactSim->bodySimIndexB = B3_NULL_INDEX;
	contactSim->invMassA = 0.0f;
	contactSim->invIA = {};
	contactSim->invMassB = 0.0f;
	contactSim->invIB = {};
	contactSim->shapeIdA = shapeIdA;
	contactSim->shapeIdB = shapeIdB;
	contactSim->childIndex = childIndex;
	contactSim->cache = {};
	contactSim->manifolds = nullptr;
	contactSim->manifoldCount = 0;
	contactSim->meshQueryBounds = B3_BOUNDS3_EMPTY;
	contactSim->triangleCache = {};

	// These are computed in the narrow phase so they are always fresh
	contactSim->friction = 0.0f;
	contactSim->restitution = 0.0f;

	float radiusA = 0.0f;
	if ( typeA == b3_sphereShape )
	{
		radiusA = shapeA->sphere.radius;
	}
	else if ( typeA == b3_capsuleShape )
	{
		radiusA = shapeA->capsule.radius;
	}

	float radiusB = 0.0f;
	if ( typeB == b3_sphereShape )
	{
		radiusB = shapeB->sphere.radius;
	}
	else if ( typeB == b3_capsuleShape )
	{
		radiusB = shapeB->capsule.radius;
	}

	float maxRadius = b3MaxFloat( radiusA, radiusB );

	// Assuming the rolling resistance doesn't change
	contactSim->rollingResistance =
		b3MaxFloat( shapeA->materials[0].rollingResistance, shapeB->materials[0].rollingResistance ) * maxRadius;

	contactSim->tangentVelocity = b3Vec3_zero;
	contactSim->simFlags = 0;

	if ( shapeA->enablePreSolveEvents || shapeB->enablePreSolveEvents )
	{
		contactSim->simFlags |= b3_simEnablePreSolveEvents;
	}
}

// A contact is destroyed when:
// - broad-phase proxies stop overlapping
// - a body is destroyed
// - a body is disabled
// - a body changes type from dynamic to kinematic or static
// - a shape is destroyed
// - contact filtering is modified
void b3DestroyContact( b3World* world, b3Contact* contact, bool wakeBodies )
{
	// Remove pair from set
	uint64_t pairKey = b3ShapePairKey( contact->shapeIdA, contact->shapeIdB, contact->childIndex );
	b3RemoveKey( &world->broadPhase.pairSet, pairKey );

	b3ContactEdge* edgeA = contact->edges + 0;
	b3ContactEdge* edgeB = contact->edges + 1;

	int bodyIdA = edgeA->bodyId;
	int bodyIdB = edgeB->bodyId;
	b3Body* bodyA = world->bodies.Get( bodyIdA );
	b3Body* bodyB = world->bodies.Get( bodyIdB );

	uint32_t flags = contact->flags;
	bool touching = ( flags & b3_contactTouchingFlag ) != 0;

	// End touch event
	if ( touching && ( flags & b3_contactEnableContactEvents ) != 0 )
	{
		uint16_t worldId = world->worldId;
		const b3Shape* shapeA = world->shapes.Get( contact->shapeIdA );
		const b3Shape* shapeB = world->shapes.Get( contact->shapeIdB );
		b3ShapeId shapeIdA = { shapeA->id + 1, worldId, shapeA->generation };
		b3ShapeId shapeIdB = { shapeB->id + 1, worldId, shapeB->generation };

		b3ContactId contactId = {
			.index1 = contact->contactId + 1,
			.world0 = world->worldId,
			.padding = 0,
			.generation = contact->generation,
		};

		b3ContactEndTouchEvent event = {
			.shapeIdA = shapeIdA,
			.shapeIdB = shapeIdB,
			.contactId = contactId,
		};

		world->contactEndEvents[world->endEventArrayIndex].PushBack( event );
	}

	// Remove from body A
	if ( edgeA->prevKey != B3_NULL_INDEX )
	{
		b3Contact* prevContact = world->contacts.Get( edgeA->prevKey >> 1 );
		b3ContactEdge* prevEdge = prevContact->edges + ( edgeA->prevKey & 1 );
		prevEdge->nextKey = edgeA->nextKey;
	}

	if ( edgeA->nextKey != B3_NULL_INDEX )
	{
		b3Contact* nextContact = world->contacts.Get( edgeA->nextKey >> 1 );
		b3ContactEdge* nextEdge = nextContact->edges + ( edgeA->nextKey & 1 );
		nextEdge->prevKey = edgeA->prevKey;
	}

	int contactId = contact->contactId;

	int edgeKeyA = ( contactId << 1 ) | 0;
	if ( bodyA->headContactKey == edgeKeyA )
	{
		bodyA->headContactKey = edgeA->nextKey;
	}

	bodyA->contactCount -= 1;

	// Remove from body B
	if ( edgeB->prevKey != B3_NULL_INDEX )
	{
		b3Contact* prevContact = world->contacts.Get( edgeB->prevKey >> 1 );
		b3ContactEdge* prevEdge = prevContact->edges + ( edgeB->prevKey & 1 );
		prevEdge->nextKey = edgeB->nextKey;
	}

	if ( edgeB->nextKey != B3_NULL_INDEX )
	{
		b3Contact* nextContact = world->contacts.Get( edgeB->nextKey >> 1 );
		b3ContactEdge* nextEdge = nextContact->edges + ( edgeB->nextKey & 1 );
		nextEdge->prevKey = edgeB->prevKey;
	}

	int edgeKeyB = ( contactId << 1 ) | 1;
	if ( bodyB->headContactKey == edgeKeyB )
	{
		bodyB->headContactKey = edgeB->nextKey;
	}

	bodyB->contactCount -= 1;

	// Remove contact from the array that owns it
	if ( contact->islandId != B3_NULL_INDEX )
	{
		b3UnlinkContact( world, contact );
	}

	if ( contact->colorIndex != B3_NULL_INDEX )
	{
		// contact is an active constraint
		B3_ASSERT( contact->setIndex == b3_awakeSet );
		bool destroy = true;
		b3RemoveContactFromGraph( world, bodyIdA, bodyIdB, contact->colorIndex, contact->localIndex, destroy );
	}
	else
	{
		// contact is non-touching or is sleeping or is a sensor
		B3_ASSERT( contact->setIndex != b3_awakeSet || ( contact->flags & b3_contactTouchingFlag ) == 0 );
		b3SolverSet* set = world->solverSets.Get( contact->setIndex );
		b3ContactSim* sim = set->contactSims.Get( contact->localIndex );
		sim->triangleCache.Destroy();

		if ( sim->manifoldCount > 0 )
		{
			b3FreeManifolds( world, sim->manifolds, sim->manifoldCount );
		}

		int movedIndex = set->contactSims.RemoveSwap( contact->localIndex );
		if ( movedIndex != B3_NULL_INDEX )
		{
			b3ContactSim* movedContactSim = set->contactSims.data + contact->localIndex;
			b3Contact* movedContact = world->contacts.Get( movedContactSim->contactId );
			movedContact->localIndex = contact->localIndex;
		}
	}

	// Free contact and id (preserve generation)
	contact->contactId = B3_NULL_INDEX;
	contact->setIndex = B3_NULL_INDEX;
	contact->colorIndex = B3_NULL_INDEX;
	contact->localIndex = B3_NULL_INDEX;
	b3FreeId( &world->contactIdPool, contactId );

	if ( wakeBodies && touching )
	{
		b3WakeBody( world, bodyA );
		b3WakeBody( world, bodyB );
	}
}

b3ContactSim* b3GetContactSim( b3World* world, b3Contact* contact )
{
	if ( contact->setIndex == b3_awakeSet && contact->colorIndex != B3_NULL_INDEX )
	{
		// contact lives in constraint graph
		B3_ASSERT( 0 <= contact->colorIndex && contact->colorIndex < B3_GRAPH_COLOR_COUNT );
		b3GraphColor* color = world->constraintGraph.colors + contact->colorIndex;
		return color->contactSims.Get( contact->localIndex );
	}

	b3SolverSet* set = world->solverSets.Get( contact->setIndex );
	return set->contactSims.Get( contact->localIndex );
}

static bool b3ComputeManifold( b3World* world, int workerIndex, b3ContactSim* sim, const b3Shape* shapeA, b3Transform xfA,
							   const b3Shape* shapeB, b3Transform xfB )
{
	b3ShapeType typeA = shapeA->type;
	b3ShapeType typeB = shapeB->type;

	b3Manifold manifold;
	manifold.pointCount = 0;

	if ( typeA == b3_sphereShape )
	{
		B3_ASSERT( typeB == b3_sphereShape );
		manifold = b3CollideSpheres( &shapeA->sphere, xfA, &shapeB->sphere, xfB );
	}
	else if ( typeA == b3_capsuleShape )
	{
		if ( typeB == b3_sphereShape )
		{
			manifold = b3CollideCapsuleAndSphere( &shapeA->capsule, xfA, &shapeB->sphere, xfB );
		}
		else
		{
			B3_ASSERT( typeB == b3_capsuleShape );
			manifold = b3CollideCapsules( &shapeA->capsule, xfA, &shapeB->capsule, xfB );
		}
	}
	else
	{
		B3_ASSERT( typeA == b3_hullShape );

		if ( typeB == b3_sphereShape )
		{
			manifold = b3CollideHullAndSphere( shapeA->hull, xfA, &shapeB->sphere, xfB, &sim->cache.simplexCache );
		}
		else if ( typeB == b3_capsuleShape )
		{
			manifold = b3CollideHullAndCapsule( shapeA->hull, xfA, &shapeB->capsule, xfB, &sim->cache.simplexCache );
		}
		else
		{
			B3_ASSERT( typeB == b3_hullShape );
			manifold = b3CollideHulls( shapeA->hull, xfA, shapeB->hull, xfB, &sim->cache.satCache );
			world->taskContexts.data[workerIndex].satCallCount += 1;
			world->taskContexts.data[workerIndex].satCacheHitCount += sim->cache.satCache.hit;
		}
	}

	if ( manifold.pointCount > 0 )
	{
		if ( sim->manifoldCount == 0 )
		{
			sim->manifolds = b3AllocateManifolds( world, 1 );
		}

		memcpy( sim->manifolds, &manifold, sizeof( b3Manifold ) );
		sim->manifoldCount = 1;
	}
	else if ( sim->manifoldCount == 1 )
	{
		b3FreeManifolds( world, sim->manifolds, 1 );
		sim->manifolds = nullptr;
		sim->manifoldCount = 0;
	}

	return sim->manifoldCount > 0;
}

static bool b3UpdateConvexContact( b3World* world, int workerIndex, b3ContactSim* contactSim, b3Shape* shapeA,
								   b3Vec3 centerOffsetA, b3Transform xfA, b3Shape* shapeB, b3Vec3 centerOffsetB, b3Transform xfB,
								   bool flip )
{
	// Save old manifold
	b3Manifold oldManifold;
	if ( contactSim->manifoldCount == 1 )
	{
		memcpy( &oldManifold, contactSim->manifolds, sizeof( b3Manifold ) );
	}
	else
	{
		memset( &oldManifold, 0, sizeof( b3Manifold ) );
	}

	// Compute new manifold
	bool touching = b3ComputeManifold( world, workerIndex, contactSim, shapeA, xfA, shapeB, xfB );

	if ( touching )
	{
		if ( flip )
		{
			B3_ASSERT( contactSim->manifoldCount == 1 );
			contactSim->manifolds[0].normal = b3Neg( contactSim->manifolds[0].normal );

			int pointCount = contactSim->manifolds[0].pointCount;
			for ( int i = 0; i < pointCount; ++i )
			{
				b3ManifoldPoint* mp = contactSim->manifolds[0].points + i;
				B3_SWAP( mp->anchorA, mp->anchorB );
			}
		}

		const b3SurfaceMaterial* materialA = shapeA->materials + 0;
		const b3SurfaceMaterial* materialB = shapeB->materials + 0;

		// Keep these updated in case the values on the shapes are modified
		contactSim->friction = world->frictionCallback( materialA->friction, materialA->userMaterialId, materialB->friction,
														materialB->userMaterialId );
		contactSim->restitution = world->restitutionCallback( materialA->restitution, materialA->userMaterialId,
															  materialB->restitution, materialB->userMaterialId );

		if ( materialA->rollingResistance > 0.0f || materialB->rollingResistance > 0.0f )
		{
			b3ShapeType typeA = shapeA->type;
			b3ShapeType typeB = shapeB->type;

			float radiusA = 0.0f;
			if ( typeA == b3_sphereShape )
			{
				radiusA = shapeA->sphere.radius;
			}
			else if ( typeA == b3_capsuleShape )
			{
				radiusA = shapeA->capsule.radius;
			}

			float radiusB = 0.0f;
			if ( typeB == b3_sphereShape )
			{
				radiusB = shapeB->sphere.radius;
			}
			else if ( typeB == b3_capsuleShape )
			{
				radiusB = shapeB->capsule.radius;
			}

			float maxRadius = b3MaxFloat( radiusA, radiusB );
			contactSim->rollingResistance = b3MaxFloat( materialA->rollingResistance, materialB->rollingResistance ) * maxRadius;
		}
		else
		{
			contactSim->rollingResistance = 0.0f;
		}

		b3Vec3 tangentVelocityA = b3RotateVector( xfA.q, materialA->tangentVelocity );
		b3Vec3 tangentVelocityB = b3RotateVector( xfB.q, materialB->tangentVelocity );
		contactSim->tangentVelocity = b3Sub( tangentVelocityA, tangentVelocityB );

		// These get persisted regardless of feature ids
		if ( oldManifold.pointCount > 0 )
		{
			contactSim->manifolds[0].frictionImpulse = oldManifold.frictionImpulse;
			contactSim->manifolds[0].twistImpulse = oldManifold.twistImpulse;
			contactSim->manifolds[0].rollingImpulse = oldManifold.rollingImpulse;
		}
		else
		{
			contactSim->manifolds[0].frictionImpulse = b3Vec3_zero;
			contactSim->manifolds[0].twistImpulse = 0.0f;
			contactSim->manifolds[0].rollingImpulse = b3Vec3_zero;
		}
	}

	if ( touching && world->preSolveFcn && ( contactSim->simFlags & b3_simEnablePreSolveEvents ) != 0 )
	{
		b3ShapeId shapeIdA = { shapeA->id + 1, world->worldId, shapeA->generation };
		b3ShapeId shapeIdB = { shapeB->id + 1, world->worldId, shapeB->generation };

		b3Manifold* manifold = contactSim->manifolds + 0;
		b3Vec3 bestPoint = manifold->points[0].point;
		float bestSeparation = manifold->points[0].separation;

		// Get deepest point
		for ( int i = 1; i < manifold->pointCount; ++i )
		{
			float separation = manifold->points[i].separation;
			if ( separation < bestSeparation )
			{
				bestSeparation = separation;
				bestPoint = manifold->points[i].point;
			}
		}

		// this call assumes thread safety
		touching = world->preSolveFcn( shapeIdA, shapeIdB, bestPoint, manifold->normal, world->preSolveContext );
		if ( touching == false )
		{
			// disable contact
			manifold->pointCount = 0;
		}
	}

	if ( touching && ( shapeA->enableHitEvents || shapeB->enableHitEvents ) )
	{
		contactSim->simFlags |= b3_simEnableHitEvent;
	}
	else
	{
		contactSim->simFlags &= ~b3_simEnableHitEvent;
	}

	// Match old contact ids to new contact ids and copy the
	// stored impulses to warm start the solver.
	int pointCount = touching ? contactSim->manifolds[0].pointCount : 0;
	for ( int i = 0; i < pointCount; ++i )
	{
		b3ManifoldPoint* mp2 = contactSim->manifolds[0].points + i;

		// shift anchors to be center of mass relative
		mp2->anchorA = b3Sub( mp2->anchorA, centerOffsetA );
		mp2->anchorB = b3Sub( mp2->anchorB, centerOffsetB );

		mp2->normalImpulse = 0.0f;
		mp2->totalNormalImpulse = 0.0f;
		mp2->normalVelocity = 0.0f;
		mp2->persisted = false;

		uint32_t id2 = mp2->id;

		for ( int j = 0; j < oldManifold.pointCount; ++j )
		{
			b3ManifoldPoint* mp1 = oldManifold.points + j;

			if ( mp1->id == id2 )
			{
				mp2->normalImpulse = mp1->normalImpulse;
				mp2->persisted = true;

				// clear old impulse
				mp1->normalImpulse = 0.0f;
				break;
			}
		}
	}

	return touching;
}

// Update the contact manifold and touching status.
// Note: do not assume the shape AABBs are overlapping or are valid.
bool b3UpdateContact( b3World* world, int workerIndex, b3ContactSim* contactSim, b3Shape* shapeA, b3Vec3 centerOffsetA,
					  b3Transform xfA, b3Shape* shapeB, b3Vec3 centerOffsetB, b3Transform xfB )
{
	bool touching;

	B3_ASSERT( shapeB->type != b3_compoundShape );

	if ( shapeA->type == b3_compoundShape )
	{
		int childIndex = contactSim->childIndex;
		b3ChildShape child = b3GetCompoundChild( shapeA->compound, childIndex );

		// Temporary child shape to match existing function signatures
		b3Shape childShapeA;
		memcpy( &childShapeA, shapeA, sizeof( b3Shape ) );

		childShapeA.type = child.type;

		if ( child.type == b3_capsuleShape )
		{
			childShapeA.capsule = child.capsule;
			if ( shapeB->type == b3_hullShape )
			{
				// Flip
				bool flip = true;
				touching = b3UpdateConvexContact( world, workerIndex, contactSim, shapeB, centerOffsetB, xfB, &childShapeA,
												  centerOffsetA, xfA, flip );
			}
			else
			{
				bool flip = false;
				touching = b3UpdateConvexContact( world, workerIndex, contactSim, &childShapeA, centerOffsetA, xfA, shapeB,
												  centerOffsetB, xfB, flip );
			}
		}
		else if ( child.type == b3_hullShape )
		{
			childShapeA.hull = child.hull;
			xfA = b3MulTransforms( xfA, child.transform );
			bool flip = false;
			touching = b3UpdateConvexContact( world, workerIndex, contactSim, &childShapeA, centerOffsetA, xfA, shapeB,
											  centerOffsetB, xfB, flip );
		}
		else if ( child.type == b3_meshShape )
		{
			childShapeA.mesh = child.mesh;
			xfA = b3MulTransforms( xfA, child.transform );

			touching =
				b3ComputeMeshManifolds( world, workerIndex, contactSim, &childShapeA, xfA, child.materialIndices, shapeB, xfB );

			if ( touching && ( shapeA->enableHitEvents || shapeB->enableHitEvents ) )
			{
				contactSim->simFlags |= b3_simEnableHitEvent;
			}
			else
			{
				contactSim->simFlags &= ~b3_simEnableHitEvent;
			}

			B3_ASSERT( ( touching == true && contactSim->manifoldCount > 0 ) ||
					   ( touching == false && contactSim->manifoldCount == 0 ) );

			// Adjust anchors to be relative to center of mass
			for ( int i = 0; i < contactSim->manifoldCount; ++i )
			{
				b3Manifold* manifold = contactSim->manifolds + i;
				int pointCount = manifold->pointCount;
				for ( int j = 0; j < pointCount; ++j )
				{
					b3ManifoldPoint* mp = manifold->points + j;
					mp->anchorA -= centerOffsetA;
					mp->anchorB -= centerOffsetB;
				}
			}
		}
		else
		{
			B3_ASSERT( child.type == b3_sphereShape );

			childShapeA.sphere = child.sphere;
			if ( shapeB->type == b3_capsuleShape || shapeB->type == b3_hullShape )
			{
				// Flip
				bool flip = true;
				touching = b3UpdateConvexContact( world, workerIndex, contactSim, shapeB, centerOffsetB, xfB, &childShapeA,
												  centerOffsetA, xfA, flip );
			}
			else
			{
				bool flip = false;
				touching = b3UpdateConvexContact( world, workerIndex, contactSim, &childShapeA, centerOffsetA, xfA, shapeB,
												  centerOffsetB, xfB, flip );
			}
		}
	}
	else if ( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape )
	{
		// Does this contact touch a mesh or height-field?

		// Compute mesh manifolds
		touching = b3ComputeMeshManifolds( world, workerIndex, contactSim, shapeA, xfA, nullptr, shapeB, xfB );

		if ( touching && ( shapeA->enableHitEvents || shapeB->enableHitEvents ) )
		{
			contactSim->simFlags |= b3_simEnableHitEvent;
		}
		else
		{
			contactSim->simFlags &= ~b3_simEnableHitEvent;
		}

		B3_ASSERT( ( touching == true && contactSim->manifoldCount > 0 ) ||
				   ( touching == false && contactSim->manifoldCount == 0 ) );

		// Adjust anchors to be relative to center of mass
		for ( int i = 0; i < contactSim->manifoldCount; ++i )
		{
			b3Manifold* manifold = contactSim->manifolds + i;
			int pointCount = manifold->pointCount;
			for ( int j = 0; j < pointCount; ++j )
			{
				b3ManifoldPoint* mp = manifold->points + j;
				mp->anchorA -= centerOffsetA;
				mp->anchorB -= centerOffsetB;
			}
		}
	}
	else
	{
		// Convex-vs-convex
		bool flip = false;
		touching =
			b3UpdateConvexContact( world, workerIndex, contactSim, shapeA, centerOffsetA, xfA, shapeB, centerOffsetB, xfB, flip );
	}

	if ( touching )
	{
		contactSim->simFlags |= b3_simTouchingFlag;
	}
	else
	{
		contactSim->simFlags &= ~b3_simTouchingFlag;
	}

	return touching;
}
