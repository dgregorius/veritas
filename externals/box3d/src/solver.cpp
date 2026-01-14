// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver.h"

#include "arena_allocator.h"
#include "array.h"
#include "bitset.h"
#include "body.h"
#include "contact.h"
#include "contact_solver.h"
#include "core.h"
#include "ctz.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"
#include "platform.h"
#include "sensor.h"
#include "shape.h"
#include "solver_set.h"

#include <limits.h>
#include <stddef.h>

#if ( defined( __GNUC__ ) || defined( __clang__ ) ) && ( defined( __i386__ ) || defined( __x86_64__ ) )
static void b3Pause()
{
	__asm__ __volatile__( "pause\n" );
}
#elif ( defined( __arm__ ) && defined( __ARM_ARCH ) && __ARM_ARCH >= 7 ) || defined( __aarch64__ )
static void b3Pause()
{
	__asm__ __volatile__( "yield" ::: "memory" );
}
#elif defined( _MSC_VER ) && ( defined( _M_IX86 ) || defined( _M_X64 ) )
static void b3Pause()
{
	_mm_pause();
}
#elif defined( _MSC_VER ) && ( defined( _M_ARM ) || defined( _M_ARM64 ) )
static void b3Pause()
{
	__yield();
}
#else
static void b3Pause()
{
}
#endif

typedef struct b3WorkerContext
{
	b3StepContext* context;
	int workerIndex;
	void* userTask;
} b3WorkerContext;

// Integrate velocities and apply damping
static void b3IntegrateVelocitiesTask( int startIndex, int endIndex, b3StepContext* context )
{
	b3TracyCZoneNC( integrate_velocity, "IntVel", b3_colorDeepPink, true );

	b3BodyState* states = context->states;
	b3BodySim* sims = context->sims;

	b3Vec3 gravity = context->world->gravity;
	float h = context->h;
	float maxLinearSpeed = context->maxLinearVelocity;
	float maxAngularSpeed = B3_MAX_ROTATION * context->inv_dt;
	float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
	float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3BodySim* sim = sims + i;
		b3BodyState* state = states + i;

		b3Vec3 v = state->linearVelocity;
		b3Vec3 w = state->angularVelocity;

		// Damping math
		// Differential equation: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		float linearDamping = 1.0f / ( 1.0f + h * sim->linearDamping );
		float angularDamping = 1.0f / ( 1.0f + h * sim->angularDamping );

		// Gravity scale will be zero for kinematic bodies
		float gravityScale = sim->invMass > 0.0f ? sim->gravityScale : 0.0f;

		b3Vec3 linearVelocityDelta = b3Add( b3MulSV( h * sim->invMass, sim->force ), b3MulSV( h * gravityScale, gravity ) );
		v = b3MulAdd( linearVelocityDelta, linearDamping, v );

		// todo_erin gyroscopic
		b3Vec3 angularVelocityDelta = h * ( sim->invInertiaWorld * sim->torque );
		w = angularVelocityDelta + angularDamping * w;

		// Gyroscopic torque by solving this nonlinear equation using Newton-Raphson.
		// I * (w2 - w1) + h * cross(w2, I * w2) = 0
		// This is all done in local coordinates where the Jacobian is easier to compute.
		// This improves the simulation of long skinny bodies.
		{
			// Get current rotation.
			b3Quat q0 = sim->transform.q;
			b3Quat q = b3MulQuat( state->deltaRotation, q0 );

			// todo wasteful computation
			b3Matrix3 inertiaLocal = b3InvertMatrix( sim->invInertiaLocal );

			// Compute local angular velocity
			b3Vec3 omega1 = b3InvRotateVector( q, w );
			b3Vec3 omega2 = omega1;

			// Two iterations is better, but the matrix multiplication is expensive
			// todo optimize math
			for ( int gyroIteration = 0; gyroIteration < 1; ++gyroIteration )
			{
				// Residual
				b3Vec3 b = b3MulMV( inertiaLocal, omega2 - omega1 ) + h * b3Cross( omega2, b3MulMV( inertiaLocal, omega2 ) );

				// Jacobian derived by Erin Catto, Ph.D. Do not attempt to do this without a Ph.D.
				b3Matrix3 J = inertiaLocal;
				//b3Matrix3 B1;

				/*
					[          -i12*w3 + i13*w2            i13*w1 - i22*w3 + 2*i23*w2 + i33*w3   -i12*w1 - i22*w2 - 2*i23*w3 + i33*w2]
					[                                                                                                                ]
					[i11*w3 - 2*i13*w1 - i23*w2 - i33*w3             i12*w3 - i23*w1             i11*w1 + i12*w2 + 2*i13*w3 - i33*w1 ]
					[                                                                                                                ]
					[-i11*w2 + 2*i12*w1 + i22*w2 + i23*w3  -i11*w1 - 2*i12*w2 - i13*w3 + i22*w1            -i13*w2 + i23*w1          ]
				*/

				//B1.cx = {-inertiaLocal.cx.y * omega2.z + }
				J += h * ( b3MulMM( b3Skew( omega2 ), inertiaLocal ) - b3Skew( b3MulMV( inertiaLocal, omega2 ) ) );

				omega2 -= b3Solve3( J, b );
			}

			w = b3RotateVector( q, omega2 );
		}

		// Clamp to max linear speed
		if ( b3Dot( v, v ) > maxLinearSpeedSquared )
		{
			float ratio = maxLinearSpeed / b3Length( v );
			v = b3MulSV( ratio, v );
			sim->flags |= b3_isSpeedCapped;
		}

		// Clamp to max angular speed
		if ( b3Dot( w, w ) > maxAngularSpeedSquared && ( sim->flags & b3_allowFastRotation ) == 0 )
		{
			float ratio = maxAngularSpeed / b3Length( w );
			w *= ratio;
			sim->flags |= b3_isSpeedCapped;
		}

		v.x = ( state->flags & b3_lockLinearX ) ? 0.0f : v.x;
		v.y = ( state->flags & b3_lockLinearY ) ? 0.0f : v.y;
		v.z = ( state->flags & b3_lockLinearZ ) ? 0.0f : v.z;

		w.x = ( state->flags & b3_lockAngularX ) ? 0.0f : w.x;
		w.y = ( state->flags & b3_lockAngularY ) ? 0.0f : w.y;
		w.z = ( state->flags & b3_lockAngularZ ) ? 0.0f : w.z;

		state->linearVelocity = v;
		state->angularVelocity = w;
	}

	b3TracyCZoneEnd( integrate_velocity );
}

static void b3PrepareJointsTask( int startIndex, int endIndex, b3StepContext* context )
{
	b3TracyCZoneNC( prepare_joints, "PrepJoints", b3_colorOldLace, true );

	b3JointSim** joints = context->joints;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3JointSim* joint = joints[i];
		b3PrepareJoint( joint, context );
	}

	b3TracyCZoneEnd( prepare_joints );
}

static void b3WarmStartJointsTask( int startIndex, int endIndex, b3StepContext* context, int colorIndex )
{
	b3TracyCZoneNC( warm_joints, "WarmJoints", b3_colorGold, true );

	b3GraphColor* color = context->graph->colors + colorIndex;
	b3JointSim* joints = color->jointSims.data;
	B3_ASSERT( 0 <= startIndex && startIndex < color->jointSims.count );
	B3_ASSERT( startIndex <= endIndex && endIndex <= color->jointSims.count );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3JointSim* joint = joints + i;
		b3WarmStartJoint( joint, context );
	}

	b3TracyCZoneEnd( warm_joints );
}

static void b3SolveJointsTask( int startIndex, int endIndex, b3StepContext* context, int colorIndex, bool useBias,
							   int workerIndex )
{
	b3TracyCZoneNC( solve_joints, "SolveJoints", b3_colorLemonChiffon, true );

	b3GraphColor* color = context->graph->colors + colorIndex;
	b3JointSim* joints = color->jointSims.data;
	B3_ASSERT( 0 <= startIndex && startIndex < color->jointSims.count );
	B3_ASSERT( startIndex <= endIndex && endIndex <= color->jointSims.count );

	b3BitSet* jointStateBitSet = &context->world->taskContexts.data[workerIndex].jointStateBitSet;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3JointSim* joint = joints + i;
		b3SolveJoint( joint, context, useBias );

		if ( useBias && ( joint->forceThreshold < FLT_MAX || joint->torqueThreshold < FLT_MAX ) &&
			 b3GetBit( jointStateBitSet, joint->jointId ) == false )
		{
			float force, torque;
			b3GetJointReaction( context->world, joint, context->inv_h, &force, &torque );

			// Check thresholds. A zero threshold means all awake joints get reported.
			if ( force >= joint->forceThreshold || torque >= joint->torqueThreshold )
			{
				// Flag this joint for processing.
				b3SetBit( jointStateBitSet, joint->jointId );
			}
		}
	}

	b3TracyCZoneEnd( solve_joints );
}

static void b3IntegratePositionsTask( int startIndex, int endIndex, b3StepContext* context )
{
	b3TracyCZoneNC( integrate_positions, "IntPos", b3_colorDarkSeaGreen, true );

	b3BodyState* states = context->states;
	float h = context->h;

	B3_ASSERT( startIndex <= endIndex );

	float maxAngularSpeed = B3_MAX_ROTATION * context->inv_dt;
	float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3BodyState* state = states + i;

		b3Vec3 v = state->linearVelocity;
		b3Vec3 w = state->angularVelocity;

		v.x = ( state->flags & b3_lockLinearX ) ? 0.0f : v.x;
		v.y = ( state->flags & b3_lockLinearY ) ? 0.0f : v.y;
		v.z = ( state->flags & b3_lockLinearZ ) ? 0.0f : v.z;

		w.x = ( state->flags & b3_lockAngularX ) ? 0.0f : w.x;
		w.y = ( state->flags & b3_lockAngularY ) ? 0.0f : w.y;
		w.z = ( state->flags & b3_lockAngularZ ) ? 0.0f : w.z;

		// Clamp to max angular speed so that rotation integration work has sufficient accuracy
		if ( (state->flags & b3_allowFastRotation) == 0 && b3Dot( w, w ) > maxAngularSpeedSquared )
		{
			float ratio = maxAngularSpeed / b3Length( w );
			w *= ratio;

			// Update the angular velocity for solver consistency
			state->angularVelocity = w;
		}

		state->linearVelocity = v;
		state->angularVelocity = w;

		state->deltaRotation = b3IntegrateRotation( state->deltaRotation, h * w );
		state->deltaPosition = b3MulAdd( state->deltaPosition, h, v );
	}

	b3TracyCZoneEnd( integrate_positions );
}

#define B2_MAX_CONTINUOUS_SENSOR_HITS 8

struct b3ContinuousContext
{
	b3World* world;
	b3BodySim* fastBodySim;
	b3Shape* fastShape;
	b3Vec3 centroid1, centroid2;
	b3Sweep sweep;
	float fraction;
	b3SensorHit sensorHits[B2_MAX_CONTINUOUS_SENSOR_HITS];
	float sensorFractions[B2_MAX_CONTINUOUS_SENSOR_HITS];
	int sensorCount;
};

// This is called from b3DynamicTree_Query for continuous collision
static bool b3ContinuousQueryCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = int( userData );
	b3ContinuousContext* continuousContext = (b3ContinuousContext*)context;
	b3Shape* fastShape = continuousContext->fastShape;
	b3BodySim* fastBodySim = continuousContext->fastBodySim;

	B3_ASSERT( fastShape->sensorIndex == B3_NULL_INDEX );

	// Skip same shape
	if ( shapeId == fastShape->id )
	{
		return true;
	}

	b3World* world = continuousContext->world;

	b3Shape* shape = world->shapes.Get( shapeId );

	// Skip same body
	if ( shape->bodyId == fastShape->bodyId )
	{
		return true;
	}

	// Skip sensors unless the shapes want sensor events
	bool isSensor = shape->sensorIndex != B3_NULL_INDEX;
	if ( isSensor && ( shape->enableSensorEvents == false || fastShape->enableSensorEvents == false ) )
	{
		return true;
	}

	// Skip filtered shapes
	bool canCollide = b3ShouldShapesCollide( fastShape->filter, shape->filter );
	if ( canCollide == false )
	{
		return true;
	}

	b3Body* body = world->bodies.Get( shape->bodyId );

	b3BodySim* bodySim = b3GetBodySim( world, body );
	B3_ASSERT( body->type == b3_staticBody || ( fastBodySim->flags & b3_isBullet ) );

	// Skip bullets
	if ( bodySim->flags & b3_isBullet )
	{
		return true;
	}

	// Skip filtered bodies
	b3Body* fastBody = world->bodies.Get( fastBodySim->bodyId );
	canCollide = b3ShouldBodiesCollide( world, fastBody, body );
	if ( canCollide == false )
	{
		return true;
	}

	// Custom user filtering
	if ( shape->enableCustomFiltering || fastShape->enableCustomFiltering )
	{
		b3CustomFilterFcn* customFilterFcn = world->customFilterFcn;
		if ( customFilterFcn != nullptr )
		{
			b3ShapeId idA = { shape->id + 1, world->worldId, shape->generation };
			b3ShapeId idB = { fastShape->id + 1, world->worldId, fastShape->generation };
			canCollide = customFilterFcn( idA, idB, world->customFilterContext );
			if ( canCollide == false )
			{
				return true;
			}
		}
	}

	// todo does having a sweep on shapeA help with bullets?
	b3Sweep sweepA = b3MakeSweep( bodySim );

	// Time of impact versus shape. Supports all shape types
	b3TOIOutput output = b3ShapeTimeOfImpact( shape, fastShape, &sweepA, &continuousContext->sweep, continuousContext->fraction );

	if (isSensor)
	{
		// Only accept a sensor hit that is sooner than the current solid hit.
		if ( output.fraction <= continuousContext->fraction && continuousContext->sensorCount < B2_MAX_CONTINUOUS_SENSOR_HITS )
		{
			int index = continuousContext->sensorCount;

			// The hit shape is a sensor
			b3SensorHit sensorHit = {
				.sensorId = shape->id,
				.visitorId = fastShape->id,
			};

			continuousContext->sensorHits[index] = sensorHit;
			continuousContext->sensorFractions[index] = output.fraction;
			continuousContext->sensorCount += 1;
		}
	}
	else if ( 0.0f < output.fraction && output.fraction < continuousContext->fraction )
	{
		bool didHit = true;

		if ( didHit && ( shape->enablePreSolveEvents || fastShape->enablePreSolveEvents ) )
		{
			b3ShapeId shapeIdA = { shape->id + 1, world->worldId, shape->generation };
			b3ShapeId shapeIdB = { fastShape->id + 1, world->worldId, fastShape->generation };
			didHit = world->preSolveFcn( shapeIdA, shapeIdB, output.point, output.normal, world->preSolveContext );
		}

		if ( didHit )
		{
			fastBodySim->flags |= b3_hadTimeOfImpact;
			continuousContext->fraction = output.fraction;
		}
	}

	// Continue query
	return true;
}

// Continuous collision of dynamic versus static
static void b3SolveContinuous( b3World* world, int bodySimIndex, b3TaskContext* taskContext )
{
	b3TracyCZoneNC( fast_body, "Fast Body", b3_colorDarkGoldenRod, true );

	b3SolverSet* awakeSet = world->solverSets.Get( b3_awakeSet );
	b3BodySim* fastBodySim = awakeSet->bodySims.Get( bodySimIndex );
	B3_ASSERT( fastBodySim->flags & b3_isFast );

	b3Sweep sweep = b3MakeSweep( fastBodySim );

	b3Transform xf1;
	xf1.q = sweep.q1;
	xf1.p = b3Sub( sweep.c1, b3RotateVector( sweep.q1, sweep.localCenter ) );

	b3Transform xf2;
	xf2.q = sweep.q2;
	xf2.p = b3Sub( sweep.c2, b3RotateVector( sweep.q2, sweep.localCenter ) );

	b3DynamicTree* staticTree = world->broadPhase.trees + b3_staticBody;
	b3DynamicTree* kinematicTree = world->broadPhase.trees + b3_kinematicBody;
	b3DynamicTree* dynamicTree = world->broadPhase.trees + b3_dynamicBody;
	b3Body* fastBody = world->bodies.Get( fastBodySim->bodyId );

	b3ContinuousContext context = {};
	context.world = world;
	context.sweep = sweep;
	context.fastBodySim = fastBodySim;
	context.fraction = 1.0f;

	bool isBullet = ( fastBodySim->flags & b3_isBullet ) != 0;

	int shapeId = fastBody->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		b3Shape* fastShape = world->shapes.Get( shapeId );
		shapeId = fastShape->nextShapeId;

		context.fastShape = fastShape;
		context.centroid1 = b3TransformPoint( xf1, fastShape->localCentroid );
		context.centroid2 = b3TransformPoint( xf2, fastShape->localCentroid );

		b3AABB box1 = fastShape->aabb;
		b3AABB box2 = b3ComputeShapeAABB( fastShape, xf2 );

		// Store this to avoid double computation in the case there is no impact event
		fastShape->aabb = box2;

		// No continuous collision for meshes
		if ( fastShape->type == b3_meshShape || fastShape->type == b3_heightShape )
		{
			continue;
		}

		// No continuous collision for sensors
		if ( fastShape->sensorIndex != B3_NULL_INDEX )
		{
			continue;
		}

		b3AABB sweptBox = b3AABB_Union( box1, box2 );
		b3DynamicTree_Query( staticTree, sweptBox, B3_DEFAULT_MASK_BITS, false, b3ContinuousQueryCallback, &context );

		if ( isBullet )
		{
			b3DynamicTree_Query( kinematicTree, sweptBox, B3_DEFAULT_MASK_BITS, false, b3ContinuousQueryCallback, &context );
			b3DynamicTree_Query( dynamicTree, sweptBox, B3_DEFAULT_MASK_BITS, false, b3ContinuousQueryCallback, &context );
		}
	}

	const float speculativeScalar = B3_SPECULATIVE_DISTANCE;
	const float marginScalar = B3_AABB_MARGIN;
	const b3Vec3 speculativeMargin = { speculativeScalar, speculativeScalar, speculativeScalar };
	const b3Vec3 aabbMargin = { marginScalar, marginScalar, marginScalar };

	if ( context.fraction < 1.0f )
	{
		// Handle time of impact event
		b3Quat q = b3NLerp( sweep.q1, sweep.q2, context.fraction );
		b3Vec3 c = b3Lerp( sweep.c1, sweep.c2, context.fraction );
		b3Vec3 origin = b3Sub( c, b3RotateVector( q, sweep.localCenter ) );

		// Advance body
		b3Transform transform = { origin, q };
		fastBodySim->transform = transform;
		fastBodySim->center = c;
		fastBodySim->rotation0 = q;
		fastBodySim->center0 = c;

		// Prepare AABBs for broad-phase.
		// Even though a body is fast, it may not move much. So the AABB may not need enlargement.

		shapeId = fastBody->headShapeId;
		while ( shapeId != B3_NULL_INDEX )
		{
			b3Shape* shape = world->shapes.Get( shapeId );

			// Must recompute aabb at the interpolated transform
			b3AABB aabb = b3ComputeShapeAABB( shape, transform );
			aabb.lowerBound -= speculativeMargin;
			aabb.upperBound += speculativeMargin;
			shape->aabb = aabb;

			if ( b3AABB_Contains( shape->fatAABB, aabb ) == false )
			{
				shape->fatAABB = { aabb.lowerBound - aabbMargin, aabb.upperBound + aabbMargin };

				shape->enlargedAABB = true;
				fastBodySim->flags |= b3_enlargeBounds;
			}

			shapeId = shape->nextShapeId;
		}
	}
	else
	{
		// No time of impact event

		// Advance body
		fastBodySim->rotation0 = fastBodySim->transform.q;
		fastBodySim->center0 = fastBodySim->center;

		// Prepare AABBs for broad-phase
		shapeId = fastBody->headShapeId;
		while ( shapeId != B3_NULL_INDEX )
		{
			b3Shape* shape = world->shapes.Get( shapeId );

			// shape->aabb is still valid from above

			if ( b3AABB_Contains( shape->fatAABB, shape->aabb ) == false )
			{
				shape->fatAABB = { shape->aabb.lowerBound - aabbMargin, shape->aabb.upperBound + aabbMargin };

				if ( b3IsSaneAABB( shape->fatAABB ) == false )
				{
					b3Vec3 c = fastBodySim->center;
					b3Log( "body %s out of bounds at %g %g %g", fastBody->name, c.x, c.y, c.z );
				}

				// B3_ASSERT( b3IsSaneAABB( shape->fatAABB ) );

				shape->enlargedAABB = true;
				fastBodySim->flags |= b3_enlargeBounds;
			}

			shapeId = shape->nextShapeId;
		}
	}
	
	// Push sensor hits on the the task context for serial processing.
	for ( int i = 0; i < context.sensorCount; ++i )
	{
		// Skip any sensor hits that occurred after a solid hit
		if ( context.sensorFractions[i] < context.fraction )
		{
			taskContext->sensorHits.PushBack( context.sensorHits[i] );
		}
	}

	b3TracyCZoneEnd( fast_body );
}

static void b3FinalizeBodiesTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b3TracyCZoneNC( finalize_bodies, "Positions", b3_colorMediumSeaGreen, true );

	b3StepContext* stepContext = (b3StepContext*)context;
	b3World* world = stepContext->world;

	B3_ASSERT( (int)threadIndex < world->workerCount );

	bool enableSleep = world->enableSleep;
	b3BodyState* states = stepContext->states;
	b3BodySim* sims = stepContext->sims;
	b3Body* bodies = world->bodies.data;
	float timeStep = stepContext->dt;
	float invTimeStep = stepContext->inv_dt;

	uint16_t worldId = world->worldId;

	// The body move event array has should already have the correct size
	B3_ASSERT( endIndex <= world->bodyMoveEvents.count );
	b3BodyMoveEvent* moveEvents = world->bodyMoveEvents.data;

	b3BitSet* enlargedSimBitSet = &world->taskContexts.data[threadIndex].enlargedSimBitSet;
	b3BitSet* awakeIslandBitSet = &world->taskContexts.data[threadIndex].awakeIslandBitSet;
	b3TaskContext* taskContext = world->taskContexts.data + threadIndex;

	bool enableContinuous = world->enableContinuous;

	const float speculativeScalar = B3_SPECULATIVE_DISTANCE;
	const float marginScalar = B3_AABB_MARGIN;
	const b3Vec3 speculativeMargin = { speculativeScalar, speculativeScalar, speculativeScalar };
	const b3Vec3 aabbMargin = { marginScalar, marginScalar, marginScalar };

	B3_ASSERT( startIndex <= endIndex );

	for ( int simIndex = startIndex; simIndex < endIndex; ++simIndex )
	{
		b3BodyState* state = states + simIndex;
		b3BodySim* sim = sims + simIndex;

		b3Vec3 v = state->linearVelocity;
		b3Vec3 w = state->angularVelocity;

		v.x = ( state->flags & b3_lockLinearX ) ? 0.0f : v.x;
		v.y = ( state->flags & b3_lockLinearY ) ? 0.0f : v.y;
		v.z = ( state->flags & b3_lockLinearZ ) ? 0.0f : v.z;

		w.x = ( state->flags & b3_lockAngularX ) ? 0.0f : w.x;
		w.y = ( state->flags & b3_lockAngularY ) ? 0.0f : w.y;
		w.z = ( state->flags & b3_lockAngularZ ) ? 0.0f : w.z;

		state->linearVelocity = v;
		state->angularVelocity = w;

		if ( b3IsValidVec3( v ) == false || b3IsValidVec3( w ) == false )
		{
			b3Log( "unstable: %s, force = %g %g %g", bodies[sim->bodyId].name, sim->force.x, sim->force.y, sim->force.z );
		}

		B3_ASSERT( b3IsValidVec3( v ) );
		B3_ASSERT( b3IsValidVec3( w ) );

		sim->center = b3Add( sim->center, state->deltaPosition );
		sim->transform.q = b3NormalizeQuat( b3MulQuat( state->deltaRotation, sim->transform.q ) );

		// Use the velocity of the farthest point on the body to account for rotation.
		float maxVelocity = b3Length( v ) + b3Length( w ) * sim->maxExtent;

		// Sleep needs to observe position correction as well as true velocity.
		float maxDeltaPosition = b3Length( state->deltaPosition ) + b3Length( state->deltaRotation.v ) * sim->maxExtent;

		// Position correction is not as important for sleep as true velocity.
		float positionSleepFactor = 0.5f;

		float sleepVelocity = b3MaxFloat( maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition );

		// reset state deltas
		state->deltaPosition = b3Vec3_zero;
		state->deltaRotation = b3Quat_identity;

		sim->transform.p = b3Sub( sim->center, b3RotateVector( sim->transform.q, sim->localCenter ) );

		// cache miss here, however I need the shape list below
		b3Body* body = bodies + sim->bodyId;
		body->bodyMoveIndex = simIndex;
		moveEvents[simIndex].userData = body->userData;
		moveEvents[simIndex].transform = sim->transform;
		moveEvents[simIndex].bodyId = b3BodyId{ sim->bodyId + 1, worldId, body->generation };
		moveEvents[simIndex].fellAsleep = false;

		// reset applied force and torque
		sim->force = b3Vec3_zero;
		sim->torque = b3Vec3_zero;

		body->flags &= ~( b3_isFast | b3_isSpeedCapped | b3_hadTimeOfImpact );
		body->flags |= ( sim->flags & ( b3_isSpeedCapped | b3_hadTimeOfImpact ) );
		sim->flags &= ~( b3_isFast | b3_isSpeedCapped | b3_hadTimeOfImpact );

		if ( enableSleep == false || body->enableSleep == false || sleepVelocity > body->sleepThreshold )
		{
			// Body is not sleepy
			body->sleepTime = 0.0f;

			const float safetyFactor = 0.5f;
			if ( body->type == b3_dynamicBody && enableContinuous && maxVelocity * timeStep > safetyFactor * sim->minExtent )
			{
				// This flag is only retained for debug draw
				sim->flags |= b3_isFast;

				// Store in fast array for the continuous collision stage
				// This is deterministic because the order of TOI sweeps doesn't matter
				if ( sim->flags & b3_isBullet )
				{
					int bulletIndex = b3AtomicFetchAddInt( &stepContext->bulletBodyCount, 1 );
					stepContext->bulletBodies[bulletIndex] = simIndex;
				}
				else
				{
					b3SolveContinuous( world, simIndex, taskContext );
				}
			}
			else
			{
				// Body is safe to advance
				sim->center0 = sim->center;
				sim->rotation0 = sim->transform.q;
			}
		}
		else
		{
			// Body is safe to advance and is falling asleep
			sim->center0 = sim->center;
			sim->rotation0 = sim->transform.q;
			body->sleepTime += timeStep;
		}

		// Update world space inverse inertia tensor.
		b3Matrix3 rotationMatrix = b3MakeMatrixFromQuat( sim->transform.q );
		sim->invInertiaWorld = rotationMatrix * sim->invInertiaLocal * b3Transpose( rotationMatrix );

		// Any single body in an island can keep it awake
		b3Island* island = world->islands.Get( body->islandId );
		if ( body->sleepTime < B3_TIME_TO_SLEEP )
		{
			// keep island awake
			int islandIndex = island->localIndex;
			b3SetBit( awakeIslandBitSet, islandIndex );
		}
		else if ( island->constraintRemoveCount > 0 )
		{
			// body wants to sleep but its island needs splitting first
			if ( body->sleepTime > taskContext->splitSleepTime )
			{
				// pick the sleepiest candidate
				taskContext->splitIslandId = body->islandId;
				taskContext->splitSleepTime = body->sleepTime;
			}
		}

		// Update shapes AABBs
		b3Transform transform = sim->transform;
		bool isFast = ( sim->flags & b3_isFast ) != 0;
		int shapeId = body->headShapeId;
		while ( shapeId != B3_NULL_INDEX )
		{
			b3Shape* shape = world->shapes.Get( shapeId );

			if ( isFast )
			{
				// For fast non-bullet bodies the AABB has already been updated in b3SolveContinuous
				// For fast bullet bodies the AABB will be updated at a later stage

				// Add to enlarged shapes regardless of AABB changes.
				// Bit-set to keep the move array sorted
				b3SetBit( enlargedSimBitSet, simIndex );
			}
			else
			{
				b3AABB aabb = b3ComputeShapeAABB( shape, transform );
				aabb.lowerBound -= speculativeMargin;
				aabb.upperBound += speculativeMargin;
				shape->aabb = aabb;

				B3_ASSERT( shape->enlargedAABB == false );

				if ( b3AABB_Contains( shape->fatAABB, aabb ) == false )
				{
					shape->fatAABB = { aabb.lowerBound - aabbMargin, aabb.upperBound + aabbMargin };
					shape->enlargedAABB = true;

					// Bit-set to keep the move array sorted
					b3SetBit( enlargedSimBitSet, simIndex );
				}
			}

			shapeId = shape->nextShapeId;
		}
	}

	b3TracyCZoneEnd( finalize_bodies );
}

/*
 typedef enum b3SolverStageType
{
	b3_stagePrepareJoints,
	b3_stagePrepareContacts,
	b3_stageIntegrateVelocities,
	b3_stageWarmStart,
	b3_stageSolve,
	b3_stageIntegratePositions,
	b3_stageRelax,
	b3_stageRestitution,
	b3_stageStoreImpulses
} b3SolverStageType;

typedef enum b3SolverBlockType
{
	b3_bodyBlock,
	b3_jointBlock,
	b3_contactBlock,
	b3_graphJointBlock,
	b3_graphContactBlock
} b3SolverBlockType;
*/

static void b3ExecuteBlock( b3SolverStage* stage, b3StepContext* context, b3SolverBlock* block, int workerIndex )
{
	b3SolverStageType stageType = stage->type;
	b3SolverBlockType blockType = block->blockType;
	int startIndex = block->startIndex;
	int endIndex = startIndex + block->count;

	switch ( stageType )
	{
		case b3_stagePrepareJoints:
			b3PrepareJointsTask( startIndex, endIndex, context );
			break;

		case b3_stagePrepareContacts1:
			B3_ASSERT( blockType == b3_contact1Block );
			b3PrepareContactsTask( startIndex, endIndex, context, 1 );
			break;

		case b3_stagePrepareContacts2:
			B3_ASSERT( blockType == b3_contact2Block );
			b3PrepareContactsTask( startIndex, endIndex, context, 2 );
			break;

		case b3_stagePrepareContacts3:
			B3_ASSERT( blockType == b3_contact3Block );
			b3PrepareContactsTask( startIndex, endIndex, context, 3 );
			break;

		case b3_stageIntegrateVelocities:
			b3IntegrateVelocitiesTask( startIndex, endIndex, context );
			break;

		case b3_stageWarmStart:
			if ( context->world->enableWarmStarting )
			{
				if ( blockType == b3_graphContact1Block )
				{
					b3WarmStartManifoldTask( startIndex, endIndex, context, stage->colorIndex, 1 );
				}
				else if ( blockType == b3_graphContact2Block )
				{
					b3WarmStartManifoldTask( startIndex, endIndex, context, stage->colorIndex, 2 );
				}
				else if ( blockType == b3_graphContact3Block )
				{
					b3WarmStartManifoldTask( startIndex, endIndex, context, stage->colorIndex, 3 );
				}
				else if ( blockType == b3_graphJointBlock )
				{
					b3WarmStartJointsTask( startIndex, endIndex, context, stage->colorIndex );
				}
			}
			break;

		case b3_stageSolve:
			if ( blockType == b3_graphContact1Block )
			{
				b3SolveManifoldTask( startIndex, endIndex, context, stage->colorIndex, 1, true );
			}
			else if ( blockType == b3_graphContact2Block )
			{
				b3SolveManifoldTask( startIndex, endIndex, context, stage->colorIndex, 2, true );
			}
			else if ( blockType == b3_graphContact3Block )
			{
				b3SolveManifoldTask( startIndex, endIndex, context, stage->colorIndex, 3, true );
			}
			else if ( blockType == b3_graphJointBlock )
			{
				b3SolveJointsTask( startIndex, endIndex, context, stage->colorIndex, true, workerIndex );
			}
			break;

		case b3_stageIntegratePositions:
			b3IntegratePositionsTask( startIndex, endIndex, context );
			break;

		case b3_stageRelax:
			if ( blockType == b3_graphContact1Block )
			{
				b3SolveManifoldTask( startIndex, endIndex, context, stage->colorIndex, 1, false );
			}
			else if ( blockType == b3_graphContact2Block )
			{
				b3SolveManifoldTask( startIndex, endIndex, context, stage->colorIndex, 2, false );
			}
			else if ( blockType == b3_graphContact3Block )
			{
				b3SolveManifoldTask( startIndex, endIndex, context, stage->colorIndex, 3, false );
			}
			else if ( blockType == b3_graphJointBlock )
			{
				b3SolveJointsTask( startIndex, endIndex, context, stage->colorIndex, false, workerIndex );
			}
			break;

		case b3_stageRestitution:
			if ( blockType == b3_graphContact1Block )
			{
				b3ApplyRestitutionTask( startIndex, endIndex, context, stage->colorIndex, 1 );
			}
			else if ( blockType == b3_graphContact2Block )
			{
				b3ApplyRestitutionTask( startIndex, endIndex, context, stage->colorIndex, 2 );
			}
			else if ( blockType == b3_graphContact3Block )
			{
				b3ApplyRestitutionTask( startIndex, endIndex, context, stage->colorIndex, 3 );
			}
			break;

		case b3_stageStoreImpulses1:
			B3_ASSERT( blockType == b3_contact1Block );
			b3StoreImpulsesTask( startIndex, endIndex, context, 1 );
			break;

		case b3_stageStoreImpulses2:
			B3_ASSERT( blockType == b3_contact2Block );
			b3StoreImpulsesTask( startIndex, endIndex, context, 2 );
			break;

		case b3_stageStoreImpulses3:
			B3_ASSERT( blockType == b3_contact3Block );
			b3StoreImpulsesTask( startIndex, endIndex, context, 3 );
			break;
	}
}

static inline int GetWorkerStartIndex( int workerIndex, int blockCount, int workerCount )
{
	if ( blockCount <= workerCount )
	{
		return workerIndex < blockCount ? workerIndex : B3_NULL_INDEX;
	}

	int blocksPerWorker = blockCount / workerCount;
	int remainder = blockCount - blocksPerWorker * workerCount;
	return blocksPerWorker * workerIndex + b3MinInt( remainder, workerIndex );
}

static void b3ExecuteStage( b3SolverStage* stage, b3StepContext* context, int previousSyncIndex, int syncIndex, int workerIndex )
{
	int completedCount = 0;
	b3SolverBlock* blocks = stage->blocks;
	int blockCount = stage->blockCount;

	int expectedSyncIndex = previousSyncIndex;

	int startIndex = GetWorkerStartIndex( workerIndex, blockCount, context->workerCount );
	if ( startIndex == B3_NULL_INDEX )
	{
		return;
	}

	B3_ASSERT( 0 <= startIndex && startIndex < blockCount );

	int blockIndex = startIndex;

	while ( b3AtomicCompareExchangeInt( &blocks[blockIndex].syncIndex, expectedSyncIndex, syncIndex ) == true )
	{
		// todo what is this assert doing?
		B3_ASSERT( stage->type != b3_stagePrepareContacts1 || syncIndex < 2 );

		B3_ASSERT( completedCount < blockCount );

		b3ExecuteBlock( stage, context, blocks + blockIndex, workerIndex );

		completedCount += 1;
		blockIndex += 1;
		if ( blockIndex >= blockCount )
		{
			// Keep looking for work
			blockIndex = 0;
		}

		expectedSyncIndex = previousSyncIndex;
	}

	// Search backwards for blocks
	blockIndex = startIndex - 1;
	while ( true )
	{
		if ( blockIndex < 0 )
		{
			blockIndex = blockCount - 1;
		}

		expectedSyncIndex = previousSyncIndex;

		if ( b3AtomicCompareExchangeInt( &blocks[blockIndex].syncIndex, expectedSyncIndex, syncIndex ) == false )
		{
			break;
		}

		b3ExecuteBlock( stage, context, blocks + blockIndex, workerIndex );
		completedCount += 1;
		blockIndex -= 1;
	}

	(void)b3AtomicFetchAddInt( &stage->completionCount, completedCount );
}

static void b3ExecuteMainStage( b3SolverStage* stage, b3StepContext* context, uint32_t syncBits )
{
	int blockCount = stage->blockCount;
	if ( blockCount == 0 )
	{
		return;
	}

	const int workerIndex = 0;

	if ( blockCount == 1 )
	{
		b3ExecuteBlock( stage, context, stage->blocks, workerIndex );
	}
	else
	{
		b3AtomicStoreU32( &context->atomicSyncBits, syncBits );

		int syncIndex = ( syncBits >> 16 ) & 0xFFFF;
		B3_ASSERT( syncIndex > 0 );
		int previousSyncIndex = syncIndex - 1;

		b3ExecuteStage( stage, context, previousSyncIndex, syncIndex, workerIndex );

		// todo consider using the cycle counter as well
		while ( b3AtomicLoadInt( &stage->completionCount ) != blockCount )
		{
			b3Pause();
		}

		b3AtomicStoreInt( &stage->completionCount, 0 );
	}
}

// This should not use the thread index because thread 0 can be called twice by enkiTS.
static void b3SolverTask( int startIndex, int endIndex, uint32_t threadIndexDontUse, void* taskContext )
{
	B3_UNUSED( startIndex );
	B3_UNUSED( endIndex );
	B3_UNUSED( threadIndexDontUse );

	b3WorkerContext* workerContext = (b3WorkerContext*)taskContext;
	int workerIndex = workerContext->workerIndex;
	b3StepContext* context = workerContext->context;
	int activeColorCount = context->activeColorCount;
	b3SolverStage* stages = context->stages;
	b3Profile* profile = &context->world->profile;

	if ( workerIndex == 0 )
	{
		// Main thread synchronizes the workers and does work itself.
		//
		// Stages are re-used by loops so that I don't need more stages for large iteration counts.
		// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
		// The stage index and sync indices are combined in to sync bits for atomic synchronization.
		// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
		// setup makes this easy to do.

		/*
		b3_stagePrepareJoints,
		b3_stagePrepareContacts,
		b3_stageIntegrateVelocities,
		b3_stageWarmStart,
		b3_stageSolve,
		b3_stageIntegratePositions,
		b3_stageRelax,
		b3_stageRestitution,
		b3_stageStoreImpulses
		*/

		uint64_t ticks = b3GetTicks();

		int bodySyncIndex = 1;
		int stageIndex = 0;

		// This stage loops over all awake joints
		uint32_t jointSyncIndex = 1;
		uint32_t syncBits = ( jointSyncIndex << 16 ) | stageIndex;
		B3_ASSERT( stages[stageIndex].type == b3_stagePrepareJoints );
		b3ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;
		jointSyncIndex += 1;

		// This stage loops over all contact1 constraints
		uint32_t contact1SyncIndex = 1;
		syncBits = ( contact1SyncIndex << 16 ) | stageIndex;
		B3_ASSERT( stages[stageIndex].type == b3_stagePrepareContacts1 );
		b3ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;
		contact1SyncIndex += 1;

		uint32_t contact2SyncIndex = 1;
		syncBits = ( contact2SyncIndex << 16 ) | stageIndex;
		B3_ASSERT( stages[stageIndex].type == b3_stagePrepareContacts2 );
		b3ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;
		contact2SyncIndex += 1;

		uint32_t contact3SyncIndex = 1;
		syncBits = ( contact3SyncIndex << 16 ) | stageIndex;
		B3_ASSERT( stages[stageIndex].type == b3_stagePrepareContacts3 );
		b3ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;
		contact3SyncIndex += 1;

		int graphSyncIndex = 1;

		// Single-threaded overflow work. These constraints don't fit in the graph coloring.
		// todo prepare in parallel
		b3PrepareOverflowJoints( context );
		b3PrepareOverflowContacts( context );

		profile->prepareConstraints += b3GetMillisecondsAndReset( &ticks );

		int subStepCount = context->subStepCount;
		for ( int i = 0; i < subStepCount; ++i )
		{
			// stage index restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			int iterationStageIndex = stageIndex;

			// integrate velocities
			syncBits = ( bodySyncIndex << 16 ) | iterationStageIndex;
			B3_ASSERT( stages[iterationStageIndex].type == b3_stageIntegrateVelocities );
			b3ExecuteMainStage( stages + iterationStageIndex, context, syncBits );
			iterationStageIndex += 1;
			bodySyncIndex += 1;

			profile->integrateVelocities += b3GetMillisecondsAndReset( &ticks );

			// warm start constraints
			b3WarmStartOverflowJoints( context );
			b3WarmStartOverflowContacts( context );

			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterationStageIndex;
				B3_ASSERT( stages[iterationStageIndex].type == b3_stageWarmStart );
				b3ExecuteMainStage( stages + iterationStageIndex, context, syncBits );
				iterationStageIndex += 1;
			}
			graphSyncIndex += 1;

			profile->warmStart += b3GetMillisecondsAndReset( &ticks );

			// solve constraints
			bool useBias = true;
			b3SolveOverflowJoints( context, useBias );
			b3SolveOverflowContacts( context, useBias );

			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterationStageIndex;
				B3_ASSERT( stages[iterationStageIndex].type == b3_stageSolve );
				b3ExecuteMainStage( stages + iterationStageIndex, context, syncBits );
				iterationStageIndex += 1;
			}
			graphSyncIndex += 1;

			profile->solveImpulses += b3GetMillisecondsAndReset( &ticks );

			// integrate positions
			B3_ASSERT( stages[iterationStageIndex].type == b3_stageIntegratePositions );
			syncBits = ( bodySyncIndex << 16 ) | iterationStageIndex;
			b3ExecuteMainStage( stages + iterationStageIndex, context, syncBits );
			iterationStageIndex += 1;
			bodySyncIndex += 1;

			profile->integratePositions += b3GetMillisecondsAndReset( &ticks );

			// relax constraints
			useBias = false;
			b3SolveOverflowJoints( context, useBias );
			b3SolveOverflowContacts( context, useBias );

			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterationStageIndex;
				B3_ASSERT( stages[iterationStageIndex].type == b3_stageRelax );
				b3ExecuteMainStage( stages + iterationStageIndex, context, syncBits );
				iterationStageIndex += 1;
			}
			graphSyncIndex += 1;

			profile->relaxImpulses += b3GetMillisecondsAndReset( &ticks );
		}

		// advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + activeColorCount + 1 + activeColorCount;

		// Restitution
		{
			b3ApplyOverflowRestitution( context );

			int iterStageIndex = stageIndex;
			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
				B3_ASSERT( stages[iterStageIndex].type == b3_stageRestitution );
				b3ExecuteMainStage( stages + iterStageIndex, context, syncBits );
				iterStageIndex += 1;
			}
			// graphSyncIndex += 1;
			stageIndex += activeColorCount;
		}

		profile->applyRestitution += b3GetMillisecondsAndReset( &ticks );

		b3StoreOverflowImpulses( context );

		syncBits = ( contact1SyncIndex << 16 ) | stageIndex;
		B3_ASSERT( stages[stageIndex].type == b3_stageStoreImpulses1 );
		b3ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;

		syncBits = ( contact2SyncIndex << 16 ) | stageIndex;
		B3_ASSERT( stages[stageIndex].type == b3_stageStoreImpulses2 );
		b3ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;

		syncBits = ( contact3SyncIndex << 16 ) | stageIndex;
		B3_ASSERT( stages[stageIndex].type == b3_stageStoreImpulses3 );
		b3ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;

		profile->storeImpulses += b3GetMillisecondsAndReset( &ticks );

		// Signal workers to finish
		b3AtomicStoreU32( &context->atomicSyncBits, UINT_MAX );

		B3_ASSERT( stageIndex == context->stageCount );
		return;
	}

	// Worker spins and waits for work
	uint32_t lastSyncBits = 0;
	// uint64_t maxSpinTime = 10;
	while ( true )
	{
		// Spin until main thread bumps changes the sync bits. This can waste significant time overall, but it is necessary for
		// parallel simulation with graph coloring.
		uint32_t syncBits;
		int spinCount = 0;
		while ( ( syncBits = b3AtomicLoadU32( &context->atomicSyncBits ) ) == lastSyncBits )
		{
			if ( spinCount > 5 )
			{
				b3Yield();
				spinCount = 0;
			}
			else
			{
				// Using the cycle counter helps to account for variation in mm_pause timing across different
				// CPUs. However, this is X64 only.
				// uint64_t prev = __rdtsc();
				// do
				//{
				//	b3Pause();
				//}
				// while ((__rdtsc() - prev) < maxSpinTime);
				// maxSpinTime += 10;
				b3Pause();
				b3Pause();
				spinCount += 1;
			}
		}

		if ( syncBits == UINT_MAX )
		{
			// sentinel hit
			break;
		}

		int stageIndex = syncBits & 0xFFFF;
		B3_ASSERT( stageIndex < context->stageCount );

		int syncIndex = ( syncBits >> 16 ) & 0xFFFF;
		B3_ASSERT( syncIndex > 0 );

		int previousSyncIndex = syncIndex - 1;

		b3SolverStage* stage = stages + stageIndex;
		b3ExecuteStage( stage, context, previousSyncIndex, syncIndex, workerIndex );

		lastSyncBits = syncBits;
	}
}

static void b3BulletBodyTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	B3_UNUSED( threadIndex );

	b3TracyCZoneNC( bullet_body_task, "Bullet Body Task", b3_colorLightSkyBlue, true );

	b3StepContext* stepContext = (b3StepContext*)context;
	b3TaskContext* taskContext = stepContext->world->taskContexts.Get( threadIndex );

	B3_ASSERT( startIndex <= endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		int simIndex = stepContext->bulletBodies[i];
		b3SolveContinuous( stepContext->world, simIndex, taskContext );
	}

	b3TracyCZoneEnd( bullet_body_task );
}

#if B3_SIMD_WIDTH == 8
#define B3_SIMD_SHIFT 3
#else
#define B3_SIMD_SHIFT 2
#endif

static int b3SimdCount( int count )
{
	return count > 0 ? ( ( count - 1 ) >> B3_SIMD_SHIFT ) + 1 : 0;
}

// Solve with graph coloring
void b3Solve( b3World* world, b3StepContext* stepContext )
{
	world->stepIndex += 1;

	// Are there any awake bodies? This scenario should not be important for profiling.
	b3SolverSet* awakeSet = world->solverSets.Get( b3_awakeSet );
	int awakeBodyCount = awakeSet->bodySims.count;
	B3_ASSERT( awakeBodyCount > 0 );

	{
		// Prepare buffers for continuous collision (fast bodies)
		b3AtomicStoreInt( &stepContext->bulletBodyCount, 0 );
		stepContext->bulletBodies = (int*)b3AllocateArenaItem( &world->arena, awakeBodyCount * sizeof( int ), "bullet bodies" );

		// Solve constraints using graph coloring
		b3TracyCZoneNC( prepare_stages, "Prepare Stages", b3_colorDarkOrange, true );
		uint64_t prepareTicks = b3GetTicks();

		b3ConstraintGraph* graph = &world->constraintGraph;
		b3GraphColor* colors = graph->colors;

		stepContext->sims = awakeSet->bodySims.data;
		stepContext->states = awakeSet->bodyStates.data;

		// count contacts, joints, and colors
		int awakeJointCount = 0;
		int activeColorCount = 0;
		for ( int i = 0; i < B3_GRAPH_COLOR_COUNT - 1; ++i )
		{
			int perColorContactCount = colors[i].contactSims.count;
			int perColorJointCount = colors[i].jointSims.count;
			int occupancyCount = perColorContactCount + perColorJointCount;
			activeColorCount += occupancyCount > 0 ? 1 : 0;
			awakeJointCount += perColorJointCount;
		}

		// Deal with void**
		{
			world->bodyMoveEvents.Resize( awakeBodyCount );
		}

		// Each worker receives at most M blocks of work. The workers may receive less than there is not sufficient work.
		// Each block of work has a minimum number of elements (block size). This in turn may limit number of blocks.
		// If there are many elements then the block size is increased so there are still at most M blocks of work per worker.
		// M is a tunable number that has two goals:
		// 1. keep M small to reduce overhead
		// 2. keep M large enough for other workers to be able to steal work
		// The block size is a power of two to make math efficient.

		int workerCount = world->workerCount;
		const int blocksPerWorker = 4;
		const int maxBlockCount = blocksPerWorker * workerCount;

		// Configure blocks for tasks that parallel-for bodies
		int bodyBlockSize = 1 << 5;
		int bodyBlockCount;
		if ( awakeBodyCount > bodyBlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			bodyBlockSize = awakeBodyCount / maxBlockCount;
			bodyBlockCount = maxBlockCount;
		}
		else
		{
			bodyBlockCount = ( ( awakeBodyCount - 1 ) >> 5 ) + 1;
		}

		// Compact monotonic array of active graph color indices
		int activeColorIndices[B3_GRAPH_COLOR_COUNT];

		// Configure blocks for tasks parallel-for each active graph color
		// Note: these arrays are filled according to active colors. Empty colors are skipped.

		// The number of contacts with 1/2/3 manifolds
		int contactTypeCounts[B3_GRAPH_COLOR_COUNT][3] = {};

		// Contact type SIMD bundle counts.
		// For example, a 2-manifold count of 3 implies 6 SIMD contact constraints.
		int contactTypeCountsW[B3_GRAPH_COLOR_COUNT][3] = {};
		int totalContactTypeW[3] = {};

		// The manifold work block size for each active color
		int colorManifoldBlockSizes[B3_GRAPH_COLOR_COUNT][3] = {};

		// The number manifold work blocks for each active color
		int colorManifoldBlockCounts[B3_GRAPH_COLOR_COUNT][3] = {};

		// The number of joint constraints in each active color
		int colorJointCounts[B3_GRAPH_COLOR_COUNT];

		// The joint work block size for each active color
		int colorJointBlockSizes[B3_GRAPH_COLOR_COUNT];

		// The number joint work blocks for each active color
		int colorJointBlockCounts[B3_GRAPH_COLOR_COUNT];

		int graphBlockCount = 0;

		// A SIMD contact constraint solves multiple manifolds simultaneously. Such as 4 or 8 manifolds.
		// A b3ContactSim may have 1 to 3 manifolds. These manifolds cannot be in the same SIMD constraint or
		// they will race. However graph coloring is by b3ContactSim, not by manifold.
		// If a b3ContactSim has 3 manifolds of a single body versus ground, then I need 3 SIMD constraints that will
		// be solved sequentially. So we can think of a contact with 3 manifolds as having a different kind of constraint
		// than a contact with 1 manifold, although I do make use of some shared code for them.

		// For each graph color, I need to count the number of contact sims with 1 manifolds, 2 manifolds, and 3
		// manifolds. Then SIMD constraints can be built for each manifold bucket.
		// In order to prepare contact constraints in parallel, I need to know which constraint each manifold
		// belongs to. So I need the total number of manifolds in a graph color and I need a map from manifold
		// to SIMD contact constraint and lane. This mapping should be monotonic so that traversing b3ContactSims
		// in order of a graph color maps to SIMD contact constraints in that order.

		// Another issue is the parallel-for to prepare SIMD contact constraints must not race. So each prepare task must
		// fully populate each SIMD contact constraint. Update: I don't think this will race, however it is better to organize it
		// this way so it aligns with storing impulses.

		// The contact blocks must be on manifolds ranges, not contact sims. So the tasks may read from identical contact
		// sims but only write to manifolds.

		// I'm arranging this now to treat 1-manifold, 2-manifold, and 3-manifold contacts as separate constraint types, similar
		// to different joint types. However, contact constraint memory is temporary so lots of bookkeeping. In the future
		// multi-manifold contacts may use a different solver, such as a different normal for each contact point and friction per
		// point.

		int activeCount = 0;
		for ( int colorIndex = 0; colorIndex < B3_GRAPH_COLOR_COUNT - 1; ++colorIndex )
		{
			b3GraphColor* color = colors + colorIndex;
			int jointCount = color->jointSims.count;

			int contactSimCount = color->contactSims.count;
			const b3ContactSim* colorContactSims = color->contactSims.data;
			for ( int i = 0; i < contactSimCount; ++i )
			{
				int count = colorContactSims[i].manifoldCount;
				contactTypeCounts[activeCount][count - 1] += 1;
			}

			int constraintCount = contactTypeCounts[activeCount][0] + contactTypeCounts[activeCount][1] +
								  contactTypeCounts[activeCount][2] + jointCount;

			if ( constraintCount > 0 )
			{
				activeColorIndices[activeCount] = colorIndex;

				// 4/8-way SIMD
				contactTypeCountsW[activeCount][0] = b3SimdCount( contactTypeCounts[activeCount][0] );
				contactTypeCountsW[activeCount][1] = b3SimdCount( contactTypeCounts[activeCount][1] );
				contactTypeCountsW[activeCount][2] = b3SimdCount( contactTypeCounts[activeCount][2] );

				totalContactTypeW[0] += contactTypeCountsW[activeCount][0];
				totalContactTypeW[1] += contactTypeCountsW[activeCount][1];
				totalContactTypeW[2] += contactTypeCountsW[activeCount][2];

				// determine the number of manifold work blocks for this color
				for ( int i = 0; i < 3; ++i )
				{
					int countW = contactTypeCountsW[activeCount][i];
					if ( countW > blocksPerWorker * maxBlockCount )
					{
						// too many blocks, make the blocks bigger
						colorManifoldBlockSizes[activeCount][i] = countW / maxBlockCount;
						colorManifoldBlockCounts[activeCount][i] = maxBlockCount;
					}
					else if ( contactTypeCountsW[activeCount][i] > 0 )
					{
						// dividing by blocksPerWorker (4)
						colorManifoldBlockSizes[activeCount][i] = blocksPerWorker;
						colorManifoldBlockCounts[activeCount][i] = ( ( countW - 1 ) >> 2 ) + 1;
					}
					else
					{
						// no contacts in this color
						colorManifoldBlockSizes[activeCount][i] = 0;
						colorManifoldBlockCounts[activeCount][i] = 0;
					}
				}

				colorJointCounts[activeCount] = jointCount;

				// determine number of joint work blocks for this color
				if ( jointCount > blocksPerWorker * maxBlockCount )
				{
					// too many joint blocks, make the blocks bigger
					colorJointBlockSizes[activeCount] = jointCount / maxBlockCount;
					colorJointBlockCounts[activeCount] = maxBlockCount;
				}
				else if ( jointCount > 0 )
				{
					// dividing by blocksPerWorker (4)
					colorJointBlockSizes[activeCount] = blocksPerWorker;
					colorJointBlockCounts[activeCount] = ( ( jointCount - 1 ) >> 2 ) + 1;
				}
				else
				{
					colorJointBlockSizes[activeCount] = 0;
					colorJointBlockCounts[activeCount] = 0;
				}

				graphBlockCount += colorManifoldBlockCounts[activeCount][0] + colorManifoldBlockCounts[activeCount][1] +
								   colorManifoldBlockCounts[activeCount][2] + colorJointBlockCounts[activeCount];

				activeCount += 1;
			}
		}
		activeColorCount = activeCount;

		// Manifold pointers for easy parallel-for traversal. Some may be nullptr due to SIMD remainders.
		b3ManifoldLookup* contact1Lookup = (b3ManifoldLookup*)b3AllocateArenaItem(
			&world->arena, B3_SIMD_WIDTH * totalContactTypeW[0] * sizeof( b3ManifoldLookup ), "contact1 lookup" );

		b3ManifoldLookup* contact2Lookup = (b3ManifoldLookup*)b3AllocateArenaItem(
			&world->arena, B3_SIMD_WIDTH * totalContactTypeW[1] * sizeof( b3ManifoldLookup ), "contact2 lookup" );

		b3ManifoldLookup* contact3Lookup = (b3ManifoldLookup*)b3AllocateArenaItem(
			&world->arena, B3_SIMD_WIDTH * totalContactTypeW[2] * sizeof( b3ManifoldLookup ), "contact3 lookup" );

		// Gather joint pointers for easy parallel-for traversal.
		b3JointSim** jointSims =
			(b3JointSim**)b3AllocateArenaItem( &world->arena, awakeJointCount * sizeof( b3JointSim* ), "joint pointers" );

		// I'm putting these all together for 1/2/3-manifold contacts but the complexity might not be worth it.
		int simdConstraintByteCount = b3GetContactConstraintSIMDByteCount();
		b3ContactConstraintSIMD* contact1Constraints = (b3ContactConstraintSIMD*)b3AllocateArenaItem(
			&world->arena, 1 * totalContactTypeW[0] * simdConstraintByteCount, "contact1 constraint" );

		b3ContactConstraintSIMD* contact2Constraints = (b3ContactConstraintSIMD*)b3AllocateArenaItem(
			&world->arena, 2 * totalContactTypeW[1] * simdConstraintByteCount, "contact2 constraint" );

		b3ContactConstraintSIMD* contact3Constraints = (b3ContactConstraintSIMD*)b3AllocateArenaItem(
			&world->arena, 3 * totalContactTypeW[2] * simdConstraintByteCount, "contact3 constraint" );

		int overflowContactSimCount = colors[B3_OVERFLOW_INDEX].contactSims.count;
		int overflowManifoldCount = 0;
		const b3ContactSim* overflowContactSims = colors[B3_OVERFLOW_INDEX].contactSims.data;
		for ( int i = 0; i < overflowContactSimCount; ++i )
		{
			// todo hot path
			overflowManifoldCount += overflowContactSims[i].manifoldCount;
		}

		b3ContactConstraint* overflowManifoldConstraints = (b3ContactConstraint*)b3AllocateArenaItem(
			&world->arena, overflowManifoldCount * sizeof( b3ContactConstraint ), "overflow contact constraint" );

		graph->colors[B3_OVERFLOW_INDEX].overflowConstraints = overflowManifoldConstraints;
		graph->colors[B3_OVERFLOW_INDEX].overflowManifoldCount = overflowManifoldCount;

		// Distribute transient constraints to each graph color and build flat arrays of manifold and joint pointers
		int contactTypeBaseW[3] = {};
		b3ManifoldLookup* contactLookups[3] = { contact1Lookup, contact2Lookup, contact3Lookup };
		int jointBase = 0;
		for ( int activeIndex = 0; activeIndex < activeColorCount; ++activeIndex )
		{
			b3GraphColor* color = colors + activeColorIndices[activeIndex];

			int contactSimCount = color->contactSims.count;

			if ( contactSimCount == 0 )
			{
				color->simdConstraints[0] = nullptr;
				color->simdConstraints[1] = nullptr;
				color->simdConstraints[2] = nullptr;
			}
			else
			{
				// Populate color contact constraints
				int countW1 = contactTypeCountsW[activeIndex][0];
				int countW2 = contactTypeCountsW[activeIndex][1];
				int countW3 = contactTypeCountsW[activeIndex][2];

				color->simdConstraints[0] = nullptr;
				color->simdConstraints[1] = nullptr;
				color->simdConstraints[2] = nullptr;

				if ( countW1 > 0 )
				{
					int byteOffset = 1 * contactTypeBaseW[0] * simdConstraintByteCount;
					color->simdConstraints[0] = (b3ContactConstraintSIMD*)( (uint8_t*)contact1Constraints + byteOffset );
				}

				if ( countW2 > 0 )
				{
					int byteOffset = 2 * contactTypeBaseW[1] * simdConstraintByteCount;
					color->simdConstraints[1] = (b3ContactConstraintSIMD*)( (uint8_t*)contact2Constraints + byteOffset );
				}

				if ( countW3 > 0 )
				{
					int byteOffset = 3 * contactTypeBaseW[2] * simdConstraintByteCount;
					color->simdConstraints[2] = (b3ContactConstraintSIMD*)( (uint8_t*)contact3Constraints + byteOffset );
				}

				// Build manifold lookup for each SIMD lane based on contact type.
				// Offsets for assigning contact lookups, one offset per 1/2/3 manifolds
				int offsets[3] = {};

				for ( int k = 0; k < contactSimCount; ++k )
				{
					b3ContactSim* contactSim = color->contactSims.data + k;
					int contactType = contactSim->manifoldCount - 1;
					int index = B3_SIMD_WIDTH * contactTypeBaseW[contactType] + offsets[contactType];
					contactLookups[contactType][index] = { contactSim };
					offsets[contactType] += 1;
				}

				// remainder manifold1
				for ( int k = offsets[0]; k < B3_SIMD_WIDTH * countW1; ++k )
				{
					contactLookups[0][B3_SIMD_WIDTH * contactTypeBaseW[0] + k] = { .contactSim = nullptr };
				}

				// remainder manifold2
				for ( int k = offsets[1]; k < B3_SIMD_WIDTH * countW2; ++k )
				{
					contactLookups[1][B3_SIMD_WIDTH * contactTypeBaseW[1] + k] = { .contactSim = nullptr };
				}

				// remainder manifold3
				for ( int k = offsets[2]; k < B3_SIMD_WIDTH * countW3; ++k )
				{
					contactLookups[2][B3_SIMD_WIDTH * contactTypeBaseW[2] + k] = { .contactSim = nullptr };
				}

				contactTypeBaseW[0] += countW1;
				contactTypeBaseW[1] += countW2;
				contactTypeBaseW[2] += countW3;
			}

			int colorJointCount = color->jointSims.count;
			for ( int k = 0; k < colorJointCount; ++k )
			{
				jointSims[jointBase + k] = color->jointSims.data + k;
			}
			jointBase += colorJointCount;
		}

		B3_ASSERT( contactTypeBaseW[0] == totalContactTypeW[0] );
		B3_ASSERT( contactTypeBaseW[1] == totalContactTypeW[1] );
		B3_ASSERT( contactTypeBaseW[2] == totalContactTypeW[2] );
		B3_ASSERT( jointBase == awakeJointCount );

		// Define work blocks for preparing contacts and storing contact impulses
		int contact1BlockSize = blocksPerWorker;
		int contact1BlockCount = totalContactTypeW[0] > 0 ? ( ( totalContactTypeW[0] - 1 ) >> 2 ) + 1 : 0;
		if ( totalContactTypeW[0] > contact1BlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			contact1BlockSize = totalContactTypeW[0] / maxBlockCount;
			contact1BlockCount = maxBlockCount;
		}

		int contact2BlockSize = blocksPerWorker;
		int contact2BlockCount = totalContactTypeW[1] > 0 ? ( ( totalContactTypeW[1] - 1 ) >> 2 ) + 1 : 0;
		if ( totalContactTypeW[1] > contact2BlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			contact2BlockSize = totalContactTypeW[1] / maxBlockCount;
			contact2BlockCount = maxBlockCount;
		}

		int contact3BlockSize = blocksPerWorker;
		int contact3BlockCount = totalContactTypeW[2] > 0 ? ( ( totalContactTypeW[2] - 1 ) >> 2 ) + 1 : 0;
		if ( totalContactTypeW[2] > contact3BlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			contact3BlockSize = totalContactTypeW[2] / maxBlockCount;
			contact3BlockCount = maxBlockCount;
		}

		// Define work blocks for preparing joints
		int jointBlockSize = blocksPerWorker;
		int jointBlockCount = awakeJointCount > 0 ? ( ( awakeJointCount - 1 ) >> 2 ) + 1 : 0;
		if ( awakeJointCount > jointBlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			jointBlockSize = awakeJointCount / maxBlockCount;
			jointBlockCount = maxBlockCount;
		}

		int stageCount = 0;

		// b3_stagePrepareJoints
		stageCount += 1;
		// b3_stagePrepareContacts (contact1/2/3)
		stageCount += 3;
		// b3_stageIntegrateVelocities
		stageCount += 1;
		// b3_stageWarmStart
		stageCount += activeColorCount;
		// b3_stageSolve
		stageCount += activeColorCount;
		// b3_stageIntegratePositions
		stageCount += 1;
		// b3_stageRelax
		stageCount += activeColorCount;
		// b3_stageRestitution
		stageCount += activeColorCount;
		// b3_stageStoreImpulses (contact1/2/3)
		stageCount += 3;

		b3SolverStage* stages =
			(b3SolverStage*)b3AllocateArenaItem( &world->arena, stageCount * sizeof( b3SolverStage ), "stages" );
		b3SolverBlock* bodyBlocks =
			(b3SolverBlock*)b3AllocateArenaItem( &world->arena, bodyBlockCount * sizeof( b3SolverBlock ), "body blocks" );
		b3SolverBlock* contact1Blocks =
			(b3SolverBlock*)b3AllocateArenaItem( &world->arena, contact1BlockCount * sizeof( b3SolverBlock ), "contact1 blocks" );
		b3SolverBlock* contact2Blocks =
			(b3SolverBlock*)b3AllocateArenaItem( &world->arena, contact2BlockCount * sizeof( b3SolverBlock ), "contact2 blocks" );
		b3SolverBlock* contact3Blocks =
			(b3SolverBlock*)b3AllocateArenaItem( &world->arena, contact3BlockCount * sizeof( b3SolverBlock ), "contact3 blocks" );
		b3SolverBlock* jointBlocks =
			(b3SolverBlock*)b3AllocateArenaItem( &world->arena, jointBlockCount * sizeof( b3SolverBlock ), "joint blocks" );
		b3SolverBlock* graphBlocks =
			(b3SolverBlock*)b3AllocateArenaItem( &world->arena, graphBlockCount * sizeof( b3SolverBlock ), "graph blocks" );

		// Split an awake island. This modifies:
		// - stack allocator
		// - world island array and solver set
		// - island indices on bodies, contacts, and joints
		// I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
		// Note: cannot split islands in parallel with FinalizeBodies
		void* splitIslandTask = nullptr;
		if ( world->splitIslandId != B3_NULL_INDEX )
		{
			splitIslandTask = world->enqueueTaskFcn( &b3SplitIslandTask, 1, 1, world, world->userTaskContext );
			world->taskCount += 1;
			world->activeTaskCount += splitIslandTask == nullptr ? 0 : 1;
		}

		// Prepare body work blocks
		for ( int i = 0; i < bodyBlockCount; ++i )
		{
			b3SolverBlock* block = bodyBlocks + i;
			block->startIndex = i * bodyBlockSize;
			block->count = (int16_t)bodyBlockSize;
			block->blockType = b3_bodyBlock;
			b3AtomicStoreInt( &block->syncIndex, 0 );
		}
		bodyBlocks[bodyBlockCount - 1].count = (int16_t)( awakeBodyCount - ( bodyBlockCount - 1 ) * bodyBlockSize );

		// Prepare joint work blocks
		for ( int i = 0; i < jointBlockCount; ++i )
		{
			b3SolverBlock* block = jointBlocks + i;
			block->startIndex = i * jointBlockSize;
			block->count = (int16_t)jointBlockSize;
			block->blockType = b3_jointBlock;
			b3AtomicStoreInt( &block->syncIndex, 0 );
		}

		if ( jointBlockCount > 0 )
		{
			// Last block may be partially full
			jointBlocks[jointBlockCount - 1].count = (int16_t)( awakeJointCount - ( jointBlockCount - 1 ) * jointBlockSize );
		}

		// Prepare contact1 work blocks
		for ( int i = 0; i < contact1BlockCount; ++i )
		{
			b3SolverBlock* block = contact1Blocks + i;
			block->startIndex = i * contact1BlockSize;
			block->count = (int16_t)contact1BlockSize;
			block->blockType = b3_contact1Block;
			b3AtomicStoreInt( &block->syncIndex, 0 );
		}

		if ( contact1BlockCount > 0 )
		{
			// Last block may be partially full
			contact1Blocks[contact1BlockCount - 1].count =
				int16_t( totalContactTypeW[0] - ( contact1BlockCount - 1 ) * contact1BlockSize );
		}

		// Prepare contact2 work blocks
		for ( int i = 0; i < contact2BlockCount; ++i )
		{
			b3SolverBlock* block = contact2Blocks + i;
			block->startIndex = i * contact2BlockSize;
			block->count = (int16_t)contact2BlockSize;
			block->blockType = b3_contact2Block;
			b3AtomicStoreInt( &block->syncIndex, 0 );
		}

		if ( contact2BlockCount > 0 )
		{
			// Last block may be partially full
			contact2Blocks[contact2BlockCount - 1].count =
				int16_t( totalContactTypeW[1] - ( contact2BlockCount - 1 ) * contact2BlockSize );
		}

		// Prepare contact3 work blocks
		for ( int i = 0; i < contact3BlockCount; ++i )
		{
			b3SolverBlock* block = contact3Blocks + i;
			block->startIndex = i * contact3BlockSize;
			block->count = (int16_t)contact3BlockSize;
			block->blockType = b3_contact3Block;
			b3AtomicStoreInt( &block->syncIndex, 0 );
		}

		if ( contact3BlockCount > 0 )
		{
			// Last block may be partially full
			contact3Blocks[contact3BlockCount - 1].count =
				int16_t( totalContactTypeW[2] - ( contact3BlockCount - 1 ) * contact3BlockSize );
		}

		// Prepare graph work blocks
		b3SolverBlock* graphColorBlocks[B3_GRAPH_COLOR_COUNT];
		b3SolverBlock* baseGraphBlock = graphBlocks;

		for ( int activeIndex = 0; activeIndex < activeColorCount; ++activeIndex )
		{
			graphColorBlocks[activeIndex] = baseGraphBlock;

			int colorJointBlockCount = colorJointBlockCounts[activeIndex];
			int colorJointBlockSize = colorJointBlockSizes[activeIndex];
			for ( int j = 0; j < colorJointBlockCount; ++j )
			{
				b3SolverBlock* block = baseGraphBlock + j;
				block->startIndex = j * colorJointBlockSize;
				block->count = (int16_t)colorJointBlockSize;
				block->blockType = b3_graphJointBlock;
				b3AtomicStoreInt( &block->syncIndex, 0 );
			}

			if ( colorJointBlockCount > 0 )
			{
				baseGraphBlock[colorJointBlockCount - 1].count =
					(int16_t)( colorJointCounts[activeIndex] - ( colorJointBlockCount - 1 ) * colorJointBlockSize );
				baseGraphBlock += colorJointBlockCount;
			}

			// Iterate for 1/2/3 manifold contacts
			b3SolverBlockType contactBlockTypes[3] = {
				b3_graphContact1Block,
				b3_graphContact2Block,
				b3_graphContact3Block,
			};

			for ( int k = 0; k < 3; ++k )
			{
				int colorManifoldBlockCount = colorManifoldBlockCounts[activeIndex][k];
				int colorManifoldBlockSize = colorManifoldBlockSizes[activeIndex][k];
				for ( int j = 0; j < colorManifoldBlockCount; ++j )
				{
					b3SolverBlock* block = baseGraphBlock + j;
					block->startIndex = j * colorManifoldBlockSize;
					block->count = (int16_t)colorManifoldBlockSize;
					block->blockType = contactBlockTypes[k];
					b3AtomicStoreInt( &block->syncIndex, 0 );
				}

				if ( colorManifoldBlockCount > 0 )
				{
					int countW = contactTypeCountsW[activeIndex][k];
					baseGraphBlock[colorManifoldBlockCount - 1].count =
						int16_t( countW - ( colorManifoldBlockCount - 1 ) * colorManifoldBlockSize );
					baseGraphBlock += colorManifoldBlockCount;
				}
			}
		}

		B3_ASSERT( baseGraphBlock - graphBlocks == graphBlockCount );

		// Fill stages
		b3SolverStage* stage = stages;

		// Prepare joints
		stage->type = b3_stagePrepareJoints;
		stage->blocks = jointBlockCount > 0 ? jointBlocks : nullptr;
		stage->blockCount = jointBlockCount;
		stage->colorIndex = -1;
		b3AtomicStoreInt( &stage->completionCount, 0 );
		stage += 1;

		// Prepare contact1
		stage->type = b3_stagePrepareContacts1;
		stage->blocks = contact1BlockCount > 0 ? contact1Blocks : nullptr;
		stage->blockCount = contact1BlockCount;
		stage->colorIndex = -1;
		b3AtomicStoreInt( &stage->completionCount, 0 );
		stage += 1;

		// Prepare contact2
		stage->type = b3_stagePrepareContacts2;
		stage->blocks = contact2BlockCount > 0 ? contact2Blocks : nullptr;
		stage->blockCount = contact2BlockCount;
		stage->colorIndex = -1;
		b3AtomicStoreInt( &stage->completionCount, 0 );
		stage += 1;

		// Prepare contact3
		stage->type = b3_stagePrepareContacts3;
		stage->blocks = contact3BlockCount > 0 ? contact3Blocks : nullptr;
		stage->blockCount = contact3BlockCount;
		stage->colorIndex = -1;
		b3AtomicStoreInt( &stage->completionCount, 0 );
		stage += 1;

		// Integrate velocities
		stage->type = b3_stageIntegrateVelocities;
		stage->blocks = bodyBlocks;
		stage->blockCount = bodyBlockCount;
		stage->colorIndex = -1;
		b3AtomicStoreInt( &stage->completionCount, 0 );
		stage += 1;

		// Warm start
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage->type = b3_stageWarmStart;
			stage->blocks = graphColorBlocks[i];
			stage->blockCount = colorJointBlockCounts[i] + colorManifoldBlockCounts[i][0] + colorManifoldBlockCounts[i][1] +
								colorManifoldBlockCounts[i][2];
			stage->colorIndex = activeColorIndices[i];
			b3AtomicStoreInt( &stage->completionCount, 0 );
			stage += 1;
		}

		// Solve graph
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage->type = b3_stageSolve;
			stage->blocks = graphColorBlocks[i];
			stage->blockCount = colorJointBlockCounts[i] + colorManifoldBlockCounts[i][0] + colorManifoldBlockCounts[i][1] +
								colorManifoldBlockCounts[i][2];
			stage->colorIndex = activeColorIndices[i];
			b3AtomicStoreInt( &stage->completionCount, 0 );
			stage += 1;
		}

		// Integrate positions
		stage->type = b3_stageIntegratePositions;
		stage->blocks = bodyBlocks;
		stage->blockCount = bodyBlockCount;
		stage->colorIndex = -1;
		b3AtomicStoreInt( &stage->completionCount, 0 );
		stage += 1;

		// Relax constraints
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage->type = b3_stageRelax;
			stage->blocks = graphColorBlocks[i];
			stage->blockCount = colorJointBlockCounts[i] + colorManifoldBlockCounts[i][0] + colorManifoldBlockCounts[i][1] +
								colorManifoldBlockCounts[i][2];
			stage->colorIndex = activeColorIndices[i];
			b3AtomicStoreInt( &stage->completionCount, 0 );
			stage += 1;
		}

		// Restitution
		// Note: joint blocks mixed in, could have joint limit restitution
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage->type = b3_stageRestitution;
			stage->blocks = graphColorBlocks[i];
			stage->blockCount = colorJointBlockCounts[i] + colorManifoldBlockCounts[i][0] + colorManifoldBlockCounts[i][1] +
								colorManifoldBlockCounts[i][2];
			stage->colorIndex = activeColorIndices[i];
			b3AtomicStoreInt( &stage->completionCount, 0 );
			stage += 1;
		}

		// Store contact1 impulses
		stage->type = b3_stageStoreImpulses1;
		stage->blocks = contact1BlockCount > 0 ? contact1Blocks : nullptr;
		stage->blockCount = contact1BlockCount;
		stage->colorIndex = -1;
		b3AtomicStoreInt( &stage->completionCount, 0 );
		stage += 1;

		// Store contact2 impulses
		stage->type = b3_stageStoreImpulses2;
		stage->blocks = contact2BlockCount > 0 ? contact2Blocks : nullptr;
		stage->blockCount = contact2BlockCount;
		stage->colorIndex = -1;
		b3AtomicStoreInt( &stage->completionCount, 0 );
		stage += 1;

		// Store contact3 impulses
		stage->type = b3_stageStoreImpulses3;
		stage->blocks = contact3BlockCount > 0 ? contact3Blocks : nullptr;
		stage->blockCount = contact3BlockCount;
		stage->colorIndex = -1;
		b3AtomicStoreInt( &stage->completionCount, 0 );
		stage += 1;

		B3_ASSERT( (int)( stage - stages ) == stageCount );

		B3_ASSERT( workerCount <= B3_MAX_WORKERS );
		b3WorkerContext workerContext[B3_MAX_WORKERS];

		stepContext->graph = graph;
		stepContext->joints = awakeJointCount > 0 ? jointSims : nullptr;
		stepContext->contactSims = nullptr;
		stepContext->manifoldLookups[0] = totalContactTypeW[0] > 0 ? contact1Lookup : nullptr;
		stepContext->manifoldLookups[1] = totalContactTypeW[1] > 0 ? contact2Lookup : nullptr;
		stepContext->manifoldLookups[2] = totalContactTypeW[2] > 0 ? contact3Lookup : nullptr;
		stepContext->contactConstraints[0] = totalContactTypeW[0] > 0 ? contact1Constraints : nullptr;
		stepContext->contactConstraints[1] = totalContactTypeW[1] > 0 ? contact2Constraints : nullptr;
		stepContext->contactConstraints[2] = totalContactTypeW[2] > 0 ? contact3Constraints : nullptr;
		stepContext->activeColorCount = activeColorCount;
		stepContext->workerCount = workerCount;
		stepContext->stageCount = stageCount;
		stepContext->stages = stages;
		b3AtomicStoreU32( &stepContext->atomicSyncBits, 0 );

		world->profile.prepareStages = b3GetMillisecondsAndReset( &prepareTicks );
		b3TracyCZoneEnd( prepare_stages );

		b3TracyCZoneNC( solve_constraints, "Solve Constraints", b3_colorIndigo, true );
		uint64_t constraintTicks = b3GetTicks();

		// Must use worker index because thread 0 can be assigned multiple tasks by enkiTS
		int jointIdCapacity = b3GetIdCapacity( &world->jointIdPool );
		for ( int i = 0; i < workerCount; ++i )
		{
			b3SetBitCountAndClear( &world->taskContexts.data[i].jointStateBitSet, jointIdCapacity );

			workerContext[i].context = stepContext;
			workerContext[i].workerIndex = i;
			workerContext[i].userTask = world->enqueueTaskFcn( b3SolverTask, 1, 1, workerContext + i, world->userTaskContext );
			world->taskCount += 1;
			world->activeTaskCount += workerContext[i].userTask == nullptr ? 0 : 1;
		}

		// Finish island split
		if ( splitIslandTask != nullptr )
		{
			world->finishTaskFcn( splitIslandTask, world->userTaskContext );
			world->activeTaskCount -= 1;
		}
		world->splitIslandId = B3_NULL_INDEX;

		// Finish constraint solve
		for ( int i = 0; i < workerCount; ++i )
		{
			if ( workerContext[i].userTask != nullptr )
			{
				world->finishTaskFcn( workerContext[i].userTask, world->userTaskContext );
				world->activeTaskCount -= 1;
			}
		}

		world->profile.solveConstraints = b3GetMillisecondsAndReset( &constraintTicks );
		b3TracyCZoneEnd( solve_constraints );

		b3TracyCZoneNC( update_transforms, "Update Transforms", b3_colorMediumSeaGreen, true );
		uint64_t transformTicks = b3GetTicks();

		// Prepare contact, enlarged body, and island bit sets used in body finalization.
		int awakeIslandCount = awakeSet->islandSims.count;
		for ( int i = 0; i < world->workerCount; ++i )
		{
			b3TaskContext* taskContext = world->taskContexts.data + i;
			taskContext->sensorHits.Clear();
			b3SetBitCountAndClear( &taskContext->enlargedSimBitSet, awakeBodyCount );
			b3SetBitCountAndClear( &taskContext->awakeIslandBitSet, awakeIslandCount );
			taskContext->splitIslandId = B3_NULL_INDEX;
			taskContext->splitSleepTime = 0.0f;
		}

		// Finalize bodies. Must happen after the constraint solver and after island splitting.
		void* finalizeBodiesTask =
			world->enqueueTaskFcn( b3FinalizeBodiesTask, awakeBodyCount, 64, stepContext, world->userTaskContext );
		world->taskCount += 1;
		if ( finalizeBodiesTask != nullptr )
		{
			world->finishTaskFcn( finalizeBodiesTask, world->userTaskContext );
		}

		// Free in reverse order
		b3FreeArenaItem( &world->arena, graphBlocks );
		b3FreeArenaItem( &world->arena, jointBlocks );
		b3FreeArenaItem( &world->arena, contact3Blocks );
		b3FreeArenaItem( &world->arena, contact2Blocks );
		b3FreeArenaItem( &world->arena, contact1Blocks );
		b3FreeArenaItem( &world->arena, bodyBlocks );
		b3FreeArenaItem( &world->arena, stages );
		b3FreeArenaItem( &world->arena, overflowManifoldConstraints );
		b3FreeArenaItem( &world->arena, contact3Constraints );
		b3FreeArenaItem( &world->arena, contact2Constraints );
		b3FreeArenaItem( &world->arena, contact1Constraints );
		b3FreeArenaItem( &world->arena, jointSims );
		b3FreeArenaItem( &world->arena, contact3Lookup );
		b3FreeArenaItem( &world->arena, contact2Lookup );
		b3FreeArenaItem( &world->arena, contact1Lookup );

		world->profile.transforms = b3GetMilliseconds( transformTicks );
		b3TracyCZoneEnd( update_transforms );
	}

	// Report joint events
	{
		b3TracyCZoneNC( joint_events, "Joint Events", b3_colorPeru, true );
		uint64_t jointEventTicks = b3GetTicks();

		// Gather bits for all joints that have force/torque events
		b3BitSet* jointStateBitSet = &world->taskContexts.data[0].jointStateBitSet;
		for ( int i = 1; i < world->workerCount; ++i )
		{
			b3InPlaceUnion( jointStateBitSet, &world->taskContexts.data[i].jointStateBitSet );
		}

		{
			uint32_t wordCount = jointStateBitSet->blockCount;
			uint64_t* bits = jointStateBitSet->bits;

			b3Joint* jointArray = world->joints.data;
			uint16_t worldIndex0 = world->worldId;

			for ( uint32_t k = 0; k < wordCount; ++k )
			{
				uint64_t word = bits[k];
				while ( word != 0 )
				{
					uint32_t ctz = b3CTZ64( word );
					int jointId = (int)( 64 * k + ctz );

					B3_ASSERT( jointId < world->joints.capacity );

					b3Joint* joint = jointArray + jointId;

					B3_ASSERT( joint->setIndex == b3_awakeSet );

					b3JointEvent event = {
						.jointId =
							{
								.index1 = jointId + 1,
								.world0 = worldIndex0,
								.generation = joint->generation,
							},
						.userData = joint->userData,
					};

					world->jointEvents.PushBack( event );

					// Clear the smallest set bit
					word = word & ( word - 1 );
				}
			}
		}

		world->profile.jointEvents = b3GetMilliseconds( jointEventTicks );
		b3TracyCZoneEnd( joint_events );
	}

	// Report hit events
	// todo_erin perhaps optimize this with a bitset
	// todo_erin perhaps do this in parallel with other work below
	{
		b3TracyCZoneNC( hit_events, "Hit", b3_colorRosyBrown, true );
		uint64_t hitTicks = b3GetTicks();

		B3_ASSERT( world->contactHitEvents.count == 0 );

		float threshold = world->hitEventThreshold;
		b3GraphColor* colors = world->constraintGraph.colors;
		for ( int colorIndex = 0; colorIndex < B3_GRAPH_COLOR_COUNT; ++colorIndex )
		{
			b3GraphColor* color = colors + colorIndex;
			int contactCount = color->contactSims.count;
			b3ContactSim* contactSims = color->contactSims.data;
			for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
			{
				b3ContactSim* contactSim = contactSims + contactIndex;
				if ( ( contactSim->simFlags & b3_simEnableHitEvent ) == 0 )
				{
					continue;
				}

				b3Shape* shapeA = world->shapes.Get( contactSim->shapeIdA );
				b3Shape* shapeB = world->shapes.Get( contactSim->shapeIdB );

				b3ContactHitEvent event = {};
				event.approachSpeed = threshold;

				bool hit = false;
				for ( int manifoldIndex = 0; manifoldIndex < contactSim->manifoldCount; ++manifoldIndex )
				{
					const b3Manifold* manifold = contactSim->manifolds + manifoldIndex;
					int pointCount = manifold->pointCount;
					for ( int k = 0; k < pointCount; ++k )
					{
						const b3ManifoldPoint* mp = manifold->points + k;
						float approachSpeed = -mp->normalVelocity;

						// Need to check max impulse because the point may be speculative and not colliding
						if ( approachSpeed > event.approachSpeed && mp->totalNormalImpulse > 0.0f )
						{
							event.approachSpeed = approachSpeed;
							event.point = mp->point;
							event.normal = manifold->normal;
							event.triangleIndex = mp->triangleIndex;

							// todo missing per triangle material
							event.userMaterialIdA = shapeA->materials[0].userMaterialId;
							event.userMaterialIdB = shapeB->materials[0].userMaterialId;

							if ( shapeA->materialCount > 1 )
							{
								if ( shapeA->type == b3_meshShape )
								{
									if ( 0 <= event.triangleIndex && event.triangleIndex < shapeA->mesh.data->triangleCount )
									{
										const uint8_t* materialIndices = b3GetMeshMaterialIndices( shapeA->mesh.data );
										int materialIndex = materialIndices[event.triangleIndex];
										materialIndex = b3ClampInt( materialIndex, 0, shapeA->materialCount - 1 );
										event.userMaterialIdA = shapeA->materials[materialIndex].userMaterialId;
									}
								}
								else if ( shapeA->type == b3_heightShape )
								{
									int materialIndex = b3GetHeightFieldMaterial( shapeA->heightField, event.triangleIndex );
									if (materialIndex != B3_NULL_INDEX)
									{
										materialIndex = b3ClampInt( materialIndex, 0, shapeA->materialCount - 1 );
										event.userMaterialIdA = shapeA->materials[materialIndex].userMaterialId;
									}
								}
							}

							hit = true;
						}
					}
				}

				if ( hit == true )
				{
					event.shapeIdA = b3ShapeId{ shapeA->id + 1, world->worldId, shapeA->generation };
					event.shapeIdB = b3ShapeId{ shapeB->id + 1, world->worldId, shapeB->generation };

					world->contactHitEvents.PushBack( event );
				}
			}
		}

		world->profile.hitEvents = b3GetMilliseconds( hitTicks );
		b3TracyCZoneEnd( hit_events );
	}

	{
		b3TracyCZoneNC( refit_bvh, "Refit BVH", b3_colorFireBrick, true );
		uint64_t refitTicks = b3GetTicks();

		// Finish the user tree task that was queued earlier in the time step. This must be complete before touching the
		// broad-phase.
		if ( world->userTreeTask != nullptr )
		{
			world->finishTaskFcn( world->userTreeTask, world->userTaskContext );
			world->userTreeTask = nullptr;
			world->activeTaskCount -= 1;
		}

		b3ValidateNoEnlarged( &world->broadPhase );

		// Gather bits for all sim bodies that have enlarged AABBs
		b3BitSet* enlargedBodyBitSet = &world->taskContexts.data[0].enlargedSimBitSet;
		for ( int i = 1; i < world->workerCount; ++i )
		{
			b3InPlaceUnion( enlargedBodyBitSet, &world->taskContexts.data[i].enlargedSimBitSet );
		}

		// Enlarge broad-phase proxies and build move array
		// Apply shape AABB changes to broad-phase. This also create the move array which must be
		// in deterministic order. I'm tracking sim bodies because the number of shape ids can be huge.
		// This has to happen before bullets are processed.
		{
			b3BroadPhase* broadPhase = &world->broadPhase;
			uint32_t wordCount = enlargedBodyBitSet->blockCount;
			uint64_t* bits = enlargedBodyBitSet->bits;

			// Fast array access is important here
			b3Body* bodyArray = world->bodies.data;
			b3BodySim* bodySimArray = awakeSet->bodySims.data;
			b3Shape* shapeArray = world->shapes.data;

			for ( uint32_t k = 0; k < wordCount; ++k )
			{
				uint64_t word = bits[k];
				while ( word != 0 )
				{
					uint32_t ctz = b3CTZ64( word );
					uint32_t bodySimIndex = 64 * k + ctz;

					b3BodySim* bodySim = bodySimArray + bodySimIndex;

					b3Body* body = bodyArray + bodySim->bodyId;

					int shapeId = body->headShapeId;
					if ( ( bodySim->flags & ( b3_isBullet | b3_isFast ) ) == ( b3_isBullet | b3_isFast ) )
					{
						// Fast bullet bodies don't have their final AABB yet
						while ( shapeId != B3_NULL_INDEX )
						{
							b3Shape* shape = shapeArray + shapeId;

							// Shape is fast. It's aabb will be enlarged in continuous collision.
							// Update the move array here for determinism because bullets are processed
							// below in non-deterministic order.
							b3BufferMove( broadPhase, shape->proxyKey );

							shapeId = shape->nextShapeId;
						}
					}
					else
					{
						while ( shapeId != B3_NULL_INDEX )
						{
							b3Shape* shape = shapeArray + shapeId;

							// The AABB may not have been enlarged, despite the body being flagged as enlarged.
							// For example, a body with multiple shapes may have not have all shapes enlarged.
							// A fast body may have been flagged as enlarged despite having no shapes enlarged.
							if ( shape->enlargedAABB )
							{
								B3_VALIDATE( b3IsSaneAABB( shape->fatAABB ) );

								b3BroadPhase_EnlargeProxy( broadPhase, shape->proxyKey, shape->fatAABB );
								shape->enlargedAABB = false;
							}

							shapeId = shape->nextShapeId;
						}
					}

					// Clear the smallest set bit
					word = word & ( word - 1 );
				}
			}
		}

		b3ValidateBroadphase( &world->broadPhase );

		world->profile.refit = b3GetMilliseconds( refitTicks );
		b3TracyCZoneEnd( refit_bvh );
	}

	int bulletBodyCount = b3AtomicLoadInt( &stepContext->bulletBodyCount );
	if ( bulletBodyCount > 0 )
	{
		b3TracyCZoneNC( bullets, "Bullets", b3_colorDarkGoldenRod, true );
		uint64_t bulletTicks = b3GetTicks();

		// Fast bullet bodies
		// Note: a bullet body may be moving slow
		int minRange = 8;
		void* userBulletBodyTask =
			world->enqueueTaskFcn( &b3BulletBodyTask, bulletBodyCount, minRange, stepContext, world->userTaskContext );
		world->taskCount += 1;
		if ( userBulletBodyTask != nullptr )
		{
			world->finishTaskFcn( userBulletBodyTask, world->userTaskContext );
		}

		// Serially enlarge broad-phase proxies for bullet shapes
		b3BroadPhase* broadPhase = &world->broadPhase;
		b3DynamicTree* dynamicTree = broadPhase->trees + b3_dynamicBody;

		// Fast array access is important here
		b3Body* bodyArray = world->bodies.data;
		b3BodySim* bodySimArray = awakeSet->bodySims.data;
		b3Shape* shapeArray = world->shapes.data;

		// Serially enlarge broad-phase proxies for bullet shapes
		int* bulletBodySimIndices = stepContext->bulletBodies;

		// This loop has non-deterministic order but it shouldn't affect the result
		for ( int i = 0; i < bulletBodyCount; ++i )
		{
			b3BodySim* bulletBodySim = bodySimArray + bulletBodySimIndices[i];
			if ( ( bulletBodySim->flags & b3_enlargeBounds ) == 0 )
			{
				continue;
			}

			// Clear flag
			bulletBodySim->flags &= ~b3_enlargeBounds;

			int bodyId = bulletBodySim->bodyId;
			B3_ASSERT( 0 <= bodyId && bodyId < world->bodies.count );
			b3Body* bulletBody = bodyArray + bodyId;

			int shapeId = bulletBody->headShapeId;
			while ( shapeId != B3_NULL_INDEX )
			{
				b3Shape* shape = shapeArray + shapeId;
				if ( shape->enlargedAABB == false )
				{
					shapeId = shape->nextShapeId;
					continue;
				}

				// clear flag
				shape->enlargedAABB = false;

				int proxyKey = shape->proxyKey;
				int proxyId = B3_PROXY_ID( proxyKey );
				B3_ASSERT( B3_PROXY_TYPE( proxyKey ) == b3_dynamicBody );

				// all fast bullet shapes should already be in the move buffer
				B3_ASSERT( b3ContainsKey( &broadPhase->moveSet, proxyKey + 1 ) );

				b3DynamicTree_EnlargeProxy( dynamicTree, proxyId, shape->fatAABB );

				shapeId = shape->nextShapeId;
			}
		}

		world->profile.bullets = b3GetMilliseconds( bulletTicks );
		b3TracyCZoneEnd( bullets );
	}

	b3FreeArenaItem( &world->arena, stepContext->bulletBodies );
	stepContext->bulletBodies = nullptr;
	b3AtomicStoreInt( &stepContext->bulletBodyCount, 0 );

	// Report sensor hits. This may include bullets sensor hits.
	{
		b3TracyCZoneNC( sensor_hits, "Sensor Hits", b3_colorPowderBlue, true );
		uint64_t sensorHitTicks = b3GetTicks();

		int workerCount = world->workerCount;
		B3_ASSERT( workerCount == world->taskContexts.count );

		for ( int i = 0; i < workerCount; ++i )
		{
			b3TaskContext* taskContext = world->taskContexts.data + i;
			int hitCount = taskContext->sensorHits.count;
			b3SensorHit* hits = taskContext->sensorHits.data;

			for ( int j = 0; j < hitCount; ++j )
			{
				b3SensorHit hit = hits[j];
				b3Shape* sensorShape = world->shapes.Get( hit.sensorId );
				b3Shape* visitor = world->shapes.Get( hit.visitorId );

				b3Sensor* sensor = world->sensors.Get( sensorShape->sensorIndex );
				b3Visitor shapeRef = {
					.shapeId = hit.visitorId,
					.generation = visitor->generation,
				};
				sensor->hits.PushBack( shapeRef );
			}
		}

		world->profile.sensorHits = b3GetMilliseconds( sensorHitTicks );
		b3TracyCZoneEnd( sensor_hits );
	}

	// Island sleeping
	// This must be done last because putting islands to sleep invalidates the enlarged body bits.
	// todo_erin figure out how to do this in parallel with tree refit
	if ( world->enableSleep == true )
	{
		b3TracyCZoneNC( sleep_islands, "Island Sleep", b3_colorLightSlateGray, true );
		uint64_t sleepTicks = b3GetTicks();

		// Collect split island candidate for the next time step. No need to split if sleeping is disabled.
		B3_ASSERT( world->splitIslandId == B3_NULL_INDEX );
		float splitSleepTimer = 0.0f;
		for ( int i = 0; i < world->workerCount; ++i )
		{
			b3TaskContext* taskContext = world->taskContexts.data + i;
			if ( taskContext->splitIslandId != B3_NULL_INDEX && taskContext->splitSleepTime >= splitSleepTimer )
			{
				B3_ASSERT( taskContext->splitSleepTime > 0.0f );

				// Tie breaking for determinism. Largest island id wins. Needed due to work stealing.
				if ( taskContext->splitSleepTime == splitSleepTimer && taskContext->splitIslandId < world->splitIslandId )
				{
					continue;
				}

				world->splitIslandId = taskContext->splitIslandId;
				splitSleepTimer = taskContext->splitSleepTime;
			}
		}

		b3BitSet* awakeIslandBitSet = &world->taskContexts.data[0].awakeIslandBitSet;
		for ( int i = 1; i < world->workerCount; ++i )
		{
			b3InPlaceUnion( awakeIslandBitSet, &world->taskContexts.data[i].awakeIslandBitSet );
		}

		// Need to process in reverse because this moves islands to sleeping solver sets.
		b3IslandSim* islands = awakeSet->islandSims.data;
		int count = awakeSet->islandSims.count;
		for ( int islandIndex = count - 1; islandIndex >= 0; islandIndex -= 1 )
		{
			if ( b3GetBit( awakeIslandBitSet, islandIndex ) == true )
			{
				// this island is still awake
				continue;
			}

			b3IslandSim* island = islands + islandIndex;
			int islandId = island->islandId;

			b3TrySleepIsland( world, islandId );
		}

		b3ValidateSolverSets( world );

		world->profile.sleepIslands = b3GetMilliseconds( sleepTicks );
		b3TracyCZoneEnd( sleep_islands );
	}
}
