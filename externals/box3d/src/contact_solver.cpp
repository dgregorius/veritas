// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact_solver.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "math_internal.h"
#include "physics_world.h"
#include "solver_set.h"

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

void b3PrepareOverflowContacts( b3StepContext* context )
{
	b3TracyCZoneNC( prepare_overflow_contact, "Prepare Overflow Contact", b3_colorYellow, true );

	b3World* world = context->world;
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;
	b3ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contactSims.count;
	b3ContactSim* contacts = color->contactSims.data;
	b3BodyState* awakeStates = context->states;

#if B3_ENABLE_VALIDATION
	b3Body* bodies = world->bodies.data;
#endif

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	b3Softness contactSoftness = context->contactSoftness;
	b3Softness staticSoftness = context->staticSoftness;

	float warmStartScale = world->enableWarmStarting ? 1.0f : 0.0f;

	int constraintIndex = 0;

	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		b3ContactSim* contactSim = contacts + contactIndex;
		int indexA = contactSim->bodySimIndexA;
		int indexB = contactSim->bodySimIndexB;

#if B3_ENABLE_VALIDATION
		b3Body* bodyA = bodies + contactSim->bodyIdA;
		int validIndexA = bodyA->setIndex == b3_awakeSet ? bodyA->localIndex : B3_NULL_INDEX;
		B3_ASSERT( indexA == validIndexA );

		b3Body* bodyB = bodies + contactSim->bodyIdB;
		int validIndexB = bodyB->setIndex == b3_awakeSet ? bodyB->localIndex : B3_NULL_INDEX;
		B3_ASSERT( indexB == validIndexB );
#endif

		b3Vec3 vA = b3Vec3_zero;
		b3Vec3 wA = b3Vec3_zero;
		float mA = contactSim->invMassA;
		b3Matrix3 iA = contactSim->invIA;
		if ( indexA != B3_NULL_INDEX )
		{
			b3BodyState* stateA = awakeStates + indexA;
			vA = stateA->linearVelocity;
			wA = stateA->angularVelocity;
		}

		b3Vec3 vB = b3Vec3_zero;
		b3Vec3 wB = b3Vec3_zero;
		float mB = contactSim->invMassB;
		b3Matrix3 iB = contactSim->invIB;
		if ( indexB != B3_NULL_INDEX )
		{
			b3BodyState* stateB = awakeStates + indexB;
			vB = stateB->linearVelocity;
			wB = stateB->angularVelocity;
		}

		int manifoldCount = contactSim->manifoldCount;
		B3_ASSERT( 0 < manifoldCount && manifoldCount <= 3 );
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3Manifold* manifold = contactSim->manifolds + manifoldIndex;
			int pointCount = manifold->pointCount;

			B3_ASSERT( 0 < pointCount && pointCount <= 4 );

			b3ContactConstraint* constraint = constraints + constraintIndex;
			constraint->manifold = manifold;
			constraint->indexA = indexA;
			constraint->indexB = indexB;
			constraint->normal = manifold->normal;
			constraint->tangent1 = b3Perp( manifold->normal );
			constraint->tangent2 = b3Cross( constraint->tangent1, manifold->normal );
			constraint->friction = contactSim->friction;
			constraint->restitution = contactSim->restitution;
			constraint->rollingResistance = contactSim->rollingResistance;
			constraint->tangentVelocity1 = b3Dot( contactSim->tangentVelocity, constraint->tangent1 );
			constraint->tangentVelocity2 = b3Dot( contactSim->tangentVelocity, constraint->tangent2 );
			constraint->pointCount = pointCount;

			if ( indexA == B3_NULL_INDEX || indexB == B3_NULL_INDEX )
			{
				constraint->softness = staticSoftness;
			}
			else
			{
				constraint->softness = contactSoftness;
			}

			// copy mass into constraint to avoid cache misses during sub-stepping
			constraint->invMassA = mA;
			constraint->invIA = iA;
			constraint->invMassB = mB;
			constraint->invIB = iB;

			b3Vec3 normal = constraint->normal;
			b3Vec3 tangent1 = constraint->tangent1;
			b3Vec3 tangent2 = constraint->tangent2;
			b3Vec3 originA = b3Vec3_zero;
			b3Vec3 originB = b3Vec3_zero;

			// todo testing speculative affect on central friction
			int touchingPointCount = 0;
			float touchTolerance = 2.0f * B3_SPECULATIVE_DISTANCE + B3_HUGE;

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				const b3ManifoldPoint* mp = manifold->points + pointIndex;
				b3ContactConstraintPoint* cp = constraint->points + pointIndex;

				cp->normalImpulse = warmStartScale * mp->normalImpulse;
				cp->totalNormalImpulse = 0.0f;

				b3Vec3 rA = mp->anchorA;
				b3Vec3 rB = mp->anchorB;

				if ( mp->separation < touchTolerance )
				{
					originA += rA;
					originB += rB;
					touchingPointCount += 1;
				}

				cp->anchorA = rA;
				cp->anchorB = rB;
				cp->baseSeparation = mp->separation - b3Dot( b3Sub( rB, rA ), normal );

				b3Vec3 rnA = b3Cross( rA, normal );
				b3Vec3 rnB = b3Cross( rB, normal );
				float kNormal = mA + mB + b3Dot( rnA, b3MulMV( iA, rnA ) ) + b3Dot( rnB, b3MulMV( iB, rnB ) );
				cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

				// Save relative velocity for restitution
				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				cp->relativeVelocity = b3Dot( normal, b3Sub( vrB, vrA ) );
			}

			B3_ASSERT( touchingPointCount > 0 );

			float invCount = 1.0f / touchingPointCount;
			originA *= invCount;
			originB *= invCount;
			constraint->originA = originA;
			constraint->originB = originB;

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				b3ContactConstraintPoint* cp = constraint->points + pointIndex;
				if ( manifold->points[pointIndex].separation < touchTolerance )
				{
					constraint->leverArms[pointIndex] = b3Distance( cp->anchorA, constraint->originA );
				}
				else
				{
					constraint->leverArms[pointIndex] = 0.0f;
				}
			}

			b3Vec3 rtA1 = b3Cross( originA, tangent1 );
			b3Vec3 rtA2 = b3Cross( originA, tangent2 );

			b3Vec3 rtB1 = b3Cross( originB, tangent1 );
			b3Vec3 rtB2 = b3Cross( originB, tangent2 );

			{
				b3Matrix2 k;
				k.cx.x = mA + mB + b3Dot( rtA1, b3MulMV( iA, rtA1 ) ) + b3Dot( rtB1, b3MulMV( iB, rtB1 ) );
				k.cy.y = mA + mB + b3Dot( rtA2, b3MulMV( iA, rtA2 ) ) + b3Dot( rtB2, b3MulMV( iB, rtB2 ) );
				k.cx.y = k.cy.x = b3Dot( rtA1, b3MulMV( iA, rtA2 ) ) + b3Dot( rtB1, b3MulMV( iB, rtB2 ) );

				constraint->tangentMass = b3Invert2( k );
				constraint->frictionImpulse.x = warmStartScale * b3Dot( manifold->frictionImpulse, tangent1 );
				constraint->frictionImpulse.y = warmStartScale * b3Dot( manifold->frictionImpulse, tangent2 );
			}

			{
				float k = b3Dot( normal, b3MulMV( iA + iB, normal ) );
				constraint->twistMass = k > 0.0f ? 1.0f / k : 0.0f;
				constraint->twistImpulse = warmStartScale * manifold->twistImpulse;
			}

			{
				constraint->rollingMass = b3InvertMatrix( iA + iB );
				constraint->rollingImpulse = warmStartScale * manifold->rollingImpulse;
			}

			constraintIndex += 1;
		}
	}

	B3_ASSERT( constraintIndex == color->overflowManifoldCount );

	b3TracyCZoneEnd( prepare_overflow_contact );
}

void b3WarmStartOverflowContacts( b3StepContext* context )
{
	b3TracyCZoneNC( warmstart_overflow_contact, "WarmStart Overflow Contact", b3_colorDarkOrange, true );

	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;
	b3ContactConstraint* constraints = color->overflowConstraints;
	int constraintCount = color->overflowManifoldCount;
	b3World* world = context->world;
	b3SolverSet* awakeSet = world->solverSets.Get( b3_awakeSet );
	b3BodyState* states = awakeSet->bodyStates.data;

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	b3BodyState dummyState = b3_identityBodyState;

	for ( int i = 0; i < constraintCount; ++i )
	{
		const b3ContactConstraint* constraint = constraints + i;

		int indexA = constraint->indexA;
		int indexB = constraint->indexB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;

		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;

		float mA = constraint->invMassA;
		b3Matrix3 iA = constraint->invIA;
		float mB = constraint->invMassB;
		b3Matrix3 iB = constraint->invIB;

		// Normal impulses
		b3Vec3 normal = constraint->normal;
		int pointCount = constraint->pointCount;
		for ( int j = 0; j < pointCount; ++j )
		{
			const b3ContactConstraintPoint* cp = constraint->points + j;

			// fixed anchors
			b3Vec3 rA = cp->anchorA;
			b3Vec3 rB = cp->anchorB;

			b3Vec3 impulse = b3MulSV( cp->normalImpulse, normal );
			wA -= iA * b3Cross( rA, impulse );
			vA = b3MulSub( vA, mA, impulse );
			wB += iB * b3Cross( rB, impulse );
			vB = b3MulAdd( vB, mB, impulse );
		}

		// Central friction
		{
			b3Vec3 rA = constraint->originA;
			b3Vec3 rB = constraint->originB;
			b3Vec3 impulse = constraint->frictionImpulse.x * constraint->tangent1;
			impulse += constraint->frictionImpulse.y * constraint->tangent2;

			wA -= iA * b3Cross( rA, impulse );
			vA = b3MulSub( vA, mA, impulse );
			wB += iB * b3Cross( rB, impulse );
			vB = b3MulAdd( vB, mB, impulse );
		}

		// Central twist friction
		{
			b3Vec3 impulse = constraint->twistImpulse * constraint->normal;
			wA -= b3MulMV( iA, impulse );
			wB += b3MulMV( iB, impulse );
		}

		// Rolling resistance
		{
			b3Vec3 impulse = constraint->rollingImpulse;
			wA -= b3MulMV( iA, impulse );
			wB += b3MulMV( iB, impulse );
		}

		if ( stateA->flags & b3_dynamicFlag )
		{
			stateA->linearVelocity = vA;
			stateA->angularVelocity = wA;
		}

		if ( stateB->flags & b3_dynamicFlag )
		{
			stateB->linearVelocity = vB;
			stateB->angularVelocity = wB;
		}
	}

	b3TracyCZoneEnd( warmstart_overflow_contact );
}

void b3SolveOverflowContacts( b3StepContext* context, bool useBias )
{
	b3TracyCZoneNC( solve_contact, "Solve Contact", b3_colorAliceBlue, true );

	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;
	b3ContactConstraint* constraints = color->overflowConstraints;
	int constraintCount = color->overflowManifoldCount;
	b3World* world = context->world;
	b3SolverSet* awakeSet = world->solverSets.Get( b3_awakeSet );
	b3BodyState* states = awakeSet->bodyStates.data;

	float inv_h = context->inv_h;
	const float contactSpeed = context->world->contactSpeed;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b3BodyState dummyState = b3_identityBodyState;

	for ( int i = 0; i < constraintCount; ++i )
	{
		b3ContactConstraint* constraint = constraints + i;
		float mA = constraint->invMassA;
		b3Matrix3 iA = constraint->invIA;
		float mB = constraint->invMassB;
		b3Matrix3 iB = constraint->invIB;

		b3BodyState* stateA = constraint->indexA == B3_NULL_INDEX ? &dummyState : states + constraint->indexA;
		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Quat dqA = stateA->deltaRotation;

		b3BodyState* stateB = constraint->indexB == B3_NULL_INDEX ? &dummyState : states + constraint->indexB;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;
		b3Quat dqB = stateB->deltaRotation;

		b3Vec3 dp = b3Sub( stateB->deltaPosition, stateA->deltaPosition );

		b3Vec3 normal = constraint->normal;
		b3Softness softness = constraint->softness;

		int pointCount = constraint->pointCount;

		float totalNormalImpulse = 0.0f;
		float totalTwistLimit = 0.0f;

		for ( int j = 0; j < pointCount; ++j )
		{
			b3ContactConstraintPoint* cp = constraint->points + j;

			// Fixed anchor points for applying impulses
			b3Vec3 rA = cp->anchorA;
			b3Vec3 rB = cp->anchorB;

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			b3Vec3 ds = b3Add( dp, b3Sub( b3RotateVector( dqB, rB ), b3RotateVector( dqA, rA ) ) );
			float s = b3Dot( ds, normal ) + cp->baseSeparation;

			float velocityBias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( s > 0.0f )
			{
				// speculative bias
				velocityBias = s * inv_h;
			}
			else if ( useBias )
			{
				velocityBias = b3MaxFloat( softness.massScale * softness.biasRate * s, -contactSpeed );
				massScale = softness.massScale;
				impulseScale = softness.impulseScale;
			}

			// relative normal velocity at contact
			b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
			b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
			float vn = b3Dot( b3Sub( vrB, vrA ), normal );

			// incremental normal impulse
			float deltaImpulse = -cp->normalMass * ( massScale * vn + velocityBias ) - impulseScale * cp->normalImpulse;

			// clamp the accumulated impulse
			float newImpulse = b3MaxFloat( cp->normalImpulse + deltaImpulse, 0.0f );
			deltaImpulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;
			cp->totalNormalImpulse += newImpulse;

			totalNormalImpulse += newImpulse;
			totalTwistLimit += constraint->leverArms[j] * cp->normalImpulse;

			// apply normal impulse
			b3Vec3 P = b3MulSV( deltaImpulse, normal );
			vA = b3MulSub( vA, mA, P );
			wA -= iA * b3Cross( rA, P );

			vB = b3MulAdd( vB, mB, P );
			wB += iB * b3Cross( rB, P );
		}

		// Central twist friction
		{
			float twistSpeed = b3Dot( constraint->normal, wB - wA );
			float maxImpulse = constraint->friction * totalTwistLimit;
			float deltaImpulse = -constraint->twistMass * twistSpeed;
			float oldImpulse = constraint->twistImpulse;
			constraint->twistImpulse = b3ClampFloat( oldImpulse + deltaImpulse, -maxImpulse, maxImpulse );
			deltaImpulse = constraint->twistImpulse - oldImpulse;

			wA -= b3MulMV( iA, deltaImpulse * constraint->normal );
			wB += b3MulMV( iB, deltaImpulse * constraint->normal );
		}

		// Rolling resistance
		if (constraint->rollingResistance > 0.0f)
		{
			b3Vec3 deltaImpulse = -b3MulMV( constraint->rollingMass, wB - wA );
			b3Vec3 oldImpulse = constraint->rollingImpulse;
			constraint->rollingImpulse = oldImpulse + deltaImpulse;

			float maxImpulse = constraint->rollingResistance * totalNormalImpulse;
			float magSqr = b3Dot( constraint->rollingImpulse, constraint->rollingImpulse );
			if ( magSqr > maxImpulse * maxImpulse + FLT_EPSILON )
			{
				constraint->rollingImpulse *= maxImpulse / sqrtf( magSqr );
			}

			deltaImpulse = constraint->rollingImpulse - oldImpulse;

			wA -= iA * deltaImpulse;
			wB += iB * deltaImpulse;
		}

		// Central friction
		{
			b3Vec3 tangent1 = constraint->tangent1;
			b3Vec3 tangent2 = constraint->tangent2;

			// Fixed anchor points for applying impulses
			b3Vec3 rA = constraint->originA;
			b3Vec3 rB = constraint->originB;

			// Relative tangent velocity at contact
			b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
			b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
			b3Vec3 vr = vrB - vrA;
			b3Vec2 vt = {
				b3Dot( vr, tangent1 ) - constraint->tangentVelocity1,
				b3Dot( vr, tangent2 ) - constraint->tangentVelocity2,
			};

			// Incremental tangent impulse
			b3Vec2 deltaImpulse = -b3MulMV2( constraint->tangentMass, vt );
			b3Vec2 newImpulse = constraint->frictionImpulse + deltaImpulse;

			float friction = constraint->friction;
			float maxImpulse = friction * totalNormalImpulse;

			// Clamp the accumulated impulse
			float lengthSquared = b3Dot2( newImpulse, newImpulse );
			if ( lengthSquared > maxImpulse * maxImpulse )
			{
				newImpulse *= maxImpulse / sqrtf( lengthSquared );
			}
			deltaImpulse = newImpulse - constraint->frictionImpulse;
			constraint->frictionImpulse = newImpulse;

			// Apply delta impulse
			b3Vec3 P = deltaImpulse.x * tangent1 + deltaImpulse.y * tangent2;
			vA = b3MulSub( vA, mA, P );
			wA -= iA * b3Cross( rA, P );
			vB = b3MulAdd( vB, mB, P );
			wB += iB * b3Cross( rB, P );
		}

		uint32_t flagsA = stateA->flags;
		if ( flagsA & b3_allLocks )
		{
			vA.x = ( flagsA & b3_lockLinearX ) ? 0.0f : vA.x;
			vA.y = ( flagsA & b3_lockLinearY ) ? 0.0f : vA.y;
			vA.z = ( flagsA & b3_lockLinearZ ) ? 0.0f : vA.z;
			wA.x = ( flagsA & b3_lockAngularX ) ? 0.0f : wA.x;
			wA.y = ( flagsA & b3_lockAngularY ) ? 0.0f : wA.y;
			wA.z = ( flagsA & b3_lockAngularZ ) ? 0.0f : wA.z;
		}

		uint32_t flagsB = stateB->flags;
		if ( flagsB & b3_allLocks )
		{
			vB.x = ( flagsB & b3_lockLinearX ) ? 0.0f : vB.x;
			vB.y = ( flagsB & b3_lockLinearY ) ? 0.0f : vB.y;
			vB.z = ( flagsB & b3_lockLinearZ ) ? 0.0f : vB.z;
			wB.x = ( flagsB & b3_lockAngularX ) ? 0.0f : wB.x;
			wB.y = ( flagsB & b3_lockAngularY ) ? 0.0f : wB.y;
			wB.z = ( flagsB & b3_lockAngularZ ) ? 0.0f : wB.z;
		}

		if ( stateA->flags & b3_dynamicFlag )
		{
			stateA->linearVelocity = vA;
			stateA->angularVelocity = wA;
		}

		if ( stateB->flags & b3_dynamicFlag )
		{
			stateB->linearVelocity = vB;
			stateB->angularVelocity = wB;
		}
	}

	b3TracyCZoneEnd( solve_contact );
}

void b3ApplyOverflowRestitution( b3StepContext* context )
{
	b3TracyCZoneNC( overflow_resitution, "Overflow Restitution", b3_colorViolet, true );

	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;
	b3ContactConstraint* constraints = color->overflowConstraints;
	int constraintCount = color->overflowManifoldCount;
	b3World* world = context->world;
	b3SolverSet* awakeSet = world->solverSets.Get( b3_awakeSet );
	b3BodyState* states = awakeSet->bodyStates.data;

	float threshold = context->world->restitutionThreshold;

	// dummy state to represent a static body
	b3BodyState dummyState = b3_identityBodyState;

	for ( int i = 0; i < constraintCount; ++i )
	{
		b3ContactConstraint* constraint = constraints + i;

		float restitution = constraint->restitution;
		if ( restitution == 0.0f )
		{
			continue;
		}

		float mA = constraint->invMassA;
		b3Matrix3 iA = constraint->invIA;
		float mB = constraint->invMassB;
		b3Matrix3 iB = constraint->invIB;

		b3BodyState* stateA = constraint->indexA == B3_NULL_INDEX ? &dummyState : states + constraint->indexA;
		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;

		b3BodyState* stateB = constraint->indexB == B3_NULL_INDEX ? &dummyState : states + constraint->indexB;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;

		b3Vec3 normal = constraint->normal;
		int pointCount = constraint->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			b3ContactConstraintPoint* cp = constraint->points + j;

			// if the normal impulse is zero then there was no collision
			// this skips speculative contact points that didn't generate an impulse
			// The max normal impulse is used in case there was a collision that moved away within the sub-step process
			if ( cp->relativeVelocity > -threshold || cp->totalNormalImpulse == 0.0f )
			{
				continue;
			}

			// fixed anchor points
			b3Vec3 rA = cp->anchorA;
			b3Vec3 rB = cp->anchorB;

			// relative normal velocity at contact
			b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
			b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
			float vn = b3Dot( b3Sub( vrB, vrA ), normal );

			// compute normal impulse
			float impulse = -cp->normalMass * ( vn + restitution * cp->relativeVelocity );

			// clamp the accumulated impulse
			// todo should this be stored?
			float newImpulse = b3MaxFloat( cp->normalImpulse + impulse, 0.0f );
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;

			// Add the incremental impulse rather than the full impulse because this is not a sub-step
			cp->totalNormalImpulse += impulse;

			// apply contact impulse
			b3Vec3 P = b3MulSV( impulse, normal );
			vA = b3MulSub( vA, mA, P );
			wA -= iA * b3Cross( rA, P );
			vB = b3MulAdd( vB, mB, P );
			wB += iB * b3Cross( rB, P );
		}

		if ( stateA->flags & b3_dynamicFlag )
		{
			stateA->linearVelocity = vA;
			stateA->angularVelocity = wA;
		}

		if ( stateB->flags & b3_dynamicFlag )
		{
			stateB->linearVelocity = vB;
			stateB->angularVelocity = wB;
		}
	}

	b3TracyCZoneEnd( overflow_resitution );
}

void b3StoreOverflowImpulses( b3StepContext* context )
{
	b3TracyCZoneNC( store_impulses, "Store", b3_colorFireBrick, true );

	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;
	b3ContactConstraint* constraints = color->overflowConstraints;
	int constraintCount = color->overflowManifoldCount;

	for ( int constraintIndex = 0; constraintIndex < constraintCount; ++constraintIndex )
	{
		const b3ContactConstraint* constraint = constraints + constraintIndex;
		b3Manifold* manifold = constraint->manifold;
		int pointCount = manifold->pointCount;
		B3_ASSERT( pointCount == constraint->pointCount );

		for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
		{
			manifold->points[pointIndex].normalImpulse = constraint->points[pointIndex].normalImpulse;
			manifold->points[pointIndex].totalNormalImpulse = constraint->points[pointIndex].totalNormalImpulse;
			manifold->points[pointIndex].normalVelocity = constraint->points[pointIndex].relativeVelocity;
		}

		manifold->frictionImpulse = constraint->frictionImpulse.x * constraint->tangent1;
		manifold->frictionImpulse += constraint->frictionImpulse.y * constraint->tangent2;
		manifold->twistImpulse = constraint->twistImpulse;
		manifold->rollingImpulse = constraint->rollingImpulse;
	}

	b3TracyCZoneEnd( store_impulses );
}

#if defined( B3_SIMD_NEON )

#include <arm_neon.h>

// wide float holds 4 numbers
typedef float32x4_t b3FloatW;

#elif defined( B3_SIMD_SSE2 )

#include <emmintrin.h>

// wide float holds 4 numbers
typedef __m128 b3FloatW;

#else

// scalar math
struct b3FloatW
{
	float x, y, z, w;
};

#endif

// Wide vec2
struct b3Vec2W
{
	b3FloatW x, y;
};

// Wide vec3
struct b3Vec3W
{
	b3FloatW X, Y, Z;
};

// Wide quaternion
struct b3QuatW
{
	b3Vec3W V;
	b3FloatW S;
};

// Wide symmetric matrix2
struct b3SymMatrix2W
{
	b3FloatW cxx, cxy, cyy;
};

// Wide symmetric matrix3
struct b3SymMatrix3W
{
	b3FloatW cxx, cxy, cxz, cyy, cyz, czz;
};

#if defined( B3_SIMD_NEON )

static inline b3FloatW b3ZeroW()
{
	return vdupq_n_f32( 0.0f );
}

static inline b3FloatW b3SplatW( float scalar )
{
	return vdupq_n_f32( scalar );
}

static inline b3FloatW b3NegW( b3FloatW a )
{
	return vnegq_f32( a );
}

static inline b3FloatW b3SetW( float a, float b, float c, float d )
{
	float32_t array[4] = { a, b, c, d };
	return vld1q_f32( array );
}

static inline b3FloatW b3AddW( b3FloatW a, b3FloatW b )
{
	return vaddq_f32( a, b );
}

static inline b3FloatW b3SubW( b3FloatW a, b3FloatW b )
{
	return vsubq_f32( a, b );
}

static inline b3FloatW b3MulW( b3FloatW a, b3FloatW b )
{
	return vmulq_f32( a, b );
}

static inline b3FloatW b3DivW( b3FloatW a, b3FloatW b )
{
	return vdivq_f32( a, b );
}

static inline b3FloatW b3SqrtW( b3FloatW a )
{
	return vsqrtq_f32( a );
}

static inline b3FloatW b3MulAddW( b3FloatW a, b3FloatW b, b3FloatW c )
{
	return vmlaq_f32( a, b, c );
}

// static inline b3FloatW b3MulSubW( b3FloatW a, b3FloatW b, b3FloatW c )
//{
//	return vmlsq_f32( a, b, c );
// }

static inline b3FloatW b3MinW( b3FloatW a, b3FloatW b )
{
	return vminq_f32( a, b );
}

static inline b3FloatW b3MaxW( b3FloatW a, b3FloatW b )
{
	return vmaxq_f32( a, b );
}

// clamp a to [-b, b]
static inline b3FloatW b3SymClampW( b3FloatW a, b3FloatW b )
{
	b3FloatW nb = b3NegW( b );
	b3FloatW c = b3MaxW( nb, a );
	return b3MinW( c, b );
}

static inline b3FloatW b3OrW( b3FloatW a, b3FloatW b )
{
	return vreinterpretq_f32_u32( vorrq_u32( vreinterpretq_u32_f32( a ), vreinterpretq_u32_f32( b ) ) );
}

static inline b3FloatW b3GreaterThanW( b3FloatW a, b3FloatW b )
{
	return vreinterpretq_f32_u32( vcgtq_f32( a, b ) );
}

static inline b3FloatW b3EqualsW( b3FloatW a, b3FloatW b )
{
	return vreinterpretq_f32_u32( vceqq_f32( a, b ) );
}

static inline bool b3AllZeroW( b3FloatW a )
{
	// Create a zero vector for comparison
	b3FloatW zero = vdupq_n_f32( 0.0f );

	// Compare the input vector with zero
	uint32x4_t cmp_result = vceqq_f32( a, zero );

// Check if all comparison results are non-zero using vminvq
#ifdef __ARM_FEATURE_SVE
	// ARM v8.2+ has horizontal minimum instruction
	return vminvq_u32( cmp_result ) != 0;
#else
	// For older ARM architectures, we need to manually check all lanes
	return vgetq_lane_u32( cmp_result, 0 ) != 0 && vgetq_lane_u32( cmp_result, 1 ) != 0 && vgetq_lane_u32( cmp_result, 2 ) != 0 &&
		   vgetq_lane_u32( cmp_result, 3 ) != 0;
#endif
}

// component-wise returns mask ? b : a
static inline b3FloatW b3BlendW( b3FloatW a, b3FloatW b, b3FloatW mask )
{
	uint32x4_t mask32 = vreinterpretq_u32_f32( mask );
	return vbslq_f32( mask32, b, a );
}

#if 0
static inline b3FloatW b3LoadW( const float32_t* data )
{
	return vld1q_f32( data );
}

static inline void b3StoreW( float32_t* data, b3FloatW a )
{
	return vst1q_f32( data, a );
}

static inline b3FloatW b3UnpackLoW( b3FloatW a, b3FloatW b )
{
#if defined( __aarch64__ )
	return vzip1q_f32( a, b );
#else
	float32x2_t a1 = vget_low_f32( a );
	float32x2_t b1 = vget_low_f32( b );
	float32x2x2_t result = vzip_f32( a1, b1 );
	return vcombine_f32( result.val[0], result.val[1] );
#endif
}

static inline b3FloatW b3UnpackHiW( b3FloatW a, b3FloatW b )
{
#if defined( __aarch64__ )
	return vzip2q_f32( a, b );
#else
	float32x2_t a1 = vget_high_f32( a );
	float32x2_t b1 = vget_high_f32( b );
	float32x2x2_t result = vzip_f32( a1, b1 );
	return vcombine_f32( result.val[0], result.val[1] );
#endif
}
#endif

#elif defined( B3_SIMD_SSE2 )

static inline b3FloatW b3ZeroW()
{
	return _mm_setzero_ps();
}

static inline b3FloatW b3SplatW( float scalar )
{
	return _mm_set1_ps( scalar );
}

static inline b3FloatW b3NegW( b3FloatW a )
{
	// Create a mask with the sign bit set for each element
	__m128 mask = _mm_set1_ps( -0.0f );

	// XOR the input with the mask to negate each element
	return _mm_xor_ps( a, mask );
}

static inline b3FloatW b3SetW( float a, float b, float c, float d )
{
	return _mm_setr_ps( a, b, c, d );
}

static inline b3FloatW b3AddW( b3FloatW a, b3FloatW b )
{
	return _mm_add_ps( a, b );
}

static inline b3FloatW b3SubW( b3FloatW a, b3FloatW b )
{
	return _mm_sub_ps( a, b );
}

static inline b3FloatW b3MulW( b3FloatW a, b3FloatW b )
{
	return _mm_mul_ps( a, b );
}

static inline b3FloatW b3DivW( b3FloatW a, b3FloatW b )
{
	return _mm_div_ps( a, b );
}

static inline b3FloatW b3SqrtW( b3FloatW a )
{
	return _mm_sqrt_ps( a );
}

static inline b3FloatW b3MulAddW( b3FloatW a, b3FloatW b, b3FloatW c )
{
	return _mm_add_ps( a, _mm_mul_ps( b, c ) );
}

// static inline b3FloatW b3MulSubW( b3FloatW a, b3FloatW b, b3FloatW c )
//{
//	return _mm_sub_ps( a, _mm_mul_ps( b, c ) );
// }

static inline b3FloatW b3MinW( b3FloatW a, b3FloatW b )
{
	return _mm_min_ps( a, b );
}

static inline b3FloatW b3MaxW( b3FloatW a, b3FloatW b )
{
	return _mm_max_ps( a, b );
}

// clamp a to [-b, b]
static inline b3FloatW b3SymClampW( b3FloatW a, b3FloatW b )
{
	b3FloatW nb = b3NegW( b );
	b3FloatW c = b3MaxW( nb, a );
	return b3MinW( c, b );
}

static inline b3FloatW b3OrW( b3FloatW a, b3FloatW b )
{
	return _mm_or_ps( a, b );
}

static inline b3FloatW b3GreaterThanW( b3FloatW a, b3FloatW b )
{
	return _mm_cmpgt_ps( a, b );
}

static inline b3FloatW b3EqualsW( b3FloatW a, b3FloatW b )
{
	return _mm_cmpeq_ps( a, b );
}

static inline bool b3AllZeroW( b3FloatW a )
{
	// Compare each element with zero
	b3FloatW zero = _mm_setzero_ps();
	b3FloatW cmp = _mm_cmpeq_ps( a, zero );

	// Create a mask from the comparison results
	int mask = _mm_movemask_ps( cmp );

	// If all elements are zero, the mask will be 0xF (1111 in binary)
	return mask == 0xF;
}

// component-wise returns mask ? b : a
static inline b3FloatW b3BlendW( b3FloatW a, b3FloatW b, b3FloatW mask )
{
	return _mm_or_ps( _mm_and_ps( mask, b ), _mm_andnot_ps( mask, a ) );
}

// static inline b3FloatW b3LoadW( const float* data )
//{
//	return _mm_load_ps( data );
// }

// static inline void b3StoreW( float* data, b3FloatW a )
//{
//	_mm_store_ps( data, a );
// }

// static inline b3FloatW b3UnpackLoW( b3FloatW a, b3FloatW b )
//{
//	return _mm_unpacklo_ps( a, b );
// }

// static inline b3FloatW b3UnpackHiW( b3FloatW a, b3FloatW b )
//{
//	return _mm_unpackhi_ps( a, b );
// }

#else

static inline b3FloatW b3ZeroW()
{
	return { 0.0f, 0.0f, 0.0f, 0.0f };
}

static inline b3FloatW b3SplatW( float scalar )
{
	return { scalar, scalar, scalar, scalar };
}

static inline b3FloatW b3NegW( b3FloatW a )
{
	return { -a.x, -a.y, -a.z, -a.w };
}

static inline b3FloatW b3AddW( b3FloatW a, b3FloatW b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w };
}

static inline b3FloatW b3SubW( b3FloatW a, b3FloatW b )
{
	return { a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
}

static inline b3FloatW b3MulW( b3FloatW a, b3FloatW b )
{
	return { a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w };
}

static inline b3FloatW b3DivW( b3FloatW a, b3FloatW b )
{
	return { a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w };
}

static inline b3FloatW b3SqrtW( b3FloatW a )
{
	return { sqrtf( a.x ), sqrtf( a.y ), sqrtf( a.z ), sqrtf( a.w ) };
}

static inline b3FloatW b3MulAddW( b3FloatW a, b3FloatW b, b3FloatW c )
{
	return { a.x + b.x * c.x, a.y + b.y * c.y, a.z + b.z * c.z, a.w + b.w * c.w };
}

// static inline b3FloatW b3MulSubW( b3FloatW a, b3FloatW b, b3FloatW c )
//{
//	return { a.x - b.x * c.x, a.y - b.y * c.y, a.z - b.z * c.z, a.w - b.w * c.w };
// }

// static inline b3FloatW b3MinW( b3FloatW a, b3FloatW b )
//{
//	b3FloatW r;
//	r.x = a.x <= b.x ? a.x : b.x;
//	r.y = a.y <= b.y ? a.y : b.y;
//	r.z = a.z <= b.z ? a.z : b.z;
//	r.w = a.w <= b.w ? a.w : b.w;
//	return r;
// }

static inline b3FloatW b3MaxW( b3FloatW a, b3FloatW b )
{
	b3FloatW r;
	r.x = a.x >= b.x ? a.x : b.x;
	r.y = a.y >= b.y ? a.y : b.y;
	r.z = a.z >= b.z ? a.z : b.z;
	r.w = a.w >= b.w ? a.w : b.w;
	return r;
}

// clamp a to [-b, b]
static inline b3FloatW b3SymClampW( b3FloatW a, b3FloatW b )
{
	b3FloatW r;
	r.x = a.x <= b.x ? a.x : b.x;
	r.y = a.y <= b.y ? a.y : b.y;
	r.z = a.z <= b.z ? a.z : b.z;
	r.w = a.w <= b.w ? a.w : b.w;
	r.x = r.x <= -b.x ? -b.x : r.x;
	r.y = r.y <= -b.y ? -b.y : r.y;
	r.z = r.z <= -b.z ? -b.z : r.z;
	r.w = r.w <= -b.w ? -b.w : r.w;
	return r;
}

static inline b3FloatW b3OrW( b3FloatW a, b3FloatW b )
{
	b3FloatW r;
	r.x = a.x != 0.0f || b.x != 0.0f ? 1.0f : 0.0f;
	r.y = a.y != 0.0f || b.y != 0.0f ? 1.0f : 0.0f;
	r.z = a.z != 0.0f || b.z != 0.0f ? 1.0f : 0.0f;
	r.w = a.w != 0.0f || b.w != 0.0f ? 1.0f : 0.0f;
	return r;
}

static inline b3FloatW b3GreaterThanW( b3FloatW a, b3FloatW b )
{
	b3FloatW r;
	r.x = a.x > b.x ? 1.0f : 0.0f;
	r.y = a.y > b.y ? 1.0f : 0.0f;
	r.z = a.z > b.z ? 1.0f : 0.0f;
	r.w = a.w > b.w ? 1.0f : 0.0f;
	return r;
}

static inline b3FloatW b3EqualsW( b3FloatW a, b3FloatW b )
{
	b3FloatW r;
	r.x = a.x == b.x ? 1.0f : 0.0f;
	r.y = a.y == b.y ? 1.0f : 0.0f;
	r.z = a.z == b.z ? 1.0f : 0.0f;
	r.w = a.w == b.w ? 1.0f : 0.0f;
	return r;
}

static inline bool b3AllZeroW( b3FloatW a )
{
	return a.x == 0.0f && a.y == 0.0f && a.z == 0.0f && a.w == 0.0f;
}

// component-wise returns mask ? b : a
static inline b3FloatW b3BlendW( b3FloatW a, b3FloatW b, b3FloatW mask )
{
	b3FloatW r;
	r.x = mask.x != 0.0f ? b.x : a.x;
	r.y = mask.y != 0.0f ? b.y : a.y;
	r.z = mask.z != 0.0f ? b.z : a.z;
	r.w = mask.w != 0.0f ? b.w : a.w;
	return r;
}

#endif

// s * a
static inline b3Vec3W b3MulSVW( b3FloatW s, b3Vec3W a )
{
	return { b3MulW( s, a.X ), b3MulW( s, a.Y ), b3MulW( s, a.Z ) };
}

// a - s * b
static inline b3Vec3W b3MulSubSVW( b3Vec3W a, b3FloatW s, b3Vec3W b )
{
	return { b3SubW( a.X, b3MulW( s, b.X ) ), b3SubW( a.Y, b3MulW( s, b.Y ) ), b3SubW( a.Z, b3MulW( s, b.Z ) ) };
}

// a + s * b
static inline b3Vec3W b3MulAddSVW( b3Vec3W a, b3FloatW s, b3Vec3W b )
{
	return { b3AddW( a.X, b3MulW( s, b.X ) ), b3AddW( a.Y, b3MulW( s, b.Y ) ), b3AddW( a.Z, b3MulW( s, b.Z ) ) };
}

// a + b
static inline b3Vec2W b3AddV2W( b3Vec2W a, b3Vec2W b )
{
	return {
		b3AddW( a.x, b.x ),
		b3AddW( a.y, b.y ),
	};
}

// a - b
static inline b3Vec3W b3SubVW( b3Vec3W a, b3Vec3W b )
{
	return {
		b3SubW( a.X, b.X ),
		b3SubW( a.Y, b.Y ),
		b3SubW( a.Z, b.Z ),
	};
}

// a + b
static inline b3Vec3W b3AddVW( b3Vec3W a, b3Vec3W b )
{
	return {
		b3AddW( a.X, b.X ),
		b3AddW( a.Y, b.Y ),
		b3AddW( a.Z, b.Z ),
	};
}

// m * a
static inline b3Vec2W b3MulMV2W( b3SymMatrix2W m, b3Vec2W a )
{
	b3Vec2W b = {
		b3AddW( b3MulW( m.cxx, a.x ), b3MulW( m.cxy, a.y ) ),
		b3AddW( b3MulW( m.cxy, a.x ), b3MulW( m.cyy, a.y ) ),
	};

	return b;
}

// m * a
static inline b3Vec3W b3MulMVW( b3SymMatrix3W m, b3Vec3W a )
{
	b3Vec3W b = {
		b3AddW( b3MulW( m.cxx, a.X ), b3AddW( b3MulW( m.cxy, a.Y ), b3MulW( m.cxz, a.Z ) ) ),
		b3AddW( b3MulW( m.cxy, a.X ), b3AddW( b3MulW( m.cyy, a.Y ), b3MulW( m.cyz, a.Z ) ) ),
		b3AddW( b3MulW( m.cxz, a.X ), b3AddW( b3MulW( m.cyz, a.Y ), b3MulW( m.czz, a.Z ) ) ),
	};

	return b;
}

// a - m * b
static inline b3Vec3W b3MulSubMVW( b3Vec3W a, b3SymMatrix3W m, b3Vec3W b )
{
	b3Vec3W c = {
		b3AddW( b3MulW( m.cxx, b.X ), b3AddW( b3MulW( m.cxy, b.Y ), b3MulW( m.cxz, b.Z ) ) ),
		b3AddW( b3MulW( m.cxy, b.X ), b3AddW( b3MulW( m.cyy, b.Y ), b3MulW( m.cyz, b.Z ) ) ),
		b3AddW( b3MulW( m.cxz, b.X ), b3AddW( b3MulW( m.cyz, b.Y ), b3MulW( m.czz, b.Z ) ) ),
	};

	return { b3SubW( a.X, c.X ), b3SubW( a.Y, c.Y ), b3SubW( a.Z, c.Z ) };
}

// a + m * b
static inline b3Vec3W b3MulAddMVW( b3Vec3W a, b3SymMatrix3W m, b3Vec3W b )
{
	b3Vec3W c = {
		b3AddW( b3MulW( m.cxx, b.X ), b3AddW( b3MulW( m.cxy, b.Y ), b3MulW( m.cxz, b.Z ) ) ),
		b3AddW( b3MulW( m.cxy, b.X ), b3AddW( b3MulW( m.cyy, b.Y ), b3MulW( m.cyz, b.Z ) ) ),
		b3AddW( b3MulW( m.cxz, b.X ), b3AddW( b3MulW( m.cyz, b.Y ), b3MulW( m.czz, b.Z ) ) ),
	};

	return { b3AddW( a.X, c.X ), b3AddW( a.Y, c.Y ), b3AddW( a.Z, c.Z ) };
}

static inline b3FloatW b3DotW( b3Vec3W a, b3Vec3W b )
{
	return b3AddW( b3AddW( b3MulW( a.X, b.X ), b3MulW( a.Y, b.Y ) ), b3MulW( a.Z, b.Z ) );
}

static inline b3Vec3W b3CrossW( b3Vec3W a, b3Vec3W b )
{
	b3Vec3W c;
	c.X = b3SubW( b3MulW( a.Y, b.Z ), b3MulW( a.Z, b.Y ) );
	c.Y = b3SubW( b3MulW( a.Z, b.X ), b3MulW( a.X, b.Z ) );
	c.Z = b3SubW( b3MulW( a.X, b.Y ), b3MulW( a.Y, b.X ) );
	return c;
}

static inline b3Vec3W b3RotateVectorW( b3QuatW q, b3Vec3W a )
{
	b3Vec3W t1 = b3CrossW( q.V, a );
	b3Vec3W t2;
	t2.X = b3MulAddW( t1.X, q.S, a.X );
	t2.Y = b3MulAddW( t1.Y, q.S, a.Y );
	t2.Z = b3MulAddW( t1.Z, q.S, a.Z );
	b3Vec3W t3 = b3CrossW( q.V, t2 );
	b3FloatW two = b3SplatW( 2.0f );
	b3Vec3W b;
	b.X = b3MulAddW( a.X, two, t3.X );
	b.Y = b3MulAddW( a.Y, two, t3.Y );
	b.Z = b3MulAddW( a.Z, two, t3.Z );
	return b;
}

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

typedef struct b3ContactConstraintSIMD
{
	// todo make these start at 1 and 0 for null to allow zero initialize
	int indexBase1A[B3_SIMD_WIDTH];
	int indexBase1B[B3_SIMD_WIDTH];

	b3FloatW invMassA, invMassB;
	b3SymMatrix3W invIA, invIB;
	b3Vec3W normal;

	// todo test computing the tangents on the fly, at least tangent2
	b3Vec3W tangent1;
	b3Vec3W tangent2;

	b3Vec3W originA, originB;
	b3FloatW twistMass;
	b3FloatW twistImpulse;
	b3SymMatrix2W tangentMass;
	b3Vec2W frictionImpulse;
	b3SymMatrix3W rollingMass;
	b3Vec3W rollingImpulse;
	b3FloatW friction;
	b3FloatW rollingResistance;
	b3FloatW tangentVelocity1;
	b3FloatW tangentVelocity2;

	b3FloatW biasRate;
	b3FloatW massScale;
	b3FloatW impulseScale;

	b3Vec3W anchorAs[4], anchorBs[4];
	b3FloatW baseSeparations[4];
	b3FloatW normalImpulses[4];
	b3FloatW totalNormalImpulses[4];
	b3FloatW normalMasses[4];
	b3FloatW leverArms[4];

	// todo what if restitution was central?
	b3FloatW restitution;
	b3FloatW relativeVelocities[4];

	b3Manifold* manifolds[B3_SIMD_WIDTH];
} b3ContactConstraintSIMD;

int b3GetContactConstraintSIMDByteCount( void )
{
	return sizeof( b3ContactConstraintSIMD );
}

// wide version of b3BodyState
typedef struct b3BodyStateW
{
	b3Vec3W v;
	b3Vec3W w;
	b3Vec3W dp;
	b3QuatW dq;
} b3BodyStateW;

#if defined( B3_SIMD_SSE2 ) || defined( B3_SIMD_NEON )

static b3BodyStateW b3GatherBodies( const b3BodyState* states, int* indices )
{
	b3BodyState dummy = {};
	dummy.deltaRotation.s = 1.0f;

	// Indices are 0 for null
	b3BodyState b1 = indices[0] == 0 ? dummy : states[indices[0] - 1];
	b3BodyState b2 = indices[1] == 0 ? dummy : states[indices[1] - 1];
	b3BodyState b3 = indices[2] == 0 ? dummy : states[indices[2] - 1];
	b3BodyState b4 = indices[3] == 0 ? dummy : states[indices[3] - 1];

	b3BodyStateW s;
	s.v.X = b3SetW( b1.linearVelocity.x, b2.linearVelocity.x, b3.linearVelocity.x, b4.linearVelocity.x );
	s.v.Y = b3SetW( b1.linearVelocity.y, b2.linearVelocity.y, b3.linearVelocity.y, b4.linearVelocity.y );
	s.v.Z = b3SetW( b1.linearVelocity.z, b2.linearVelocity.z, b3.linearVelocity.z, b4.linearVelocity.z );

	s.w.X = b3SetW( b1.angularVelocity.x, b2.angularVelocity.x, b3.angularVelocity.x, b4.angularVelocity.x );
	s.w.Y = b3SetW( b1.angularVelocity.y, b2.angularVelocity.y, b3.angularVelocity.y, b4.angularVelocity.y );
	s.w.Z = b3SetW( b1.angularVelocity.z, b2.angularVelocity.z, b3.angularVelocity.z, b4.angularVelocity.z );

	s.dp.X = b3SetW( b1.deltaPosition.x, b2.deltaPosition.x, b3.deltaPosition.x, b4.deltaPosition.x );
	s.dp.Y = b3SetW( b1.deltaPosition.y, b2.deltaPosition.y, b3.deltaPosition.y, b4.deltaPosition.y );
	s.dp.Z = b3SetW( b1.deltaPosition.z, b2.deltaPosition.z, b3.deltaPosition.z, b4.deltaPosition.z );

	s.dq.V.X = b3SetW( b1.deltaRotation.v.x, b2.deltaRotation.v.x, b3.deltaRotation.v.x, b4.deltaRotation.v.x );
	s.dq.V.Y = b3SetW( b1.deltaRotation.v.y, b2.deltaRotation.v.y, b3.deltaRotation.v.y, b4.deltaRotation.v.y );
	s.dq.V.Z = b3SetW( b1.deltaRotation.v.z, b2.deltaRotation.v.z, b3.deltaRotation.v.z, b4.deltaRotation.v.z );
	s.dq.S = b3SetW( b1.deltaRotation.s, b2.deltaRotation.s, b3.deltaRotation.s, b4.deltaRotation.s );
	return s;
}

// This writes only the velocities back to the solver bodies
static void b3ScatterBodies( b3BodyState* states, int* indices, const b3BodyStateW* simdBody )
{
	const float* vx = (const float*)&simdBody->v.X;
	const float* vy = (const float*)&simdBody->v.Y;
	const float* vz = (const float*)&simdBody->v.Z;
	const float* wx = (const float*)&simdBody->w.X;
	const float* wy = (const float*)&simdBody->w.Y;
	const float* wz = (const float*)&simdBody->w.Z;

	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.

	// Warning: indices start at 1 with 0 indicating null

	if ( indices[0] != 0 && ( states[indices[0] - 1].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* s = states + ( indices[0] - 1 );

		b3Vec3 v = { vx[0], vy[0], vz[0] };
		b3Vec3 w = { wx[0], wy[0], wz[0] };

		uint32_t flags = s->flags;
		if ( flags & b3_allLocks )
		{
			v.x = ( flags & b3_lockLinearX ) ? 0.0f : v.x;
			v.y = ( flags & b3_lockLinearY ) ? 0.0f : v.y;
			v.z = ( flags & b3_lockLinearZ ) ? 0.0f : v.z;
			w.x = ( flags & b3_lockAngularX ) ? 0.0f : w.x;
			w.y = ( flags & b3_lockAngularY ) ? 0.0f : w.y;
			w.z = ( flags & b3_lockAngularZ ) ? 0.0f : w.z;
		}

		s->linearVelocity = v;
		s->angularVelocity = w;
	}

	if ( indices[1] != 0 && ( states[indices[1] - 1].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* s = states + ( indices[1] - 1 );

		b3Vec3 v = { vx[1], vy[1], vz[1] };
		b3Vec3 w = { wx[1], wy[1], wz[1] };

		uint32_t flags = s->flags;
		if ( flags & b3_allLocks )
		{
			v.x = ( flags & b3_lockLinearX ) ? 0.0f : v.x;
			v.y = ( flags & b3_lockLinearY ) ? 0.0f : v.y;
			v.z = ( flags & b3_lockLinearZ ) ? 0.0f : v.z;
			w.x = ( flags & b3_lockAngularX ) ? 0.0f : w.x;
			w.y = ( flags & b3_lockAngularY ) ? 0.0f : w.y;
			w.z = ( flags & b3_lockAngularZ ) ? 0.0f : w.z;
		}

		s->linearVelocity = v;
		s->angularVelocity = w;
	}

	if ( indices[2] != 0 && ( states[indices[2] - 1].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* s = states + ( indices[2] - 1 );

		b3Vec3 v = { vx[2], vy[2], vz[2] };
		b3Vec3 w = { wx[2], wy[2], wz[2] };

		uint32_t flags = s->flags;
		if ( flags & b3_allLocks )
		{
			v.x = ( flags & b3_lockLinearX ) ? 0.0f : v.x;
			v.y = ( flags & b3_lockLinearY ) ? 0.0f : v.y;
			v.z = ( flags & b3_lockLinearZ ) ? 0.0f : v.z;
			w.x = ( flags & b3_lockAngularX ) ? 0.0f : w.x;
			w.y = ( flags & b3_lockAngularY ) ? 0.0f : w.y;
			w.z = ( flags & b3_lockAngularZ ) ? 0.0f : w.z;
		}

		s->linearVelocity = v;
		s->angularVelocity = w;
	}

	if ( indices[3] != 0 && ( states[indices[3] - 1].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* s = states + ( indices[3] - 1 );

		b3Vec3 v = { vx[3], vy[3], vz[3] };
		b3Vec3 w = { wx[3], wy[3], wz[3] };

		uint32_t flags = s->flags;
		if ( flags & b3_allLocks )
		{
			v.x = ( flags & b3_lockLinearX ) ? 0.0f : v.x;
			v.y = ( flags & b3_lockLinearY ) ? 0.0f : v.y;
			v.z = ( flags & b3_lockLinearZ ) ? 0.0f : v.z;
			w.x = ( flags & b3_lockAngularX ) ? 0.0f : w.x;
			w.y = ( flags & b3_lockAngularY ) ? 0.0f : w.y;
			w.z = ( flags & b3_lockAngularZ ) ? 0.0f : w.z;
		}

		s->linearVelocity = v;
		s->angularVelocity = w;
	}
}

#else // non-simd

static b3BodyStateW b3GatherBodies( const b3BodyState* states, int* indices )
{
	b3BodyState identity = b3_identityBodyState;

	b3BodyState s1 = indices[0] == 0 ? identity : states[indices[0] - 1];
	b3BodyState s2 = indices[1] == 0 ? identity : states[indices[1] - 1];
	b3BodyState s3 = indices[2] == 0 ? identity : states[indices[2] - 1];
	b3BodyState s4 = indices[3] == 0 ? identity : states[indices[3] - 1];

	b3BodyStateW simdBody;
	simdBody.v.X = { s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x };
	simdBody.v.Y = { s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y };
	simdBody.v.Z = { s1.linearVelocity.z, s2.linearVelocity.z, s3.linearVelocity.z, s4.linearVelocity.z };
	simdBody.w.X = { s1.angularVelocity.x, s2.angularVelocity.x, s3.angularVelocity.x, s4.angularVelocity.x };
	simdBody.w.Y = { s1.angularVelocity.y, s2.angularVelocity.y, s3.angularVelocity.y, s4.angularVelocity.y };
	simdBody.w.Z = { s1.angularVelocity.z, s2.angularVelocity.z, s3.angularVelocity.z, s4.angularVelocity.z };
	simdBody.dp.X = { s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x };
	simdBody.dp.Y = { s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y };
	simdBody.dp.Z = { s1.deltaPosition.z, s2.deltaPosition.z, s3.deltaPosition.z, s4.deltaPosition.z };
	simdBody.dq.V.X = { s1.deltaRotation.v.x, s2.deltaRotation.v.x, s3.deltaRotation.v.x, s4.deltaRotation.v.x };
	simdBody.dq.V.Y = { s1.deltaRotation.v.y, s2.deltaRotation.v.y, s3.deltaRotation.v.y, s4.deltaRotation.v.y };
	simdBody.dq.V.Z = { s1.deltaRotation.v.z, s2.deltaRotation.v.z, s3.deltaRotation.v.z, s4.deltaRotation.v.z };
	simdBody.dq.S = { s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s };

	return simdBody;
}

// This writes only the velocities back to the solver bodies
static void b3ScatterBodies( b3BodyState* states, int* indices, const b3BodyStateW* simdBody )
{
	int index1 = indices[0] - 1;
	if ( index1 != -1 && ( states[index1].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index1;
		state->linearVelocity.x = simdBody->v.X.x;
		state->linearVelocity.y = simdBody->v.Y.x;
		state->linearVelocity.z = simdBody->v.Z.x;
		state->angularVelocity.x = simdBody->w.X.x;
		state->angularVelocity.y = simdBody->w.Y.x;
		state->angularVelocity.z = simdBody->w.Z.x;
	}

	int index2 = indices[1] - 1;
	if ( index2 != -1 && ( states[index2].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index2;
		state->linearVelocity.x = simdBody->v.X.y;
		state->linearVelocity.y = simdBody->v.Y.y;
		state->linearVelocity.z = simdBody->v.Z.y;
		state->angularVelocity.x = simdBody->w.X.y;
		state->angularVelocity.y = simdBody->w.Y.y;
		state->angularVelocity.z = simdBody->w.Z.y;
	}

	int index3 = indices[2] - 1;
	if ( index3 != -1 && ( states[index3].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index3;
		state->linearVelocity.x = simdBody->v.X.z;
		state->linearVelocity.y = simdBody->v.Y.z;
		state->linearVelocity.z = simdBody->v.Z.z;
		state->angularVelocity.x = simdBody->w.X.z;
		state->angularVelocity.y = simdBody->w.Y.z;
		state->angularVelocity.z = simdBody->w.Z.z;
	}

	int index4 = indices[3] - 1;
	if ( index4 != -1 && ( states[index4].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index4;
		state->linearVelocity.x = simdBody->v.X.w;
		state->linearVelocity.y = simdBody->v.Y.w;
		state->linearVelocity.z = simdBody->v.Z.w;
		state->angularVelocity.x = simdBody->w.X.w;
		state->angularVelocity.y = simdBody->w.Y.w;
		state->angularVelocity.z = simdBody->w.Z.w;
	}
}

#endif

// Prepare contact constraints
// Handles multiple manifolds per contact, but needs a lookup from constraint lane to manifold
void b3PrepareContactsTask( int startIndex, int endIndex, b3StepContext* context, int manifoldCount )
{
	b3TracyCZoneNC( prepare_contact, "Prepare Contact", b3_colorYellow, true );

	b3World* world = context->world;
	b3ManifoldLookup* manifoldLookup = context->manifoldLookups[manifoldCount - 1];
	b3ContactConstraintSIMD* constraints = context->contactConstraints[manifoldCount - 1];

	b3BodyState* awakeStates = context->states;

#if B3_ENABLE_VALIDATION
	b3Body* bodies = world->bodies.data;
#endif

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	b3Softness contactSoftness = context->contactSoftness;
	b3Softness staticSoftness = context->staticSoftness;

	float warmStartScale = world->enableWarmStarting ? 1.0f : 0.0f;

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		b3ContactConstraintSIMD* constraintBase = constraints + manifoldCount * constraintIndex;

		for ( int laneIndex = 0; laneIndex < B3_SIMD_WIDTH; ++laneIndex )
		{
			int lookupIndex = B3_SIMD_WIDTH * constraintIndex + laneIndex;
			b3ContactSim* contactSim = manifoldLookup[lookupIndex].contactSim;

			if ( contactSim != nullptr )
			{
				int indexA = contactSim->bodySimIndexA;
				int indexB = contactSim->bodySimIndexB;

#if B3_ENABLE_VALIDATION
				B3_ASSERT( contactSim->manifoldCount == manifoldCount );

				b3Body* bodyA = bodies + contactSim->bodyIdA;
				int validIndexA = bodyA->setIndex == b3_awakeSet ? bodyA->localIndex : B3_NULL_INDEX;
				b3Body* bodyB = bodies + contactSim->bodyIdB;
				int validIndexB = bodyB->setIndex == b3_awakeSet ? bodyB->localIndex : B3_NULL_INDEX;

				B3_ASSERT( indexA == validIndexA );
				B3_ASSERT( indexB == validIndexB );
#endif

				for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
				{
					b3ContactConstraintSIMD* constraint = constraintBase + manifoldIndex;

					constraint->indexBase1A[laneIndex] = indexA + 1;
					constraint->indexBase1B[laneIndex] = indexB + 1;

					b3Manifold* manifold = contactSim->manifolds + manifoldIndex;
					constraint->manifolds[laneIndex] = manifold;

					b3Vec3 vA = b3Vec3_zero;
					b3Vec3 wA = b3Vec3_zero;
					float mA = contactSim->invMassA;
					b3Matrix3 iA = contactSim->invIA;
					if ( indexA != B3_NULL_INDEX )
					{
						b3BodyState* stateA = awakeStates + indexA;
						vA = stateA->linearVelocity;
						wA = stateA->angularVelocity;
					}

					b3Vec3 vB = b3Vec3_zero;
					b3Vec3 wB = b3Vec3_zero;
					float mB = contactSim->invMassB;
					b3Matrix3 iB = contactSim->invIB;
					if ( indexB != B3_NULL_INDEX )
					{
						b3BodyState* stateB = awakeStates + indexB;
						vB = stateB->linearVelocity;
						wB = stateB->angularVelocity;
					}

					( (float*)&constraint->invMassA )[laneIndex] = mA;
					( (float*)&constraint->invMassB )[laneIndex] = mB;

					( (float*)&constraint->invIA.cxx )[laneIndex] = iA.cx.x;
					( (float*)&constraint->invIA.cxy )[laneIndex] = iA.cx.y;
					( (float*)&constraint->invIA.cxz )[laneIndex] = iA.cx.z;
					( (float*)&constraint->invIA.cyy )[laneIndex] = iA.cy.y;
					( (float*)&constraint->invIA.cyz )[laneIndex] = iA.cy.z;
					( (float*)&constraint->invIA.czz )[laneIndex] = iA.cz.z;

					( (float*)&constraint->invIB.cxx )[laneIndex] = iB.cx.x;
					( (float*)&constraint->invIB.cxy )[laneIndex] = iB.cx.y;
					( (float*)&constraint->invIB.cxz )[laneIndex] = iB.cx.z;
					( (float*)&constraint->invIB.cyy )[laneIndex] = iB.cy.y;
					( (float*)&constraint->invIB.cyz )[laneIndex] = iB.cy.z;
					( (float*)&constraint->invIB.czz )[laneIndex] = iB.cz.z;

					b3Softness soft = ( indexA == B3_NULL_INDEX || indexB == B3_NULL_INDEX ) ? staticSoftness : contactSoftness;

					b3Vec3 normal = manifold->normal;
					( (float*)&constraint->normal.X )[laneIndex] = normal.x;
					( (float*)&constraint->normal.Y )[laneIndex] = normal.y;
					( (float*)&constraint->normal.Z )[laneIndex] = normal.z;

					b3Vec3 tangent1 = b3Perp( normal );
					( (float*)&constraint->tangent1.X )[laneIndex] = tangent1.x;
					( (float*)&constraint->tangent1.Y )[laneIndex] = tangent1.y;
					( (float*)&constraint->tangent1.Z )[laneIndex] = tangent1.z;

					b3Vec3 tangent2 = b3Cross( tangent1, normal );
					( (float*)&constraint->tangent2.X )[laneIndex] = tangent2.x;
					( (float*)&constraint->tangent2.Y )[laneIndex] = tangent2.y;
					( (float*)&constraint->tangent2.Z )[laneIndex] = tangent2.z;

					( (float*)&constraint->friction )[laneIndex] = contactSim->friction;
					( (float*)&constraint->restitution )[laneIndex] = contactSim->restitution;
					( (float*)&constraint->rollingResistance )[laneIndex] = contactSim->rollingResistance;

					( (float*)&constraint->tangentVelocity1 )[laneIndex] = b3Dot( contactSim->tangentVelocity, tangent1 );
					( (float*)&constraint->tangentVelocity2 )[laneIndex] = b3Dot( contactSim->tangentVelocity, tangent2 );

					( (float*)&constraint->biasRate )[laneIndex] = soft.biasRate;
					( (float*)&constraint->massScale )[laneIndex] = soft.massScale;
					( (float*)&constraint->impulseScale )[laneIndex] = soft.impulseScale;

					int pointCount = manifold->pointCount;
					b3Vec3 originA = b3Vec3_zero;
					b3Vec3 originB = b3Vec3_zero;

					for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
					{
						const b3ManifoldPoint* mp = manifold->points + pointIndex;

						b3Vec3 rA = mp->anchorA;
						b3Vec3 rB = mp->anchorB;
						originA += rA;
						originB += rB;

						( (float*)&constraint->anchorAs[pointIndex].X )[laneIndex] = rA.x;
						( (float*)&constraint->anchorAs[pointIndex].Y )[laneIndex] = rA.y;
						( (float*)&constraint->anchorAs[pointIndex].Z )[laneIndex] = rA.z;

						( (float*)&constraint->anchorBs[pointIndex].X )[laneIndex] = rB.x;
						( (float*)&constraint->anchorBs[pointIndex].Y )[laneIndex] = rB.y;
						( (float*)&constraint->anchorBs[pointIndex].Z )[laneIndex] = rB.z;

						float baseSeparation = mp->separation - b3Dot( b3Sub( rB, rA ), normal );
						( (float*)&constraint->baseSeparations[pointIndex] )[laneIndex] = baseSeparation;

						( (float*)&constraint->normalImpulses[pointIndex] )[laneIndex] = warmStartScale * mp->normalImpulse;
						( (float*)&constraint->totalNormalImpulses[pointIndex] )[laneIndex] = 0.0f;

						b3Vec3 rnA = b3Cross( rA, normal );
						b3Vec3 rnB = b3Cross( rB, normal );
						float kNormal = mA + mB + b3Dot( rnA, b3MulMV( iA, rnA ) ) + b3Dot( rnB, b3MulMV( iB, rnB ) );
						( (float*)&constraint->normalMasses[pointIndex] )[laneIndex] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

						// Save relative velocity for restitution
						b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
						b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
						( (float*)&constraint->relativeVelocities[pointIndex] )[laneIndex] = b3Dot( normal, b3Sub( vrB, vrA ) );
					}

					float invCount = 1.0f / pointCount;
					originA *= invCount;
					originB *= invCount;

					( (float*)&constraint->originA.X )[laneIndex] = originA.x;
					( (float*)&constraint->originA.Y )[laneIndex] = originA.y;
					( (float*)&constraint->originA.Z )[laneIndex] = originA.z;
					( (float*)&constraint->originB.X )[laneIndex] = originB.x;
					( (float*)&constraint->originB.Y )[laneIndex] = originB.y;
					( (float*)&constraint->originB.Z )[laneIndex] = originB.z;

					for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
					{
						const b3ManifoldPoint* mp = manifold->points + pointIndex;
						( (float*)&constraint->leverArms[pointIndex] )[laneIndex] = b3Distance( mp->anchorA, originA );
					}

					b3Vec3 rtA1 = b3Cross( originA, tangent1 );
					b3Vec3 rtA2 = b3Cross( originA, tangent2 );

					b3Vec3 rtB1 = b3Cross( originB, tangent1 );
					b3Vec3 rtB2 = b3Cross( originB, tangent2 );

					{
						b3Matrix2 k;
						k.cx.x = mA + mB + b3Dot( rtA1, b3MulMV( iA, rtA1 ) ) + b3Dot( rtB1, b3MulMV( iB, rtB1 ) );
						k.cy.y = mA + mB + b3Dot( rtA2, b3MulMV( iA, rtA2 ) ) + b3Dot( rtB2, b3MulMV( iB, rtB2 ) );
						k.cx.y = k.cy.x = b3Dot( rtA1, b3MulMV( iA, rtA2 ) ) + b3Dot( rtB1, b3MulMV( iB, rtB2 ) );
						b3Matrix2 tangentMass = b3Invert2( k );

						( (float*)&constraint->tangentMass.cxx )[laneIndex] = tangentMass.cx.x;
						( (float*)&constraint->tangentMass.cxy )[laneIndex] = tangentMass.cx.y;
						( (float*)&constraint->tangentMass.cyy )[laneIndex] = tangentMass.cy.y;

						( (float*)&constraint->frictionImpulse.x )[laneIndex] =
							warmStartScale * b3Dot( manifold->frictionImpulse, tangent1 );
						( (float*)&constraint->frictionImpulse.y )[laneIndex] =
							warmStartScale * b3Dot( manifold->frictionImpulse, tangent2 );
					}

					{
						float k = b3Dot( normal, b3MulMV( iA + iB, normal ) );
						( (float*)&constraint->twistMass )[laneIndex] = k > 0.0f ? 1.0f / k : 0.0f;
						( (float*)&constraint->twistImpulse )[laneIndex] = warmStartScale * manifold->twistImpulse;
					}

					{
						b3Matrix3 rollingMass = b3InvertMatrix( iA + iB );

						( (float*)&constraint->rollingMass.cxx )[laneIndex] = rollingMass.cx.x;
						( (float*)&constraint->rollingMass.cxy )[laneIndex] = rollingMass.cx.y;
						( (float*)&constraint->rollingMass.cxz )[laneIndex] = rollingMass.cx.z;
						( (float*)&constraint->rollingMass.cyy )[laneIndex] = rollingMass.cy.y;
						( (float*)&constraint->rollingMass.cyz )[laneIndex] = rollingMass.cy.z;
						( (float*)&constraint->rollingMass.czz )[laneIndex] = rollingMass.cz.z;

						( (float*)&constraint->rollingImpulse.X )[laneIndex] = warmStartScale * manifold->rollingImpulse.x;
						( (float*)&constraint->rollingImpulse.Y )[laneIndex] = warmStartScale * manifold->rollingImpulse.y;
						( (float*)&constraint->rollingImpulse.Z )[laneIndex] = warmStartScale * manifold->rollingImpulse.z;
					}

					// zero remaining points
					for ( int pointIndex = pointCount; pointIndex < 4; ++pointIndex )
					{
						( (float*)&constraint->anchorAs[pointIndex].X )[laneIndex] = 0.0f;
						( (float*)&constraint->anchorAs[pointIndex].Y )[laneIndex] = 0.0f;
						( (float*)&constraint->anchorAs[pointIndex].Z )[laneIndex] = 0.0f;
						( (float*)&constraint->anchorBs[pointIndex].X )[laneIndex] = 0.0f;
						( (float*)&constraint->anchorBs[pointIndex].Y )[laneIndex] = 0.0f;
						( (float*)&constraint->anchorBs[pointIndex].Z )[laneIndex] = 0.0f;
						( (float*)&constraint->baseSeparations[pointIndex] )[laneIndex] = 0.0f;
						( (float*)&constraint->normalImpulses[pointIndex] )[laneIndex] = 0.0f;
						( (float*)&constraint->totalNormalImpulses[pointIndex] )[laneIndex] = 0.0f;
						( (float*)&constraint->normalMasses[pointIndex] )[laneIndex] = 0.0f;
						( (float*)&constraint->relativeVelocities[pointIndex] )[laneIndex] = 0.0f;
						( (float*)&constraint->leverArms[pointIndex] )[laneIndex] = 0.0f;
					}
				}
			}
			else
			{
				// todo test using zero init, may need to use indexA/B as 0 meaning null
				for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
				{
					b3ContactConstraintSIMD* constraint = constraintBase + manifoldIndex;

					// SIMD remainder
					constraint->indexBase1A[laneIndex] = 0;
					constraint->indexBase1B[laneIndex] = 0;

					( (float*)&constraint->invMassA )[laneIndex] = 0.0f;
					( (float*)&constraint->invMassB )[laneIndex] = 0.0f;
					( (float*)&constraint->invIA.cxx )[laneIndex] = 0.0f;
					( (float*)&constraint->invIA.cxy )[laneIndex] = 0.0f;
					( (float*)&constraint->invIA.cxz )[laneIndex] = 0.0f;
					( (float*)&constraint->invIA.cyy )[laneIndex] = 0.0f;
					( (float*)&constraint->invIA.cyz )[laneIndex] = 0.0f;
					( (float*)&constraint->invIA.czz )[laneIndex] = 0.0f;

					( (float*)&constraint->invIB.cxx )[laneIndex] = 0.0f;
					( (float*)&constraint->invIB.cxy )[laneIndex] = 0.0f;
					( (float*)&constraint->invIB.cxz )[laneIndex] = 0.0f;
					( (float*)&constraint->invIB.cyy )[laneIndex] = 0.0f;
					( (float*)&constraint->invIB.cyz )[laneIndex] = 0.0f;
					( (float*)&constraint->invIB.czz )[laneIndex] = 0.0f;

					( (float*)&constraint->normal.X )[laneIndex] = 0.0f;
					( (float*)&constraint->normal.Y )[laneIndex] = 0.0f;
					( (float*)&constraint->normal.Z )[laneIndex] = 0.0f;
					( (float*)&constraint->tangent1.X )[laneIndex] = 0.0f;
					( (float*)&constraint->tangent1.Y )[laneIndex] = 0.0f;
					( (float*)&constraint->tangent1.Z )[laneIndex] = 0.0f;
					( (float*)&constraint->tangent2.X )[laneIndex] = 0.0f;
					( (float*)&constraint->tangent2.Y )[laneIndex] = 0.0f;
					( (float*)&constraint->tangent2.Z )[laneIndex] = 0.0f;
					( (float*)&constraint->friction )[laneIndex] = 0.0f;
					( (float*)&constraint->restitution )[laneIndex] = 0.0f;
					( (float*)&constraint->rollingResistance )[laneIndex] = 0.0f;
					( (float*)&constraint->tangentVelocity1 )[laneIndex] = 0.0f;
					( (float*)&constraint->tangentVelocity2 )[laneIndex] = 0.0f;
					( (float*)&constraint->biasRate )[laneIndex] = 0.0f;
					( (float*)&constraint->massScale )[laneIndex] = 0.0f;
					( (float*)&constraint->impulseScale )[laneIndex] = 0.0f;

					for ( int pointIndex = 0; pointIndex < 4; ++pointIndex )
					{
						( (float*)&constraint->anchorAs[pointIndex].X )[laneIndex] = 0.0f;
						( (float*)&constraint->anchorAs[pointIndex].Y )[laneIndex] = 0.0f;
						( (float*)&constraint->anchorAs[pointIndex].Z )[laneIndex] = 0.0f;
						( (float*)&constraint->anchorBs[pointIndex].X )[laneIndex] = 0.0f;
						( (float*)&constraint->anchorBs[pointIndex].Y )[laneIndex] = 0.0f;
						( (float*)&constraint->anchorBs[pointIndex].Z )[laneIndex] = 0.0f;
						( (float*)&constraint->baseSeparations[pointIndex] )[laneIndex] = 0.0f;
						( (float*)&constraint->normalImpulses[pointIndex] )[laneIndex] = 0.0f;
						( (float*)&constraint->totalNormalImpulses[pointIndex] )[laneIndex] = 0.0f;
						( (float*)&constraint->normalMasses[pointIndex] )[laneIndex] = 0.0f;
						( (float*)&constraint->relativeVelocities[pointIndex] )[laneIndex] = 0.0f;
						( (float*)&constraint->leverArms[pointIndex] )[laneIndex] = 0.0f;
					}

					( (float*)&constraint->originA.X )[laneIndex] = 0.0f;
					( (float*)&constraint->originA.Y )[laneIndex] = 0.0f;
					( (float*)&constraint->originA.Z )[laneIndex] = 0.0f;
					( (float*)&constraint->originB.X )[laneIndex] = 0.0f;
					( (float*)&constraint->originB.Y )[laneIndex] = 0.0f;
					( (float*)&constraint->originB.Z )[laneIndex] = 0.0f;
					( (float*)&constraint->tangentMass.cxx )[laneIndex] = 0.0f;
					( (float*)&constraint->tangentMass.cxy )[laneIndex] = 0.0f;
					( (float*)&constraint->tangentMass.cyy )[laneIndex] = 0.0f;
					( (float*)&constraint->frictionImpulse.x )[laneIndex] = 0.0f;
					( (float*)&constraint->frictionImpulse.y )[laneIndex] = 0.0f;
					( (float*)&constraint->twistMass )[laneIndex] = 0.0f;
					( (float*)&constraint->twistImpulse )[laneIndex] = 0.0f;
					( (float*)&constraint->rollingMass.cxx )[laneIndex] = 0.0f;
					( (float*)&constraint->rollingMass.cxy )[laneIndex] = 0.0f;
					( (float*)&constraint->rollingMass.cxz )[laneIndex] = 0.0f;
					( (float*)&constraint->rollingMass.cyy )[laneIndex] = 0.0f;
					( (float*)&constraint->rollingMass.cyz )[laneIndex] = 0.0f;
					( (float*)&constraint->rollingMass.czz )[laneIndex] = 0.0f;
					( (float*)&constraint->rollingImpulse.X )[laneIndex] = 0.0f;
					( (float*)&constraint->rollingImpulse.Y )[laneIndex] = 0.0f;
					( (float*)&constraint->rollingImpulse.Z )[laneIndex] = 0.0f;

					constraint->manifolds[laneIndex] = nullptr;
				}
			}
		}
	}
	b3TracyCZoneEnd( prepare_contact );
}

void b3WarmStartManifoldTask( int startIndex, int endIndex, b3StepContext* context, int colorIndex, int manifoldCount )
{
	b3TracyCZoneNC( warm_start_contact, "Warm Start", b3_colorGreen, true );

	b3BodyState* states = context->states;
	b3ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].simdConstraints[manifoldCount - 1];

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		// Get base constraint according to stride which is the manifoldCount
		b3ContactConstraintSIMD* base = constraints + manifoldCount * constraintIndex;

		// Single gather for all manifolds
		b3BodyStateW bA = b3GatherBodies( states, base->indexBase1A );
		b3BodyStateW bB = b3GatherBodies( states, base->indexBase1B );

		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ContactConstraintSIMD* c = base + manifoldIndex;

#if B3_ENABLE_VALIDATION
			for ( int i = 0; i < B3_SIMD_WIDTH; ++i )
			{
				B3_ASSERT( c->indexBase1A[i] == base->indexBase1A[i] );
				B3_ASSERT( c->indexBase1B[i] == base->indexBase1B[i] );
			}
#endif

			// Normal impulses
			for ( int pointIndex = 0; pointIndex < 4; ++pointIndex )
			{
				b3Vec3W rA = c->anchorAs[pointIndex];
				b3Vec3W rB = c->anchorBs[pointIndex];

				b3Vec3W impulse;
				impulse.X = b3MulW( c->normalImpulses[pointIndex], c->normal.X );
				impulse.Y = b3MulW( c->normalImpulses[pointIndex], c->normal.Y );
				impulse.Z = b3MulW( c->normalImpulses[pointIndex], c->normal.Z );

				bA.w = b3MulSubMVW( bA.w, c->invIA, b3CrossW( rA, impulse ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, impulse );
				bB.w = b3MulAddMVW( bB.w, c->invIB, b3CrossW( rB, impulse ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, impulse );
			}

			// Central friction
			{
				b3Vec3W rA = c->originA;
				b3Vec3W rB = c->originB;
				b3Vec3W impulse = b3MulSVW( c->frictionImpulse.x, c->tangent1 );
				impulse = b3MulAddSVW( impulse, c->frictionImpulse.y, c->tangent2 );

				bA.w = b3MulSubMVW( bA.w, c->invIA, b3CrossW( rA, impulse ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, impulse );
				bB.w = b3MulAddMVW( bB.w, c->invIB, b3CrossW( rB, impulse ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, impulse );
			}

			// Central twist friction
			{
				b3Vec3W impulse = b3MulSVW( c->twistImpulse, c->normal );
				bA.w = b3MulSubMVW( bA.w, c->invIA, impulse );
				bB.w = b3MulAddMVW( bB.w, c->invIB, impulse );
			}

			// Rolling resistance
			{
				b3Vec3W impulse = c->rollingImpulse;
				bA.w = b3MulSubMVW( bA.w, c->invIA, impulse );
				bB.w = b3MulAddMVW( bB.w, c->invIB, impulse );
			}
		}

		b3ScatterBodies( states, base->indexBase1A, &bA );
		b3ScatterBodies( states, base->indexBase1B, &bB );
	}

	b3TracyCZoneEnd( warm_start_contact );
}

void b3SolveManifoldTask( int startIndex, int endIndex, b3StepContext* context, int colorIndex, int manifoldCount, bool useBias )
{
	b3TracyCZoneNC( solve_contact, "Solve Contact", b3_colorAliceBlue, true );

	b3BodyState* states = context->states;
	b3ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].simdConstraints[manifoldCount - 1];
	b3FloatW inv_h = b3SplatW( context->inv_h );
	b3FloatW contactSpeed = b3SplatW( -context->world->contactSpeed );
	b3FloatW oneW = b3SplatW( 1.0f );
	b3FloatW epsilonW = b3SplatW( FLT_EPSILON );

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		// Get base constraint according to stride which is the manifoldCount
		b3ContactConstraintSIMD* base = constraints + manifoldCount * constraintIndex;

		// Single gather for all manifolds
		b3BodyStateW bA = b3GatherBodies( states, base->indexBase1A );
		b3BodyStateW bB = b3GatherBodies( states, base->indexBase1B );

		b3FloatW biasRate, massScale, impulseScale;
		if ( useBias )
		{
			biasRate = b3MulW( base->massScale, base->biasRate );
			massScale = base->massScale;
			impulseScale = base->impulseScale;
		}
		else
		{
			biasRate = b3ZeroW();
			massScale = oneW;
			impulseScale = b3ZeroW();
		}

		b3Vec3W dp = b3SubVW( bB.dp, bA.dp );

		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ContactConstraintSIMD* c = base + manifoldIndex;

			b3FloatW totalNormalImpulse = b3ZeroW();
			b3FloatW totalTwistLimit = b3ZeroW();

			// todo_erin use the max point count of the four manifolds
			for ( int pointIndex = 0; pointIndex < 4; ++pointIndex )
			{
				// Fixed anchor points for applying impulses
				b3Vec3W rA = c->anchorAs[pointIndex];
				b3Vec3W rB = c->anchorBs[pointIndex];

				// Moving anchors for current separation
				// todo speed this up using matrices
				b3Vec3W rsA = b3RotateVectorW( bA.dq, rA );
				b3Vec3W rsB = b3RotateVectorW( bB.dq, rB );

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				b3Vec3W ds = b3AddVW( dp, b3SubVW( rsB, rsA ) );
				b3FloatW s = b3AddW( b3DotW( c->normal, ds ), c->baseSeparations[pointIndex] );

				// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
				b3FloatW mask = b3GreaterThanW( s, b3ZeroW() );
				b3FloatW specBias = b3MulW( s, inv_h );
				b3FloatW softBias = b3MaxW( b3MulW( biasRate, s ), contactSpeed );
				b3FloatW bias = b3BlendW( softBias, specBias, mask );

				b3FloatW pointMassScale = b3BlendW( massScale, oneW, mask );
				b3FloatW pointImpulseScale = b3BlendW( impulseScale, b3ZeroW(), mask );

				// Relative velocity at contact
				b3Vec3W vrA = b3AddVW( bA.v, b3CrossW( bA.w, rA ) );
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossW( bB.w, rB ) );
				b3FloatW vn = b3DotW( b3SubVW( vrB, vrA ), c->normal );

				// Compute normal impulse
				b3FloatW negImpulse = b3AddW( b3MulW( c->normalMasses[pointIndex], b3AddW( b3MulW( pointMassScale, vn ), bias ) ),
											  b3MulW( pointImpulseScale, c->normalImpulses[pointIndex] ) );

				// Clamp the accumulated impulse
				b3FloatW newImpulse = b3MaxW( b3SubW( c->normalImpulses[pointIndex], negImpulse ), b3ZeroW() );
				b3FloatW deltaImpulse = b3SubW( newImpulse, c->normalImpulses[pointIndex] );
				c->normalImpulses[pointIndex] = newImpulse;
				c->totalNormalImpulses[pointIndex] = b3AddW( c->totalNormalImpulses[pointIndex], newImpulse );

				totalNormalImpulse = b3AddW( totalNormalImpulse, newImpulse );
				totalTwistLimit = b3AddW( totalTwistLimit, b3MulW( c->leverArms[pointIndex], newImpulse ) );

				// Apply contact impulse
				b3Vec3W P = b3MulSVW( deltaImpulse, c->normal );
				bA.w = b3MulSubMVW( bA.w, c->invIA, b3CrossW( rA, P ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bB.w = b3MulAddMVW( bB.w, c->invIB, b3CrossW( rB, P ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
			}

			// Central twist friction
			{
				b3FloatW twistSpeed = b3DotW( c->normal, b3SubVW( bB.w, bA.w ) );
				b3FloatW maxLambda = b3MulW( c->friction, totalTwistLimit );
				b3FloatW deltaImpulse = b3NegW( b3MulW( c->twistMass, twistSpeed ) );
				b3FloatW oldImpulse = c->twistImpulse;
				c->twistImpulse = b3SymClampW( b3AddW( oldImpulse, deltaImpulse ), maxLambda );
				deltaImpulse = b3SubW( c->twistImpulse, oldImpulse );

				b3Vec3W L = b3MulSVW( deltaImpulse, c->normal );
				bA.w = b3MulSubMVW( bA.w, c->invIA, L );
				bB.w = b3MulAddMVW( bB.w, c->invIB, L );
			}

			// Rolling resistance
			if (b3AllZeroW(c->rollingResistance) == false)
			{
				// flip A/B order to negate
				b3Vec3W deltaImpulse = b3MulMVW( c->rollingMass, b3SubVW( bA.w, bB.w ) );
				b3Vec3W oldImpulse = c->rollingImpulse;
				c->rollingImpulse = b3AddVW( oldImpulse, deltaImpulse );

				b3FloatW maxImpulse = b3MulW( c->rollingResistance, totalNormalImpulse );
				b3FloatW lengthSquared = b3DotW( c->rollingImpulse, c->rollingImpulse );

				// if ( magSqr > maxLambda * maxLambda + FLT_EPSILON )
				//{
				//	c->rollingImpulse *= maxLambda / sqrtf( magSqr );
				// }

				b3FloatW mask = b3GreaterThanW( lengthSquared, b3MulAddW( epsilonW, maxImpulse, maxImpulse ) );

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism
				b3FloatW normalize = b3DivW( maxImpulse, b3AddW( b3SqrtW( lengthSquared ), epsilonW ) );
				b3FloatW scale = b3BlendW( oneW, normalize, mask );

				// Ensure zero rolling resistance yields no impulse
				b3FloatW rollingMask = b3GreaterThanW( c->rollingResistance, b3ZeroW() );
				scale = b3BlendW( b3ZeroW(), scale, rollingMask );

				c->rollingImpulse = b3MulSVW( scale, c->rollingImpulse );

				deltaImpulse = b3SubVW( c->rollingImpulse, oldImpulse );

				bA.w = b3MulSubMVW( bA.w, c->invIA, deltaImpulse );
				bB.w = b3MulAddMVW( bB.w, c->invIB, deltaImpulse );
			}

			// Central friction
			{
				b3Vec3W tangent1 = c->tangent1;
				b3Vec3W tangent2 = c->tangent2;

				// Fixed anchor points for applying impulses
				b3Vec3W rA = c->originA;
				b3Vec3W rB = c->originB;

				// Relative tangent velocity at contact
				b3Vec3W vrA = b3AddVW( bA.v, b3CrossW( bA.w, rA ) );
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossW( bB.w, rB ) );
				b3Vec3W vr = b3SubVW( vrB, vrA );
				b3Vec2W vt = {
					b3SubW( b3DotW( vr, tangent1 ), c->tangentVelocity1 ),
					b3SubW( b3DotW( vr, tangent2 ), c->tangentVelocity2 ),
				};

				// Incremental tangent impulse
				b3Vec2W deltaImpulse = b3MulMV2W( c->tangentMass, vt );
				deltaImpulse = { b3NegW( deltaImpulse.x ), b3NegW( deltaImpulse.y ) };
				b3Vec2W newImpulse = b3AddV2W( c->frictionImpulse, deltaImpulse );

				b3FloatW friction = c->friction;
				b3FloatW maxImpulse = b3MulW( friction, totalNormalImpulse );

				// Clamp the accumulated impulse
				b3FloatW lengthSquared = b3AddW( b3MulW( newImpulse.x, newImpulse.x ), b3MulW( newImpulse.y, newImpulse.y ) );

				// Max impulse can be zero
				b3FloatW mask = b3GreaterThanW( lengthSquared, b3MulW( maxImpulse, maxImpulse ) );

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism. Add epsilon to avoid divide by zero.
				b3FloatW normalize = b3DivW( maxImpulse, b3AddW( b3SqrtW( lengthSquared ), epsilonW ) );
				b3FloatW scale = b3BlendW( oneW, normalize, mask );
				newImpulse = {
					b3MulW( scale, newImpulse.x ),
					b3MulW( scale, newImpulse.y ),
				};

				deltaImpulse = {
					b3SubW( newImpulse.x, c->frictionImpulse.x ),
					b3SubW( newImpulse.y, c->frictionImpulse.y ),
				};

				c->frictionImpulse = newImpulse;

				// Apply delta impulse
				b3Vec3W P = b3AddVW( b3MulSVW( deltaImpulse.x, tangent1 ), b3MulSVW( deltaImpulse.y, tangent2 ) );
				bA.w = b3MulSubMVW( bA.w, c->invIA, b3CrossW( rA, P ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bB.w = b3MulAddMVW( bB.w, c->invIB, b3CrossW( rB, P ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
			}
		}

		b3ScatterBodies( states, base->indexBase1A, &bA );
		b3ScatterBodies( states, base->indexBase1B, &bB );
	}

	b3TracyCZoneEnd( solve_contact );
}

void b3ApplyRestitutionTask( int startIndex, int endIndex, b3StepContext* context, int colorIndex, int manifoldCount )
{
	b3TracyCZoneNC( restitution, "Restitution", b3_colorDodgerBlue, true );

	b3BodyState* states = context->states;
	b3ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].simdConstraints[manifoldCount - 1];
	b3FloatW threshold = b3SplatW( context->world->restitutionThreshold );
	b3FloatW zero = b3ZeroW();

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		// Get base constraint according to stride which is the manifoldCount
		b3ContactConstraintSIMD* base = constraints + manifoldCount * constraintIndex;

		// Single gather for all manifolds
		b3BodyStateW bA = b3GatherBodies( states, base->indexBase1A );
		b3BodyStateW bB = b3GatherBodies( states, base->indexBase1B );

		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ContactConstraintSIMD* c = base + manifoldIndex;
			if ( b3AllZeroW( c->restitution ) )
			{
				// No lanes have restitution. Common case.
				continue;
			}

			// Create a mask based on restitution so that lanes with no restitution are not affected
			// by the calculations below.
			b3FloatW restitutionMask = b3EqualsW( c->restitution, zero );

			for ( int pointIndex = 0; pointIndex < 4; ++pointIndex )
			{
				// Set effective mass to zero if restitution should not be applied
				b3FloatW mask1 = b3GreaterThanW( b3AddW( c->relativeVelocities[pointIndex], threshold ), zero );
				b3FloatW mask2 = b3EqualsW( c->totalNormalImpulses[pointIndex], zero );
				b3FloatW mask = b3OrW( b3OrW( mask1, mask2 ), restitutionMask );
				b3FloatW mass = b3BlendW( c->normalMasses[pointIndex], zero, mask );

				// fixed anchors for Jacobians
				b3Vec3W rA = c->anchorAs[pointIndex];
				b3Vec3W rB = c->anchorBs[pointIndex];

				// Relative velocity at contact
				b3Vec3W vrA = b3AddVW( bA.v, b3CrossW( bA.w, rA ) );
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossW( bB.w, rB ) );
				b3FloatW vn = b3DotW( b3SubVW( vrB, vrA ), c->normal );

				// Compute normal impulse
				b3FloatW negImpulse = b3MulW( mass, b3AddW( vn, b3MulW( c->restitution, c->relativeVelocities[pointIndex] ) ) );

				// Clamp the accumulated impulse
				b3FloatW newImpulse = b3MaxW( b3SubW( c->normalImpulses[pointIndex], negImpulse ), b3ZeroW() );
				b3FloatW deltaImpulse = b3SubW( newImpulse, c->normalImpulses[pointIndex] );
				c->normalImpulses[pointIndex] = newImpulse;

				// Add the incremental impulse rather than the full impulse because this is not a sub-step
				c->totalNormalImpulses[pointIndex] = b3AddW( c->totalNormalImpulses[pointIndex], deltaImpulse );

				// Apply contact impulse
				b3Vec3W P = b3MulSVW( deltaImpulse, c->normal );
				bA.w = b3MulSubMVW( bA.w, c->invIA, b3CrossW( rA, P ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bB.w = b3MulAddMVW( bB.w, c->invIB, b3CrossW( rB, P ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
			}
		}

		b3ScatterBodies( states, base->indexBase1A, &bA );
		b3ScatterBodies( states, base->indexBase1B, &bB );
	}

	b3TracyCZoneEnd( restitution );
}

// Store impulses by contact constraint
void b3StoreImpulsesTask( int startIndex, int endIndex, b3StepContext* context, int manifoldCount )
{
	b3TracyCZoneNC( store_impulses, "Store", b3_colorFireBrick, true );

	const b3ContactConstraintSIMD* constraints = context->contactConstraints[manifoldCount - 1];

	// todo why am I storing into a dummy manifold? Just continue loop?
	b3Manifold dummy = {};

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		const b3ContactConstraintSIMD* constraintBase = constraints + manifoldCount * constraintIndex;

		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			const b3ContactConstraintSIMD* c = constraintBase + manifoldIndex;
			const float* frictionImpulse1 = (float*)&c->frictionImpulse.x;
			const float* frictionImpulse2 = (float*)&c->frictionImpulse.y;
			const float* tangent1X = (float*)&c->tangent1.X;
			const float* tangent1Y = (float*)&c->tangent1.Y;
			const float* tangent1Z = (float*)&c->tangent1.Z;
			const float* tangent2X = (float*)&c->tangent2.X;
			const float* tangent2Y = (float*)&c->tangent2.Y;
			const float* tangent2Z = (float*)&c->tangent2.Z;
			const float* twistImpulse = (float*)&c->twistImpulse;
			const float* rollingImpulseX = (float*)&c->rollingImpulse.X;
			const float* rollingImpulseY = (float*)&c->rollingImpulse.Y;
			const float* rollingImpulseZ = (float*)&c->rollingImpulse.Z;

			for ( int laneIndex = 0; laneIndex < B3_SIMD_WIDTH; ++laneIndex )
			{
				b3Manifold* m = c->manifolds[laneIndex] == nullptr ? &dummy : c->manifolds[laneIndex];

				float f1 = frictionImpulse1[laneIndex];
				float f2 = frictionImpulse2[laneIndex];
				m->frictionImpulse = {
					f1 * tangent1X[laneIndex] + f2 * tangent2X[laneIndex],
					f1 * tangent1Y[laneIndex] + f2 * tangent2Y[laneIndex],
					f1 * tangent1Z[laneIndex] + f2 * tangent2Z[laneIndex],
				};
				m->twistImpulse = twistImpulse[laneIndex];
				m->rollingImpulse = {
					rollingImpulseX[laneIndex],
					rollingImpulseY[laneIndex],
					rollingImpulseZ[laneIndex],
				};

				int pointCount = m->pointCount;
				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					const float* normalImpulse = (float*)&c->normalImpulses[pointIndex];
					const float* totalNormalImpulse = (float*)&c->totalNormalImpulses[pointIndex];
					const float* normalVelocity = (float*)&c->relativeVelocities[pointIndex];

					b3ManifoldPoint* mp = m->points + pointIndex;
					mp->normalImpulse = normalImpulse[laneIndex];
					mp->totalNormalImpulse = totalNormalImpulse[laneIndex];
					mp->normalVelocity = normalVelocity[laneIndex];
				}
			}
		}
	}

	b3TracyCZoneEnd( store_impulses );
}
