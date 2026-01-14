// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "joint.h"
#include "math_internal.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box3d/box3d.h"

// Linear constraint (point-to-line)
// joint axis is along joint frame A local z-axis
// perpX and perpY are world vectors fixed in A
//
// d = pB - pA = xB + rB - xA - rA
// Cx = dot(perpX, d)
// Cy = dot(perpY, d)

// CdotX = dot(d, cross(wA, perpX)) + dot(perpX, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(perpX, vA) - dot(cross(d + rA, perpX), wA) + dot(perpX, vB) + dot(cross(rB, perpX), vB)
// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]
// similar for perpY
//
// Simplification dropping dot(d, cross(wA, perpX)) (todo needs testing)
// CdotXs = dot(perpX, vB + cross(wB, rB) - vA - cross(wA, rA))
// Jxs = [-perpX, -cross(rA, perpX), perpX, cross(rB, perpX)]

// Motor/limit/spring linear constraint
// axis is the world joint axis fixed in A

// C = dot(axis, d)
// Cdot = dot(d, cross(wA, axis)) + dot(axis, vB + cross(wB, rB) - vA - cross(wA, rA))
// Cdot = -dot(axis, vA) - dot(cross(d + rA, axis), wA) + dot(axis, vB) + dot(cross(rB, axis), vB)
// J = [-axis -cross(d + rA, axis) axis cross(rB, axis)]
//
// Simplified (todo needs testing)
// Cdot = -dot(axis, vA) - dot(cross(rA, axis), wA) + dot(axis, vB) + dot(cross(rB, axis), vB)
// J = [-axis -cross(rA, axis) axis cross(rB, axis)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

void b3PrismaticJoint_EnableLimit( b3JointId jointId, bool enableLimit )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	if ( enableLimit != base->prismaticJoint.enableLimit )
	{
		base->prismaticJoint.lowerImpulse = 0.0f;
		base->prismaticJoint.upperImpulse = 0.0f;
	}
	base->prismaticJoint.enableLimit = enableLimit;
}

bool b3PrismaticJoint_IsLimitEnabled( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	return base->prismaticJoint.enableLimit;
}

float b3PrismaticJoint_GetLowerLimit( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	return base->prismaticJoint.lowerTranslation;
}

float b3PrismaticJoint_GetUpperLimit( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	return base->prismaticJoint.upperTranslation;
}

void b3PrismaticJoint_SetLimits( b3JointId jointId, float lower, float upper )
{
	B3_ASSERT( b3IsValidFloat( lower ) && b3IsValidFloat( upper ) );
	float lowerAngle = b3MinFloat( lower, upper );
	float upperAngle = b3MaxFloat( lower, upper );

	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	base->prismaticJoint.lowerTranslation = lowerAngle;
	base->prismaticJoint.upperTranslation = upperAngle;
}

float b3PrismaticJoint_GetTranslation( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	b3Transform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3Transform transformB = b3GetBodyTransform( world, base->bodyIdB );

	b3Vec3 jointAxis = b3RotateVector( base->localFrameA.q, b3Vec3_axisX );
	jointAxis = b3RotateVector( transformA.q, jointAxis );

	b3Vec3 d = ( transformB.p - transformA.p ) + b3RotateVector( transformB.q, base->localFrameB.p ) -
			   b3RotateVector( transformA.q, base->localFrameA.p );
	float translation = b3Dot( d, jointAxis );
	return translation;
}

void b3PrismaticJoint_EnableSpring( b3JointId jointId, bool enableSpring )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	if ( enableSpring != base->prismaticJoint.enableSpring )
	{
		base->prismaticJoint.springImpulse = 0.0f;
	}
	base->prismaticJoint.enableSpring = enableSpring;
}

bool b3PrismaticJoint_IsSpringEnabled( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	return base->prismaticJoint.enableSpring;
}

void b3PrismaticJoint_SetTargetTranslation( b3JointId jointId, float targetTranslation )
{
	B3_ASSERT( b3IsValidFloat( targetTranslation ) );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	base->prismaticJoint.targetTranslation = targetTranslation;
}

float b3PrismaticJoint_GetTargetTranslation( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	return base->prismaticJoint.targetTranslation;
}

void b3PrismaticJoint_SetSpringHertz( b3JointId jointId, float hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && hertz >= 0.0f );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	base->prismaticJoint.hertz = hertz;
}

float b3PrismaticJoint_GetSpringHertz( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	return base->prismaticJoint.hertz;
}

void b3PrismaticJoint_SetSpringDampingRatio( b3JointId jointId, float dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	base->prismaticJoint.dampingRatio = dampingRatio;
}

float b3PrismaticJoint_GetSpringDampingRatio( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	return base->prismaticJoint.dampingRatio;
}

void b3PrismaticJoint_EnableMotor( b3JointId jointId, bool enableMotor )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	if ( enableMotor != base->prismaticJoint.enableMotor )
	{
		base->prismaticJoint.motorImpulse = 0.0f;
	}
	base->prismaticJoint.enableMotor = enableMotor;
}

bool b3PrismaticJoint_IsMotorEnabled( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	return base->prismaticJoint.enableMotor;
}

void b3PrismaticJoint_SetMotorSpeed( b3JointId jointId, float motorSpeed )
{
	B3_ASSERT( b3IsValidFloat( motorSpeed ) );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	base->prismaticJoint.motorSpeed = motorSpeed;
}

float b3PrismaticJoint_GetMotorSpeed( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	return base->prismaticJoint.motorSpeed;
}

void b3PrismaticJoint_SetMaxMotorForce( b3JointId jointId, float maxForce )
{
	B3_ASSERT( b3IsValidFloat( maxForce ) && maxForce >= 0.0f );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	base->prismaticJoint.maxMotorForce = maxForce;
}

float b3PrismaticJoint_GetMaxMotorForce( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_prismaticJoint );
	return base->prismaticJoint.maxMotorForce;
}

b3Vec3 b3GetPrismaticJointForce( b3World* world, b3JointSim* base )
{
	b3Transform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3PrismaticJoint* joint = &base->prismaticJoint;

	// impulse in joint space
	b3Vec3 impulse = {
		joint->perpImpulse.x,
		joint->perpImpulse.y,
		joint->motorImpulse + joint->lowerImpulse + joint->upperImpulse + joint->springImpulse,
	};

	// convert impulse to force
	b3Vec3 force = world->inv_h * impulse;

	// convert to body space
	force = b3RotateVector( base->localFrameA.q, force );

	// convert to world space
	force = b3RotateVector( transformA.q, force );
	return force;
}

b3Vec3 b3GetPrismaticJointTorque( b3World* world, b3JointSim* base )
{
	b3Transform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3PrismaticJoint* joint = &base->prismaticJoint;

	b3Vec3 torque = world->inv_h * joint->angularImpulse;
	torque = b3RotateVector( base->localFrameA.q, torque );
	torque = b3RotateVector( transformA.q, torque );
	return torque;
}

void b3PreparePrismaticJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_prismaticJoint );

	b3World* world = context->world;

	b3Body* bodyA = world->bodies.Get( base->bodyIdA );
	b3Body* bodyB = world->bodies.Get( base->bodyIdB );

	B3_ASSERT( bodyB->setIndex == b3_awakeSet );
	b3SolverSet* setA = world->solverSets.Get( bodyA->setIndex );
	b3SolverSet* setB = world->solverSets.Get( bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b3BodySim* bodySimA = setA->bodySims.Get( localIndexA );
	b3BodySim* bodySimB = setB->bodySims.Get( localIndexB );

	base->invMassA = bodySimA->invMass;
	base->invMassB = bodySimB->invMass;
	base->invIA = bodySimA->invInertiaWorld;
	base->invIB = bodySimB->invInertiaWorld;

	b3Matrix3 invInertiaSum = base->invIA + base->invIB;
	base->fixedRotation = b3Det( invInertiaSum ) < 1000.0f * FLT_MIN;

	b3PrismaticJoint* joint = &base->prismaticJoint;
	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint->frameA.q = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b3RotateVector( bodySimA->transform.q, base->localFrameA.p - bodySimA->localCenter );
	joint->frameB.q = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b3RotateVector( bodySimB->transform.q, base->localFrameB.p - bodySimB->localCenter );

	joint->deltaCenter = bodySimB->center - bodySimA->center;
	joint->rotationMass = b3InvertMatrix( invInertiaSum );

	// Initial joint axes in world space
	b3Matrix3 matrixA = b3MakeMatrixFromQuat( joint->frameA.q );
	joint->jointAxis = matrixA.cx;
	joint->perpAxisY = matrixA.cy;
	joint->perpAxisZ = matrixA.cz;

	joint->springSoftness = b3MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->perpImpulse = { 0.0f, 0.0f };
		joint->angularImpulse = { 0.0f, 0.0f, 0.0f };
		joint->motorImpulse = 0.0f;
		joint->springImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b3WarmStartPrismaticJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_prismaticJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	// dummy state for static bodies
	b3BodyState dummyState = b3_identityBodyState;

	b3PrismaticJoint* joint = &base->prismaticJoint;

	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	// todo make this code and the wheel joint more similar

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );
	b3Vec3 d = ( stateB->deltaPosition - stateA->deltaPosition ) + ( rB - rA ) + joint->deltaCenter;
	b3Vec3 jointAxis = b3RotateVector( stateA->deltaRotation, joint->jointAxis );
	b3Vec3 sAx = b3Cross( rA + d, jointAxis );
	b3Vec3 sBx = b3Cross( rB, jointAxis );

	b3Vec3 perpY = b3RotateVector( stateA->deltaRotation, joint->perpAxisY );
	b3Vec3 perpZ = b3RotateVector( stateA->deltaRotation, joint->perpAxisZ );
	b3Vec3 sAy = b3Cross( rA + d, perpY );
	b3Vec3 sBy = b3Cross( rB, perpY );
	b3Vec3 sAz = b3Cross( rA + d, perpZ );
	b3Vec3 sBz = b3Cross( rB, perpZ );

	float axialImpulse = joint->springImpulse + joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse;
	b3Vec2 perpImpulse = joint->perpImpulse;

	b3Vec3 P = axialImpulse * jointAxis + perpImpulse.x * perpY + perpImpulse.y * perpZ;
	b3Vec3 LA = axialImpulse * sAx + perpImpulse.x * sAy + perpImpulse.y * sAz + joint->angularImpulse;
	b3Vec3 LB = axialImpulse * sBx + perpImpulse.x * sBy + perpImpulse.y * sBz + joint->angularImpulse;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;
	vA -= mA * P;
	wA -= b3MulMV( iA, LA );
	vB += mB * P;
	wB += b3MulMV( iB, LB );

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

void b3SolvePrismaticJoint( b3JointSim* base, b3StepContext* context, bool useBias )
{
	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	// dummy state for static bodies
	b3BodyState dummyState = b3_identityBodyState;

	b3PrismaticJoint* joint = &base->prismaticJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	bool fixedRotation = base->fixedRotation;
	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	b3Vec3 dcA = stateA->deltaPosition;
	b3Vec3 dcB = stateB->deltaPosition;
	b3Vec3 d = ( dcB - dcA ) + ( rB - rA ) + joint->deltaCenter;

	b3Vec3 jointAxis = b3RotateVector( stateA->deltaRotation, joint->jointAxis );
	b3Vec3 sAx = b3Cross( rA + d, jointAxis );
	b3Vec3 sBx = b3Cross( rB, jointAxis );
	float jointTranslation = b3Dot( d, jointAxis );
	float targetTranslation = joint->targetTranslation;

	// The axial effective mass must be fresh to avoid divergence when the joint is stressed
	float ka = mA + mB + b3Dot( sAx, b3MulMV( iA, sAx ) ) + b3Dot( sBx, b3MulMV( iB, sBx ) );
	float axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	// Solve spring
	if ( joint->enableSpring && fixedRotation == false )
	{
		// Get the substep relative rotation
		float c = jointTranslation - targetTranslation;

		float bias = joint->springSoftness.biasRate * c;
		float massScale = joint->springSoftness.massScale;
		float impulseScale = joint->springSoftness.impulseScale;

		float cdot = b3Dot( vB + b3Cross( wB, rB ) - vA - b3Cross( wA, rA + d ), jointAxis );
		float deltaImpulse = -massScale * axialMass * ( cdot + bias ) - impulseScale * joint->springImpulse;
		joint->springImpulse += deltaImpulse;

		b3Vec3 P = deltaImpulse * jointAxis;
		b3Vec3 LA = deltaImpulse * sAx;
		b3Vec3 LB = deltaImpulse * sBx;

		vA -= mA * P;
		wA -= b3MulMV( iA, LA );
		vB += mB * P;
		wB += b3MulMV( iB, LB );
	}

	if ( joint->enableMotor && fixedRotation == false )
	{
		float cdot = b3Dot( vB + b3Cross( wB, rB ) - vA - b3Cross( wA, rA + d ), jointAxis ) - joint->motorSpeed;

		float deltaImpulse = -axialMass * cdot;
		float newImpulse = joint->motorImpulse + deltaImpulse;
		float maxImpulse = joint->maxMotorForce * context->h;
		newImpulse = b3ClampFloat( newImpulse, -maxImpulse, maxImpulse );
		deltaImpulse = newImpulse - joint->motorImpulse;
		joint->motorImpulse = newImpulse;

		b3Vec3 P = deltaImpulse * jointAxis;
		b3Vec3 LA = deltaImpulse * sAx;
		b3Vec3 LB = deltaImpulse * sBx;

		vA -= mA * P;
		wA -= b3MulMV( iA, LA );
		vB += mB * P;
		wB += b3MulMV( iB, LB );
	}

	if ( joint->enableLimit && fixedRotation == false )
	{
		float speculativeDistance = 0.25f * ( joint->upperTranslation - joint->lowerTranslation );

		// Lower limit
		{
			float C = jointTranslation - joint->lowerTranslation;

			if ( C < speculativeDistance )
			{
				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if ( C > 0.0f )
				{
					// speculation
					bias = C * context->inv_h;
				}
				else if ( useBias )
				{
					bias = base->constraintSoftness.biasRate * C;
					massScale = base->constraintSoftness.massScale;
					impulseScale = base->constraintSoftness.impulseScale;
				}

				float cdot = b3Dot( vB + b3Cross( wB, rB ) - vA - b3Cross( wA, rA + d ), jointAxis );
				float oldImpulse = joint->lowerImpulse;
				float deltaImpulse = -massScale * axialMass * ( cdot + bias ) - impulseScale * oldImpulse;
				joint->lowerImpulse = b3MaxFloat( oldImpulse + deltaImpulse, 0.0f );
				deltaImpulse = joint->lowerImpulse - oldImpulse;

				b3Vec3 P = deltaImpulse * jointAxis;
				b3Vec3 LA = deltaImpulse * sAx;
				b3Vec3 LB = deltaImpulse * sBx;

				vA -= mA * P;
				wA -= b3MulMV( iA, LA );
				vB += mB * P;
				wB += b3MulMV( iB, LB );
			}
			else
			{
				joint->lowerImpulse = 0.0f;
			}
		}

		// Upper limit
		{
			float C = joint->upperTranslation - jointTranslation;

			if ( C < speculativeDistance )
			{
				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if ( C > 0.0f )
				{
					// speculation
					bias = C * context->inv_h;
				}
				else if ( useBias )
				{
					bias = base->constraintSoftness.biasRate * C;
					massScale = base->constraintSoftness.massScale;
					impulseScale = base->constraintSoftness.impulseScale;
				}

				// sign flipped on Cdot
				float cdot = -b3Dot( vB + b3Cross( wB, rB ) - vA - b3Cross( wA, rA + d ), jointAxis );
				float oldImpulse = joint->upperImpulse;
				float deltaImpulse = -massScale * axialMass * ( cdot + bias ) - impulseScale * oldImpulse;
				joint->upperImpulse = b3MaxFloat( oldImpulse + deltaImpulse, 0.0f );

				// sign flipped on applied impulse
				float negDeltaImpulse = oldImpulse - joint->upperImpulse;
				b3Vec3 P = negDeltaImpulse * jointAxis;
				b3Vec3 LA = negDeltaImpulse * sAx;
				b3Vec3 LB = negDeltaImpulse * sBx;

				vA -= mA * P;
				wA -= b3MulMV( iA, LA );
				vB += mB * P;
				wB += b3MulMV( iB, LB );
			}
			else
			{
				joint->upperImpulse = 0.0f;
			}
		}
	}

	// Rotation constraint
	if ( fixedRotation == false )
	{
		b3Vec3 bias = { 0.0f, 0.0f, 0.0f };
		float massScale = 1.0f;
		float impulseScale = 0.0f;

		if ( useBias )
		{
			b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
			b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );

			b3Quat relQ = b3InvMulQuat( quatA, quatB );

			b3Quat targetQuat = b3Quat_identity;

			// Check polarity since I'm subtracting coordinates
			if ( b3DotQuat( relQ, targetQuat ) < 0.0f )
			{
				relQ = b3NegateQuat( relQ );
			}

			b3Quat quatError = 2.0f * b3MulQuat( targetQuat - relQ, b3Conjugate( relQ ) );
			b3Vec3 c = -b3RotateVector( quatA, quatError.v );

			bias = b3MulSV( base->constraintSoftness.biasRate, c );
			massScale = base->constraintSoftness.massScale;
			impulseScale = base->constraintSoftness.impulseScale;
		}

		b3Vec3 cdot = wB - wA;
		b3Vec3 impulse = -massScale * joint->rotationMass * ( cdot + bias ) - impulseScale * joint->angularImpulse;
		joint->angularImpulse += impulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve point-to-line constraint
	{
		b3Vec3 perpY = b3RotateVector( stateA->deltaRotation, joint->perpAxisY );
		b3Vec3 perpZ = b3RotateVector( stateA->deltaRotation, joint->perpAxisZ );

		b3Vec2 bias = { 0.0f, 0.0f };
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			b3Vec2 c = { b3Dot( perpY, d ), b3Dot( perpZ, d ) };
			bias = base->constraintSoftness.biasRate * c;
			massScale = base->constraintSoftness.massScale;
			impulseScale = base->constraintSoftness.impulseScale;
		}

		b3Vec3 vRel = vB + b3Cross( wB, rB ) - vA - b3Cross( wA, rA + d );
		b3Vec2 cdot = { b3Dot( perpY, vRel ), b3Dot( perpZ, vRel ) };

		// K = [(1/mA + 1/mB) * eye(2) - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB)]
		// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]
		b3Vec3 sAy = b3Cross( rA + d, perpY );
		b3Vec3 sBy = b3Cross( rB, perpY );
		b3Vec3 sAz = b3Cross( rA + d, perpZ );
		b3Vec3 sBz = b3Cross( rB, perpZ );

		float kyy = mA + mB + b3Dot( sAy, b3MulMV( iA, sAy ) ) + b3Dot( sBy, b3MulMV( iB, sBy ) );
		float kyz = b3Dot( sAy, b3MulMV( iA, sAz ) ) + b3Dot( sBy, b3MulMV( iB, sBz ) );
		float kzz = mA + mB + b3Dot( sAz, b3MulMV( iA, sAz ) ) + b3Dot( sBz, b3MulMV( iB, sBz ) );

		b3Matrix2 K = { { kyy, kyz }, { kyz, kzz } };

		b3Vec2 oldImpulse = joint->perpImpulse;
		b3Vec2 deltaImpulse = -massScale * b3Solve2( K, cdot + bias ) - impulseScale * oldImpulse;
		joint->perpImpulse += deltaImpulse;

		b3Vec3 P = deltaImpulse.x * perpY + deltaImpulse.y * perpZ;

		vA -= mA * P;
		wA -= iA * ( deltaImpulse.x * sAy + deltaImpulse.y * sAz );
		vB += mB * P;
		wB += iB * ( deltaImpulse.x * sBy + deltaImpulse.y * sBz );
	}

	B3_ASSERT( b3IsValidVec3( vA ) );
	B3_ASSERT( b3IsValidVec3( wA ) );
	B3_ASSERT( b3IsValidVec3( vB ) );
	B3_ASSERT( b3IsValidVec3( wB ) );

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

void b3DrawPrismaticJoint( b3DebugDraw* draw, b3JointSim* base, b3Transform transformA, b3Transform transformB, float scale )
{
	b3Transform frameA = b3MulTransforms( transformA, base->localFrameA );
	b3Transform frameB = b3MulTransforms( transformB, base->localFrameB );

	b3Matrix3 R = b3MakeMatrixFromQuat( frameA.q );
	b3Vec3 axis = R.cx;
	b3Vec3 perpY = R.cy;
	b3Vec3 perpZ = R.cz;

	float s = 0.2f * scale;
	draw->DrawSegmentFcn( frameA.p, frameA.p + s * perpY, b3_colorGreen, draw->context );
	draw->DrawSegmentFcn( frameA.p, frameA.p + s * perpZ, b3_colorBlue, draw->context );

	b3PrismaticJoint* joint = &base->prismaticJoint;
	if ( joint->enableLimit )
	{
		b3Vec3 p1 = frameA.p + joint->lowerTranslation * axis;
		b3Vec3 p2 = frameA.p + joint->upperTranslation * axis;
		draw->DrawSegmentFcn( p1, p2, b3_colorOrange, draw->context );
		draw->DrawPointFcn( p1, 10.0f, b3_colorGreen, draw->context );
		draw->DrawPointFcn( p2, 10.0f, b3_colorRed, draw->context );
	}
	else
	{
		b3Vec3 p1 = frameA.p - 0.5f * scale * axis;
		b3Vec3 p2 = frameA.p + 0.5f * scale * axis;
		draw->DrawSegmentFcn( p1, p2, b3_colorOrange, draw->context );
	}

	draw->DrawPointFcn( frameB.p, 8.0f, b3_colorViolet, draw->context );
}
