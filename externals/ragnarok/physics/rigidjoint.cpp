//--------------------------------------------------------------------------------------------------
// rigidjoint.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "rigidjoint.h"
#include "body.h"
#include "world.h"

// Internal 
#include "solver.h"
#include "jointsolver.h"


//--------------------------------------------------------------------------------------------------
// RkRigidJoint
//--------------------------------------------------------------------------------------------------
RkRigidJoint::RkRigidJoint( RkWorld* World, RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision )
	: RkJoint( RK_RIGID_JOINT, World, Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision )
	{
	mRelT0 = RK_TRANSFORM_IDENTITY;

	mAngularFrequency = 0.0f;
	mAngularDampingRatio = 0.0f;
	mMaxTorque = RK_F32_MAX;
	mLinearFrequency = 0.0f;
	mLinearDampingRatio = 0.0f;
	mMaxForce = RK_F32_MAX;

	mAngularLambda = RK_VEC3_ZERO;
	mLinearLambda = RK_VEC3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
void RkRigidJoint::SetRelativeTransform( const RkTransform& RelT0 )
	{
	mRelT0 = RelT0;
	}


//--------------------------------------------------------------------------------------------------
RkTransform RkRigidJoint::GetRelativeTransform() const
	{
	return mRelT0;
	}


//--------------------------------------------------------------------------------------------------
void RkRigidJoint::SetAngularFrequency( float Frequency )
	{
	RK_ASSERT( Frequency >= 0.0f );
	mAngularFrequency = Frequency;
	}


//--------------------------------------------------------------------------------------------------
float RkRigidJoint::GetAngularFrequency() const
	{
	return mAngularFrequency;
	}


//--------------------------------------------------------------------------------------------------
void RkRigidJoint::SetAngularDampingRatio( float DampingRatio )
	{
	RK_ASSERT( DampingRatio >= 0.0f );
	mAngularDampingRatio = DampingRatio;
	}


//--------------------------------------------------------------------------------------------------
float RkRigidJoint::GetAngularDampingRatio() const
	{
	return mAngularDampingRatio;
	}


//--------------------------------------------------------------------------------------------------
void RkRigidJoint::SetMaxTorque( float MaxTorque )
	{
	RK_ASSERT( MaxTorque >= 0.0f );
	mMaxTorque = MaxTorque;
	if ( mMaxTorque == 0.0f )
		{
		// We effectively disable the angular constraint
		mAngularLambda = RK_VEC3_ZERO;
		}
	}


//--------------------------------------------------------------------------------------------------
float RkRigidJoint::GetMaxTorque() const
	{
	return mMaxTorque;
	}


//--------------------------------------------------------------------------------------------------
void RkRigidJoint::SetLinearFrequency( float Frequency )
	{
	RK_ASSERT( Frequency >= 0.0f );
	mLinearFrequency = Frequency;
	}


//--------------------------------------------------------------------------------------------------
float RkRigidJoint::GetLinearFrequency() const
	{
	return mLinearFrequency;
	}


//--------------------------------------------------------------------------------------------------
void RkRigidJoint::SetLinearDampingRatio( float DampingRatio )
	{
	RK_ASSERT( DampingRatio >= 0.0f );
	mLinearDampingRatio = DampingRatio;
	}


//--------------------------------------------------------------------------------------------------
float RkRigidJoint::GetLinearDampingRatio() const
	{
	return mLinearDampingRatio;
	}


//--------------------------------------------------------------------------------------------------
void RkRigidJoint::SetMaxForce( float MaxForce )
	{
	RK_ASSERT( MaxForce >= 0.0f );
	mMaxForce = MaxForce;
	if ( mMaxForce == 0.0f )
		{
		// We effectively disable the linear constraint
		mLinearLambda = RK_VEC3_ZERO;
		}
	}


//--------------------------------------------------------------------------------------------------
float RkRigidJoint::GetMaxForce() const
	{
	return mMaxForce;
	}


//--------------------------------------------------------------------------------------------------
void RkRigidJoint::LoadConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep )
	{
	// Constraint geometry
	float InvM1 = mBody1->GetMassInv();
	RkMatrix3 InvI1 = mBody1->GetInertiaInv();
	RkVector3 Center1 = mBody1->GetMassCenter();
	RkQuaternion Basis1 = GetBasis1();
	RkVector3 Origin1 = GetOrigin1();
	RkVector3 Offset1 = Origin1 - Center1;

	float InvM2 = mBody2->GetMassInv();
	RkMatrix3 InvI2 = mBody2->GetInertiaInv();
	RkVector3 Center2 = mBody2->GetMassCenter();
	RkQuaternion Basis2 = GetBasis2();
	RkVector3 Origin2 = GetOrigin2();
	RkVector3 Offset2 = Origin2 - Center2;

	// Setup constraint
	RkJointConstraint* Constraint = ConstraintBuffer;
	
	int BodyIndex1 = mBody1->SolverIndex + 1;
	RK_ASSERT( BodyIndex1 > 0 || BodyIndex1 == 0 && mBody1->GetType() == RK_STATIC_BODY );
	Constraint->BodyIndex1 = BodyIndex1;
	int BodyIndex2 = mBody2->SolverIndex + 1;
	RK_ASSERT( BodyIndex2 > 0 || BodyIndex2 == 0 && mBody2->GetType() == RK_STATIC_BODY );
	Constraint->BodyIndex2 = BodyIndex2;

	RkQuaternion RelQ = rkCMul( Basis1, Basis2 );
	if ( rkDot( RelQ, mRelT0.Rotation ) < 0.0f )
		{
		RelQ = -RelQ;
		}
	RkQuaternion Omega = 2.0f * ( mRelT0.Rotation - RelQ ) * rkConjugate( RelQ );
	
	RkVector3 AngularBias = Basis1 * -Omega.V;
	RkCompliance AngularCompliance = mAngularFrequency > 0.0f ? rkCreateCompliance( mAngularFrequency, mAngularDampingRatio, Timestep ) : Compliance;
	rkCreateConstraint( Constraint->Rigid.Angular, mBody1, mBody2, AngularBias, mAngularLambda, mMaxTorque * Timestep, AngularCompliance );
	RkVector3 LinearBias = Origin2 - Origin1 - mRelT0.Translation;
	RkCompliance LinearCompliance = mLinearFrequency > 0.0f ? rkCreateCompliance( mLinearFrequency, mLinearDampingRatio, Timestep ) : Compliance;
	rkCreateConstraint( Constraint->Rigid.Linear, mBody1, Offset1, mBody2, Offset2, LinearBias, mLinearLambda, mMaxForce * Timestep, LinearCompliance );

	// Warmstart
	BodyBuffer[ BodyIndex1 ].LinearVelocity -= InvM1 * mLinearLambda;
	BodyBuffer[ BodyIndex1 ].AngularVelocity -= InvI1 * ( mAngularLambda + rkCross( Offset1, mLinearLambda ) );
	BodyBuffer[ BodyIndex2 ].LinearVelocity += InvM2 * mLinearLambda;
	BodyBuffer[ BodyIndex2 ].AngularVelocity += InvI2 * ( mAngularLambda + rkCross( Offset2, mLinearLambda ) );
	}


//--------------------------------------------------------------------------------------------------
void RkRigidJoint::SolveConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance )
	{
	RkJointConstraint* Constraint = ConstraintBuffer;
	
	// Load velocities
	int BodyIndex1 = Constraint->BodyIndex1;
	int BodyIndex2 = Constraint->BodyIndex2;

	RkVector3 V1 = BodyBuffer[ BodyIndex1 ].LinearVelocity;
	RkVector3 W1 = BodyBuffer[ BodyIndex1 ].AngularVelocity;
	RkVector3 V2 = BodyBuffer[ BodyIndex2 ].LinearVelocity;
	RkVector3 W2 = BodyBuffer[ BodyIndex2 ].AngularVelocity;

	// Solve constraint
	rkSolveConstraint( Constraint->Rigid.Angular, W1, W2 );
	rkSolveConstraint( Constraint->Rigid.Linear, V1, W1, V2, W2 );

	// Store velocities
	BodyBuffer[ BodyIndex1 ].LinearVelocity = V1;
	BodyBuffer[ BodyIndex1 ].AngularVelocity = W1;
	BodyBuffer[ BodyIndex2 ].LinearVelocity = V2;
	BodyBuffer[ BodyIndex2 ].AngularVelocity = W2;
	}


//--------------------------------------------------------------------------------------------------
void RkRigidJoint::SaveConstraints( RkJointConstraint* ConstraintBuffer )
	{
	RkJointConstraint* Constraint = ConstraintBuffer;
	
	mAngularLambda = Constraint->Rigid.Angular.Lambda;
	mLinearLambda = Constraint->Rigid.Linear.Lambda;
	}