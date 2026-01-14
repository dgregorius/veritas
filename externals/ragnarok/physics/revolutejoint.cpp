//--------------------------------------------------------------------------------------------------
// revolutejoint.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "revolutejoint.h"
#include "body.h"
#include "world.h"

// Internal 
#include "solver.h"
#include "jointsolver.h"


//--------------------------------------------------------------------------------------------------
// RkRevoluteJoint
//--------------------------------------------------------------------------------------------------
RkRevoluteJoint::RkRevoluteJoint( RkWorld* World, RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision )
	: RkJoint( RK_REVOLUTE_JOINT, World, Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision )
	{
	mIsLimitEnabled = false;
	mMinAngle = 0.0f;
	mMinLambda = 0.0f;
	mMaxAngle = 0.0f;
	mMaxLambda = 0.0f;

	mAngularLambda = RK_VEC2_ZERO;
	mLinearLambda = RK_VEC3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
void RkRevoluteJoint::EnableLimit()
	{
	if ( !mIsLimitEnabled )
		{
		mMinLambda = 0.0f;
		mMaxLambda = 0.0f;
		mIsLimitEnabled = true;
		}
	}


//--------------------------------------------------------------------------------------------------
void RkRevoluteJoint::DisableLimit()
	{
	if ( mIsLimitEnabled )
		{
		mMinLambda = 0.0f;
		mMaxLambda = 0.0f;
		mIsLimitEnabled = false;
		}
	}


//--------------------------------------------------------------------------------------------------
bool RkRevoluteJoint::IsLimitEnabled() const
	{
	return mIsLimitEnabled;
	}


//--------------------------------------------------------------------------------------------------
void RkRevoluteJoint::GetLimit( float& MinAngle, float& MaxAngle ) const
	{
	MinAngle = mMinAngle;
	MaxAngle = mMaxAngle;
	}


//--------------------------------------------------------------------------------------------------
void RkRevoluteJoint::SetLimit( float MinAngle, float MaxAngle )
	{
	RK_ASSERT( MinAngle <= MaxAngle );
	MinAngle = rkClamp( MinAngle, -RK_PI, RK_PI );
	MaxAngle = rkClamp( MaxAngle, -RK_PI, RK_PI );
	if ( mMinAngle != MinAngle || mMaxAngle != MaxAngle )
		{
		mMinAngle = MinAngle;
		mMaxAngle = MaxAngle;
		}
	}


//--------------------------------------------------------------------------------------------------
float RkRevoluteJoint::GetAngle() const
	{
	RkQuaternion Basis1 = GetBasis1();
	RkQuaternion Basis2 = GetBasis2();
	if ( rkDot( Basis1, Basis2 ) < 0.0f )
		{
		// This keeps the angle in the range [-pi, pi]
		Basis2 = -Basis2;
		}

	RkQuaternion RelQ = rkCMul( Basis1, Basis2 );
	return rkTwistAngle( RelQ );
	}


//--------------------------------------------------------------------------------------------------
void RkRevoluteJoint::LoadConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep )
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

	// Relative quaternion
	if ( rkDot( Basis1, Basis2 ) < 0.0f )
		{
		Basis2 = -Basis2;
		}
	RkQuaternion RelQ = rkCMul( Basis1, Basis2 );

	RkVector3 AxisX = 0.5f * ( Basis1 * ( RelQ.S * RK_VEC3_AXIS_X + rkCross( RelQ.V, RK_VEC3_AXIS_X ) ) );
	RkVector3 AxisY = 0.5f * ( Basis1 * ( RelQ.S * RK_VEC3_AXIS_Y + rkCross( RelQ.V, RK_VEC3_AXIS_Y ) ) );

	// Setup constraint
	RkJointConstraint* Constraint = ConstraintBuffer;
	
	int BodyIndex1 = mBody1->SolverIndex + 1;
	RK_ASSERT( BodyIndex1 > 0 || BodyIndex1 == 0 && mBody1->GetType() == RK_STATIC_BODY );
	Constraint->BodyIndex1 = BodyIndex1;
	int BodyIndex2 = mBody2->SolverIndex + 1;
	RK_ASSERT( BodyIndex2 > 0 || BodyIndex2 == 0 && mBody2->GetType() == RK_STATIC_BODY );
	Constraint->BodyIndex2 = BodyIndex2;

	rkMemZero( &Constraint->Revolute.MinLimit, sizeof( RkAngularLimit ) );
	rkMemZero( &Constraint->Revolute.MaxLimit, sizeof( RkAngularLimit ) );
	if ( mIsLimitEnabled )
		{
		RkVector3 AxisZ1 = Basis1 * RK_VEC3_AXIS_Z;
		RkVector3 AxisZ2 = Basis2 * RK_VEC3_AXIS_Z;

		float TanThetaOver2 = rkSqrt( ( RelQ.X * RelQ.X + RelQ.Y * RelQ.Y ) / ( RelQ.Z * RelQ.Z + RelQ.W * RelQ.W ) );
		RkVector3 LimitAxis = AxisZ1 + TanThetaOver2 * rkCross( rkNormalize( rkCross( AxisZ1, AxisZ2 ) ), AxisZ1 );

		// Compute angle error around limit midpoint.Effectively making the limit symmetric.This way 
		// we always recover from the smallest error.E.g. We want Min = 0, Max = 180 and Angle = -175 
		// to resolve towards 180 and not 0.
		float MidPoint = ( mMaxAngle + mMinAngle ) / 2.0f;
		float AngleMin = mMinAngle - MidPoint;
		float AngleMax = mMaxAngle - MidPoint;
		float Angle = rkTwistAngle( RelQ );
		Angle = rkUnwind( Angle - MidPoint );

		float MinBias = Angle - AngleMin;
		RkCompliance MinCompliance = MinBias < 0.0f ? Compliance : RkCompliance{ 1.0f / Timestep, 0.0f, 1.0f };
		rkCreateConstraint( Constraint->Revolute.MinLimit, mBody1, mBody2, LimitAxis, MinBias, mMinLambda, 0.0f, RK_F32_MAX, MinCompliance );
		float MaxBias = Angle - AngleMax;
		RkCompliance MaxCompliance = MaxBias > 0.0f ? Compliance : RkCompliance{ 1.0f / Timestep, 0.0f, 1.0f };
		rkCreateConstraint( Constraint->Revolute.MaxLimit, mBody1, mBody2, LimitAxis, MaxBias, mMaxLambda, -RK_F32_MAX, 0.0f, MaxCompliance );
		}

	RkVector3 AngularAxes[] = { AxisX, AxisY };
	RkVector2 AngularBias( RelQ.X, RelQ.Y );
	rkCreateConstraint( Constraint->Revolute.Angular, mBody1, mBody2, AngularAxes, AngularBias, mAngularLambda );
	RkVector3 LinearBias = Origin2 - Origin1;
	rkCreateConstraint( Constraint->Revolute.Linear, mBody1, Offset1, mBody2, Offset2, LinearBias, mLinearLambda );

	// Warmstart
	RkVector3 AngularLambda = mMinLambda * Constraint->Revolute.MinLimit.Axis + mMaxLambda * Constraint->Revolute.MaxLimit.Axis + mAngularLambda[ 0 ] * Constraint->Revolute.Angular.Axes[ 0 ] + mAngularLambda[ 1 ] * Constraint->Revolute.Angular.Axes[ 1 ];

	BodyBuffer[ BodyIndex1 ].LinearVelocity -= InvM1 * mLinearLambda;
	BodyBuffer[ BodyIndex1 ].AngularVelocity -= InvI1 * ( AngularLambda + rkCross( Offset1, mLinearLambda ) );
	BodyBuffer[ BodyIndex2 ].LinearVelocity += InvM2 * mLinearLambda;
	BodyBuffer[ BodyIndex2 ].AngularVelocity += InvI2 * ( AngularLambda + rkCross( Offset2, mLinearLambda ) );
	}


//--------------------------------------------------------------------------------------------------
void RkRevoluteJoint::SolveConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance )
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
	rkSolveConstraint( Constraint->Revolute.MinLimit, W1, W2 );
	rkSolveConstraint( Constraint->Revolute.MaxLimit, W1, W2 );
	rkSolveConstraint( Constraint->Revolute.Angular, W1, W2, Compliance );
	rkSolveConstraint( Constraint->Revolute.Linear, V1, W1, V2, W2, Compliance );

	// Store velocities
	BodyBuffer[ BodyIndex1 ].LinearVelocity = V1;
	BodyBuffer[ BodyIndex1 ].AngularVelocity = W1;
	BodyBuffer[ BodyIndex2 ].LinearVelocity = V2;
	BodyBuffer[ BodyIndex2 ].AngularVelocity = W2;
	}


//--------------------------------------------------------------------------------------------------
void RkRevoluteJoint::SaveConstraints( RkJointConstraint* ConstraintBuffer )
	{
	RkJointConstraint* Constraint = ConstraintBuffer;
	
	mMinLambda = Constraint->Revolute.MinLimit.Lambda;
	mMaxLambda = Constraint->Revolute.MaxLimit.Lambda;
	mAngularLambda = Constraint->Revolute.Angular.Lambda;
	mLinearLambda = Constraint->Revolute.Linear.Lambda;
	}