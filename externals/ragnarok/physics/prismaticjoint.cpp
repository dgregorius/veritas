//--------------------------------------------------------------------------------------------------
// prismaticjoint.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "prismaticjoint.h"
#include "body.h"
#include "world.h"

// Internal 
#include "solver.h"
#include "jointsolver.h"


//--------------------------------------------------------------------------------------------------
// RkPrismaticJoint
//--------------------------------------------------------------------------------------------------
RkPrismaticJoint::RkPrismaticJoint( RkWorld* World, RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision )
	: RkJoint( RK_PRISMATIC_JOINT, World, Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision )
	{
	mIsLimitEnabled = false;
	mMinOffset = 0.0f;
	mMinLambda = 0.0f;
	mMaxOffset = 0.0f;
	mMaxLambda = 0.0f;

	mAngularLambda = RK_VEC3_ZERO;
	mLinearLambda = RK_VEC2_ZERO;
	}


//--------------------------------------------------------------------------------------------------
void RkPrismaticJoint::EnableLimit()
	{
	if ( !mIsLimitEnabled )
		{
		mMinLambda = 0.0f;
		mMaxLambda = 0.0f;
		mIsLimitEnabled = true;
		}
	}


//--------------------------------------------------------------------------------------------------
void RkPrismaticJoint::DisableLimit()
	{
	if ( mIsLimitEnabled )
		{
		mMinLambda = 0.0f;
		mMaxLambda = 0.0f;
		mIsLimitEnabled = false;
		}
	}


//--------------------------------------------------------------------------------------------------
bool RkPrismaticJoint::IsLimitEnabled() const
	{
	return mIsLimitEnabled;
	}


//--------------------------------------------------------------------------------------------------
void RkPrismaticJoint::GetLimit( float& MinOffset, float& MaxOffset ) const
	{
	MinOffset = mMinOffset;
	MaxOffset = mMaxOffset;
	}


//--------------------------------------------------------------------------------------------------
void RkPrismaticJoint::SetLimit( float MinOffset, float MaxOffset )
	{
	RK_ASSERT( MinOffset <= MaxOffset );
	if ( mMinOffset != MinOffset || mMaxOffset != MaxOffset )
		{
		mMinOffset = MinOffset;
		mMaxOffset = MaxOffset;
		}
	}


//--------------------------------------------------------------------------------------------------
float RkPrismaticJoint::GetOffset() const
	{
	RkQuaternion Basis1 = GetBasis1();
	RkVector3 Axis = Basis1 * RK_VEC3_AXIS_Z;

	RkVector3 Origin1 = GetOrigin1();
	RkVector3 Origin2 = GetOrigin2();

	return rkDot( Axis, Origin2 - Origin1 );
	}


//--------------------------------------------------------------------------------------------------
void RkPrismaticJoint::LoadConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep )
	{
	// Constraint geometry
	float InvM1 = mBody1->GetMassInv();
	RkMatrix3 InvI1 = mBody1->GetInertiaInv();
	RkVector3 Center1 = mBody1->GetMassCenter();
	RkVector3 Origin1 = GetOrigin1();
	RkQuaternion Basis1 = GetBasis1();

	float InvM2 = mBody2->GetMassInv();
	RkMatrix3 InvI2 = mBody2->GetInertiaInv();
	RkVector3 Center2 = mBody2->GetMassCenter();
	RkVector3 Origin2 = GetOrigin2();
	RkQuaternion Basis2 = GetBasis2();

	// The relative velocity is computed at pivot 2
	RkVector3 Offset1 = Origin2 - Center1;
	RkVector3 Offset2 = Origin2 - Center2;

	// The z-axis of basis 1 defines the slider axis
	RkVector3 AxisX = Basis1 * RK_VEC3_AXIS_X;
	RkVector3 AxisY = Basis1 * RK_VEC3_AXIS_Y;
	RkVector3 AxisZ = Basis1 * RK_VEC3_AXIS_Z;

	// Relative quaternion
	RkQuaternion RelQ = rkCMul( Basis1, Basis2 );
	if ( rkDot( RelQ, RK_QUAT_IDENTITY ) < 0.0f )
		{
		RelQ = -RelQ;
		}

	RkQuaternion Omega = 2.0f * ( RK_QUAT_IDENTITY - RelQ ) * rkConjugate( RelQ );

	// Setup constraint
	RkJointConstraint* Constraint = ConstraintBuffer;
	
	int BodyIndex1 = mBody1->SolverIndex + 1;
	RK_ASSERT( BodyIndex1 > 0 || BodyIndex1 == 0 && mBody1->GetType() == RK_STATIC_BODY );
	Constraint->BodyIndex1 = BodyIndex1;
	int BodyIndex2 = mBody2->SolverIndex + 1;
	RK_ASSERT( BodyIndex2 > 0 || BodyIndex2 == 0 && mBody2->GetType() == RK_STATIC_BODY );
	Constraint->BodyIndex2 = BodyIndex2;

	rkMemZero( &Constraint->Prismatic.MinLimit, sizeof( RkLinearLimit ) );
	rkMemZero( &Constraint->Prismatic.MaxLimit, sizeof( RkLinearLimit ) );
	if ( mIsLimitEnabled )
		{
		float Offset = rkDot( Origin2 - Origin1, AxisZ );

		float MinBias = Offset - mMinOffset;
		RkCompliance MinCompliance = MinBias < 0.0f ? Compliance : RkCompliance{ 1.0f / Timestep, 0.0f, 1.0f };
		rkCreateConstraint( Constraint->Prismatic.MinLimit, mBody1, Offset1, mBody2, Offset2, AxisZ, MinBias, mMinLambda, 0.0f, RK_F32_MAX, MinCompliance );
		float MaxBias = Offset - mMaxOffset;
		RkCompliance MaxCompliance = MaxBias > 0.0f ? Compliance : RkCompliance{ 1.0f / Timestep, 0.0f, 1.0f };
		rkCreateConstraint( Constraint->Prismatic.MaxLimit, mBody1, Offset1, mBody2, Offset2, AxisZ, MaxBias, mMaxLambda, -RK_F32_MAX, 0.0f, MaxCompliance );
		}

	RkVector3 AngularBias = Basis1 * -Omega.V;
	rkCreateConstraint( Constraint->Prismatic.Angular, mBody1, mBody2, AngularBias, mAngularLambda );
	RkVector3 LinearAxes[] = { AxisX, AxisY };
	RkVector2 LinearBias( rkDot( AxisX, Origin2 - Origin1 ), rkDot( AxisY, Origin2 - Origin1 ) );
	rkCreateConstraint( Constraint->Prismatic.Linear, mBody1, Offset1, mBody2, Offset2, LinearAxes, LinearBias, mLinearLambda );

	// Warmstart
	RkVector3 LinearLambda = mMinLambda * Constraint->Prismatic.MinLimit.Axis + mMaxLambda * Constraint->Prismatic.MaxLimit.Axis + mLinearLambda[ 0 ] * Constraint->Prismatic.Linear.Axes[ 0 ] + mLinearLambda[1] * Constraint->Prismatic.Linear.Axes[ 1 ];

	BodyBuffer[ BodyIndex1 ].LinearVelocity -= InvM1 * LinearLambda;
	BodyBuffer[ BodyIndex1 ].AngularVelocity -= InvI1 * ( mAngularLambda + rkCross( Offset1, LinearLambda ) );
	BodyBuffer[ BodyIndex2 ].LinearVelocity += InvM2 * LinearLambda;
	BodyBuffer[ BodyIndex2 ].AngularVelocity += InvI2 * ( mAngularLambda + rkCross( Offset2, LinearLambda ) );
	}


//--------------------------------------------------------------------------------------------------
void RkPrismaticJoint::SolveConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance )
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
	rkSolveConstraint( Constraint->Prismatic.MinLimit, V1, W1, V2, W2 );
	rkSolveConstraint( Constraint->Prismatic.MaxLimit, V1, W1, V2, W2 );
	rkSolveConstraint( Constraint->Prismatic.Angular, W1, W2, Compliance );
	rkSolveConstraint( Constraint->Prismatic.Linear, V1, W1, V2, W2, Compliance );

	// Store velocities
	BodyBuffer[ BodyIndex1 ].LinearVelocity = V1;
	BodyBuffer[ BodyIndex1 ].AngularVelocity = W1;
	BodyBuffer[ BodyIndex2 ].LinearVelocity = V2;
	BodyBuffer[ BodyIndex2 ].AngularVelocity = W2;
	}


//--------------------------------------------------------------------------------------------------
void RkPrismaticJoint::SaveConstraints( RkJointConstraint* ConstraintBuffer )
	{
	RkJointConstraint* Constraint = ConstraintBuffer;
	
	mMinLambda = Constraint->Prismatic.MinLimit.Lambda;
	mMaxLambda = Constraint->Prismatic.MaxLimit.Lambda;
	mAngularLambda = Constraint->Prismatic.Angular.Lambda;
	mLinearLambda = Constraint->Prismatic.Linear.Lambda;
	}