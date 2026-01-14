//--------------------------------------------------------------------------------------------------
// sphericaljoint.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "sphericaljoint.h"
#include "body.h"
#include "world.h"

// Internal 
#include "solver.h"
#include "jointsolver.h"


//--------------------------------------------------------------------------------------------------
// RkSphericalJoint
//--------------------------------------------------------------------------------------------------
RkSphericalJoint::RkSphericalJoint( RkWorld* World, RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision )
	: RkJoint( RK_SPHERICAL_JOINT, World, Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision )
	{
	mIsTwistLimitEnabled = false;
	mMinTwist = -RK_PI;
	mMinTwistLambda = 0.0f;
	mMaxTwist = RK_PI;
	mMaxTwistLambda = 0.0f;

	mIsSwingLimitEnabled = false;
	mMaxSwing = RK_PI_OVER_TWO;
	mMaxSwingLambda = 0.0f;

	mLinearLambda = RK_VEC3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
void RkSphericalJoint::EnableTwistLimit()
	{
	if ( !mIsTwistLimitEnabled )
		{
		mMinTwistLambda = 0.0f;
		mMaxTwistLambda = 0.0f;
		mIsTwistLimitEnabled = true;
		}
	}


//--------------------------------------------------------------------------------------------------
void RkSphericalJoint::DisableTwistLimit()
	{
	if ( mIsTwistLimitEnabled )
		{
		mMinTwistLambda = 0.0f;
		mMaxTwistLambda = 0.0f;
		mIsTwistLimitEnabled = false;
		}
	}


//--------------------------------------------------------------------------------------------------
bool RkSphericalJoint::IsTwistLimitEnabled() const
	{
	return mIsTwistLimitEnabled;
	}


//--------------------------------------------------------------------------------------------------
void RkSphericalJoint::GetTwistLimit( float& MinTwist, float& MaxTwist ) const
	{
	MinTwist = mMinTwist;
	MaxTwist = mMaxTwist;
	}


//--------------------------------------------------------------------------------------------------
void RkSphericalJoint::SetTwistLimit( float MinTwist, float MaxTwist )
	{
	RK_ASSERT( MinTwist <= MaxTwist );
	MinTwist = rkClamp( MinTwist, -RK_PI, RK_PI );
	MaxTwist = rkClamp( MaxTwist, -RK_PI, RK_PI );
	if ( MinTwist != mMinTwist || MaxTwist != mMaxTwist )
		{
		mMinTwist = MinTwist;
		mMaxTwist = MaxTwist;
		}
	}


//--------------------------------------------------------------------------------------------------
float RkSphericalJoint::GetTwistAngle() const
	{
	RkQuaternion Basis1 = GetBasis1();
	RkQuaternion Basis2 = GetBasis2();
	if ( rkDot( Basis1, Basis2 ) < 0.0f )
		{
		// This keeps the twist angle in the range [-pi, pi]
		Basis2 = -Basis2;
		}

	RkQuaternion RelQ = rkCMul( Basis1, Basis2 );
	return rkTwistAngle( RelQ );
	}


//--------------------------------------------------------------------------------------------------
void RkSphericalJoint::EnableSwingLimit()
	{
	if ( !mIsSwingLimitEnabled )
		{
		mMaxSwingLambda = 0.0f;
		mIsSwingLimitEnabled = true;
		}
	}


//--------------------------------------------------------------------------------------------------
void RkSphericalJoint::DisableSwingLimit()
	{
	if ( mIsSwingLimitEnabled )
		{
		mMaxSwingLambda = 0.0f;
		mIsSwingLimitEnabled = false;
		}
	}


//--------------------------------------------------------------------------------------------------
bool RkSphericalJoint::IsSwingLimitEnabled() const
	{
	return mIsSwingLimitEnabled;
	}


//--------------------------------------------------------------------------------------------------
float RkSphericalJoint::GetSwingLimit() const
	{
	return mMaxSwing;
	}


//--------------------------------------------------------------------------------------------------
void RkSphericalJoint::SetSwingLimit( float MaxSwing )
	{
	MaxSwing = rkClamp( MaxSwing, 0.0f, RK_PI_OVER_TWO );
	if ( mMaxSwing != MaxSwing )
		{
		mMaxSwing = MaxSwing;
		}
	}


//--------------------------------------------------------------------------------------------------
float RkSphericalJoint::GetSwingAngle() const
	{
	RkQuaternion Basis1 = GetBasis1();
	RkQuaternion Basis2 = GetBasis2();
	if ( rkDot( Basis1, Basis2 ) < 0.0f )
		{
		// This keeps the swing angle in the range [0, pi]
		Basis2 = -Basis2;
		}

	RkQuaternion RelQ = rkCMul( Basis1, Basis2 );
	return rkSwingAngle( RelQ );
	}


//--------------------------------------------------------------------------------------------------
void RkSphericalJoint::LoadConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep )
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

	RkVector3 AxisZ1 = Basis1 * RK_VEC3_AXIS_Z;
	RkVector3 AxisZ2 = Basis2 * RK_VEC3_AXIS_Z;
	
	// Setup constraint
	RkJointConstraint* Constraint = ConstraintBuffer;
	
	int BodyIndex1 = mBody1->SolverIndex + 1;
	RK_ASSERT( BodyIndex1 > 0 || BodyIndex1 == 0 && mBody1->GetType() == RK_STATIC_BODY );
	Constraint->BodyIndex1 = BodyIndex1;
	int BodyIndex2 = mBody2->SolverIndex + 1;
	RK_ASSERT( BodyIndex2 > 0 || BodyIndex2 == 0 && mBody2->GetType() == RK_STATIC_BODY );
	Constraint->BodyIndex2 = BodyIndex2;

	rkMemZero( &Constraint->Spherical.MinTwist, sizeof( RkAngularLimit ) );
	rkMemZero( &Constraint->Spherical.MaxTwist, sizeof( RkAngularLimit ) );
	if ( mIsTwistLimitEnabled )
		{
		float TanThetaOver2 = rkSqrt( ( RelQ.X * RelQ.X + RelQ.Y * RelQ.Y ) / ( RelQ.Z * RelQ.Z + RelQ.W * RelQ.W ) );
		RkVector3 TwistAxis = AxisZ1 + TanThetaOver2 * rkCross( rkNormalize( rkCross( AxisZ1, AxisZ2 ) ), AxisZ1 );

		// Compute angle error around limit midpoint. Effectively making the limit symmetric.This way 
		// we always recover from the smallest error.E.g. We want Min = 0, Max = 180 and Angle = -175 
		// to resolve towards 180 and not 0.
		float MidPoint = ( mMaxTwist + mMinTwist ) / 2.0f;
		float TwistMin = mMinTwist - MidPoint;
		float TwistMax = mMaxTwist - MidPoint;
		float Twist = rkTwistAngle( RelQ );
		Twist = rkUnwind( Twist - MidPoint );

 		float MinBias = Twist - TwistMin;
		RkCompliance MinCompliance = MinBias < 0.0f ? Compliance : RkCompliance{ 1.0f / Timestep, 0.0f, 1.0f };
 		rkCreateConstraint( Constraint->Spherical.MinTwist, mBody1, mBody2, TwistAxis, MinBias, mMinTwistLambda, 0.0f, RK_F32_MAX, MinCompliance );
 		float MaxBias = Twist - TwistMax;
		RkCompliance MaxCompliance = MaxBias > 0.0f ? Compliance : RkCompliance{ 1.0f / Timestep, 0.0f, 1.0f };
 		rkCreateConstraint( Constraint->Spherical.MaxTwist, mBody1, mBody2, TwistAxis, MaxBias, mMaxTwistLambda, -RK_F32_MAX, 0.0f, MaxCompliance );
		}

	rkMemZero( &Constraint->Spherical.Swing, sizeof( RkAngularLimit ) );
	if ( mIsSwingLimitEnabled )
		{
		float Swing = rkSwingAngle( RelQ );
		RkVector3 SwingAxis = rkNormalize( rkCross( AxisZ1, AxisZ2 ) );

		float SwingBias = Swing - mMaxSwing;
		RkCompliance SwingCompliance = SwingBias > 0.0f ? Compliance : RkCompliance { 1.0f / Timestep, 0.0f, 1.0f };
		rkCreateConstraint( Constraint->Spherical.Swing, mBody1, mBody2, SwingAxis, SwingBias, mMaxSwingLambda, -RK_F32_MAX, 0.0f, SwingCompliance );
		}

	RkVector3 Bias = Origin2 - Origin1;
	rkCreateConstraint( Constraint->Spherical.Linear, mBody1, Offset1, mBody2, Offset2, Bias, mLinearLambda );

	// Warmstart
	RkVector3 AngularLambda = mMinTwistLambda * Constraint->Spherical.MinTwist.Axis + mMaxTwistLambda * Constraint->Spherical.MaxTwist.Axis + mMaxSwingLambda * Constraint->Spherical.Swing.Axis;

	BodyBuffer[ BodyIndex1 ].LinearVelocity -= InvM1 * mLinearLambda;
	BodyBuffer[ BodyIndex1 ].AngularVelocity -= InvI1 * ( AngularLambda + rkCross( Offset1, mLinearLambda ) );
	BodyBuffer[ BodyIndex2 ].LinearVelocity += InvM2 * mLinearLambda;
	BodyBuffer[ BodyIndex2 ].AngularVelocity += InvI2 * ( AngularLambda + rkCross( Offset2, mLinearLambda ) );
	}


//--------------------------------------------------------------------------------------------------
void RkSphericalJoint::SolveConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance )
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
	rkSolveConstraint( Constraint->Spherical.MinTwist, W1, W2 );
	rkSolveConstraint( Constraint->Spherical.MaxTwist, W1, W2 );
	rkSolveConstraint( Constraint->Spherical.Swing, W1, W2 );
	rkSolveConstraint( Constraint->Spherical.Linear, V1, W1, V2, W2, Compliance );

	// Store velocities
	BodyBuffer[ BodyIndex1 ].LinearVelocity = V1;
	BodyBuffer[ BodyIndex1 ].AngularVelocity = W1;
	BodyBuffer[ BodyIndex2 ].LinearVelocity = V2;
	BodyBuffer[ BodyIndex2 ].AngularVelocity = W2;
	}


//--------------------------------------------------------------------------------------------------
void RkSphericalJoint::SaveConstraints( RkJointConstraint* ConstraintBuffer )
	{
	RkJointConstraint* Constraint = ConstraintBuffer;
	
	mMinTwistLambda = Constraint->Spherical.MinTwist.Lambda;
	mMaxTwistLambda = Constraint->Spherical.MaxTwist.Lambda;
	mMaxSwingLambda = Constraint->Spherical.Swing.Lambda;
	mLinearLambda = Constraint->Spherical.Linear.Lambda;
	}