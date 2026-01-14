//--------------------------------------------------------------------------------------------------
/*
	@file		jointsolver.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/solver.h"


//--------------------------------------------------------------------------------------------------
// Swing/Twist Decomposition
//--------------------------------------------------------------------------------------------------
float rkSwingAngle( const RkQuaternion& RelQ );
float rkTwistAngle( const RkQuaternion& RelQ );
float rkUnwind( float Angle );


//--------------------------------------------------------------------------------------------------
// RkLinearConstraint2
//--------------------------------------------------------------------------------------------------
struct RkLinearConstraint2
	{
	RkVector3 Offset1, Offset2;
	RkVector3 Axes[ 2 ];
	
	RkVector2 Bias;
	RkMatrix2 Mass;
	RkVector2 Lambda;

	float InvM1, InvM2;
	RkVector3 W_JT1[ 2 ], W_JT2[ 2 ];
	};

void rkCreateConstraint( RkLinearConstraint2& Constraint, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, RkVector3 Axes[ 2 ], const RkVector2& Bias, const RkVector2& Lambda );
void rkSolveConstraint( RkLinearConstraint2& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2, const RkCompliance& Compliance );


//--------------------------------------------------------------------------------------------------
// RkLinearConstraint3
//--------------------------------------------------------------------------------------------------
struct RkLinearConstraint3
	{
	RkVector3 Offset1, Offset2;

	RkVector3 Bias;
	RkMatrix3 Mass;
	RkVector3 Lambda;

	float InvM1, InvM2;
	RkMatrix3 InvI1, InvI2;
	};

void rkCreateConstraint( RkLinearConstraint3& Constraint, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, const RkVector3& Bias, const RkVector3& Lambda );
void rkSolveConstraint( RkLinearConstraint3& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2, const RkCompliance& Compliance );


//--------------------------------------------------------------------------------------------------
// RkSoftLinearConstraint3
//--------------------------------------------------------------------------------------------------
struct RkSoftLinearConstraint3
	{
	RkVector3 Offset1, Offset2;
	
	float BiasScale;
	RkVector3 Bias;
	float MassScale;
	RkMatrix3 Mass;
	float LambdaScale;
	RkVector3 Lambda;
	float MaxLambda;

	float InvM1, InvM2;
	RkMatrix3 InvI1, InvI2;
	};

void rkCreateConstraint( RkSoftLinearConstraint3& Constraint, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, const RkVector3& Bias, const RkVector3& Lambda, float MaxLambda, const RkCompliance& Compliance );
void rkSolveConstraint( RkSoftLinearConstraint3& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2 );


//--------------------------------------------------------------------------------------------------
// RkLinearLimit
//--------------------------------------------------------------------------------------------------
struct RkLinearLimit
	{
	RkVector3 Offset1, Offset2;
	RkVector3 Axis;

	float BiasScale;
	float Bias;
	float MassScale;
	float Mass;
	float LambdaScale;
	float Lambda;
	float MinLambda;
	float MaxLambda;

	float InvM1, InvM2;
	RkVector3 W_JT1, W_JT2;
	};

void rkCreateConstraint( RkLinearLimit& Constraint, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, const RkVector3& Axis, float Bias, float Lambda, float MinLambda, float MaxLambda, const RkCompliance& Compliance );
void rkSolveConstraint( RkLinearLimit& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2 );


//--------------------------------------------------------------------------------------------------
// RkAngularConstraint2
//--------------------------------------------------------------------------------------------------
struct RkAngularConstraint2
	{
	RkVector3 Axes[ 2 ];

	RkVector2 Bias;
	RkMatrix2 Mass;
	RkVector2 Lambda;

	RkVector3 W_JT1[ 2 ], W_JT2[ 2 ];
	};

void rkCreateConstraint( RkAngularConstraint2& Constraint, const RkBody* Body1, const RkBody* Body2, RkVector3 Axes[ 2 ], const RkVector2& Bias, const RkVector2& Lambda);
void rkSolveConstraint( RkAngularConstraint2& Constraint, RkVector3& W1, RkVector3& W2, const RkCompliance& Compliance );


//--------------------------------------------------------------------------------------------------
// RkAngularConstraint3
//--------------------------------------------------------------------------------------------------
struct RkAngularConstraint3
	{
	RkVector3 Bias;
	RkMatrix3 Mass;
	RkVector3 Lambda;

	RkMatrix3 InvI1, InvI2;
	};

void rkCreateConstraint( RkAngularConstraint3& Constraint, const RkBody* Body1, const RkBody* Body2, const RkVector3& Bias, const RkVector3& Lambda );
void rkSolveConstraint( RkAngularConstraint3& Constraint, RkVector3& W1, RkVector3& W2, const RkCompliance& Compliance );


//--------------------------------------------------------------------------------------------------
// RkSoftAngularConstraint3
//--------------------------------------------------------------------------------------------------
struct RkSoftAngularConstraint3
	{
	float BiasScale;
	RkVector3 Bias;
	float MassScale;
	RkMatrix3 Mass;
	float LambdaScale;
	RkVector3 Lambda;
	float MaxLambda;

	RkMatrix3 InvI1, InvI2;
	};

void rkCreateConstraint( RkSoftAngularConstraint3& Constraint, const RkBody* Body1, const RkBody* Body2, const RkVector3& Bias, const RkVector3& Lambda, float MaxLambda, const RkCompliance& Compliance );
void rkSolveConstraint( RkSoftAngularConstraint3& Constraint, RkVector3& W1, RkVector3& W2 );


//--------------------------------------------------------------------------------------------------
// RkAngularLimit
//--------------------------------------------------------------------------------------------------
struct RkAngularLimit
	{
	RkVector3 Axis;

	float BiasScale;
	float Bias;
	float MassScale;
	float Mass;
	float LambdaScale;
	float Lambda;
	float MinLambda;
	float MaxLambda;

	RkVector3 W_JT1, W_JT2;
	};

void rkCreateConstraint( RkAngularLimit& Constraint, const RkBody* Body1, const RkBody* Body2, const RkVector3& Axis, float Bias, float Lambda, float MinLambda, float MaxLambda, const RkCompliance& Compliance );
void rkSolveConstraint( RkAngularLimit& Constraint, RkVector3& W1, RkVector3& W2 );


//--------------------------------------------------------------------------------------------------
// RkSphericalConstraint
//--------------------------------------------------------------------------------------------------
struct RkSphericalConstraint
	{
	RkAngularLimit MinTwist;
	RkAngularLimit MaxTwist;
	RkAngularLimit Swing;
	RkLinearConstraint3 Linear;
	};


//--------------------------------------------------------------------------------------------------
// RkRevoluteConstraint
//--------------------------------------------------------------------------------------------------
struct RkRevoluteConstraint
	{
	RkAngularLimit MinLimit;
	RkAngularLimit MaxLimit;
	RkAngularConstraint2 Angular;
	RkLinearConstraint3 Linear;
	};


//--------------------------------------------------------------------------------------------------
// RkPrismaticConstraint
//--------------------------------------------------------------------------------------------------
struct RkPrismaticConstraint
	{
	RkLinearLimit MinLimit;
	RkLinearLimit MaxLimit;
	RkAngularConstraint3 Angular;
	RkLinearConstraint2 Linear;
	};


//--------------------------------------------------------------------------------------------------
// RkRigidConstraint
//--------------------------------------------------------------------------------------------------
struct RkRigidConstraint
	{
	RkSoftAngularConstraint3 Angular;
	RkSoftLinearConstraint3 Linear;
	};


//--------------------------------------------------------------------------------------------------
// RkJointConstraint
//--------------------------------------------------------------------------------------------------
struct RkJointConstraint
	{
	int BodyIndex1, BodyIndex2;

	union
		{
		RkSphericalConstraint Spherical;
		RkRevoluteConstraint Revolute;
		RkPrismaticConstraint Prismatic;
		RkRigidConstraint Rigid;
		};
	};


#include "jointsolver.inl"










