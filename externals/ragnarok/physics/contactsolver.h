//--------------------------------------------------------------------------------------------------
/*
	@file		contactsolver.h

	@author		Dirk Gregorius
	@version	0.1
	@date		07/01/2012

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/solver.h"



//--------------------------------------------------------------------------------------------------
// RkAngularFrictionConstraint
//--------------------------------------------------------------------------------------------------
struct RkAngularFrictionConstraint
	{
	// J = ( 0 | -n | 0 | n )
	RkVector3 Normal;

	float Mass;
	float Lambda;
	float MaxLambda;

	// P = JT * lambda => dv = M^-1 * JT * lambda 
	RkVector3 W_JT1, W_JT2;
	};

void rkCreateConstraint( RkAngularFrictionConstraint& Out, const RkBody* Body1, const RkBody* Body2, const RkVector3& Normal, float Lambda, float MaxLambda );
void rkWarmstartConstraint( const RkAngularFrictionConstraint& Constraint, RkVector3& W1, RkVector3& W2 );
void rkSolveConstraint( RkAngularFrictionConstraint& Constraint, RkVector3& W1, RkVector3& W2 );


//--------------------------------------------------------------------------------------------------
// RkLinearFrictionConstraint
//--------------------------------------------------------------------------------------------------
struct RkLinearFrictionConstraint
	{
	RkVector3 Tangent[ 2 ];
	RkVector3 Offset1, Offset2;

	RkMatrix2 Mass;
	RkVector2 Lambda;
	float MaxLambda;

	// P = JT * lambda => dv = M^-1 * JT * lambda 
	float InvM1, InvM2;
	RkVector3 W_JT1[ 2 ], W_JT2[ 2 ];
	};

void rkCreateConstraint( RkLinearFrictionConstraint& Out, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, RkVector3 Tangent[ 2 ], RkVector2 Lambda, float MaxLambda);
void rkWarmstartConstraint( const RkLinearFrictionConstraint& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2 );
void rkSolveConstraint( RkLinearFrictionConstraint& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2 );


//--------------------------------------------------------------------------------------------------
// RkNonPenetrationConstraint
//--------------------------------------------------------------------------------------------------
struct RkNonPenetrationConstraint
	{
	// J = ( -n | -r1 x n | n | r2 x n )
	RkVector3 Normal;
	RkVector3 Offset1, Offset2;

	float Bias;
	float Mass;
	float Lambda;

	// P = JT * lambda => dv = M^-1 * JT * lambda 
	float InvM1, InvM2;
	RkVector3 W_JT1, W_JT2;
	};

void rkCreateConstraint( RkNonPenetrationConstraint& Out, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, const RkVector3& Normal, float Bias, float Lambda );
void rkWarmstartConstraint( const RkNonPenetrationConstraint& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2 );
void rkSolveConstraint( RkNonPenetrationConstraint& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2, const RkCompliance& Compliance );


//--------------------------------------------------------------------------------------------------
// RkContactConstraint
//--------------------------------------------------------------------------------------------------
struct RkContactConstraint
	{
	int BodyIndex1, BodyIndex2;

	float Friction;
	RkAngularFrictionConstraint AngularFrictionConstraint;
	RkLinearFrictionConstraint LinearFrictionConstraint;

	int PointCount;
	RkCompliance Compliance;
	RkNonPenetrationConstraint NonPenetrationConstraints[ RK_MAX_MANIFOLD_POINTS ];
	float LeverArms[ RK_MAX_MANIFOLD_POINTS ];
	};


#include "contactsolver.inl"