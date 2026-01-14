//--------------------------------------------------------------------------------------------------
/*
	@file		manifold.h

	@author		Dirk Gregorius
	@version	0.1
	@date		04/01/2012

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/constants.h"

#include "ragnarok/common/math.h"
#include "ragnarok/common/types.h"


//--------------------------------------------------------------------------------------------------
// RkManifoldPoint
//--------------------------------------------------------------------------------------------------
struct RkManifoldPoint
	{
	RkVector3 LocalPosition1 = RK_VEC3_ZERO;
	RkVector3 LocalPosition2 = RK_VEC3_ZERO;
	uint32 Key = RK_INVALID_FEATURE_PAIR;
	float Impulse = 0.0f;
	int Triangle = -1;
	bool New = true;
	};


//--------------------------------------------------------------------------------------------------
// RkManifold
//--------------------------------------------------------------------------------------------------
struct RkManifold
	{
	int PointCount = 0;
	RkManifoldPoint Points[ RK_MAX_MANIFOLD_POINTS ];

	RkVector3 Center = RK_VEC3_ZERO;
	RkVector3 Normal = RK_VEC3_ZERO;
	RkVector3 Tangent1 = RK_VEC3_ZERO;
	RkVector3 Tangent2 = RK_VEC3_ZERO;

	float AngularFrictionImpulse = 0.0f;
	RkVector2 LinearFrictionImpulse = RK_VEC2_ZERO;

	bool Empty() const
		{
		return PointCount == 0;
		}
	};


float rkFindImpulse( const RkManifold& Manifold, uint32 Key );
void rkProjectFriction( RkManifold& Out, const RkManifold& Manifold );