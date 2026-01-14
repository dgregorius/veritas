//--------------------------------------------------------------------------------------------------
/*
	@file		mass.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/math.h"
#include "ragnarok/common/types.h"

struct RkSphere;
struct RkCapsule;
struct RkHull;


//--------------------------------------------------------------------------------------------------
// RkMassProperties
//--------------------------------------------------------------------------------------------------
struct RkMassProperties
	{
	RkMassProperties();
	RkMassProperties( float Mass, const RkVector3& Center, const RkMatrix3& Inertia );

	float Mass;
	RkVector3 Center;
	RkMatrix3 Inertia;

	bool AreValid() const;
	};

RkMassProperties rkComputeMassProperties( int TriangleCount, const int* Triangles, int VertexCount, const RkVector3* Vertices, float Density = 1.0f );

// Basic inertia
RkMatrix3 rkSphereInertia( float Mass, float Radius );
RkMatrix3 rkCylinderInertia( float Mass, float Radius, float Height );

// Inertia helper (Io = Ic + Is and Ic = Io - Is)
RkMatrix3 rkSteiner( float Mass, const RkVector3& Origin );