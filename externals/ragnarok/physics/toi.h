//--------------------------------------------------------------------------------------------------
/*
	@file		toi.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/gjk.h"

struct RkHull;


//--------------------------------------------------------------------------------------------------
// RkTOIState
//--------------------------------------------------------------------------------------------------
enum RkTOIState
	{
	RK_TOI_UNKNOWN,
	RK_TOI_FAILED,
	RK_TOI_OVERLAPPED,
	RK_TOI_TOUCHING,
	RK_TOI_SEPARATED,
	RK_TOI_OUT_OF_ITERATIONS
	};


//--------------------------------------------------------------------------------------------------
// RkTOIQuery
//--------------------------------------------------------------------------------------------------
struct RkTOIQuery
	{
	// Attributes
	RkTOIState State = RK_TOI_UNKNOWN;
	float Alpha = 1.0f;

	// Only valid if state is touching
	RkVector3 Closest1 = RK_VEC3_ZERO;
	RkVector3 Closest2 = RK_VEC3_ZERO;
	};


//--------------------------------------------------------------------------------------------------
// RkTOIProxy
//--------------------------------------------------------------------------------------------------
class RkTOIProxy : public RkGJKProxy
	{
	public:
		RkTOIProxy( int VertexCount, const RkVector3* VertexBuffer, float Radius = 0.0f, RkHull* Hull = nullptr );

		float GetRadius() const;
		RkHull* GetHull() const;

	private:
		float mRadius;
		RkHull* mHull;
	};


//--------------------------------------------------------------------------------------------------
// TOI (Time of Impact using conservative advancement)
//--------------------------------------------------------------------------------------------------
RkTOIQuery rkTOI( const RkSweep& Sweep1, const RkTOIProxy& Proxy1, const RkSweep& Sweep2, const RkTOIProxy& Proxy2, float MaxAlpha = 1.0f, int MaxIterations = 32 );
RkTOIQuery rkTOI( const RkTransform& Start1, const RkVector3& End1, const RkTOIProxy& Proxy1, const RkTransform& Start2, const RkVector3& End2, const RkTOIProxy& Proxy2, float MaxAlpha = 1.0f, int MaxIterations = 32 );