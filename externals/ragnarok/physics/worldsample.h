//--------------------------------------------------------------------------------------------------
/*
	@file		worldsample.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/math.h"
#include "ragnarok/common/memory.h"


//--------------------------------------------------------------------------------------------------
// RkWorldSample
//--------------------------------------------------------------------------------------------------
struct RkWorldSample
	{
	// Main hustles
	float Step = 0.0f;
		float Broadphase = 0.0f;
			float UpdatePairs = 0.0f;
			float AddContacts = 0.0f;
		float Narrowphase = 0.0f;
			float CollideContacts = 0.0f;
			float UpdateContacts = 0.0f;
		float Solve = 0.0f;
			float Allocate = 0.0f;
			float IntegrateForces = 0.0f;
			float LoadConstraints = 0.0f;
			float SolveConstraints = 0.0f;
			float SaveConstraints = 0.0f;
			float IntegrateVelocities = 0.0f;
			float RefitProxies = 0.0f;
				float UpdateBounds = 0.0f;
				float PropagateBounds = 0.0f;
			float Sleeping = 0.0f;
				float AdvanceIslands = 0.0f;
				float SleepIslands = 0.0f;

	// Side hustles
	float RebuildTask = 0.0f;
	float SplitTask = 0.0f;

	void Reset() { rkMemZero( this, sizeof( RkWorldSample ) ); }
	};

RkWorldSample rkAdd( const RkWorldSample& Lhs, const RkWorldSample& Rhs );
RkWorldSample rkMin( const RkWorldSample& Lhs, const RkWorldSample& Rhs );
RkWorldSample rkMax( const RkWorldSample& Lhs, const RkWorldSample& Rhs );

RkWorldSample operator+( const RkWorldSample& Lhs, const RkWorldSample& Rhs );
RkWorldSample operator*( float Rhs, const RkWorldSample& Lhs );
RkWorldSample operator*( const RkWorldSample& Lhs, float Rhs );
RkWorldSample operator/( const RkWorldSample& Lhs, float Rhs );