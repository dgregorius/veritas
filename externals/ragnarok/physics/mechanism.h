//--------------------------------------------------------------------------------------------------
/*
	@file		mechanism.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/array.h"
#include "ragnarok/common/math.h"

class RkJoint;
class RkWorld;

struct RkCompliance;


//--------------------------------------------------------------------------------------------------
// RkMechanism
//--------------------------------------------------------------------------------------------------
class RkMechanism
	{
	public:
		// Construction / Destruction
		RkMechanism( RkWorld* World, const RkArray< RkJoint* >& Joints );
		~RkMechanism();

		// World
		RkWorld* GetWorld() const;
		
	ragnarok:
		// World
		int WorldIndex = -1;

		// Solver
		int LoadConstraints( uint8* ConstraintBuffer, RkVector3* VelocityBuffer, const RkCompliance& Compliance, float Timestep );
		int SolveConstraints( uint8* ConstraintBuffer, RkVector3* VelocityBuffer, const RkCompliance& Compliance );
		int SaveConstraints( uint8* ConstraintBuffer );
	
		int Factor();
		int Solve( RkVector3* VelocityBuffer );

	private:
		RkWorld* mWorld;
		RkArray< RkJoint* > mJoints;

		int mMatrixSize;
		int mColumnCount;
		RkArray< int > mColumnWidth;
		RkArray< int > mColumnHeight;
		RkArray< int > mColumnOffset;
	};


