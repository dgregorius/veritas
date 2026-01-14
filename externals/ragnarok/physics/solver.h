//--------------------------------------------------------------------------------------------------
/*
	@file		solver.h

	@author		Dirk Gregorius
	@version	02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once


//--------------------------------------------------------------------------------------------------
// RkCompliance
//--------------------------------------------------------------------------------------------------
struct RkCompliance
	{
	float BiasScale;
	float LambdaScale;
	float MassScale;
	};

RkCompliance rkCreateCompliance( float Frequency, float DampingRatio, float Timestep );


//--------------------------------------------------------------------------------------------------
// RkSolverBody
//--------------------------------------------------------------------------------------------------
struct RK_ALIGN16 RkSolverBody
	{
	RkVector3 LinearVelocity;
	uint8 LinearPadding[ 4 ];
	RkVector3 AngularVelocity;
	uint8 AngularPadding[ 4 ];
	};


#include "solver.inl"

