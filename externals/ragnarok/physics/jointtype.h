//--------------------------------------------------------------------------------------------------
/*
	@file		jointtype.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once


//--------------------------------------------------------------------------------------------------
// RkJointType
//--------------------------------------------------------------------------------------------------
enum RkJointType
	{
	RK_SPHERICAL_JOINT,
	RK_REVOLUTE_JOINT,
	RK_PRISMATIC_JOINT,
	RK_RIGID_JOINT,

	RK_JOINT_TYPE_COUNT
	};