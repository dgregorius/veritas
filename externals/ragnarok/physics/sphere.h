//--------------------------------------------------------------------------------------------------
/*
	@file		sphere.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/math.h"


//--------------------------------------------------------------------------------------------------
// RkSphere
//--------------------------------------------------------------------------------------------------
struct RkSphere
	{
	RkVector3 Center;
	float Radius;

	RkSphere() = default;
	RkSphere( const RkVector3& Center, float Radius );

	RkBounds3 ComputeBounds() const;
	RkBounds3 ComputeBounds( const RkTransform& Transform ) const;
	};


#include "sphere.inl"