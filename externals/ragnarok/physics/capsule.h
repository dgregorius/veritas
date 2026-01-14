//--------------------------------------------------------------------------------------------------
/*
	@file		capsule.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/math.h"


//--------------------------------------------------------------------------------------------------
// RkCapsule
//--------------------------------------------------------------------------------------------------
struct RkCapsule
	{
	RkVector3 Center1;
	RkVector3 Center2;
	float Radius;

	RkCapsule() = default;
	RkCapsule( const RkVector3& Center1, const RkVector3& Center2, float Radius );

	RkBounds3 ComputeBounds() const;
	RkBounds3 ComputeBounds( const RkTransform& Transform ) const;

	int FindSupportVertex( const RkVector3& Direction ) const;
	RkVector3 GetCenter( int Index ) const;
	};


#include "capsule.inl"