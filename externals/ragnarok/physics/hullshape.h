//--------------------------------------------------------------------------------------------------
/*
	@file		hullshape.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/shape.h"
#include "ragnarok/physics/hull.h"


//--------------------------------------------------------------------------------------------------
// RkHullShape
//--------------------------------------------------------------------------------------------------
class RkHullShape : public RkShape
	{
	public:
		// Construction 
		RkHullShape( RkBody* Body, RkHull* Hull );

		// Capsule
		RkHull* GetHull() const;

		// Casting (assumes ray in local space of parent body) 
		virtual RkShapeCastResult CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, float MaxAlpha ) const override;

		// Helpers
		virtual RkBounds3 ComputeBounds( const RkTransform& Transform ) const override;
		virtual RkMassProperties ComputeMassProperties() const override;
		virtual float GetMinMotionRadius() const override;
		virtual float GetMaxMotionRadius( const RkVector3& Center ) const override;

	private:
		RkHull* mHull;
	};