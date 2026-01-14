//--------------------------------------------------------------------------------------------------
/*
	@file		capsuleshape.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/shape.h"
#include "ragnarok/physics/capsule.h"


//--------------------------------------------------------------------------------------------------
// RkCapsuleShape
//--------------------------------------------------------------------------------------------------
class RkCapsuleShape : public RkShape
	{
	public:
		// Construction 
		RkCapsuleShape( RkBody* Body, const RkCapsule& Capsule );

		// Capsule
		RkCapsule GetCapsule() const;

		// Casting (assumes ray in local space of parent body) 
		virtual RkShapeCastResult CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, float MaxAlpha ) const override;

		// Helpers
		virtual RkBounds3 ComputeBounds( const RkTransform& Transform ) const override;
		virtual RkMassProperties ComputeMassProperties() const override;
		virtual float GetMinMotionRadius() const override;
		virtual float GetMaxMotionRadius( const RkVector3& Center ) const override;

	private:
		RkCapsule mCapsule;
	};