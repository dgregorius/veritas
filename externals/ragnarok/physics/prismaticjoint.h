//--------------------------------------------------------------------------------------------------
/*
	@file		prismaticjoint.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/joint.h"


//--------------------------------------------------------------------------------------------------
// RkPrismaticJoint
//--------------------------------------------------------------------------------------------------
class RkPrismaticJoint : public RkJoint
	{
	public:
		// Construction
		RkPrismaticJoint( RkWorld* World, RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision );

		// Limit
		void EnableLimit();
		void DisableLimit();
		bool IsLimitEnabled() const;
		void GetLimit( float& MinOffset, float& MaxOffset ) const;
		void SetLimit( float MinOffset, float MaxOffset );
		float GetOffset() const;

	ragnarok:
		// Solver interface
		virtual void LoadConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep ) override;
		virtual void SolveConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance ) override;
		virtual void SaveConstraints( RkJointConstraint* ConstraintBuffer ) override;
	
	private:
		bool mIsLimitEnabled;
		float mMinOffset;
		float mMinLambda;
		float mMaxOffset;
		float mMaxLambda;

		RkVector3 mAngularLambda;
		RkVector2 mLinearLambda;
	};

