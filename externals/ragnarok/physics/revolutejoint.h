//--------------------------------------------------------------------------------------------------
/*
	@file		revolutejoint.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/joint.h"


//--------------------------------------------------------------------------------------------------
// RkRevoluteJoint
//--------------------------------------------------------------------------------------------------
class RkRevoluteJoint : public RkJoint
	{
	public:
		// Construction
		RkRevoluteJoint( RkWorld* World, RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision );

		// Limit
		void EnableLimit();
		void DisableLimit();
		bool IsLimitEnabled() const;
		void GetLimit( float& MinAngle, float& MaxAngle ) const;
		void SetLimit( float MinAngle, float MaxAngle );
		float GetAngle() const;
		
	ragnarok:
		// Solver interface
		virtual void LoadConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep ) override;
		virtual void SolveConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance ) override;
		virtual void SaveConstraints( RkJointConstraint* ConstraintBuffer ) override;
	
	private:
		bool mIsLimitEnabled;
		float mMinAngle;
		float mMinLambda;
		float mMaxAngle;
		float mMaxLambda;

		RkVector2 mAngularLambda;
		RkVector3 mLinearLambda;
	};

