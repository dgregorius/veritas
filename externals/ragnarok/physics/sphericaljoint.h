//--------------------------------------------------------------------------------------------------
/*
	@file		sphericaljoint.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/joint.h"


//--------------------------------------------------------------------------------------------------
// RkSphericalJoint
//--------------------------------------------------------------------------------------------------
class RkSphericalJoint : public RkJoint
	{
	public:
		// Construction
		RkSphericalJoint( RkWorld* World, RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision );

		// Twist
		void EnableTwistLimit();
		void DisableTwistLimit();
		bool IsTwistLimitEnabled() const;
		void GetTwistLimit( float& MinTwist, float& MaxTwist ) const;
		void SetTwistLimit( float MinTwist, float MaxTwist );
		float GetTwistAngle() const;

		// Swing
		void EnableSwingLimit();
		void DisableSwingLimit();
		bool IsSwingLimitEnabled() const;
		float GetSwingLimit() const;
		void SetSwingLimit( float MaxSwing );
		float GetSwingAngle() const;

	ragnarok:
		// Solver interface
		virtual void LoadConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep ) override;
		virtual void SolveConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance ) override;
		virtual void SaveConstraints( RkJointConstraint* ConstraintBuffer ) override;
	
	private:
		bool mIsTwistLimitEnabled;
		float mMinTwist;
		float mMinTwistLambda;
		float mMaxTwist;
		float mMaxTwistLambda;

		bool mIsSwingLimitEnabled;
		float mMaxSwing;
		float mMaxSwingLambda;

		RkVector3 mLinearLambda;
	};

