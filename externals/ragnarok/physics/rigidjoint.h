//--------------------------------------------------------------------------------------------------
/*
	@file		rigidjoint.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/joint.h"


//--------------------------------------------------------------------------------------------------
// RkRigidJoint
//--------------------------------------------------------------------------------------------------
class RkRigidJoint : public RkJoint
	{
	public:
		// Construction
		RkRigidJoint( RkWorld* World, RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision );

		// Relative target transform 
		void SetRelativeTransform( const RkTransform& RelT0 );
		RkTransform GetRelativeTransform() const;

		// Angular spring parameters
		void SetAngularFrequency( float Frequency );
		float GetAngularFrequency() const;
		void SetAngularDampingRatio( float DampingRatio );
		float GetAngularDampingRatio() const;
		void SetMaxTorque( float MaxTorque );
		float GetMaxTorque() const;

		// Linear spring parameters
		void SetLinearFrequency( float Frequency );
		float GetLinearFrequency() const;
		void SetLinearDampingRatio( float DampingRatio );
		float GetLinearDampingRatio() const;
		void SetMaxForce( float MaxForce );
		float GetMaxForce() const;

	ragnarok:
		// Solver interface
		virtual void LoadConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep ) override;
		virtual void SolveConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance ) override;
		virtual void SaveConstraints( RkJointConstraint* ConstraintBuffer ) override;
	
	private:
		RkTransform mRelT0;

		float mAngularFrequency;
		float mAngularDampingRatio;
		float mMaxTorque;

		float mLinearFrequency;
		float mLinearDampingRatio;
		float mMaxForce;

		RkVector3 mAngularLambda;
		RkVector3 mLinearLambda;
	};

