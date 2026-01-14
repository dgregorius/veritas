//--------------------------------------------------------------------------------------------------
/*
	@file		joint.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/jointtype.h"

#include "ragnarok/common/array.h"
#include "ragnarok/common/math.h"

class RkBody;
class RkIsland;
class RkJoint;
class RkMechanism;
class RkWorld;

struct RkCompliance;
struct RkJacobianEntry;
struct RkJointConstraint;
struct RkSolverBody;


//--------------------------------------------------------------------------------------------------
// RkJointEdge
//--------------------------------------------------------------------------------------------------
struct RkJointEdge
	{
	RkBody* Other;
	RkJoint* Joint;
	RkJointEdge* Prev;
	RkJointEdge* Next;
	};


//--------------------------------------------------------------------------------------------------
// RkJoint
//--------------------------------------------------------------------------------------------------
class RkJoint
	{
	public:
		// Construction / Destruction
		RkJoint( RkJointType Type, RkWorld* World, RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision );
		virtual ~RkJoint();

		// Type
		RkJointType GetType() const;

		// World 
		RkWorld* GetWorld() const;

		// Bodies
		RkBody* GetBody1() const;
		RkBody* GetBody2() const;

		// Constraint Frames
		RkVector3 GetOrigin1() const;
		RkVector3 GetLocalOrigin1() const;
		RkQuaternion GetBasis1() const;
		RkQuaternion GetLocalBasis1() const;
		RkTransform GetFrame1() const;
		RkTransform GetLocalFrame1() const;

		RkVector3 GetOrigin2() const;
		RkVector3 GetLocalOrigin2() const;
		RkQuaternion GetBasis2() const;
		RkQuaternion GetLocalBasis2() const;
		RkTransform GetFrame2() const;
		RkTransform GetLocalFrame2() const;

		// Collision
		bool IsCollisionEnabled() const;

		// User data
		void SetUserScale( float UserScale ) const;
		float GetUserScale() const;
		void SetUserData( void* UserData ) const;
		void* GetUserData() const;

	ragnarok:	
		// World
		int WorldIndex = -1;
		int SolverIndex = -1;
		int GraphIndex = -1;
		int ColorIndex = -1;

		// Mechanisms
		RkMechanism* GetMechanism() const;
		void SetMechanism( RkMechanism* Mechanism );

		// Islands
		RkIsland* GetIsland() const;
		void SetIsland( RkIsland* Island );
		int GetIslandIndex() const;
		void SetIslandIndex( int IslandIndex );
		
		// Solver
		virtual void LoadConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep ) = 0;
		virtual void SolveConstraints( RkJointConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance ) = 0;
		virtual void SaveConstraints( RkJointConstraint* ConstraintBuffer ) = 0;

	protected:
		RkJointType mType;

		RkWorld* mWorld;
		RkBody* mBody1;
		RkBody* mBody2;
		RkTransform mLocalFrame1;
		RkTransform mLocalFrame2;
		bool mIsCollisionEnabled;
		RkMechanism* mMechanism;

		RkJointEdge mEdge1;
		RkJointEdge mEdge2;

		RkIsland* mIsland;
		int mIslandIndex;

		mutable float mUserScale;
		mutable void* mUserData;
	};