//--------------------------------------------------------------------------------------------------
/*
	@file		world.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/body.h"
#include "ragnarok/physics/constraintgraph.h"
#include "ragnarok/physics/contact.h"
#include "ragnarok/physics/filter.h"
#include "ragnarok/physics/island.h"
#include "ragnarok/physics/joint.h"
#include "ragnarok/physics/mechanism.h"
#include "ragnarok/physics/worldsample.h"

#include "ragnarok/common/arena.h"
#include "ragnarok/common/array.h"
#include "ragnarok/common/bitset.h"
#include "ragnarok/common/math.h"
#include "ragnarok/common/pool.h"
#include "ragnarok/common/timer.h"
#include "ragnarok/common/tracy.h"

// Scheduler
#include <taskflow/taskflow.hpp>
#include <taskflow/algorithm/for_each.hpp>

using RkBodyList = RkIndexedArray< RkBody, &RkBody::WorldIndex >;
using RkContactList = RkIndexedArray< RkContact, &RkContact::WorldIndex >;
using RkJointList = RkIndexedArray< RkJoint, &RkJoint::WorldIndex >;
using RkMechanismList = RkIndexedArray< RkMechanism, &RkMechanism::WorldIndex >;
using RkIslandList = RkIndexedArray< RkIsland, &RkIsland::WorldIndex >;

class RkBroadphase;
class RkSphericalJoint;
class RkRevoluteJoint;
class RkPrismaticJoint;
class RkRigidJoint;
class RkMechanism;

class RkArena;
class RkPool;


//--------------------------------------------------------------------------------------------------
// RkWorldCastResult
//--------------------------------------------------------------------------------------------------
struct RkWorldCastResult
	{
	RkBody* HitBody = nullptr;
	RkShape* HitShape = nullptr;

	float HitTime = 1.0f;
	RkVector3 HitPoint = RK_VEC3_ZERO;
	RkVector3 HitNormal = RK_VEC3_ZERO;
	int Triangle = -1;

	inline bool DidHit() const
		{
		return HitTime < 1.0f;
		}
	};


//--------------------------------------------------------------------------------------------------
// RkWorld
//--------------------------------------------------------------------------------------------------
class RkWorld
	{
	public:
		// Construction / Destruction
		explicit RkWorld( tf::Executor* Executor );
		~RkWorld();

		// Broadphase
		RkBroadphase* GetBroadphase();
		const RkBroadphase* GetBroadphase() const;

		// Gravity
		void SetGravity( const RkVector3& Gravity );
		RkVector3 GetGravity() const;

		// Bodies
		RkBody* AddBody();
		bool RemoveBody( RkBody*& Body );

		int GetBodyCount() const;
		RkBody* GetBody( int BodyIndex );
		const RkBody* GetBody( int BodyIndex ) const;
		const RkBodyList& GetBodies() const;

		// Contacts
		void GetContactParameters( float& Frequency, float& DampingRatio ) const;
		void SetContactParameters( float Frequency, float DampingRatio );

		RkContact* AddContact( RkShape* Shape1, RkShape* Shape2 );
		bool RemoveContact( std::derived_from< RkContact > auto*& Contact );
		
		int GetContactCount() const;
		RkContact* GetContact( int ContactIndex );
		const RkContact* GetContact( int ContactIndex ) const;
		const RkContactList& GetContacts() const;

		int GetActiveContactCount() const;
		RkContact* GetActiveContact( int ActiveIndex );
		const RkContact* GetActiveContact( int ActiveIndex ) const;

		// Joints
		void GetJointParameters( float& Frequency, float& DampingRatio ) const;
		void SetJointParameters( float Frequency, float DampingRatio );

		RkSphericalJoint* AddSphericalJoint( RkBody* Body1, RkBody* Body2, const RkVector3& Pivot, bool EnableCollision = false );
		RkSphericalJoint* AddSphericalJoint( RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision = false );
		RkRevoluteJoint* AddRevoluteJoint( RkBody* Body1, RkBody* Body2, const RkVector3& Pivot, const RkVector3& Axis, bool EnableCollision = false );
		RkRevoluteJoint* AddRevoluteJoint( RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision = false );
		RkPrismaticJoint* AddPrismaticJoint( RkBody* Body1, RkBody* Body2, const RkVector3& Pivot, const RkVector3& Axis, bool EnableCollision = false );
		RkPrismaticJoint* AddPrismaticJoint( RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision = false );
		RkRigidJoint* AddRigidJoint( RkBody* Body1, RkBody* Body2, const RkVector3& Pivot, bool EnableCollision = false );
		RkRigidJoint* AddRigidJoint( RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision = false );
		bool RemoveJoint( std::derived_from< RkJoint > auto*& Joint );

		int GetJointCount() const;
		RkJoint* GetJoint( int JointIndex );
		const RkJoint* GetJoint( int JointIndex ) const;
		const RkJointList& GetJoints() const;

		// Mechanisms
		RkMechanism* AddMechanism( const RkArray< RkJoint* >& Joints );
		bool RemoveMechanism( RkMechanism*& Mechanism );

		int GetMechanismCount() const;
		RkMechanism* GetMechanism( int MechanismIndex );
		const RkMechanism* GetMechanism( int MechanismIndex ) const;
		const RkMechanismList& GetMechanisms() const;

		// Sleeping
		bool IsAutoSleepingEnabled() const;
		void EnableAutoSleeping( bool Enable );
		bool IsSleeping() const;

		int GetIslandCount( RkIslandState State ) const;
		RkIsland* GetIsland( RkIslandState State, int IslandIndex );
		const RkIsland* GetIsland( RkIslandState State, int IslandIndex ) const;
		const RkIslandList& GetIslands( RkIslandState State ) const;

		// Queries
		RkWorldCastResult CastRay( const RkVector3& RayStart, const RkVector3& RayEnd ) const;
		RkWorldCastResult CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, RkFilter Filter ) const;

		// Simulation
		void Step( int Iterations, float Timestep );
		const RkWorldSample& GetSample() const;

		// User data
		void* GetUserData() const;
		void SetUserData( void* UserData ) const;

	ragnarok:
		// Memory
		RkPool* GetIslandPool();
		RkPool* GetShapePool();
		RkPool* GetBodyPool();
		RkPool* GetContactPool();
		RkPool* GetJointPool();
		RkPool* GetMechanismPool();

		// Islands
		RkIsland* AddIsland();
		bool RemoveIsland( RkIsland*& Island );
		RkIsland* MergeIslands( RkIsland* Island1, RkIsland* Island2 );
		void SplitIsland( RkIsland* Island );
		void SplitIsland1( RkIsland* Island );
		void SplitIsland2( RkIsland* Island );
		void SleepIsland( RkIsland* Island );
		void WakeIsland( RkIsland* Island );

		void LinkToIsland( RkBody* Body );
		void UnlinkFromIsland( RkBody* Body );
		void LinkToIsland( RkContact* Contact );
		void UnlinkFromIsland( RkContact* Contact );
		void LinkToIsland( RkJoint* Joint );
		void UnlinkFromIsland( RkJoint* Joint );

		// Solver
		void Broadphase();
		void Narrowphase();
		void Solve( int Iterations, float Timestep );
		
		void IntegrateForces( RkSolverBody* BodyBuffer, float Timestep );
		void LoadConstraints( RkConstraintBuffer& ConstraintBuffer, RkSolverBody* BodyBuffer, float Timestep );
		void SolveConstraints( RkConstraintBuffer& ConstraintBuffer, RkSolverBody* BodyBuffer, int Iterations, float Timestep );
		void SaveConstraints( RkConstraintBuffer& ConstraintBuffer );
		void IntegrateVelocities( RkSolverBody* BodyBuffer, float Timestep );
		void RefitProxies( float Timestep );
		void Sleeping( float Timestep );
		
		const RkConstraintGraph& GetConstraintGraph() const;

	private:
		tf::Executor* mExecutor;

		RkBroadphase* mBroadphase;
		std::future< void > mRebuildTask;
		bool mRebuilding;

		RkVector3 mGravity;
		RkBodyList mBodyList;
		float mContactFrequency, mContactDampingRatio;
		RkContactMap mContactMap;
		RkContactList mContactList;
		float mJointFrequency, mJointDampingRatio;
		RkJointList mJointList;
		RkMechanismList mMechanismList;
		
		bool mAutoSleeping;
		RkIslandList mIslandList[ RK_ISLAND_STATE_COUNT ];
		RkBitset mActiveIslandSet;
		RkIsland* mSplitIsland;
		std::future< void > mSplitTask;
		bool mSplitting;

		RkIndexedArray< RkContact, &RkContact::CollisionIndex > mCollisionSet;
		RkIndexedArray< RkBody, &RkBody::SolverIndex > mBodySet;
		RkIndexedArray< RkContact, &RkContact::SolverIndex > mContactSet;
		RkIndexedArray< RkJoint, &RkJoint::SolverIndex > mJointSet;
		RkConstraintGraph mConstraintGraph;

		RkPool* mIslandPool;
		RkPool* mShapePool;
		RkPool* mBodyPool;
		RkPool* mContactPool;
		RkPool* mJointPool;
		RkPool* mMechanismPool;
		RkArena* mMainArena;

		mutable RkWorldSample mSample;
		mutable void* mUserData;
	};
