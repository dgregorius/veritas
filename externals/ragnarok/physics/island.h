
//--------------------------------------------------------------------------------------------------
/*
	@file		island.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/islandstate.h"

#include "ragnarok/common/array.h"
#include "ragnarok/common/types.h"

class RkBody;
class RkContact;
class RkJoint;
class RkMechanism;
class RkWorld;


//--------------------------------------------------------------------------------------------------
// RkIsland
//--------------------------------------------------------------------------------------------------
class RkIsland
	{
	public:
		// Construction / Destruction
		explicit RkIsland( RkWorld* World );
		~RkIsland() = default;

		// World
		RkWorld* GetWorld() const;

		// Accessors
		int Size() const;
		bool Empty() const;

		// Bodies
		int AddBody( RkBody* Body );
		void RemoveBody( RkBody* Body );

		int GetBodyCount() const;
		RkBody* GetBody( int BodyIndex );
		const RkBody* GetBody( int BodyIndex ) const;
	    RkArray< RkBody* >& GetBodies();
		const RkArray< RkBody* >& GetBodies() const;

		// Contacts
		int AddContact( RkContact* Contact );
		void RemoveContact( RkContact* Contact );

		int GetContactCount() const;
		RkContact* GetContact( int ContactIndex );
		const RkContact* GetContact( int ContactIndex ) const;
		RkArray< RkContact* >& GetContacts();
		const RkArray< RkContact* >& GetContacts() const;

		// Joints
		int AddJoint( RkJoint* Joint );
		void RemoveJoint( RkJoint* Joint );

		int GetJointCount() const;
		RkJoint* GetJoint( int JointIndex );
		const RkJoint* GetJoint( int JointIndex ) const;
		RkArray< RkJoint* >& GetJoints();
		const RkArray< RkJoint* >& GetJoints() const;

		// Sleeping
		void WakeUp();
		void Sleep();
		bool IsSleeping() const;

		RkIslandState GetState() const;
		void SetState( RkIslandState State );

		float GetMinSleepTime() const;
		float GetMaxSleepTime() const;

		// Splitting
		bool HasRemovedConstraints() const;
		int GetRemovedConstraints() const;
		void SetRemovedConstraints( int RemovedConstraints );

		// Validation
		bool IsConsistent() const;

	ragnarok:
		int WorldIndex = -1;
		
	private:
		RkWorld* mWorld;
		RkArray< RkBody* > mBodies;
		RkArray< RkContact* > mContacts;
		RkArray< RkJoint* > mJoints;

		bool mIsSleeping;
		float mMinSleepTime;
		float mMaxSleepTime;
		
		int mRemovedConstraints;
	};
