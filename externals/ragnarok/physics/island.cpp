//--------------------------------------------------------------------------------------------------
// island.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "island.h"
#include "body.h"
#include "contact.h"
#include "joint.h"
#include "mechanism.h"
#include "world.h"

// Internal
#include "solver.h"
#include "contactsolver.h"
#include "jointsolver.h"


//--------------------------------------------------------------------------------------------------
// RkIsland
//--------------------------------------------------------------------------------------------------
RkIsland::RkIsland( RkWorld* World )
	{
	RK_ASSERT( World );
	mWorld = World;
	
	mIsSleeping = false;
	mMinSleepTime = RK_F32_MAX;
	mMaxSleepTime = 0.0f;
	mRemovedConstraints = 0;
	}


//--------------------------------------------------------------------------------------------------
RkWorld* RkIsland::GetWorld() const
	{
	return mWorld;
	}


//--------------------------------------------------------------------------------------------------
int RkIsland::Size() const
	{
	return mBodies.Size() + mJoints.Size() + mContacts.Size();
	}


//--------------------------------------------------------------------------------------------------
bool RkIsland::Empty() const
	{
	return Size() == 0;
	}


//--------------------------------------------------------------------------------------------------
int RkIsland::AddBody( RkBody* Body )
	{
	RK_ASSERT( Body );
	RK_ASSERT( Body->GetType() != RK_STATIC_BODY );
	RK_ASSERT( Body->GetWorld() == mWorld );
	RK_ASSERT( Body->GetIsland() == this );
	RK_ASSERT( !mBodies.Contains( Body ) );

	return mBodies.PushBack( Body );
	}


//--------------------------------------------------------------------------------------------------
void RkIsland::RemoveBody( RkBody* Body )
	{
	RK_ASSERT( Body );
	RK_ASSERT( Body->GetWorld() == mWorld );
	RK_ASSERT( Body->GetIsland() == this );

	int IslandIndex = Body->GetIslandIndex();
	RK_ASSERT( mBodies[ IslandIndex ] == Body );
	RkBody* BackBody = mBodies.Back();
	mBodies[ IslandIndex ] = BackBody;
	BackBody->SetIslandIndex( IslandIndex );
	mBodies.PopBack();
	}


//--------------------------------------------------------------------------------------------------
int RkIsland::GetBodyCount() const
	{
	return mBodies.Size();
	}


//--------------------------------------------------------------------------------------------------
RkBody* RkIsland::GetBody( int BodyIndex )
	{
	return ( 0 <= BodyIndex && BodyIndex < mBodies.Size() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkBody* RkIsland::GetBody( int BodyIndex ) const
	{
	return ( 0 <= BodyIndex && BodyIndex < mBodies.Size() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkArray< RkBody* >& RkIsland::GetBodies()
	{
	return mBodies;
	}


//--------------------------------------------------------------------------------------------------
const RkArray< RkBody* >& RkIsland::GetBodies() const
	{
	return mBodies;
	}


//--------------------------------------------------------------------------------------------------
int RkIsland::AddContact( RkContact* Contact )
	{
	RK_ASSERT( Contact );
	RK_ASSERT( Contact->GetWorld() == mWorld );
	RK_ASSERT( Contact->GetIsland() == this );
	RK_ASSERT( !mContacts.Contains( Contact ) );

	return mContacts.PushBack( Contact );
	}


//--------------------------------------------------------------------------------------------------
void RkIsland::RemoveContact( RkContact* Contact )
	{
	RK_ASSERT( Contact );
	RK_ASSERT( Contact->GetWorld() == mWorld );
	RK_ASSERT( Contact->GetIsland() == this );

	int IslandIndex = Contact->GetIslandIndex();
	RK_ASSERT( mContacts[ IslandIndex ] == Contact );
	RkContact* BackContact = mContacts.Back();
	mContacts[ IslandIndex ] = BackContact;
	BackContact->SetIslandIndex( IslandIndex );
	mContacts.PopBack();

	mRemovedConstraints++;
	}


//--------------------------------------------------------------------------------------------------
int RkIsland::GetContactCount() const
	{
	return mContacts.Size();
	}


//--------------------------------------------------------------------------------------------------
RkContact* RkIsland::GetContact( int ContactIndex )
	{
	return ( 0 <= ContactIndex && ContactIndex < mContacts.Size() ) ? mContacts[ ContactIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkContact* RkIsland::GetContact( int ContactIndex ) const
	{
	return ( 0 <= ContactIndex && ContactIndex < mContacts.Size() ) ? mContacts[ ContactIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkArray< RkContact* >& RkIsland::GetContacts()
	{
	return mContacts;
	}


//--------------------------------------------------------------------------------------------------
const RkArray< RkContact* >& RkIsland::GetContacts() const
	{
	return mContacts;
	}


//--------------------------------------------------------------------------------------------------
int RkIsland::AddJoint( RkJoint* Joint )
	{
	RK_ASSERT( Joint );
	RK_ASSERT( Joint->GetWorld() == mWorld );
	RK_ASSERT( Joint->GetIsland() == this );
	RK_ASSERT( !mJoints.Contains( Joint ) );

	return mJoints.PushBack( Joint );
	}


//--------------------------------------------------------------------------------------------------
void RkIsland::RemoveJoint( RkJoint* Joint )
	{
	RK_ASSERT( Joint );
	RK_ASSERT( Joint->GetWorld() == mWorld );
	RK_ASSERT( Joint->GetIsland() == this );

	int IslandIndex = Joint->GetIslandIndex();
	RK_ASSERT( mJoints[ IslandIndex ] == Joint );
	RkJoint* BackJoint = mJoints.Back();
	mJoints[ IslandIndex ] = BackJoint;
	BackJoint->SetIslandIndex( IslandIndex );
	mJoints.PopBack();

	mRemovedConstraints++;
	}


//--------------------------------------------------------------------------------------------------
int RkIsland::GetJointCount() const
	{
	return mJoints.Size();
	}


//--------------------------------------------------------------------------------------------------
RkJoint* RkIsland::GetJoint( int JointIndex )
	{
	return ( 0 <= JointIndex && JointIndex < mJoints.Size() ) ? mJoints[ JointIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkJoint* RkIsland::GetJoint( int JointIndex ) const
	{
	return ( 0 <= JointIndex && JointIndex < mJoints.Size() ) ? mJoints[ JointIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkArray< RkJoint* >& RkIsland::GetJoints()
	{
	return mJoints;
	}


//--------------------------------------------------------------------------------------------------
const RkArray< RkJoint* >& RkIsland::GetJoints() const
	{
	return mJoints;
	}


//--------------------------------------------------------------------------------------------------
void RkIsland::WakeUp()
	{
	mIsSleeping = false;
	}


//--------------------------------------------------------------------------------------------------
void RkIsland::Sleep()
	{
	RK_ASSERT( mRemovedConstraints == 0 );
	mIsSleeping = true;
	}


//--------------------------------------------------------------------------------------------------
bool RkIsland::IsSleeping() const
	{
	return mIsSleeping;
	}


//--------------------------------------------------------------------------------------------------
RkIslandState RkIsland::GetState() const
	{
	return !mIsSleeping ? RK_ACTIVE_ISLAND : RK_SLEEPING_ISLAND;
	}


//--------------------------------------------------------------------------------------------------
void RkIsland::SetState( RkIslandState State )
	{
	mIsSleeping = State == RK_SLEEPING_ISLAND ? true : false;
	}


//--------------------------------------------------------------------------------------------------
float RkIsland::GetMinSleepTime() const
	{
	return mMinSleepTime;
	}


//--------------------------------------------------------------------------------------------------
float RkIsland::GetMaxSleepTime() const
	{
	return mMaxSleepTime;
	}


//--------------------------------------------------------------------------------------------------
bool RkIsland::HasRemovedConstraints() const
	{
	return mRemovedConstraints > 0;
	}


//--------------------------------------------------------------------------------------------------
int RkIsland::GetRemovedConstraints() const
	{
	return mRemovedConstraints;
	}


//--------------------------------------------------------------------------------------------------
void RkIsland::SetRemovedConstraints( int RemovedConstraints )
	{
	RK_ASSERT( RemovedConstraints >= 0 );
	mRemovedConstraints = RemovedConstraints;
	}


//--------------------------------------------------------------------------------------------------
bool RkIsland::IsConsistent() const
	{
	// Validate empty island
	if ( mBodies.Empty() )
		{
		if ( !mJoints.Empty() )
			{
			return false;
			}

		if ( !mContacts.Empty() )
			{
			return false;
			}

		return true;
		}

	// Validate non-empty island topology
	for ( RkBody* Body : mBodies )
		{
		if ( Body->GetIsland() != this )
			{
			return false;
			}

		if ( mBodies[ Body->GetIslandIndex() ] != Body )
			{
			return false;
			}
		}

	for ( RkJoint* Joint : mJoints )
		{
		if ( Joint->GetIsland() != this )
			{
			return false;
			}

		if ( mJoints[ Joint->GetIslandIndex() ] != Joint )
			{
			return false;
			}

		RkBody* Body1 = Joint->GetBody1();
		if ( Body1->GetType() != RK_STATIC_BODY && Body1->GetIsland() != this )
			{
			return false;
			}

		RkBody* Body2 = Joint->GetBody2();
		if ( Body2->GetType() != RK_STATIC_BODY && Body2->GetIsland() != this )
			{
			return false;
			}
		}

	for ( RkContact* Contact : mContacts )
		{
		if ( Contact->GetIsland() != this )
			{
			return false;
			}

		if ( mContacts[ Contact->GetIslandIndex() ] != Contact )
			{
			return false;
			}

		RkBody* Body1 = Contact->GetBody1();
		if ( Body1->GetType() != RK_STATIC_BODY && Body1->GetIsland() != this )
			{
			return false;
			}

		RkBody* Body2 = Contact->GetBody2();
		if ( Body2->GetType() != RK_STATIC_BODY && Body2->GetIsland() != this )
			{
			return false;
			}
		}

	return true;
	}