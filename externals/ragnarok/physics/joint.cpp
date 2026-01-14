//--------------------------------------------------------------------------------------------------
// joint.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "joint.h"
#include "body.h"
#include "island.h"
#include "world.h"


//--------------------------------------------------------------------------------------------------
// RkJoint
//--------------------------------------------------------------------------------------------------
RkJoint::RkJoint( RkJointType Type, RkWorld* World, RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision )
	{
	mType = Type;

	RK_ASSERT( World );
	mWorld = World;
	RK_ASSERT( Body1 );
	RK_ASSERT( Body1->GetWorld() == World );
	RK_ASSERT( Body2 );
	RK_ASSERT( Body2->GetWorld() == World );
	RK_ASSERT( Body1 != Body2 );
	mBody1 = Body1;
	mLocalFrame1 = LocalFrame1;
	mBody2 = Body2;
	mLocalFrame2 = LocalFrame2;
	mIsCollisionEnabled = EnableCollision;
	mMechanism = nullptr;
	
	// Register this joint with the associated bodies
	mEdge1.Other = Body2;
	mEdge1.Joint = this;
	mEdge1.Prev = nullptr;  
	mEdge1.Next = nullptr;
	Body1->AddJoint( &mEdge1 );

	mEdge2.Other = Body1;
	mEdge2.Joint = this;
	mEdge2.Prev = nullptr;  
	mEdge2.Next = nullptr;
	Body2->AddJoint( &mEdge2 );

	// Joints must be explicitly added to islands
	mIsland = nullptr;
	mIslandIndex = -1;

	// User data
	mUserScale = 1.0f;
	mUserData = nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkJoint::~RkJoint()
	{
	RK_ASSERT( !mMechanism );

	// Unregister this joint from the associated bodies
	mBody2->RemoveJoint( &mEdge2 );
	mBody1->RemoveJoint( &mEdge1 );
	}


//--------------------------------------------------------------------------------------------------
RkJointType RkJoint::GetType() const
	{
	return mType;
	}


//--------------------------------------------------------------------------------------------------
RkWorld* RkJoint::GetWorld() const
	{
	return mWorld;
	}


//--------------------------------------------------------------------------------------------------
RkBody* RkJoint::GetBody1() const
	{
	return mBody1;
	}


//--------------------------------------------------------------------------------------------------
RkBody* RkJoint::GetBody2() const
	{
	return mBody2;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkJoint::GetOrigin1() const
	{
	return mBody1->TransformPointToWorld( mLocalFrame1.Translation );
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkJoint::GetLocalOrigin1() const
	{
	return mLocalFrame1.Translation;
	}


//--------------------------------------------------------------------------------------------------
RkQuaternion RkJoint::GetBasis1() const
	{
	return mBody1->TransformBasisToWorld( mLocalFrame1.Rotation );
	}


//--------------------------------------------------------------------------------------------------
RkQuaternion RkJoint::GetLocalBasis1() const
	{
	return mLocalFrame1.Rotation;
	}


//--------------------------------------------------------------------------------------------------
RkTransform RkJoint::GetFrame1() const
	{
	return mBody1->GetTransform() * mLocalFrame1;
	}


//--------------------------------------------------------------------------------------------------
RkTransform RkJoint::GetLocalFrame1() const
	{
	return mLocalFrame1;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkJoint::GetOrigin2() const
	{
	return mBody2->TransformPointToWorld( mLocalFrame2.Translation );
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkJoint::GetLocalOrigin2() const
	{
	return mLocalFrame2.Translation;
	}


//--------------------------------------------------------------------------------------------------
RkQuaternion RkJoint::GetBasis2() const
	{
	return mBody2->TransformBasisToWorld( mLocalFrame2.Rotation );
	}


//--------------------------------------------------------------------------------------------------
RkQuaternion RkJoint::GetLocalBasis2() const
	{
	return mLocalFrame2.Rotation;
	}


//--------------------------------------------------------------------------------------------------
RkTransform RkJoint::GetFrame2() const
	{
	return mBody2->GetTransform() * mLocalFrame2;
	}


//--------------------------------------------------------------------------------------------------
RkTransform RkJoint::GetLocalFrame2() const
	{
	return mLocalFrame2;
	}


//--------------------------------------------------------------------------------------------------
bool RkJoint::IsCollisionEnabled() const
	{
	return mIsCollisionEnabled;;
	}


//--------------------------------------------------------------------------------------------------
void RkJoint::SetUserScale( float UserScale ) const
	{
	RK_ASSERT( UserScale >= 0.0f );
	mUserScale = UserScale;
	}


//--------------------------------------------------------------------------------------------------
float RkJoint::GetUserScale() const
	{
	return mUserScale;
	}


//--------------------------------------------------------------------------------------------------
void RkJoint::SetUserData( void* UserData ) const
	{
	mUserData = UserData;
	}


//--------------------------------------------------------------------------------------------------
void* RkJoint::GetUserData() const
	{
	return mUserData;
	}


//--------------------------------------------------------------------------------------------------
RkMechanism* RkJoint::GetMechanism() const
	{
	return mMechanism;
	}


//--------------------------------------------------------------------------------------------------
void RkJoint::SetMechanism( RkMechanism* Mechanism )
	{
	mMechanism = Mechanism;
	}


//--------------------------------------------------------------------------------------------------
RkIsland* RkJoint::GetIsland() const
	{
	return mIsland;
	}


//--------------------------------------------------------------------------------------------------
void RkJoint::SetIsland( RkIsland* Island )
	{
	mIsland = Island;
	}


//--------------------------------------------------------------------------------------------------
int RkJoint::GetIslandIndex() const
	{
	return mIslandIndex;
	}


//--------------------------------------------------------------------------------------------------
void RkJoint::SetIslandIndex( int IslandIndex )
	{
	RK_ASSERT( IslandIndex < 0 || mIsland->GetJoint( IslandIndex ) == this );
	mIslandIndex = IslandIndex;
	}