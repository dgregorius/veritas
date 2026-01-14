//--------------------------------------------------------------------------------------------------
// world.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "world.h"
#include "body.h"
#include "broadphase.h"
#include "event.h"
#include "island.h"

#include "shape.h"
#include "sphereshape.h"
#include "capsuleshape.h"
#include "hullshape.h"
#include "meshshape.h"

#include "joint.h"
#include "sphericaljoint.h"
#include "revolutejoint.h"
#include "prismaticjoint.h"
#include "rigidjoint.h"

#include "contact.h"
#include "convexcontact.h"
#include "meshcontact.h"

// Internal 
#include "contactsolver.h"
#include "jointsolver.h"


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static bool rkShouldCollide( const RkBody* Body1, const RkBody* Body2 )
	{
	// At least one body must be dynamic
	if ( Body1->GetType() != RK_DYNAMIC_BODY && Body2->GetType() != RK_DYNAMIC_BODY )
		{
		return false;
		}

	// Does a joint prevent collision (make sure we search the shorter list!)
	if ( Body1->GetJointCount() > Body2->GetJointCount() )
		{
		std::swap( Body1, Body2 );
		}

	for ( const RkJointEdge* Edge = Body1->GetJointList(); Edge; Edge = Edge->Next )
		{
		if ( Edge->Other == Body2 )
			{
			const RkJoint* Joint = Edge->Joint;
			if ( !Joint->IsCollisionEnabled() )
				{
				return false;
				}
			}
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
static bool rkShouldCollide( const RkShape* Shape1, const RkShape* Shape2 )
	{
	// No collision between meshes
	if ( Shape1->GetType() == RK_MESH_SHAPE && Shape2->GetType() == RK_MESH_SHAPE )
		{
		return false;
		}

	return rkShouldCollide( Shape1->GetFilter(), Shape2->GetFilter() );
	}


//--------------------------------------------------------------------------------------------------
static void rkRemoveContacts( RkWorld* World, RkBody* Body1, RkBody* Body2 )
	{
	// Make sure we search the shorter list
	if ( Body1->GetContactCount() > Body2->GetContactCount() )
		{
		std::swap( Body1, Body2 );
		}

	// Search list and remove all contacts between body1 and body2
	RkContactEdge* Edge = Body1->GetContactList();
	while ( Edge )
		{
		RkContact* Contact = Edge->Contact;
		RkBody* Other = Edge->Other;
		Edge = Edge->Next;

		if ( Other == Body2 )
			{
			World->RemoveContact( Contact );
			}
		}
	}


//--------------------------------------------------------------------------------------------------
// RkWorld
//--------------------------------------------------------------------------------------------------
RkWorld::RkWorld( tf::Executor* Executor )
	{
	RK_ASSERT( Executor );
	mExecutor = Executor;
	
	mBroadphase = new RkBroadphase( this, Executor );
	mRebuilding = false;
	
	mGravity = RkVector3( 0.0f, -10.0f, 0.0f );
	mContactFrequency = 15.0f;
	mContactDampingRatio = 7.5f;
	mJointFrequency = 30.0f;
	mJointDampingRatio = 2.0f;

	mAutoSleeping = true;
	mSplitIsland = nullptr;
	mSplitting = false;

	mIslandPool = new RkPool( sizeof( RkIsland ) );
	mShapePool = new RkPool( std::max( { sizeof( RkSphereShape ), sizeof( RkCapsuleShape ), sizeof( RkHullShape ), sizeof( RkMeshShape ) } ) );
	mBodyPool = new RkPool( sizeof( RkBody ) );
	mContactPool = new RkPool( std::max( sizeof( RkConvexContact ), sizeof( RkMeshContact ) ) );
	mJointPool = new RkPool( std::max( { sizeof( RkSphericalJoint ), sizeof( RkRevoluteJoint ), sizeof( RkPrismaticJoint ), sizeof( RkRigidJoint ) } ) );
	mMechanismPool = new RkPool( sizeof( RkMechanism ) );

	const size_t ArenaSize = 4ULL * 1024 * 1024 * 1024;
	mMainArena = new RkArena( ArenaSize );

	mUserData = nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkWorld::~RkWorld()
	{
	delete mMainArena;

	while ( !mMechanismList.Empty() )
		{
		RkMechanism* Mechanism = mMechanismList.Back();
		RemoveMechanism( Mechanism );
		}
	delete mMechanismPool;

	while ( !mJointList.Empty() )
		{
		RkJoint* Joint = mJointList.Back();
		RemoveJoint( Joint );
		}
	delete mJointPool;

	while ( !mContactList.Empty() )
		{
		RkContact* Contact = mContactList.Back();
		RemoveContact( Contact );
		}
	delete mContactPool;

	while ( !mBodyList.Empty() )
		{
		RkBody* Body = mBodyList.Back();
		RemoveBody( Body );
		}
	delete mBodyPool;
	delete mShapePool;

	RK_ASSERT( mIslandList[ 0 ].Empty() );
	RK_ASSERT( mIslandList[ 1 ].Empty() );
	delete mIslandPool;

	delete mBroadphase;
	}


//--------------------------------------------------------------------------------------------------
RkBroadphase* RkWorld::GetBroadphase()
	{
	return mBroadphase;
	}


//--------------------------------------------------------------------------------------------------
const RkBroadphase* RkWorld::GetBroadphase() const
	{
	return mBroadphase;
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::SetGravity( const RkVector3& Gravity )
	{
	mGravity = Gravity;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkWorld::GetGravity() const
	{
	return mGravity;
	}


//--------------------------------------------------------------------------------------------------
RkBody* RkWorld::AddBody()
	{
	void* Address = mBodyPool->Allocate();
	RkBody* Body = new ( Address) RkBody( this );
	RK_ASSERT( Body->GetType() == RK_STATIC_BODY );
	mBodyList.PushBack( Body );
	
	RkEvent::BodyAdded.Emit( this, Body );

	return Body;
	}


//--------------------------------------------------------------------------------------------------
bool RkWorld::RemoveBody( RkBody*& Body )
	{
	if ( !Body )
		{
		return false;
		}

	if ( Body->GetWorld() != this )
		{
		return false;
		}

	// Delete attached contacts
		{
		RkContactEdge* Edge = Body->GetContactList();
		while ( Edge  )
			{
			// Pop next contact
			RkContact* Contact = Edge->Contact;
			Edge = Edge->Next;

			// Destroy contact (this will wake up associated bodies)
			RemoveContact( Contact );
			}
		}

	// Delete attached joints
		{
		RkJointEdge* Edge = Body->GetJointList();
		while ( Edge )
			{
			// Pop next joint
			RkJoint* Joint = Edge->Joint;
			Edge = Edge->Next;

			// Destroy joint (this will wake up associated bodies)
			RemoveJoint( Joint );
			}
		}
		
	// Remove from body list
	UnlinkFromIsland( Body );
	RK_ASSERT( mBodyList.Contains( Body ) );
	mBodyList.Remove( Body );

	RkEvent::BodyRemoved.Emit( this, Body );

	std::destroy_at( Body );
	mBodyPool->Deallocate( Body );
	Body = nullptr;

	return true;
	}


//--------------------------------------------------------------------------------------------------
int RkWorld::GetBodyCount() const
	{
	return mBodyList.Size();
	}


//--------------------------------------------------------------------------------------------------
RkBody* RkWorld::GetBody( int BodyIndex )
	{
	return ( 0 <= BodyIndex && BodyIndex < mBodyList.Size() ) ? mBodyList[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkBody* RkWorld::GetBody( int BodyIndex ) const
	{
	return ( 0 <= BodyIndex && BodyIndex < mBodyList.Size() ) ? mBodyList[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkBodyList& RkWorld::GetBodies() const
	{
	return mBodyList;
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::GetContactParameters( float& Frequency, float& DampingRatio ) const
	{
	Frequency = mContactFrequency;
	DampingRatio = mContactDampingRatio;
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::SetContactParameters( float Frequency, float DampingRatio )
	{
	RK_ASSERT( 0.0f < Frequency && Frequency <= 15.0f );
	mContactFrequency = Frequency;
	RK_ASSERT( DampingRatio > 0.0f );
	mContactDampingRatio = DampingRatio;
	}


//--------------------------------------------------------------------------------------------------
RkContact* RkWorld::AddContact( RkShape* Shape1, RkShape* Shape2 )
	{
	RK_ASSERT( Shape1 );
	RK_ASSERT( Shape2 );
	RK_ASSERT( Shape1 != Shape2 );
	RK_ASSERT( rkShouldCollide( Shape1, Shape2 ) );

	RkBody* Body1 = Shape1->GetBody();
	RkBody* Body2 = Shape2->GetBody();
	RK_ASSERT( Body1->GetWorld() == this );
	RK_ASSERT( Body2->GetWorld() == this );
	RK_ASSERT( Body1 != Body2 );
	RK_ASSERT( rkShouldCollide( Body1, Body2 ) );
	RK_ASSERT( !mContactMap.Contains( RkShapePair( Shape1, Shape2 ) ) );
		
	// Assure correct ordering to simplify collision matrix
	if ( Shape1->GetType() > Shape2->GetType() )
		{
		std::swap( Shape1, Shape2 );
		std::swap( Body1, Body2 );
		}

	// Dispatch contact type
	void* Address = mContactPool->Allocate();
	RkContact* Contact = Shape2->GetType() != RK_MESH_SHAPE ? static_cast< RkContact* >( new ( Address ) RkConvexContact( this, Shape1, Shape2 ) ) : static_cast< RkContact* >( new ( Address ) RkMeshContact( this, Shape1, Shape2 ) );
	mContactMap.Insert( RkShapePair( Shape1, Shape2 ), Contact );
	mContactList.PushBack( Contact );
	mCollisionSet.PushBack( Contact );

	return Contact;
	}


//--------------------------------------------------------------------------------------------------
bool RkWorld::RemoveContact( std::derived_from< RkContact > auto*& Contact )
	{
	if ( !Contact )
		{
		return false;
		}

	if ( Contact->GetWorld() != this )
		{
		return false;
		}
	
	// DIRK_TODO: Validate order of next three operations (WakeUp has side effects)
	if ( Contact->IsTouching() )
		{
		RkBody* Body1 = Contact->GetBody1();
		Body1->WakeUp();
		RkBody* Body2 = Contact->GetBody2();
		Body2->WakeUp();

		RkEvent::TouchEnd.Emit( this, Contact );
		}

	if ( Contact->IsActive() )
		{
		mCollisionSet.Remove( Contact );
		}

	UnlinkFromIsland( Contact );
	RK_ASSERT( mContactList.Contains( Contact ) );
	mContactList.Remove( Contact );
	RK_ASSERT( mContactMap.Contains( RkShapePair( Contact->GetShape1(), Contact->GetShape2() ) ) );
	mContactMap.Remove( RkShapePair( Contact->GetShape1(), Contact->GetShape2() ) );
	
	std::destroy_at( Contact );
	mContactPool->Deallocate( Contact );
	Contact = nullptr;

	return true;
	}

// Specializations
template bool RkWorld::RemoveContact< RkContact >( RkContact*& );
template bool RkWorld::RemoveContact< RkConvexContact >( RkConvexContact*& );
template bool RkWorld::RemoveContact< RkMeshContact >( RkMeshContact*& );


//--------------------------------------------------------------------------------------------------
int RkWorld::GetContactCount() const
	{
	return mContactList.Size();
	}


//--------------------------------------------------------------------------------------------------
RkContact* RkWorld::GetContact( int ContactIndex )
	{
	return ( 0 <= ContactIndex && ContactIndex < mContactList.Size() ) ? mContactList[ ContactIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkContact* RkWorld::GetContact( int ContactIndex ) const
	{
	return ( 0 <= ContactIndex && ContactIndex < mContactList.Size() ) ? mContactList[ ContactIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkContactList& RkWorld::GetContacts() const
	{
	return mContactList;
	}


//--------------------------------------------------------------------------------------------------
int RkWorld::GetActiveContactCount() const
	{
	return mCollisionSet.Size();
	}


//--------------------------------------------------------------------------------------------------
RkContact* RkWorld::GetActiveContact( int ActiveIndex )
	{
	return ( 0 <= ActiveIndex && ActiveIndex < mCollisionSet.Size() ) ? mCollisionSet[ ActiveIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkContact* RkWorld::GetActiveContact( int ActiveIndex ) const
	{
	return ( 0 <= ActiveIndex && ActiveIndex < mCollisionSet.Size() ) ? mCollisionSet[ ActiveIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::GetJointParameters( float& Frequency, float& DampingRatio ) const
	{
	Frequency = mJointFrequency;
	DampingRatio = mJointDampingRatio;
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::SetJointParameters( float Frequency, float DampingRatio )
	{
	RK_ASSERT( 0.0f < Frequency && Frequency <= 15.0f );
	mJointFrequency = Frequency;
	RK_ASSERT( DampingRatio > 0.0f );
	mJointDampingRatio = DampingRatio;
	}


//--------------------------------------------------------------------------------------------------
RkSphericalJoint* RkWorld::AddSphericalJoint( RkBody* Body1, RkBody* Body2, const RkVector3& Pivot, bool EnableCollision )
	{
	RkTransform LocalFrame1;
	LocalFrame1.Translation = Body1->TransformPointToLocal( Pivot );
	LocalFrame1.Rotation = RK_QUAT_IDENTITY;

	RkTransform LocalFrame2;
	LocalFrame2.Translation = Body2->TransformPointToLocal( Pivot );
	LocalFrame2.Rotation = RK_QUAT_IDENTITY;

	return AddSphericalJoint( Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision );
	}


//--------------------------------------------------------------------------------------------------
RkSphericalJoint* RkWorld::AddSphericalJoint( RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision )
	{
	if ( !Body1 || Body1->GetWorld() != this )
		{
		return nullptr;
		}

	if ( !Body2 || Body2->GetWorld() != this )
		{
		return nullptr;
		}

	if ( Body1 == Body2 )
		{
		return nullptr;
		}

	void* Address = mJointPool->Allocate();
	RkSphericalJoint* Joint = new ( Address ) RkSphericalJoint( this, Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision );
	if ( !EnableCollision )
		{
		// If this joint prevents collision, remove contacts between associated bodies!
		rkRemoveContacts( this, Body1, Body2 );
		}

	if ( Body1->GetType() == RK_DYNAMIC_BODY || Body2->GetType() == RK_DYNAMIC_BODY )
		{
		LinkToIsland( Joint );
		}

	mJointList.PushBack( Joint );
	RkEvent::JointAdded.Emit( this, Joint );

	return Joint;
	}


//--------------------------------------------------------------------------------------------------
RkRevoluteJoint* RkWorld::AddRevoluteJoint( RkBody* Body1, RkBody* Body2, const RkVector3& Pivot, const RkVector3& Axis, bool EnableCollision )
	{
	RkQuaternion Basis = rkShortestArc( RK_VEC3_AXIS_Z, Axis );
	RkVector3 Origin = Pivot;

	RkTransform LocalFrame1;
	LocalFrame1.Translation = Body1->TransformPointToLocal( Origin );
	LocalFrame1.Rotation = Body1->TransformBasisToLocal( Basis );

	RkTransform LocalFrame2;
	LocalFrame2.Translation = Body2->TransformPointToLocal( Origin );
	LocalFrame2.Rotation = Body2->TransformBasisToLocal( Basis );

	return AddRevoluteJoint( Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision );
	}


//--------------------------------------------------------------------------------------------------
RkRevoluteJoint* RkWorld::AddRevoluteJoint( RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision )
	{
	if ( !Body1 || Body1->GetWorld() != this )
		{
		return nullptr;
		}

	if ( !Body2 || Body2->GetWorld() != this )
		{
		return nullptr;
		}

	if ( Body1 == Body2 )
		{
		return nullptr;
		}

	void* Address = mJointPool->Allocate();
	RkRevoluteJoint* Joint = new (Address ) RkRevoluteJoint( this, Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision );
	if ( !EnableCollision )
		{
		// If this joint prevents collision, remove contacts between associated bodies!
		rkRemoveContacts( this, Body1, Body2 );
		}

	if ( Body1->GetType() == RK_DYNAMIC_BODY || Body2->GetType() == RK_DYNAMIC_BODY )
		{
		LinkToIsland( Joint );
		}

	mJointList.PushBack( Joint );
	RkEvent::JointAdded.Emit( this, Joint );

	return Joint;
	}


//--------------------------------------------------------------------------------------------------
RkPrismaticJoint* RkWorld::AddPrismaticJoint( RkBody* Body1, RkBody* Body2, const RkVector3& Pivot, const RkVector3& Axis, bool EnableCollision )
	{
	RkQuaternion Basis = rkShortestArc( RK_VEC3_AXIS_Z, Axis );
	RkVector3 Origin = Pivot;

	RkTransform LocalFrame1;
	LocalFrame1.Translation = Body1->TransformPointToLocal( Origin );
	LocalFrame1.Rotation = Body1->TransformBasisToLocal( Basis );

	RkTransform LocalFrame2;
	LocalFrame2.Translation = Body2->TransformPointToLocal( Origin );
	LocalFrame2.Rotation = Body2->TransformBasisToLocal( Basis );

	return AddPrismaticJoint( Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision );
	}


//--------------------------------------------------------------------------------------------------
RkPrismaticJoint* RkWorld::AddPrismaticJoint( RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision )
	{
	if ( !Body1 || Body1->GetWorld() != this )
		{
		return nullptr;
		}

	if ( !Body2 || Body2->GetWorld() != this )
		{
		return nullptr;
		}

	if ( Body1 == Body2 )
		{
		return nullptr;
		}

	void* Address = mJointPool->Allocate();
	RkPrismaticJoint* Joint = new ( Address ) RkPrismaticJoint( this, Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision );
	if ( !EnableCollision )
		{
		// If this joint prevents collision, remove contacts between associated bodies!
		rkRemoveContacts( this, Body1, Body2 );
		}

	if ( Body1->GetType() == RK_DYNAMIC_BODY || Body2->GetType() == RK_DYNAMIC_BODY )
		{
		LinkToIsland( Joint );
		}

	mJointList.PushBack( Joint );
	RkEvent::JointAdded.Emit( this, Joint );

	return Joint;
	}


//--------------------------------------------------------------------------------------------------
RkRigidJoint* RkWorld::AddRigidJoint( RkBody* Body1, RkBody* Body2, const RkVector3& Pivot, bool EnableCollision )
	{
	RkTransform LocalFrame1;
	LocalFrame1.Translation = Body1->TransformPointToLocal( Pivot );
	LocalFrame1.Rotation = RK_QUAT_IDENTITY;

	RkTransform LocalFrame2;
	LocalFrame2.Translation = Body2->TransformPointToLocal( Pivot );
	LocalFrame2.Rotation = RK_QUAT_IDENTITY;

	return AddRigidJoint( Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision );
	}


//--------------------------------------------------------------------------------------------------
RkRigidJoint* RkWorld::AddRigidJoint( RkBody* Body1, const RkTransform& LocalFrame1, RkBody* Body2, const RkTransform& LocalFrame2, bool EnableCollision )
	{
	if ( !Body1 || Body1->GetWorld() != this )
		{
		return nullptr;
		}

	if ( !Body2 || Body2->GetWorld() != this )
		{
		return nullptr;
		}

	if ( Body1 == Body2 )
		{
		return nullptr;
		}

	void* Address = mJointPool->Allocate();
	RkRigidJoint* Joint = new ( Address ) RkRigidJoint( this, Body1, LocalFrame1, Body2, LocalFrame2, EnableCollision );
	if ( !EnableCollision )
		{
		// If this joint prevents collision, remove contacts between associated bodies!
		rkRemoveContacts( this, Body1, Body2 );
		}

	if ( Body1->GetType() == RK_DYNAMIC_BODY || Body2->GetType() == RK_DYNAMIC_BODY )
		{
		LinkToIsland( Joint );
		}

	mJointList.PushBack( Joint );
	RkEvent::JointAdded.Emit( this, Joint );

	return Joint;
	}


//--------------------------------------------------------------------------------------------------s
bool RkWorld::RemoveJoint( std::derived_from< RkJoint > auto*& Joint )
	{
	if ( !Joint )
		{
		return false;
		}

	if ( Joint->GetWorld() != this )
		{
		return false;
		}

	RkBody* Body1 = Joint->GetBody1();
	Body1->WakeUp();
	RkBody* Body2 = Joint->GetBody2();
	Body2->WakeUp();
	
	UnlinkFromIsland( Joint );
	RK_ASSERT( mJointList.Contains( Joint ) );
	mJointList.Remove( Joint );

	RkEvent::JointRemoved.Emit( this, Joint );

	std::destroy_at( Joint );
	mJointPool->Deallocate( Joint );
	Joint = nullptr;

	return true;
	}

// Specializations
template bool RkWorld::RemoveJoint< RkJoint >( RkJoint*& );
template bool RkWorld::RemoveJoint< RkSphericalJoint >( RkSphericalJoint*& );
template bool RkWorld::RemoveJoint< RkRevoluteJoint >( RkRevoluteJoint*& );
template bool RkWorld::RemoveJoint< RkPrismaticJoint >( RkPrismaticJoint*& );
template bool RkWorld::RemoveJoint< RkRigidJoint >( RkRigidJoint*& );


//--------------------------------------------------------------------------------------------------
int RkWorld::GetJointCount() const
	{
	return mJointList.Size();
	}


//--------------------------------------------------------------------------------------------------
RkJoint* RkWorld::GetJoint( int JointIndex )
	{
	return ( 0 <= JointIndex && JointIndex < mJointList.Size() ) ? mJointList[ JointIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkJoint* RkWorld::GetJoint( int JointIndex ) const
	{
	return ( 0 <= JointIndex && JointIndex < mJointList.Size() ) ? mJointList[ JointIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkJointList& RkWorld::GetJoints() const
	{
	return mJointList;
	}


//--------------------------------------------------------------------------------------------------
RkMechanism* RkWorld::AddMechanism( const RkArray< RkJoint* >& Joints )
	{
	// No joints, no mechanism
	if ( Joints.Empty() )
		{
		return nullptr;
		}

	for ( int JointIndex1 = 0; JointIndex1 < Joints.Size(); ++JointIndex1 )
		{ 
		RkJoint* Joint1 = Joints[ JointIndex1 ];
		RK_ASSERT( Joint1 );

		// Only one mechanism per joint
		if ( Joint1->GetMechanism() )
			{
			return nullptr;
			}

		// Not more than one joints can act on the same pair of bodies
		for ( int JointIndex2 = JointIndex1 + 1; JointIndex2 < Joints.Size(); ++JointIndex2 )
			{
			RkJoint* Joint2 = Joints[ JointIndex2 ];
			RK_ASSERT( Joint2 );

			if ( Joint1->GetBody1() == Joint2->GetBody1() && Joint1->GetBody2() == Joint2->GetBody2() )
				{
				return nullptr;
				}
			if ( Joint1->GetBody1() == Joint2->GetBody2() && Joint1->GetBody2() == Joint2->GetBody1() )
				{
				return nullptr;
				}
			}
		}

	void* Address = mMechanismPool->Allocate();
	RkMechanism* Mechanism = new ( Address ) RkMechanism( this, Joints );
	mMechanismList.PushBack( Mechanism );

	return Mechanism;
	}


//--------------------------------------------------------------------------------------------------
bool RkWorld::RemoveMechanism( RkMechanism*& Mechanism )
	{
	if ( !Mechanism )
		{
		return false;
		}

	if ( Mechanism->GetWorld() != this )
		{
		return false;
		}

	RK_ASSERT( mMechanismList.Contains( Mechanism ) );
	mMechanismList.Remove( Mechanism );

	Mechanism->~RkMechanism();
	mMechanismPool->Deallocate( Mechanism );
	Mechanism = nullptr;

	return true;
	}


//--------------------------------------------------------------------------------------------------
int RkWorld::GetMechanismCount() const
	{
	return mMechanismList.Size();
	}


//--------------------------------------------------------------------------------------------------
RkMechanism* RkWorld::GetMechanism( int MechanismIndex )
	{
	return ( 0 <= MechanismIndex && MechanismIndex < mMechanismList.Size() ) ? mMechanismList[ MechanismIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkMechanism* RkWorld::GetMechanism( int MechanismIndex ) const
	{
	return ( 0 <= MechanismIndex && MechanismIndex < mMechanismList.Size() ) ? mMechanismList[ MechanismIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkMechanismList& RkWorld::GetMechanisms() const
	{
	return mMechanismList;
	}


//--------------------------------------------------------------------------------------------------
bool RkWorld::IsAutoSleepingEnabled() const
	{
	return mAutoSleeping;
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::EnableAutoSleeping( bool Enable )
	{
	if ( mAutoSleeping && !Enable )
		{
		// Wake up all sleeping islands
		RkIslandList& SleepingIslands = mIslandList[ RK_SLEEPING_ISLAND ];
		for ( int IslandIndex = SleepingIslands.Size() - 1; IslandIndex >= 0; --IslandIndex )
			{
			RkIsland* Island = SleepingIslands[ IslandIndex ];
			WakeIsland( Island );
			}
		RK_ASSERT( SleepingIslands.Empty() );
		}

	mAutoSleeping = Enable;
	}


//--------------------------------------------------------------------------------------------------
bool RkWorld::IsSleeping() const
	{
	return mIslandList[ RK_ACTIVE_ISLAND ].Empty();
	}


//--------------------------------------------------------------------------------------------------
int RkWorld::GetIslandCount( RkIslandState State ) const
	{
	return mIslandList[ State ].Size();
	}


//--------------------------------------------------------------------------------------------------
RkIsland* RkWorld::GetIsland( RkIslandState State, int IslandIndex )
	{
	return mIslandList[ State ][ IslandIndex ];
	}


//--------------------------------------------------------------------------------------------------
const RkIsland* RkWorld::GetIsland( RkIslandState State, int IslandIndex ) const
	{
	return mIslandList[ State ][ IslandIndex ];
	}


//--------------------------------------------------------------------------------------------------
const RkIslandList& RkWorld::GetIslands( RkIslandState State ) const
	{
	return mIslandList[ State ];
	}


//--------------------------------------------------------------------------------------------------
RkWorldCastResult RkWorld::CastRay( const RkVector3& RayStart, const RkVector3& RayEnd ) const
	{
	RkWorldCastResult WorldResult;
	mBroadphase->CastRay( RayStart, RayEnd, [&]( void* UserData, const RkVector3& RayStart, const RkVector3& RayEnd, float MaxHitTime )
		{
		RkShape* Shape = static_cast< RkShape* >( UserData );

		RkBody* Body = Shape->GetBody();
		RkTransform Transform = Body->GetTransform();

		RkShapeCastResult ShapeResult = Shape->CastRay( rkTMul( Transform, RayStart ), rkTMul( Transform, RayEnd ), MaxHitTime );
		if ( ShapeResult.HitTime < MaxHitTime )
			{
			WorldResult.HitBody = Body;
			WorldResult.HitShape = Shape;
			WorldResult.HitTime = ShapeResult.HitTime;
			WorldResult.HitPoint = Transform * ShapeResult.HitPoint;
			WorldResult.HitNormal = Transform.Rotation * ShapeResult.HitNormal;
			WorldResult.Triangle = ShapeResult.Triangle;

			return ShapeResult.HitTime;
			}

		// Report miss
		return 1.0f;
		} );

	return WorldResult;
	}


//--------------------------------------------------------------------------------------------------
RkWorldCastResult RkWorld::CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, RkFilter Filter ) const
	{
	RkWorldCastResult WorldResult;
	mBroadphase->CastRay( RayStart, RayEnd, [&]( void* UserData, const RkVector3& RayStart, const RkVector3& RayEnd, float MaxHitTime )
		{
		RkShape* Shape = static_cast< RkShape* >( UserData );
		if ( rkShouldCollide( Shape->GetFilter(), Filter ) )
			{
			RkBody* Body = Shape->GetBody();
			RkTransform Transform = Body->GetTransform();

			RkShapeCastResult ShapeResult = Shape->CastRay( rkTMul( Transform, RayStart ), rkTMul( Transform, RayEnd ), MaxHitTime );
			if ( ShapeResult.HitTime < MaxHitTime )
				{
				WorldResult.HitBody = Body;
				WorldResult.HitShape = Shape;
				WorldResult.HitTime = ShapeResult.HitTime;
				WorldResult.HitPoint = Transform * ShapeResult.HitPoint;
				WorldResult.HitNormal = Transform.Rotation * ShapeResult.HitNormal;
				WorldResult.Triangle = ShapeResult.Triangle;

				return ShapeResult.HitTime;
				}
			}

		// Report miss
		return 1.0f;
		} );

	return WorldResult;
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::Step( int Iterations, float Timestep )
	{
	RK_TRACY_ZONE_LVL1( "Step", Tracy_Step );
	uint64 StepTicks = rkGetTicks();
	mSample.Reset();

	Broadphase();
	Narrowphase();
	Solve( Iterations, Timestep );

	mSample.Step = rkTicksToMilliSeconds( rkGetTicks() - StepTicks );
	}


//--------------------------------------------------------------------------------------------------
const RkWorldSample& RkWorld::GetSample() const
	{
	return mSample;
	}


//--------------------------------------------------------------------------------------------------
void* RkWorld::GetUserData() const
	{
	return mUserData;
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::SetUserData( void* UserData ) const
	{
	mUserData = UserData;
	}


//--------------------------------------------------------------------------------------------------
RkPool* RkWorld::GetIslandPool()
	{
	return mIslandPool;
	}


//--------------------------------------------------------------------------------------------------
RkPool* RkWorld::GetShapePool()
	{
	return mShapePool;
	}


//--------------------------------------------------------------------------------------------------
RkPool* RkWorld::GetBodyPool()
	{
	return mBodyPool;
	}


//--------------------------------------------------------------------------------------------------
RkPool* RkWorld::GetContactPool()
	{
	return mContactPool;
	}


//--------------------------------------------------------------------------------------------------
RkPool* RkWorld::GetJointPool()
	{
	return mJointPool;
	}


//--------------------------------------------------------------------------------------------------
RkPool* RkWorld::GetMechanismPool()
	{
	return mMechanismPool;
	}


//--------------------------------------------------------------------------------------------------
RkIsland* RkWorld::AddIsland()
	{
	void* Address = mIslandPool->Allocate();
	RkIsland* Island = new ( Address ) RkIsland( this );
	mIslandList[ RK_ACTIVE_ISLAND ].PushBack( Island );

	return Island;
	}


//--------------------------------------------------------------------------------------------------
bool RkWorld::RemoveIsland( RkIsland*& Island )
	{
	RK_ASSERT( Island );
	RK_ASSERT( Island->GetWorld() == this );

	// We want to be lazy with splitting and give up if things change.
	// This is also called from SplitIsland() which means mSplitIsland
	// becomes nullptr if we split it!
	if ( Island == mSplitIsland )
		{
		mSplitIsland = nullptr;
		}

	RkIslandState State = Island->GetState();
	RK_ASSERT( !mIslandList[ 1 - State ].Contains( Island ) );
	mIslandList[ State ].Remove( Island );

	std::destroy_at( Island );
	mIslandPool->Deallocate( Island );
	Island = nullptr;

	return true;
	}


//--------------------------------------------------------------------------------------------------
RkIsland* RkWorld::MergeIslands( RkIsland* Island1, RkIsland* Island2 )
	{
	RK_ASSERT( Island1 || Island2 );
	if ( Island1 == Island2 )
		{
		return Island1;
		}

	if ( !Island1 )
		{
		return Island2;
		}

	if ( !Island2 )
		{
		return Island1;
		}

	// Make sure we merge the smaller island into the larger one
	RK_ASSERT( Island1->IsSleeping() == Island2->IsSleeping() );
	if ( Island1->Size() < Island2->Size() )
		{
		std::swap( Island1, Island2 );
		}

	// Merge bodies and fix back pointers
	RkArray< RkBody* >& Bodies1 = Island1->GetBodies();
	RkArray< RkBody* >& Bodies2 = Island2->GetBodies();
	Bodies1.Reserve( Bodies1.Size() + Bodies2.Size() );
	for ( RkBody* Body : Bodies2 )
		{
		RK_ASSERT( Body->GetIsland() == Island2 );
		Body->SetIsland( Island1 );
		Body->SetIslandIndex( Bodies1.PushBack( Body ) );
		}
	Bodies2.Clear();

	// Merge joints and fix back pointers
	RkArray< RkJoint* >& Joints1 = Island1->GetJoints();
	RkArray< RkJoint* >& Joints2 = Island2->GetJoints();
	Joints1.Reserve( Joints1.Size() + Joints2.Size() );
	for ( RkJoint* Joint : Joints2 )
		{
		RK_ASSERT( Joint->GetIsland() == Island2 );
		Joint->SetIsland( Island1 );
		Joint->SetIslandIndex( Joints1.PushBack( Joint ) );
		}
	Joints2.Clear();

	// Merge contacts and fix back pointers
	RkArray< RkContact* >& Contacts1 = Island1->GetContacts();
	RkArray< RkContact* >& Contacts2 = Island2->GetContacts();
	Contacts1.Reserve( Contacts1.Size() + Contacts2.Size() );
	for ( RkContact* Contact : Contacts2 )
		{
		RK_ASSERT( Contact->GetIsland() == Island2 );
		Contact->SetIsland( Island1 );
		Contact->SetIslandIndex( Contacts1.PushBack( Contact ) );
		}
	Contacts2.Clear();

	// Track if joints or contacts were removed
	int RemovedConstraints = Island1->GetRemovedConstraints() + Island2->GetRemovedConstraints();
	Island1->SetRemovedConstraints( RemovedConstraints );
	RK_ASSERT( Island1->IsConsistent() );

	// Island2 can be removed after it was merged into island1
	RK_ASSERT( Island2->Empty() );
	RemoveIsland( Island2 );
	
	return Island1;
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::SplitIsland( RkIsland* Island )
	{
	//SplitIsland1( Island );
	SplitIsland2( Island );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::SplitIsland1( RkIsland* Island )
	{
	RK_ASSERT( Island );
	RK_ASSERT( Island->GetWorld() == this );
	RK_ASSERT( Island->IsConsistent() );
	RK_ASSERT( Island->HasRemovedConstraints() );
	RK_ASSERT( !Island->IsSleeping() );

	// Allocate stack for DFS traversal
	const int StackSize = Island->GetBodyCount();
	RkStackArray< RkBody*, 512 > StackBuffer( StackSize );
	RkBody** Stack = StackBuffer.Data();
	
	const RkArray< RkBody* >& Seeds = Island->GetBodies();
	for ( RkBody* Seed : Seeds )
		{
		RK_ASSERT( Seed && Seed->GetType() != RK_STATIC_BODY );

		// This body already has been transfered into a split off island
		if ( Seed->GetIsland() != Island )
			{
			continue;
			}

		// Begin new island
		int StackCount = 0;
		Stack[ StackCount++ ] = Seed;

		RkIsland* SplitIsland = AddIsland();
		Seed->SetIsland( SplitIsland );
		Seed->SetIslandIndex( SplitIsland->AddBody( Seed ) );

		// Perform a depth first search (DFS) on the constraint graph to populate the new island
		while ( StackCount > 0 )
			{
			// Grab the next body off the stack 
			RkBody* Body = Stack[ --StackCount ];
			RK_ASSERT( Body );

			// Search all contacts connected to this body
			for ( RkContactEdge* Edge = Body->GetContactList(); Edge; Edge = Edge->Next )
				{
				// Pop next contact
				RkContact* Contact = Edge->Contact;
				RK_ASSERT( Contact );

				// Has this contact already has been transfered to the new split island
				if ( Contact->GetIsland() == SplitIsland )
					{
					continue;
					}
				
				// Skip non-touching contacts that do not belong to any island yet
				if ( !Contact->IsTouching() )
					{
					continue;
					}
				RK_ASSERT( Contact->GetIsland() == Island );
				
				// Add this contact to new split island
				Contact->SetIsland( SplitIsland );
				Contact->SetIslandIndex( SplitIsland->AddContact( Contact ) );

				// Was the other body already transfered to new split island
				RkBody* Other = Edge->Other;
				if ( Other->GetIsland() != SplitIsland )
					{
					// Islands don't propagate across static bodies
					if ( Other->GetType() != RK_STATIC_BODY )
						{
						RK_ASSERT( StackCount < StackSize );
						Stack[ StackCount++ ] = Other;

						// Add body to new split island
						RK_ASSERT( Other->GetIsland() == Island );
						Other->SetIsland( SplitIsland );
						Other->SetIslandIndex( SplitIsland->AddBody( Other ) );
						}
					}
				}

			// Search all joints connected to this body
			for ( RkJointEdge* Edge = Body->GetJointList(); Edge; Edge = Edge->Next )
				{
				// Pop next joint
				RkJoint* Joint = Edge->Joint;
				RK_ASSERT( Joint );

				// Has this joint already has been transfered to new split island
				if ( Joint->GetIsland() == SplitIsland )
					{
					continue;
					}
				RK_ASSERT( Joint->GetIsland() == Island );

				// Add this joint to new split island
				Joint->SetIsland( SplitIsland );
				Joint->SetIslandIndex( SplitIsland->AddJoint( Joint ) );

				// Was the other body already transfered to new split island
				RkBody* Other = Edge->Other;
				if ( Other->GetIsland() != SplitIsland )
					{
					// Islands don't propagate across static bodies
					if ( Other->GetType() != RK_STATIC_BODY )
						{
						RK_ASSERT( StackCount < StackSize );
						Stack[ StackCount++ ] = Other;

 						// Add body to new split island
						RK_ASSERT( Other->GetIsland() == Island );
						Other->SetIsland( SplitIsland );
						Other->SetIslandIndex( SplitIsland->AddBody( Other ) );
						}
					}
				}
			}

		RK_ASSERT( SplitIsland->IsConsistent() );
		}

	// This sets mSplitIsland to null if Island == mSplitIsland (which it probably true)
	RemoveIsland( Island );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::SplitIsland2( RkIsland* Island )
	{
	RK_ASSERT( Island );
	RK_ASSERT( Island->GetWorld() == this );
	RK_ASSERT( Island->IsConsistent() );
	RK_ASSERT( Island->HasRemovedConstraints() );
	RK_ASSERT( !Island->IsSleeping() );

	int BodyCount = Island->GetBodyCount();
	if ( BodyCount <= 1 )
		{
		// Single body that peeled of other island...
		Island->SetRemovedConstraints( 0 );

		return;
		}

	// Union Find (DSU) of disjoint sets
	RkStackArray< int, 512 > Parents( BodyCount );
	std::iota( Parents.Begin(), Parents.End(), 0 );

	// DSU Find() operation
	auto Find = [ & ]( int Node )
		{
		while ( Parents[ Node ] != Node )
			{
			// Path halving
			Parents[ Node ] = Parents[ Parents[ Node ] ];
			Node = Parents[ Node ];
			}

		return Node;
		};

	// DSU Union() operation
	auto Union = [ & ]( int Node1, int Node2 )
		{
		int Root1 = Find( Node1 );
		int Root2 = Find( Node2 );
		if ( Root1 != Root2 )
			{
			Parents[ Root1 ] = Root2;
			}
		};

	for ( RkContact* Contact : Island->GetContacts() )
		{
		RkBody* Body1 = Contact->GetBody1();
		int IslandIndex1 = Body1->GetIslandIndex();
		RkBody* Body2 = Contact->GetBody2();
		int IslandIndex2 = Body2->GetIslandIndex();

		// Islands don't propagate across static bodies
		if ( IslandIndex1 < 0 || IslandIndex2 < 0 )
			{
			continue;
			}

		Union( IslandIndex1, IslandIndex2 );
		}

	for ( RkJoint* Joint : Island->GetJoints() )
		{
		RkBody* Body1 = Joint->GetBody1();
		int IslandIndex1 = Body1->GetIslandIndex();
		RkBody* Body2 = Joint->GetBody2();
		int IslandIndex2 = Body2->GetIslandIndex();

		// Islands don't propagate across static bodies
		if ( IslandIndex1 < 0 || IslandIndex2 < 0 )
			{
			continue;
			}

		Union( IslandIndex1, IslandIndex2 );
		}

	// Create new Islands
	RkStackArray< RkIsland*, 512 > SplitIslands;
	RkStackArray< RkIsland*, 512 > RootToIslandMap( BodyCount, nullptr );

	for ( RkBody* Body : Island->GetBodies() )
		{
		int Root = Find( Body->GetIslandIndex() );
		RkIsland* SplitIsland = RootToIslandMap[ Root ];

		if ( !SplitIsland )
			{
			SplitIsland = AddIsland();
			SplitIslands.PushBack( SplitIslands );
			RootToIslandMap[ Root ] = SplitIsland;
			}

		Body->SetIsland( SplitIsland );
		Body->SetIslandIndex( SplitIsland->AddBody( Body ) );
		}

	for ( RkContact* Contact : Island->GetContacts() )
		{
		RkBody* Body1 = Contact->GetBody1();
		RkBody* Body2 = Contact->GetBody2();

		// Single out static bodies
		RkBody* DynamicBody = Body1->GetType() == RK_DYNAMIC_BODY ? Body1 : Body2;
		RK_ASSERT( DynamicBody->GetType() == RK_DYNAMIC_BODY );
		RkIsland* SplitIsland = DynamicBody->GetIsland();
		RK_ASSERT( SplitIsland && SplitIsland != Island );

		Contact->SetIsland( SplitIsland );
		Contact->SetIslandIndex( SplitIsland->AddContact( Contact ) );
		}

	for ( RkJoint* Joint : Island->GetJoints() )
		{
		RkBody* Body1 = Joint->GetBody1();
		RkBody* Body2 = Joint->GetBody2();

		// Single out static bodies
		RkBody* DynamicBody = Body1->GetType() == RK_DYNAMIC_BODY ? Body1 : Body2;
		RK_ASSERT( DynamicBody->GetType() == RK_DYNAMIC_BODY );
		RkIsland* SplitIsland = DynamicBody->GetIsland();
		RK_ASSERT( SplitIsland && SplitIsland != Island );

		Joint->SetIsland( SplitIsland );
		Joint->SetIslandIndex( SplitIsland->AddJoint( Joint ) );
		}

	RemoveIsland( Island );
	RK_ASSERT( std::all_of( SplitIslands.Begin(), SplitIslands.End(), []( RkIsland* SplitIsland ) { return SplitIsland->IsConsistent(); } ) );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::SleepIsland( RkIsland* Island )
	{
	RK_ASSERT( Island );
	RK_ASSERT( Island->GetWorld() == this );
	RK_ASSERT( Island->IsConsistent() );
	RK_ASSERT( !Island->GetRemovedConstraints() );
	RK_ASSERT( !Island->IsSleeping() );

	// Transfer joints
	for ( int JointIndex = 0; JointIndex < Island->GetJointCount(); ++JointIndex )
		{
		RkJoint* Joint = Island->GetJoint( JointIndex );
		RK_ASSERT( Joint->GetIsland() == Island && Joint->GetIslandIndex() == JointIndex );

		RK_ASSERT( Joint->SolverIndex >= 0 );
		mConstraintGraph.RemoveJoint( Joint );
		mJointSet.Remove( Joint );
		}

	// Transfer contacts
	for ( int ContactIndex = 0; ContactIndex < Island->GetContactCount(); ++ContactIndex )
		{
		RkContact* Contact = Island->GetContact( ContactIndex );
		RK_ASSERT( Contact->GetIsland() == Island && Contact->GetIslandIndex() == ContactIndex );

		// Note: Constraint graph uses index from contact set and must be filled second
		RK_ASSERT( Contact->SolverIndex >= 0 );
		mConstraintGraph.RemoveContact( Contact );
		mContactSet.Remove( Contact );

		mCollisionSet.Remove( Contact );
		}

	// Transfer bodies
	for ( int BodyIndex = 0; BodyIndex < Island->GetBodyCount(); ++BodyIndex )
		{
		RkBody* Body = Island->GetBody( BodyIndex );
		RK_ASSERT( Body->GetIsland() == Island && Body->GetIslandIndex() == BodyIndex );

		// Remove non-touching contacts from active list
		for ( RkContactEdge* Edge = Body->GetContactList(); Edge; Edge = Edge->Next )
			{
			RkContact* Contact = Edge->Contact;
			if ( Contact->IsTouching() )
				{
				// Contact is touching and will be moved below
				RK_ASSERT( Contact->GetIsland() == Island );
				continue;
				}

			if ( !Contact->IsActive() )
				{
				// Already disabled by previous body in the island
				RK_ASSERT( Edge->Other->GetIslandIndex() < BodyIndex );
				continue;
				}

			RkBody* Other = Edge->Other;
			if ( Other->GetIsland() != Island && !Other->IsSleeping() )
				{
				// Non-touching contact with different still awake island 
				continue;
				}

			// Non-touching contact in same island or with *different* island, but other body/island is sleeping - should be disabled
			mCollisionSet.Remove( Contact );
			}

		// Remove body from body set. This *clears* the RkBody::SolverIndex which
		// might be used by contacts and/or joints. So we set this last here!
		mBodySet.Remove( Body );
		}

	// Set new state
	mIslandList[ RK_ACTIVE_ISLAND ].Remove( Island );
	mIslandList[ RK_SLEEPING_ISLAND ].PushBack( Island );
	
	Island->Sleep();
	RK_ASSERT( Island->IsConsistent() );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::WakeIsland( RkIsland* Island )
	{
	RK_ASSERT( Island );
	RK_ASSERT( Island->GetWorld() == this );
	RK_ASSERT( Island->IsConsistent() );
	RK_ASSERT( Island->IsSleeping() );

	// Transfer bodies
	for ( int BodyIndex = 0; BodyIndex < Island->GetBodyCount(); ++BodyIndex )
		{
		// Move body into body set. This *sets* the RkBody::SolverIndex which
		// might be used by contacts and/or joints. So we set this first here!
		RkBody* Body = Island->GetBody( BodyIndex );
		RK_ASSERT( Body->GetIsland() == Island && Body->GetIslandIndex() == BodyIndex );
		mBodySet.PushBack( Body );

		// Reset sleep timer
		Body->SetSleepTime( 0.0f );

		// Move non-touching contacts back into active list
		for ( RkContactEdge* Edge = Body->GetContactList(); Edge; Edge = Edge->Next )
			{
			RkContact* Contact = Edge->Contact;
			if ( Contact->IsTouching() )
				{
				// Contact is touching and will be moved below. The contact island can be 
				// null if we link a new contact between an active and sleeping island.
				RK_ASSERT( Contact->GetIsland() == Island || !Contact->GetIsland() );
				continue;
				}

			if ( Contact->IsActive() )
				{
				// Contact is active if it spans across different islands or has been already woken up by previous body
				RK_ASSERT( Contact->GetIsland() != Island || Edge->Other->GetIslandIndex() < BodyIndex );
				continue;
				}

			RK_ASSERT( !Contact->IsTouching() );
			mCollisionSet.PushBack( Contact );
			}
		}

	// Transfer contacts
	for ( int ContactIndex = 0; ContactIndex < Island->GetContactCount(); ++ContactIndex )
		{
		RkContact* Contact = Island->GetContact( ContactIndex );
		RK_ASSERT( Contact->GetIsland() == Island && Contact->GetIslandIndex() == ContactIndex );

		RK_ASSERT( Contact->IsTouching() );
		mCollisionSet.PushBack( Contact );

		// Note: Constraint graph uses index from contact set and must be filled second
		mContactSet.PushBack( Contact );
		mConstraintGraph.AddContact( Contact );
		}

	// Transfer joints
	for ( int JointIndex = 0; JointIndex < Island->GetJointCount(); ++JointIndex )
		{
		RkJoint* Joint = Island->GetJoint( JointIndex );
		RK_ASSERT( Joint->GetIsland() == Island && Joint->GetIslandIndex() == JointIndex );
		
		mJointSet.PushBack( Joint );
		mConstraintGraph.AddJoint( Joint );
		}

	// Set new state
	mIslandList[ RK_SLEEPING_ISLAND ].Remove( Island );
	mIslandList[ RK_ACTIVE_ISLAND ].PushBack( Island );
	
	Island->WakeUp();
	RK_ASSERT( Island->IsConsistent() );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::LinkToIsland( RkBody* Body )
	{
	RK_ASSERT( Body );

	if ( Body->GetType() == RK_STATIC_BODY )
		{
		return;
		}

	RK_ASSERT( !Body->GetIsland() );
	RkIsland* Island = AddIsland();
	Body->SetIsland( Island );
	Body->SetIslandIndex( Island->AddBody( Body ) );

	mBodySet.PushBack( Body );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::UnlinkFromIsland( RkBody* Body )
	{
	RK_ASSERT( Body );

	RkIsland* Island = Body->GetIsland();
	if ( !Island )
		{
		RK_ASSERT( Body->GetType() == RK_STATIC_BODY );
		return;
		}

	if ( !Island->IsSleeping() )
		{
		mBodySet.Remove( Body );
		}

	Island->RemoveBody( Body );
	Body->SetIsland( nullptr );
	Body->SetIslandIndex( -1 );

	if ( Island->GetBodyCount() == 0 )
		{
		RK_ASSERT( Island->Empty() );
		RemoveIsland( Island );
		}
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::LinkToIsland( RkContact* Contact )
	{
	RK_ASSERT( Contact );
	RK_ASSERT( Contact->GraphIndex < 0 );

	RkBody* Body1 = Contact->GetBody1();
	RkIsland* Island1 = Body1->GetIsland();
	RkBody* Body2 = Contact->GetBody2();
	RkIsland* Island2 = Body2->GetIsland();
	RK_ASSERT( Island1 || Island2 );

	if ( Body1->IsSleeping() && !Body2->IsSleeping() )
		{
		// This also wakes up island1 (if body1 not static)
		Body1->WakeUp();
		}
	if ( Body2->IsSleeping() && !Body1->IsSleeping() )
		{
		// This also wakes up island2 (if body2 not static)
		Body2->WakeUp();
		}

	RK_ASSERT( !Contact->GetIsland() );
	RkIsland* Island = MergeIslands( Island1, Island2 );
	Contact->SetIsland( Island );
	Contact->SetIslandIndex( Island->AddContact( Contact ) );

	// Note: Constraint graph uses index from contact set and must be filled second
	RK_ASSERT( Contact->IsTouching() );
	mContactSet.PushBack( Contact );
	mConstraintGraph.AddContact( Contact );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::UnlinkFromIsland( RkContact* Contact )
	{
	RK_ASSERT( Contact );

	RkIsland* Island = Contact->GetIsland();
	if ( !Island )
		{
		return;
		}

	if ( !Island->IsSleeping() )
		{
		// Note: Constraint graph uses index from contact set and must be cleared first
		mConstraintGraph.RemoveContact( Contact );
		mContactSet.Remove( Contact );
		}

	Island->RemoveContact( Contact );
	Contact->SetIsland( nullptr );
	Contact->SetIslandIndex( -1 );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::LinkToIsland( RkJoint* Joint )
	{
	RK_ASSERT( Joint );
	RK_ASSERT( Joint->GraphIndex < 0 );

	RkBody* Body1 = Joint->GetBody1();
	RkIsland* Island1 = Body1->GetIsland();
	RkBody* Body2 = Joint->GetBody2();
	RkIsland* Island2 = Body2->GetIsland();
	RK_ASSERT( Island1 || Island2 );

	if ( Body1->IsSleeping() && !Body2->IsSleeping() )
		{
		Body1->WakeUp();
		}
	if ( Body2->IsSleeping() && !Body1->IsSleeping() )
		{
		Body2->WakeUp();
		}

	RK_ASSERT( !Joint->GetIsland() );
	RkIsland* Island = MergeIslands( Island1, Island2 );
	Joint->SetIsland( Island );
	Joint->SetIslandIndex( Island->AddJoint( Joint ) );

	mJointSet.PushBack( Joint );
	mConstraintGraph.AddJoint( Joint );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::UnlinkFromIsland( RkJoint* Joint )
	{
	RK_ASSERT( Joint );

	RkIsland* Island = Joint->GetIsland();
	if ( !Island )
		{
		return;
		}

	if ( !Island->IsSleeping() )
		{
		mConstraintGraph.RemoveJoint( Joint );
		mJointSet.Remove( Joint );
		}
	
	Island->RemoveJoint( Joint );
	Joint->SetIsland( nullptr );
	Joint->SetIslandIndex( -1 );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::Broadphase()
	{
	RK_TRACY_ZONE_LVL1( "Broadphase", Tracy_Broadphase );
	uint64 BroadphaseTicks = rkGetTicks();

	// Update pairs
	RkStackArray< RkProxyPair, 512 > Pairs;
		{
		RK_TRACY_ZONE_LVL2( "Update Pairs", Tracy_UpdatePairs );
		uint64 UpdatePairsTicks = rkGetTicks();

		mBroadphase->Update( Pairs, [ & ]( const RkProxy& Proxy1, const RkProxy& Proxy2 )
			{
			RK_ASSERT( Proxy1 != RK_NULL_PROXY );
			RK_ASSERT( Proxy2 != RK_NULL_PROXY );
			RK_ASSERT( Proxy1 != Proxy2 );

			RkShape* Shape1 = mBroadphase->GetProxyShape( Proxy1 );
			RkShape* Shape2 = mBroadphase->GetProxyShape( Proxy2 );
			RK_ASSERT( Shape1 != Shape2 );

			RkBody* Body1 = Shape1->GetBody();
			RkBody* Body2 = Shape2->GetBody();
			if ( Body1 == Body2 )
				{
				// Ignore shapes that are associated with the same body (e.g. compounds)
				return false;
				}

			// Test if at least one body is dynamic or if joints prevent collision
			if ( !rkShouldCollide( Body1, Body2 ) )
				{
				return false;
				}

			// Test if the collision filters prevent collision
			if ( !rkShouldCollide( Shape1, Shape2 ) )
				{
				return false;
				}

			// Skip contacts that already exist 
			if ( mContactMap.Contains( RkShapePair( Shape1, Shape2 ) ) )
				{
				return false;
				}

			return true;
			} );

		mSample.UpdatePairs = rkTicksToMilliSeconds( rkGetTicks() - UpdatePairsTicks );
		}

	// Add Contacts
		{
		RK_TRACY_ZONE_LVL2( "Add Contacts", Tracy_AddContacts );
		uint64 AddContactsTicks = rkGetTicks();

		for ( RkProxyPair Pair : Pairs )
			{
			RkShape* Shape1 = mBroadphase->GetProxyShape( Pair.Proxy1 );
			RkShape* Shape2 = mBroadphase->GetProxyShape( Pair.Proxy2 );
			AddContact( Shape1, Shape2 );
			}

		mSample.AddContacts = rkTicksToMilliSeconds( rkGetTicks() - AddContactsTicks );
		}

	mSample.Broadphase = rkTicksToMilliSeconds( rkGetTicks() - BroadphaseTicks );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::Narrowphase()
	{
	RK_TRACY_ZONE_LVL1( "Narrowphase", Tracy_Narrowphase );
	uint64 NarrowphaseTicks = rkGetTicks();

	// Rebuild tree concurrently with narrowphase
	if ( mBroadphase->IsDirty() )
		{
		mRebuildTask = mExecutor->async( [ & ]()
			{
			RK_TRACY_ZONE_LVL3( "Rebuild Task", Tracy_BackgroundTask );
			uint64 RebuildTaskTicks = rkGetTicks();

			mBroadphase->Rebuild();

			mSample.RebuildTask = rkTicksToMilliSeconds( rkGetTicks() - RebuildTaskTicks );
			} );

		mRebuilding = true;
		}

	if ( !mCollisionSet.Empty() )
		{
		// Parallel *forward* sweep over *active* contacts to create manifold geometry
		int WorkerCount = static_cast<int>( mExecutor->num_workers() );
		RkStackArray< RkBitset, 32 > ChangedContactSets( WorkerCount );
		for ( RkBitset& ChangedContactSet : ChangedContactSets )
			{
			ChangedContactSet.Resize( mCollisionSet.Size() );
			}

		// Collide Contacts
			{
			RK_TRACY_ZONE_LVL2( "Collide Contacts", Tracy_CollideContacts );
			uint64 CollideContactsTicks = rkGetTicks();

			tf::Taskflow Taskflow;
			Taskflow.for_each( mCollisionSet.Begin(), mCollisionSet.End(), [ & ]( RkContact* Contact )
				{
				RK_TRACY_ZONE_LVL3( "Collide Contacts Task", Tracy_CollideContactsTask );

				// Skip contact if both bodies want to sleep
				const float SkipTime = 0.5f * RK_SLEEP_TIME;

				RkBody* Body1 = Contact->GetBody1();
				bool CanSkip1 = Body1->GetType() == RK_STATIC_BODY || Body1->GetSleepTime() > SkipTime;
				RkBody* Body2 = Contact->GetBody2();
				bool CanSkip2 = Body2->GetType() == RK_STATIC_BODY || Body2->GetSleepTime() > SkipTime;
				if ( CanSkip1 && CanSkip2 )
					{
					return;
					}

				int CollisionIndex = Contact->CollisionIndex;
				int WorkerIndex = mExecutor->this_worker_id();
				RkBitset& ChangedContactSet = ChangedContactSets[ WorkerIndex ];
				RK_ASSERT( 0 <= CollisionIndex && CollisionIndex < ChangedContactSet.Size() );
				RK_ASSERT( ChangedContactSet.Test( CollisionIndex ) == false );
				RK_ASSERT( Contact->GetCollideFlags() == 0 );

				// Confirm contact
				RkShape* Shape1 = Contact->GetShape1();
				RkShape* Shape2 = Contact->GetShape2();
				if ( !rkOverlap( Shape1->GetProxyBounds(), Shape2->GetProxyBounds() ) )
					{
					// Proxy bounds ceased to overlap - destroy contact
					ChangedContactSet.Set( CollisionIndex );
					Contact->SetCollideFlag( RK_DESTROY_CONTACT );
					return;
					}

				// Update contact 
				int ManifoldCount1 = Contact->GetManifoldCount();
				bool WasTouching = ManifoldCount1 > 0;
				Contact->Collide();
				int ManifoldCount2 = Contact->GetManifoldCount();
				bool IsTouching = ManifoldCount2 > 0;

				if ( IsTouching && !WasTouching )
					{
					// TouchBegin
					ChangedContactSet.Set( CollisionIndex );
					Contact->SetCollideFlag( RK_CONTACT_BEGIN_TOUCH );
					}
				if ( WasTouching && !IsTouching )
					{
					// TouchEnd
					ChangedContactSet.Set( CollisionIndex );
					Contact->SetCollideFlag( RK_CONTACT_END_TOUCH );
					}
					
				} );

			mExecutor->run( Taskflow ).wait();

			mSample.CollideContacts = rkTicksToMilliSeconds( rkGetTicks() - CollideContactsTicks );
			}

		// Update contacts
			{
			RK_TRACY_ZONE_LVL2( "Update Contacts", Tracy_UpdateContacts );
			uint64 UpdateContactsTicks = rkGetTicks();

			// Consolidate local results
			RkBitset ChangedContactSet( mCollisionSet.Size() );
			for ( int WorkerIndex = 0; WorkerIndex < WorkerCount; ++WorkerIndex )
				{
				ChangedContactSet |= ChangedContactSets[ WorkerIndex ];
				}
		
			// Serial *backward* sweep over *active* contacts to remove unconfirmed manifolds and fire callbacks
			ChangedContactSet.ForEachSetBitR( [ & ]( int CollisionIndex )
				{
				RkContact* Contact = mCollisionSet[ CollisionIndex ];
		
				// If the contact ceased to overlap we can now destroy it
				if ( Contact->TestCollideFlag( RK_DESTROY_CONTACT ) )
					{
					// This will report end touch if needed!
					RemoveContact( Contact );
					return;
					}
		
				// Report touch events
				if ( Contact->TestCollideFlag( RK_CONTACT_BEGIN_TOUCH ) )
					{
					// Begin touch
					LinkToIsland( Contact );
					RkEvent::TouchBegin.Emit( this, Contact );
					}
		
				if ( Contact->TestCollideFlag( RK_CONTACT_END_TOUCH ) )
					{
					// End touch
					RkEvent::TouchEnd.Emit( this, Contact );
					UnlinkFromIsland( Contact );
					}
		
				Contact->ClearCollideFlags();
				} );

			mSample.UpdateContacts = rkTicksToMilliSeconds( rkGetTicks() - UpdateContactsTicks );
			}
		}

	// Sync rebuild task
	if ( mRebuilding )
		{
		mRebuildTask.wait();
		mRebuilding = false;

		//mSample.RebuildTrees = mBroadphase->GetRebuild();
		}

	mSample.Narrowphase = rkTicksToMilliSeconds( rkGetTicks() - NarrowphaseTicks );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::Solve( int Iterations, float Timestep )
	{
	RK_TRACY_ZONE_LVL1( "Solve", Tracy_Solve );
	uint64 SolveTicks = rkGetTicks();

	// Split island concurrently with constraint solver
	if ( mSplitIsland )
		{
		mSplitTask = mExecutor->async( [ & ]()
			{
			RK_TRACY_ZONE_LVL2( "Split Island Task", Tracy_BackgroundTask );
			uint64 SplitTaskTicks = rkGetTicks();

			SplitIsland( mSplitIsland );

			mSample.SplitTask = rkTicksToMilliSeconds( rkGetTicks() - SplitTaskTicks );
			} );

		mSplitting = true;
		}

	// Allocate temporary memory
	RkConstraintBuffer ConstraintBuffer;
		{
		RK_TRACY_ZONE_LVL2( "Alloc", Tracy_Solve );
		uint64 AllocateTicks = rkGetTicks();

		rkAllocConstraintBuffer( mMainArena, ConstraintBuffer, mConstraintGraph );

		mSample.Allocate = rkTicksToMilliSeconds( rkGetTicks() - AllocateTicks );
		}

	int BodyCount = mBodySet.Size();
	RkSolverBody* BodyBuffer = static_cast< RkSolverBody* >( mMainArena->Allocate( ( BodyCount + 1 ) * sizeof( RkSolverBody ), alignof( RkSolverBody ) ) );
	RK_ASSERT( BodyBuffer && rkIsAligned( BodyBuffer, alignof( RkSolverBody ) ) );
	BodyBuffer[ 0 ].LinearVelocity = RK_VEC3_ZERO;
	BodyBuffer[ 0 ].AngularVelocity = RK_VEC3_ZERO;

	// Solve
	IntegrateForces( BodyBuffer, Timestep);
	LoadConstraints( ConstraintBuffer, BodyBuffer, Timestep );
	SolveConstraints( ConstraintBuffer, BodyBuffer, Iterations, Timestep );
	SaveConstraints( ConstraintBuffer );
	IntegrateVelocities( BodyBuffer, Timestep );
	
	// Free memory
	rkFreeConstraintBuffer( mMainArena, ConstraintBuffer );

	// Sync rebuild task
	if ( mSplitting )
		{
		mSplitTask.wait();
		mSplitIsland = nullptr;
		mSplitting = false;
		}

	RefitProxies( Timestep );
	Sleeping( Timestep );
	
	mSample.Solve = rkTicksToMilliSeconds( rkGetTicks() - SolveTicks );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::IntegrateForces( RkSolverBody* BodyBuffer, float Timestep )
	{
	RK_TRACY_ZONE_LVL2( "Integrate Forces", Tracy_IntegrateForces );
	uint64 IntegrateForcesTicks = rkGetTicks();

	if ( !mBodySet.Empty() )
		{
		tf::Taskflow Taskflow;
		Taskflow.for_each( mBodySet.Begin(), mBodySet.End(), [ &BodyBuffer, Timestep ]( RkBody* Body )
			{
			RK_TRACY_ZONE_LVL3( "Integrate Forces Task", Tracy_IntegrateForcesTask );

			int BodyIndex = Body->SolverIndex + 1;
			BodyBuffer[ BodyIndex ].LinearVelocity = Body->IntegrateForces( Timestep );
			Body->ClearNetForce();
			BodyBuffer[ BodyIndex ].AngularVelocity = Body->IntegrateTorques( Timestep );
			Body->ClearNetTorque();
			} );

		mExecutor->run( Taskflow ).wait();
		}

	mSample.IntegrateForces = rkTicksToMilliSeconds( rkGetTicks() - IntegrateForcesTicks );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::LoadConstraints( RkConstraintBuffer& ConstraintBuffer, RkSolverBody* BodyBuffer, float Timestep )
	{
	RK_TRACY_ZONE_LVL2( "Load Constraints", Tracy_LoadConstraints );
	uint64 LoadConstraintsTicks = rkGetTicks();

	if ( !mBodySet.Empty() )
		{
		// Compliance (stiffer for dynamic vs static/keyframed)
		float JointDampingRatio = mJointDampingRatio;
		float JointFrequency = rkMin( mJointFrequency, 0.5f / Timestep );
		const RkCompliance JointCompliance = rkCreateCompliance( JointFrequency, JointDampingRatio, Timestep );

		float ContactDampingRatio = mContactDampingRatio;
		float ContactFrequency = rkMin( mContactFrequency, 0.25f / Timestep );
		const RkCompliance DynamicContactCompliance = rkCreateCompliance( ContactFrequency, ContactDampingRatio, Timestep );
		const RkCompliance StaticContactCompliance = rkCreateCompliance( 2.0f * ContactFrequency, ContactDampingRatio, Timestep );

		tf::Task Barrier0;
		tf::Taskflow Taskflow;
		for ( int ColorIndex = 0; ColorIndex < RK_MAX_GRAPH_COLORS; ++ColorIndex )
			{
			RkGraphColor& Color = mConstraintGraph.Colors[ ColorIndex ];
			if ( Color.Empty() )
				{
				continue;
				};

			int JointCount = Color.Joints.Size();
			RkIndexedArray< RkJoint, &RkJoint::ColorIndex >& Joints = Color.Joints;
			RkJointConstraint* JointConstraints = ConstraintBuffer.JointConstraints[ ColorIndex ];
			tf::Task JointTask = Taskflow.for_each_index( 0, JointCount, 1, [ &Joints, &BodyBuffer, JointConstraints, JointCompliance, Timestep ]( int JointIndex )
				{
				RK_TRACY_ZONE_LVL3( "Load Joint Constraints Task", Tracy_LoadConstraintsTask );

				RkJoint* Joint = Joints[ JointIndex ];
				RkJointConstraint* JointConstraint = JointConstraints + JointIndex;
				Joint->LoadConstraints( JointConstraint, BodyBuffer, JointCompliance, Timestep );
				} );

			int ContactCount = Color.Contacts.Size();
			RkIndexedArray< RkContact, &RkContact::ColorIndex >& Contacts = Color.Contacts;
			RkContactConstraint* ContactConstraints = ConstraintBuffer.ContactConstraints[ ColorIndex ];
			int* ContactMappings = ConstraintBuffer.ContactMappings[ ColorIndex ];
			tf::Task ContactTask = Taskflow.for_each_index( 0, ContactCount, 1, [ &Contacts, &BodyBuffer, ContactConstraints, ContactMappings, StaticContactCompliance, DynamicContactCompliance, Timestep ]( int ContactIndex )
				{
				RK_TRACY_ZONE_LVL3( "Load Contact Constraints Task", Tracy_LoadConstraintsTask );

				RkContact* Contact = Contacts[ ContactIndex ];
				int ContactOffset = ContactMappings[ ContactIndex ];
				RkContactConstraint* ContactConstraint = ContactConstraints + ContactOffset;
				RkCompliance ContactCompliance = Contact->IsStatic() ? StaticContactCompliance : DynamicContactCompliance;
				Contact->LoadConstraints( ContactConstraint, BodyBuffer, ContactCompliance, Timestep );
				} );

			tf::Task Barrier = Taskflow.emplace( [] {} );
			JointTask.precede( Barrier );
			ContactTask.precede( Barrier );

			if ( !Barrier0.empty() )
				{
				JointTask.succeed( Barrier0 );
				ContactTask.succeed( Barrier0 );
				}
			Barrier0 = Barrier;
			}

		mExecutor->run( Taskflow ).wait();
		}

	mSample.LoadConstraints = rkTicksToMilliSeconds( rkGetTicks() - LoadConstraintsTicks );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::SolveConstraints( RkConstraintBuffer& ConstraintBuffer, RkSolverBody* BodyBuffer, int Iterations, float Timestep )
	{
	RK_TRACY_ZONE_LVL2( "Solve Constraints", Tracy_SolveConstraints );
	uint64 SolveConstraintsTicks = rkGetTicks();
	
	if ( !mBodySet.Empty() )
		{
		// Compliance (stiffer for dynamic vs static/keyframed)
		float JointDampingRatio = mJointDampingRatio;
		float JointFrequency = rkMin( mJointFrequency, 0.5f / Timestep );
		const RkCompliance JointCompliance = rkCreateCompliance( JointFrequency, JointDampingRatio, Timestep );

		tf::Task Barrier0;
		tf::Taskflow Taskflow;
		for ( int ColorIndex = 0; ColorIndex < RK_MAX_GRAPH_COLORS; ++ColorIndex )
			{
			RkGraphColor& Color = mConstraintGraph.Colors[ ColorIndex ];
			if ( Color.Empty() )
				{
				continue;
				};

			int JointCount = Color.Joints.Size();
			RkIndexedArray< RkJoint, &RkJoint::ColorIndex >& Joints = Color.Joints;
			RkJointConstraint* JointConstraints = ConstraintBuffer.JointConstraints[ ColorIndex ];
			tf::Task JointTask = Taskflow.for_each_index( 0, JointCount, 1, [ &Joints, &BodyBuffer, JointConstraints, JointCompliance ]( int JointIndex )
				{
				RK_TRACY_ZONE_LVL3( "Solve Joint Constraints", Tracy_SolveConstraintsTask );

				RkJoint* Joint = Joints[ JointIndex ];
				RkJointConstraint* JointConstraint = JointConstraints + JointIndex;
				Joint->SolveConstraints( JointConstraint, BodyBuffer, JointCompliance );
				} );

			int ContactCount = Color.Contacts.Size();
			RkIndexedArray< RkContact, &RkContact::ColorIndex >& Contacts = Color.Contacts;
			RkContactConstraint* ContactConstraints = ConstraintBuffer.ContactConstraints[ ColorIndex ];
			int* ContactMappings = ConstraintBuffer.ContactMappings[ ColorIndex ];
			tf::Task ContactTask = Taskflow.for_each_index( 0, ContactCount, 1, [ &Contacts, &BodyBuffer, ContactConstraints, ContactMappings ]( int ContactIndex )
				{
				RK_TRACY_ZONE_LVL3( "Solve Contact Constraints", Tracy_SolveConstraintsTask );

				RkContact* Contact = Contacts[ ContactIndex ];
				int ContactOffset = ContactMappings[ ContactIndex ];
				RkContactConstraint* ContactConstraint = ContactConstraints + ContactOffset;
				Contact->SolveConstraints( ContactConstraint, BodyBuffer );
				} );

			tf::Task Barrier = Taskflow.emplace( [] {} );
			JointTask.precede( Barrier );
			ContactTask.precede( Barrier );

			if ( !Barrier0.empty() )
				{
				JointTask.succeed( Barrier0 );
				ContactTask.succeed( Barrier0 );
				}
			Barrier0 = Barrier;
			}

		mExecutor->run_n( Taskflow, Iterations ).wait();
		}

	mSample.SolveConstraints = rkTicksToMilliSeconds( rkGetTicks() - SolveConstraintsTicks );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::SaveConstraints( RkConstraintBuffer& ConstraintBuffer )
	{
	RK_TRACY_ZONE_LVL2( "Save Constraints", Tracy_SaveConstraints );
	uint64 SaveConstraintsTicks = rkGetTicks();

	if ( !mBodySet.Empty() )
		{
		tf::Task Barrier0;
		tf::Taskflow Taskflow;
		for ( int ColorIndex = 0; ColorIndex < RK_MAX_GRAPH_COLORS; ++ColorIndex )
			{
			RkGraphColor& Color = mConstraintGraph.Colors[ ColorIndex ];
			if ( Color.Empty() )
				{
				continue;
				};

			int JointCount = Color.Joints.Size();
			RkIndexedArray< RkJoint, &RkJoint::ColorIndex >& Joints = Color.Joints;
			RkJointConstraint* JointConstraints = ConstraintBuffer.JointConstraints[ ColorIndex ];
			tf::Task JointTask = Taskflow.for_each_index( 0, JointCount, 1, [ &Joints, JointConstraints ]( int JointIndex )
				{
				RK_TRACY_ZONE_LVL3( "Save Joint Constraints", Tracy_SaveConstraintsTask );

				RkJoint* Joint = Joints[ JointIndex ];
				RkJointConstraint* JointConstraint = JointConstraints + JointIndex;
				Joint->SaveConstraints( JointConstraint );
				} );

			int ContactCount = Color.Contacts.Size();
			RkIndexedArray< RkContact, &RkContact::ColorIndex >& Contacts = Color.Contacts;
			RkContactConstraint* ContactConstraints = ConstraintBuffer.ContactConstraints[ ColorIndex ];
			int* ContactMappings = ConstraintBuffer.ContactMappings[ ColorIndex ];
			tf::Task ContactTask = Taskflow.for_each_index( 0, ContactCount, 1, [ &Contacts, ContactConstraints, ContactMappings ]( int ContactIndex )
				{
				RK_TRACY_ZONE_LVL3( "Save Contact Constraints", Tracy_SaveConstraintsTask );

				RkContact* Contact = Contacts[ ContactIndex ];
				int ContactOffset = ContactMappings[ ContactIndex ];
				RkContactConstraint* ContactConstraint = ContactConstraints + ContactOffset;
				Contact->SaveConstraints( ContactConstraint );
				} );

			tf::Task Barrier = Taskflow.emplace( [] {} );
			JointTask.precede( Barrier );
			ContactTask.precede( Barrier );

			if ( !Barrier0.empty() )
				{
				JointTask.succeed( Barrier0 );
				ContactTask.succeed( Barrier0 );
				}
			Barrier0 = Barrier;
			}

		mExecutor->run( Taskflow ).wait();
		}

	mSample.SaveConstraints = rkTicksToMilliSeconds( rkGetTicks() - SaveConstraintsTicks );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::IntegrateVelocities( RkSolverBody* BodyBuffer, float Timestep )
	{
	RK_TRACY_ZONE_LVL2( "Integrate Velocities", Tracy_IntegrateVelocities );
	uint64 IntegrateVelocitiesTicks = rkGetTicks();
	
	if ( !mBodySet.Empty() )
		{
		tf::Taskflow Taskflow;
		Taskflow.for_each( mBodySet.Begin(), mBodySet.End(), [ &BodyBuffer, Timestep ]( RkBody* Body )
			{
			RK_TRACY_ZONE_LVL3( "Integrate Velocities Task", Tracy_IntegrateVelocitiesTask );

			int BodyIndex = Body->SolverIndex + 1;
			RkVector3 LinearVelocity = BodyBuffer[ BodyIndex ].LinearVelocity;
			RkVector3 AngularVelocity = BodyBuffer[ BodyIndex ].AngularVelocity;
			Body->IntegrateVelocities( LinearVelocity, AngularVelocity, Timestep );
			} );

		mExecutor->run( Taskflow ).wait();
		}

	mSample.IntegrateVelocities = rkTicksToMilliSeconds( rkGetTicks() - IntegrateVelocitiesTicks );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::RefitProxies( float Timestep )
	{
	RK_TRACY_ZONE_LVL2( "Refit Proxies", Tracy_RefitProxies );
	uint64 RefitProxiesTicks = rkGetTicks();

	// Body set for proxy refit
	if ( !mBodySet.Empty() )
		{
		int WorkerCount = static_cast< int >( mExecutor->num_workers() );

		int BodyCount = mBodySet.Size();
		RkStackArray< RkBitset, 32 > BodySets( WorkerCount );
		for ( RkBitset& BodySet : BodySets )
			{
			BodySet.Resize( BodyCount );
			}

		// Update leaf bounds
			{
			RK_TRACY_ZONE_LVL3( "Update Bounds", Tracy_UpdateBounds );
			uint64 UpdateBoundsTicks = rkGetTicks();

			tf::Taskflow Taskflow;
			Taskflow.for_each_index( 0, BodyCount, 1, [ & ]( int BodyIndex )
				{
				RK_TRACY_ZONE_LVL3( "Update Bounds Task", Tracy_UpdateBoundsTask );

				RkBody* Body = mBodySet[ BodyIndex ];
				RK_ASSERT( Body->SolverIndex == BodyIndex );
				int WorkerIndex = mExecutor->this_worker_id();
				RK_ASSERT( WorkerIndex >= 0 );

				// Update shape bounds
				if ( Body->UpdateShapes() )
					{
					// Mark bodies with updated shapes for refit below
					RkBitset& BodySet = BodySets[ WorkerIndex ];
					BodySet.Set( BodyIndex );
					}
				} );

			mExecutor->run( Taskflow ).wait();

			mSample.UpdateBounds = rkTicksToMilliSeconds( rkGetTicks() - UpdateBoundsTicks );
			}

		// Propagate leaf bounds 
			{
			RK_TRACY_ZONE_LVL3( "Propagate Bounds", Tracy_PropagateBounds );
			uint64 PropagateBoundsTicks = rkGetTicks();

			// Consolidate results
			RkBitset BodySet( BodyCount );
			for ( int WorkerIndex = 0; WorkerIndex < WorkerCount; ++WorkerIndex )
				{
				BodySet |= BodySets[ WorkerIndex ];
				}

			// Refit proxies
			BodySet.ForEachSetBit( [ & ]( int BodyIndex )
				{
				RkBody* Body = mBodySet[ BodyIndex ];
				for ( RkShape* Shape : Body->GetShapes() )
					{
					if ( Shape->IsDirty() )
						{
						// Refit proxies - this also marks proxies for rebuild
						mBroadphase->RefitProxy( Shape->GetProxy(), Shape->GetProxyBounds() );
						Shape->SetDirty( false );
						}
					}
				} );

			mSample.PropagateBounds = rkTicksToMilliSeconds( rkGetTicks() - PropagateBoundsTicks );
			}
		}

	mSample.RefitProxies = rkTicksToMilliSeconds( rkGetTicks() - RefitProxiesTicks );
	}


//--------------------------------------------------------------------------------------------------
void RkWorld::Sleeping( float Timestep )
	{
	RK_TRACY_ZONE_LVL2( "Sleeping", Tracy_Sleeping );
	uint64 SleepingTicks = rkGetTicks();

	// Body set for proxy refit
	if ( mAutoSleeping && !mBodySet.Empty() )
		{
		// Advance
			{
			RK_TRACY_ZONE_LVL3( "Advance Islands", Tracy_AdvanceIslands );
			uint64 AdvanceIslandsTicks = rkGetTicks();

			// Island sets to keep islands awake
			int WorkerCount = static_cast<int>( mExecutor->num_workers() );

			int IslandCount = mIslandList[ RK_ACTIVE_ISLAND ].Size();
			const RkIslandList& IslandList = mIslandList[ RK_ACTIVE_ISLAND ];
			RkStackArray< RkBitset, 32 > IslandSets( WorkerCount );
			for ( RkBitset& IslandSet : IslandSets )
				{
				IslandSet.Resize( IslandCount );
				}

			// Best island for splitting
			RkArray< float > BestSplitTimes( WorkerCount, RK_SLEEP_TIME );
			RkArray< RkIsland* > BestSplitIslands( WorkerCount );

			tf::Taskflow Taskflow;
			Taskflow.for_each_index( 0, mBodySet.Size(), 1, [ & ]( int BodyIndex )
				{
				RK_TRACY_ZONE_LVL3( "Advance Islands Task", Tracy_AdvanceIslandsTask );

				RkBody* Body = mBodySet[ BodyIndex ];
				RK_ASSERT( Body->SolverIndex == BodyIndex );
				int WorkerIndex = mExecutor->this_worker_id();
				RK_ASSERT( WorkerIndex >= 0 );

				// Islands
				RkIsland* Island = Body->GetIsland();
				RK_ASSERT( IslandList[ Island->WorldIndex ] == Island );

				float SleepTime = Body->AdvanceSleepTime( Timestep );
				if ( SleepTime < RK_SLEEP_TIME )
					{
					// Keep island awake
					RkBitset& ActiveIslandSet = IslandSets[ WorkerIndex ];
					ActiveIslandSet.Set( Island->WorldIndex );
					}
				else if ( Island->HasRemovedConstraints() )
					{
					if ( SleepTime > BestSplitTimes[ WorkerIndex ] )
						{
						// Body wants to sleep, but island needs splitting first
						BestSplitTimes[ WorkerIndex ] = SleepTime;
						BestSplitIslands[ WorkerIndex ] = Island;
						}
					}
				} );

			mExecutor->run( Taskflow ).wait();

			// Consolidate local results
			float BestSplitTime = 0.0f;
			RkIsland* BestSplitIsland = nullptr;
			RkBitset ActiveIslandSet( IslandCount );
			for ( int WorkerIndex = 0; WorkerIndex < WorkerCount; ++WorkerIndex )
				{
				// DIRK_TODO: Determinism
				if ( BestSplitTime < BestSplitTimes[ WorkerIndex ] )
					{
					BestSplitTime = BestSplitTimes[ WorkerIndex ];
					BestSplitIsland = BestSplitIslands[ WorkerIndex ];
					}

				ActiveIslandSet |= IslandSets[ WorkerIndex ];
				}

			RK_ASSERT( mActiveIslandSet.Empty() );
			mActiveIslandSet = ActiveIslandSet;
			RK_ASSERT( !mSplitIsland );
			mSplitIsland = BestSplitIsland;

			mSample.AdvanceIslands = rkTicksToMilliSeconds( rkGetTicks() - AdvanceIslandsTicks );
			}
		
		// Sleep islands
			{
			RK_TRACY_ZONE_LVL3( "Sleep Islands", Tracy_SleepIslands );
			uint64 SleepIslandsTicks = rkGetTicks();

			RkIslandList& ActiveIslands = mIslandList[ RK_ACTIVE_ISLAND ];
			for ( int IslandIndex = ActiveIslands.Size() - 1; IslandIndex >= 0; --IslandIndex )
				{
				if ( mActiveIslandSet.Test( IslandIndex ) )
					{
					// Island is still active
					continue;
					}

				RkIsland* Island = ActiveIslands[ IslandIndex ];
				RK_ASSERT( Island->IsConsistent() && !Island->IsSleeping() );
				if ( Island->HasRemovedConstraints() )
					{
					// Island wants to sleep, but needs spitting first
					continue;
					}

				SleepIsland( Island );
				}

			mSample.SleepIslands = rkTicksToMilliSeconds( rkGetTicks() - SleepIslandsTicks );
			}

		// DIRK_TODO: Make local
		mActiveIslandSet.Clear(); 
		}

	mSample.Sleeping = rkTicksToMilliSeconds( rkGetTicks() - SleepingTicks );
	}


//--------------------------------------------------------------------------------------------------
const RkConstraintGraph& RkWorld::GetConstraintGraph() const
	{
	return mConstraintGraph;
	}