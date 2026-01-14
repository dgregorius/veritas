//--------------------------------------------------------------------------------------------------
// body.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "body.h"
#include "broadphase.h"
#include "constants.h"
#include "contact.h"
#include "event.h"
#include "island.h"
#include "joint.h"
#include "mass.h"
#include "world.h"

#include "shape.h"
#include "sphereshape.h"
#include "capsuleshape.h"
#include "hullshape.h"
#include "meshshape.h"


//--------------------------------------------------------------------------------------------------
// RkBody
//--------------------------------------------------------------------------------------------------
RkBody::RkBody( RkWorld* World )
	{
	RK_ASSERT( World );
	mWorld = World;

	mType = RK_STATIC_BODY;
	mShapeAllocator = World->GetShapePool();

	mMassInv = 0.0f;
	mLocalMassCenter = RK_VEC3_ZERO;
	mMassCenter = RK_VEC3_ZERO;
	mLocalInertiaInv = RK_MAT3_ZERO;
	mInertiaInv = RK_MAT3_ZERO;
	
	mPosition = RK_VEC3_ZERO;
	mOrientation = RK_QUAT_IDENTITY;
	mLinearVelocity = RK_VEC3_ZERO;
	mAngularVelocity = RK_VEC3_ZERO;
	mNetForce = RK_VEC3_ZERO;
	mNetTorque = RK_VEC3_ZERO;

	mJointCount = 0;
	mJointList = nullptr;
	mContactCount = 0;
	mContactList = nullptr;

	RK_ASSERT( mType == RK_STATIC_BODY );
	mIsland = nullptr;
	mIslandIndex = -1;
	mAutoSleeping = true;
	mSleepTime = 0.0f;

	mMinMotionRadius = RK_F32_MAX;
	mMaxMotionRadius = 0.0f;

	mUserData = nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkBody::~RkBody()
	{
	RK_ASSERT( !mJointCount && !mJointList );
	RK_ASSERT( !mContactCount && !mContactList );

	// Delete island (if any)
	if ( mIsland )
		{
		mIsland->RemoveBody( this );
		if ( mIsland->Empty() )
			{
			mWorld->RemoveIsland( mIsland );
			mIslandIndex = -1;
			}
		}

	// Delete shapes
	while ( !mShapeList.Empty() )
		{
		// Pop next shape
		RkShape* Shape = mShapeList.Back();
		mShapeList.PopBack();

		// Remove from broadphase
		RkBroadphase* Broadphase = mWorld->GetBroadphase();
		Shape->Remove( Broadphase );
		
		// Callback (implicit destruction)
		RkEvent::ShapeRemoved.Emit( this, Shape );

		std::destroy_at( Shape );
		mShapeAllocator->Deallocate( Shape );
		}
	}


//--------------------------------------------------------------------------------------------------
RkWorld* RkBody::GetWorld() const
	{
	return mWorld;
	}


//--------------------------------------------------------------------------------------------------
RkBodyType RkBody::GetType() const
	{
	return mType;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::SetType( RkBodyType Type )
	{
	if ( mType == Type )
		{
		return;
		}

	// Clear velocity if this body is about to become static
	if ( Type == RK_STATIC_BODY )
		{
		mLinearVelocity = RK_VEC3_ZERO;
		mAngularVelocity = RK_VEC3_ZERO;
		}

	// Either we are becoming dynamic in which case forces are already zero or
	// we are about to become static/keyframed in which case forces should be zero
	mNetForce = RK_VEC3_ZERO;
	mNetTorque = RK_VEC3_ZERO;

	// Remove all associated contacts from island
		{
		RkContactEdge* Edge = mContactList;
		while ( Edge )
			{
			RkContact* Contact = Edge->Contact;
			Edge = Edge->Next;

			mWorld->UnlinkFromIsland( Contact );
			}
		}

	// Remove all associated joints from island
		{
		RkJointEdge* Edge = mJointList;
		while ( Edge )
			{
			RkJoint* Joint = Edge->Joint;
			Edge = Edge->Next;

			mWorld->UnlinkFromIsland( Joint );
			}
		}

	// Remove body from island
	mWorld->UnlinkFromIsland( this );

	// Remove shapes from broadphase
	RkBroadphase* Broadphase = mWorld->GetBroadphase();
	for ( RkShape* Shape : mShapeList )
		{
		Shape->Remove( Broadphase );
		}

	// Save new type
	mType = Type;

	// Reinsert shapes with new body type
	RkTransform Transform = GetTransform();
	for ( RkShape* Shape : mShapeList )
		{
		// This also touches proxies so new contacts can be created
		Shape->Insert( Broadphase, Transform );
		}

	// Reinsert non-static body into island
	RK_ASSERT( !mIsland );
	mWorld->LinkToIsland( this );

	// Reinsert joints into island
		{
		RkJointEdge* Edge = mJointList;
		while ( Edge )
			{
			RkJoint* Joint = Edge->Joint;
			RkBody* Other = Edge->Other;
			Edge = Edge->Next;

			if ( mType == RK_DYNAMIC_BODY || Other->GetType() == RK_DYNAMIC_BODY )
				{
				mWorld->LinkToIsland( Joint );
				}
			}
		}

	// Reinsert contacts into island
		{
		RkContactEdge* Edge = mContactList;
		while ( Edge )
			{
			RkContact* Contact = Edge->Contact;
			RkBody* Other = Edge->Other;
			Edge = Edge->Next;

			if ( mType == RK_DYNAMIC_BODY || Other->GetType() == RK_DYNAMIC_BODY )
				{
				mWorld->LinkToIsland( Contact );
				}
			else
				{
				// Destroy non-physical contacts
				mWorld->RemoveContact( Contact );
				}
			}
		}

	// Compute new mass
	RebuildMass();

	// Wake up body (does nothing if static)
	WakeUp();
	}


//--------------------------------------------------------------------------------------------------
RkSphereShape* RkBody::AddSphere( const RkSphere& Sphere )
	{
	void* Address = mShapeAllocator->Allocate();
	RkSphereShape* Shape = new ( Address ) RkSphereShape( this, Sphere );
	Shape->Insert( mWorld->GetBroadphase(), GetTransform() );
	mShapeList.PushBack( Shape );
	RebuildMass();

	RkEvent::ShapeAdded.Emit( this, Shape );

	return Shape;
	}


//--------------------------------------------------------------------------------------------------
RkCapsuleShape* RkBody::AddCapsule( const RkCapsule& Capsule )
	{
	void* Address = mShapeAllocator->Allocate();
	RkCapsuleShape* Shape = new (Address ) RkCapsuleShape( this, Capsule );
	Shape->Insert( mWorld->GetBroadphase(), GetTransform() );
	mShapeList.PushBack( Shape );
	RebuildMass();

	RkEvent::ShapeAdded.Emit( this, Shape );

	return Shape;
	}


//--------------------------------------------------------------------------------------------------
RkHullShape* RkBody::AddHull( RkHull* Hull )
	{
	if ( !Hull )
		{
		return nullptr;
		}
	
	void* Address = mShapeAllocator->Allocate();
	RkHullShape* Shape = new ( Address ) RkHullShape( this, Hull );
	Shape->Insert( mWorld->GetBroadphase(), GetTransform() );
	mShapeList.PushBack( Shape );
	RebuildMass();

	RkEvent::ShapeAdded.Emit( this, Shape );

	return Shape;
	}


//--------------------------------------------------------------------------------------------------
RkMeshShape* RkBody::AddMesh( RkBVH* BVH )
	{
	if ( !BVH )
		{
		return nullptr;
		}

	void* Address = mShapeAllocator->Allocate();
	RkMeshShape* Shape = new ( Address ) RkMeshShape( this, BVH );
	Shape->Insert( mWorld->GetBroadphase(), GetTransform() );
	mShapeList.PushBack( Shape );
	RebuildMass(); 

	RkEvent::ShapeAdded.Emit( this, Shape );

	return Shape;
	}


//--------------------------------------------------------------------------------------------------
bool RkBody::RemoveShape( std::derived_from< RkShape > auto*& Shape )
	{
	if ( !Shape )
		{
		return false;
		}

	if ( Shape->GetBody() != this )
		{
		return false;
		}

	RkContactEdge* Edge = mContactList;
	while ( Edge )
		{
		RkContact* Contact = Edge->Contact;
		Edge = Edge->Next;

		// Destroy contact
		if ( Contact->GetShape1() == Shape || Contact->GetShape2() == Shape )
			{
			mWorld->RemoveContact( Contact );
			}
		}

	// Remove from broadphase
	RkBroadphase* Broadphase = mWorld->GetBroadphase();
	Shape->Remove( Broadphase );

	// Remove from shape list
	mShapeList.Remove( Shape );

	// Rebuild mass if necessary
	if ( Shape->GetType() != RK_MESH_SHAPE )
		{
		RebuildMass();
		}

	RkEvent::ShapeRemoved.Emit( this, Shape );

	std::destroy_at( Shape );
	mShapeAllocator->Deallocate( Shape );
	Shape = nullptr;

	return true;
	}

// Specializations
template bool RkBody::RemoveShape< RkShape >( RkShape*& );
template bool RkBody::RemoveShape< RkSphereShape >( RkSphereShape*& );
template bool RkBody::RemoveShape< RkCapsuleShape >( RkCapsuleShape*& );
template bool RkBody::RemoveShape< RkHullShape >( RkHullShape*& );
template bool RkBody::RemoveShape< RkMeshShape >( RkMeshShape*& );


//--------------------------------------------------------------------------------------------------
int RkBody::GetShapeCount() const
	{
	return mShapeList.Size();
	}


//--------------------------------------------------------------------------------------------------
RkShape* RkBody::GetShape( int ShapeIndex )
	{
	return ( 0 <= ShapeIndex && ShapeIndex < mShapeList.Size() ) ? mShapeList[ ShapeIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkShape* RkBody::GetShape( int ShapeIndex ) const
	{
	return ( 0 <= ShapeIndex && ShapeIndex < mShapeList.Size() ) ? mShapeList[ ShapeIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const RkShapeList& RkBody::GetShapes() const
	{
	return mShapeList;
	}


//--------------------------------------------------------------------------------------------------
float RkBody::GetMass() const
	{
	return mMassInv > 0.0f ? 1.0f / mMassInv : 0.0f;
	}


//--------------------------------------------------------------------------------------------------
float RkBody::GetMassInv() const
	{
	return mMassInv;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::GetMassCenter() const
	{
	return mMassCenter;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::GetLocalMassCenter() const
	{
	return mLocalMassCenter;
	}


//--------------------------------------------------------------------------------------------------
RkMatrix3 RkBody::GetInertia() const
	{
	return mMassInv > 0.0f ? rkInvertT( mInertiaInv ) : RK_MAT3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RkMatrix3 RkBody::GetInertiaInv() const
	{
	return mInertiaInv;
	}


//--------------------------------------------------------------------------------------------------
RkMatrix3 RkBody::GetLocalInertia() const
	{
	return mMassInv > 0.0f ? rkInvertT( mLocalInertiaInv ) : RK_MAT3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RkMatrix3 RkBody::GetLocalInertiaInv() const
	{
	return mLocalInertiaInv;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::RebuildMass()
	{
	// Compute mass data from shapes. Each shape has its own density.
	mMassInv = 0.0f;
	mLocalMassCenter = RK_VEC3_ZERO;
	mMassCenter = mPosition;
	mLocalInertiaInv = RK_MAT3_ZERO;
	mInertiaInv = RK_MAT3_ZERO;
	mMinMotionRadius = RK_F32_MAX;
	mMaxMotionRadius = 0.0f;
	
	// Static bodies have zero mass.
	if ( mType == RK_STATIC_BODY )
		{
		return;
		}

	// Accumulate mass properties for dynamic bodies over all shapes at the origin
	if ( mType == RK_DYNAMIC_BODY )
		{
		float Mass = 0.0f;
		RkVector3 Center = RK_VEC3_ZERO;
		RkMatrix3 Inertia = RK_MAT3_ZERO;

		for ( RkShape* Shape : mShapeList )
			{
			if ( Shape->GetType() == RK_MESH_SHAPE )
				{
				continue;
				}

			RkMassProperties Properties = Shape->ComputeMassProperties();
			Mass += Properties.Mass;
			Center += Properties.Mass * Properties.Center;
			Inertia += Properties.Inertia;
			}

		if ( Mass > 0.0f )
			{
			// Compute center of mass
			Center /= Mass;

			// Shift inertia to mass center
			Inertia -= rkSteiner( Mass, Center );

			// Save result
			RkVector3 Translation = mPosition;
			RkMatrix3 Rotation( mOrientation );

			mMassInv = 1.0f / Mass;
			mLocalMassCenter = Center;
			mMassCenter = Rotation * mLocalMassCenter + Translation;
			mLocalInertiaInv = rkInvertT( Inertia );
			mInertiaInv = Rotation * mLocalInertiaInv * rkTranspose( Rotation );
			}
		else
			{
			// Dynamic body without convex shapes (use unit sphere inertia I = 2/5 * m * r^2)
			mMassInv = 1.0f;
			mLocalMassCenter = RK_VEC3_ZERO;
			mMassCenter = mOrientation * mLocalMassCenter + mPosition;
			mLocalInertiaInv = 0.4f * RK_MAT3_IDENTITY;
			mInertiaInv = 0.4f * RK_MAT3_IDENTITY;
			}
		}

	// Rebuild motion radii for non-static bodies (relative to COM)
	for ( RkShape* Shape : mShapeList )
		{
		mMinMotionRadius = rkMin( mMinMotionRadius, Shape->GetMinMotionRadius() );
		mMaxMotionRadius = rkMax( mMaxMotionRadius, Shape->GetMaxMotionRadius( mLocalMassCenter ) );
		}
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::GetPosition() const
	{
	return mPosition;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::SetPosition( const RkVector3& Position )
	{
	// Update position and mass properties
	mPosition = Position;
	mMassCenter = mOrientation * mLocalMassCenter + mPosition;

	// Update broadphase
	MoveProxies();
	}


//--------------------------------------------------------------------------------------------------
RkQuaternion RkBody::GetOrientation() const
	{
	return mOrientation;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::SetOrientation( const RkQuaternion& Orientation )
	{
	// Update orientation and mass properties
	mOrientation = Orientation;
	mMassCenter = mOrientation * mLocalMassCenter + mPosition;

	RkMatrix3 Rotation( Orientation );
	mInertiaInv = Rotation * mLocalInertiaInv * rkTranspose( Rotation );

	// Update broadphase
	MoveProxies();
	}


//--------------------------------------------------------------------------------------------------
RkTransform RkBody::GetTransform() const
	{
	return RkTransform( mPosition, mOrientation );
	}


//--------------------------------------------------------------------------------------------------
void RkBody::SetTransform( const RkTransform& Transform )
	{
	// Update transform and mass properties
	mPosition = Transform.Translation;
	mOrientation = RkQuaternion( Transform.Rotation );
	mMassCenter = mOrientation * mLocalMassCenter + mPosition;

	RkMatrix3 Rotation( Transform.Rotation );
	mInertiaInv = Rotation * mLocalInertiaInv * rkTranspose( Rotation );

	// Update broadphase
	MoveProxies();
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::TransformVectorToLocal( const RkVector3& Vector ) const
	{
	// v' = RT * v
	return rkTMul( mOrientation, Vector );
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::TransformVectorToWorld( const RkVector3& Vector ) const
	{
	// v = R * v' 
	return rkMul( mOrientation, Vector );
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::TransformPointToLocal( const RkVector3& Point ) const
	{
	// v' = RT * (v - t)
	return rkTMul( mOrientation, Point - mPosition );
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::TransformPointToWorld( const RkVector3& Point ) const
	{
	// v = R * v' + t
	return rkMul( mOrientation, Point ) + mPosition;
	}


//--------------------------------------------------------------------------------------------------
RkQuaternion RkBody::TransformBasisToLocal( const RkQuaternion& Basis ) const
	{
	RkQuaternion Out;
	Out = rkCMul( mOrientation, Basis );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RkQuaternion RkBody::TransformBasisToWorld( const RkQuaternion& Basis ) const
	{
	RkQuaternion Out;
	Out = rkMul( mOrientation, Basis );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::GetLinearVelocity() const
	{
	return mLinearVelocity;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::SetLinearVelocity( const RkVector3& LinearVelocity )
	{
	if ( mType == RK_STATIC_BODY )
		{
		return;
		}

	mLinearVelocity = LinearVelocity;
	if ( rkLengthSq( mLinearVelocity ) > 0.0f )
		{
		WakeUp();
		}
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::GetAngularVelocity() const
	{
	return mAngularVelocity;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::SetAngularVelocity( const RkVector3& AngularVelocity )
	{
	if ( mType == RK_STATIC_BODY )
		{
		return;
		}

	mAngularVelocity = AngularVelocity;
	if ( rkLengthSq( mAngularVelocity ) > 0.0f )
		{
		WakeUp();
		}
	}


//--------------------------------------------------------------------------------------------------
void RkBody::ApplyLinearImpule( const RkVector3& Impulse, bool Wake )
	{
	if ( mType == RK_DYNAMIC_BODY )
		{
		return;
		}

	if ( Wake )
		{
		WakeUp();
		}

	mLinearVelocity += mMassInv * Impulse;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::ApplyLinearImpulseAt( const RkVector3& Impulse, const RkVector3& Point, bool Wake )
	{
	if ( mType == RK_DYNAMIC_BODY )
		{
		return;
		}

	if ( Wake )
		{
		WakeUp();
		}

	mLinearVelocity += mMassInv * Impulse;
	mAngularVelocity += mInertiaInv * rkCross( Point - mMassCenter, Impulse );
	}


//--------------------------------------------------------------------------------------------------
void RkBody::ApplyAngularImpulse( const RkVector3& Impulse, bool Wake )
	{
	if ( mType == RK_DYNAMIC_BODY )
		{
		return;
		}

	if ( Wake )
		{
		WakeUp();
		}

	mAngularVelocity += mInertiaInv * Impulse;
	}


//--------------------------------------------------------------------------------------------------
bool RkBody::SetVelocityFromKeyframe( const RkTransform& TargetTransform, float Timestep )
	{
	if ( mType == RK_STATIC_BODY || Timestep <= 0.0f )
		{
		return false;
		}

	// Compute linear velocity
	RkVector3 vCenter1 = mMassCenter;
	RkVector3 vCenter2 = TargetTransform * mLocalMassCenter;
	mLinearVelocity = ( vCenter2 - vCenter1 ) / Timestep;

	// Compute angular velocity:
	// q' = 0.5 * w * q 
	// <~> ( q2 - q1 ) / dt =  0.5 * w * q1 
	// <=> w = 2 * ( q2 - q1 ) * Conjugate( q1 ) / dt
	RkQuaternion Orientation1 = mOrientation;
	RkQuaternion Orientation2 = TargetTransform.Rotation;
	if ( rkDot( Orientation1, Orientation2 ) < 0.0f )
		{
		// Use the shortest arc quaternion
		Orientation2 = -Orientation2;
		}
	RkQuaternion Omega = ( 2.0f / Timestep ) * ( Orientation2 - Orientation1 ) * rkConjugate( Orientation1 );
	mAngularVelocity = Omega.V;

	// DIRK_TODo: v = |v| + |w| * r_max
	if ( rkLengthSq( mLinearVelocity ) + rkLengthSq( mAngularVelocity ) > 0.0f )
		{
		WakeUp();
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::ApplyForce( const RkVector3& Force, bool Wake )
	{
	if ( mType != RK_DYNAMIC_BODY )
		{
		return;
		}

	if ( Wake )
		{
		WakeUp();
		}

	mNetForce += Force;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::ApplyForceAt( const RkVector3& Force, const RkVector3& Point, bool Wake )
	{
	if ( mType != RK_DYNAMIC_BODY )
		{
		return;
		}

	if ( Wake )
		{
		WakeUp();
		}

	mNetForce += Force;
	mNetTorque += rkCross( Point - mMassCenter, Force );
	}


//--------------------------------------------------------------------------------------------------
void RkBody::ApplyTorque( const RkVector3& Torque, bool Wake )
	{
	if ( mType != RK_DYNAMIC_BODY )
		{
		return;
		}

	if ( Wake )
		{
		WakeUp();
		}

	mNetTorque += Torque;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::GetNetForce() const
	{
	return mNetForce;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::ClearNetForce()
	{
	mNetForce = RK_VEC3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::GetNetTorque() const
	{
	return mNetTorque;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::ClearNetTorque()
	{
	mNetTorque = RK_VEC3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
bool RkBody::IsAutoSleepingEnabled() const
	{
	return mAutoSleeping;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::EnableAutoSleeping( bool Enable )
	{
	mAutoSleeping = Enable;
	if ( !mAutoSleeping )
		{
		WakeUp();
		}
	}


//--------------------------------------------------------------------------------------------------
void RkBody::WakeUp()
	{
	if ( IsSleeping() )
		{
		if ( mType != RK_STATIC_BODY )
			{
			RK_ASSERT( mIsland && mIsland->GetBody( mIslandIndex ) == this );
			mWorld->WakeIsland( mIsland );
			}

		RK_ASSERT( mSleepTime == 0.0f );
		}
	}


//--------------------------------------------------------------------------------------------------
bool RkBody::IsSleeping() const
	{
	if ( mType == RK_STATIC_BODY )
		{
		RK_ASSERT( !mIsland );
		return true;
		}

	RK_ASSERT( mIsland );
	return mIsland->IsSleeping();
	}


//--------------------------------------------------------------------------------------------------
void* RkBody::GetUserData() const
	{
	return mUserData;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::SetUserData( void* UserData ) const
	{
	mUserData = UserData;
	}


//--------------------------------------------------------------------------------------------------
RkIsland* RkBody::GetIsland() const
	{
	return mIsland;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::SetIsland( RkIsland* Island )
	{
	mIsland = Island;
	}


//--------------------------------------------------------------------------------------------------
int RkBody::GetIslandIndex() const
	{
	return mIslandIndex;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::SetIslandIndex( int IslandIndex )
	{
	RK_ASSERT( IslandIndex < 0 || mIsland->GetBody( IslandIndex ) == this );
	mIslandIndex = IslandIndex;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::AddJoint( RkJointEdge* Edge )
	{
	// Add joint to list
	RK_ASSERT( mJointCount >= 0 );
	mJointCount++;

	RK_ASSERT( Edge && Edge->Other != this );
	Edge->Prev = nullptr;
	Edge->Next = mJointList;
	if ( mJointList )
		{
		mJointList->Prev = Edge;
		}
	mJointList = Edge;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::RemoveJoint( RkJointEdge* Edge )
	{
	// Remove joint from list
	RK_ASSERT( mJointCount > 0 );
	mJointCount--;

	RK_ASSERT( Edge && Edge->Other != this );
	if ( Edge->Prev )
		{
		Edge->Prev->Next = Edge->Next;
		}
	if ( Edge->Next )
		{
		Edge->Next->Prev = Edge->Prev;
		}
	if ( mJointList == Edge )
		{
		mJointList = Edge->Next;
		}
	}


//--------------------------------------------------------------------------------------------------
int RkBody::GetJointCount() const
	{
	return mJointCount;
	}


//--------------------------------------------------------------------------------------------------
RkJointEdge* RkBody::GetJointList()
	{
	return mJointList;
	}


//--------------------------------------------------------------------------------------------------
const RkJointEdge* RkBody::GetJointList() const
	{
	return mJointList;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::AddContact( RkContactEdge* Edge )
	{
	// Add pair to list
	RK_ASSERT( mContactCount >= 0 );
	mContactCount++;

	RK_ASSERT( Edge && Edge->Other != this );
	Edge->Prev = nullptr;
	Edge->Next = mContactList;
	if ( mContactList )
		{
		mContactList->Prev = Edge;
		}
	mContactList = Edge;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::RemoveContact( RkContactEdge* Edge )
	{
	// Remove pair from list
	RK_ASSERT( mContactCount > 0 );
	mContactCount--;

	RK_ASSERT( Edge && Edge->Other != this );
	if ( Edge->Prev )
		{
		Edge->Prev->Next = Edge->Next;
		}
	if ( Edge->Next )
		{
		Edge->Next->Prev = Edge->Prev;
		}

	if ( mContactList == Edge )
		{
		mContactList = Edge->Next;
		}
	}


//--------------------------------------------------------------------------------------------------
int RkBody::GetContactCount() const
	{
	return mContactCount;
	}


//--------------------------------------------------------------------------------------------------
RkContactEdge* RkBody::GetContactList()
	{
	return mContactList;
	}


//--------------------------------------------------------------------------------------------------
const RkContactEdge* RkBody::GetContactList() const
	{
	return mContactList;
	}


//--------------------------------------------------------------------------------------------------
float RkBody::GetMinMotionRadius() const
	{
	return mMinMotionRadius;
	}


//--------------------------------------------------------------------------------------------------
float RkBody::GetMaxMotionRadius() const
	{
	return mMaxMotionRadius;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::IntegrateForces( float Timestep ) const
	{
	if ( mType != RK_DYNAMIC_BODY )
		{
		return mLinearVelocity;
		}

	// Explicit Euler integration
	RkVector3 Velocity = mLinearVelocity + ( mMassInv * mNetForce + mWorld->GetGravity() ) * Timestep;

	// DIRK_TODO: Damping...

	return Velocity;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkBody::IntegrateTorques( float Timestep ) const
	{
	if ( mType != RK_DYNAMIC_BODY )
		{
		return mAngularVelocity;
		}

	// Implicit-Explicit Euler integration (body coordinates)
	RkMatrix3 I = rkInvertT( mLocalInertiaInv );
	RkVector3 Omega1 = rkTMul( mOrientation, mAngularVelocity + GetInertiaInv() * mNetTorque * Timestep );
	RkVector3 Omega2 = Omega1;

	const int Iterations = 2;
	for ( int Iteration = 0; Iteration < Iterations; ++Iteration )
		{
		RkVector3 S = rkCross( Omega2, I * Omega2 ) * Timestep + I * ( Omega2 - Omega1 );
		RkMatrix3 J = ( rkSkew( Omega2 ) * I - rkSkew( I * Omega2 ) ) * Timestep + I;

		RkVector3 DeltaOmega = rkInvert( J ) * -S;
		Omega2 += DeltaOmega;
		}

	// Compute orientation at the *end* of the timestep so we can.
	// transform the local angular velocity w' into world coordinates!
	// We utilize q' = 0.5 * w * q = 0.5 * (q * w' * qc) * q = 0.5 * q * w'
	RkQuaternion Q2 = rkNormalize( mOrientation * rkExp( Omega2 * Timestep ) );
	RkVector3 Omega = Q2 * Omega2;

	// DIRK_TODO: Damping...

	return Omega;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::IntegrateVelocities( const RkVector3& LinearVelocity, const RkVector3& AngularVelocity, float Timestep )
	{
	RK_ASSERT( mType != RK_STATIC_BODY );

	// Save new angular velocity and integrate
	mAngularVelocity = AngularVelocity;
	mOrientation = rkNormalize( rkExp( mAngularVelocity * Timestep ) * mOrientation );
	
	// Save new linear velocity and integrate
	mLinearVelocity = LinearVelocity;
	mMassCenter += mLinearVelocity * Timestep;
	mPosition = mMassCenter - mOrientation * mLocalMassCenter;
	
	// Update inertia tensor
	RkMatrix3 R( mOrientation );
	mInertiaInv = R * mLocalInertiaInv * rkTranspose( R );
	}


//--------------------------------------------------------------------------------------------------
void RkBody::MoveProxies()
	{
	// Don't skip static bodies (or similar) here since this would break SetPosition()
	RkTransform Transform = GetTransform();
	for ( RkShape* Shape : mShapeList )
		{
		Shape->Move( mWorld->GetBroadphase(), Transform );
		}
	}


//--------------------------------------------------------------------------------------------------
bool RkBody::UpdateShapes()
	{
	bool Refit = false;

	// Don't skip static bodies (or similar) here since this would break SetPosition()
	RkTransform Transform = GetTransform();
	for ( RkShape* Shape : mShapeList )
		{
		if ( Shape->Update( Transform ) )
			{
			Refit = true;
			}
		}

	return Refit;
	}


//--------------------------------------------------------------------------------------------------
float RkBody::GetSleepTime() const
	{
	return mSleepTime;
	}


//--------------------------------------------------------------------------------------------------
void RkBody::SetSleepTime( float SleepTime )
	{
	RK_ASSERT( SleepTime >= 0.0f );
	mSleepTime = SleepTime;
	}


//--------------------------------------------------------------------------------------------------
float RkBody::AdvanceSleepTime( float Timestep )
	{
	RK_ASSERT( mWorld->IsAutoSleepingEnabled() && Timestep >= 0.0f );

	float MaxVelocity = rkLength( mLinearVelocity ) + rkLength( mAngularVelocity ) * mMaxMotionRadius;
	if ( !mAutoSleeping || MaxVelocity > RK_SLEEP_THRESHOLD )
		{
		mSleepTime = 0.0f;
		}
	else
		{
		mSleepTime += Timestep;
		}
		
	return mSleepTime;
	}
