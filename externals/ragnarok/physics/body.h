
//--------------------------------------------------------------------------------------------------
/*
	@file		body.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/bodytype.h"
#include "ragnarok/physics/shape.h"

#include "ragnarok/common/array.h"
#include "ragnarok/common/math.h"
#include "ragnarok/common/pool.h"

class RkSphereShape;
class RkCapsuleShape;
class RkHullShape;
class RkMeshShape;
class RkIsland;
class RkWorld;

struct RkBVH;
struct RkCapsule;
struct RkContactEdge;
struct RkHull;
struct RkJointEdge;
struct RkMassProperties;
struct RkSphere;

using RkShapeList = RkIndexedArray< RkShape, &RkShape::BodyIndex >;


//--------------------------------------------------------------------------------------------------
// RkBody
//--------------------------------------------------------------------------------------------------
class RkBody
	{
	public:
		// Construction / Destruction
		explicit RkBody( RkWorld* World );
		~RkBody();

		// World
		RkWorld* GetWorld() const;

		// Type
		RkBodyType GetType() const;
		void SetType( RkBodyType Type );

		// Shapes
		RkSphereShape* AddSphere( const RkSphere& Sphere );
		RkCapsuleShape* AddCapsule( const RkCapsule& Capsule );
		RkHullShape* AddHull( RkHull* Hull );
		RkMeshShape* AddMesh( RkBVH* BVH );
		bool RemoveShape( std::derived_from< RkShape > auto*& Shape );

		int GetShapeCount() const;
		RkShape* GetShape( int ShapeIndex );
		const RkShape* GetShape( int ShapeIndex ) const;
		const RkShapeList& GetShapes() const;

		// Mass
		float GetMass() const;
		float GetMassInv() const;
		RkVector3 GetMassCenter() const;
		RkVector3 GetLocalMassCenter() const;
		RkMatrix3 GetInertia() const;
		RkMatrix3 GetInertiaInv() const;
		RkMatrix3 GetLocalInertia() const;
		RkMatrix3 GetLocalInertiaInv() const;
		void RebuildMass();

		// Transform
		RkVector3 GetPosition() const;
		void SetPosition( const RkVector3& Position );
		RkQuaternion GetOrientation() const;
		void SetOrientation( const RkQuaternion& Orientation );
		RkTransform GetTransform() const;
		void SetTransform( const RkTransform& Transform );

		RkVector3 TransformVectorToLocal( const RkVector3& Vector ) const;
		RkVector3 TransformVectorToWorld( const RkVector3& Vector ) const;
		RkVector3 TransformPointToLocal( const RkVector3& Point ) const;
		RkVector3 TransformPointToWorld( const RkVector3& Point ) const;
		RkQuaternion TransformBasisToLocal( const RkQuaternion& Basis ) const;
		RkQuaternion TransformBasisToWorld( const RkQuaternion& Basis ) const;
	
		// Velocities
		RkVector3 GetLinearVelocity() const;
		void SetLinearVelocity( const RkVector3& LinearVelocity );
		RkVector3 GetAngularVelocity() const;
		void SetAngularVelocity( const RkVector3& AngularVelocity );
		
		void ApplyLinearImpule( const RkVector3& Impulse, bool Wake = true );
		void ApplyLinearImpulseAt( const RkVector3& Impulse, const RkVector3& Point, bool Wake = true );
		void ApplyAngularImpulse( const RkVector3& Impulse, bool Wake = true );

		bool SetVelocityFromKeyframe( const RkTransform& TargetTransform, float Timestep );

		// Forces
		void ApplyForce( const RkVector3& Force, bool Wake = true );
		void ApplyForceAt( const RkVector3& Force, const RkVector3& Point, bool Wake = true );
		void ApplyTorque( const RkVector3& Torque, bool Wake = true );
		
		RkVector3 GetNetForce() const;
		void ClearNetForce();
		RkVector3 GetNetTorque() const;
		void ClearNetTorque();

		// Sleeping
		bool IsAutoSleepingEnabled() const;
		void EnableAutoSleeping( bool Enable );
		
		void WakeUp();
		bool IsSleeping() const;

		// User data
		void* GetUserData() const;
		void SetUserData( void* UserData ) const;

	ragnarok:
		// World 
		int WorldIndex = -1;
		int SolverIndex = -1;
		uint32 Colors = 0;

		// Island
		RkIsland* GetIsland() const;
		void SetIsland( RkIsland* Island );
		int GetIslandIndex() const;
		void SetIslandIndex( int IslandIndex );
		
		// Constraint graph
		void AddJoint( RkJointEdge* Edge );
		void RemoveJoint( RkJointEdge* Edge );
		int GetJointCount() const;
		RkJointEdge* GetJointList();
		const RkJointEdge* GetJointList() const;

		void AddContact( RkContactEdge* Edge );
		void RemoveContact( RkContactEdge* Edge );
		int GetContactCount() const;
		RkContactEdge* GetContactList();
		const RkContactEdge* GetContactList() const;

		// Motion radii
		float GetMinMotionRadius() const;
		float GetMaxMotionRadius() const;

		// Solver
		RkVector3 IntegrateForces( float Timestep ) const;
		RkVector3 IntegrateTorques( float Timestep ) const;
		void IntegrateVelocities( const RkVector3& LinearVelocity, const RkVector3& AngularVelocity, float Timestep );
		bool UpdateShapes();
		void MoveProxies();
		
		// Sleeping
		float GetSleepTime() const;
		void SetSleepTime( float SleepTime );
		float AdvanceSleepTime( float Timestep );

	private:
		RkWorld* mWorld;

		RkBodyType mType;
		RkShapeList mShapeList;
		RkPool* mShapeAllocator;

		float mMassInv;
		RkVector3 mLocalMassCenter;
		RkVector3 mMassCenter;
		RkMatrix3 mLocalInertiaInv;
		RkMatrix3 mInertiaInv;
		
		RkVector3 mPosition;
		RkQuaternion mOrientation;
		RkVector3 mLinearVelocity;
		RkVector3 mAngularVelocity;
		RkVector3 mNetForce;
		RkVector3 mNetTorque;

		int mJointCount;
		RkJointEdge* mJointList;
		int mContactCount;
		RkContactEdge* mContactList;

		RkIsland* mIsland;
		int mIslandIndex;
		bool mAutoSleeping;
		float mSleepTime;

		float mMinMotionRadius;
		float mMaxMotionRadius;

		mutable void* mUserData;
	};