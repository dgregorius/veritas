//--------------------------------------------------------------------------------------------------
/**
	@file		physxplugin.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <veritas/veritas.h>

// PhysX
#include <PxPhysicsAPI.h>
#include <Extensions/PxDefaultAllocator.h>
using namespace physx;

// STL includes
#include <algorithm>
#include <thread>

// Forward 
class VsPhysXSphereShape;
class VsPhysXCapsuleShape;
class VsPhysXHull;
class VsPhysXHullShape;
class VsPhysXMesh;
class VsPhysXMeshShape;
class VsPhysXBody;
class VsPhysXWorld;
class VsPhysXPlugin;


//--------------------------------------------------------------------------------------------------
// VsPhysXHull
//--------------------------------------------------------------------------------------------------
class VsPhysXHull : public IVsHull
	{
	public:
		// Construction / Destruction
		explicit VsPhysXHull( PxConvexMesh* ConvexMesh );
		virtual ~VsPhysXHull() override;

		// IVsHull
		virtual int GetVertexCount() const override;
		virtual const VsVector3* GetVertexPositions() const override;
		virtual const VsVector3* GetVertexNormals() const override;

		virtual int GetEdgeCount() const override;
		virtual const VsVector3* GetEdgePositions() const override;

		// PhysX Box3d type
		PxConvexMesh* GetNative() const;

	private:
		std::vector< VsVector3 > mVertexPositions;
		std::vector< VsVector3 > mVertexNormals;
		std::vector< VsVector3 > mEdgePositions;
		PxConvexMesh* mNative = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsPhysXHullShape
//--------------------------------------------------------------------------------------------------
class VsPhysXHullShape : public IVsHullShape
	{
	public:
		// Construction / Destruction
		explicit VsPhysXHullShape( VsPhysXBody* Body, const VsPhysXHull* Hull );
		virtual ~VsPhysXHullShape();

		// IVsShape
		virtual VsShapeType GetType() const override;
		virtual IVsBody* GetBody() const override;
		virtual VsColor GetColor() const override;
		virtual void SetColor( const VsColor& Color ) override;

		// IVsHullShape
		virtual const IVsHull* GetHull() const override;

		// Material
		void SetFriction( float Friction );
		void SetRestitution( float Restitution );

		// PhysX
		PxShape* GetNative() const;

	private:
		VsPhysXBody* mBody = nullptr;
		VsColor mColor = VS_COLOR_TRANSPARENT;
		const VsPhysXHull* mHull = nullptr;
		PxMaterial* mMaterial = nullptr;
		PxShape* mNative = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsPhysXMesh
//--------------------------------------------------------------------------------------------------
class VsPhysXMesh : public IVsMesh
	{
	public:
		// Construction / Destruction
		explicit VsPhysXMesh();
		virtual ~VsPhysXMesh() override;

		// IVsMesh
		virtual int GetVertexCount() const override;
		virtual const VsVector3* GetVertexPositions() const override;
		virtual const VsVector3* GetVertexNormals() const override;

	private:
		std::vector< VsVector3 > mVertexPositions;
		std::vector< VsVector3 > mVertexNormals;
	};


//--------------------------------------------------------------------------------------------------
// VsPhysXBody
//--------------------------------------------------------------------------------------------------
class VsPhysXBody : public IVsBody
	{
	public:
		// Construction / Destruction
		explicit VsPhysXBody( VsPhysXWorld* World, VsBodyType Type );
		virtual ~VsPhysXBody() override;

		// World
		virtual IVsWorld* GetWorld() const override;

		// Type
		virtual VsBodyType GetType() const override;

		// Transform
		virtual VsVector3 GetPosition() const override;
		virtual void SetPosition( const VsVector3& Position ) override;
		virtual VsQuaternion GetOrientation() const override;
		virtual void SetOrientation( const VsQuaternion& Orientation ) override;

		// Velocity
		virtual VsVector3 GetLinearVelocity() const override;
		virtual void SetLinearVelocity( const VsVector3& LinearVelocity ) override;
		virtual VsVector3 GetAngularVelocity() const override;
		virtual void SetAngularVelocity( const VsVector3& AngularVelocity ) override;

		virtual void SetVelocityFromKeyframe( const VsFrame& Keyframe, float Timestep ) override;

		// Materials
		virtual float GetFriction() const override;
		virtual void SetFriction( float Friction ) override;
		virtual float GetRestitution() const override;
		virtual void SetRestitution( float Restitution ) override;

		// Shapes
		virtual IVsSphereShape* CreateSphere( const VsVector3& Center, float Radius ) override;
		virtual IVsCapsuleShape* CreateCapulse( const VsVector3& Center1, const VsVector3& Center2, float Radius ) override;
		virtual IVsHullShape* CreateHull( const IVsHull* Hull ) override;
		virtual IVsMeshShape* CreateMesh( const IVsMesh* Mesh ) override;
		virtual void DestroyShape( IVsShape* Shape ) override;

		virtual int GetShapeCount() const override;
		virtual IVsShape* GetShape( int ShapeIndex ) override;
		virtual const IVsShape* GetShape( int ShapeIndex ) const override;

		// PhysX
		PxRigidActor* GetNative() const;

	private:
		VsPhysXWorld* mWorld = nullptr;
		float mFriction = 0.6f;
		float mRestitution = 0.0f;
		std::vector< IVsShape* > mShapes;
		PxRigidActor* mNative = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsPhysXWorld
//--------------------------------------------------------------------------------------------------
class VsPhysXWorld : public IVsWorld
	{
	public:
		// Construction / Destruction
		explicit VsPhysXWorld( VsPhysXPlugin* Plugin );
		virtual ~VsPhysXWorld() override;

		// Plugin
		virtual IVsPlugin* GetPlugin() const override;

		// Color
		virtual VsColor GetColor() const override;
		virtual void SetColor( const VsColor& Color ) override;

		// Events
		virtual void AddListener( IVsWorldListener* Listener ) override;
		virtual void RemoveListener( IVsWorldListener* Listener ) override;

		void NotifyBodyAdded( IVsBody* Body );
		void NotifyBodyRemoved( IVsBody* Body );
		void NotifyShapeAdded( IVsBody* Body, IVsShape* Shape );
		void NotifyShapeRemoved( IVsBody* Body, IVsShape* Shape );

		// Gravity
		virtual VsVector3 GetGravity() const override;
		virtual void SetGravity( const VsVector3& Gravity ) override;

		// Bodies
		virtual IVsBody* CreateBody( VsBodyType Type ) override;
		virtual void DestroyBody( IVsBody* Body ) override;
		virtual int GetBodyCount() const override;
		virtual IVsBody* GetBody( int BodyIndex ) override;
		virtual const IVsBody* GetBody( int BodyIndex ) const override;

		// Simulation
		virtual void Step( float Timestep ) override;

		// PhysX 
		PxScene* GetNative() const;

	private:
		VsPhysXPlugin* mPlugin = nullptr;
		VsColor mColor = { 0.9f, 0.8f, 0.5f, 1.00f };
		std::vector< IVsWorldListener* > mListeners;
		std::vector< VsPhysXBody* > mBodies;

		PxScene* mNative = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsPhysXPlugin
//--------------------------------------------------------------------------------------------------
class VsPhysXPlugin : public IVsPlugin
	{
	public:
		// Construction / Destruction
		explicit VsPhysXPlugin( ImGuiContext* Context );
		virtual ~VsPhysXPlugin() override;

		// Module
		virtual void Release() override;
		virtual const char* GetName() const override;
		virtual const char* GetVersion() const override;

		virtual bool IsEnabled() const override;
		virtual void SetEnabled( bool Enabled ) override;

		virtual bool OnInspectorGUI() override;

		// Hulls
		virtual IVsHull* CreateHull( int VertexCount, const VsVector3* Vertices ) override;
		virtual void DestroyHull( IVsHull* Hull ) override;

		virtual int GetHullCount() const override;
		virtual IVsHull* GetHull( int HullIndex ) override;
		virtual const IVsHull* GetHull( int HullIndex ) const override;

		// Meshes
		virtual IVsMesh* CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices ) override;
		virtual void DestroyMesh( IVsMesh* Mesh ) override;

		virtual int GetMeshCount() const override;
		virtual IVsMesh* GetMesh( int MeshIndex ) override;
		virtual const IVsMesh* GetMesh( int MeshIndex ) const override;

		// Worlds
		virtual IVsWorld* CreateWorld() override;
		virtual void DestroyWorld( IVsWorld* World ) override;

		virtual int GetWorldCount() const override;
		virtual IVsWorld* GetWorld( int WorldIndex ) override;
		virtual const IVsWorld* GetWorld( int WorldIndex ) const override;

		// Shared
		PxFoundation* GetFoundation() const;
		PxPhysics* GetPhysics() const;
		PxCpuDispatcher* GetDispatcher() const;

	private:
		char mVersion[ 64 ];
		bool mEnabled = true;
		std::vector< VsPhysXHull* > mHulls;
		std::vector< VsPhysXMesh* > mMeshes;
		std::vector< VsPhysXWorld* > mWorlds;

		PxDefaultAllocator mAllocator;
		PxDefaultErrorCallback mErrorCallback;
		PxFoundation* mFoundation = nullptr;
		PxPhysics* mPhysics = nullptr;
		PxDefaultCpuDispatcher* mDispatcher = nullptr;
	};