//--------------------------------------------------------------------------------------------------
/**
	@file		plugin.h

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

	private:
		VsPhysXPlugin* mPlugin = nullptr;
		VsColor mColor = { 1.0f, 0.83f, 0.61f, 1.0f };
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
		VsPhysXPlugin();
		virtual ~VsPhysXPlugin() override;

		// Module
		virtual void Release() override;
		virtual const char* GetName() const override;
		virtual const char* GetVersion() const override;

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
		std::vector< IVsHull* > mHulls;
		std::vector< IVsMesh* > mMeshes;
		std::vector< IVsWorld* > mWorlds;

		PxDefaultAllocator mAllocator;
		PxDefaultErrorCallback mErrorCallback;
		PxFoundation* mFoundation = nullptr;
		PxPhysics* mPhysics = nullptr;
		PxDefaultCpuDispatcher* mDispatcher = nullptr;
	};