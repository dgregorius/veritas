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

// Forward declarations
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

		// Events
		virtual void AddListener( IVsWorldListener* Listener ) override;
		virtual void RemoveListener( IVsWorldListener* Listener ) override;

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

	private:
		std::vector< IVsHull* > mHulls;
		std::vector< IVsMesh* > mMeshes;
		std::vector< IVsWorld* > mWorlds;
	};