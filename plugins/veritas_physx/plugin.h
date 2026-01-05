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


//--------------------------------------------------------------------------------------------------
// VsPhysXPlugin
//--------------------------------------------------------------------------------------------------
struct VsPhysXPlugin : IVsPlugin
	{
	virtual ~VsPhysXPlugin() override;

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

	std::vector< IVsHull* > Hulls;
	std::vector< IVsMesh* > Meshes;
	std::vector< IVsWorld* > Worlds;
	};