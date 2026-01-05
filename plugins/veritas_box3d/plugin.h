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

// Box3D
#include <box3d/box3d.h>


//--------------------------------------------------------------------------------------------------
// VsBox3Hull
//--------------------------------------------------------------------------------------------------
struct VsBox3dHull : IVsHull
	{
	explicit VsBox3dHull( b3Hull* Hull );
	virtual ~VsBox3dHull() override;

	virtual int GetVertexCount() const override;
	virtual const VsVector3* GetVertexPositions() const override;
	virtual const VsVector3* GetVertexNormals() const override;
	virtual int GetEdgeCount() const override;
	virtual const VsVector3* GetEdges() const override;

	b3Hull* Native = nullptr;
	std::vector< VsVector3 > VertexPositions;
	std::vector< VsVector3 > VertexNormals;
	std::vector< VsVector3 > Edges;
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dHullShape
//--------------------------------------------------------------------------------------------------
struct VsBox3dHullShape : IVsHullShape
	{
	explicit VsBox3dHullShape( b3ShapeId ShapeId, const IVsHull* Hull );
	virtual ~VsBox3dHullShape();

	virtual const IVsHull* GetHull() const override;

	b3ShapeId ShapeId = {};
	const IVsHull* Hull = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dMesh
//--------------------------------------------------------------------------------------------------
struct VsBox3dMesh : IVsMesh
	{
	explicit VsBox3dMesh( b3MeshData* Mesh );
	virtual ~VsBox3dMesh() override;

	virtual int GetVertexCount() const override;
	virtual const VsVector3* GetVertexPositions() const override;
	virtual const VsVector3* GetVertexNormals() const override;

	b3MeshData* Native = nullptr;
	std::vector< VsVector3 > VertexPositions;
	std::vector< VsVector3 > VertexNormals;
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dMeshShape
//--------------------------------------------------------------------------------------------------
struct VsBox3dMeshShape : IVsMeshShape
	{
	explicit VsBox3dMeshShape( b3ShapeId ShapeId, const IVsMesh* Mesh );
	virtual ~VsBox3dMeshShape();

	virtual const IVsMesh* GetMesh() const override;

	b3ShapeId ShapeId = {};
	const IVsMesh* Mesh = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dBody
//--------------------------------------------------------------------------------------------------
struct VsBox3dBody :IVsBody
	{
	VsBox3dBody( b3BodyId BodyId );
	virtual ~VsBox3dBody() override;

	virtual VsBodyType GetType() const override;

	virtual VsVector3 GetPosition() const override;
	virtual void SetPosition( const VsVector3& Position ) override;
	virtual VsQuaternion GetOrientation() const override;
	virtual void SetOrientation( const VsQuaternion& Orientation ) override;

	virtual IVsSphereShape* CreateSphere( const VsVector3& Center, float Radius ) override;
	virtual IVsCapsuleShape* CreateCapulse( const VsVector3& Center1, const VsVector3& Center2, float Radius ) override;
	virtual IVsHullShape* CreateHull( const IVsHull* Hull ) override;
	virtual IVsMeshShape* CreateMesh( const IVsMesh* Mesh ) override;
	virtual void DestroyShape( IVsShape* Shape ) override;

	b3BodyId BodyId = {};
	std::vector< IVsShape* > Shapes;
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dWorld
//--------------------------------------------------------------------------------------------------
struct VsBox3dWorld : IVsWorld
	{
	explicit VsBox3dWorld( b3WorldId WorldId );
	virtual ~VsBox3dWorld() override;

	virtual VsVector3 GetGravity() const override;
	virtual void SetGravity( const VsVector3& Gravity ) override;

	virtual IVsBody* CreateBody( VsBodyType Type ) override;
	virtual void DestroyBody( IVsBody* Body ) override;

	b3WorldId WorldId = {};
	std::vector< IVsBody* > Bodies;
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dPlugin
//--------------------------------------------------------------------------------------------------
struct VsBox3dPlugin : IVsPlugin
	{
	virtual ~VsBox3dPlugin() override;

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