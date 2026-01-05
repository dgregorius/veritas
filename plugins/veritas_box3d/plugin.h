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
#include <taskscheduler.h>


struct VsBox3dHull;
struct VsBox3dMesh;

struct VsBox3dSphereShape;
struct VsBox3dCapsuleShape;
struct VsBox3dHullShape;
struct VsBox3dMeshShape;
struct VsBox3dBody;
struct VsBox3dWorld;
struct VsBox3dPlugin;


//--------------------------------------------------------------------------------------------------
// VsBox3dTask
//--------------------------------------------------------------------------------------------------
class VsBox3dTask : public enki::ITaskSet
	{
	public:
		virtual void ExecuteRange( enki::TaskSetPartition Range, uint32_t WorkerIndex ) override;

		b3TaskCallback* TaskCallback = nullptr;
		void* TaskContext = nullptr;
	};


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
	explicit VsBox3dHullShape( VsBox3dBody* Body, const VsBox3dHull* Hull );
	virtual ~VsBox3dHullShape();

	virtual const IVsHull* GetHull() const override;

	const VsBox3dHull* Hull = nullptr;
	b3ShapeId Native = {};
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
	explicit VsBox3dMeshShape( VsBox3dBody* Body, const VsBox3dMesh* Mesh );
	virtual ~VsBox3dMeshShape();

	virtual const IVsMesh* GetMesh() const override;

	const VsBox3dMesh* Mesh = nullptr;
	b3ShapeId Native = {};
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dBody
//--------------------------------------------------------------------------------------------------
struct VsBox3dBody :IVsBody
	{
	explicit VsBox3dBody( VsBox3dWorld* World, VsBodyType Type );
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

	b3BodyId Native = {};
	std::vector< IVsShape* > Shapes;
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dWorld
//--------------------------------------------------------------------------------------------------
struct VsBox3dWorld : IVsWorld
	{
	explicit VsBox3dWorld( enki::TaskScheduler& TaskScheduler );
	virtual ~VsBox3dWorld() override;

	virtual VsVector3 GetGravity() const override;
	virtual void SetGravity( const VsVector3& Gravity ) override;

	virtual IVsBody* CreateBody( VsBodyType Type ) override;
	virtual void DestroyBody( IVsBody* Body ) override;
	virtual int GetBodyCount() const override;
	virtual IVsBody* GetBody( int BodyIndex ) override;
	virtual const IVsBody* GetBody( int BodyIndex ) const override;

	virtual void Step( float Timestep ) override;

	int TaskCount = 0;
	enum { MaxTasks = 32 };
	VsBox3dTask TaskList[ MaxTasks ];
	enki::TaskScheduler& TaskScheduler;

	b3WorldId Native = {};
	std::vector< VsBox3dBody* > Bodies;
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dPlugin
//--------------------------------------------------------------------------------------------------
struct VsBox3dPlugin : IVsPlugin
	{
	VsBox3dPlugin();
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

	// Threading
	enki::TaskScheduler TaskScheduler;

	std::vector< VsBox3dHull* > Hulls;
	std::vector< VsBox3dMesh* > Meshes;
	std::vector< VsBox3dWorld* > Worlds;
	};