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

// Forward
class VsBox3dTask;
class VsBox3dSphereShape;
class VsBox3dCapsuleShape;
class VsBox3dHull;
class VsBox3dHullShape;
class VsBox3dMesh;
class VsBox3dMeshShape;
class VsBox3dBody;
class VsBox3dWorld;
class VsBox3dPlugin;


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
class VsBox3dHull : public IVsHull
	{
	public:
		// Construction / Destruction
		explicit VsBox3dHull( b3Hull* Hull );
		virtual ~VsBox3dHull() override;
		
		// IVsHull
		virtual int GetVertexCount() const override;
		virtual const VsVector3* GetVertexPositions() const override;
		virtual const VsVector3* GetVertexNormals() const override;

		virtual int GetEdgeCount() const override;
		virtual const VsVector3* GetEdgePositions() const override;

		// Native Box3d type
		b3Hull* GetNative() const;

	private:
		std::vector< VsVector3 > mVertexPositions;
		std::vector< VsVector3 > mVertexNormals;
		std::vector< VsVector3 > mEdges;
		b3Hull* mNative = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dHullShape
//--------------------------------------------------------------------------------------------------
class VsBox3dHullShape : public IVsHullShape
	{
	public:
		// Construction / Destruction
		explicit VsBox3dHullShape( VsBox3dBody* Body, const VsBox3dHull* Hull );
		virtual ~VsBox3dHullShape();

		// IVsShape
		virtual VsShapeType GetType() const override;
		virtual IVsBody* GetBody() const override;
		virtual VsColor GetColor() const override;
		virtual void SetColor( const VsColor& Color ) override;

		// IVsHullShape
		virtual const IVsHull* GetHull() const override;

		// Native Box3d type
		b3ShapeId GetNative() const;

	private:
		VsBox3dBody* mBody = nullptr;
		VsColor mColor = VS_COLOR_TRANSPARENT;
		const VsBox3dHull* mHull = nullptr;
		b3ShapeId mNative = {};
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dMesh
//--------------------------------------------------------------------------------------------------
class VsBox3dMesh : public IVsMesh
	{
	public:
		// Construction / Destruction
		explicit VsBox3dMesh( b3MeshData* Mesh );
		virtual ~VsBox3dMesh() override;

		// IVsMesh
		virtual int GetVertexCount() const override;
		virtual const VsVector3* GetVertexPositions() const override;
		virtual const VsVector3* GetVertexNormals() const override;

		// Native Box3d type
		b3MeshData* GetNative() const;

	private:
		std::vector< VsVector3 > mVertexPositions;
		std::vector< VsVector3 > mVertexNormals;
		b3MeshData* mNative = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dMeshShape
//--------------------------------------------------------------------------------------------------
class VsBox3dMeshShape : public IVsMeshShape
	{
	public:
		// Construction / Destruction
		VsBox3dMeshShape( VsBox3dBody* Body, const VsBox3dMesh* Mesh );
		virtual ~VsBox3dMeshShape();

		// IVsShape
		virtual VsShapeType GetType() const override;
		virtual IVsBody* GetBody() const override;
		virtual VsColor GetColor() const override;
		virtual void SetColor( const VsColor& Color ) override;

		// IVsMeshShape
		virtual const IVsMesh* GetMesh() const override;

		// Native Box3d type
		b3ShapeId GetNative() const;

	private:
		VsBox3dBody* mBody = nullptr;
		VsColor mColor = VS_COLOR_TRANSPARENT;
		const VsBox3dMesh* mMesh = nullptr;
		b3ShapeId mNative = {};
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dBody
//--------------------------------------------------------------------------------------------------
class VsBox3dBody : public IVsBody
	{
	public:
		// Construction / Destruction
		explicit VsBox3dBody( VsBox3dWorld* World, VsBodyType Type );
		virtual ~VsBox3dBody() override;

		// World
		virtual IVsWorld* GetWorld() const override;

		// Type
		virtual VsBodyType GetType() const override;

		// Transform
		virtual VsVector3 GetPosition() const override;
		virtual void SetPosition( const VsVector3& Position ) override;
		virtual VsQuaternion GetOrientation() const override;
		virtual void SetOrientation( const VsQuaternion& Orientation ) override;

		// Shapes
		virtual IVsSphereShape* CreateSphere( const VsVector3& Center, float Radius ) override;
		virtual IVsCapsuleShape* CreateCapulse( const VsVector3& Center1, const VsVector3& Center2, float Radius ) override;
		virtual IVsHullShape* CreateHull( const IVsHull* Hull ) override;
		virtual IVsMeshShape* CreateMesh( const IVsMesh* Mesh ) override;
		virtual void DestroyShape( IVsShape* Shape ) override;

		virtual int GetShapeCount() const override;
		virtual IVsShape* GetShape( int ShapeIndex ) override;
		virtual const IVsShape* GetShape( int ShapeIndex ) const override;

		// Native
		b3BodyId GetNative() const;

	private:	
		VsBox3dWorld* mWorld = nullptr;
		std::vector< IVsShape* > mShapes;
		b3BodyId mNative = {};
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dWorld
//--------------------------------------------------------------------------------------------------
class VsBox3dWorld : public IVsWorld
	{
	public:
		// Construction / Destruction
		explicit VsBox3dWorld( VsBox3dPlugin* Plugin );
		virtual ~VsBox3dWorld() override;

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

		// Native 
		b3WorldId GetNative() const;

	private:
		static void* EnqueueTask( b3TaskCallback* TaskCallback, int ItemCount, int MinRange, void* TaskContext, void* UserContext );
		void* EnqueueTask( b3TaskCallback* TaskCallback, int ItemCount, int MinRange, void* TaskContext );
		static void FinishTask( void* Task, void* UserContext );
		void FinishTask( void* Task );

		VsBox3dPlugin* mPlugin = nullptr;
		VsColor mColor = { 0.41f, 0.55f, 1.0f, 1.0f };
		std::vector< IVsWorldListener* > mListeners;
		std::vector< VsBox3dBody* > mBodies;
		

		int mTaskCount = 0;
		enum { MaxTasks = 32 };
		VsBox3dTask mTaskList[ MaxTasks ];
		b3WorldId mNative = {};
	};


//--------------------------------------------------------------------------------------------------
// VsBox3dPlugin
//--------------------------------------------------------------------------------------------------
class VsBox3dPlugin : public IVsPlugin
	{
	public:
		// Construction / Destruction
		VsBox3dPlugin();
		virtual ~VsBox3dPlugin() override;

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

		// Scheduler
		enki::TaskScheduler& GetTaskScheduler();
		const enki::TaskScheduler& GetTaskScheduler() const;

	private:
		std::vector< VsBox3dHull* > mHulls;
		std::vector< VsBox3dMesh* > mMeshes;
		std::vector< VsBox3dWorld* > mWorlds;

		enki::TaskScheduler mTaskScheduler;
	};