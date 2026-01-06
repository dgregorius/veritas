//--------------------------------------------------------------------------------------------------
// plugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "plugin.h"


//--------------------------------------------------------------------------------------------------
// VsBox3dTask
//--------------------------------------------------------------------------------------------------
void VsBox3dTask::ExecuteRange( enki::TaskSetPartition Range, uint32_t WorkerIndex )
	{
	TaskCallback( Range.start, Range.end, WorkerIndex, TaskContext );
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dHull
//--------------------------------------------------------------------------------------------------
VsBox3dHull::VsBox3dHull( b3Hull* Hull )
	{
	B3_ASSERT( Hull );
	mNative = Hull;

	int FaceCount = Hull->faceCount;
	if ( FaceCount > 0  )
		{
		mVertexPositions.reserve( 3 * FaceCount );
		mVertexNormals.reserve( 3 * FaceCount );

		const b3Vec3* HullVertices = b3GetHullPoints( Hull );
		const b3HullHalfEdge* HullEdges = b3GetHullEdges( Hull );
		const b3HullFace* HullFaces = b3GetHullFaces( Hull );
		const b3Plane* HullPlanes = b3GetHullPlanes( Hull );
		
		for ( int FaceIndex = 0; FaceIndex < FaceCount; ++FaceIndex )
			{
			const b3HullFace* Face = HullFaces + FaceIndex;
			const b3Plane& FacePlane = HullPlanes[ FaceIndex ];
			b3Vec3 FaceNormal = FacePlane.normal;

			const b3HullHalfEdge* Edge1 = HullEdges + Face->edge ;
			const b3HullHalfEdge* Edge2 = HullEdges + Edge1->next;
			const b3HullHalfEdge* Edge3 = HullEdges + Edge2->next;
			
			do
				{
				int VertexIndex1 = Edge1->origin;
				b3Vec3 Vertex1 = HullVertices[ VertexIndex1 ];
				mVertexPositions.push_back( { Vertex1.x, Vertex1.y, Vertex1.z } );
				mVertexNormals.push_back( { FaceNormal.x, FaceNormal.y, FaceNormal.z } );
				
				int VertexIndex2 = Edge2->origin;
				b3Vec3 Vertex2 = HullVertices[ VertexIndex2 ];
				mVertexPositions.push_back( { Vertex2.x, Vertex2.y, Vertex2.z } );
				mVertexNormals.push_back( { FaceNormal.x, FaceNormal.y, FaceNormal.z } );
				
				int VertexIndex3 = Edge3->origin;
				b3Vec3 Vertex3 = HullVertices[ VertexIndex3 ];
				mVertexPositions.push_back( { Vertex3.x, Vertex3.y, Vertex3.z } );
				mVertexNormals.push_back( { FaceNormal.x, FaceNormal.y, FaceNormal.z } );

				Edge2 = Edge3;
				Edge3 = HullEdges + Edge3->next;
				}
			while ( Edge1 != Edge3 );
			}

		int EdgeCount = Hull->edgeCount;
		
		if ( EdgeCount > 0 && HullEdges )
			{
			mEdges.resize( EdgeCount );
			for ( int EdgeIndex = 0; EdgeIndex < EdgeCount; EdgeIndex += 2 )
				{
				const b3HullHalfEdge* Edge = HullEdges + EdgeIndex;
				const b3HullHalfEdge* Twin = HullEdges + Edge->twin;

				int VertexIndex1 = Edge->origin;
				b3Vec3 Vertex1 = HullVertices[ VertexIndex1 ];
				int VertexIndex2 = Twin->origin;
				b3Vec3 Vertex2 = HullVertices[ VertexIndex2 ];

				mEdges[ EdgeIndex + 0 ] = { Vertex1.x, Vertex1.y, Vertex1.z };
				mEdges[ EdgeIndex + 1 ] = { Vertex2.x, Vertex2.y, Vertex2.z };
				}
			}
		}
	}


//--------------------------------------------------------------------------------------------------
VsBox3dHull::~VsBox3dHull()
	{
	b3DestroyHull( mNative );
	}


//--------------------------------------------------------------------------------------------------
b3Hull* VsBox3dHull::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dHull::GetVertexCount() const
	{
	return static_cast< int >( mVertexPositions.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dHull::GetVertexPositions() const
	{
	return mVertexPositions.data();
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dHull::GetVertexNormals() const
	{
	return mVertexNormals.data();
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dHull::GetEdgeCount() const
	{
	return static_cast< int >( mEdges.size() / 2 );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dHull::GetEdges() const
	{
	return mEdges.data();
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dHullShape
//--------------------------------------------------------------------------------------------------
VsBox3dHullShape::VsBox3dHullShape( VsBox3dBody* Body, const VsBox3dHull* Hull )
	: mBody( Body )
	, mHull( Hull )
	{
	b3ShapeDef ShapeDef = b3DefaultShapeDef();
	mNative = b3CreateHullShape( Body->GetNative(), &ShapeDef, Hull->GetNative() );
	}


//--------------------------------------------------------------------------------------------------
VsBox3dHullShape::~VsBox3dHullShape()
	{
	b3DestroyShape( mNative, true );
	}


//--------------------------------------------------------------------------------------------------
b3ShapeId VsBox3dHullShape::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
VsShapeType VsBox3dHullShape::GetType() const
	{
	return VS_HULL_SHAPE;
	}


//--------------------------------------------------------------------------------------------------
const IVsHull* VsBox3dHullShape::GetHull() const
	{
	return mHull;
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dMesh
//--------------------------------------------------------------------------------------------------
VsBox3dMesh::VsBox3dMesh( b3MeshData* Mesh )
	{
	B3_ASSERT( Mesh );
	mNative = Mesh;

	int TriangleCount = 3 * Mesh->triangleCount;
	const b3MeshTriangle* MeshTriangles = b3GetMeshTriangles( Mesh );
	const b3Vec3* MeshVertices = b3GetMeshVertices( Mesh );
	if ( TriangleCount > 0 && MeshTriangles && MeshVertices )
		{
		mVertexPositions.resize( 3 * TriangleCount );
		mVertexNormals.resize( 3 * TriangleCount );
		for ( int TriangleIndex = 0; TriangleIndex < TriangleCount; ++TriangleIndex )
			{
			int VertexIndex1 = MeshTriangles[ TriangleIndex ].index1;
			b3Vec3 Vertex1 = MeshVertices[ VertexIndex1 ];
			int VertexIndex2 = MeshTriangles[ TriangleIndex ].index2;
			b3Vec3 Vertex2 = MeshVertices[ VertexIndex2 ];
			int VertexIndex3 = MeshTriangles[ TriangleIndex ].index3;
			b3Vec3 Vertex3 = MeshVertices[ VertexIndex3 ];

			b3Vec3 Edge1 = b3Sub( Vertex2, Vertex1 );
			b3Vec3 Edge2 = b3Sub( Vertex3, Vertex1 );
			b3Vec3 Normal = b3Cross( Edge1, Edge2 );
			Normal = b3Normalize( Normal );

			mVertexPositions[ 3 * TriangleIndex + 0 ] = { Vertex1.x, Vertex1.y, Vertex1.z };
			mVertexNormals[ 3 * TriangleIndex + 0 ] = { Normal.x, Normal.y, Normal.z };
			mVertexPositions[ 3 * TriangleIndex + 1 ] = { Vertex2.x, Vertex2.y, Vertex2.z };
			mVertexNormals[ 3 * TriangleIndex + 1 ] = { Normal.x, Normal.y, Normal.z };
			mVertexPositions[ 3 * TriangleIndex + 2 ] = { Vertex3.x, Vertex3.y, Vertex3.z };
			mVertexNormals[ 3 * TriangleIndex + 2 ] = { Normal.x, Normal.y, Normal.z };
			}
		}
	}


//--------------------------------------------------------------------------------------------------
VsBox3dMesh::~VsBox3dMesh()
	{
	b3DestroyMesh( mNative );
	}


//--------------------------------------------------------------------------------------------------
b3MeshData* VsBox3dMesh::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dMesh::GetVertexCount() const
	{
	return static_cast< int >( mVertexPositions.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dMesh::GetVertexPositions() const
	{
	return mVertexPositions.data();
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dMesh::GetVertexNormals() const
	{
	return mVertexNormals.data();
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dMeshShape
//--------------------------------------------------------------------------------------------------
VsBox3dMeshShape::VsBox3dMeshShape( VsBox3dBody* Body, const VsBox3dMesh* Mesh )
	: mBody( Body )
	, mMesh( Mesh )
	{
	b3ShapeDef ShapeDef = b3DefaultShapeDef();
	mNative = b3CreateMeshShape( Body->GetNative(), &ShapeDef, Mesh->GetNative(), { 1.0f, 1.0f, 1.0f } );
	}


//--------------------------------------------------------------------------------------------------
VsBox3dMeshShape::~VsBox3dMeshShape()
	{
	b3DestroyShape( mNative, false );
	}


//--------------------------------------------------------------------------------------------------
b3ShapeId VsBox3dMeshShape::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
VsShapeType VsBox3dMeshShape::GetType() const
	{
	return VS_MESH_SHAPE;
	}


//--------------------------------------------------------------------------------------------------
const IVsMesh* VsBox3dMeshShape::GetMesh() const
	{
	return mMesh;
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dBody
//--------------------------------------------------------------------------------------------------
VsBox3dBody::VsBox3dBody( VsBox3dWorld* World, VsBodyType Type )
	: mWorld( World )
	{
	const b3BodyType TypeMap[] = { b3_staticBody, b3_kinematicBody, b3_staticBody };

	b3BodyDef BodyDef = b3DefaultBodyDef();
	BodyDef.type = TypeMap[ Type ];

	mNative = b3CreateBody( World->GetNative(), &BodyDef );
	}


//--------------------------------------------------------------------------------------------------
VsBox3dBody::~VsBox3dBody()
	{
	b3DestroyBody( mNative );
	}


//--------------------------------------------------------------------------------------------------
b3BodyId VsBox3dBody::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
VsBodyType VsBox3dBody::GetType() const
	{
	b3BodyType Type = b3Body_GetType( mNative );

	VsBodyType TypeMap[] = { VS_STATIC_BODY, VS_KEYFRAMED_BODY, VS_STATIC_BODY };
	return TypeMap[ Type ];
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsBox3dBody::GetPosition() const
	{
	b3Vec3 Position = b3Body_GetPosition( mNative );
	return VsVector3{ Position.x, Position.y, Position.z };
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dBody::SetPosition( const VsVector3& Position )
	{
	b3Quat Rotation = b3Body_GetRotation( mNative );
	b3Body_SetTransform( mNative, b3Vec3{ Position.X, Position.Y, Position.Z }, Rotation );
	}


//--------------------------------------------------------------------------------------------------
VsQuaternion VsBox3dBody::GetOrientation() const
	{
	b3Quat Rotation = b3Body_GetRotation( mNative );
	return VsQuaternion{ Rotation.v.x, Rotation.v.y, Rotation.v.z, Rotation.s };
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dBody::SetOrientation( const VsQuaternion& Orientation )
	{
	b3Vec3 Position = b3Body_GetPosition( mNative );
	b3Body_SetTransform( mNative, Position, b3Quat( { Orientation.X, Orientation.Y, Orientation.Z }, Orientation.W ) );
	}


//--------------------------------------------------------------------------------------------------
IVsSphereShape* VsBox3dBody::CreateSphere( const VsVector3& Center, float Radius )
	{
	// DIRK_TODO: ...
	B3_ASSERT( 0 );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsCapsuleShape* VsBox3dBody::CreateCapulse( const VsVector3& Center1, const VsVector3& Center2, float Radius )
	{
	// DIRK_TODO: ...
	B3_ASSERT( 0 );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsHullShape* VsBox3dBody::CreateHull( const IVsHull* Hull )
	{
	if ( !Hull )
		{
		return nullptr;
		}

	VsBox3dHullShape* Shape = new VsBox3dHullShape( this, static_cast< const VsBox3dHull* >( Hull ) );
	mShapes.push_back( Shape );
	mWorld->NotifyShapeAdded( this, Shape );

	return Shape;
	}


//--------------------------------------------------------------------------------------------------
IVsMeshShape* VsBox3dBody::CreateMesh( const IVsMesh* Mesh )
	{
	if ( !Mesh )
		{
		return nullptr;
		}

	VsBox3dMeshShape* Shape = new VsBox3dMeshShape( this, static_cast< const VsBox3dMesh* >( Mesh ) );
	mShapes.push_back( Shape );
	mWorld->NotifyShapeAdded( this, Shape );
	
	return Shape;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dBody::DestroyShape( IVsShape* Shape )
	{
	if ( !Shape )
		{
		return;
		}

	mWorld->NotifyShapeRemoved( this, Shape );
	std::erase( mShapes, Shape );

	DeleteShape( Shape );
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dBody::GetShapeCount() const
	{
	return static_cast< int >( mShapes.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsShape* VsBox3dBody::GetShape( int ShapeIndex )
	{
	return ( 0 <= ShapeIndex && ShapeIndex < GetShapeCount() ) ? mShapes[ ShapeIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsShape* VsBox3dBody::GetShape( int ShapeIndex ) const
	{
	return ( 0 <= ShapeIndex && ShapeIndex < GetShapeCount() ) ? mShapes[ ShapeIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dWorld
//--------------------------------------------------------------------------------------------------
VsBox3dWorld::VsBox3dWorld( VsBox3dPlugin* Plugin )
	: mPlugin( Plugin )
	{
	enki::TaskScheduler& TaskScheduler = Plugin->GetTaskScheduler();

	b3WorldDef WorldDef = b3DefaultWorldDef();
	WorldDef.gravity = { 0.0f, -10.0f, 0.0f };
	WorldDef.workerCount = TaskScheduler.GetNumTaskThreads();
	WorldDef.enqueueTask = EnqueueTask;
	WorldDef.finishTask = FinishTask;

	mNative = b3CreateWorld( &WorldDef );
	}


//--------------------------------------------------------------------------------------------------
VsBox3dWorld::~VsBox3dWorld()
	{
	while ( !mBodies.empty() )
		{
		VsBox3dBody* Body = mBodies.back();
		mBodies.pop_back();

		delete Body;
		}

	b3DestroyWorld( mNative );
	}


//--------------------------------------------------------------------------------------------------
b3WorldId VsBox3dWorld::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::AddListener( IVsWorldListener* Listener )
	{
	// Add 
	if ( !Listener )
		{
		return;
		}

	if ( std::find( mListeners.begin(), mListeners.end(), Listener ) != mListeners.end() )
		{
		return;
		}
	mListeners.push_back( Listener );

	// Sync
	for ( VsBox3dBody* Body : mBodies )
		{
		Listener->OnBodyAdded( Body );
		for ( int ShapeIndex = 0; ShapeIndex < Body->GetShapeCount(); ++ShapeIndex )
			{
			IVsShape* Shape = Body->GetShape( ShapeIndex );
			Listener->OnShapeAdded( Body, Shape );
			}
		}
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::RemoveListener( IVsWorldListener* Listener )
	{
	if ( !Listener )
		{
		return;
		}

	std::erase( mListeners, Listener );
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::NotifyBodyAdded( IVsBody* Body )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnBodyAdded( Body ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::NotifyBodyRemoved( IVsBody* Body )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnBodyRemoved( Body ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::NotifyShapeAdded( IVsBody* Body, IVsShape* Shape )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnShapeAdded( Body, Shape ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::NotifyShapeRemoved( IVsBody* Body, IVsShape* Shape )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnShapeRemoved( Body, Shape ); } );
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsBox3dWorld::GetGravity() const
	{
	b3Vec3 Gravity = b3World_GetGravity( mNative );
	return { Gravity.x, Gravity.y, Gravity.z };
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::SetGravity( const VsVector3& Gravity )
	{
	b3World_SetGravity( mNative, { Gravity.X, Gravity.Y, Gravity.Z } );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsBox3dWorld::CreateBody( VsBodyType Type )
	{
	VsBox3dBody* Body = new VsBox3dBody( this, Type );
	mBodies.push_back( Body );
	NotifyBodyAdded( Body );

	return Body;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::DestroyBody( IVsBody* Body )
	{
	if ( !Body )
		{
		return;
		}

	NotifyBodyRemoved( Body );
	std::erase( mBodies, Body );
	delete static_cast< VsBox3dBody* >( Body );
	}



//--------------------------------------------------------------------------------------------------
int VsBox3dWorld::GetBodyCount() const
	{
	return static_cast< int >( mBodies.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsBox3dWorld::GetBody( int BodyIndex )
	{
	return ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsBody* VsBox3dWorld::GetBody( int BodyIndex ) const
	{
	return ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::Step( float Timestep )
	{
	b3World_Step( mNative, Timestep, 4 );
	mTaskCount = 0;
	}


//--------------------------------------------------------------------------------------------------
void* VsBox3dWorld::EnqueueTask( b3TaskCallback* TaskCallback, int ItemCount, int MinRange, void* TaskContext, void* UserContext )
	{
	static VsBox3dWorld* World = static_cast<VsBox3dWorld*>( UserContext );
	return World->EnqueueTask( TaskCallback, ItemCount, MinRange, TaskContext );
	}

//--------------------------------------------------------------------------------------------------
void* VsBox3dWorld::EnqueueTask( b3TaskCallback* TaskCallback, int ItemCount, int MinRange, void* TaskContext )
	{
	if ( mTaskCount < MaxTasks )
		{
		VsBox3dTask* Task = mTaskList + mTaskCount++;
		Task->m_SetSize = ItemCount;
		Task->m_MinRange = MinRange;
		Task->TaskCallback = TaskCallback;
		Task->TaskContext = TaskContext;

		enki::TaskScheduler& TaskScheduler = mPlugin->GetTaskScheduler();
		TaskScheduler.AddTaskSetToPipe( Task );

		return Task;
		}

	// This is not fatal but the MaxTasks should be increased
	B3_ASSERT( false );
	TaskCallback( 0, ItemCount, 0, TaskContext );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::FinishTask( void* Task, void* UserContext )
	{
	static VsBox3dWorld* World = static_cast<VsBox3dWorld*>( UserContext );
	return World->FinishTask( Task );
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::FinishTask( void* Task )
	{
	if ( Task )
		{
		enki::TaskScheduler& TaskScheduler = mPlugin->GetTaskScheduler();
		TaskScheduler.WaitforTask( static_cast<VsBox3dTask*>( Task ) );
		}
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dPlugin
//--------------------------------------------------------------------------------------------------
VsBox3dPlugin::VsBox3dPlugin()
	{
	int WorkerCount = b3MinInt( 8, (int)enki::GetNumHardwareThreads() / 2 );
	mTaskScheduler.Initialize( WorkerCount );
	}


//--------------------------------------------------------------------------------------------------
VsBox3dPlugin::~VsBox3dPlugin()
	{
	while ( !mWorlds.empty() )
		{
		VsBox3dWorld* World = mWorlds.back();
		mWorlds.pop_back();

		delete World;
		}

	while ( !mMeshes.empty() )
		{
		VsBox3dMesh* Mesh = mMeshes.back();
		mMeshes.pop_back();

		delete Mesh;
		}

	while ( !mHulls.empty() )
		{
		VsBox3dHull* Hull = mHulls.back();
		mHulls.pop_back();

		delete Hull;
		}
	}


//--------------------------------------------------------------------------------------------------
const char* VsBox3dPlugin::GetName() const
	{
	return "Box3d";
	}


//--------------------------------------------------------------------------------------------------
const char* VsBox3dPlugin::GetVersion() const
	{
	return "0.1.1";
	}


//--------------------------------------------------------------------------------------------------
enki::TaskScheduler& VsBox3dPlugin::GetTaskScheduler()
	{
	return mTaskScheduler;
	}


//--------------------------------------------------------------------------------------------------
const enki::TaskScheduler& VsBox3dPlugin::GetTaskScheduler() const
	{
	return mTaskScheduler;
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsBox3dPlugin::CreateHull( int VertexCount, const VsVector3* Vertices )
	{
	if ( b3Hull* Hull = b3CreateHull( (const b3Vec3*)Vertices, VertexCount ) )
		{
		mHulls.push_back( new VsBox3dHull( Hull ) );
		return mHulls.back();
		}

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dPlugin::DestroyHull( IVsHull* Hull )
	{
	if ( !Hull )
		{
		return;
		}

	std::erase( mHulls, Hull );
	delete static_cast< VsBox3dHull* >( Hull );
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dPlugin::GetHullCount() const
	{
	return static_cast< int >( mHulls.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsBox3dPlugin::GetHull( int HullIndex )
	{
	return ( 0 <= HullIndex && HullIndex < GetHullCount() ) ? mHulls[ HullIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsHull* VsBox3dPlugin::GetHull( int HullIndex ) const
	{
	return ( 0 <= HullIndex && HullIndex < GetHullCount() ) ? mHulls[ HullIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsBox3dPlugin::CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices )
	{
	b3MeshDef MeshDef = {};
	MeshDef.triangleCount = TriangleCount;
	MeshDef.indices = (int*)TriangleIndices;
	MeshDef.vertexCount = VertexCount;
	MeshDef.vertices = (b3Vec3*)Vertices;
	
	if ( b3MeshData* Mesh = b3CreateMesh( &MeshDef, NULL, 0 ) )
		{
		mMeshes.push_back( new VsBox3dMesh( Mesh ) );
		return mMeshes.back();
		}

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dPlugin::DestroyMesh( IVsMesh* Mesh )
	{
	if ( !Mesh )
		{
		return;
		}

	std::erase( mMeshes, Mesh );
	delete static_cast< VsBox3dMesh* >( Mesh );
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dPlugin::GetMeshCount() const
	{
	return static_cast< int >( mMeshes.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsBox3dPlugin::GetMesh( int MeshIndex )
	{
	return ( 0 <= MeshIndex && MeshIndex < GetMeshCount() ) ? mMeshes[ MeshIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsMesh* VsBox3dPlugin::GetMesh( int MeshIndex ) const
	{
	return ( 0 <= MeshIndex && MeshIndex < GetMeshCount() ) ? mMeshes[ MeshIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsBox3dPlugin::CreateWorld()
	{
	VsBox3dWorld* World = new VsBox3dWorld( this );
	mWorlds.push_back( World );

	return World;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dPlugin::DestroyWorld( IVsWorld* World )
	{
	if ( !World )
		{
		return;
		}

	std::erase( mWorlds, World );
	delete static_cast< VsBox3dWorld* >( World );
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dPlugin::GetWorldCount() const
	{
	return static_cast< int >( mWorlds.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsBox3dPlugin::GetWorld( int WorldIndex )
	{
	return ( 0 <= WorldIndex && WorldIndex < GetWorldCount() ) ? mWorlds[ WorldIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsWorld* VsBox3dPlugin::GetWorld( int WorldIndex ) const
	{
	return ( 0 <= WorldIndex && WorldIndex < GetWorldCount() ) ? mWorlds[ WorldIndex ] : nullptr;
	}


// Export
VS_EXPORT_PLUGIN( VsBox3dPlugin );