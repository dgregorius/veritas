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

static void* EnqueueTask( b3TaskCallback* TaskCallback, int ItemCount, int MinRange, void* TaskContext, void* UserContext )
	{
	VsBox3dWorld* World = static_cast< VsBox3dWorld* >( UserContext );
	if ( World->TaskCount < VsBox3dWorld::MaxTasks )
		{
		VsBox3dTask* Task = World->TaskList + World->TaskCount++;
		Task->m_SetSize = ItemCount;
		Task->m_MinRange = MinRange;
		Task->TaskCallback = TaskCallback;
		Task->TaskContext = TaskContext;

		World->TaskScheduler.AddTaskSetToPipe( Task );
		return Task;
		}

	// This is not fatal but the MaxTasks should be increased
	B3_ASSERT( false );
	TaskCallback( 0, ItemCount, 0, TaskContext );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void FinishTask( void* Task, void* UserContext )
	{
	VsBox3dWorld* World = static_cast<VsBox3dWorld*>( UserContext );
	World->TaskScheduler.WaitforTask( static_cast<VsBox3dTask*>( Task ) );
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dHull
//--------------------------------------------------------------------------------------------------
VsBox3dHull::VsBox3dHull( b3Hull* Hull )
	{
	B3_ASSERT( Hull );
	Native = Hull;

	int FaceCount = Hull->faceCount;
	if ( FaceCount > 0  )
		{
		VertexPositions.reserve( 3 * FaceCount );
		VertexNormals.reserve( 3 * FaceCount );

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
				VertexPositions.push_back( { Vertex1.x, Vertex1.y, Vertex1.z } );
				VertexNormals.push_back( { FaceNormal.x, FaceNormal.y, FaceNormal.z } );
				
				int VertexIndex2 = Edge2->origin;
				b3Vec3 Vertex2 = HullVertices[ VertexIndex2 ];
				VertexPositions.push_back( { Vertex2.x, Vertex2.y, Vertex2.z } );
				VertexNormals.push_back( { FaceNormal.x, FaceNormal.y, FaceNormal.z } );
				
				int VertexIndex3 = Edge3->origin;
				b3Vec3 Vertex3 = HullVertices[ VertexIndex3 ];
				VertexPositions.push_back( { Vertex3.x, Vertex3.y, Vertex3.z } );
				VertexNormals.push_back( { FaceNormal.x, FaceNormal.y, FaceNormal.z } );

				Edge2 = Edge3;
				Edge3 = HullEdges + Edge3->next;
				}
			while ( Edge1 != Edge3 );
			}

		int EdgeCount = Hull->edgeCount;
		
		if ( EdgeCount > 0 && HullEdges )
			{
			Edges.resize( EdgeCount );
			for ( int EdgeIndex = 0; EdgeIndex < EdgeCount; EdgeIndex += 2 )
				{
				const b3HullHalfEdge* Edge = HullEdges + EdgeIndex;
				const b3HullHalfEdge* Twin = HullEdges + Edge->twin;

				int VertexIndex1 = Edge->origin;
				b3Vec3 Vertex1 = HullVertices[ VertexIndex1 ];
				int VertexIndex2 = Twin->origin;
				b3Vec3 Vertex2 = HullVertices[ VertexIndex2 ];

				Edges[ EdgeIndex + 0 ] = { Vertex1.x, Vertex1.y, Vertex1.z };
				Edges[ EdgeIndex + 1 ] = { Vertex2.x, Vertex2.y, Vertex2.z };
				}
			}
		}
	}


//--------------------------------------------------------------------------------------------------
VsBox3dHull::~VsBox3dHull()
	{
	b3DestroyHull( Native );
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dHull::GetVertexCount() const
	{
	return static_cast< int >( VertexPositions.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dHull::GetVertexPositions() const
	{
	return VertexPositions.data();
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dHull::GetVertexNormals() const
	{
	return VertexNormals.data();
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dHull::GetEdgeCount() const
	{
	return static_cast< int >( Edges.size() / 2 );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dHull::GetEdges() const
	{
	return Edges.data();
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dHullShape
//--------------------------------------------------------------------------------------------------
VsBox3dHullShape::VsBox3dHullShape( VsBox3dBody* Body, const VsBox3dHull* Hull )
	: Hull( Hull )
	{
	b3ShapeDef ShapeDef = b3DefaultShapeDef();
	Native = b3CreateHullShape( Body->Native, &ShapeDef, Hull->Native );
	}


//--------------------------------------------------------------------------------------------------
VsBox3dHullShape::~VsBox3dHullShape()
	{
	b3DestroyShape( Native, true );
	}


//--------------------------------------------------------------------------------------------------
const IVsHull* VsBox3dHullShape::GetHull() const
	{
	return Hull;
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dMesh
//--------------------------------------------------------------------------------------------------
VsBox3dMesh::VsBox3dMesh( b3MeshData* Mesh )
	{
	B3_ASSERT( Mesh );
	Native = Mesh;

	int TriangleCount = 3 * Mesh->triangleCount;
	const b3MeshTriangle* MeshTriangles = b3GetMeshTriangles( Mesh );
	const b3Vec3* MeshVertices = b3GetMeshVertices( Mesh );
	if ( TriangleCount > 0 && MeshTriangles && MeshVertices )
		{
		VertexPositions.resize( 3 * TriangleCount );
		VertexNormals.resize( 3 * TriangleCount );
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

			VertexPositions[ 3 * TriangleIndex + 0 ] = { Vertex1.x, Vertex1.y, Vertex1.z };
			VertexNormals[ 3 * TriangleIndex + 0 ] = { Normal.x, Normal.y, Normal.z };
			VertexPositions[ 3 * TriangleIndex + 1 ] = { Vertex2.x, Vertex2.y, Vertex2.z };
			VertexNormals[ 3 * TriangleIndex + 1 ] = { Normal.x, Normal.y, Normal.z };
			VertexPositions[ 3 * TriangleIndex + 2 ] = { Vertex3.x, Vertex3.y, Vertex3.z };
			VertexNormals[ 3 * TriangleIndex + 2 ] = { Normal.x, Normal.y, Normal.z };
			}
		}
	}


//--------------------------------------------------------------------------------------------------
VsBox3dMesh::~VsBox3dMesh()
	{
	b3DestroyMesh( Native );
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dMesh::GetVertexCount() const
	{
	return static_cast< int >( VertexPositions.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dMesh::GetVertexPositions() const
	{
	return VertexPositions.data();
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dMesh::GetVertexNormals() const
	{
	return VertexNormals.data();
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dMeshShape
//--------------------------------------------------------------------------------------------------
VsBox3dMeshShape::VsBox3dMeshShape( VsBox3dBody* Body, const VsBox3dMesh* Mesh )
	: Mesh( Mesh )
	{
	b3ShapeDef ShapeDef = b3DefaultShapeDef();
	Native = b3CreateMeshShape( Body->Native, &ShapeDef, Mesh->Native, { 1.0f, 1.0f, 1.0f } );
	}


//--------------------------------------------------------------------------------------------------
VsBox3dMeshShape::~VsBox3dMeshShape()
	{
	b3DestroyShape( Native, false );
	}


//--------------------------------------------------------------------------------------------------
const IVsMesh* VsBox3dMeshShape::GetMesh() const
	{
	return Mesh;
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dBody
//--------------------------------------------------------------------------------------------------
VsBox3dBody::VsBox3dBody( VsBox3dWorld* World, VsBodyType Type )
	{
	const b3BodyType TypeMap[] = { b3_staticBody, b3_kinematicBody, b3_staticBody };

	b3BodyDef BodyDef = b3DefaultBodyDef();
	BodyDef.type = TypeMap[ Type ];

	Native = b3CreateBody( World->Native, &BodyDef );
	}


//--------------------------------------------------------------------------------------------------
VsBox3dBody::~VsBox3dBody()
	{
	b3DestroyBody( Native );
	}


//--------------------------------------------------------------------------------------------------
VsBodyType VsBox3dBody::GetType() const
	{
	b3BodyType Type = b3Body_GetType( Native );

	VsBodyType TypeMap[] = { VS_STATIC_BODY, VS_KEYFRAMED_BODY, VS_STATIC_BODY };
	return TypeMap[ Type ];
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsBox3dBody::GetPosition() const
	{
	b3Vec3 Position = b3Body_GetPosition( Native );
	return VsVector3{ Position.x, Position.y, Position.z };
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dBody::SetPosition( const VsVector3& Position )
	{
	b3Quat Rotation = b3Body_GetRotation( Native );
	b3Body_SetTransform( Native, b3Vec3{ Position.X, Position.Y, Position.Z }, Rotation );
	}


//--------------------------------------------------------------------------------------------------
VsQuaternion VsBox3dBody::GetOrientation() const
	{
	b3Quat Rotation = b3Body_GetRotation( Native );
	return VsQuaternion{ Rotation.v.x, Rotation.v.y, Rotation.v.z, Rotation.s };
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dBody::SetOrientation( const VsQuaternion& Orientation )
	{
	b3Vec3 Position = b3Body_GetPosition( Native );
	b3Body_SetTransform( Native, Position, b3Quat( { Orientation.X, Orientation.Y, Orientation.Z }, Orientation.W ) );
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
	Shapes.push_back( Shape );

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
	Shapes.push_back( Shape );

	return Shape;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dBody::DestroyShape( IVsShape* Shape )
	{
	if ( !Shape )
		{
		return;
		}

	size_t Count = std::erase( Shapes, Shape );
	B3_ASSERT( Count == 1 );
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dWorld
//--------------------------------------------------------------------------------------------------
VsBox3dWorld::VsBox3dWorld( enki::TaskScheduler& TaskScheduler )
	: TaskScheduler( TaskScheduler )
	{
	b3WorldDef WorldDef = b3DefaultWorldDef();
	WorldDef.gravity = { 0.0f, -10.0f, 0.0f };
	WorldDef.workerCount = TaskScheduler.GetNumTaskThreads();
	WorldDef.enqueueTask = EnqueueTask;
	WorldDef.finishTask = FinishTask;

	Native = b3CreateWorld( &WorldDef );
	}


//--------------------------------------------------------------------------------------------------
VsBox3dWorld::~VsBox3dWorld()
	{
	while ( !Bodies.empty() )
		{
		VsBox3dBody* Body = Bodies.back();
		Bodies.pop_back();

		delete Body;
		}

	b3DestroyWorld( Native );
	}



//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::AddListener( IVsWorldListener* Listener )
	{
	// Add 
	if ( !Listener )
		{
		return;
		}

	if ( std::find( Listeners.begin(), Listeners.end(), Listener ) != Listeners.end() )
		{
		return;
		}
	Listeners.push_back( Listener );

	// Sync
	for ( VsBox3dBody* Body : Bodies )
		{
		Listener->OnBodyAdded( Body );
		for ( IVsShape* Shape : Body->Shapes )
			{
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

	std::erase( Listeners, Listener );
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsBox3dWorld::GetGravity() const
	{
	b3Vec3 Gravity = b3World_GetGravity( Native );
	return { Gravity.x, Gravity.y, Gravity.z };
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::SetGravity( const VsVector3& Gravity )
	{
	b3World_SetGravity( Native, { Gravity.X, Gravity.Y, Gravity.Z } );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsBox3dWorld::CreateBody( VsBodyType Type )
	{
	VsBox3dBody* Body = new VsBox3dBody( this, Type );
	Bodies.push_back( Body );

	return Body;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::DestroyBody( IVsBody* Body )
	{
	if ( !Body )
		{
		return;
		}

	std::erase( Bodies, Body );
	delete static_cast< VsBox3dBody* >( Body );
	}



//--------------------------------------------------------------------------------------------------
int VsBox3dWorld::GetBodyCount() const
	{
	return static_cast<int>( Bodies.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsBox3dWorld::GetBody( int BodyIndex )
	{
	return ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? Bodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsBody* VsBox3dWorld::GetBody( int BodyIndex ) const
	{
	return ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? Bodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::Step( float Timestep )
	{
	b3World_Step( Native, Timestep, 4 );
	TaskCount = 0;
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dPlugin
//--------------------------------------------------------------------------------------------------
VsBox3dPlugin::VsBox3dPlugin()
	{
	int WorkerCount = b3MinInt( 8, (int)enki::GetNumHardwareThreads() / 2 );
	TaskScheduler.Initialize( WorkerCount );
	}


//--------------------------------------------------------------------------------------------------
VsBox3dPlugin::~VsBox3dPlugin()
	{
	while ( !Worlds.empty() )
		{
		VsBox3dWorld* World = Worlds.back();
		Worlds.pop_back();

		delete World;
		}

	while ( !Meshes.empty() )
		{
		VsBox3dMesh* Mesh = Meshes.back();
		Meshes.pop_back();

		delete Mesh;
		}

	while ( !Hulls.empty() )
		{
		VsBox3dHull* Hull = Hulls.back();
		Hulls.pop_back();

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
IVsHull* VsBox3dPlugin::CreateHull( int VertexCount, const VsVector3* Vertices )
	{
	if ( b3Hull* Hull = b3CreateHull( (const b3Vec3*)Vertices, VertexCount ) )
		{
		Hulls.push_back( new VsBox3dHull( Hull ) );
		return Hulls.back();
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

	std::erase( Hulls, Hull );
	delete static_cast< VsBox3dHull* >( Hull );
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dPlugin::GetHullCount() const
	{
	return static_cast< int >( Hulls.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsBox3dPlugin::GetHull( int HullIndex )
	{
	return ( 0 <= HullIndex && HullIndex < GetHullCount() ) ? Hulls[ HullIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsHull* VsBox3dPlugin::GetHull( int HullIndex ) const
	{
	return ( 0 <= HullIndex && HullIndex < GetHullCount() ) ? Hulls[ HullIndex ] : nullptr;
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
		Meshes.push_back( new VsBox3dMesh( Mesh ) );
		return Meshes.back();
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

	std::erase( Meshes, Mesh );
	delete static_cast< VsBox3dMesh* >( Mesh );
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dPlugin::GetMeshCount() const
	{
	return static_cast< int >( Meshes.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsBox3dPlugin::GetMesh( int MeshIndex )
	{
	return ( 0 <= MeshIndex && MeshIndex < GetMeshCount() ) ? Meshes[ MeshIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsMesh* VsBox3dPlugin::GetMesh( int MeshIndex ) const
	{
	return ( 0 <= MeshIndex && MeshIndex < GetMeshCount() ) ? Meshes[ MeshIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsBox3dPlugin::CreateWorld()
	{
	VsBox3dWorld* World = new VsBox3dWorld( TaskScheduler );
	Worlds.push_back( World );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dPlugin::DestroyWorld( IVsWorld* World )
	{
	if ( !World )
		{
		return;
		}

	std::erase( Worlds, World );
	delete static_cast< VsBox3dWorld* >( World );
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dPlugin::GetWorldCount() const
	{
	return static_cast< int >( Worlds.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsBox3dPlugin::GetWorld( int WorldIndex )
	{
	return ( 0 <= WorldIndex && WorldIndex < GetWorldCount() ) ? Worlds[ WorldIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsWorld* VsBox3dPlugin::GetWorld( int WorldIndex ) const
	{
	return ( 0 <= WorldIndex && WorldIndex < GetWorldCount() ) ? Worlds[ WorldIndex ] : nullptr;
	}


// Export
VS_EXPORT_PLUGIN( VsBox3dPlugin );