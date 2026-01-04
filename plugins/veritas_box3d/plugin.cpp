//--------------------------------------------------------------------------------------------------
// plugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "plugin.h"


//--------------------------------------------------------------------------------------------------
// VsBox3dHull
//--------------------------------------------------------------------------------------------------
VsBox3dHull::VsBox3dHull( b3Hull* Hull )
	{
	B3_ASSERT( Hull );
	Native = Hull;

	int VertexCount = Hull->vertexCount;
	const b3Vec3* HullVertices = b3GetHullPoints( Hull );
	if ( VertexCount > 0 && HullVertices )
		{
		Vertices.resize( VertexCount );
		for ( int VertexIndex = 0; VertexIndex < VertexCount; ++VertexIndex )
			{
			b3Vec3 Vertex = HullVertices[ VertexIndex ];
			Vertices[ VertexIndex ] = { Vertex.x, Vertex.y, Vertex.z };
			}

		int EdgeCount = Hull->edgeCount;
		const b3HullHalfEdge* HullEdges = b3GetHullEdges( Hull );
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

				Edges[ 2 * EdgeIndex + 0 ] = { Vertex1.x, Vertex1.y, Vertex1.z };
				Edges[ 2 * EdgeIndex + 1 ] = { Vertex2.x, Vertex2.y, Vertex2.z };
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
	return static_cast< int >( Vertices.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dHull::GetVertices() const
	{
	return Vertices.data();
	}


//--------------------------------------------------------------------------------------------------
int VsBox3dHull::GetEdgeCount() const
	{
	return static_cast< int >( Edges.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dHull::GetEdges() const
	{
	return Edges.data();
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dHullShape
//--------------------------------------------------------------------------------------------------
VsBox3dHullShape::VsBox3dHullShape( b3ShapeId ShapeId, const VsBox3dHull* Hull )
	: ShapeId( ShapeId )
	, Hull( Hull )
	{

	}


//--------------------------------------------------------------------------------------------------
VsBox3dHullShape::~VsBox3dHullShape()
	{
	b3DestroyShape( ShapeId, true );
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
		Vertices.resize( 3 * TriangleCount );
		for ( int TriangleIndex = 0; TriangleIndex < TriangleCount; ++TriangleIndex )
			{
			int VertexIndex1 = MeshTriangles[ TriangleIndex ].index1;
			b3Vec3 Vertex1 = MeshVertices[ VertexIndex1 ];
			int VertexIndex2 = MeshTriangles[ TriangleIndex ].index2;
			b3Vec3 Vertex2 = MeshVertices[ VertexIndex2 ];
			int VertexIndex3 = MeshTriangles[ TriangleIndex ].index3;
			b3Vec3 Vertex3 = MeshVertices[ VertexIndex3 ];

			Vertices[ 3 * TriangleIndex + 0 ] = { Vertex1.x, Vertex1.y, Vertex1.z };
			Vertices[ 3 * TriangleIndex + 1 ] = { Vertex2.x, Vertex2.y, Vertex2.z };
			Vertices[ 3 * TriangleIndex + 2 ] = { Vertex3.x, Vertex3.y, Vertex3.z };
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
	return static_cast< int >( Vertices.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsBox3dMesh::GetVertices() const
	{
	return Vertices.data();
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dMeshShape
//--------------------------------------------------------------------------------------------------
VsBox3dMeshShape::VsBox3dMeshShape( b3ShapeId ShapeId, const VsBox3dMesh* Mesh )
	: ShapeId( ShapeId )
	, Mesh( Mesh )
	{

	}


//--------------------------------------------------------------------------------------------------
VsBox3dMeshShape::~VsBox3dMeshShape()
	{
	b3DestroyShape( ShapeId, false );
	}


//--------------------------------------------------------------------------------------------------
const IVsMesh* VsBox3dMeshShape::GetMesh() const
	{
	return Mesh;
	}



//--------------------------------------------------------------------------------------------------
// VsBox3dBody
//--------------------------------------------------------------------------------------------------
VsBox3dBody::VsBox3dBody( b3BodyId BodyId )
	: BodyId( BodyId )
	{

	}


//--------------------------------------------------------------------------------------------------
VsBox3dBody::~VsBox3dBody()
	{
	b3DestroyBody( BodyId );
	}


//--------------------------------------------------------------------------------------------------
VsBodyType VsBox3dBody::GetType() const
	{
	b3BodyType Type = b3Body_GetType( BodyId );

	VsBodyType TypeMap[] = { VS_STATIC_BODY, VS_KEYFRAMED_BODY, VS_STATIC_BODY };
	return TypeMap[ Type ];
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsBox3dBody::GetPosition() const
	{
	b3Vec3 Position = b3Body_GetPosition( BodyId );
	return VsVector3{ Position.x, Position.y, Position.z };
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dBody::SetPosition( const VsVector3& Position )
	{
	b3Quat Rotation = b3Body_GetRotation( BodyId );
	b3Body_SetTransform( BodyId, b3Vec3{ Position.X, Position.Y, Position.Z }, Rotation );
	}


//--------------------------------------------------------------------------------------------------
VsQuaternion VsBox3dBody::GetOrientation() const
	{
	b3Quat Rotation = b3Body_GetRotation( BodyId );
	return VsQuaternion{ Rotation.v.x, Rotation.v.y, Rotation.v.z, Rotation.s };
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dBody::SetOrientation( const VsQuaternion& Orientation )
	{
	b3Vec3 Position = b3Body_GetPosition( BodyId );
	b3Body_SetTransform( BodyId, Position, b3Quat( { Orientation.X, Orientation.Y, Orientation.Z }, Orientation.W ) );
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

	b3ShapeDef ShapeDef = b3DefaultShapeDef();
	
	b3ShapeId ShapeId = b3CreateHullShape( BodyId, &ShapeDef, static_cast< const VsBox3dHull* >( Hull )->Native );
	return B3_IS_NON_NULL( ShapeId ) ? new VsBox3dHullShape( ShapeId, static_cast< const VsBox3dHull* >( Hull ) ) : nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsMeshShape* VsBox3dBody::CreateMesh( const IVsMesh* Mesh )
	{
	if ( !Mesh )
		{
		return nullptr;
		}

	b3ShapeDef ShapeDef = b3DefaultShapeDef();

	b3ShapeId ShapeId = b3CreateMeshShape( BodyId, &ShapeDef, static_cast< const VsBox3dMesh* >( Mesh )->Native, { 1.0f, 1.0f, 1.0f } );
	return B3_IS_NON_NULL( ShapeId ) ? new VsBox3dMeshShape( ShapeId, static_cast< const VsBox3dMesh* >( Mesh ) ) : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dBody::DestroyShape( IVsShape* Shape )
	{
	
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dWorld
//--------------------------------------------------------------------------------------------------
VsBox3dWorld::VsBox3dWorld( b3WorldId WorldId )
	: WorldId( WorldId )
	{
	
	}


//--------------------------------------------------------------------------------------------------
VsBox3dWorld::~VsBox3dWorld()
	{
	b3DestroyWorld( WorldId );
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsBox3dWorld::GetGravity() const
	{
	b3Vec3 Gravity = b3World_GetGravity( WorldId );
	return VsVector3{ Gravity.x, Gravity.y, Gravity.z };
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::SetGravity( const VsVector3& Gravity )
	{
	b3World_SetGravity( WorldId, { Gravity.X, Gravity.Y, Gravity.Z } );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsBox3dWorld::CreateBody( VsBodyType Type )
	{
	const b3BodyType TypeMap[] = { b3_staticBody, b3_kinematicBody, b3_staticBody };
	
	b3BodyDef BodyDef = b3DefaultBodyDef();
	BodyDef.type = TypeMap[ Type ];

	b3BodyId BodyId = b3CreateBody( WorldId, &BodyDef );
	return B3_IS_NON_NULL( BodyId ) ? new VsBox3dBody( BodyId ) : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dWorld::DestroyBody( IVsBody* Body )
	{
	delete static_cast< VsBox3dBody* >( Body );
	}


//--------------------------------------------------------------------------------------------------
// VsBox3dPlugin
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
	b3Hull* Hull = b3CreateHull( (const b3Vec3*)Vertices, VertexCount );
	return Hull ? new VsBox3dHull( Hull ) : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dPlugin::DestroyHull( IVsHull* Hull )
	{
	delete static_cast< VsBox3dHull* >( Hull );
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsBox3dPlugin::CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices )
	{
	b3MeshDef MeshDef = {};
	MeshDef.triangleCount = TriangleCount;
	MeshDef.indices = (int*)TriangleIndices;
	MeshDef.vertexCount = VertexCount;
	MeshDef.vertices = (b3Vec3*)Vertices;
	
	b3MeshData* Mesh = b3CreateMesh( &MeshDef, NULL, 0 );
	return Mesh ? new VsBox3dMesh( Mesh ) : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dPlugin::DestroyMesh( IVsMesh* Mesh )
	{
	delete static_cast< VsBox3dMesh* >( Mesh );
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsBox3dPlugin::CreateWorld()
	{
	b3WorldDef WorldDef = b3DefaultWorldDef();
	
	b3WorldId WorldId = b3CreateWorld( &WorldDef );
	return B3_IS_NON_NULL( WorldId ) ? new VsBox3dWorld( WorldId ) : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3dPlugin::DestroyWorld( IVsWorld* World )
	{
	delete static_cast< VsBox3dWorld* >( World );
	}

// Export
VS_EXPORT_PLUGIN( VsBox3dPlugin );