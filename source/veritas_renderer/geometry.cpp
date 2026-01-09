//--------------------------------------------------------------------------------------------------
// geometry.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "geometry.h"

// OpenGL
#include <glad.h>
#include <glfw3.h>


//--------------------------------------------------------------------------------------------------
// VsGeometry
//--------------------------------------------------------------------------------------------------
VsGeometry::VsGeometry( const std::vector< VsMeshVertex >& Vertices )
	{
	// Vertices
	VS_ASSERT( !Vertices.empty() );
	mVertexCount = static_cast< int >( Vertices.size() );
	glCreateBuffers( 1, &mVertexBuffer );
	glNamedBufferStorage( mVertexBuffer, Vertices.size() * sizeof( VsMeshVertex ), Vertices.data(), GL_DYNAMIC_STORAGE_BIT );
	}


//--------------------------------------------------------------------------------------------------
VsGeometry::VsGeometry( const std::vector< VsMeshVertex >& Vertices, const std::vector< VsEdgeVertex >& Edges )
	{
	// Vertices
	VS_ASSERT( !Vertices.empty() );
	mVertexCount = static_cast< int >( Vertices.size() );
	glCreateBuffers( 1, &mVertexBuffer );
	glNamedBufferStorage( mVertexBuffer, Vertices.size() * sizeof( VsMeshVertex ), Vertices.data(), GL_DYNAMIC_STORAGE_BIT );

	// Edges
	VS_ASSERT( !Edges.empty() );
	mEdgeCount = static_cast< int >( Edges.size() / 2 );
	glCreateBuffers( 1, &mEdgeBuffer );
	glNamedBufferStorage( mEdgeBuffer, Edges.size() * sizeof( VsEdgeVertex ), Edges.data(), GL_DYNAMIC_STORAGE_BIT );
	}


//--------------------------------------------------------------------------------------------------
VsGeometry::VsGeometry( const std::vector< int >& Indices, const std::vector< VsMeshVertex >& Vertices )
	{
	// Elements
	VS_ASSERT( !Indices.empty() );
	mElementCount = static_cast< int >( Indices.size() );
	glCreateBuffers( 1, &mElementBuffer );
	glNamedBufferStorage( mElementBuffer, Indices.size() * sizeof( int ), Indices.data(), GL_DYNAMIC_STORAGE_BIT );

	// Vertices
	VS_ASSERT( !Vertices.empty() );
	mVertexCount = static_cast< int >( Vertices.size() );
	glCreateBuffers( 1, &mVertexBuffer );
	glNamedBufferStorage( mVertexBuffer, Vertices.size() * sizeof( VsMeshVertex ), Vertices.data(), GL_DYNAMIC_STORAGE_BIT );
	}


//--------------------------------------------------------------------------------------------------
VsGeometry::VsGeometry( const std::vector< int >& Indices, const std::vector< VsMeshVertex >& Vertices, const std::vector< VsEdgeVertex >& Edges )
	{
	// Elements
	VS_ASSERT( !Indices.empty() );
	mElementCount = static_cast<int>( Indices.size() );
	glCreateBuffers( 1, &mElementBuffer );
	glNamedBufferStorage( mElementBuffer, Indices.size() * sizeof( int ), Indices.data(), GL_DYNAMIC_STORAGE_BIT );

	// Vertices
	VS_ASSERT( !Vertices.empty() );
	mVertexCount = static_cast<int>( Vertices.size() );
	glCreateBuffers( 1, &mVertexBuffer );
	glNamedBufferStorage( mVertexBuffer, Vertices.size() * sizeof( VsMeshVertex ), Vertices.data(), GL_DYNAMIC_STORAGE_BIT );

	// Edges
	VS_ASSERT( !Edges.empty() );
	mEdgeCount = static_cast<int>( Edges.size() / 2 );
	glCreateBuffers( 1, &mEdgeBuffer );
	glNamedBufferStorage( mEdgeBuffer, Edges.size() * sizeof( VsEdgeVertex ), Edges.data(), GL_DYNAMIC_STORAGE_BIT );
	}


//--------------------------------------------------------------------------------------------------
VsGeometry::~VsGeometry()
	{
	glDeleteBuffers( 1, &mEdgeBuffer );
	glDeleteBuffers( 1, &mVertexBuffer );
	glDeleteBuffers( 1, &mElementBuffer );
	}


//--------------------------------------------------------------------------------------------------
void VsGeometry::RenderFaces( int InstanceCount )
	{
	if ( InstanceCount > 0 )
		{
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, mElementBuffer );
		glBindVertexBuffer( 0, mVertexBuffer, 0, sizeof( VsMeshVertex ) );
		if ( mElementCount && mElementBuffer )
			{
			glDrawElementsInstanced( GL_TRIANGLES, mElementCount, GL_UNSIGNED_INT, NULL, InstanceCount );
			}
		else
			{
			glDrawArraysInstanced( GL_TRIANGLES, 0, mVertexCount, InstanceCount );
			}
		}
	}


//--------------------------------------------------------------------------------------------------
void VsGeometry::RenderEdges( int InstanceCount )
	{
	if ( InstanceCount > 0 )
		{
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, GL_NONE );
		glBindVertexBuffer( 0, mEdgeBuffer, 0, sizeof( VsEdgeVertex ) );

		glDrawArraysInstanced( GL_LINES, 0, 2 * mEdgeCount, InstanceCount );
		}
	}


//--------------------------------------------------------------------------------------------------
static VsGeometry* vsOnCreateSphere( IVsShape* Shape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
static VsGeometry* vsOnCreateCapsule( IVsShape* Shape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
static VsGeometry* vsOnCreateHull( IVsShape* Shape )
	{
	VS_ASSERT( Shape );
	VS_ASSERT( Shape->GetType() == VS_HULL_SHAPE );
	IVsHullShape* HullShape = static_cast< IVsHullShape* >( Shape );

	const IVsHull* Hull = HullShape->GetHull();
	VS_ASSERT( Hull );

	// Faces
	int VertexCount = Hull->GetVertexCount();
	const VsVector3* VertexPositions = Hull->GetVertexPositions();
	const VsVector3* VertexNormals = Hull->GetVertexNormals();

	std::vector< VsMeshVertex > Vertices( VertexCount );
	for ( int VertexIndex = 0; VertexIndex < VertexCount; ++VertexIndex )
		{
		VsVector3 P = VertexPositions[ VertexIndex ];
		VsVector3 N = VertexNormals[ VertexIndex ];

		Vertices[ VertexIndex ] = { { P.X, P.Y, P.Z }, { N.X, N.Y, N.Z } };
		}

	// Edges
	int EdgeCount = Hull->GetEdgeCount();
	const VsVector3* EdgePositions = Hull->GetEdgePositions();

	std::vector< VsEdgeVertex > Edges( 2 * EdgeCount );
	for ( int EdgeIndex = 0; EdgeIndex < EdgeCount; ++EdgeIndex )
		{
		int VertexIndex1 = 2 * EdgeIndex + 0;
		VsVector3 P1 = EdgePositions[ VertexIndex1 ];
		int VertexIndex2 = 2 * EdgeIndex + 1;
		VsVector3 P2 = EdgePositions[ VertexIndex2 ];

		Edges[ VertexIndex1 ] = { { P1.X, P1.Y, P1.Z }, 0xFF000000 };
		Edges[ VertexIndex2 ] = { { P2.X, P2.Y, P2.Z }, 0xFF000000 };
		}

	return new VsGeometry( Vertices, Edges );
	}


//--------------------------------------------------------------------------------------------------
static VsGeometry* vsOnCreateMesh( IVsShape* Shape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
VsGeometry* vsCreateMeshGeometry( IVsShape* Shape )
	{
	typedef VsGeometry* ( *VsGeometryCreator )( IVsShape* );
	static const VsGeometryCreator Creator[] =
		{
		vsOnCreateSphere,
		vsOnCreateCapsule,
		vsOnCreateHull,
		vsOnCreateMesh
		};

	VsShapeType Type = Shape->GetType();
	return Creator[ Type ]( Shape );
	}

