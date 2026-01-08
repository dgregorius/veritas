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
VsGeometry::VsGeometry( const std::vector< VsEdgeVertex >& Vertices )
	{
	// Type
	mType = GL_LINES;

	// Vertices
	VS_ASSERT( !Vertices.empty() );
	mVertexCount = static_cast< int >( Vertices.size() );
	glCreateBuffers( 1, &mVertexBuffer );
	glNamedBufferStorage( mVertexBuffer, mVertexCount * sizeof( VsEdgeVertex ), Vertices.data(), GL_DYNAMIC_STORAGE_BIT );
	}


//--------------------------------------------------------------------------------------------------
VsGeometry::VsGeometry( const std::vector< VsMeshVertex >& Vertices )
	{
	// Type
	mType = GL_TRIANGLES;

	// Vertices
	VS_ASSERT( !Vertices.empty() );
	mVertexCount = static_cast< int >( Vertices.size() );
	glCreateBuffers( 1, &mVertexBuffer );
	glNamedBufferStorage( mVertexBuffer, mVertexCount * sizeof( VsMeshVertex ), Vertices.data(), GL_DYNAMIC_STORAGE_BIT );
	}


//--------------------------------------------------------------------------------------------------
VsGeometry::VsGeometry( const std::vector< int >& Indices, const std::vector< VsMeshVertex >& Vertices )
	{
	// Type
	mType = GL_TRIANGLES;

	// Elements
	VS_ASSERT( !Indices.empty() );
	mElementCount = static_cast< int >( Indices.size() );
	glCreateBuffers( 1, &mElementBuffer );
	glNamedBufferStorage( mElementBuffer, mElementCount * sizeof( int ), Indices.data(), GL_DYNAMIC_STORAGE_BIT );

	// Vertices
	VS_ASSERT( !Vertices.empty() );
	mVertexCount = static_cast< int >( Vertices.size() );
	glCreateBuffers( 1, &mVertexBuffer );
	glNamedBufferStorage( mVertexBuffer, mVertexCount * sizeof( VsMeshVertex ), Vertices.data(), GL_DYNAMIC_STORAGE_BIT );
	}


//--------------------------------------------------------------------------------------------------
VsGeometry::~VsGeometry()
	{
	glDeleteBuffers( 1, &mVertexBuffer );
	glDeleteBuffers( 1, &mElementBuffer );
	}


//--------------------------------------------------------------------------------------------------
void VsGeometry::Render( int InstanceCount )
	{
	if ( InstanceCount > 0 )
		{
		glBindVertexBuffer( 0, mVertexBuffer, 0, sizeof( VsMeshVertex ) );

		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, mElementBuffer );
		if ( mElementCount && mElementBuffer )
			{
			glDrawElementsInstanced( mType, mElementCount, GL_UNSIGNED_INT, NULL, InstanceCount );
			}
		else
			{
			glDrawArraysInstanced( mType, 0, mVertexCount, InstanceCount );
			}
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

	return new VsGeometry( Vertices );
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



//--------------------------------------------------------------------------------------------------
VsGeometry* vsCreateEdgeGeometry( IVsShape* Shape )
	{
	VS_ASSERT( Shape );
	VS_ASSERT( Shape->GetType() == VS_HULL_SHAPE );
	IVsHullShape* HullShape = static_cast< IVsHullShape* >( Shape );

	const IVsHull* Hull = HullShape->GetHull();
	VS_ASSERT( Hull );

	int EdgeCount = Hull->GetEdgeCount();
	const VsVector3* EdgePositions = Hull->GetEdgePositions();

	std::vector< VsEdgeVertex > Vertices( EdgeCount );
	for ( int EdgeIndex = 0; EdgeIndex < EdgeCount; ++EdgeIndex )
		{
		VsVector3 P = EdgePositions[ EdgeIndex ];
	
		Vertices[ EdgeIndex ] = { { P.X, P.Y, P.Z }, 0xFF000000 };
		}

	return new VsGeometry( Vertices );
	}

