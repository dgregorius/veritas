//--------------------------------------------------------------------------------------------------
// instancedmesh.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "instancedmesh.h"
#include "geometry.h"

// OpenGL
#include <glad.h>
#include <glfw3.h>


//--------------------------------------------------------------------------------------------------
// VsInstancedMesh
//--------------------------------------------------------------------------------------------------
VsInstancedMesh::VsInstancedMesh( VsGeometry* Geometry )
	{
	VS_ASSERT( Geometry );
	mGeometry = Geometry;

	mInstanceSize = 0;
	mInstanceCapacity = 0;
	glCreateBuffers( 1, &mInstanceBuffer );
	VS_ASSERT( glGetError() == GL_NO_ERROR );
	}


//--------------------------------------------------------------------------------------------------
VsInstancedMesh::~VsInstancedMesh()
	{
	glDeleteBuffers( 1, &mInstanceBuffer );
	}


//--------------------------------------------------------------------------------------------------
VsGeometry* VsInstancedMesh::GetGeometry() const
	{
	return mGeometry;
	}


//--------------------------------------------------------------------------------------------------
void VsInstancedMesh::Upload( const std::vector< VsInstanceData >& InstanceData )
	{
	mInstanceSize = static_cast< int >( InstanceData.size() );
	if ( mInstanceCapacity < mInstanceSize )
		{
		// Reallocate buffer
		mInstanceCapacity = mInstanceSize;
		glInvalidateBufferData( mInstanceBuffer );
		glNamedBufferData( mInstanceBuffer, mInstanceSize * sizeof( VsInstanceData ), InstanceData.data(), GL_DYNAMIC_DRAW );
		}
	else
		{
		// Update buffer
		glNamedBufferSubData( mInstanceBuffer, 0, mInstanceSize * sizeof( VsInstanceData ), InstanceData.data() );
		}
	}


//--------------------------------------------------------------------------------------------------
void VsInstancedMesh::RenderFaces()
	{
	glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 0, mInstanceBuffer );
	mGeometry->RenderFaces( mInstanceSize );
	}


//--------------------------------------------------------------------------------------------------
void VsInstancedMesh::RenderEdges()
	{
	glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 0, mInstanceBuffer );
	mGeometry->RenderEdges( mInstanceSize );
	}
