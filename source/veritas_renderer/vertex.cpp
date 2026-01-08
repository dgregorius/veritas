//--------------------------------------------------------------------------------------------------
// vertex.cpp
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "vertex.h"

// OpenGL
#include <glad.h>
#include <glfw3.h>


//--------------------------------------------------------------------------------------------------
// Vertex library
//--------------------------------------------------------------------------------------------------
void vsLoadFormats()
	{
	// Create vertex formats
	VS_ASSERT( !VsEmptyVertex::Format );
		{
		uint32_t VAO = 0;
		glCreateVertexArrays( 1, &VAO );
		VS_ASSERT( glGetError() == GL_NO_ERROR );
		VsEmptyVertex::Format = VAO;
		}

	VS_ASSERT( !VsMeshVertex::Format );
		{
		uint32_t VAO = 0;
		glCreateVertexArrays( 1, &VAO );
		VS_ASSERT( VAO );

		glEnableVertexArrayAttrib( VAO, 0 );
		glVertexArrayAttribFormat( VAO, 0, 3, GL_FLOAT, GL_FALSE, offsetof( VsMeshVertex, Position ) );
		glVertexArrayAttribBinding( VAO, 0, 0 );

		glEnableVertexArrayAttrib( VAO, 1 );
		glVertexArrayAttribFormat( VAO, 1, 3, GL_FLOAT, GL_FALSE, offsetof( VsMeshVertex, Normal ) );
		glVertexArrayAttribBinding( VAO, 1, 0 );

		VS_ASSERT( glGetError() == GL_NO_ERROR );
		VsMeshVertex::Format = VAO;
		}
	}


//--------------------------------------------------------------------------------------------------
void vsUnloadFormats()
	{
	// Free vertex formats
	VS_ASSERT( VsMeshVertex::Format );
	glDeleteVertexArrays( 1, &VsMeshVertex::Format );
	VsMeshVertex::Format = 0;

	VS_ASSERT( VsEmptyVertex::Format );
	glDeleteVertexArrays( 1, &VsEmptyVertex::Format );
	VsEmptyVertex::Format = 0;
	}