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
void vsLoadVertexLibrary()
	{
	// Initialize vertex formats
	VS_ASSERT( !VsClearVertex::Format );
		{
		uint32_t VAO = 0;
		glCreateVertexArrays( 1, &VAO );
		VS_ASSERT( glGetError() == GL_NO_ERROR );
		VsClearVertex::Format = VAO;
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
void vsUnloadVertexLibrary()
	{
	// Terminate vertex formats
	VS_ASSERT( VsMeshVertex::Format );
	glDeleteVertexArrays( 1, &VsMeshVertex::Format );
	VsMeshVertex::Format = 0;

	VS_ASSERT( VsClearVertex::Format );
	glDeleteVertexArrays( 1, &VsClearVertex::Format );
	VsClearVertex::Format = 0;
	}