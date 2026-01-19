//--------------------------------------------------------------------------------------------------
// shader.cpp
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "shader.h"

// OpenGL
#include <glad.h>
#include <glfw3.h>

// STD
#include <fstream>
#include <sstream>


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static uint32_t vsLoadShaderFromSource( uint32_t Type, const char* Source )
	{
	VS_ASSERT( Source && *Source );
	uint32_t Shader = glCreateShader( Type );
	glShaderSource( Shader, 1, &Source, NULL );
	glCompileShader( Shader );

	int Success = 0;
	glGetShaderiv( Shader, GL_COMPILE_STATUS, &Success );
	VS_ASSERT( Success );
	if ( !Success )
		{
		GLchar InfoLog[ 512 ] = { 0 };
		glGetShaderInfoLog( Shader, sizeof( InfoLog ), NULL, InfoLog );
		glDeleteShader( Shader );
		Shader = 0;
		}

	VS_ASSERT( glGetError() == GL_NO_ERROR );
	return Shader;
	}


//--------------------------------------------------------------------------------------------------
static uint32_t vsLoadShaderFromFile( uint32_t Type, const fs::path& Path )
	{
	std::ifstream File( Path, std::ios::in | std::ios::binary );
	if ( !File )
		{
		return 0;
		}

	File.seekg( 0, std::ios::end );
	size_t Length = File.tellg();
	File.seekg( 0, std::ios::beg );
	char* Source = static_cast<char*>( alloca( Length + 1 ) );
	if ( !Source )
		{
		return 0;
		}

	File.read( Source, Length );
	Source[ Length ] = '\0';

	return vsLoadShaderFromSource( Type, Source );
	}


//--------------------------------------------------------------------------------------------------
// VsShader
//--------------------------------------------------------------------------------------------------
VsShader::VsShader( const char* VertexShaderSource, const char* FragmentShaderSource )
	{
	uint32_t VertexShader = vsLoadShaderFromSource( GL_VERTEX_SHADER, VertexShaderSource );
	uint32_t FragmentShader = vsLoadShaderFromSource( GL_FRAGMENT_SHADER, FragmentShaderSource );

	mProgram = glCreateProgram();
	glAttachShader( mProgram, VertexShader );
	glAttachShader( mProgram, FragmentShader );
	glLinkProgram( mProgram );

	int Success = 0;
	glGetProgramiv( mProgram, GL_LINK_STATUS, &Success );
	VS_ASSERT( Success );
	if ( !Success )
		{
		GLchar InfoLog[ 512 ];
		glGetProgramInfoLog( mProgram, 512, NULL, InfoLog );
		glDeleteProgram( mProgram );
		mProgram = 0;
		}

	glDetachShader( mProgram, FragmentShader );
	glDetachShader( mProgram, VertexShader );
	VS_ASSERT( glGetError() == GL_NO_ERROR );
	}


//--------------------------------------------------------------------------------------------------
VsShader::VsShader( const fs::path& VertexShaderPath, const fs::path& FragmentShaderPath )
	{
	uint32_t VertexShader = vsLoadShaderFromFile( GL_VERTEX_SHADER, VertexShaderPath );
	uint32_t FragmentShader = vsLoadShaderFromFile( GL_FRAGMENT_SHADER, FragmentShaderPath );

	mProgram = glCreateProgram();
	glAttachShader( mProgram, VertexShader );
	glAttachShader( mProgram, FragmentShader );
	glLinkProgram( mProgram );

	int Success = 0;
	glGetProgramiv( mProgram, GL_LINK_STATUS, &Success );
	VS_ASSERT( Success );
	if ( !Success )
		{
		GLchar InfoLog[ 512 ];
		glGetProgramInfoLog( mProgram, 512, NULL, InfoLog );
		glDeleteProgram( mProgram );
		mProgram = 0;
		}

	glDetachShader( mProgram, FragmentShader );
	glDetachShader( mProgram, VertexShader );
	VS_ASSERT( glGetError() == GL_NO_ERROR );
	}


//--------------------------------------------------------------------------------------------------
VsShader::~VsShader()
	{
	glDeleteProgram( mProgram );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::Use()
	{
	glUseProgram( mProgram );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniform( const char* Name, bool Value )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform1i( Location, static_cast<int>( Value ) );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniform( const char* Name, int Value )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform1i( Location, Value );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniform( const char* Name, float Value )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform1f( Location, Value );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniform( const char* Name, const glm::vec2& Value )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform2f( Location, Value.x, Value.y );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniform( const char* Name, const glm::vec3& Value )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform3f( Location, Value.x, Value.y, Value.z );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniform( const char* Name, const glm::vec4& Value )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform4f( Location, Value.x, Value.y, Value.z, Value.w);
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniform( const char* Name, const glm::mat2& Value )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniformMatrix2fv( Location, 1, GL_FALSE, glm::value_ptr( Value ) );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniform( const char* Name, const glm::mat3& Value )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniformMatrix3fv( Location, 1, GL_FALSE, glm::value_ptr( Value ) );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniform( const char* Name, const glm::mat4& Value )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniformMatrix4fv( Location, 1, GL_FALSE, glm::value_ptr( Value ) );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniformArray( const char* Name, int Count, const int* Values )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform1iv( Location, Count, Values );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniformArray( const char* Name, int Count, const float* Values )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform1fv( Location, Count, Values );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniformArray( const char* Name, int Count, const glm::vec2* Values )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform2fv( Location, Count, reinterpret_cast< const GLfloat* >( Values ) );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniformArray( const char* Name, int Count, const glm::vec3* Values )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform3fv( Location, Count, reinterpret_cast< const GLfloat* >( Values ) );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniformArray( const char* Name, int Count, const glm::vec4* Values )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniform4fv( Location, Count, reinterpret_cast< const GLfloat* >( Values ) );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniformArray( const char* Name, int Count, const glm::mat2* Values )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniformMatrix2fv( Location, Count, GL_FALSE, reinterpret_cast< const GLfloat* >( Values ) );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniformArray( const char* Name, int Count, const glm::mat3* Values )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniformMatrix3fv( Location, Count, GL_FALSE, reinterpret_cast< const GLfloat* >( Values ) );
	}


//--------------------------------------------------------------------------------------------------
void VsShader::SetUniformArray( const char* Name, int Count, const glm::mat4* Values )
	{
	GLint Location = glGetUniformLocation( mProgram, Name );
	glUniformMatrix4fv( Location, Count, GL_FALSE, reinterpret_cast< const GLfloat* >( Values ) );
	}


//--------------------------------------------------------------------------------------------------
// VsShaderLibrary
//--------------------------------------------------------------------------------------------------
void vsLoadShaders()
	{
	VS_ASSERT( !VsShader::GradientShader );
	VsShader::GradientShader = new VsShader( fs::path( "shaders/gradient.vs" ), fs::path( "shaders/gradient.fs" ) );
	VS_ASSERT( !VsShader::GridShader );
	VsShader::GridShader = new VsShader( fs::path( "shaders/grid.vs" ), fs::path( "shaders/grid.fs" ) );
	VS_ASSERT( !VsShader::MeshShader );
	VsShader::MeshShader = new VsShader( fs::path( "shaders/mesh.vs" ), fs::path( "shaders/mesh.fs" ) );
	VS_ASSERT( !VsShader::EdgeShader );
	VsShader::EdgeShader = new VsShader( fs::path( "shaders/edge.vs" ), fs::path( "shaders/edge.fs" ) );
	}


//--------------------------------------------------------------------------------------------------
void vsUnloadShaders()
	{
	VS_ASSERT( VsShader::EdgeShader );
	delete VsShader::EdgeShader;
	VsShader::EdgeShader = nullptr;

	VS_ASSERT( VsShader::MeshShader );
	delete VsShader::MeshShader;
	VsShader::MeshShader = nullptr;

	VS_ASSERT( VsShader::GridShader );
	delete VsShader::GridShader;
	VsShader::GridShader = nullptr;

	VS_ASSERT( VsShader::GradientShader );
	delete VsShader::GradientShader;
	VsShader::GradientShader = nullptr;
	}