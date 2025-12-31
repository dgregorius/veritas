//--------------------------------------------------------------------------------------------------
// testlab.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "testlab.h"

// ImGUI
#include <imgui.h>
#include <imgui_internal.h>

// OpenGL
#include <glad.h>
#include <glfw3.h>


//--------------------------------------------------------------------------------------------------
// TlTestLab
//--------------------------------------------------------------------------------------------------
int TlTestLab::Run( const char* Title, int Width, int Height, bool VSync = true )
	{
	// Initialize GLFW
	if ( !glfwInit() )
		{
		return EXIT_FAILURE;
		}

	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 4 );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 6 );
	glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );
	glfwWindowHint( GLFW_OPENGL_FORWARD_COMPAT, GLFW_FALSE );

#if defined( DEBUG ) || defined( _DEBUG )
	glfwWindowHint( GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE );
#endif

	mWindow = glfwCreateWindow( Width, Height, Title, nullptr, nullptr );
	if ( !mWindow )
		{
		glfwTerminate();
		return EXIT_FAILURE;
		}

	glfwMakeContextCurrent( mWindow );
	glfwSwapInterval( VSync ? 1 : 0 );

	// Setup GLAD bindings
	if ( !gladLoadGLLoader( (GLADloadproc)glfwGetProcAddress ) )
		{
		glfwTerminate();
		return EXIT_FAILURE;
		}

#if defined( DEBUG ) || defined( _DEBUG )
	glEnable( GL_DEBUG_OUTPUT );
	glEnable( GL_DEBUG_OUTPUT_SYNCHRONOUS );
	glDebugMessageCallback( gladDebugOutput, nullptr );
	glDebugMessageControl( GL_DEBUG_SOURCE_API, GL_DEBUG_TYPE_ERROR, GL_DEBUG_SEVERITY_LOW, 0, NULL, GL_TRUE );
#endif

	// Simulation loop
	ImGui::Startup( mWindow );
		{
		Startup();
		while ( !glfwWindowShouldClose( mWindow ) )
			{
			ImGui::BeginFrame( mWindow );
				{
				BeginFrame();
				UpdateFrame();
				EndFrame();
				}
			ImGui::EndFrame( mWindow );

			glfwSwapBuffers( mWindow );
			glfwPollEvents();
			}
		Shutdown();
		}
	ImGui::Shutdown( mWindow );

	// Terminate GLFW
	glfwDestroyWindow( mWindow );
	mWindow = nullptr;
	glfwTerminate();

	return EXIT_SUCCESS;
	}


//--------------------------------------------------------------------------------------------------
void TlTestLab::Startup()
	{

	}


//--------------------------------------------------------------------------------------------------
void TlTestLab::BeginFrame()
	{

	}


//--------------------------------------------------------------------------------------------------
void TlTestLab::UpdateFrame()
	{

	}


//--------------------------------------------------------------------------------------------------
void TlTestLab::EndFrame()
	{

	}


//--------------------------------------------------------------------------------------------------
void TlTestLab::Shutdown()
	{

	}