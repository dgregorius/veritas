//--------------------------------------------------------------------------------------------------
// application.cpp	
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
// VsApplication
//--------------------------------------------------------------------------------------------------
int VsApplication::Run()
	{
	// Initialize GLFW
	if ( !glfwInit() )
		{
		return EXIT_FAILURE;
		}

	glfwWindowHint( GLFW_MAXIMIZED, GLFW_TRUE );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 4 );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 6 );
	glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );
	glfwWindowHint( GLFW_OPENGL_FORWARD_COMPAT, GLFW_FALSE );

#if defined( DEBUG ) || defined( _DEBUG )
	glfwWindowHint( GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE );
#endif

	mWindow = glfwCreateWindow( 1920, 1080, "TestLab", NULL, NULL );
	if ( !mWindow )
		{
		glfwTerminate();
		return EXIT_FAILURE;
		}

	glfwMakeContextCurrent( mWindow );
	glfwSwapInterval( 1 );

	// Setup GLAD bindings
	if ( !gladLoadGLLoader( (GLADloadproc)glfwGetProcAddress ) )
		{
		glfwTerminate();
		return EXIT_FAILURE;
		}

	// Initialize OpenGL
	glEnable( GL_POLYGON_OFFSET_FILL );
	glPolygonOffset( 2.0f, 2.0f );
	glEnable( GL_CULL_FACE );
	glCullFace( GL_BACK );
	glFrontFace( GL_CCW );

#if defined( DEBUG ) || defined( _DEBUG )
	glEnable( GL_DEBUG_OUTPUT );
	glEnable( GL_DEBUG_OUTPUT_SYNCHRONOUS );
	glDebugMessageCallback( gladDebugOutput, NULL );
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
				RenderFrame();
				EndFrame();
				}
			ImGui::EndFrame( mWindow );

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