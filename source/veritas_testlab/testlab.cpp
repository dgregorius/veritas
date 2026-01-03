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

// CRT's memory leak detection
#if defined( DEBUG ) || defined( _DEBUG )
#  include <crtdbg.h>
#endif


//--------------------------------------------------------------------------------------------------
// VsTestlab
//--------------------------------------------------------------------------------------------------
int VsTestlab::Run()
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


//--------------------------------------------------------------------------------------------------
void VsTestlab::Startup()
	{
	// Initialize plugin framework
	for ( const auto& Entry : fs::recursive_directory_iterator( "plugins" ) )
		{
		if ( Entry.is_regular_file() )
			{
			// Check for your "veritas_" prefix
			std::string Filename = Entry.path().filename().string();
			if ( Filename.rfind( "veritas_", 0 ) == 0 )
				{
				// Get the absolute path for the loader
				fs::path PluginPath = Entry.path();
				mPlugins.emplace_back( PluginPath );
				}
			}
		}

	// Initialize test framework
	std::vector< VsTestEntry >& TestEntries = vsGetTestEntries();
	std::sort( TestEntries.begin(), TestEntries.end(), []( VsTestEntry Lhs, VsTestEntry Rhs )
		{
		int CategoryCompare = strcmp( Lhs.Category, Rhs.Category );
		if ( CategoryCompare == 0 )
			{
			return strcmp( Lhs.Name, Rhs.Name ) < 0;
			}

		return CategoryCompare < 0;
		} );

	mTests.reserve( mPlugins.size() );
	for ( VsPluginInstance& PluginInstance : mPlugins )
		{
		mTests.push_back( TestEntries[ mTestIndex ].Creator( PluginInstance.Get() ) );
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::BeginFrame()
	{

	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::UpdateFrame()
	{

	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::EndFrame()
	{

	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::Shutdown()
	{
	// Terminate test framework
	mTestIndex = -1;
	std::for_each( mTests.begin(), mTests.end(), []( VsTest* Test ) { delete Test; } );
	mTests.clear();
	
	// Terminate plugin framework
	mPlugins.clear();
	}


//--------------------------------------------------------------------------------------------------
// Every saga begins with a first step...
//--------------------------------------------------------------------------------------------------
int main()
	{
	// Enable run-time memory check for debug builds
#if defined( DEBUG ) || defined( _DEBUG )
	_CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	//_CrtSetBreakAlloc( 666 );
#endif

	VsTestlab PhysicsLab;
	return PhysicsLab.Run();
	}