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
// TlTestLab
//--------------------------------------------------------------------------------------------------
int TlTestLab::Run()
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
void TlTestLab::Startup()
	{
	// Use recursive_directory_iterator to look inside /box3d, /jolt, etc.
	for ( const auto& Entry : fs::recursive_directory_iterator( "plugins" ) )
		{
		if ( Entry.is_regular_file() )
			{
			// Check for your "veritas_" prefix
			std::string Filename = Entry.path().filename().string();
			if ( Filename.rfind( "veritas_", 0 ) == 0 )
				{
				// Get the absolute path for the loader
				std::string PluginPath = Entry.path().string();
				mPlugins.emplace_back( vsLoadPlugin( PluginPath.c_str() ) );
				}
			}
		}

	// Initialize test framework
	std::vector< TlTestEntry >& TestEntries = tlGetTestEntries();
	std::sort( TestEntries.begin(), TestEntries.end(), []( TlTestEntry Lhs, TlTestEntry Rhs )
		{
		int CategoryCompare = strcmp( Lhs.Category, Rhs.Category );
		if ( CategoryCompare == 0 )
			{
			return strcmp( Lhs.Name, Rhs.Name ) < 0;
			}

		return CategoryCompare < 0;
		} );

	mTestIndex = 0;
	mTest = TestEntries[ mTestIndex ].Creator();
	mSingleStep = false;
	mShowProfiler = false;
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
	// Terminate test framework
	delete mTest;
	mTest = nullptr;
	mTestIndex = -1;
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

	TlTestLab PhysicsLab;
	return PhysicsLab.Run();
	}