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


//--------------------------------------------------------------------------------------------------
void VsTestlab::Startup()
	{
	// Initialize renderer
	vsLoadFormats();
	vsLoadShaders();
	
	mCamera = new VsCamera;
	mRenderTarget = new VsRenderTarget;

	// Initialize plugin framework
	for ( const auto& Entry : fs::recursive_directory_iterator( "plugins" ) )
		{
		if ( Entry.is_regular_file() )
			{
			// Check for your "veritas_" prefix
			std::string Filename = Entry.path().filename().string();
			if ( Filename.rfind( "veritas_", 0 ) == 0 )
				{
				if ( VsModule* Module = vsLoadModule( Entry.path() ) )
					{
					mModules.push_back( Module );
					}
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

	mTests.reserve( mModules.size() );
	for ( VsModule* Module : mModules )
		{
		VsTest* Test = TestEntries[ mTestIndex ].Creator( vsGetPlugin( Module ) );
		Test->Create( mCamera );

		mTests.push_back( Test );
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
void VsTestlab::RenderFrame()
	{
	mCamera->Update();

	// DIRK_TODO: Lighting (environment)...

	mRenderTarget->Bind();
	mRenderTarget->Clear();

	RenderGradient();
	RenderGrid();
	RenderTests();

	mRenderTarget->Unbind();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::EndFrame()
	{
	BeginDockspace();
		DrawInspector();
		DrawOutliner();
		DrawProfiler();
		DrawViewport();
	EndDockspace();
	Status();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::Shutdown()
	{
	// Terminate test framework
	mTestIndex = -1;
	while ( !mTests.empty() )
		{
		VsTest* Test = mTests.back();
		mTests.pop_back();

		Test->Destroy();
		delete Test;
		}

	// Terminate plugin framework
	while ( !mModules.empty() )
		{
		VsModule* Module = mModules.back();
		mModules.pop_back();

		vsFreeModule( Module );
		}

	// Terminate renderer
	delete mRenderTarget;
	mRenderTarget = nullptr;
	
	delete mCamera;
	mCamera = nullptr;

	vsUnloadShaders();
	vsUnloadFormats();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::RenderGradient()
	{
	VsShader* BackgroundShader = VsShader::GradientShader;
	BackgroundShader->Use();

	glBindVertexArray( VsEmptyVertex::Format );
	
	glDepthMask( GL_FALSE );
	glDrawArrays( GL_TRIANGLES, 0, 3 );
	glDepthMask( GL_TRUE );
	glEnable( GL_DEPTH_TEST );
	
	glBindVertexArray( GL_NONE );
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::RenderGrid()
	{
	VsShader* GridShader = VsShader::GridShader;
	GridShader->Use();

	glBindVertexArray( VsEmptyVertex::Format );
	
	glDisable( GL_CULL_FACE );
	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	glEnable( GL_DEPTH_TEST );
	glDepthMask( GL_FALSE );
	glDrawArrays( GL_TRIANGLES, 0, 3 );
	glDepthMask( GL_TRUE );
	glDisable( GL_BLEND );
	glEnable( GL_CULL_FACE );

	glBindVertexArray( GL_NONE );
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::RenderTests()
	{
	double Time = 0;
	float ElapsedTime = 0.0f;

	for ( VsTest* Test : mTests )
		{
		Test->Render( Time, ElapsedTime );
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::BeginDockspace()
	{
	ImGuiWindowFlags WindowFlags = ImGuiWindowFlags_NoDocking;
	WindowFlags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
	WindowFlags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

	const ImGuiViewport* Viewport = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos( Viewport->WorkPos );
	ImGui::SetNextWindowSize( Viewport->WorkSize );
	ImGui::SetNextWindowViewport( Viewport->ID );

	ImGui::PushStyleVar( ImGuiStyleVar_WindowRounding, 0.0f );
	ImGui::PushStyleVar( ImGuiStyleVar_WindowBorderSize, 0.0f ); 
	ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImVec2( 0.0f, 0.0f ) );
	bool Open = ImGui::Begin( "##Testlab", nullptr, WindowFlags );
	ImGui::PopStyleVar( 3 );

	if ( Open )
		{
		ImGuiID DockspaceID = ImGui::GetID( "##DockspaceId" );
		if ( !ImGui::DockBuilderGetNode( DockspaceID )  )
			{
			ImGui::DockBuilderRemoveNode( DockspaceID );
			ImGui::DockBuilderAddNode( DockspaceID );

			ImGuiID DockLeftID, DockCenterID;
			ImGui::DockBuilderSplitNode( DockspaceID, ImGuiDir_Left, 0.15f, &DockLeftID, &DockCenterID );
			ImGuiID DockTopCenterID, DockBottomCenterID;
			ImGui::DockBuilderSplitNode( DockCenterID, ImGuiDir_Up, 0.7f, &DockTopCenterID, &DockBottomCenterID );
			ImGuiID DockLeftTopCenterID, DockRightTopCenterID;
			ImGui::DockBuilderSplitNode( DockTopCenterID, ImGuiDir_Left, 0.85f, &DockLeftTopCenterID, &DockRightTopCenterID );

			ImGui::DockBuilderDockWindow( "Inspector", DockRightTopCenterID );
			ImGui::DockBuilderDockWindow( "Outliner", DockLeftID );
			ImGui::DockBuilderDockWindow( "Profiler", DockBottomCenterID );
			ImGui::DockBuilderDockWindow( "Viewport", DockLeftTopCenterID );
			ImGui::DockBuilderFinish( DockspaceID );
			}

		ImGuiDockNodeFlags NodeFlags = ImGuiDockNodeFlags_NoWindowMenuButton | ImGuiDockNodeFlags_NoTabBar;
		ImGui::DockSpace( DockspaceID, ImVec2( 0.0f, 0.0f ), NodeFlags );
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::DrawInspector()
	{
	float Scale = ImGui::GetWindowDpiScale();
	ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 6, 6 ) * Scale ) );
	if ( ImGui::Begin( "Inspector" ) )
		{
		ImGui::PushStyleVar( ImGuiStyleVar_ChildRounding, 6.0f );
		ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 6, 2 ) * Scale ) );
		ImGui::PushStyleColor( ImGuiCol_ChildBg, IM_COL32( 48, 48, 48, 255 ) );
		ImGui::BeginChild( "##Child", ImVec2( 0.0f, 0.0f ), ImGuiChildFlags_AlwaysUseWindowPadding );

		ImGui::EndChild();
		ImGui::PopStyleColor();
		ImGui::PopStyleVar( 2 );
		}
	ImGui::End();
	ImGui::PopStyleVar();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::DrawOutliner()
	{
	float Scale = ImGui::GetWindowDpiScale();
	ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 6, 6 ) * Scale ) );
	if ( ImGui::Begin( "Outliner" ) )
		{
		ImGui::PushStyleVar( ImGuiStyleVar_ChildRounding, 6.0f );
		ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 6, 2 ) * Scale ) );
		ImGui::PushStyleColor( ImGuiCol_ChildBg, IM_COL32( 48, 48, 48, 255 ) );
		ImGui::BeginChild( "##Child", ImVec2( 0.0f, 0.0f ), ImGuiChildFlags_AlwaysUseWindowPadding );

		ImGui::EndChild();
		ImGui::PopStyleColor();
		ImGui::PopStyleVar( 2 );
		}
	ImGui::End();
	ImGui::PopStyleVar();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::DrawProfiler()
	{
	float Scale = ImGui::GetWindowDpiScale();
	ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 6, 6 ) * Scale ) );
	if ( ImGui::Begin( "Profiler" ) )
		{
		ImGui::PushStyleVar( ImGuiStyleVar_ChildRounding, 6.0f );
		ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 6, 2 ) * Scale ) );
		ImGui::PushStyleColor( ImGuiCol_ChildBg, IM_COL32( 48, 48, 48, 255 ) );
		ImGui::BeginChild( "##Child", ImVec2( 0.0f, 0.0f ), ImGuiChildFlags_AlwaysUseWindowPadding );

		ImGui::EndChild();
		ImGui::PopStyleColor();
		ImGui::PopStyleVar( 2 );
		}
	ImGui::End();
	ImGui::PopStyleVar();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::DrawViewport()
	{
	float Scale = ImGui::GetWindowDpiScale();
	ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 6, 6 ) * Scale ) );
	if ( ImGui::Begin( "Viewport" ) )
		{
		ImGui::PushStyleVar( ImGuiStyleVar_ChildRounding, 6.0f );
		ImGui::PushStyleColor( ImGuiCol_ChildBg, IM_COL32( 48, 48, 48, 255 ) );
		ImGui::BeginChild( "##Child" );

		ImVec2 WindowPos = ImGui::GetCursorScreenPos();
		ImVec2 WindowSize = ImGui::GetContentRegionAvail();
		mCamera->SetAspectRatio( WindowSize.x / ImMax( 1.0f, WindowSize.y ) );
		mRenderTarget->Resize( static_cast<int>( WindowSize.x ), static_cast<int>( WindowSize.y ) );

		ImDrawList* DrawList = ImGui::GetWindowDrawList();
		DrawList->AddImageRounded( (ImTextureID)(uintptr_t)mRenderTarget->GetTexture(), WindowPos, WindowPos + WindowSize, ImVec2( 0, 1 ), ImVec2( 1, 0 ), IM_COL32_WHITE, 6.0f );

		ImGui::EndChild();
		ImGui::PopStyleVar();
		ImGui::PopStyleColor();
		}
	ImGui::End();
	ImGui::PopStyleVar();	
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::EndDockspace()
	{
	ImGui::End();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::Status()
	{
	ImGuiWindowFlags WindowFlags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar;
	if ( ImGui::BeginViewportSideBar( "##Status", NULL, ImGuiDir_Down, ImGui::GetFrameHeight(), WindowFlags ) )
		{
		if ( ImGui::BeginMenuBar() )
			{
			ImGui::Text( "Ready..." );
			ImGui::EndMenuBar();
			}
		}
	ImGui::End();
	}


//--------------------------------------------------------------------------------------------------
// Every saga begins with a first step...
//--------------------------------------------------------------------------------------------------
int main()
	{
	// Enable run-time memory check for debug builds
#if defined( DEBUG ) || defined( _DEBUG )
	_CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	//_CrtSetBreakAlloc( 1012 );
#endif

	VsTestlab PhysicsLab;
	return PhysicsLab.Run();
	}