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
		mTests.push_back( TestEntries[ mTestIndex ].Creator( vsGetPlugin( Module ) ) );
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
	// Render UI
	BeginDockspace();
	Explorer();
	Viewport();
	Profiler();
	EndDockspace();
	Shortcuts();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::Shutdown()
	{
	// Terminate test framework
	mTestIndex = -1;
	std::for_each( mTests.begin(), mTests.end(), []( VsTest* Test ) { delete Test; } );
	mTests.clear();
	
	// Terminate plugin framework
	std::for_each( mModules.begin(), mModules.end(), []( VsModule* Module ) { vsFreeModule( Module ); } );
	mModules.clear();
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

			ImGuiID DockLeftID, DockRightID;
			ImGui::DockBuilderSplitNode( DockspaceID, ImGuiDir_Left, 0.1f, &DockLeftID, &DockRightID );
			ImGuiID DockTopRightID, DockBottomIRightID;
			ImGui::DockBuilderSplitNode( DockRightID, ImGuiDir_Up, 0.7f, &DockTopRightID, &DockBottomIRightID );

			ImGui::DockBuilderDockWindow( "Explorer", DockLeftID );
			ImGui::DockBuilderDockWindow( "Viewport", DockTopRightID );
			ImGui::DockBuilderDockWindow( "Profiler", DockBottomIRightID );
			ImGui::DockBuilderFinish( DockspaceID );
			}

		ImGuiDockNodeFlags NodeFlags = ImGuiDockNodeFlags_NoWindowMenuButton | ImGuiDockNodeFlags_NoTabBar;
		ImGui::DockSpace( DockspaceID, ImVec2( 0.0f, 0.0f ), NodeFlags );
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::Explorer()
	{
	static bool Visible = true;
	if ( Visible )
		{
		float Scale = ImGui::GetWindowDpiScale();
		ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 8, 8 ) * Scale ) );
		if ( ImGui::Begin( "Explorer" ) )
			{
			ImGui::PushStyleVar( ImGuiStyleVar_ChildRounding, 8.0f );
			ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 6, 2 ) * Scale ) );
			ImGui::PushStyleColor( ImGuiCol_ChildBg, IM_COL32( 48, 48, 48, 255 ) );
			ImGui::BeginChild( "##Child", ImVec2( 0.0f, 0.0f ), ImGuiChildFlags_AlwaysUseWindowPadding );
			
			// Test selections
			const std::vector< VsTestEntry >& TestEntries = vsGetTestEntries();
			for ( size_t TestIndex = 0; TestIndex < TestEntries.size(); ++TestIndex )
				{
				// New category
				const char* Category = TestEntries[ TestIndex ].Category;
				if ( strcmp( TestEntries[ mTestIndex ].Category, Category ) == 0 )
					{
					// Assure the parent of the initial test selection is always open
					ImGui::SetNextItemOpen( true, ImGuiCond_Once );
					}

				ImGuiTreeNodeFlags CategoryFlags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_FramePadding;
				bool CategoryOpen = ImGui::TreeNodeEx( Category, CategoryFlags );

				while ( true )
					{
					if ( CategoryOpen )
						{
						bool Selected = ( TestIndex == mTestIndex );
						ImGuiTreeNodeFlags TestFlags = ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_FramePadding;
						if ( Selected )
							{
							TestFlags |= ImGuiTreeNodeFlags_Selected;
							}

						if ( ImGui::TreeNodeEx( TestEntries[ TestIndex ].Name, TestFlags ) )
							{
							if ( ImGui::IsItemClicked() )
								{
// 								delete mTest;
// 								mTestIndex = TestIndex;
// 								mTest = TestEntries[ mTestIndex ].Creator( mCamera );
								}

							ImGui::TreePop();
							}
						}

					const char* NextCategory = TestIndex + 1 < TestEntries.size() ? TestEntries[ TestIndex + 1 ].Category : "";
					if ( strcmp( NextCategory, Category ) != 0 )
						{
						break;
						}

					TestIndex++;
					}

				if ( CategoryOpen )
					{
					ImGui::TreePop();
					}
				}

			ImGui::Spacing();
			ImGui::Separator();
			ImGui::Spacing();

			// Test management
			if ( ImGui::Button( "Pause", ImVec2( -1, 0 ) ) )
				{
// 				bool Paused = RkClock::IsPaused();
// 				RkClock::SetPaused( !Paused );
				}

			if ( ImGui::Button( "Restart", ImVec2( -1, 0 ) ) )
				{
// 				delete mTest;
// 				mTest = TestEntries[ mTestIndex ].Creator( nullptr );
				}

			if ( ImGui::Button( "Quit", ImVec2( -1, 0 ) ) )
				{
				glfwSetWindowShouldClose( mWindow, true );
				}

			ImGui::EndChild();
			ImGui::PopStyleColor();
			ImGui::PopStyleVar( 2 );
			}
		ImGui::End();
		ImGui::PopStyleVar();
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::Viewport()
	{
	static bool Visible = true;
	if ( Visible )
		{
		float Scale = ImGui::GetWindowDpiScale();
		ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 8, 8 ) * Scale ) );
		if ( ImGui::Begin( "Viewport" ) )
			{
			ImGui::PushStyleVar( ImGuiStyleVar_ChildRounding, 8.0f );
			ImGui::PushStyleColor( ImGuiCol_ChildBg, IM_COL32( 48, 48, 48, 255 ) );
			ImGui::BeginChild( "##Child" );

			ImGui::EndChild();
			ImGui::PopStyleVar();
			ImGui::PopStyleColor();
			}
		ImGui::End();
		ImGui::PopStyleVar();
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::Profiler()
	{
	if ( mShowProfiler )
		{
		float Scale = ImGui::GetWindowDpiScale();
		ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 8, 8 ) * Scale ) );
		if ( ImGui::Begin( "Profiler" ) )
			{
			ImGui::PushStyleVar( ImGuiStyleVar_ChildRounding, 8.0f );
			ImGui::PushStyleColor( ImGuiCol_ChildBg, IM_COL32( 48, 48, 48, 255 ) );
			ImGui::BeginChild( "##Child" );

			ImGui::EndChild();
			ImGui::PopStyleVar();
			ImGui::PopStyleColor();
			}
		ImGui::End();
		ImGui::PopStyleVar();
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::EndDockspace()
	{
	ImGui::End();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::Shortcuts()
	{
	if ( ImGui::IsKeyPressed( ImGuiKey_P ) )
		{
		
		}

	if ( ImGui::IsKeyPressed( ImGuiKey_R ) )
		{

		}

	if ( ImGui::IsKeyPressed( ImGuiKey_Tab ) )
		{
		mShowProfiler = !mShowProfiler;
		}

	if ( ImGui::IsKeyPressed( ImGuiKey_Space ) )
		{
		
		}

	if ( ImGui::IsKeyPressed( ImGuiKey_Escape ) )
		{
		glfwSetWindowShouldClose( mWindow, true );
		}
	}


//--------------------------------------------------------------------------------------------------
// Every saga begins with a first step...
//--------------------------------------------------------------------------------------------------
int main()
	{
	// Enable run-time memory check for debug builds
#if defined( DEBUG ) || defined( _DEBUG )
	_CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	//_CrtSetBreakAlloc( 873 );
#endif

	VsTestlab PhysicsLab;
	return PhysicsLab.Run();
	}