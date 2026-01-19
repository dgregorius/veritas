//--------------------------------------------------------------------------------------------------
// testlab.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "testlab.h"
#include "clock.h"

// Windows
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// OpenGL
#include <glad.h>
#include <glfw3.h>

// CRT's memory leak detection
#if defined( DEBUG ) || defined( _DEBUG )
#  include <crtdbg.h>
#endif

//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static void* vsAlloc( size_t Size, void* )
	{
	return malloc( Size );
	}


//--------------------------------------------------------------------------------------------------
static void vsFree( void* Address, void* )
	{
	free( Address );
	}


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
	ImGui::SetAllocatorFunctions( vsAlloc, vsFree );
	ImGui::Startup( mWindow );
		{
		Startup();
		VsClock::Start();
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
			
			glfwSwapBuffers( mWindow );
			glfwPollEvents();

			VsClock::Advance();
			}
		VsClock::Stop();
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
				fs::path ModulePath = Entry.path();
				SetDllDirectoryW( ModulePath.parent_path().c_str() );
				HMODULE hModule = LoadLibraryW( ModulePath.c_str() );
				SetDllDirectoryW( NULL );
				if ( !hModule )
					{
					continue;
					}

				VsCreatePluginFunc vsCreatePlugin = (VsCreatePluginFunc)GetProcAddress( hModule, "vsCreatePlugin" );
				if ( !vsCreatePlugin )
					{
					FreeLibrary( hModule );
					continue;
					}
				
				ImGuiContext* Context = ImGui::GetCurrentContext();
				if ( IVsPlugin* Plugin = vsCreatePlugin( Context ) )
					{
					mPlugins.emplace_back( Plugin, [ = ]( IVsPlugin* Plugin)
						{
						Plugin->Release();
						FreeLibrary( hModule );
						} );
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

	// DIRK_TODO: Find initial test index (e.g. pass in cmdline)
	int TestIndex = 0;
	mTests.resize( mPlugins.size() );
	mSamples.resize( mPlugins.size() );
	CreateTests( TestIndex );
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::BeginFrame()
	{

	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::UpdateFrame()
	{
	// Simulate test scene
	if ( VsClock::IsPaused() && !mSingleStep )
		{
		return;
		}
	mSingleStep = false;

	mTime += VsClock::GetElapsedTime();
	float Timestep = 1.0f / VsClock::GetFrequency();
	for ( size_t TestIndex = 0; TestIndex < mTests.size(); ++TestIndex )
		{
		if ( VsTest* Test = mTests[ TestIndex ] )
			{
			uint64_t Ticks1 = vsGetTicks();
			Test->Update( mCamera, Timestep );
			uint64_t Ticks2 = vsGetTicks();

			float DeltaTime = static_cast<float>( vsTicksToMilliSeconds( Ticks2 - Ticks1 ) );
			mSamples[ TestIndex ].AddPoint( mTime, DeltaTime );
			}
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::RenderFrame()
	{
	mCamera->Update();

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

	// DIRK_TODO: Check IMGUI shortcut API
	if ( ImGui::IsKeyPressed( ImGuiKey_P ) )
		{
		bool Paused = VsClock::IsPaused();
		VsClock::SetPaused( !Paused );
		}

	if ( ImGui::IsKeyPressed( ImGuiKey_R ) )
		{
		DestroyTests();
		CreateTests( mTestIndex );
		}

	if ( ImGui::IsKeyPressed( ImGuiKey_Space ) )
		{
		if ( VsClock::IsPaused() )
			{
			mSingleStep = true;
			}
		}

	if ( ImGui::IsKeyPressed( ImGuiKey_Escape ) )
		{
		glfwSetWindowShouldClose( mWindow, true );
		}
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

		if ( Test )
			{
			Test->Destroy();
			delete Test;
			}
		}

	// Terminate plugin framework
	mPlugins.clear();

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
	for ( VsTest* Test : mTests )
		{
		if ( Test )
			{
			VS_ASSERT( VsClock::GetFrequency() > 0.0f );
			Test->Render( mCamera, 1.0f / VsClock::GetFrequency() );
			}
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::BeginDockspace()
	{
	const ImGuiViewport* Viewport = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos( Viewport->WorkPos );
	ImGui::SetNextWindowSize( Viewport->WorkSize );
	ImGui::SetNextWindowViewport( Viewport->ID );

	ImGuiWindowFlags WindowFlags = 0;
	WindowFlags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking;
	WindowFlags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground;

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

			ImGui::DockBuilderDockWindow( "Common", DockRightTopCenterID );
			for ( const VsPluginPtr& Plugin : mPlugins )
				{
				ImGui::DockBuilderDockWindow( Plugin->GetName(), DockRightTopCenterID );
				}

			ImGui::DockBuilderDockWindow( "Outliner", DockLeftID );
			ImGui::DockBuilderDockWindow( "Profiler", DockBottomCenterID );
			ImGui::DockBuilderDockWindow( "Viewport", DockLeftTopCenterID );

			ImGui::DockBuilderFinish( DockspaceID );
			}

		ImGuiDockNodeFlags NodeFlags = ImGuiDockNodeFlags_NoWindowMenuButton;
		ImGui::PushStyleColor( ImGuiCol_WindowBg, IMGUI_COLOR_BACKGROUND_DARK );
		ImGui::DockSpace( DockspaceID, ImVec2( 0.0f, 0.0f ), NodeFlags );
		ImGui::PopStyleColor( 1 );
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::DrawInspector()
	{
	// Common properties
	if ( ImGui::Begin( "Common" ) )
		{
		if ( ImGui::BeginProperties( "Common" ) )
			{
			if ( ImGui::BeginSection( "Plugins" ) )
				{
				for ( const VsPluginPtr& Plugin : mPlugins )
					{
					bool Enabled = Plugin->IsEnabled();
					if ( ImGui::Property( Plugin->GetName(), Enabled ) )
						{
						DestroyTests();
						Plugin->SetEnabled( Enabled );
						CreateTests( mTestIndex );
						}
					}
				}

			if ( ImGui::BeginSection( "Settings" ) )
				{
				static bool Sleeping = true;
				if ( ImGui::Property( "Sleeping", Sleeping ) )
					{
					DestroyTests();
					CreateTests( mTestIndex );
					}
				}

			ImGui::EndProperties();
			}
		}
	ImGui::End();

	// Plugin properties
	for ( const VsPluginPtr& Plugin : mPlugins )
		{
		if ( ImGui::Begin( Plugin->GetName(), NULL, ImGuiWindowFlags_NoFocusOnAppearing ) )
			{
			if ( Plugin->OnInspectorGUI() )
				{
				DestroyTests();
				CreateTests( mTestIndex );
				}
			}
		ImGui::End();
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::DrawOutliner()
	{
	if ( ImGui::Begin( "Outliner" ) )
		{
		static char Buffer[ 256 ] = "";
		ImGui::SetNextItemWidth( -1 );  
		ImGui::PushStyleVar( ImGuiStyleVar_FrameRounding, 4.0f );
		ImGui::InputTextWithHint( "##Search", "Filter...", Buffer, sizeof( Buffer ) );
		ImGui::PopStyleVar();

		//ImGui::Separator();
		ImGui::Spacing();
	
		// Test selections
		ImGui::PushStyleColor( ImGuiCol_ChildBg, IMGUI_COLOR_BACKGROUND_DARK );
		if ( ImGui::BeginChild( "##Child", ImVec2( 0, -121 ) ) )
			{
			const std::vector< VsTestEntry >& TestEntries = vsGetTestEntries();
			for ( int TestIndex = 0; TestIndex < static_cast<int>( TestEntries.size() ); ++TestIndex )
				{
				// New category
				const char* Category = TestEntries[ TestIndex ].Category;
				if ( strcmp( TestEntries[ mTestIndex ].Category, Category ) == 0 )
					{
					// Assure the parent of the initial test selection is always open
					ImGui::SetNextItemOpen( true, ImGuiCond_Once );
					}

				ImGui::PushFont( IMGUI_FONT_REGULAR_BOLD, 0.0f );;
				ImGuiTreeNodeFlags CategoryFlags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_FramePadding;
				bool CategoryOpen = ImGui::TreeNodeEx( Category, CategoryFlags );
				ImGui::PopFont();

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
								DestroyTests();
								CreateTests( TestIndex );
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
			}
		ImGui::EndChild();
		ImGui::PopStyleColor();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		// Test management
		if ( ImGui::Button( "Pause", ImVec2( -1, 0 ) ) )
			{
			bool Paused = VsClock::IsPaused();
			VsClock::SetPaused( !Paused );
			}

		if ( ImGui::Button( "Restart", ImVec2( -1, 0 ) ) )
			{
			DestroyTests();
			CreateTests( mTestIndex );
			}

		if ( ImGui::Button( "Quit", ImVec2( -1, 0 ) ) )
			{
			glfwSetWindowShouldClose( mWindow, GLFW_TRUE );
			}
		}
	ImGui::End();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::DrawProfiler()
	{
	if ( ImGui::Begin( "Profiler" ) )
		{
		const float History = 30.0f;
		if ( ImPlot::BeginPlot( "##Scrolling", ImVec2( -1, -1 ) ) )
			{
			ImPlot::SetupAxes( "Time [s]", "Samples [ms]", ImPlotAxisFlags_NoTickLabels, ImPlotAxisFlags_AutoFit );
			ImPlot::SetupAxisLimits( ImAxis_X1, mTime - History, mTime, ImGuiCond_Always );

			ImPlot::PushStyleVar( ImPlotStyleVar_FillAlpha, 0.2f );
			for ( size_t PluginIndex = 0; PluginIndex < mPlugins.size(); ++PluginIndex )
				{
				const VsPluginPtr& Plugin = mPlugins[ PluginIndex ];
				const ImScrollingBuffer& Samples = mSamples[ PluginIndex ];
				if ( Plugin->IsEnabled() && !Samples.Empty() )
					{
					ImPlot::PlotShaded( Plugin->GetName(), &Samples.Data[ 0 ].x, &Samples.Data[ 0 ].y, Samples.Data.size(), 0.0f, 0, Samples.Offset, 2 * sizeof( float ) );
					}
				}
			ImPlot::PopStyleVar();

			for ( size_t PluginIndex = 0; PluginIndex < mPlugins.size(); ++PluginIndex )
				{
				const VsPluginPtr& Plugin = mPlugins[ PluginIndex ];
				const ImScrollingBuffer& Samples = mSamples[ PluginIndex ];
				if ( Plugin->IsEnabled() && !Samples.Empty() )
					{
					ImPlot::PlotLine( Plugin->GetName(), &Samples.Data[ 0 ].x, &Samples.Data[ 0 ].y, Samples.Data.size(), 0, Samples.Offset, 2 * sizeof( float ) );
					}
				}

			ImPlot::EndPlot();
			}
		}
	ImGui::End();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::DrawViewport()
	{
	ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImVec2( 4, 4 ) );
	if ( ImGui::Begin( "Viewport" ) )
		{
		ImGui::BeginChild( "##Child" );

		ImVec2 WindowPos = ImGui::GetCursorScreenPos();
		ImVec2 WindowSize = ImGui::GetContentRegionAvail();
		mCamera->Resize( static_cast<int>( WindowSize.x ), static_cast<int>( WindowSize.y ) );
		mRenderTarget->Resize( static_cast< int >( WindowSize.x ), static_cast< int >( WindowSize.y ) );

		ImDrawList* DrawList = ImGui::GetWindowDrawList();
		DrawList->AddImageRounded( (ImTextureID)(uintptr_t)mRenderTarget->GetTexture(), WindowPos, WindowPos + WindowSize, ImVec2( 0, 1 ), ImVec2( 1, 0 ), IM_COL32_WHITE, 4.0f );
		
		ImGui::EndChild();
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
			VsOrbit Orbit = mCamera->GetOrbit();
			ImGui::Text( "Yaw: %g, Pitch: %g, Radius %g, Target: (%g, %g, %g)", Orbit.Yaw, Orbit.Pitch, Orbit.Radius, Orbit.Target.x, Orbit.Target.y, Orbit.Target.z );
			ImGui::EndMenuBar();
			}
		}
	ImGui::End();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::CreateTests( int TestIndex )
	{
	const std::vector< VsTestEntry >& TestEntries = vsGetTestEntries();
	if ( mTestIndex != TestIndex )
		{
		mTestIndex = TestIndex;
		mCamera->SetOrbit( TestEntries[ TestIndex ].Orbit );
		}
	
	mTime = 0.0;

	int PluginCount = static_cast< int >( mPlugins.size() );
	VS_ASSERT( mTests.size() == PluginCount );
	VS_ASSERT( mSamples.size() == PluginCount );

	for ( int PluginIndex = 0; PluginIndex < PluginCount; ++PluginIndex )
		{
		VsPluginPtr& Plugin = mPlugins[ PluginIndex ];
		if ( Plugin->IsEnabled() )
			{
			VS_ASSERT( !mTests[ PluginIndex ] );
			VsCreator vsCreateTest = TestEntries[ TestIndex ].Creator;
			VsTest* Test = vsCreateTest( Plugin.get() );
			Test->Create();

			mTests[ PluginIndex ] = Test;
			}
		}
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::DestroyTests()
	{
	int PluginCount = static_cast< int >( mPlugins.size() );
	for ( int PluginIndex = 0; PluginIndex < PluginCount; ++PluginIndex )
		{
		VsTest* Test = mTests[ PluginIndex ];
		if ( Test )
			{
			Test->Destroy();
			delete Test;
			}

		mTests[ PluginIndex ] = nullptr;
		mSamples[ PluginIndex ].Erase();
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
	//_CrtSetBreakAlloc( 1012 );
#endif

	VsTestlab PhysicsLab;
	return PhysicsLab.Run();
	}