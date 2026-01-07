//--------------------------------------------------------------------------------------------------
// testlab.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "testlab.h"
#include "session.h"

#include "windows/inspectorwindow.h"
#include "windows/outlinerwindow.h"
#include "windows/profilewindow.h"
#include "windows/viewportwindow.h"

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
void VsTestlab::Startup()
	{
	vsLoadShaderLibrary();
	vsLoadVertexLibrary();

	// Start session
	mSession = new VsSession;

	// Create dock windows
	mDockWindows.push_back( new VsInspectorWindow( "Inspector" ) );
	mDockWindows.push_back( new VsOutlinerWindow( "Outliner" ) );
	mDockWindows.push_back( new VsProfileWindow( "Profile" ) );
	mDockWindows.push_back( new VsViewportWindow( "Viewport" ) );
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
	BeginDockspace();
	for ( VsWindow* DockWindow : mDockWindows )
		{
		DockWindow->Render( mSession );
		}
	EndDockspace();
	Status();
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::EndFrame()
	{
	
	}


//--------------------------------------------------------------------------------------------------
void VsTestlab::Shutdown()
	{
	// Destroy dock windows
	std::for_each( mDockWindows.begin(), mDockWindows.end(), []( VsWindow* DockWindow ) { delete DockWindow; } );
	mDockWindows.clear();

	// Close session
	delete mSession;
	mSession = nullptr;

	vsUnloadShaderLibrary();
	vsUnloadVertexLibrary();
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
			ImGui::DockBuilderDockWindow( "Viewport", DockLeftTopCenterID );
			ImGui::DockBuilderDockWindow( "Profile", DockBottomCenterID );
			ImGui::DockBuilderFinish( DockspaceID );
			}

		ImGuiDockNodeFlags NodeFlags = ImGuiDockNodeFlags_NoWindowMenuButton | ImGuiDockNodeFlags_NoTabBar;
		ImGui::DockSpace( DockspaceID, ImVec2( 0.0f, 0.0f ), NodeFlags );
		}
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
	//_CrtSetBreakAlloc( 873 );
#endif

	VsTestlab PhysicsLab;
	return PhysicsLab.Run();
	}