//--------------------------------------------------------------------------------------------------
// profilewindow.cpp
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "profilewindow.h"

// ImGUI
#include <imgui.h>
#include <imgui_internal.h>


//--------------------------------------------------------------------------------------------------
// VsProfileWindow
//--------------------------------------------------------------------------------------------------
VsProfileWindow::VsProfileWindow( const char* Name, bool Visible )
	: VsWindow( Name, Visible )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsProfileWindow::Render( VsSession* Session )
	{
	if ( mVisible )
		{
		float Scale = ImGui::GetWindowDpiScale();
		ImGui::PushStyleVar( ImGuiStyleVar_WindowPadding, ImFloor( ImVec2( 6, 6 ) * Scale ) );
		if ( ImGui::Begin( "Profile" ) )
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
	}