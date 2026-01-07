//--------------------------------------------------------------------------------------------------
// inspectorwindow.cpp
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "inspectorwindow.h"

// ImGUI
#include <imgui.h>
#include <imgui_internal.h>


//--------------------------------------------------------------------------------------------------
// VsInspectorWindow
//--------------------------------------------------------------------------------------------------
VsInspectorWindow::VsInspectorWindow( const char* Name, bool Visible )
	: VsWindow( Name, Visible )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsInspectorWindow::Render( VsSession* Session )
	{
	if ( mVisible )
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
	}