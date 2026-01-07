//--------------------------------------------------------------------------------------------------
// viewportwindow.cpp
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "viewportwindow.h"

// ImGUI
#include <imgui.h>
#include <imgui_internal.h>

// OpenGL
#include <glad.h>
#include <glfw3.h>


//--------------------------------------------------------------------------------------------------
// VsViewportWindow
//--------------------------------------------------------------------------------------------------
VsViewportWindow::VsViewportWindow( const char* Name, bool Visible )
	: VsWindow( Name, Visible )
	{
	mCamera = new VsCamera;
	mRenderTarget = new VsRenderTarget;
	}


//--------------------------------------------------------------------------------------------------
VsViewportWindow::~VsViewportWindow()
	{
	delete mRenderTarget;
	delete mCamera;
	}


//--------------------------------------------------------------------------------------------------
void VsViewportWindow::Render( VsSession* Session )
	{
	mRenderTarget->Bind();
	mRenderTarget->Clear();

	VsShader* BackgroundShader = VsShaderLibrary::BackgroundShader;
	BackgroundShader->Use();

	glBindVertexArray( VsEmptyVertex::Format );
	glDepthMask( GL_FALSE );
	glDrawArrays( GL_TRIANGLES, 0, 3 );
	glDepthMask( GL_TRUE );
	glEnable( GL_DEPTH_TEST );
	glBindVertexArray( GL_NONE );

	mRenderTarget->Unbind();

	if ( mVisible )
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
	}