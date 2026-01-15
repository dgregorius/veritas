
// dear imgui: imgui_user.cpp
#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_user.h"

#include "implot.h"
#include "implot_internal.h"

#include <windows.h>
#include <dwmapi.h>

#define GLFW_EXPOSE_NATIVE_WIN32
#include <glad.h>
#include <glfw3.h>
#include <glfw3native.h>

#pragma comment(lib, "dwmapi.lib")


//--------------------------------------------------------------------------------------------------
// Constants
//--------------------------------------------------------------------------------------------------
float IMGUI_UI_SCALE = 0.0f;
float IMGUI_FONT_SIZE = 0.0f;
ImFont* IMGUI_FONT_REGULAR = nullptr;
ImFont* IMGUI_FONT_REGULAR_MEDIUM = nullptr;
ImFont* IMGUI_FONT_REGULAR_BOLD = nullptr;
ImFont* IMGUI_FONT_SMALL = nullptr;


//--------------------------------------------------------------------------------------------------
// Helper functions
//--------------------------------------------------------------------------------------------------
namespace ImGui
{
void StyleColorsCustom()
	{
	ImGuiStyle& style = ImGui::GetStyle();
	ImGui::StyleColorsDark( &style );

	// --- Style (spacing/rounding close to Godot's feel) ---
	style.Alpha = 1.0f;
	style.DisabledAlpha = 0.60f;
	style.WindowPadding = ImVec2( 10, 10 );
	style.FramePadding = ImVec2( 6, 4 );
	style.ItemSpacing = ImVec2( 8, 6 );
	style.ItemInnerSpacing = ImVec2( 6, 4 );
	style.IndentSpacing = 16.0f;
	style.ScrollbarSize = 14.0f;
	style.GrabMinSize = 10.0f;

	style.WindowBorderSize = 1.0f;
	style.ChildBorderSize = 1.0f;
	style.PopupBorderSize = 1.0f;
	style.FrameBorderSize = 1.0f;
	style.TabBorderSize = 0.0f;

	style.WindowRounding = 6.0f;
	style.ChildRounding = 6.0f;
	style.PopupRounding = 6.0f;
	style.FrameRounding = 5.0f;
	style.ScrollbarRounding = 9.0f;
	style.GrabRounding = 5.0f;
	style.TabRounding = 6.0f;

	// --- Colors ---
	// Base
	const ImVec4 text = ImVec4( 0.951f, 0.951f, 0.951f, 1.00f ); // #D7D7D7
	const ImVec4 text_dim = ImVec4( 0.647f, 0.647f, 0.647f, 1.00f ); // #A5A5A5
	const ImVec4 text_dis = ImVec4( 0.463f, 0.463f, 0.463f, 1.00f ); // #767676

	const ImVec4 bg0 = ImVec4( 0.075f, 0.075f, 0.075f, 1.00f ); // #131313
	const ImVec4 bg1 = ImVec4( 0.098f, 0.098f, 0.098f, 1.00f ); // #191919
	const ImVec4 bg2 = ImVec4( 0.145f, 0.145f, 0.145f, 1.00f ); // #252525
	const ImVec4 bg3 = ImVec4( 0.224f, 0.224f, 0.224f, 1.00f ); // #393939

	const ImVec4 border = ImVec4( 0.286f, 0.286f, 0.286f, 1.00f ); // #494949
	const ImVec4 border_soft = ImVec4( 0.188f, 0.192f, 0.196f, 1.00f ); // #303132

	// Slight bluish frames (Godot-ish)
	const ImVec4 frame = ImVec4( 0.114f, 0.137f, 0.169f, 1.00f ); // #1D232B
	const ImVec4 frame_h = ImVec4( 0.137f, 0.165f, 0.208f, 1.00f ); // #232A35
	const ImVec4 frame_a = ImVec4( 0.161f, 0.208f, 0.275f, 1.00f ); // #293546

	// Accent blues from the selection highlight in the screenshot
	const ImVec4 blue = ImVec4( 0.220f, 0.376f, 0.584f, 1.00f ); // #386095
	const ImVec4 blue_h = ImVec4( 0.184f, 0.306f, 0.467f, 1.00f ); // #2F4E77
	const ImVec4 blue_a = ImVec4( 0.369f, 0.592f, 0.863f, 1.00f ); // #5E97DC
	const ImVec4 blue_light = ImVec4( 0.471f, 0.643f, 0.878f, 1.00f ); // #78A4E0

	ImVec4* c = style.Colors;
	c[ ImGuiCol_Text ] = text;
	c[ ImGuiCol_TextDisabled ] = text_dis;

	c[ ImGuiCol_WindowBg ] = bg1;
	c[ ImGuiCol_ChildBg ] = bg1;
	c[ ImGuiCol_PopupBg ] = ImVec4( bg2.x, bg2.y, bg2.z, 0.98f );

	c[ ImGuiCol_Border ] = border;
	c[ ImGuiCol_BorderShadow ] = ImVec4( 0, 0, 0, 0 );

	c[ ImGuiCol_FrameBg ] = frame;
	c[ ImGuiCol_FrameBgHovered ] = frame_h;
	c[ ImGuiCol_FrameBgActive ] = frame_a;

	c[ ImGuiCol_TitleBg ] = bg2;
	c[ ImGuiCol_TitleBgActive ] = bg3;
	c[ ImGuiCol_TitleBgCollapsed ] = bg2;

	c[ ImGuiCol_MenuBarBg ] = bg2;

	c[ ImGuiCol_ScrollbarBg ] = bg0;
	c[ ImGuiCol_ScrollbarGrab ] = bg3;
	c[ ImGuiCol_ScrollbarGrabHovered ] = border;
	c[ ImGuiCol_ScrollbarGrabActive ] = blue;

	c[ ImGuiCol_CheckMark ] = blue_light;

	c[ ImGuiCol_SliderGrab ] = blue;
	c[ ImGuiCol_SliderGrabActive ] = blue_a;

	c[ ImGuiCol_Button ] = bg2;
	c[ ImGuiCol_ButtonHovered ] = frame_h;
	c[ ImGuiCol_ButtonActive ] = frame_a;

	c[ ImGuiCol_Header ] = bg2;
	c[ ImGuiCol_HeaderHovered ] = frame_h;
	c[ ImGuiCol_HeaderActive ] = blue;

	c[ ImGuiCol_Separator ] = border_soft;
	c[ ImGuiCol_SeparatorHovered ] = blue_h;
	c[ ImGuiCol_SeparatorActive ] = blue;

	c[ ImGuiCol_ResizeGrip ] = ImVec4( blue.x, blue.y, blue.z, 0.15f );
	c[ ImGuiCol_ResizeGripHovered ] = ImVec4( blue_h.x, blue_h.y, blue_h.z, 0.55f );
	c[ ImGuiCol_ResizeGripActive ] = ImVec4( blue.x, blue.y, blue.z, 0.85f );

	c[ ImGuiCol_Tab ] = bg2;
	c[ ImGuiCol_TabHovered ] = frame_h;
	c[ ImGuiCol_TabSelected ] = bg3;
	c[ ImGuiCol_TabSelectedOverline ] = blue;
	c[ ImGuiCol_TabDimmed ] = bg2;
	c[ ImGuiCol_TabDimmedSelected ] = bg3;

	c[ ImGuiCol_DockingPreview ] = ImVec4( blue_a.x, blue_a.y, blue_a.z, 0.70f );
	c[ ImGuiCol_DockingEmptyBg ] = bg0;

	c[ ImGuiCol_PlotLines ] = text_dim;
	c[ ImGuiCol_PlotLinesHovered ] = blue_light;
	c[ ImGuiCol_PlotHistogram ] = blue;
	c[ ImGuiCol_PlotHistogramHovered ] = blue_a;

	c[ ImGuiCol_TableHeaderBg ] = bg3;
	c[ ImGuiCol_TableBorderStrong ] = border;
	c[ ImGuiCol_TableBorderLight ] = border_soft;
	c[ ImGuiCol_TableRowBg ] = ImVec4( bg1.x, bg1.y, bg1.z, 0.00f );
	c[ ImGuiCol_TableRowBgAlt ] = ImVec4( bg2.x, bg2.y, bg2.z, 0.35f );

	c[ ImGuiCol_TextSelectedBg ] = ImVec4( blue_h.x, blue_h.y, blue_h.z, 0.60f );
	c[ ImGuiCol_DragDropTarget ] = ImVec4( blue_a.x, blue_a.y, blue_a.z, 0.90f );

	c[ ImGuiCol_NavCursor ] = blue_a;
	c[ ImGuiCol_NavWindowingHighlight ] = ImVec4( 1, 1, 1, 0.70f );
	c[ ImGuiCol_NavWindowingDimBg ] = ImVec4( 0, 0, 0, 0.20f );
	c[ ImGuiCol_ModalWindowDimBg ] = ImVec4( 0, 0, 0, 0.55f );
	}

void Startup( GLFWwindow* Window )
	{
	BOOL DarkMode = TRUE;
	HWND hWnd = glfwGetWin32Window( Window );
	DwmSetWindowAttribute( hWnd, DWMWA_USE_IMMERSIVE_DARK_MODE, &DarkMode, sizeof( DarkMode ) );

	float ScaleX, ScaleY;
	glfwGetWindowContentScale( Window, &ScaleX, &ScaleY );
	IMGUI_UI_SCALE = ( ScaleX + ScaleY ) / 2.0f;

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImPlot::CreateContext();
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsCustom();

	ImGuiIO& IO = ImGui::GetIO();
	IO.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	IO.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
	IO.ConfigDpiScaleFonts = true;
	IO.ConfigDpiScaleViewports = true;

	ImGuiStyle& Style = ImGui::GetStyle();
	Style.ScaleAllSizes( IMGUI_UI_SCALE );
	Style.FontScaleDpi = IMGUI_UI_SCALE;
	Style.FontSizeBase = 14.0f;

	IMGUI_FONT_SIZE = 14.0f * IMGUI_UI_SCALE;
	IMGUI_FONT_REGULAR = IO.Fonts->AddFontFromFileTTF( "fonts/inter.ttf", IMGUI_FONT_SIZE );
	IMGUI_FONT_REGULAR_MEDIUM = IO.Fonts->AddFontFromFileTTF( "fonts/inter_medium.ttf", IMGUI_FONT_SIZE );
	IMGUI_FONT_REGULAR_BOLD = IO.Fonts->AddFontFromFileTTF( "fonts/inter_bold.ttf", IMGUI_FONT_SIZE );
	IMGUI_FONT_SMALL = IO.Fonts->AddFontFromFileTTF( "fonts/inter.ttf", 0.85f * IMGUI_FONT_SIZE );
	IO.FontDefault = IMGUI_FONT_REGULAR;

#if defined( DEBUG ) || defined( _DEBUG )
	IO.IniFilename = NULL;
	IO.LogFilename = NULL;
#endif

	ImGui_ImplGlfw_InitForOpenGL( Window, true );
	ImGui_ImplOpenGL3_Init();
	}

void BeginFrame( GLFWwindow* Window )
	{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	}

void EndFrame( GLFWwindow* Window )
	{
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );
	
	// Update and Render additional Platform Windows
	// (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
	//  For this specific demo app we could also call SDL_GL_MakeCurrent(window, gl_context) directly)
	ImGuiIO& IO = ImGui::GetIO();
	if ( IO.ConfigFlags & ImGuiConfigFlags_ViewportsEnable )
		{
		ImGui::UpdatePlatformWindows();
		ImGui::RenderPlatformWindowsDefault();
		glfwMakeContextCurrent( Window );
		}
	}

void Shutdown( GLFWwindow* )
	{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	
	ImGui::DestroyContext();
	ImPlot::DestroyContext();
	}
}