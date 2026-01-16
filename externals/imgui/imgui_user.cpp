
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
	// ============== BASE PALETTE (tweak these!) ==============
	const ImVec4 bg = ImVec4( 0.14f, 0.13f, 0.12f, 1.0f );  // main background
	const ImVec4 bgDark = ImVec4( 0.10f, 0.09f, 0.08f, 1.0f );  // recessed
	const ImVec4 bgLight = ImVec4( 0.20f, 0.19f, 0.18f, 1.0f );  // raised/hover
	const ImVec4 accent = ImVec4( 0.33f, 0.52f, 0.76f, 1.0f );  // blue accent
	const ImVec4 accentDim = ImVec4( 0.24f, 0.38f, 0.55f, 1.0f );  // softer blue
	const ImVec4 text = ImVec4( 0.90f, 0.90f, 0.88f, 1.0f );  // warm white
	const ImVec4 textDim = ImVec4( 0.50f, 0.48f, 0.46f, 1.0f );  // warm gray

	// ============== DERIVED COLORS ==============
	ImGuiStyle& style = ImGui::GetStyle();
	ImVec4* c = style.Colors;

	// Backgrounds
	c[ ImGuiCol_WindowBg ] = bg;
	c[ ImGuiCol_ChildBg ] = bg;
	c[ ImGuiCol_PopupBg ] = bgDark;

	// Tabs (active matches window for seamless look)
	c[ ImGuiCol_Tab ] = bgDark;     // unselected
	c[ ImGuiCol_TabHovered ] = bgLight;    // hover
	c[ ImGuiCol_TabSelected ] = bg;         // selected (seamless!)
	c[ ImGuiCol_TabSelectedOverline ] = accent;     // shows a colored line on active tab
	c[ ImGuiCol_TabDimmed ] = bgDark;     // unfocused window, unselected
	c[ ImGuiCol_TabDimmedSelected ] = bg;         // unfocused window, selected (still seamless)
	c[ ImGuiCol_TabDimmedSelectedOverline ] = accentDim;  // subtler when window unfocused

	// Title bar
	c[ ImGuiCol_TitleBg ] = bgDark;
	c[ ImGuiCol_TitleBgActive ] = bgDark;
	c[ ImGuiCol_TitleBgCollapsed ] = bgDark;

	// Frames (inputs, sliders, combo boxes)
	c[ ImGuiCol_FrameBg ] = bgDark;
	c[ ImGuiCol_FrameBgHovered ] = bgLight;
	c[ ImGuiCol_FrameBgActive ] = accentDim;

	// Buttons
	//c[ ImGuiCol_Button ] = bgLight;
	c[ ImGuiCol_Button ] = ImVec4( 0.22f, 0.21f, 0.20f, 1.0f );
	c[ ImGuiCol_ButtonHovered ] = accentDim;
	c[ ImGuiCol_ButtonActive ] = bgDark;      // pressed = recessed

	// Headers (collapsing headers, selectable, menu items)
	//c[ ImGuiCol_Header ] = accentDim;
	c[ ImGuiCol_Header ] = ImVec4( 0.28f, 0.40f, 0.55f, 1.0f );  // softer selection
	c[ ImGuiCol_HeaderHovered ] = accentDim;
	c[ ImGuiCol_HeaderActive ] = accent;

	// Separators, borders, resize grips
	c[ ImGuiCol_Separator ] = bgLight;
	c[ ImGuiCol_Border ] = ImVec4( 0, 0, 0, 0 ); 
	c[ ImGuiCol_ResizeGrip ] = bgLight;
	c[ ImGuiCol_ResizeGripHovered ] = accentDim;
	c[ ImGuiCol_ResizeGripActive ] = accent;

	// Slider grab
	c[ ImGuiCol_SliderGrab ] = accentDim;
	c[ ImGuiCol_SliderGrabActive ] = accent;

	// Checkmark, scrollbar
	c[ ImGuiCol_CheckMark ] = accent;
	c[ ImGuiCol_ScrollbarBg ] = bgDark;
	c[ ImGuiCol_ScrollbarGrab ] = bgLight;
	c[ ImGuiCol_ScrollbarGrabHovered ] = accentDim;
	c[ ImGuiCol_ScrollbarGrabActive ] = accent;

	// Text
	c[ ImGuiCol_Text ] = text;
	c[ ImGuiCol_TextDisabled ] = textDim;

	// Misc
	c[ ImGuiCol_MenuBarBg ] = bgDark;
	c[ ImGuiCol_TableHeaderBg ] = bgDark;
	c[ ImGuiCol_TableRowBg ] = bg;
	c[ ImGuiCol_TableRowBgAlt ] = bgDark;

	// ============== STYLE TWEAKS ==============
	style.WindowRounding = 6.0f;
	style.FrameRounding = 4.0f;
	style.TabRounding = 4.0f;
	style.ScrollbarRounding = 4.0f;
	style.GrabRounding = 3.0f;
	style.WindowBorderSize = 0.0f;

	style.TabBorderSize = 0.0f;
	style.FramePadding = ImVec2( 8, 4 );
	style.ItemSpacing = ImVec2( 8, 5 );
	style.TabBarOverlineSize = 2.0f;
	style.DockingSeparatorSize = 4.0f;
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
	//ImGui::StyleColorsDark();
	ImGui::StyleColorsCustom();

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