
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
	// ============== DERIVED COLORS ==============
	ImGuiStyle& Style = ImGui::GetStyle();
	ImVec4* Colors = Style.Colors;

	// Backgrounds
	Colors[ ImGuiCol_WindowBg ] = IMGUI_COLOR_BACKGROUND;
	Colors[ ImGuiCol_ChildBg ] = IMGUI_COLOR_BACKGROUND_DARK;
	Colors[ ImGuiCol_PopupBg ] = IMGUI_COLOR_BACKGROUND_DARK;

	// Tabs (active matches window for seamless look)
	Colors[ ImGuiCol_Tab ] = IMGUI_COLOR_BACKGROUND_DARK;						// unselected
	Colors[ ImGuiCol_TabHovered ] = IMGUI_COLOR_BACKGROUND_LIGHT;				// hover
	Colors[ ImGuiCol_TabSelected ] = IMGUI_COLOR_BACKGROUND;					// selected (seamless!)
	Colors[ ImGuiCol_TabSelectedOverline ] = IMGUI_COLOR_ACCENT;				// shows a colored line on active tab
	Colors[ ImGuiCol_TabDimmed ] = IMGUI_COLOR_BACKGROUND_DARK;					// unfocused window, unselected
	Colors[ ImGuiCol_TabDimmedSelected ] = IMGUI_COLOR_BACKGROUND;				// unfocused window, selected (still seamless)
	Colors[ ImGuiCol_TabDimmedSelectedOverline ] = IMGUI_COLOR_ACCENT_DIM;		// subtler when window unfocused

	// Title bar
	Colors[ ImGuiCol_TitleBg ] = IMGUI_COLOR_BACKGROUND_DARK;
	Colors[ ImGuiCol_TitleBgActive ] = IMGUI_COLOR_BACKGROUND_DARK;
	Colors[ ImGuiCol_TitleBgCollapsed ] = IMGUI_COLOR_BACKGROUND_DARK;

	// Frames (inputs, sliders, combo boxes)
	Colors[ ImGuiCol_FrameBg ] = IMGUI_COLOR_BACKGROUND_DARK;
	Colors[ ImGuiCol_FrameBgHovered ] = IMGUI_COLOR_BACKGROUND_LIGHT;
	Colors[ ImGuiCol_FrameBgActive ] = IMGUI_COLOR_ACCENT_DIM;

	// Buttons
	//c[ ImGuiCol_Button ] = bgLight;
	Colors[ ImGuiCol_Button ] = ImVec4( 0.22f, 0.22f, 0.22f, 1.0f );
	Colors[ ImGuiCol_ButtonHovered ] = IMGUI_COLOR_BACKGROUND_LIGHT;
	Colors[ ImGuiCol_ButtonActive ] = ImVec4( 0.12f, 0.12f, 0.12f, 1.0f );		// pressed = recessed

	// Headers (collapsing headers, selectable, menu items)
	//c[ ImGuiCol_Header ] = accentDim;
	Colors[ ImGuiCol_Header ] = ImVec4( 0.28f, 0.40f, 0.55f, 1.0f );			// softer selection
	Colors[ ImGuiCol_HeaderHovered ] = IMGUI_COLOR_BACKGROUND_LIGHT;
	Colors[ ImGuiCol_HeaderActive ] = IMGUI_COLOR_ACCENT;

	// Separators, borders, resize grips
	Colors[ ImGuiCol_Separator ] = IMGUI_COLOR_BACKGROUND_LIGHT;
	Colors[ ImGuiCol_Border ] = ImVec4( 0, 0, 0, 0 ); 
	Colors[ ImGuiCol_ResizeGrip ] = IMGUI_COLOR_BACKGROUND_LIGHT;
	Colors[ ImGuiCol_ResizeGripHovered ] = IMGUI_COLOR_ACCENT_DIM;
	Colors[ ImGuiCol_ResizeGripActive ] = IMGUI_COLOR_ACCENT;

	// Slider grab
	Colors[ ImGuiCol_SliderGrab ] = IMGUI_COLOR_ACCENT_DIM;
	Colors[ ImGuiCol_SliderGrabActive ] = IMGUI_COLOR_ACCENT;

	// Checkmark, scrollbar
	Colors[ ImGuiCol_CheckMark ] = IMGUI_COLOR_ACCENT;
	Colors[ ImGuiCol_ScrollbarBg ] = IMGUI_COLOR_BACKGROUND_DARK;
	Colors[ ImGuiCol_ScrollbarGrab ] = IMGUI_COLOR_BACKGROUND_LIGHT;
	Colors[ ImGuiCol_ScrollbarGrabHovered ] = IMGUI_COLOR_ACCENT_DIM;
	Colors[ ImGuiCol_ScrollbarGrabActive ] = IMGUI_COLOR_ACCENT;

	// Text
	Colors[ ImGuiCol_Text ] = IMGUI_COLOR_TEXT;
	Colors[ ImGuiCol_TextDisabled ] = IMGUI_COLOR_TEXT_DIM;

	// Misc
	Colors[ ImGuiCol_MenuBarBg ] = IMGUI_COLOR_BACKGROUND_DARK;
	Colors[ ImGuiCol_TableHeaderBg ] = IMGUI_COLOR_BACKGROUND_DARK;
	Colors[ ImGuiCol_TableRowBg ] = IMGUI_COLOR_BACKGROUND;
	Colors[ ImGuiCol_TableRowBgAlt ] = IMGUI_COLOR_BACKGROUND_DARK;

	// ============== STYLE TWEAKS ==============
	Style.WindowRounding = 6.0f;
	Style.FrameRounding = 4.0f;
	Style.TabRounding = 4.0f;
	Style.ScrollbarRounding = 4.0f;
	Style.GrabRounding = 3.0f;
	Style.WindowBorderSize = 0.0f;

	Style.TabBorderSize = 0.0f;
	Style.FramePadding = ImVec2( 8, 4 );
	Style.ItemSpacing = ImVec2( 8, 4 );
	Style.TabBarOverlineSize = 2.0f;
	Style.DockingSeparatorSize = 4.0f;
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

// Properties
static void BeginProperty( const char* Label, const char* Tooltip )
	{
	ImGui::TableNextRow();
	ImGui::TableNextColumn();

	float Padding = 8.0f;
	float ColumnWidth = ImGui::GetColumnWidth();
	float LableWidth = ImGui::CalcTextSize( Label ).x;
	ImGui::SetCursorPosX( ImGui::GetCursorPosX() + ColumnWidth - LableWidth - Padding );

	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted( Label );
	if ( Tooltip )
		{
		ImGui::SetItemTooltip( "%s", Tooltip );
		}
		
	ImGui::TableNextColumn();
	ImGui::SetCursorPosX( ImGui::GetCursorPosX() + Padding );
	ImGui::SetNextItemWidth( -Padding ); 
	}


bool BeginProperties( const char* Name )
	{
	if ( ImGui::BeginTable( Name, 2, ImGuiTableFlags_Resizable ) )
		{
		ImGui::TableSetupColumn( "Label", ImGuiTableColumnFlags_WidthFixed, 160.0f );
		ImGui::TableSetupColumn( "Value", ImGuiTableColumnFlags_WidthStretch );
		
		return true;
		}

	return false;
	}


void EndProperties()
	{
	ImGui::EndTable();
	}


bool BeginSection( const char* Label )
	{
	ImGui::EndTable();
	ImGui::PushFont( IMGUI_FONT_REGULAR_BOLD, 0.0f );
	bool Open = ImGui::CollapsingHeader( Label, ImGuiTreeNodeFlags_DefaultOpen );
	ImGui::PopFont();
	ImGui::BeginProperties( "##Properties" );
	
	return Open;
	}


bool Property( const char* Label, bool& Value, const char* Tooltip )
	{
	ImGui::PushID( Label );
	ImGui::BeginProperty( Label, Tooltip );
	bool Changed = ImGui::Checkbox( "##Value", &Value );
	ImGui::PopID();

	return Changed;
	}
}