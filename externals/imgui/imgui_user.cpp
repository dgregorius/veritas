
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
ImFont* IMGUI_FONT_REGULAR_BOLD = nullptr;
ImFont* IMGUI_FONT_SMALL = nullptr;


//--------------------------------------------------------------------------------------------------
// Helper functions
//--------------------------------------------------------------------------------------------------
namespace ImGui
{
void Startup( GLFWwindow* Window )
	{
	BOOL DarkMode = TRUE;
	HWND hWnd = glfwGetWin32Window( Window );
	DwmSetWindowAttribute( hWnd, DWMWA_USE_IMMERSIVE_DARK_MODE, &DarkMode, sizeof( DarkMode ) );

	float ScaleX, ScaleY;
	glfwGetWindowContentScale( Window, &ScaleX, &ScaleY );
	IMGUI_UI_SCALE = ImMax( ScaleX, ScaleY );

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImPlot::CreateContext();
	ImGui::StyleColorsDark();

	ImGuiIO& IO = ImGui::GetIO();
	IO.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	IO.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
	IO.ConfigDpiScaleFonts = true;
	IO.ConfigDpiScaleViewports = true;

	ImGuiStyle& Style = ImGui::GetStyle();
	Style.ScaleAllSizes( IMGUI_UI_SCALE );
	Style.FontScaleDpi = IMGUI_UI_SCALE;
	Style.FontSizeBase = 16.0f;

	if ( IO.ConfigFlags & ImGuiConfigFlags_ViewportsEnable )
		{
		Style.WindowRounding = 0.0f;
		Style.Colors[ ImGuiCol_WindowBg ].w = 1.0f;
		}

	IMGUI_FONT_SIZE = 16.0f;
	IMGUI_FONT_REGULAR = IO.Fonts->AddFontFromFileTTF( "fonts/inter_medium.ttf", IMGUI_FONT_SIZE );
	IMGUI_FONT_REGULAR_BOLD = IO.Fonts->AddFontFromFileTTF( "fonts/inter_bold.ttf", IMGUI_FONT_SIZE );
	IMGUI_FONT_SMALL = IO.Fonts->AddFontFromFileTTF( "fonts/inter_medium.ttf", 0.85f * IMGUI_FONT_SIZE );
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

	glfwSwapBuffers( Window );
	}

void Shutdown( GLFWwindow* )
	{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	
	ImGui::DestroyContext();
	ImPlot::DestroyContext();
	}
}