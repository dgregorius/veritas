
// dear imgui: imgui_user.h
#pragma  once

struct GLFWwindow;


//--------------------------------------------------------------------------------------------------
// Constants
//--------------------------------------------------------------------------------------------------
extern float IMGUI_UI_SCALE;

extern float IMGUI_FONT_SIZE;
extern ImFont* IMGUI_FONT_REGULAR;
extern ImFont* IMGUI_FONT_REGULAR_MEDIUM;
extern ImFont* IMGUI_FONT_REGULAR_BOLD;
extern ImFont* IMGUI_FONT_SMALL;

// Base palette - Godot
// static inline const ImVec4 IMGUI_COLOR_BACKGROUND = ImVec4( 0.14f, 0.13f, 0.12f, 1.0f );  // main background
// static inline const ImVec4 IMGUI_COLOR_BACKGROUND_DARK = ImVec4( 0.10f, 0.09f, 0.08f, 1.0f );  // recessed
// static inline const ImVec4 IMGUI_COLOR_BACKGROUND_LIGHT = ImVec4( 0.20f, 0.19f, 0.18f, 1.0f );  // raised/hover
// static inline const ImVec4 IMGUI_COLOR_ACCENT = ImVec4( 0.33f, 0.52f, 0.76f, 1.0f );  // blue accent
// static inline const ImVec4 IMGUI_COLOR_ACCENT_DIM = ImVec4( 0.24f, 0.38f, 0.55f, 1.0f );  // softer blue
// static inline const ImVec4 IMGUI_COLOR_TEXT = ImVec4( 0.90f, 0.90f, 0.88f, 1.0f );  // warm white
// static inline const ImVec4 IMGUI_COLOR_TEXT_DIM = ImVec4( 0.50f, 0.48f, 0.46f, 1.0f );  // warm gray

// Base palette — VS 2022 Dark
static inline const ImVec4 IMGUI_COLOR_BACKGROUND = ImVec4( 0.18f, 0.18f, 0.18f, 1.0f );  // main background
static inline const ImVec4 IMGUI_COLOR_BACKGROUND_DARK = ImVec4( 0.10f, 0.10f, 0.10f, 1.0f );  // recessed
static inline const ImVec4 IMGUI_COLOR_BACKGROUND_LIGHT = ImVec4( 0.24f, 0.24f, 0.24f, 1.0f );  // raised/hover
static inline const ImVec4 IMGUI_COLOR_ACCENT = ImVec4( 0.35f, 0.55f, 0.85f, 1.0f );  // blue accent (brighter)
static inline const ImVec4 IMGUI_COLOR_ACCENT_DIM = ImVec4( 0.24f, 0.40f, 0.64f, 1.0f );  // softer blue (scaled up)
static inline const ImVec4 IMGUI_COLOR_TEXT = ImVec4( 0.92f, 0.92f, 0.92f, 1.0f );  // neutral white
static inline const ImVec4 IMGUI_COLOR_TEXT_DIM = ImVec4( 0.50f, 0.50f, 0.50f, 1.0f );  // neutral gray


//--------------------------------------------------------------------------------------------------
// User types
//--------------------------------------------------------------------------------------------------
struct ImScrollingBuffer
	{
	int MaxSize;
	int Offset;
	ImVector<ImVec2> Data;

	ImScrollingBuffer( int max_size = 4096 )
		{
		MaxSize = max_size;
		Offset = 0;
		Data.reserve( MaxSize );
		}

	bool Empty() const
		{
		return Data.empty();
		}

	void AddPoint( float x, float y )
		{
		if ( Data.size() < MaxSize )
			Data.push_back( ImVec2( x, y ) );
		else
			{
			Data[ Offset ] = ImVec2( x, y );
			Offset = ( Offset + 1 ) % MaxSize;
			}
		}

	void Erase()
		{
		if ( Data.size() > 0 )
			{
			Data.shrink( 0 );
			Offset = 0;
			}
		}
	};
		

//--------------------------------------------------------------------------------------------------
// Helper functions
//--------------------------------------------------------------------------------------------------
namespace ImGui
	{
	// Runtime helpers
	void Startup( GLFWwindow* Window );
	void BeginFrame( GLFWwindow* Window );
	void EndFrame( GLFWwindow* Window );
	void Shutdown( GLFWwindow* Window );

	// Simple property helpers
	bool BeginProperties( const char* Name );
	void EndProperties();

	bool BeginSection( const char* Label );
	bool Property( const char* Label, bool& Value, const char* Tooltip = nullptr );
 	bool Property( const char* Label, int& Value, int Min, int Max, const char* Tooltip = nullptr );
// 	bool Property( const char* Label, float& Value, const char* Tooltip = nullptr );
	}


