
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
	}


