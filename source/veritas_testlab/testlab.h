//--------------------------------------------------------------------------------------------------
/**
	@file		testlab.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "test.h"

struct GLFWwindow;


//--------------------------------------------------------------------------------------------------
// VsTestlab
//--------------------------------------------------------------------------------------------------
class VsTestlab 
	{
	public:
		// Main loop
		int Run();

		void Startup();
		void BeginFrame();
		void UpdateFrame();
		void RenderFrame();
		void EndFrame();
		void Shutdown();

	private:
		// Viewport
		void RenderGradient();
		void RenderGrid();
		void RenderTests();

		// UI
		void BeginDockspace();
		void DrawInspector();
		void DrawOutliner();
		void DrawProfiler();
		void DrawViewport();
		void EndDockspace();
		void Status();

		// Tests
		void CreateTests( int TestIndex );
		void DestroyTests();

		GLFWwindow* mWindow = nullptr;

		VsCamera* mCamera = nullptr;
		VsRenderTarget* mRenderTarget = nullptr;

		int mTestIndex = -1;
		using VsPluginPtr = std::unique_ptr< IVsPlugin, std::function< void ( IVsPlugin* ) > >;
		std::vector< VsPluginPtr > mPlugins;
		std::vector< VsTest* > mTests;
	};