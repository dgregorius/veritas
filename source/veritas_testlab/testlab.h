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

		GLFWwindow* mWindow = nullptr;

		VsCamera* mCamera = nullptr;
		VsRenderTarget* mRenderTarget = nullptr;

		std::vector< VsModule* > mModules;
		std::vector< VsTest* > mTests;
		int mTestIndex = 0;
	};