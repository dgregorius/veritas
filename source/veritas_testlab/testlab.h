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
		void RenderBackground();
		void RenderGrid();
		void RenderTests();

		// UI
		void BeginDockspace();
		void RenderInspector();
		void RenderOutliner();
		void RenderProfiler();
		void RenderViewport();
		void EndDockspace();
		void Status();

		GLFWwindow* mWindow = nullptr;

		VsCamera* mCamera = nullptr;
		uint32_t mCameraBuffer = 0;
		VsRenderTarget* mRenderTarget = nullptr;

		std::vector< VsModule* > mModules;
		std::vector< VsTest* > mTests;
		int mTestIndex = 0;
	};