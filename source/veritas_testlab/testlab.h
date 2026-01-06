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
		int Run();

		void Startup();
		void BeginFrame();
		void UpdateFrame();
		void RenderFrame();
		void EndFrame();
		void Shutdown();

	private:
		void BeginDockspace();
		void Explorer();
		void Viewport();
		void Profiler();
		void EndDockspace();
		void Shortcuts();

		void RenderBackground();
		void RenderTests();

		GLFWwindow* mWindow = nullptr;
		bool mShowProfiler = false;

		VsCamera* mCamera = nullptr;
		VsRenderTarget* mRenderTarget = nullptr;

		std::vector< VsModule* > mModules;
		std::vector< VsTest* > mTests;
		int mTestIndex = 0;
	};