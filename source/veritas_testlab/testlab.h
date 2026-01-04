//--------------------------------------------------------------------------------------------------
/**
	@file		testlab.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
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
		void EndFrame();
		void Shutdown();

	private:
		void BeginDockspace();
		void Explorer();
		void Viewport();
		void Profiler();
		void EndDockspace();
		void Shortcuts();

		GLFWwindow* mWindow = nullptr;
		bool mShowProfiler = false;

		std::vector< VsModule* > mModules;
		std::vector< VsTest* > mTests;
		int mTestIndex = 0;
	};