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

#include <algorithm>
#include <vector>

struct GLFWwindow;


//--------------------------------------------------------------------------------------------------
// TlTestLab
//--------------------------------------------------------------------------------------------------
class TlTestLab
	{
	public:
		int Run();

	private:
		void Startup();
		void BeginFrame();
		void UpdateFrame();
		void EndFrame();
		void Shutdown();

		GLFWwindow* mWindow = nullptr;

		int mTestIndex = -1;
		TlTest* mTest = nullptr;
		bool mSingleStep = false;
		bool mShowProfiler = false;
	};