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

	private:
		void Startup();
		void BeginFrame();
		void UpdateFrame();
		void EndFrame();
		void Shutdown();

		GLFWwindow* mWindow = nullptr;

		std::vector< VsPluginInstance > mPlugins;
		std::vector< VsTest* > mTests;
		int mTestIndex = 0;
	};