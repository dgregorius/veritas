//--------------------------------------------------------------------------------------------------
/**
	@file		application.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

struct GLFWwindow;


//--------------------------------------------------------------------------------------------------
// VsApplication
//--------------------------------------------------------------------------------------------------
class VsApplication
	{
	public:
		// Construction / Destruction
		VsApplication() = default;
		virtual ~VsApplication() = default;

		// Main loop
		int Run();
		virtual void Startup() = 0;
		virtual void BeginFrame() = 0;
		virtual void UpdateFrame() = 0;
		virtual void RenderFrame() = 0;
		virtual void EndFrame() = 0;
		virtual void Shutdown() = 0;

	protected:
		GLFWwindow* mWindow = nullptr;
	};