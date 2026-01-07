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

#include "application.h"

#include <algorithm>
#include <vector>

class VsSession;
class VsWindow;


//--------------------------------------------------------------------------------------------------
// VsTestlab
//--------------------------------------------------------------------------------------------------
class VsTestlab : public VsApplication
	{
	public:
		virtual void Startup() override;
		virtual void BeginFrame() override;
		virtual void UpdateFrame() override;
		virtual void RenderFrame() override;
		virtual void EndFrame() override;
		virtual void Shutdown() override;

	private:
		void BeginDockspace();
		void EndDockspace();
		void Status();

		VsSession* mSession = nullptr;
		std::vector< VsWindow* > mDockWindows;
		bool mResetLayout = false;
	};