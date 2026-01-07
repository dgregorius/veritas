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
#include "test.h"


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
		void RenderInspector();
		void RenderOutliner();
		void RenderProfiler();
		void RenderViewport();
		void EndDockspace();
		void Status();

		VsCamera* mCamera = nullptr;
		VsRenderTarget* mRenderTarget = nullptr;

		std::vector< VsModule* > mModules;
		std::vector< VsTest* > mTests;
		int mTestIndex = 0;
	};