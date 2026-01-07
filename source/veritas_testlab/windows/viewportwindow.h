//--------------------------------------------------------------------------------------------------
/*
	@file		viewportwindow.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "veritas_testlab/window.h"

#include <veritas_renderer/camera.h>
#include <veritas_renderer/shader.h>
#include <veritas_renderer/shaderlibrary.h>
#include <veritas_renderer/rendertarget.h>
#include <veritas_renderer/vertex.h>


//--------------------------------------------------------------------------------------------------
// VsViewportWindow
//--------------------------------------------------------------------------------------------------
class VsViewportWindow : public VsWindow
	{
	public:
		// Construction / Destruction
		VsViewportWindow( const char* Name, bool Visible = true );
		virtual ~VsViewportWindow();

		// Render
		virtual void Render( VsSession* Session ) override;

	private:
		VsCamera* mCamera = nullptr;
		VsRenderTarget* mRenderTarget = nullptr;
	};