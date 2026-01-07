//--------------------------------------------------------------------------------------------------
/*
	@file		profilewindow.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "veritas_testlab/window.h"


//--------------------------------------------------------------------------------------------------
// VsProfileWindow
//--------------------------------------------------------------------------------------------------
class VsProfileWindow : public VsWindow
	{
	public:
		// Construction / Destruction
		VsProfileWindow( const char* Name, bool Visible = true );
		virtual ~VsProfileWindow() = default;

		// Render
		virtual void Render( VsSession* Session ) override;

	private:

	};