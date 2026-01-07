//--------------------------------------------------------------------------------------------------
/*
	@file		inspectorwindow.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "veritas_testlab/window.h"


//--------------------------------------------------------------------------------------------------
// VsInspectorWindow
//--------------------------------------------------------------------------------------------------
class VsInspectorWindow : public VsWindow
	{
	public:
        // Construction / Destruction
		VsInspectorWindow( const char* Name, bool Visible = true );
		virtual ~VsInspectorWindow() = default;

		// Render
		virtual void Render( VsSession* Session ) override;

	private:

	};