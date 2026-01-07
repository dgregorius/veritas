//--------------------------------------------------------------------------------------------------
/*
	@file		outlinerwindow.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "veritas_testlab/window.h"


//--------------------------------------------------------------------------------------------------
// VsOutlinerWindow
//--------------------------------------------------------------------------------------------------
class VsOutlinerWindow : public VsWindow
	{
	public:
		// Construction / Destruction
		VsOutlinerWindow( const char* Name, bool Visible = true );
		virtual ~VsOutlinerWindow() = default;

		// Render
		virtual void Update( VsSession* Session ) override;
		virtual void Render( VsSession* Session ) override;

	private:

	};