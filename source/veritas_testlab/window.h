//--------------------------------------------------------------------------------------------------
/*
	@file		window.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

class VsSession;


//--------------------------------------------------------------------------------------------------
// VsWindow
//--------------------------------------------------------------------------------------------------
class VsWindow
	{
	public:
        // Construction / Destruction
		VsWindow( const char* Name, bool Visible );
		virtual ~VsWindow() = default;

		// Name
		const char* GetName() const;

		// Visibility 
		bool IsVisible() const;
		void SetVisible( bool Visible );

		// Render
		virtual void Render( VsSession* Session ) = 0;

	protected:
		const char* mName;
		bool mVisible;
	};