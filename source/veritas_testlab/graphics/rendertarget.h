//--------------------------------------------------------------------------------------------------
/**
	@file		rendertarget.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <veritas/veritas.h>


//--------------------------------------------------------------------------------------------------
// VsRenderTarget
//--------------------------------------------------------------------------------------------------
class VsRenderTarget
	{
	public:
		// Construction / Destruction
		VsRenderTarget( int Width, int Height, int Samples = 4 );
		~VsRenderTarget();

		// Binding
		void Bind();
		void Unbind();

		// Size
		int GetWidth() const;
		int GetHeight() const;
		void GetSize( int& Width, int& Height );
		void Resize( int Width, int Height );

		// Presentation
		uint32_t GetTexture() const;

	private:
		void Create();
		void Destroy();

		int mWidth = 0;
		int mHeight = 0;
		int mSamples = 0;

		uint32_t mMSFramebuffer = 0;
		uint32_t mMSColorAttachment = 0;
		uint32_t mMSDepthAttachment = 0;
		uint32_t mFramebuffer = 0;
		uint32_t mColorAttachment = 0;
	};