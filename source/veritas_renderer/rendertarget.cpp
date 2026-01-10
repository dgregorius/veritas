//--------------------------------------------------------------------------------------------------
// rendertarget.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "rendertarget.h"

// OpenGL
#include <glad.h>
#include <glfw3.h>


//--------------------------------------------------------------------------------------------------
// VsRenderTarget
//--------------------------------------------------------------------------------------------------
VsRenderTarget::VsRenderTarget( int Samples )
	: mSamples( Samples )
	{

	}


//--------------------------------------------------------------------------------------------------
VsRenderTarget::VsRenderTarget( int Width, int Height, int Samples )
	: mWidth( Width )
	, mHeight( Height )
	, mSamples( Samples )
	{
	if ( Width > 0 && Height > 0 )
		{
		Create();
		}
	}


//--------------------------------------------------------------------------------------------------
VsRenderTarget::~VsRenderTarget()
	{
	Destroy();
	}


//--------------------------------------------------------------------------------------------------
void VsRenderTarget::Bind()
	{
	if ( !mMSFramebuffer )
		{
		return;
		}

	glEnable( GL_MULTISAMPLE );
	glBindFramebuffer( GL_FRAMEBUFFER, mMSFramebuffer );
	glViewport( 0, 0, mWidth, mHeight );
	}


//--------------------------------------------------------------------------------------------------
void VsRenderTarget::Unbind()
	{
	if ( !mMSFramebuffer )
		{
		return;
		}

	glBlitNamedFramebuffer( mMSFramebuffer, mFramebuffer,0, 0, mWidth, mHeight,0, 0, mWidth, mHeight, GL_COLOR_BUFFER_BIT, GL_NEAREST );
	glBindFramebuffer( GL_FRAMEBUFFER, GL_NONE );
	glDisable( GL_MULTISAMPLE );
	}

//--------------------------------------------------------------------------------------------------
void VsRenderTarget::Clear( float R, float G, float B, float A )
	{
	if ( !mMSFramebuffer )
		{
		return;
		}

	// Clear Color Buffer (Attachment 0)
	float Color[] = { R, G, B, A };
	glClearNamedFramebufferfv( mMSFramebuffer, GL_COLOR, 0, Color );

	// Clear Depth Buffer
	float Depth = 1.0f;
	glClearNamedFramebufferfv( mMSFramebuffer, GL_DEPTH, 0, &Depth );
	}


//--------------------------------------------------------------------------------------------------
int VsRenderTarget::GetWidth() const
	{
	return mWidth;
	}


//--------------------------------------------------------------------------------------------------
int VsRenderTarget::GetHeight() const
	{
	return mHeight;
	}


//--------------------------------------------------------------------------------------------------
void VsRenderTarget::GetSize( int& Width, int& Height )
	{
	Width = mWidth;
	Height = mHeight;
	}


//--------------------------------------------------------------------------------------------------
void VsRenderTarget::Resize( int Width, int Height )
	{
	if ( Width <= 0 || Height <= 0 )
		{
		return;
		}

	if ( Width == mWidth && Height == mHeight )
		{
		return;
		}
		
	mWidth = Width;
	mHeight = Height;

	Destroy();
	Create();
	}


//--------------------------------------------------------------------------------------------------
uint32_t VsRenderTarget::GetTexture() const
	{
	return mColorAttachment;
	}


//--------------------------------------------------------------------------------------------------
void VsRenderTarget::Create()
	{
	// Multi-sampled framebuffer
	glCreateFramebuffers( 1, &mMSFramebuffer );
	glCreateTextures( GL_TEXTURE_2D_MULTISAMPLE, 1, &mMSColorAttachment );
	glTextureStorage2DMultisample( mMSColorAttachment, mSamples, GL_RGBA8, mWidth, mHeight, GL_TRUE );
	glNamedFramebufferTexture( mMSFramebuffer, GL_COLOR_ATTACHMENT0, mMSColorAttachment, 0 );
	glCreateTextures( GL_TEXTURE_2D_MULTISAMPLE, 1, &mMSDepthAttachment );
	glTextureStorage2DMultisample( mMSDepthAttachment, mSamples, GL_DEPTH24_STENCIL8, mWidth, mHeight, GL_TRUE );
	glNamedFramebufferTexture( mMSFramebuffer, GL_DEPTH_STENCIL_ATTACHMENT, mMSDepthAttachment, 0 );
	VS_ASSERT( glCheckNamedFramebufferStatus( mMSFramebuffer, GL_FRAMEBUFFER ) == GL_FRAMEBUFFER_COMPLETE );

	// Resolve buffer
	glCreateFramebuffers( 1, &mFramebuffer );
	glCreateTextures( GL_TEXTURE_2D, 1, &mColorAttachment );
	glTextureStorage2D( mColorAttachment, 1, GL_RGBA8, mWidth, mHeight );
	glTextureParameteri( mColorAttachment, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTextureParameteri( mColorAttachment, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTextureParameteri( mColorAttachment, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
	glTextureParameteri( mColorAttachment, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
	glNamedFramebufferTexture( mFramebuffer, GL_COLOR_ATTACHMENT0, mColorAttachment, 0 );
	VS_ASSERT( glCheckNamedFramebufferStatus( mFramebuffer, GL_FRAMEBUFFER ) == GL_FRAMEBUFFER_COMPLETE );
	}


//--------------------------------------------------------------------------------------------------
void VsRenderTarget::Destroy()
	{
	glDeleteFramebuffers( 1, &mMSFramebuffer );
	glDeleteTextures( 1, &mMSColorAttachment );
	glDeleteRenderbuffers( 1, &mMSDepthAttachment );
	mMSFramebuffer = mMSColorAttachment = mMSDepthAttachment = 0;

	glDeleteFramebuffers( 1, &mFramebuffer );
	glDeleteTextures( 1, &mColorAttachment );
	mFramebuffer = mColorAttachment = 0;
	}
