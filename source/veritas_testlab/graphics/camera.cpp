//--------------------------------------------------------------------------------------------------
// camera.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "camera.h"

// ImGUI
#include <imgui.h>
#include <imgui_internal.h>


//--------------------------------------------------------------------------------------------------
// VsCamera
//--------------------------------------------------------------------------------------------------
glm::vec3 VsCamera::GetPosition() const
	{
	return mTarget - mRadius * GetForward();
	}


//--------------------------------------------------------------------------------------------------
glm::quat VsCamera::GetOrientation() const
	{
	float Theta = glm::radians( mYaw );
	float Phi = glm::radians( mPitch );

	glm::quat RotationY = glm::angleAxis( Theta, glm::vec3( 0.0f, 1.0f, 0.0f ) );
	glm::quat RotationX = glm::angleAxis( Phi, glm::vec3( 1.0f, 0.0f, 0.0f ) );
	
	return RotationY * RotationX;
	}


//--------------------------------------------------------------------------------------------------
glm::mat4 VsCamera::GetTransform() const
	{
	float Theta = glm::radians( mYaw );
	float Phi = glm::radians( mPitch );

	glm::mat4 Rotation( 1.0f );
	Rotation = glm::rotate( Rotation, Theta, glm::vec3( 0.0f, 1.0f, 0.0f ) );
	Rotation = glm::rotate( Rotation, Phi, glm::vec3( 1.0f, 0.0f, 0.0f ) );

	glm::mat4 Translation( 1.0f );
	Translation = glm::translate( Translation, GetPosition() );
	
	return Translation * Rotation;
	}


//--------------------------------------------------------------------------------------------------
glm::mat4 VsCamera::GetViewMatrix() const
	{
	glm::mat4 Transform = GetTransform();
	return glm::inverse( Transform );
	}


//--------------------------------------------------------------------------------------------------
glm::vec3 VsCamera::GetRight() const
	{
	return GetOrientation() * glm::vec3( 1.0f, 0.0f, 0.0f );
	}


//--------------------------------------------------------------------------------------------------
glm::vec3 VsCamera::GetUp() const
	{
	return GetOrientation() * glm::vec3( 0.0f, 1.0f, 0.0f );
	}


//--------------------------------------------------------------------------------------------------
glm::vec3 VsCamera::GetForward() const
	{
	return GetOrientation() * glm::vec3( 0.0f, 0.0f, -1.0f );
	}


//--------------------------------------------------------------------------------------------------
float VsCamera::GetFOV() const
	{
	return mFOV;
	}


//--------------------------------------------------------------------------------------------------
void VsCamera::SetFOV( float FOV )
	{
	VS_ASSERT( 0.0f < FOV && FOV <= 90.0f );
	mFOV = FOV;
	}


//--------------------------------------------------------------------------------------------------
float VsCamera::GetNearPlane() const
	{
	return mNearPlane;
	}


//--------------------------------------------------------------------------------------------------
void VsCamera::SetNearPlane( float NearPlane )
	{
	VS_ASSERT( NearPlane > 0.0f );
	mNearPlane = NearPlane;
	}


//--------------------------------------------------------------------------------------------------
float VsCamera::GetFarPlane() const
	{
	return mFarPlane;
	}


//--------------------------------------------------------------------------------------------------
void VsCamera::SetFarPlane( float FarPlane )
	{
	VS_ASSERT( mFarPlane > 0.0f );
	mFarPlane = FarPlane;
	}


//--------------------------------------------------------------------------------------------------
glm::mat4 VsCamera::GetProjectionMatrix( int Width, int Height ) const
	{
	float Aspect = static_cast< float >( Width ) / static_cast< float >( std::max( 1, Height ) );
	return glm::perspective( glm::radians( mFOV ), Aspect, mNearPlane, mFarPlane );
	}


//--------------------------------------------------------------------------------------------------
void VsCamera::Update()
	{
	// 1. Check if the mouse is over an ImGui window
	ImGuiIO& IO = ImGui::GetIO();
	if ( IO.WantCaptureMouse )
		{
		return;
		}

	// 2. Handle Maya-style "Alt" Navigation
	if ( ImGui::IsKeyDown( ImGuiKey_LeftAlt ) || ImGui::IsKeyDown( ImGuiKey_RightAlt ) )
		{
		// ORBIT (Alt + Left Click)
		if ( ImGui::IsMouseDown( ImGuiMouseButton_Left ) )
			{
			const float Sensitivity = 0.25f;
			mYaw += IO.MouseDelta.x * Sensitivity;
			mPitch -= IO.MouseDelta.y * Sensitivity;
			mPitch = glm::clamp( mPitch, -89.0f, 89.0f );
			}

		// PAN (Alt + Middle Click)
		if ( ImGui::IsMouseDown( ImGuiMouseButton_Middle ) )
			{
			const float Sensitivity = 0.001f;
			float PanSpeed = mRadius * Sensitivity;

			mTarget -= GetRight() * ( IO.MouseDelta.x * PanSpeed );
			mTarget += GetUp() * ( IO.MouseDelta.y * PanSpeed );
			}

		// DOLLY/ZOOM (Alt + Right Click)
		if ( ImGui::IsMouseDown( ImGuiMouseButton_Right ) )
			{
			// Horizontal mouse movement for zooming
			const float Sensitivity = 0.01f;
			float ZoomSpeed = mRadius * Sensitivity;

			mRadius += IO.MouseDelta.x * ZoomSpeed;
			mRadius = glm::max( mRadius, 0.03f );
			}
		}

	// 3. Handle Scroll Wheel Zoom (standard outside of Alt mode)
	if ( IO.MouseWheel != 0.0f )
		{
		const float Sensitivity = 0.1f;
		float ScrollSpeed = mRadius * Sensitivity;

		mRadius -= IO.MouseWheel * ScrollSpeed;
		mRadius = glm::max( mRadius, 0.03f );
		}
	}
