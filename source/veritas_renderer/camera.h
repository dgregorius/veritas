//--------------------------------------------------------------------------------------------------
/**
	@file		camera.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

// Veritas
#include <veritas//veritas.h>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>


//--------------------------------------------------------------------------------------------------
// VsCamera
//--------------------------------------------------------------------------------------------------
class VsCamera
	{
	public:
		// View
		glm::vec3 GetPosition() const;
		glm::quat GetOrientation() const;
		glm::mat4 GetTransform() const;
		glm::mat4 GetViewMatrix() const;

		glm::vec3 GetRight() const;
		glm::vec3 GetUp() const;
		glm::vec3 GetForward() const;

		// Projection
		float GetFOV() const;
		void SetFOV( float FOV );
		void SetAspectRatio( float AspectRatio );
		float GetAspectRatio() const;
		float GetNearPlane() const;
		void SetNearPlane( float NearPlane );
		float GetFarPlane() const;
		void SetFarPlane( float FarPlane );
		glm::mat4 GetProjectionMatrix() const;

		// Control
		void Update();

		// Synchronization
		static uint32_t CreateBuffer();
		static void DestroyBuffer( uint32_t UBO );
		void Upload( uint32_t UBO ) const;
		
	private:
		struct GPUData
			{
			glm::mat4 ViewMatrix;
			glm::mat4 ProjectionMatrix;
			glm::vec4 Position;
			};

		float mYaw = 45.0f;
		float mPitch = -25.0f;
		float mRadius = 15.0f;
		glm::vec3 mTarget = { 0.0f, 0.0f, 0.0f };

		float mFOV = 50.0f;
		float mAspectRatio = 0.0f;
		float mNearPlane = 0.03f;
		float mFarPlane = 1000.0f;
	};