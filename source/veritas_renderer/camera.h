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
#include <veritas/veritas.h>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>


//--------------------------------------------------------------------------------------------------
// VsOrbit
//--------------------------------------------------------------------------------------------------
struct VsOrbit
	{
	float Yaw = 45.0f;
	float Pitch = -20.0f;
	float Radius = 15.0f;

	glm::vec3 Target = { 0, 0, 0 };
	};


//--------------------------------------------------------------------------------------------------
// VsCamera
//--------------------------------------------------------------------------------------------------
class VsCamera
	{
	public:
		// Comstruction / Destruction
		VsCamera();
		~VsCamera();

		// View
		float GetYaw() const;
		void SetYaw( float Yaw );
		float GetPitch() const;
		void SetPitch( float Pitch );
		float GetRadius()  const;
		void SetRadius( float Radius );
		glm::vec3 GetTarget() const;
		void SetTarget( const glm::vec3& Target );
		VsOrbit GetOrbit() const;
		void SetOrbit( const VsOrbit& Orbit );

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
		
	private:
		struct GPUData
			{
			glm::mat4 ViewMatrix;
			glm::mat4 ProjectionMatrix;
			glm::vec4 Position;
			};

		uint32_t mUBO = 0;

		float mYaw = 45.0f;
		float mPitch = -25.0f;
		float mRadius = 25.0f;
		glm::vec3 mTarget = { 0.0f, 0.0f, 0.0f };

		float mFOV = 50.0f;
		float mAspectRatio = 0.0f;
		float mNearPlane = 0.03f;
		float mFarPlane = 1000.0f;
	};