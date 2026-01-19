//--------------------------------------------------------------------------------------------------
/*
	@file		shader.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) by D. Gregorius. All rights reserved.
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
// VsShader
//--------------------------------------------------------------------------------------------------
class VsShader
	{
	public:
		// Construction / Destruction
		VsShader( const char* VertexShaderSource, const char* FragmentShaderSource );
		VsShader( const fs::path& VertexShaderPath, const fs::path& FragmentShaderPath );
		~VsShader();

		// Binding
		void Use();

		// Uniforms
		void SetUniform( const char* Name, bool Value );
		void SetUniform( const char* Name, int Value );
		void SetUniform( const char* Name, float Value );
		void SetUniform( const char* Name, const glm::vec2& Value );
		void SetUniform( const char* Name, const glm::vec3& Value );
		void SetUniform( const char* Name, const glm::vec4& Value );
		void SetUniform( const char* Name, const glm::mat2& Value );
		void SetUniform( const char* Name, const glm::mat3& Value );
		void SetUniform( const char* Name, const glm::mat4& Value );

		void SetUniformArray( const char* Name, int Count, const int* Values );
		void SetUniformArray( const char* Name, int Count, const float* Values );
		void SetUniformArray( const char* Name, int Count, const glm::vec2* Values );
		void SetUniformArray( const char* Name, int Count, const glm::vec3* Values );
		void SetUniformArray( const char* Name, int Count, const glm::vec4* Values );
		void SetUniformArray( const char* Name, int Count, const glm::mat2* Values );
		void SetUniformArray( const char* Name, int Count, const glm::mat3* Values );
		void SetUniformArray( const char* Name, int Count, const glm::mat4* Values );

		// Shader library
		static inline VsShader* GradientShader = nullptr;
		static inline VsShader* GridShader = nullptr;
		static inline VsShader* MeshShader = nullptr;
		static inline VsShader* EdgeShader = nullptr;

	private:
		uint32_t mProgram = 0;
	};

// Shader library
void vsLoadShaders(); 
void vsUnloadShaders();