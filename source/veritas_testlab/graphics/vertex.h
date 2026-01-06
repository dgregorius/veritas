//--------------------------------------------------------------------------------------------------
/*
	@file		vertex.h

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
// VsEmptyVertex
//--------------------------------------------------------------------------------------------------
struct VsEmptyVertex
	{
	static inline uint32_t Format = 0;
	};


//--------------------------------------------------------------------------------------------------
// VsMeshVertex
//--------------------------------------------------------------------------------------------------
struct VsMeshVertex
	{
	glm::vec3 Position;
	glm::vec3 Normal;

	static inline uint32_t Format = 0;
	};


void vsLoadVertexLibrary();
void vsUnloadVertexLibrary();
