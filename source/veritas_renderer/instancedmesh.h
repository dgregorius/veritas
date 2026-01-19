//--------------------------------------------------------------------------------------------------
/**
	@file		instancedmesh.h

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

class VsGeometry;
class VsShader;


//--------------------------------------------------------------------------------------------------
// VsInstanceData
//--------------------------------------------------------------------------------------------------
struct VsInstanceData
	{
	glm::mat4 Transform;
	glm::vec4 Color;
	};


//--------------------------------------------------------------------------------------------------
// VsInstancedMesh
//--------------------------------------------------------------------------------------------------
class VsInstancedMesh
	{
	public:
		// Construction / Destruction
		explicit VsInstancedMesh( VsGeometry* Geometry );
		~VsInstancedMesh();

		// Geometry
		VsGeometry* GetGeometry() const;

		// Rendering
		void Upload( const std::vector< VsInstanceData >& InstanceData );
		
		void RenderFaces( VsShader* Shader );
		void RenderEdges( VsShader* Shader );

	private:
		VsGeometry* mGeometry = nullptr;

		int mInstanceSize = 0;
		int mInstanceCapacity = 0;
		uint32_t mInstanceBuffer = 0;
	};


