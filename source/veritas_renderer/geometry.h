//--------------------------------------------------------------------------------------------------
/**
	@file		geometry.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "vertex.h"

// Veritas
#include <veritas/veritas.h>


//--------------------------------------------------------------------------------------------------
// VsGeometry
//--------------------------------------------------------------------------------------------------
class VsGeometry
	{
	public:
		// Construction / Destruction
		explicit VsGeometry( const std::vector< VsEdgeVertex >& Vertices );
		explicit VsGeometry( const std::vector< VsMeshVertex >& Vertices );
		VsGeometry( const std::vector< int >& Indices, const std::vector< VsMeshVertex >& Vertices );
		~VsGeometry();

		// Rendering
		void Render( int InstanceCount );

	private:
		uint32_t mType = 0;
		int mElementCount = 0;
		uint32_t mElementBuffer = 0;
		int mVertexCount = 0;
		uint32_t mVertexBuffer = 0;
	};


// Physics -> Graphics bridge
VsGeometry* vsCreateMeshGeometry( IVsShape* Shape );
VsGeometry* vsCreateEdgeGeometry( IVsShape* Shape );


