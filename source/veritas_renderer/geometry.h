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
		 VsGeometry( const std::vector< int >& Indices, const std::vector< VsMeshVertex >& Vertices );
		~VsGeometry();

		// Rendering
		void Render( int InstanceCount );

	private:
		int mElementCount = 0;
		uint32_t mElementBuffer = 0;
		int mVertexCount = 0;
		uint32_t mVertexBuffer = 0;
	};


// Physics -> Graphics bridge
VsGeometry* vsCreateGeometry( IVsShape* Shape );


