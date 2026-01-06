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

struct VsGeometry;


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

	private:
		VsGeometry* mGeometry = nullptr;
	};


