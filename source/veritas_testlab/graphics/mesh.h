//--------------------------------------------------------------------------------------------------
/**
	@file		mesh.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

struct IVsShape;


//--------------------------------------------------------------------------------------------------
// VsMesh
//--------------------------------------------------------------------------------------------------
struct VsMesh
	{

	};


// Physics -> Graphics bridge
VsMesh* vsCreateMesh( IVsShape* Shape );
void vsDestroyMesh( VsMesh* Mesh );


