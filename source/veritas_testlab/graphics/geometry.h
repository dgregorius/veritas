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

// Veritas
#include <veritas//veritas.h>


//--------------------------------------------------------------------------------------------------
// VsGeometry
//--------------------------------------------------------------------------------------------------
struct VsGeometry
	{

	};


// Physics -> Graphics bridge
VsGeometry* vsCreateGeometry( IVsShape* Shape );


