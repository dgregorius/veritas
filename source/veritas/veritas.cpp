//--------------------------------------------------------------------------------------------------
// veritas.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "veritas.h"

// Windows
#define WIN32_LEAN_AND_MEAN
#include <windows.h>


//--------------------------------------------------------------------------------------------------
// IVsBody
//--------------------------------------------------------------------------------------------------
void IVsBody::DeleteShape( IVsShape* Shape )
	{
	delete Shape;
	}


//--------------------------------------------------------------------------------------------------
// IVsPlugin
//--------------------------------------------------------------------------------------------------
IVsHull* IVsPlugin::CreateBox( VsVector3 Extent )
	{
	int VertexCount = 8;
	VsVector3 Vertices[] = 
		{
		VsVector3(  Extent.X,  Extent.Y,  Extent.Z ),
		VsVector3( -Extent.X,  Extent.Y,  Extent.Z ),
		VsVector3( -Extent.X, -Extent.Y,  Extent.Z ),
		VsVector3(  Extent.X, -Extent.Y,  Extent.Z ),
		VsVector3(  Extent.X,  Extent.Y, -Extent.Z ),
		VsVector3( -Extent.X,  Extent.Y, -Extent.Z ),
		VsVector3( -Extent.X, -Extent.Y, -Extent.Z ),
		VsVector3(  Extent.X, -Extent.Y, -Extent.Z )
		};

	return CreateHull( VertexCount, Vertices );
	}