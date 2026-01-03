//--------------------------------------------------------------------------------------------------
// plugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "plugin.h"

// Box3D
#include <box3d/box3d.h>


//--------------------------------------------------------------------------------------------------
// VsBox3Plugin
//--------------------------------------------------------------------------------------------------
VsBox3Plugin::~VsBox3Plugin()
	{
	
	}


//--------------------------------------------------------------------------------------------------
const char* VsBox3Plugin::GetName() const
	{
	return "Box3D";
	}


//--------------------------------------------------------------------------------------------------
const char* VsBox3Plugin::GetVersion() const
	{
	return "0.1.1";
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsBox3Plugin::CreateHull( int VertexCount, const VsVector3* Vertices )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsBox3Plugin::CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsBox3Plugin::CreateWorld( const VsWorldDef& WorldDef )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsBox3Plugin::DestroyWorld( IVsWorld* World )
	{

	}


// Export
VS_EXPORT_PLUGIN( VsBox3Plugin );
