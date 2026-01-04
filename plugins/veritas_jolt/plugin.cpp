//--------------------------------------------------------------------------------------------------
// plugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "plugin.h"

// Jolt
#include <Jolt/Jolt.h>


//--------------------------------------------------------------------------------------------------
// VsJoltPlugin
//--------------------------------------------------------------------------------------------------
VsJoltPlugin::~VsJoltPlugin()
	{
	
	}


//--------------------------------------------------------------------------------------------------
const char* VsJoltPlugin::GetName() const
	{
	return "Jolt";
	}


//--------------------------------------------------------------------------------------------------
const char* VsJoltPlugin::GetVersion() const
	{
	return "5.2.0";
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsJoltPlugin::CreateHull( int VertexCount, const VsVector3* Vertices )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltPlugin::DestroyHull( IVsHull* Hull )
	{

	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsJoltPlugin::CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltPlugin::DestroyMesh( IVsMesh* Mesh )
	{

	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsJoltPlugin::CreateWorld()
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltPlugin::DestroyWorld( IVsWorld* World )
	{

	}


// Export
VS_EXPORT_PLUGIN( VsJoltPlugin );