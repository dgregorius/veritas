//--------------------------------------------------------------------------------------------------
// plugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "plugin.h"

// PhysX
#include <PxPhysicsAPI.h>


//--------------------------------------------------------------------------------------------------
// VsPhysXPlugin
//--------------------------------------------------------------------------------------------------
VsPhysXPlugin::~VsPhysXPlugin()
	{

	}


//--------------------------------------------------------------------------------------------------
const char* VsPhysXPlugin::GetName() const
	{
	return "PhysX";
	}


//--------------------------------------------------------------------------------------------------
const char* VsPhysXPlugin::GetVersion() const
	{
	return "5.6.1";
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsPhysXPlugin::CreateHull( int VertexCount, const VsVector3* Vertices )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsPhysXPlugin::CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsPhysXPlugin::CreateWorld( const VsWorldDef& WorldDef )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXPlugin::DestroyWorld( IVsWorld* World )
	{

	}
