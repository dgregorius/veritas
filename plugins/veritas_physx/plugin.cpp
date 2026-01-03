//--------------------------------------------------------------------------------------------------
// plugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "plugin.h"

// PhysX
#include <PxPhysicsAPI.h>

// CRT's memory leak detection
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#if defined( DEBUG ) || defined( _DEBUG )
#define CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#endif


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


//--------------------------------------------------------------------------------------------------
// Every saga begins with a first step...
//--------------------------------------------------------------------------------------------------
BOOL APIENTRY DllMain( HMODULE hModule, DWORD nReason, LPVOID )
	{
	switch ( nReason )
		{
		case DLL_PROCESS_ATTACH:
#ifdef _DEBUG
			// Setup the leak report flags
			_CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
			_CrtSetReportMode( _CRT_WARN, _CRTDBG_MODE_DEBUG );
#endif
			break;

		case DLL_PROCESS_DETACH:
#ifdef _DEBUG
			// This is where the leak report is actually printed 
			// to the Output window when the plugin is unloaded.
			_CrtDumpMemoryLeaks();
#endif
			break;
		}

	return TRUE;
	}


VS_EXPORT_PLUGIN( VsPhysXPlugin );
