//--------------------------------------------------------------------------------------------------
// plugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "plugin.h"

// Box3D
#include <box3d/box3d.h>

// CRT's memory leak detection
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#if defined( DEBUG ) || defined( _DEBUG )
#define CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#endif


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


// Export
VS_EXPORT_PLUGIN( VsBox3Plugin );
