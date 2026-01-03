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
// VsPluginModule
//--------------------------------------------------------------------------------------------------
static void vsFreeLibrary( void* hModule )
	{
	if ( hModule )
		{
		//FreeLibrary( (HMODULE)hModule );
		}
	}


//--------------------------------------------------------------------------------------------------
// VsPluginModule
//--------------------------------------------------------------------------------------------------
VsPluginInstance::VsPluginInstance( void* hModule )
	{
	if ( hModule )
		{
		VsCreatePluginFunc vsCreatePlugin = (VsCreatePluginFunc)GetProcAddress( (HMODULE)hModule, "vsCreatePlugin" );
		VsDestroyPluginFunc vsDestroyPlugin = (VsDestroyPluginFunc)GetProcAddress( (HMODULE)hModule, "vsDestroyPlugin" );
		if ( vsCreatePlugin && vsDestroyPlugin )
			{
			mModule = ModuleHandle( hModule, vsFreeLibrary );
			mPlugin = PluginHandle( vsCreatePlugin(), vsDestroyPlugin );
			}
		}
	}


//--------------------------------------------------------------------------------------------------
VsPluginInstance vsLoadPlugin( const fs::path& PluginPath )
	{
	// Add plugin directory to DLL search path
	SetDllDirectoryW( PluginPath.parent_path().c_str() );
	HMODULE hModule = LoadLibraryW( PluginPath.c_str() );
	SetDllDirectory( NULL );

	return VsPluginInstance( hModule );
	}
