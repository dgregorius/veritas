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
// VsModule
//--------------------------------------------------------------------------------------------------
struct VsModule
	{
	HMODULE Handle = nullptr;
	IVsPlugin* Plugin = nullptr;
	VsDestroyPluginFunc DestroyFunc = nullptr;
	};


//--------------------------------------------------------------------------------------------------
VsModule* vsLoadModule( const fs::path& Path )
	{
	SetDllDirectoryW( Path.parent_path().c_str() );
	HMODULE hModule = LoadLibraryW( Path.c_str() );
	SetDllDirectory( NULL );

	if ( !hModule )
		{
		return nullptr;
		}

	VsCreatePluginFunc CreateFunc = (VsCreatePluginFunc)GetProcAddress( hModule, "vsCreatePlugin" );
	VsDestroyPluginFunc DestroyFunc = (VsDestroyPluginFunc)GetProcAddress( hModule, "vsDestroyPlugin" );
	if ( !CreateFunc || !DestroyFunc )
		{
		FreeLibrary( hModule );
		return nullptr;
		}

	IVsPlugin* Plugin = CreateFunc();
	if ( !Plugin )
		{
		FreeLibrary( hModule );
		return nullptr;
		}

	return new VsModule { hModule, Plugin, DestroyFunc };
	}


//--------------------------------------------------------------------------------------------------
IVsPlugin* vsGetPlugin( VsModule* Module )
	{
	return Module ? Module->Plugin : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void vsFreeModule( VsModule* Module )
	{
	if ( !Module )
		{
		return;
		}
	
	if ( Module->Plugin && Module->DestroyFunc )
		{
		Module->DestroyFunc( Module->Plugin );

		Module->Plugin = nullptr;
		Module->DestroyFunc = nullptr;
		}
	
	if ( Module->Handle )
		{
		FreeLibrary( Module->Handle );
		Module->Handle = nullptr;
		}

	delete Module;
	}