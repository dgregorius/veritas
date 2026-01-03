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
		FreeLibrary( (HMODULE)hModule );
		}
	}


//--------------------------------------------------------------------------------------------------
// VsPluginInstance
//--------------------------------------------------------------------------------------------------
VsPluginInstance::VsPluginInstance( const fs::path& PluginPath )
	{
	SetDllDirectoryW( PluginPath.parent_path().c_str() );
	HMODULE hModule = LoadLibraryW( PluginPath.c_str() );
	SetDllDirectory( NULL );

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
const IVsPlugin* VsPluginInstance::Get() const
	{
	return mPlugin.get();
	}


//--------------------------------------------------------------------------------------------------
IVsPlugin* VsPluginInstance::Get()
	{
	return mPlugin.get();
	}


//--------------------------------------------------------------------------------------------------
const IVsPlugin* VsPluginInstance::operator->() const
	{
	return mPlugin.get();
	}


//--------------------------------------------------------------------------------------------------
IVsPlugin* VsPluginInstance::operator->()
	{
	return mPlugin.get();
	}


//--------------------------------------------------------------------------------------------------
IVsPlugin& VsPluginInstance::operator*()
	{
	return *mPlugin;
	}


//--------------------------------------------------------------------------------------------------
VsPluginInstance::operator bool() const
	{
	return mPlugin != nullptr;
	}
