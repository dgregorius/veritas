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
		Module->DestroyFunc = nullptr;
		Module->Plugin = nullptr;
		}
	
	if ( Module->Handle )
		{
		FreeLibrary( Module->Handle );
		Module->Handle = nullptr;
		}

	delete Module;
	}