//--------------------------------------------------------------------------------------------------
/**
	@file		test.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <veritas/veritas.h>


//--------------------------------------------------------------------------------------------------
// VsTest
//--------------------------------------------------------------------------------------------------
class VsTest
	{
	public:
		// Construction / Destruction
		VsTest( IVsPlugin* Plugin );
		virtual ~VsTest();

		// Frame management
		virtual void BeginFrame( double Time, float Timestep );
		virtual void UpdateFrame( double Time, float Timestep );
		virtual void RenderFrame( double Time, float Timestep );
		virtual void EndFrame( double Time, float Timestep );

	protected:
		IVsPlugin* mPlugin = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// VsTestEntry
//--------------------------------------------------------------------------------------------------
typedef VsTest* ( *VsCreator )( IVsPlugin* );

struct VsTestEntry
	{
	const char* Category = nullptr;
	const char* Name = nullptr;
	VsCreator Creator = nullptr;
	};

int vsRegisterTest( const char* Category, const char* Name, VsCreator Creator );
std::vector< VsTestEntry >& vsGetTestEntries();


//--------------------------------------------------------------------------------------------------
// Test registry
//--------------------------------------------------------------------------------------------------
template < std::derived_from< VsTest > T > 
VsTest* vsCreateTest( IVsPlugin* Plugin )
	{
	return new T( Plugin );
	}

#define VS_DEFINE_TEST( Category, Name, Type ) \
static const int s##Type = vsRegisterTest( Category, Name, vsCreateTest< Type > )
