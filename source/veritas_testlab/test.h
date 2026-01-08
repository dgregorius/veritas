//--------------------------------------------------------------------------------------------------
/**
	@file		test.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <veritas/veritas.h>

#include <veritas_renderer/camera.h>
#include <veritas_renderer/rendertarget.h>
#include <veritas_renderer/shader.h>
#include <veritas_renderer/vertex.h>
#include <veritas_renderer/worldrenderer.h>


//--------------------------------------------------------------------------------------------------
// VsTest
//--------------------------------------------------------------------------------------------------
class VsTest
	{
	public:
		// Construction / Destruction
		explicit VsTest( IVsPlugin* Plugin );
		virtual ~VsTest();

		// Lifetime management
		virtual void Create( VsCamera* Camera ) = 0;
		virtual void Update( double Time, float Timestep );
		virtual void Render( double Time, float Timestep );
		virtual void Destroy();

	protected:
		IVsPlugin* mPlugin = nullptr;
		IVsWorld* mWorld = nullptr;

		VsWorldRenderer* mWorldRenderer = nullptr;
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

std::vector< VsTestEntry >& vsGetTestEntries();


//--------------------------------------------------------------------------------------------------
// Test registry
//--------------------------------------------------------------------------------------------------
template < std::derived_from< VsTest > T > 
VsTest* vsCreateTest( IVsPlugin* Plugin )
	{
	return new T( Plugin );
	}

int vsRegisterTest( const char* Category, const char* Name, VsCreator Creator );

#define VS_DEFINE_TEST( Category, Name, Type ) \
static const int s##Type = vsRegisterTest( Category, Name, vsCreateTest< Type > )
