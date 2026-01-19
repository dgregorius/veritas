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
#include <veritas_renderer/geometry.h>
#include <veritas_renderer/instancedmesh.h>
#include <veritas_renderer/rendertarget.h>
#include <veritas_renderer/shader.h>
#include <veritas_renderer/vertex.h>
#include <veritas_renderer/worldrenderer.h>

#if defined( DEBUG ) || defined( _DEBUG )
#define VS_DEBUG_TEST 1
#else
#define VS_DEBUG_TEST 0
#endif


//--------------------------------------------------------------------------------------------------
// VsTest
//--------------------------------------------------------------------------------------------------
class VsTest
	{
	public:
		// Construction / Destruction
		explicit VsTest( IVsPlugin* Plugin );
		virtual ~VsTest() = default;

		// Lifetime management
		virtual void Create() = 0;
		virtual void Update( VsCamera* Camera, float Timestep );
		virtual void Render( VsCamera* Camera, float Timestep );
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
	VsOrbit Orbit;

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

int vsRegisterTest( const char* Category, const char* Name, VsOrbit Orbit, VsCreator Creator );

#define VS_DEFINE_TEST( Category, Name, Orbit, Type ) \
static const int s##Type = vsRegisterTest( Category, Name, Orbit, vsCreateTest< Type > )
