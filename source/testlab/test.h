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

#include "scene.h"

class TlSceneSimulation;
class TlSceneRenderer;


//--------------------------------------------------------------------------------------------------
// TlTest
//--------------------------------------------------------------------------------------------------
class TlTest
	{
	public:
		// Construction / Destruction
		TlTest() = default;
		virtual ~TlTest();
		
		void UpdateFrame( double Time, float Timestep );

	protected:
		TlScene mScene;

		std::vector< TlSceneSimulation* > mSceneSimulations;
		TlSceneRenderer* mSceneRenderer = nullptr;
	};


//--------------------------------------------------------------------------------------------------
// TlTestEntry
//--------------------------------------------------------------------------------------------------
typedef TlTest* ( *TlCreator )( );

struct TlTestEntry
	{
	const char* Category = nullptr;
	const char* Name = nullptr;
	TlCreator Creator = nullptr;
	};

int tlRegisterTest( const char* Category, const char* Name, TlCreator Creator );
std::vector< TlTestEntry >& tlGetTestEntries();


//--------------------------------------------------------------------------------------------------
// Test registry
//--------------------------------------------------------------------------------------------------
template < typename T >
TlTest* tlCreateTest()
	{
	return new T();
	}

#define TL_DEFINE_TEST( Category, Name, Type )\
static const int s##Type = tlRegisterTest( Category, Name, tlCreateTest< Type > )
