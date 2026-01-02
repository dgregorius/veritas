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

		void Finalize();

		// Frame management
		virtual void BeginFrame( double Time, float Timestep );
		virtual void UpdateFrame( double Time, float Timestep );
		virtual void RenderFrame( double Time, float Timestep );
		virtual void EndFrame( double Time, float Timestep );

	protected:
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
template < std::derived_from< TlTest > T > 
TlTest* tlCreateTest()
	{
	T* Test = new T();
	Test->Finalize();

	return Test;
	}

#define TL_DEFINE_TEST( Category, Name, Type )\
static const int s##Type = tlRegisterTest( Category, Name, tlCreateTest< Type > )
