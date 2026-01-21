//--------------------------------------------------------------------------------------------------
// test.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "test.h"
#include "clock.h"


//--------------------------------------------------------------------------------------------------
// TlTest
//--------------------------------------------------------------------------------------------------
VsTest::VsTest( IVsPlugin* Plugin, bool AutoSleeping )
	{
	VS_ASSERT( Plugin );
	mPlugin = Plugin;

	mWorld = Plugin->CreateWorld();
	mWorld->SetAutoSleeping( AutoSleeping );
	mWorldRenderer = new VsWorldRenderer( mWorld );
	mWorld->AddListener( mWorldRenderer );
	}


//--------------------------------------------------------------------------------------------------
void VsTest::Update( VsCamera* Camera, float Timestep )
	{
	mWorld->Step( Timestep );
	}


//--------------------------------------------------------------------------------------------------
void VsTest::Render( VsCamera* Camera, float Timestep )
	{
	mWorldRenderer->DrawFrame( Camera );
	}


//--------------------------------------------------------------------------------------------------
void VsTest::Destroy()
	{
	mPlugin->DestroyWorld( mWorld );
	delete mWorldRenderer;
	}


//--------------------------------------------------------------------------------------------------
// TlTestEntry
//--------------------------------------------------------------------------------------------------
std::vector< VsTestEntry >& vsGetTestEntries()
	{
	static std::vector< VsTestEntry > TestEntries;
	return TestEntries;
	}


//--------------------------------------------------------------------------------------------------
// Test registry
//--------------------------------------------------------------------------------------------------
int vsRegisterTest( const char* Category, const char* Name, VsOrbit Orbit, bool Sleeping, VsCreator Creator )
	{
	std::vector< VsTestEntry >& TestEntries = vsGetTestEntries();
	TestEntries.emplace_back( Category, Name, Orbit, Sleeping, Creator );
	
	return static_cast< int >( TestEntries.size() - 1 );
	}