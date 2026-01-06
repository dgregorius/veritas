//--------------------------------------------------------------------------------------------------
// test.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "test.h"


//--------------------------------------------------------------------------------------------------
// TlTest
//--------------------------------------------------------------------------------------------------
VsTest::VsTest( IVsPlugin* Plugin )
	{
	VS_ASSERT( Plugin );
	mPlugin = Plugin;

	mWorld = Plugin->CreateWorld();
	mWorldRenderer = new VsWorldRenderer;
	mWorld->AddListener( mWorldRenderer );
	}


//--------------------------------------------------------------------------------------------------
VsTest::~VsTest()
	{
	mWorld->RemoveListener( mWorldRenderer );
	delete mWorldRenderer;

	mPlugin->DestroyWorld( mWorld );
	}


//--------------------------------------------------------------------------------------------------
void VsTest::Update( double Time, float Timestep )
	{
	
	}


//--------------------------------------------------------------------------------------------------
void VsTest::Render( double Time, float Timestep )
	{
	mWorldRenderer->DrawFrame( nullptr );
	}


//--------------------------------------------------------------------------------------------------
void VsTest::Destroy()
	{

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
int vsRegisterTest( const char* Category, const char* Name, VsCreator Creator )
	{
	std::vector< VsTestEntry >& TestEntries = vsGetTestEntries();
	TestEntries.push_back( { Category, Name, Creator } );
	
	return static_cast< int >( TestEntries.size() - 1 );
	}