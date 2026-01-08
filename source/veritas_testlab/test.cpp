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
	mPlugin->DestroyWorld( mWorld );
	delete mWorldRenderer;
	}


//--------------------------------------------------------------------------------------------------
void VsTest::Update( double Time, float ElapsedTime )
	{
	mWorld->Step( ElapsedTime );
	}


//--------------------------------------------------------------------------------------------------
void VsTest::Render( double Time, float ElapsedTime )
	{
	mWorldRenderer->DrawFrame();
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