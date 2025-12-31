//--------------------------------------------------------------------------------------------------
// test.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "test.h"
#include "scene.h"

// Simulation
#include "simulations/box3dsimulation.h"
#include "simulations/joltsimulation.h"
#include "simulations/physxsimulation.h"
#include "simulations/ragnaroksimulation.h"

// Renderer
#include "renderer/scenerenderer.h"


//--------------------------------------------------------------------------------------------------
// TlTest
//--------------------------------------------------------------------------------------------------
TlTest::~TlTest()
	{
	delete mSceneRenderer;
	std::for_each( mSceneSimulations.begin(), mSceneSimulations.end(), []( TlSceneSimulation* Simulation ) { delete Simulation; } );
	}


//--------------------------------------------------------------------------------------------------
void TlTest::UpdateFrame( double Time, float Timestep )
	{
	// Delayed construction 
	if ( mSceneSimulations.empty() )
		{
		mSceneSimulations.push_back( new TlBox3DSimulation( mScene ) );
		mSceneSimulations.push_back( new TlJoltSimulation( mScene ) );
		mSceneSimulations.push_back( new TlPhysXSimulation( mScene ) );
		mSceneSimulations.push_back( new TlRagnarokSimulation( mScene ) );

		assert( !mSceneRenderer );
		mSceneRenderer = new TlSceneRenderer( mScene );
		}

	// Advance simulations
	for ( TlSceneSimulation* Simulation : mSceneSimulations )
		{
		std::vector< glm::mat4 > SceneTransforms;
		Simulation->Advance( SceneTransforms, Timestep );

		mSceneRenderer->DrawFrame( SceneTransforms );
		}
	}


//--------------------------------------------------------------------------------------------------
// TlTestEntry
//--------------------------------------------------------------------------------------------------
std::vector< TlTestEntry >& tlGetTestEntries()
	{
	static std::vector< TlTestEntry > TestEntries;
	return TestEntries;
	}


//--------------------------------------------------------------------------------------------------
// Test registry
//--------------------------------------------------------------------------------------------------
int tlRegisterTest( const char* Category, const char* Name, TlCreator Creator )
	{
	std::vector< TlTestEntry >& TestEntries = tlGetTestEntries();
	TestEntries.push_back( { Category, Name, Creator } );
	return static_cast< int >( TestEntries.size() - 1 );
	}