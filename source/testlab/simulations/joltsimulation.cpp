//--------------------------------------------------------------------------------------------------
// joltsimulation.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "joltsimulation.h"

// Jolt
#include <jolt/jolt.h>


//--------------------------------------------------------------------------------------------------
// TlJoltSimulation
//--------------------------------------------------------------------------------------------------
TlJoltSimulation::TlJoltSimulation( const TlScene& Scene )
	: TlSceneSimulation( Scene )
	{

	}


//--------------------------------------------------------------------------------------------------
TlJoltSimulation::~TlJoltSimulation()
	{

	}


//--------------------------------------------------------------------------------------------------
void TlJoltSimulation::Advance( std::vector< glm::mat4 >& SceneTransforms, float Timestep )
	{

	}