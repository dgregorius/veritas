//--------------------------------------------------------------------------------------------------
// physxsimulation.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "physxsimulation.h"

// PhysX
#include <PxPhysicsAPI.h>


//--------------------------------------------------------------------------------------------------
// TlPhysXSimulation
//--------------------------------------------------------------------------------------------------
TlPhysXSimulation::TlPhysXSimulation( const TlScene& Scene )
	: TlSceneSimulation( Scene )
	{

	}


//--------------------------------------------------------------------------------------------------
TlPhysXSimulation::~TlPhysXSimulation()
	{

	}


//--------------------------------------------------------------------------------------------------
void TlPhysXSimulation::Advance( std::vector< glm::mat4 >& SceneTransforms, float Timestep )
	{

	}