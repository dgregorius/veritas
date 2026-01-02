//--------------------------------------------------------------------------------------------------
// box3dsimulation.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "box3dsimulation.h"

// Box3D
#include <box3d/box3d.h>


//--------------------------------------------------------------------------------------------------
// TlBox3DSimulation
//--------------------------------------------------------------------------------------------------
TlBox3DSimulation::TlBox3DSimulation( const TlScene& Scene )
	: TlSceneSimulation( Scene )
	{

	}


//--------------------------------------------------------------------------------------------------
TlBox3DSimulation::~TlBox3DSimulation()
	{

	}


//--------------------------------------------------------------------------------------------------
void TlBox3DSimulation::Advance( std::vector< glm::mat4 >& SceneTransforms, float Timestep )
	{

	}