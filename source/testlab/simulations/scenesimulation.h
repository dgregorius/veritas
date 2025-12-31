//--------------------------------------------------------------------------------------------------
/**
	@file		scenesimulation.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "testlab/scene.h"


//--------------------------------------------------------------------------------------------------
// TlSceneSimulation
//--------------------------------------------------------------------------------------------------
class TlSceneSimulation
	{
	public:
	public:
		// Construction / Destruction
		explicit TlSceneSimulation( const TlScene& Scene );
		virtual ~TlSceneSimulation() = default;

		// Simulation
		virtual void Advance( std::vector< glm::mat4 >& SceneTransforms, float Timestep ) = 0;

	protected:
		const TlScene& mScene;
	};