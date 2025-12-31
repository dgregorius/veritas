//--------------------------------------------------------------------------------------------------
/**
	@file		joltsimulation.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "testlab/scenesimulation.h"


//--------------------------------------------------------------------------------------------------
// TlJoltSimulation
//--------------------------------------------------------------------------------------------------
class TlJoltSimulation : public TlSceneSimulation
	{
	public:
		// Construction / Destruction
		explicit TlJoltSimulation( const TlScene& Scene );
		virtual ~TlJoltSimulation() override;

		// Simulation
		virtual void Advance( std::vector< glm::mat4 >& SceneTransforms, float Timestep ) override;

	private:

	};