//--------------------------------------------------------------------------------------------------
/**
	@file		physxsimulation.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "testlab/scenesimulation.h"


//--------------------------------------------------------------------------------------------------
// TlPhysXSimulation
//--------------------------------------------------------------------------------------------------
class TlPhysXSimulation : public TlSceneSimulation
	{
	public:
		// Construction / Destruction
		explicit TlPhysXSimulation( const TlScene& Scene );
		virtual ~TlPhysXSimulation() override;

		// Simulation
		virtual void Advance( std::vector< glm::mat4 >& SceneTransforms, float Timestep ) override;

	private:

	};