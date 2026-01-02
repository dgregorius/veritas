//--------------------------------------------------------------------------------------------------
/**
	@file		box3dsimulator.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "scenesimulation.h"


//--------------------------------------------------------------------------------------------------
// TlBox3DSimulation
//--------------------------------------------------------------------------------------------------
class TlBox3DSimulation : public TlSceneSimulation
	{
	public:
		// Construction / Destruction
		explicit TlBox3DSimulation( const TlScene& Scene );
		virtual ~TlBox3DSimulation() override;

		// Simulation
		virtual void Advance( std::vector< glm::mat4 >& SceneTransforms, float Timestep ) override;
		
	private:

	};