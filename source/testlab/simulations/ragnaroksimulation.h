//--------------------------------------------------------------------------------------------------
/**
	@file		ragnaroksimulation.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "scenesimulation.h"


//--------------------------------------------------------------------------------------------------
// TlRagnarokSimulation
//--------------------------------------------------------------------------------------------------
class TlRagnarokSimulation : public TlSceneSimulation
	{
	public:
		// Construction / Destruction
		explicit TlRagnarokSimulation( const TlScene& Scene );
		virtual ~TlRagnarokSimulation() override;

		// Simulation
		virtual void Advance( std::vector< glm::mat4 >& SceneTransforms, float Timestep ) override;

	private:

	};