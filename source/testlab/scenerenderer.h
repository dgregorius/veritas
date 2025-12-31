//--------------------------------------------------------------------------------------------------
/**
	@file		scenerenderer.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <glm/fwd.hpp>

#include <algorithm>
#include <vector>

struct TlScene;


//--------------------------------------------------------------------------------------------------
// TlSceneRenderer
//--------------------------------------------------------------------------------------------------
class TlSceneRenderer
	{
	public:
		// Construction / Destruction
		explicit TlSceneRenderer( const TlScene& Scene );
		~TlSceneRenderer();

		// Rendering
		void DrawFrame( std::vector< glm::mat4 >& SceneTransforms );

	private:
		const TlScene& mScene;
	};