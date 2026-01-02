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

#include "testlab/scene.h"


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
		void DrawFrame( std::vector< glm::mat4 >& BodyTransforms );

	private:
		const TlScene& mScene;

		TlGeometry* mSphere = nullptr;
		std::unordered_map< const TlShape*, int > mShapeMap;
		std::unordered_map< const TlGeometry*, std::vector< const TlShape* > > mGeometryMap;
		
	};