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

#include <veritas/scene.h>


//--------------------------------------------------------------------------------------------------
// TlSceneRenderer
//--------------------------------------------------------------------------------------------------
class TlSceneRenderer
	{
	public:
		// Construction / Destruction
		explicit TlSceneRenderer( const TlScene& Scene );
		~TlSceneRenderer();

	private:
		const TlScene& mScene;

		TlGeometry* mSphere = nullptr;
		std::unordered_map< const TlShape*, int > mShapeMap;
		std::unordered_map< const TlGeometry*, std::vector< const TlShape* > > mGeometryMap;
		
	};