//--------------------------------------------------------------------------------------------------
/**
	@file		worldrenderer.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "geometry.h"
#include "instancedmesh.h"
#include "shader.h"
#include "vertex.h"

// Veritas
#include <veritas//veritas.h>

// STD
#include <functional>
#include <unordered_map>
#include <vector>


//--------------------------------------------------------------------------------------------------
// VsWorldRenderer
//--------------------------------------------------------------------------------------------------
class VsWorldRenderer : public IVsWorldListener
	{
	public:
		// Construction / Destruction
		explicit VsWorldRenderer( IVsWorld* World );
		~VsWorldRenderer();

		// Rendering
		void DrawFrame();
		
	private:
		// World listener 
		virtual void OnBodyAdded( IVsBody* Body ) override;
		virtual void OnBodyRemoved( IVsBody* Body ) override;
		virtual void OnShapeAdded( IVsBody* Body, IVsShape* Shape ) override;
		virtual void OnShapeRemoved( IVsBody* Body, IVsShape* Shape ) override;

		// Render bridge
		void OnSphereAdded( IVsShape* Shape );
		void OnSphereRemoved( IVsShape* Shape );
		void OnCapsuleAdded( IVsShape* Shape );
		void OnCapsuleRemoved( IVsShape* Shape );
		void OnHullAdded( IVsShape* Shape );
		void OnHullRemoved( IVsShape* Shape );
		void OnMeshAdded( IVsShape* Shape );
		void OnMeshRemoved( IVsShape* Shape );

		IVsWorld* mWorld = nullptr;
		std::unordered_map< const IVsHull*, VsInstancedMesh* > mHullMap;
		std::unordered_map< VsInstancedMesh*, std::vector< IVsHullShape* > > mHullInstances;
	};