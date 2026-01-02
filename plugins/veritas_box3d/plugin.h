//--------------------------------------------------------------------------------------------------
/**
	@file		plugin.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <veritas/veritas.h>


//--------------------------------------------------------------------------------------------------
// VsBox3Plugin
//--------------------------------------------------------------------------------------------------
struct VsBox3Plugin : IVsPlugin
	{
	virtual ~VsBox3Plugin() override;

	virtual const char* GetName() const override;
	virtual const char* GetVersion() const override;

	virtual IVsHull* CreateHull( int VertexCount, const VsVector3* Vertices ) override;
	virtual IVsMesh* CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices ) override;
	virtual IVsWorld* CreateWorld( const VsWorldDef& WorldDef ) override;
	virtual void DestroyWorld( IVsWorld* World ) override;
	};