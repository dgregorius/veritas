//--------------------------------------------------------------------------------------------------
// geometry.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "geometry.h"


//--------------------------------------------------------------------------------------------------
// VsGeometry
//--------------------------------------------------------------------------------------------------
static VsGeometry* vsOnCreateSphere( IVsShape* SphereShape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
static VsGeometry* vsOnCreateCapsule( IVsShape* CapsuleShape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
static VsGeometry* vsOnCreateHull( IVsShape* HullShape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
static VsGeometry* vsOnCreateMesh( IVsShape* MeshShape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
VsGeometry* vsCreateGeometry( IVsShape* Shape )
	{
	typedef VsGeometry* ( *VsGeometryCreator )( IVsShape* );
	static const VsGeometryCreator Creator[] =
		{
		vsOnCreateSphere,
		vsOnCreateCapsule,
		vsOnCreateHull,
		vsOnCreateMesh
		};

	VsShapeType Type = Shape->GetType();
	return Creator[ Type ]( Shape );
	}