//--------------------------------------------------------------------------------------------------
// mesh.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "mesh.h"

// Veritas
#include <veritas//veritas.h>


//--------------------------------------------------------------------------------------------------
// VsMesh
//--------------------------------------------------------------------------------------------------
static VsMesh* vsOnCreateSphere( IVsShape* SphereShape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
static VsMesh* vsOnCreateCapsule( IVsShape* CapsuleShape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
static VsMesh* vsOnCreateHull( IVsShape* HullShape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
static VsMesh* vsOnCreateMesh( IVsShape* MeshShape )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
VsMesh* vsCreateMesh( IVsShape* Shape )
	{
	typedef VsMesh* ( *VsMeshCreator )( IVsShape* );
	static const VsMeshCreator Creator[] =
		{
		vsOnCreateSphere,
		vsOnCreateCapsule,
		vsOnCreateHull,
		vsOnCreateMesh
		};

	VsShapeType Type = Shape->GetType();
	return Creator[ Type ]( Shape );
	}


//--------------------------------------------------------------------------------------------------
void vsDestroyMesh( VsMesh* Mesh )
	{
	delete Mesh;
	}
