//--------------------------------------------------------------------------------------------------
// worldrenderer.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "worldrenderer.h"


//--------------------------------------------------------------------------------------------------
// VsWorldRenderer
//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::DrawFrame( VsCamera* Camera )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnBodyAdded( IVsBody* )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnBodyRemoved( IVsBody* )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnShapeAdded( IVsBody* Body, IVsShape* Shape )
	{
	using VsShapeCallback = void ( VsWorldRenderer::* )( IVsShape* );
	VsShapeCallback Callback[] =
		{
		&VsWorldRenderer::OnSphereAdded,
		&VsWorldRenderer::OnCapsuleAdded,
		&VsWorldRenderer::OnHullAdded,
		&VsWorldRenderer::OnMeshAdded
		};

	VsShapeType Type = Shape->GetType();
	return std::invoke( Callback[ Type ], this, Shape );
	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnShapeRemoved( IVsBody* Body, IVsShape* Shape )
	{
	using VsShapeCallback = void ( VsWorldRenderer::* )( IVsShape* );
	VsShapeCallback Callback[] =
		{
		&VsWorldRenderer::OnSphereRemoved,
		&VsWorldRenderer::OnCapsuleRemoved,
		&VsWorldRenderer::OnHullRemoved,
		&VsWorldRenderer::OnMeshRemoved
		};

	VsShapeType Type = Shape->GetType();
	return std::invoke( Callback[ Type ], this, Shape );
	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnSphereAdded( IVsShape* Shape )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnSphereRemoved( IVsShape* Shape )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnCapsuleAdded( IVsShape* Shape )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnCapsuleRemoved( IVsShape* Shape )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnHullAdded( IVsShape* Shape )
	{
	VS_ASSERT( Shape->GetType() == VS_HULL_SHAPE );
	IVsHullShape* HullShape = static_cast< IVsHullShape* >( Shape );

	const IVsHull* Hull = HullShape->GetHull();
	if ( !mHullMap.contains( Hull ) )
		{
		VsMesh* Mesh = vsCreateMesh( HullShape );
		VsInstancedMesh* InstancedMesh = new VsInstancedMesh( Mesh );
		mHullMap[ Hull ] = InstancedMesh;
		}
	
	VsInstancedMesh* InstancedMesh = mHullMap[ Hull ];
	mHullInstances[ InstancedMesh ].push_back( HullShape );
	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnHullRemoved( IVsShape* Shape )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnMeshAdded( IVsShape* Shape )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnMeshRemoved( IVsShape* Shape )
	{

	}