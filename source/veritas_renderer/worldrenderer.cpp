//--------------------------------------------------------------------------------------------------
// worldrenderer.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "worldrenderer.h"

// OpenGL
#include <glad.h>
#include <glfw3.h>


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static inline glm::mat4 vsAsMat4( const VsVector3& Position, const VsQuaternion& Orientation )
	{
	glm::mat4 Rotation = glm::mat4_cast( glm::quat { Orientation.W, Orientation.X, Orientation.Y, Orientation.Z } );

	glm::mat4 Translation( 1.0f );
	Translation = glm::translate( Translation, { Position.X, Position.Y, Position.Z });

	return Translation * Rotation;
	}


//--------------------------------------------------------------------------------------------------
// VsWorldRenderer
//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::DrawFrame()
	{
	// Render instances
	glEnable( GL_CULL_FACE );
	glEnable( GL_DEPTH_TEST );
	glBindVertexArray( VsMeshVertex::Format );

	VsShader* MeshShader = VsShaderLibrary::MeshShader;
	MeshShader->Use();
	
	for ( const auto& [ InstancedHull, HullShapes ] : mHullInstances )
		{
		size_t ShapeCount = HullShapes.size();
		std::vector< VsInstanceData > InstanceData;
		InstanceData.resize( ShapeCount );

		for ( size_t ShapeIndex = 0; ShapeIndex < ShapeCount; ++ShapeIndex )
			{
			IVsShape* Shape = HullShapes[ ShapeIndex ];

			IVsBody* Body = Shape->GetBody();
			InstanceData[ ShapeIndex ].Transform = vsAsMat4( Body->GetPosition(), Body->GetOrientation() );
			InstanceData[ ShapeIndex ].Color = { 0.8f, 0.0f, 0.0f, 1.0f };
			}

		InstancedHull->Upload( InstanceData );
		InstancedHull->Render();
		}

	glBindVertexArray( GL_NONE );
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_CULL_FACE );

	VS_ASSERT( glGetError() == GL_NO_ERROR );
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
		VsGeometry* Geometry = vsCreateGeometry( HullShape );
		VsInstancedMesh* InstancedMesh = new VsInstancedMesh( Geometry );
		mHullMap[ Hull ] = InstancedMesh;
		}
	
	VsInstancedMesh* InstancedMesh = mHullMap[ Hull ];
	mHullInstances[ InstancedMesh ].push_back( HullShape );
	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnHullRemoved( IVsShape* Shape )
	{
	VS_ASSERT( Shape->GetType() == VS_HULL_SHAPE );
	IVsHullShape* HullShape = static_cast<IVsHullShape*>( Shape );

	const IVsHull* Hull = HullShape->GetHull();
	VS_ASSERT( mHullMap.contains( Hull ) );
	VsInstancedMesh* InstancedMesh = mHullMap[ Hull ];
	VS_ASSERT( mHullInstances.contains( InstancedMesh ) );
	std::vector< IVsHullShape* >& HullShapes = mHullInstances[ InstancedMesh ];
	VS_ASSERT( std::find( HullShapes.begin(), HullShapes.end(), HullShape ) != HullShapes.end() );
	std::erase( HullShapes, HullShape );
	if ( HullShapes.empty() )
		{
		mHullMap.erase( Hull );
		mHullInstances.erase( InstancedMesh );

		VsGeometry* Geomtery = InstancedMesh->GetGeometry();
		delete InstancedMesh;
		delete Geomtery;
		}
	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnMeshAdded( IVsShape* Shape )
	{

	}


//--------------------------------------------------------------------------------------------------
void VsWorldRenderer::OnMeshRemoved( IVsShape* Shape )
	{

	}