//--------------------------------------------------------------------------------------------------
// scenerenderer.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "scenerenderer.h"


//--------------------------------------------------------------------------------------------------
// TlInstanceData
//--------------------------------------------------------------------------------------------------
struct TlInstanceData
	{
	glm::mat4 Matrix;
	uint32_t Color;
	};


//--------------------------------------------------------------------------------------------------
// TlSceneRenderer
//--------------------------------------------------------------------------------------------------
TlSceneRenderer::TlSceneRenderer( const TlScene& Scene )
	: mScene( Scene )
	{
	int BodyCount = Scene.GetBodyCount();
	for ( int BodyIndex = 0; BodyIndex < BodyCount; ++BodyIndex )
		{
		const TlBody& Body = Scene.Bodies[ BodyIndex ];
		for ( const TlShape& Shape : Body.Shapes )
			{
			mShapeMap[ &Shape ] = BodyIndex;
			switch ( Shape.Variant.index() )
				{
				case kSphereShape:
					{
					mGeometryMap[ mSphere ].push_back( &Shape );
					}
					break;

				case kCapsuleShape:
					{
					// DIRK_TODO: ...
					}
					break;

				case kHullShape:
					{
					const TlHullShape& HullShape = std::get< TlHullShape >( Shape.Variant );
					mGeometryMap[ HullShape.Geometry ].push_back( &Shape );
					}
					break;

				case kMeshShape:
					{
					const TlMeshShape& MeshShape = std::get< TlMeshShape >( Shape.Variant );
					mGeometryMap[ MeshShape.Geometry ].push_back( &Shape );;
					}
					break;
				}
			}
		}
	}


//--------------------------------------------------------------------------------------------------
TlSceneRenderer::~TlSceneRenderer()
	{

	}


//--------------------------------------------------------------------------------------------------
void TlSceneRenderer::DrawFrame( std::vector< glm::mat4 >& BodyTransforms )
	{
	for ( auto [ Geometry, Shapes ] : mGeometryMap )
		{
		size_t ShapeCount = Shapes.size();
		std::vector< TlInstanceData > InstanceData( ShapeCount );

		for ( int ShapeIndex = 0; ShapeIndex < ShapeCount; ++ShapeIndex )
			{
			const TlShape* Shape = Shapes[ ShapeIndex ];

			int BodyIndex = mShapeMap[ Shape ];
			const TlBody& Body = mScene.Bodies[ BodyIndex ];
			glm::mat4 BodyTransform = BodyTransforms[ BodyIndex ];
			const uint32_t BodyColors[] = { 0xff0000ff, 0x00ff00ff, 0x0000ffff };
			uint32_t BodyColor = BodyColors[ Body.Type ];
			
			InstanceData[ ShapeIndex ].Matrix = BodyTransform;
			InstanceData[ ShapeIndex ].Color = BodyColor;
			}
		}


	}