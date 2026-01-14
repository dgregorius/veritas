//--------------------------------------------------------------------------------------------------
// meshshape.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "meshshape.h"
#include "constants.h"
#include "mass.h"

// SIMD math
#include <simd.h>


//--------------------------------------------------------------------------------------------------
// RkMeshShape
//--------------------------------------------------------------------------------------------------
RkMeshShape::RkMeshShape( RkBody* Body, RkBVH* BVH )
	: RkShape( RK_MESH_SHAPE, Body )
	{
	RK_ASSERT( BVH );
	mBVH = BVH;
	}


//--------------------------------------------------------------------------------------------------
RkBVH* RkMeshShape::GetBVH() const
	{
	return mBVH;
	}


//--------------------------------------------------------------------------------------------------
RkShapeCastResult RkMeshShape::CastRay( const RkVector3& RayStartInput, const RkVector3& RayEndInput, float MaxAlpha ) const
	{
	RK_ASSERT( mBVH );

	int BestTriangle = -1;
	float BestAlpha = MaxAlpha;
	v32 Lambda = vmSplat( BestAlpha );

	v32 RayStart = vmLoad3( RayStartInput );
	v32 RayDelta = vmLoad3( RayEndInput - RayStartInput );
	v32 RayEnd = RayStart + Lambda * RayDelta;
	
	v32 RayMin = vmMin( RayStart, RayEnd );
	v32 RayMax = vmMax( RayStart, RayEnd );

	int Count = 0;
	const RkBVHNode* Stack[ RK_STACK_SIZE ];
	const RkBVHNode* Node = mBVH->GetRoot();

	while ( true )
		{
		// Test node/ray overlap using SAT
		v32 NodeMin = vmLoad3A( Node->BoundsMin );
		v32 NodeMax = vmLoad3A( Node->BoundsMax );
		if ( vmTestBoundsOverlap( NodeMin, NodeMax, RayMin, RayMax ) && vmTestBoundsRayOverlap( NodeMin, NodeMax, RayStart, RayDelta ) )
			{
			// SAT: The node and ray overlap - process leaf node or recurse
			if ( Node->IsLeaf() )
				{
				int TriangleCount = Node->TriangleCount;
				int TriangleOffset = Node->TriangleOffset;

				for ( int Index = 0; Index < TriangleCount; ++Index )
					{
					int TriangleIndex = TriangleOffset + Index;
					const RkBVHTriangle& Triangle = mBVH->GetTriangle( TriangleIndex );

					v32 Vertex1 = vmLoad3A( mBVH->VertexList[ Triangle.Index1 ] );
					v32 Vertex2 = vmLoad3A( mBVH->VertexList[ Triangle.Index2 ] );
					v32 Vertex3 = vmLoad3A( mBVH->VertexList[ Triangle.Index3 ] );

					float Alpha = vmIntersectRayTriangle( RayStart, RayDelta, Vertex1, Vertex2, Vertex3 );
					RK_ASSERT( 0 <= Alpha && Alpha <= 1.0f );

					if ( Alpha < BestAlpha )
						{
						BestAlpha = Alpha;
						BestTriangle = TriangleIndex;

						// Update ray bounds
						Lambda = vmSplat( BestAlpha );
						RayEnd = RayStart + Lambda * RayDelta;
						RayMin = vmMin( RayStart, RayEnd );
						RayMax = vmMax( RayStart, RayEnd );
						}
					}
				}
			else
				{
				// Determine traversal order (front -> back) and recurse
				int Axis = Node->Axis;
				if ( vmGet( RayDelta, Axis ) > 0.0f )
					{
					RK_ASSERT( Count <= RK_STACK_SIZE - 1 );
					Stack[ Count++ ] = Node->GetRightChild();
					Node = Node->GetLeftChild();
					}
				else
					{
					RK_ASSERT( Count <= RK_STACK_SIZE - 1 );
					Stack[ Count++ ] = Node->GetLeftChild();
					Node = Node->GetRightChild();
					}

				continue;
				}
			}

		if ( Count == 0 )
			{
			break;
			}
		Node = Stack[ --Count ];
		}

	// Fill result structure
	RkShapeCastResult Result;
	if ( BestAlpha < MaxAlpha )
		{
		RK_ASSERT( 0 <= BestTriangle && BestTriangle < mBVH->TriangleCount );
		RkBVHTriangle Triangle = mBVH->GetTriangle( BestTriangle );

		RkVector3 Vertex1 = mBVH->VertexList[ Triangle.Index1 ];
		RkVector3 Vertex2 = mBVH->VertexList[ Triangle.Index2 ];
		RkVector3 Vertex3 = mBVH->VertexList[ Triangle.Index3 ];

		RkVector3 Edge1 = Vertex2 - Vertex1;
		RkVector3 Edge2 = Vertex3 - Vertex1;
		RkVector3 Normal = rkCross( Edge1, Edge2 );

		Result.HitPoint = rkLerp( RayStartInput, RayEndInput, BestAlpha );
		Result.HitNormal = rkNormalize( Normal );
		Result.HitTime = BestAlpha;
		Result.Triangle = BestTriangle;
		}

	return Result;
	}


//--------------------------------------------------------------------------------------------------
RkBounds3 RkMeshShape::ComputeBounds( const RkTransform& Transform ) const
	{
	// The shape bounds must contain the convex radius in the broadphase for contact generation!
	RkBounds3 Bounds = rkInflate( mBVH->Bounds, RK_CONVEX_RADIUS );
	return Transform * Bounds;
	}


//--------------------------------------------------------------------------------------------------
RkMassProperties RkMeshShape::ComputeMassProperties() const
	{
	RkMassProperties Out;
	Out.Mass = 0.0f;
	Out.Center = RK_VEC3_ZERO;
	Out.Inertia = RK_MAT3_ZERO;
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
float RkMeshShape::GetMinMotionRadius() const
	{
	return RK_F32_MAX;
	}


//--------------------------------------------------------------------------------------------------
float RkMeshShape::GetMaxMotionRadius( const RkVector3& Center ) const
	{
	// Needed for keyframed meshes to sleep
	RkBounds3 Bounds = mBVH->Bounds;
	float Radius1 = rkDistance( Bounds.Min, Center );
	float Radius2 = rkDistance( Bounds.Max, Center ); 

	return rkMax( Radius1, Radius2 );
	}
