//--------------------------------------------------------------------------------------------------
// clipping.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "clipping.h"
#include "capsule.h"
#include "hull.h"


//--------------------------------------------------------------------------------------------------
// RkFeaturePair
//--------------------------------------------------------------------------------------------------
RkFeaturePair rkMakePair( RkFeatureType Type1, int Index1, RkFeatureType Type2, int Index2 )
	{
	RkFeaturePair Pair;
	Pair.IncomingIndex = uint8( Index1 );
	Pair.IncomingType = uint8( Type1 );
	Pair.OutgoingIndex = uint8( Index2 );
	Pair.OutgoingType = uint8( Type2 );
	RK_ASSERT( Pair != RK_INVALID_FEATURE_PAIR );

	return Pair;
	}


//--------------------------------------------------------------------------------------------------
RkFeaturePair rkFlipPair( RkFeaturePair Pair )
	{
	RK_ASSERT( Pair.IncomingType == 0 || Pair.IncomingType == 1 );
	RK_ASSERT( Pair.OutgoingType == 0 || Pair.OutgoingType == 1 );
	std::swap( Pair.IncomingType, Pair.OutgoingType );
	Pair.IncomingType = 1 - Pair.IncomingType;
	Pair.OutgoingType = 1 - Pair.OutgoingType;
	std::swap( Pair.IncomingIndex, Pair.OutgoingIndex );
	RK_ASSERT( Pair != RK_INVALID_FEATURE_PAIR );

	return Pair;
	}


//--------------------------------------------------------------------------------------------------
// General clipping
//--------------------------------------------------------------------------------------------------
bool rkClipSegment( RkClipVertex Segment[ 2 ], const RkPlane3& Plane )
	{
	int VertexCount = 0;
	RkClipVertex Vertex1 = Segment[ 0 ];
	RkClipVertex Vertex2 = Segment[ 1 ];

	float Distance1 = rkDistance( Plane, Vertex1.Position );
	float Distance2 = rkDistance( Plane, Vertex2.Position );

	// If the points are behind the plane
	if ( Distance1 <= 0.0f )
		{
		Segment[ VertexCount++ ] = Vertex1;
		}
	if ( Distance2 <= 0.0f )
		{
		Segment[ VertexCount++ ] = Vertex2;
		}

	// If the points are on different sides of the plane
	if ( Distance1 * Distance2 < 0.0f )
		{
		// Find intersection point of edge and plane
		float T = Distance1 / ( Distance1 - Distance2 );
		Segment[ VertexCount ].Position = ( 1.0f - T ) * Vertex1.Position + T * Vertex2.Position;
		Segment[ VertexCount ].Pair = Distance1 > 0.0f ? Vertex1.Pair : Vertex2.Pair; 
		VertexCount++;
		}

	return VertexCount == 2;
	}


//--------------------------------------------------------------------------------------------------
// Capsule clipping
//--------------------------------------------------------------------------------------------------
void rkBuildSegment( RkClipVertex Out[ 2 ], const RkTransform& Transform, const RkCapsule& Capsule )
	{
	Out[ 0 ].Position = Transform * Capsule.Center1;
	Out[ 0 ].Separation = 0.0f;
	Out[ 0 ].Pair = rkMakePair( RK_FEATURE_SHAPE_1, 0, RK_FEATURE_SHAPE_1, 0 );
	Out[ 1 ].Position = Transform * Capsule.Center2;
	Out[ 1 ].Separation = 0.0f;
	Out[ 1 ].Pair = rkMakePair( RK_FEATURE_SHAPE_1, 1, RK_FEATURE_SHAPE_1, 1 );
	}


//--------------------------------------------------------------------------------------------------
bool rkClipSegment( RkClipVertex Segment[ 2 ], const RkTransform& Transform, const RkHull* Hull, int RefFace )
	{
	RkPlane3 RefPlane = Transform * Hull->GetPlane( RefFace );

	const RkFace* Face = Hull->GetFace( RefFace );
	const RkHalfEdge* Edge = Hull->GetEdge( Face->Edge );
	RK_ASSERT( Edge->Face == RefFace );

	do 
		{
		const RkHalfEdge* Next = Hull->GetEdge( Edge->Next );
		
		RkVector3 Vertex1 = Transform * Hull->GetPosition( Edge->Origin );
		RkVector3 Vertex2 = Transform * Hull->GetPosition( Next->Origin );
		RkVector3 Tangent = rkNormalize( Vertex2 - Vertex1 );
		RkVector3 Bitangent = rkCross( Tangent, RefPlane.Normal );

		if ( !rkClipSegment( Segment, RkPlane3( Bitangent, Vertex1 ) ) )
			{
			return false;
			}

		Edge = Next;
		} 
	while ( Edge != Hull->GetEdge( Face->Edge ) );

	return true;
	}


//--------------------------------------------------------------------------------------------------
// Hull clipping
//--------------------------------------------------------------------------------------------------
static bool rkValidatePolygon( const RkClipVertex* Polygon, int VertexCount )
	{
	// Empty polygons are valid (we can clip away all points when re-constructing manifolds from cache)
	if ( !VertexCount )
		{
		return true;
		}

	// Validate that incoming and outgoing edges match
	RkClipVertex Vertex1 = Polygon[ VertexCount - 1 ];
	for ( int VertexIndex2 = 0; VertexIndex2 < VertexCount; ++VertexIndex2 )
		{
		RkClipVertex Vertex2 = Polygon[ VertexIndex2 ];
		if ( Vertex1.Pair.OutgoingType != Vertex2.Pair.IncomingType )
			{
			return false;
			}

		if ( Vertex1.Pair.OutgoingIndex != Vertex2.Pair.IncomingIndex )
			{
			return false;
			}

		Vertex1 = Vertex2;
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
int rkBuildPolygon( RkClipVertex* Polygon, const RkTransform& Transform, const RkHull* Hull, int IncFace, const RkPlane3& RefPlane )
	{
	int VertexCount = 0;
	const RkFace* Face = Hull->GetFace( IncFace );
	const RkHalfEdge* Edge = Hull->GetEdge( Face->Edge );
	RK_ASSERT( Edge->Face == IncFace );

	do
		{
		const RkHalfEdge* Next = Hull->GetEdge( Edge->Next );

		RkClipVertex& Vertex = Polygon[ VertexCount++ ];
		Vertex.Position = Transform * Hull->GetPosition( Next->Origin );
		Vertex.Separation = rkDistance( RefPlane, Vertex.Position );
		Vertex.Pair = rkMakePair( RK_FEATURE_SHAPE_2, Hull->GetEdgeIndex( Edge ), RK_FEATURE_SHAPE_2, Hull->GetEdgeIndex( Next ) );
		
		Edge = Next;
		}
	while ( Edge != Hull->GetEdge( Face->Edge ) );
	
	RK_ASSERT( rkValidatePolygon( Polygon, VertexCount ) );
	return VertexCount;
	}


//--------------------------------------------------------------------------------------------------
int rkClipPolygon( RkClipVertex* RK_RESTRICT Out, const RkClipVertex* RK_RESTRICT Polygon, int VertexCount, const RkPlane3& ClipPlane, int Edge, const RkPlane3& RefPlane )
	{
	int Count = 0;
	RkClipVertex Vertex1 = Polygon[ VertexCount - 1 ];
	float Distance1 = rkDistance( ClipPlane, Vertex1.Position );

	for ( int VertexIndex = 0; VertexIndex < VertexCount; ++VertexIndex )
		{
		RkClipVertex Vertex2 = Polygon[ VertexIndex ];
		float Distance2 = rkDistance( ClipPlane, Vertex2.Position );

		// Clip edge against plane (Sutherland-Hodgman clipping)
		if ( Distance1 <= 0.0f && Distance2 <= 0.0f )
			{
			// Both vertices are behind the plane - keep vertex2
			Out[ Count++ ] = Vertex2;
			}
		else if ( Distance1 <= 0.0f && Distance2 > 0.0f )
			{
			// Vertex1 is behind of the plane, vertex2 is in front -> intersection point
			float Fraction = Distance1 / ( Distance1 - Distance2 );
			RkVector3 Position = Vertex1.Position + Fraction * ( Vertex2.Position - Vertex1.Position );

			// Keep intersection point and adjust outgoing edge
			RkClipVertex& Intersection = Out[ Count++ ];
			Intersection.Position = Position;
			Intersection.Separation = rkDistance( RefPlane, Position );
			Intersection.Pair = Vertex2.Pair;
			Intersection.Pair.OutgoingType = RK_FEATURE_SHAPE_1;
			Intersection.Pair.OutgoingIndex = static_cast< uint8 >( Edge );
			}
		else if ( Distance2 <= 0.0f && Distance1 > 0 )
			{
			// Vertex1 is in front, vertex2 is behind of the plane, -> intersection point
			float Fraction = Distance1 / ( Distance1 - Distance2 );
			RkVector3 Position = Vertex1.Position + Fraction * ( Vertex2.Position - Vertex1.Position );

			// Keep intersection point and adjust incoming edge
			RkClipVertex& Intersection = Out[ Count++ ];
			Intersection.Position = Position;
			Intersection.Separation = rkDistance( RefPlane, Position );
			Intersection.Pair = Vertex1.Pair;
			Intersection.Pair.IncomingType = RK_FEATURE_SHAPE_1;
			Intersection.Pair.IncomingIndex = static_cast< uint8 >( Edge );
			
			// And also keep vertex2
			Out[ Count++ ] = Vertex2;
			}

		// Keep vertex2 as starting vertex for next edge
		Vertex1 = Vertex2;
		Distance1 = Distance2;
		}

	RK_ASSERT( rkValidatePolygon( Out, Count ) );
	return Count;
	}


//--------------------------------------------------------------------------------------------------
int rkCullPolygon( RkClipVertex* Polygon, int VertexCount, const RkPlane3& RefPlane, float MaxSeparation )
	{
	// 1. Remove all non-penetrating points
	for ( int Index = VertexCount - 1; Index >= 0; --Index )
		{
		if ( Polygon[ Index ].Separation > MaxSeparation )
			{
			Polygon[ Index ] = Polygon[ --VertexCount ];
			
			continue;
			}

		// Slide point onto reference plane
		RK_ASSERT( Polygon[ Index ].Separation <= 2.0f * RK_CONVEX_RADIUS );
		Polygon[ Index ].Position -= Polygon[ Index ].Separation * RefPlane.Normal;
		}

	if ( !VertexCount )
		{
		return 0;
		}

	// 2. Find extreme point in arbitrary direction V 
	RkVector3 V = rkPerp( RefPlane.Normal );
	V = RkQuaternion( RefPlane.Normal, RK_DEG2RAD * 15.0f ) * V;
	
	int BestIndex1 = -1;
	float BestDistance1 = -RK_F32_MAX;
	
	for ( int Index = 0; Index < VertexCount; ++Index )
		{
		RkVector3 Q = Polygon[ Index ].Position;
		float Distance = rkDot( V, Q );
	
		if ( Distance > BestDistance1 )
			{
			BestDistance1 = Distance;
			BestIndex1 = Index;
			}
		}
		
	RK_ASSERT( BestIndex1 >= 0 );
	std::swap( Polygon[ 0 ], Polygon[ BestIndex1 ] );

	// 3. Find farthest point 
	RkVector3 A = Polygon[ 0 ].Position;

	int BestIndex2 = -1;
	float BestDistance2 = -RK_F32_MAX;

	for ( int Index = 1; Index < VertexCount; ++Index )
		{
		RkVector3 Q = Polygon[ Index ].Position;
		float Distance = rkDistance( A, Q );

		if ( Distance > BestDistance2 )
			{
			BestDistance2 = Distance;
			BestIndex2 = Index;
			}
		}

	if ( BestDistance2 < RK_LINEAR_SLOP )
		{
		return 1;
		}

	RK_ASSERT( BestIndex2 >= 1 );
	std::swap( Polygon[ 1 ], Polygon[ BestIndex2 ] );

	// 4. Find largest area
	RkVector3 B = Polygon[ 1 ].Position;

	int BestIndex3 = -1;
	float BestArea3 = 0.0f;

	for ( int Index = 2; Index < VertexCount; ++Index )
		{
		RkVector3 Q = Polygon[ Index ].Position;
		float Area = rkAbs( rkDot( RefPlane.Normal, rkCross( A - Q, B - Q ) ) );

		if ( Area > BestArea3 )
			{
			BestArea3 = Area;
			BestIndex3 = Index;
			}
		}

	if ( BestArea3 < 2.0f * BestDistance2 * RK_LINEAR_SLOP )
		{
		return 2;
		}

	RK_ASSERT( BestIndex3 >= 2 );
	std::swap( Polygon[ 2 ], Polygon[ BestIndex3 ] );

	// 5. Best quad area
	RkVector3 C = Polygon[ 2 ].Position;

	RkVector3 CA = A - C;
	RkVector3 CB = B - C;
	RkVector3 CA_x_CB = rkCross( CA, CB );
	RkVector3 N = rkNormalize( CA_x_CB );
	
	int BestIndex4 = -1;
	float BestArea4 = 0.0f;

	for ( int Index = 3; Index < VertexCount; ++Index )
		{
		RkVector3 Q = Polygon[ Index ].Position;

		RkVector3 QA = A - Q;
		RkVector3 QB = B - Q;
		RkVector3 QC = C - Q;

		RkVector3 QA_x_QB = rkCross( QA, QB );
		RkVector3 QB_x_QC = rkCross( QB, QC );
		RkVector3 QC_x_QA = rkCross( QC, QA );

		float Area1 = rkDot( N, QB_x_QC );
		float Area2 = rkDot( N, QC_x_QA ); 
		float Area3 = rkDot( N, QA_x_QB );
		float Area = rkMin( Area1, rkMin( Area2, Area3 ) );

		if ( Area < BestArea4 )
			{
			BestArea4 = Area;
			BestIndex4 = Index;
			}
		}

	if ( BestArea4 > -2.0f * BestDistance2 * RK_LINEAR_SLOP )
		{
		return 3;
		}

	RK_ASSERT( BestIndex4 >= 3 );
	std::swap( Polygon[ 3 ], Polygon[ BestIndex4 ] );

	return 4;
	}