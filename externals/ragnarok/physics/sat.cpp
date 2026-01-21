//--------------------------------------------------------------------------------------------------
// sat.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "sat.h"
#include "capsule.h"
#include "clipping.h"
#include "hull.h"
#include "manifold.h"


//--------------------------------------------------------------------------------------------------
// RkSATCache
//--------------------------------------------------------------------------------------------------
void rkClearCache( RkSATCache& Cache )
	{
	Cache.Type = RK_SAT_UNKNOWN;
	Cache.Index1 = -1;
	Cache.Index2 = -1;
	Cache.Separation = 0.0f;
	}


//--------------------------------------------------------------------------------------------------
// RkFaceQuery
//--------------------------------------------------------------------------------------------------
bool RkFaceQuery::IsValid() const
	{
	return FaceIndex >= 0 && VertexIndex >= 0;
	}


//--------------------------------------------------------------------------------------------------
float rkProject( const RkPlane3& Plane, const RkHull* Hull )
	{
	int Index = Hull->FindSupportVertex( -Plane.Normal );
	RkVector3 Support = Hull->GetPosition( Index );

	return rkDistance( Plane, Support );
	}


//--------------------------------------------------------------------------------------------------
void rkQueryFaceDirections( RkFaceQuery& Out, const RkTransform& CapsuleTransform, const RkCapsule& Capsule, const RkTransform& HullTransform, const RkHull* Hull )
	{
	// We perform all computations in the local space of the capsule
	RkTransform Transform = rkTMul( CapsuleTransform, HullTransform );

	int MaxFaceIndex = -1;
	int MaxVertexIndex = -1;
	float MaxFaceSeparation = -RK_F32_MAX;

	for ( int FaceIndex = 0; FaceIndex < Hull->FaceCount; ++FaceIndex )
		{
		RkPlane3 Plane = Transform * Hull->GetPlane( FaceIndex );

		int VertexIndex = Capsule.FindSupportVertex( -Plane.Normal );
		RkVector3 Support = Capsule.GetCenter( VertexIndex );
		float Separation = rkDistance( Plane, Support );

		if ( Separation > MaxFaceSeparation )
			{
			MaxVertexIndex = VertexIndex;
			MaxFaceIndex = FaceIndex;
			MaxFaceSeparation = Separation;
			}
		}

	Out.FaceIndex = MaxFaceIndex;
	Out.VertexIndex = MaxVertexIndex;
	Out.Separation = MaxFaceSeparation;
	RK_ASSERT( Out.IsValid() );
	}


//--------------------------------------------------------------------------------------------------
void rkQueryFaceDirections( RkFaceQuery& Out, const RkTransform& Transform1, const RkHull* Hull1, const RkTransform& Transform2, const RkHull* Hull2 )
	{
	// We perform all computations in the local space of the second hull
	RkTransform Transform = rkTMul( Transform2, Transform1 );

	int MaxFaceIndex = -1;
	int MaxVertexIndex = -1;
	float MaxFaceSeparation = -RK_F32_MAX;
	
	for ( int FaceIndex = 0; FaceIndex < Hull1->FaceCount; ++FaceIndex )
		{
		RkPlane3 Plane = Transform * Hull1->GetPlane( FaceIndex );

		int VertexIndex = Hull2->FindSupportVertex( -Plane.Normal );
		RkVector3 Support = Hull2->GetPosition( VertexIndex );
		float Separation = rkDistance( Plane, Support );
		
		if ( Separation > MaxFaceSeparation )
			{
			MaxVertexIndex = VertexIndex;
			MaxFaceIndex = FaceIndex;
			MaxFaceSeparation = Separation;
			}
		}

	Out.FaceIndex = MaxFaceIndex;
	Out.VertexIndex = MaxVertexIndex;
	Out.Separation = MaxFaceSeparation;
	RK_ASSERT( Out.IsValid() );
	}


//--------------------------------------------------------------------------------------------------
// Edge queries
//--------------------------------------------------------------------------------------------------
bool rkIsMinkowskiFace( const RkVector3& A, const RkVector3& B, const RkVector3& N )
	{
	// An isolated edge (e.g. like in a capsule) defines a circle through the
	// origin on the Gauss map. So testing for overlap between this circle and
	// the arc AB simplifies to a simple plane test.
	float AN = rkDot( A, N );
	float BN = rkDot( B, N );

	return AN * BN <= 0.0f;
	}


//--------------------------------------------------------------------------------------------------
bool rkIsMinkowskiFace( const RkVector3& A, const RkVector3& B, const RkVector3& B_x_A, const RkVector3& C, const RkVector3& D, const RkVector3& D_x_C )
	{
	// Two edges build a face on the Minkowski sum if the associated arcs AB and CD intersect on the Gauss map. 
	// The associated arcs are defined by the adjacent face normals of each edge.  
	float CBA = rkDot( C, B_x_A );
	float DBA = rkDot( D, B_x_A );
	float ADC = rkDot( A, D_x_C );
	float BDC = rkDot( B, D_x_C );

	return CBA * DBA < 0.0f && ADC * BDC < 0.0f && CBA * BDC > 0.0f;
	}


//--------------------------------------------------------------------------------------------------
bool RkEdgeQuery::IsValid() const
	{
	return Index1 >= 0 && Index2 >= 0;
	}


//--------------------------------------------------------------------------------------------------
float rkProject( const RkVector3& P1, const RkVector3& E1, const RkVector3& P2, const RkVector3& E2, const RkVector3& C1 )
	{
	// Build search direction
	RkVector3 E1_x_E2 = rkCross( E1, E2 );
	float Length = rkLength( E1_x_E2 );

	// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
	const float kTolerance = 0.005f;
	if ( Length < kTolerance * rkSqrt( rkLengthSq( E1 ) * rkLengthSq( E2 ) ) )
		{
		return -RK_F32_MAX;
		}

	// Make sure normal points away from the first shape
	RK_ASSERT( Length * Length > 1000.0f * RK_F32_MIN );
	RkVector3 N = E1_x_E2 / Length;
	if ( rkDot( N, P1 - C1 ) < 0.0f )
		{
		N = -N;
		}

	// s = Dot(n, p2) - d = Dot(n, p2) - Dot(n, p1) = Dot(n, p2 - p1) 
	return rkDot( N, P2 - P1 );
	}


//--------------------------------------------------------------------------------------------------
void rkQueryEdgeDirections( RkEdgeQuery& Out, const RkTransform& CapsuleTransform, const RkCapsule& Capsule, const RkTransform& HullTransform, const RkHull* Hull )
	{
	// Find axis of minimum penetration
	float MaxSeparation = -RK_F32_MAX;
	int MaxIndex1 = -1;
	int MaxIndex2 = -1;
	
	// We perform all computations in the local space of the hull
	RkTransform Transform = rkTMul( HullTransform, CapsuleTransform );
	RkVector3 P1 = Transform * Capsule.Center1;
	RkVector3 Q1 = Transform * Capsule.Center2;
	RkVector3 E1 = Q1 - P1;
	
	for ( int EdgeIndex = 0; EdgeIndex < Hull->EdgeCount; EdgeIndex += 2 )
		{
		const RkHalfEdge* Edge = Hull->GetEdge( EdgeIndex + 0 );
		const RkHalfEdge* Twin = Hull->GetEdge( EdgeIndex + 1 );
		RK_ASSERT( Edge->Twin == EdgeIndex + 1 && Twin->Twin == EdgeIndex );

		RkVector3 P2 = Hull->PositionList[ Edge->Origin ];
		RkVector3 Q2 = Hull->PositionList[ Twin->Origin ];
		RkVector3 E2 = Q2 - P2;

		RkVector3 U2 = Hull->PlaneList[ Edge->Face ].Normal;
		RkVector3 V2 = Hull->PlaneList[ Twin->Face ].Normal;

		if ( rkIsMinkowskiFace( U2, V2, E1 ) )
			{
			// We can pass any point on the edge and choose 
			// the edge centers for better numerical precision. 
			float Separation = rkProject( Q2, E2, Q1, E1, Hull->Center );
			if ( Separation > MaxSeparation )
				{
				// Note: We don't exit early if we find a separating axis here since we want to 
				// find the best one for caching and account for the convex radius later.
				MaxSeparation = Separation;
				MaxIndex1 = 0;
				MaxIndex2 = EdgeIndex;
				}
			}
		}

	// Save result
	Out.Index1 = MaxIndex1;
	Out.Index2 = MaxIndex2;
	Out.Separation = MaxSeparation;
	}


//--------------------------------------------------------------------------------------------------
void rkQueryEdgeDirections( RkEdgeQuery& Out, const RkTransform& Transform1, const RkHull* Hull1, const RkTransform& Transform2, const RkHull* Hull2 )
	{
	// Find axis of minimum penetration
	float MaxSeparation = -RK_F32_MAX;
	int MaxIndex1 = -1;
	int MaxIndex2 = -1;
	
	// We perform all computations in local space of the second hull
	RkTransform Transform = rkTMul( Transform2, Transform1 );

	for ( int EdgeIndex1 = 0; EdgeIndex1 < Hull1->EdgeCount; EdgeIndex1 += 2 )
		{
		const RkHalfEdge* Edge1 = Hull1->GetEdge( EdgeIndex1 + 0 );
		const RkHalfEdge* Twin1 = Hull1->GetEdge( EdgeIndex1 + 1 );
		RK_ASSERT( Edge1->Twin == EdgeIndex1 + 1 && Twin1->Twin == EdgeIndex1 );

		RkVector3 P1 = Transform * Hull1->PositionList[ Edge1->Origin ];
		RkVector3 Q1 = Transform * Hull1->PositionList[ Twin1->Origin ];
		RkVector3 E1 = Q1 - P1;

		RkVector3 U1 = Transform.Rotation * Hull1->PlaneList[ Edge1->Face ].Normal;
		RkVector3 V1 = Transform.Rotation * Hull1->PlaneList[ Twin1->Face ].Normal;

		for ( int EdgeIndex2 = 0; EdgeIndex2 < Hull2->EdgeCount; EdgeIndex2 += 2 )
			{
			const RkHalfEdge* Edge2 = Hull2->GetEdge( EdgeIndex2 + 0 );
			const RkHalfEdge* Twin2 = Hull2->GetEdge( EdgeIndex2 + 1 );
			RK_ASSERT( Edge2->Twin == EdgeIndex2 + 1 && Twin2->Twin == EdgeIndex2 );

			RkVector3 P2 = Hull2->PositionList[ Edge2->Origin ];
			RkVector3 Q2 = Hull2->PositionList[ Twin2->Origin ];
			RkVector3 E2 = Q2 - P2;

			RkVector3 U2 = Hull2->PlaneList[ Edge2->Face ].Normal;
			RkVector3 V2 = Hull2->PlaneList[ Twin2->Face ].Normal;

			// We only need to test edges that create a face on the Minkowski difference. For 
			// isolated edges (e.g. from a capsule) this simplifies to a simple plane test.
			if ( rkIsMinkowskiFace( U1, V1, -E1, -U2, -V2, -E2 ) )
				{
				float Separation = rkProject( Q2, E2, Q1, E1, Hull2->Center );
				if ( Separation > MaxSeparation )
					{
					// Note: We don't exit early if we find a separating axis here since we want to 
					// find the best one for caching and account for the convex radius later.
					MaxSeparation = Separation;
					MaxIndex1 = EdgeIndex1;
					MaxIndex2 = EdgeIndex2;
					}
				}
			}
		}

	// Save result
	Out.Index1 = MaxIndex1;
	Out.Index2 = MaxIndex2;
	Out.Separation = MaxSeparation;
	}


//--------------------------------------------------------------------------------------------------
// SAT Collision
//--------------------------------------------------------------------------------------------------
static float rkDeepestPointSeparation( const RkTransform& Transform1, const RkTransform& Transform2, const RkManifold& Manifold )
	{
	// Deepest point
	float MinSeparation = RK_F32_MAX;
	for ( int Index = 0; Index < Manifold.PointCount; ++Index )
		{
		const RkManifoldPoint& ManifoldPoint = Manifold.Points[ Index ];
		RkVector3 Position1 = Transform1 * ManifoldPoint.LocalPosition1;
		RkVector3 Position2 = Transform2 * ManifoldPoint.LocalPosition2;

		float Separation = rkDot( Position2 - Position1, Manifold.Normal );
		MinSeparation = rkMin( MinSeparation, Separation );
		}

	return MinSeparation;
	}


//--------------------------------------------------------------------------------------------------
static RkVector3 rkPolygonCenter( const RkClipVertex* Polygon, int VertexCount )
	{
	RkVector3 Center = RK_VEC3_ZERO;

	if ( VertexCount > 0 )
		{
		for ( int Index = 0; Index < VertexCount; ++Index )
			{
			Center += Polygon[ Index ].Position;
			}
		Center /= float( VertexCount );
		}

	return Center;
	}


//--------------------------------------------------------------------------------------------------
// static RkVector3 rkPolygonCenter( const RkArray< RkClipVertex >& Polygon )
// 	{
// 	RkVector3 Center = RK_VEC3_ZERO;
// 
// 	if ( !Polygon.Empty() )
// 		{
// 		for ( int Index = 0; Index < Polygon.Size(); ++Index )
// 			{
// 			Center += Polygon[ Index ].Position;
// 			}
// 		Center /= float( Polygon.Size() );
// 		}
// 
// 	return Center;
// 	}


//--------------------------------------------------------------------------------------------------
static int rkFindIncidentFace( const RkTransform& Transform, const RkHull* Hull, const RkPlane3& RefPlane, int VertexIndex )
	{
	RkVector3 Normal = rkTMul( Transform.Rotation, RefPlane.Normal );

	int MinEdgeIndex = -1;
	float MinEdgeProjection = RK_F32_MAX;

	const RkVertex* Vertex = Hull->GetVertex( VertexIndex );
	RK_ASSERT( Vertex );

	int EdgeIndex = Vertex->Edge;
	const RkHalfEdge* Edge = Hull->GetEdge( EdgeIndex );
	RkVector3 EdgeOrigin = Hull->GetPosition( Edge->Origin );
	RK_ASSERT( Edge->Origin == VertexIndex );

	do
		{
		const RkHalfEdge* Twin = Hull->GetEdge( Edge->Twin );
		RkVector3 TwinOrigin = Hull->GetPosition( Twin->Origin );

		RkVector3 Axis = rkNormalize( TwinOrigin - EdgeOrigin );
		float EdgeProjection = rkAbs( rkDot( Axis, Normal ) );
		if ( EdgeProjection < MinEdgeProjection )
			{
			MinEdgeIndex = EdgeIndex;
			MinEdgeProjection = EdgeProjection;
			}

		EdgeIndex = Twin->Next;
		Edge = Hull->GetEdge( EdgeIndex );
		RK_ASSERT( Edge->Origin == VertexIndex );
		} 
	while ( Edge != Hull->GetEdge( Vertex->Edge ) );
	RK_ASSERT( MinEdgeIndex >= 0 );

	const RkHalfEdge* MinEdge = Hull->GetEdge( MinEdgeIndex );
	int MinFaceIndex1 = MinEdge->Face;
	RkPlane3 MinPlane1 = Hull->GetPlane( MinFaceIndex1 );

	const RkHalfEdge* MinTwin = Hull->GetEdge( MinEdge->Twin );
	int MinFaceIndex2 = MinTwin->Face;
	RkPlane3 MinPlane2 = Hull->GetPlane( MinFaceIndex2 );

	return rkDot( MinPlane1.Normal, Normal ) < rkDot( MinPlane2.Normal, Normal ) ? MinFaceIndex1 : MinFaceIndex2;
	}


//--------------------------------------------------------------------------------------------------
static bool rkBuildFaceContact( RkManifold& Manifold, const RkTransform& CapsuleTransform, const RkCapsule& Capsule, const RkTransform& HullTransform, const RkHull* Hull, const RkFaceQuery& Query, int Triangle )
	{
	// Clip the capsule edge against the side planes of the reference face
	RK_ASSERT( Query.IsValid() );
	int RefFace = Query.FaceIndex;
	RkPlane3 RefPlane = HullTransform * Hull->PlaneList[ RefFace ];

	RkClipVertex Segment[ 2 ];
	rkBuildSegment( Segment, CapsuleTransform, Capsule );
	if ( !rkClipSegment( Segment, HullTransform, Hull, RefFace ) )
		{
		// This should really not happen!
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;

		return false;
		}

	float Separation1 = rkDistance( RefPlane, Segment[ 0 ].Position );
	float Separation2 = rkDistance( RefPlane, Segment[ 1 ].Position );
	float MinSeparation = rkMin( Separation1, Separation2 );
	float FaceSeparation = MinSeparation - Capsule.Radius - RK_CONVEX_RADIUS;
	if ( FaceSeparation > 0.0f )
		{
		// Face contact can end up empty if it does not realize the axis of minimum penetration
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;

		return false;
		}

	RkManifold Backup = Manifold;

	RkVector3 Normal = -RefPlane.Normal;
	RkVector3 Point1 = Segment[ 0 ].Position + Capsule.Radius * Normal;
	RkVector3 Point2 = Segment[ 1 ].Position + Capsule.Radius * Normal;
	RkVector3 Center = ( Point1 + Point2 ) / 2.0f;

	Manifold.Center = Center;
	Manifold.Normal = Normal;
	Manifold.Tangent1 = rkPerp( Normal );
	Manifold.Tangent2 = rkCross( Manifold.Tangent1, Normal );
	rkProjectFriction( Manifold, Backup );

	Manifold.PointCount = 0;
	if ( Separation1 <= Capsule.Radius + RK_CONVEX_RADIUS )
		{
		float Impulse1 = rkFindImpulse( Backup, Segment[ 0 ].Pair );
		Manifold.Points[ Manifold.PointCount ].LocalPosition1 = rkTMul( CapsuleTransform, Point1 );
		Manifold.Points[ Manifold.PointCount ].LocalPosition2 = rkTMul( HullTransform, Segment[ 0 ].Position + Separation1 * Normal );
		Manifold.Points[ Manifold.PointCount ].Impulse = rkMax( 0.0f, Impulse1 );
		Manifold.Points[ Manifold.PointCount ].Key = Segment[ 0 ].Pair;
		Manifold.Points[ Manifold.PointCount ].Triangle = Triangle;
		Manifold.Points[ Manifold.PointCount ].New = Impulse1 < 0.0f;
		Manifold.PointCount++;
		}
	if ( Separation2 <= Capsule.Radius + RK_CONVEX_RADIUS )
		{
		float Impulse2 = rkFindImpulse( Backup, Segment[ 1 ].Pair );
		Manifold.Points[ Manifold.PointCount ].LocalPosition1 = rkTMul( CapsuleTransform, Point2 );
		Manifold.Points[ Manifold.PointCount ].LocalPosition2 = rkTMul( HullTransform, Segment[ 1 ].Position + Separation2 * Normal );
		Manifold.Points[ Manifold.PointCount ].Impulse = rkMax( 0.0f, Impulse2 );
		Manifold.Points[ Manifold.PointCount ].Key = Segment[ 1 ].Pair;
		Manifold.Points[ Manifold.PointCount ].Triangle = Triangle;
		Manifold.Points[ Manifold.PointCount ].New = Impulse2 < 0.0f;
		Manifold.PointCount++;
		}

	RK_ASSERT( !Manifold.Empty() );
	return true;
	}


//--------------------------------------------------------------------------------------------------
static bool rkBuildFaceContact( RkManifold& Manifold, const RkTransform& Transform1, const RkHull* Hull1, const RkTransform& Transform2, const RkHull* Hull2, const RkFaceQuery& Query, bool FlipNormal, RkSATCache& Cache, int Triangle )
	{
	if ( !Query.IsValid() )
		{
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;

		rkClearCache( Cache );

		return false;
		}

	// Find incident face
	int RefFace = Query.FaceIndex;
	RkPlane3 RefPlane = Transform1 * Hull1->PlaneList[ RefFace ];

	int IncFace = rkFindIncidentFace( Transform2, Hull2, RefPlane, Query.VertexIndex );

	// Build clip polygon from incident face
	RkClipVertex InputBuffer[ 128 ];
	RkClipVertex* Input = InputBuffer;
	RkClipVertex OutputBuffer[ 128 ];
	RkClipVertex* Output = OutputBuffer;

	int VertexCount = rkBuildPolygon( Input, Transform2, Hull2, IncFace, RefPlane );

	// Clip incident face against side planes of reference face
	const RkFace* Face = Hull1->GetFace( RefFace );
	const RkHalfEdge* Edge = Hull1->GetEdge( Face->Edge );

	do
		{
		const RkHalfEdge* Next = Hull1->GetEdge( Edge->Next );
		RkVector3 Vertex1 = Transform1 * Hull1->GetPosition( Edge->Origin );
		RkVector3 Vertex2 = Transform1 * Hull1->GetPosition( Next->Origin );
		RkVector3 Tangent = rkNormalize( Vertex2 - Vertex1 );
		RkVector3 Binormal = rkCross( Tangent, RefPlane.Normal );

		RkPlane3 ClipPlane = RkPlane3( Binormal, Vertex1 );
		ClipPlane.Offset += 2.0f * RK_CONVEX_RADIUS;

		VertexCount = rkClipPolygon( Output, Input, VertexCount, ClipPlane, Hull1->GetEdgeIndex( Edge ), RefPlane );
		std::swap( Input, Output );

		if ( VertexCount < 3 )
			{
			break;
			}

		Edge = Next;
		} 
	while ( Edge != Hull1->GetEdge( Face->Edge ) );

	// Cull excessive vertices. The will slide vertices onto the reference plane
	RkClipVertex* Polygon = Input;
	VertexCount = rkCullPolygon( Polygon, VertexCount, RefPlane );

	if ( !VertexCount )
		{
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;

		rkClearCache( Cache );

		return false;
		}

	// Overwrite the manifold 
	RkManifold Backup = Manifold;

	Manifold.Center = rkPolygonCenter( Polygon, VertexCount );
	Manifold.Normal = FlipNormal ? -RefPlane.Normal : RefPlane.Normal;
	Manifold.Tangent1 = rkPerp( Manifold.Normal );
	Manifold.Tangent2 = rkCross( Manifold.Tangent1, Manifold.Normal );
	rkProjectFriction( Manifold, Backup );

	Manifold.PointCount = VertexCount;
	for ( int Index = 0; Index < VertexCount; ++Index )
		{
		// Reminder: Clip vertices are on the reference plane!
		RkVector3 Point1 = Polygon[ Index ].Position;
		RkVector3 Point2 = Polygon[ Index ].Position + Polygon[ Index ].Separation * RefPlane.Normal;
		RK_ASSERT( rkAbs( rkDot( Point2 - Point1, RefPlane.Normal ) - Polygon[ Index ].Separation ) < 100.0f * RK_F32_EPSILON );
		
		RkFeaturePair Pair = FlipNormal ? rkFlipPair( Polygon[ Index ].Pair ) : Polygon[ Index ].Pair;
		float Impulse = rkFindImpulse( Backup, Pair );

		Manifold.Points[ Index ].LocalPosition1 = FlipNormal ? rkTMul( Transform2, Point2 ) : rkTMul( Transform1, Point1 );
		Manifold.Points[ Index ].LocalPosition2 = FlipNormal ? rkTMul( Transform1, Point1 ) : rkTMul( Transform2, Point2 );
		Manifold.Points[ Index ].Impulse = rkMax( 0.0f, Impulse );
		Manifold.Points[ Index ].Key = Pair;
		Manifold.Points[ Index ].Triangle = Triangle;
		Manifold.Points[ Index ].New = Impulse < 0.0f;
		}

	// Save cache
	Cache.Type = FlipNormal ? RK_SAT_FACE2 : RK_SAT_FACE1;
	Cache.Index1 = Query.FaceIndex;
	Cache.Index2 = Query.VertexIndex;
	Cache.Separation = Query.Separation;

	return true;
	}


//--------------------------------------------------------------------------------------------------
static bool rkBuildEdgeContact( RkManifold& Manifold, const RkTransform& CapsuleTransform, const RkCapsule& Capsule, const RkTransform& HullTransform, const RkHull* Hull, const RkEdgeQuery& Query, int Triangle )
	{
	RK_ASSERT( Query.IsValid() );
	RK_ASSERT( Query.Separation <= Capsule.Radius + RK_CONVEX_RADIUS );

	RkVector3 P1 = CapsuleTransform * Capsule.Center1;
	RkVector3 Q1 = CapsuleTransform * Capsule.Center2;
	RkVector3 E1 = Q1 - P1;

	const RkHalfEdge* Edge2 = Hull->GetEdge( Query.Index2 );
	const RkHalfEdge* Twin2 = Hull->GetEdge( Edge2->Twin );
	RkVector3 C2 = HullTransform * Hull->Center;
	RkVector3 P2 = HullTransform * Hull->GetPosition( Edge2->Origin );
	RkVector3 Q2 = HullTransform * Hull->GetPosition( Twin2->Origin );
	RkVector3 E2 = Q2 - P2;

	RkVector3 Normal = rkCross( E1, E2 );
	Normal = rkNormalize( Normal );

	if ( rkDot( Normal, P2 - C2 ) > 0.0f )
		{
		Normal = -Normal;
		}

	RkClosestApproachResult Result;
	rkClosestApproachLines( Result, P1, E1, P2, E2 );

	if ( !Result.IsNormalized() )
		{
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;

		return false;
		}

	RkVector3 Point1 = Result.Point1 + Capsule.Radius * Normal;
	RkVector3 Point2 = Result.Point2;

	RkVector3 Center = Point1;
	float Separation = rkDot( Normal, Result.Point2 - Result.Point1 );
	RK_ASSERT( rkAbs( Separation - Query.Separation ) < RK_LINEAR_SLOP );

	// Overwrite the manifold 
	RkManifold Backup = Manifold;
	float Impulse = Backup.PointCount == 1 ? Manifold.Points[ 0 ].Impulse : -1.0f;

	Manifold.Center = Center;
	Manifold.Normal = Normal;
	Manifold.Tangent1 = rkPerp( Normal );
	Manifold.Tangent2 = rkCross( Manifold.Tangent1, Normal );
	rkProjectFriction( Manifold, Backup );

	Manifold.PointCount = 1;
	Manifold.Points[ 0 ].LocalPosition1 = rkTMul( CapsuleTransform, Point1 );
	Manifold.Points[ 0 ].LocalPosition2 = rkTMul( HullTransform, Point2 );
	Manifold.Points[ 0 ].Impulse = rkMax( 0.0f, Impulse );
	Manifold.Points[ 0 ].Key = 0;
	Manifold.Points[ 0 ].Triangle = Triangle;
	Manifold.Points[ 0 ].New = Impulse < 0.0f;

	return true;
	}


//--------------------------------------------------------------------------------------------------
static bool rkBuildEdgeContact( RkManifold& Manifold, const RkTransform& Transform1, const RkHull* Hull1, const RkTransform& Transform2, const RkHull* Hull2, const RkEdgeQuery& Query, RkSATCache& Cache, int Triangle )
	{
	// Edge queries can sometimes be invalid in the sense that there was no separating axis (e.g. if hulls are very small)
	if ( !Query.IsValid() )
		{
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;

		rkClearCache( Cache );

		return false;
		}

	RK_ASSERT( Query.Separation <= 2.0f * RK_CONVEX_RADIUS );
	const RkHalfEdge* Edge1 = Hull1->GetEdge( Query.Index1 );
	const RkHalfEdge* Twin1 = Hull1->GetEdge( Edge1->Twin );
	RkVector3 C1 = Transform1 * Hull1->Center;
	RkVector3 P1 = Transform1 * Hull1->GetPosition( Edge1->Origin );
	RkVector3 Q1 = Transform1 * Hull1->GetPosition( Twin1->Origin );
	RkVector3 E1 = Q1 - P1;

	const RkHalfEdge* Edge2 = Hull2->GetEdge( Query.Index2 );
	const RkHalfEdge* Twin2 = Hull2->GetEdge( Edge2->Twin );
	RkVector3 P2 = Transform2 * Hull2->GetPosition( Edge2->Origin );
	RkVector3 Q2 = Transform2 * Hull2->GetPosition( Twin2->Origin );
	RkVector3 E2 = Q2 - P2;

	RkVector3 Normal = rkCross( E1, E2 );
	Normal = rkNormalize( Normal );

	if ( rkDot( Normal, P1 - C1 ) < 0.0f )
		{
		Normal = -Normal;
		}

	RkClosestApproachResult Result;
	rkClosestApproachLines( Result, P1, E1, P2, E2 );

	if ( !Result.IsNormalized() )
		{
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;

		rkClearCache( Cache );

		return false;
		}

	float Separation = rkDot( Normal, Result.Point2 - Result.Point1 );
	RK_ASSERT( rkAbs( Separation - Query.Separation ) < RK_LINEAR_SLOP );

	// Overwrite the manifold 
	RkManifold Backup = Manifold;
	float Impulse = Backup.PointCount == 1 ? Manifold.Points[ 0 ].Impulse : -1.0f;

	RkVector3 Point1 = Result.Point1;
	RkVector3 Point2 = Result.Point2;

	Manifold.Center = Point1;
	Manifold.Normal = Normal;
	Manifold.Tangent1 = rkPerp( Normal );
	Manifold.Tangent2 = rkCross( Manifold.Tangent1, Normal );
	rkProjectFriction( Manifold, Backup );

	Manifold.PointCount = 1;
	Manifold.Points[ 0 ].LocalPosition1 = rkTMul( Transform1, Point1 );
	Manifold.Points[ 0 ].LocalPosition2 = rkTMul( Transform2, Point2 );
	Manifold.Points[ 0 ].Impulse = rkMax( 0.0f, Impulse );
	Manifold.Points[ 0 ].Key = 0;
	Manifold.Points[ 0 ].Triangle = Triangle;
	Manifold.Points[ 0 ].New = Impulse < 0.0f;

	// Save cache
	Cache.Type = RK_SAT_EDGES;
	Cache.Index1 = Query.Index1;
	Cache.Index2 = Query.Index2;
	Cache.Separation = Query.Separation;

	return true;
	}


//--------------------------------------------------------------------------------------------------
static bool rkQueryLastFeatures( RkManifold& Out, const RkTransform& Transform1, const RkHull* Hull1, const RkTransform& Transform2, const RkHull* Hull2, RkSATCache& Cache, int Triangle )
	{
	switch ( Cache.Type )
		{
		case RK_SAT_FACE1:
			{
			// We perform all computations in local space of the second hull
			RkTransform Transform = rkTMul( Transform2, Transform1 );
			RkPlane3 Plane = Transform * Hull1->GetPlane( Cache.Index1 );
			int VertexIndex = Hull2->FindSupportVertex( -Plane.Normal );
			RkVector3 Support = Hull2->GetPosition( VertexIndex );
			float Separation = rkDistance( Plane, Support );

			// DIRK_TODO: With GJK based manifolds we could be more *strictly* and
			// just measure the separation of the last face and deepest vertex. This 
			// might help with creating false contacts from the cache. E.g.
			// float Separation = rkDistance( Plane, Hull2->GetPosition( Cache.Index2 );

			if ( Separation > 2.0f * RK_CONVEX_RADIUS )
				{
				// We found a separating axis
				Out.PointCount = 0;
				Out.AngularFrictionImpulse = 0.0f;
				Out.LinearFrictionImpulse = RK_VEC2_ZERO;

				// Update cache
				Cache.Index2 = VertexIndex;
				Cache.Separation = Separation;

				return true;
				}

			if ( Cache.Separation > 2.0f * RK_CONVEX_RADIUS )
				{
				// Objects were separated and are now potentially penetrating - cache is invalidated
				rkClearCache( Cache );
				return false;
				}

			if ( rkAbs( Cache.Separation - Separation ) < 0.5f * RK_LINEAR_SLOP )
				{
				// Try to rebuild contact from last separating plane and new deepest point.
				// The deepest point can change when we roll over the surface (e.g. cylinder).
				RkFaceQuery FaceQuery;
				FaceQuery.FaceIndex = Cache.Index1;
				FaceQuery.VertexIndex = VertexIndex;
				FaceQuery.Separation = Separation;

				// We need to run a full to test if we clipped all contact points away (e.g. sliding off the face).
				// Also pass a copy of the cache so we do not shift it forward - we are trying to reconstruct here!
				RkSATCache LocalCache = Cache;
				if ( rkBuildFaceContact( Out, Transform1, Hull1, Transform2, Hull2, FaceQuery, false, LocalCache, Triangle ) )
					{
					// Cache hit, contact points generated
					RK_ASSERT( !Out.Empty() );
					return true;
					}

				// Failure: Invalidate cache and fall through
				rkClearCache( Cache );
				}
			}
			break;

		case RK_SAT_FACE2:
			{
			// We perform all computations in local space of the first hull
			RkTransform Transform = rkTMul( Transform1, Transform2 );
			RkPlane3 Plane = Transform * Hull2->GetPlane( Cache.Index1 );
			int VertexIndex = Hull1->FindSupportVertex( -Plane.Normal );
			RkVector3 Support = Hull1->GetPosition( VertexIndex );
			float Separation = rkDistance( Plane, Support );

			if ( Separation > 2.0f * RK_CONVEX_RADIUS )
				{
				// We found a separating axis
				Out.PointCount = 0;
				Out.AngularFrictionImpulse = 0.0f;
				Out.LinearFrictionImpulse = RK_VEC2_ZERO;

				// Update cache
				Cache.Index2 = VertexIndex;
				Cache.Separation = Separation;

				return true;
				}

			if ( Cache.Separation > 2.0f * RK_CONVEX_RADIUS )
				{
				// Objects were separated and are now potentially penetrating - cache is invalidated
				rkClearCache( Cache );
				return false;
				}

			if ( rkAbs( Cache.Separation - Separation ) < 0.5f * RK_LINEAR_SLOP )
				{
				// Try to rebuild contact from last separating plane and new deepest point.
				// The deepest point can change when we roll over the surface (e.g. cylinder).
				RkFaceQuery FaceQuery;
				FaceQuery.FaceIndex = Cache.Index1;
				FaceQuery.VertexIndex = VertexIndex;
				FaceQuery.Separation = Separation;

				// We need to run a full to test if we clipped all contact points away (e.g. sliding off the face). 
				// Also pass a copy of the cache so we do not shift it forward - we are trying to reconstruct here!
				RkSATCache LocalCache = Cache;
				if ( rkBuildFaceContact( Out, Transform2, Hull2, Transform1, Hull1, FaceQuery, true, LocalCache, Triangle ) )
					{
					// Cache hit, contact points generated
					RK_ASSERT( !Out.Empty() );
					return true;
					}

				// Failure: Invalidate cache and fall through
				rkClearCache( Cache );
				}
			}
			break;

		case RK_SAT_EDGES:
			{
			// We perform all computations in local space of the second hull
			RkTransform Transform = rkTMul( Transform2, Transform1 );

			int Index1 = Cache.Index1;
			const RkHalfEdge* Edge1 = Hull1->GetEdge( Index1 );
			const RkHalfEdge* Twin1 = Hull1->GetEdge( Index1 + 1 );
			RK_ASSERT( Edge1->Twin == Index1 + 1 && Twin1->Twin == Index1 );

			RkVector3 P1 = Transform * Hull1->PositionList[ Edge1->Origin ];
			RkVector3 Q1 = Transform * Hull1->PositionList[ Twin1->Origin ];
			RkVector3 E1 = Q1 - P1;

			RkVector3 U1 = Transform.Rotation * Hull1->PlaneList[ Edge1->Face ].Normal;
			RkVector3 V1 = Transform.Rotation * Hull1->PlaneList[ Twin1->Face ].Normal;

			int Index2 = Cache.Index2;
			const RkHalfEdge* Edge2 = Hull2->GetEdge( Index2 );
			const RkHalfEdge* Twin2 = Hull2->GetEdge( Index2 + 1 );
			RK_ASSERT( Edge2->Twin == Index2 + 1 && Twin2->Twin == Index2 );

			RkVector3 P2 = Hull2->PositionList[ Edge2->Origin ];
			RkVector3 Q2 = Hull2->PositionList[ Twin2->Origin ];
			RkVector3 E2 = Q2 - P2;

			RkVector3 U2 = Hull2->PlaneList[ Edge2->Face ].Normal;
			RkVector3 V2 = Hull2->PlaneList[ Twin2->Face ].Normal;

			if ( rkIsMinkowskiFace( U1, V1, -E1, -U2, -V2, -E2 ) )
				{
				float Separation = rkProject( Q2, E2, Q1, E1, Hull2->Center );
				if ( Separation > 2.0f * RK_CONVEX_RADIUS )
					{
					// We found a separating axis
					Out.PointCount = 0;
					Out.AngularFrictionImpulse = 0.0f;
					Out.LinearFrictionImpulse = RK_VEC2_ZERO;

					// Update cache
					Cache.Separation = Separation;

					return true;
					}

				if ( Cache.Separation > 2.0f * RK_CONVEX_RADIUS )
					{
					// Objects were separated and are now potentially penetrating
					return false;
					}

				if ( rkAbs( Cache.Separation - Separation ) < 0.5f * RK_LINEAR_SLOP )
					{
					// Try to rebuild contact from last features
					RkEdgeQuery EdgeQuery;
					EdgeQuery.Index1 = Cache.Index1;
					EdgeQuery.Index2 = Cache.Index2;
					EdgeQuery.Separation = Separation;

					// We need to run a full test if we clipped all contact points away (e.g. edges sliding off each other). 
					// Also pass a copy of the cache so we do not shift it forward - we are trying to reconstruct here!
					RkSATCache LocalCache = Cache;
					if ( rkBuildEdgeContact( Out, Transform1, Hull1, Transform2, Hull2, EdgeQuery, LocalCache, Triangle ) )
						{
						// Cache hit, contact points generated
						RK_ASSERT( !Out.Empty() );
						return true;
						}

					// Failure: Invalidate cache and fall through
					rkClearCache( Cache );
					}
				}
			}
			break;

		default:
			break;
		}

	return false;
	}


//--------------------------------------------------------------------------------------------------
void rkCollide( RkManifold& Manifold, const RkTransform& CapsuleTransform, const RkCapsule& Capsule, const RkTransform& HullTransform, const RkHull* Hull, int Triangle )
	{
	// SAT - face separation
	RkFaceQuery FaceQuery;
	rkQueryFaceDirections( FaceQuery, CapsuleTransform, Capsule, HullTransform, Hull );
	if ( FaceQuery.Separation > Capsule.Radius + RK_CONVEX_RADIUS ) 
		{
		// We found a separating axis
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;

		return;
		}

	// SAT - edge separation
	RkEdgeQuery EdgeQuery;
	rkQueryEdgeDirections( EdgeQuery, CapsuleTransform, Capsule, HullTransform, Hull );
	if ( EdgeQuery.Separation > Capsule.Radius + RK_CONVEX_RADIUS )
		{
		// We found a separating axis
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;

		return;
		}

	// Create face contact
	RkManifold OldManifold = Manifold;

	float FaceSeparation = FaceQuery.Separation - Capsule.Radius - RK_CONVEX_RADIUS;
	rkBuildFaceContact( Manifold, CapsuleTransform, Capsule, HullTransform, Hull, FaceQuery, Triangle );
	if ( Manifold.PointCount > 1 )
		{
		// If ( Out.PointCount <= 1 ) -> Compare with unclipped separation 
		// If ( Out.PointCount > 1 ) -> Be aggresive and compare with clipped separation 
		FaceSeparation = rkDeepestPointSeparation( CapsuleTransform, HullTransform, Manifold ) - RK_CONVEX_RADIUS;
		}
	RK_ASSERT( FaceSeparation <= 0.0f );

	// Face contact can be empty if it does not realize the axis of minimum penetration
	// Create edge contact if face contact failed or edge contact is significantly better!
	const float kRelTolerance = 0.98f;
	const float kAbsTolerance = 0.5f * RK_LINEAR_SLOP;
	float EdgeSeparation = EdgeQuery.Separation - Capsule.Radius - RK_CONVEX_RADIUS;
	if ( Manifold.Empty() || EdgeSeparation > kRelTolerance * FaceSeparation + kAbsTolerance )
		{
		// Edge contact - reset to old manifold so we don't loose warm-starting information!
		RkManifold Backup = Manifold;
		Manifold = OldManifold;

		rkBuildEdgeContact( Manifold, CapsuleTransform, Capsule, HullTransform, Hull, EdgeQuery, Triangle );
		if ( Manifold.Empty() )
			{
			// Use face manifold if we fail to create edge contact!
			Manifold = Backup;
			}
		}
	}


//--------------------------------------------------------------------------------------------------
void rkCollide( RkManifold& Manifold, const RkTransform& Transform1, const RkHull* Hull1, const RkTransform& Transform2, const RkHull* Hull2, RkSATCache& Cache, int Triangle )
	{
	// Query last features
	if ( rkQueryLastFeatures( Manifold, Transform1, Hull1, Transform2, Hull2, Cache, Triangle ) )
		{
		// SUCCESS: We rebuilt the manifold from the feature cache and can exit early!
		return;
		}

	// Find axis of minimum penetration
	RkFaceQuery FaceQuery1;
	rkQueryFaceDirections( FaceQuery1, Transform1, Hull1, Transform2, Hull2 );
	if ( FaceQuery1.Separation > 2.0f * RK_CONVEX_RADIUS )	
		{
		// We found a separating axis
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;
		
		Cache.Type = RK_SAT_FACE1;
		Cache.Index1 = FaceQuery1.FaceIndex;
		Cache.Index2 = FaceQuery1.VertexIndex;
		Cache.Separation = FaceQuery1.Separation;

		return;
		}

	RkFaceQuery FaceQuery2;
	rkQueryFaceDirections( FaceQuery2, Transform2, Hull2, Transform1, Hull1 );
	if ( FaceQuery2.Separation > 2.0f * RK_CONVEX_RADIUS )	
		{
		// We found a separating axis
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;
		
		Cache.Type = RK_SAT_FACE2;
		Cache.Index1 = FaceQuery2.FaceIndex;
		Cache.Index2 = FaceQuery2.VertexIndex;
		Cache.Separation = FaceQuery2.Separation;

		return;
		}

	RkEdgeQuery EdgeQuery;
	rkQueryEdgeDirections( EdgeQuery, Transform1, Hull1, Transform2, Hull2 );
	if ( EdgeQuery.Separation > 2.0f * RK_CONVEX_RADIUS )
		{
		// We found a separating axis
		Manifold.PointCount = 0;
		Manifold.AngularFrictionImpulse = 0.0f;
		Manifold.LinearFrictionImpulse = RK_VEC2_ZERO;

		Cache.Type = RK_SAT_EDGES;
		Cache.Index1 = EdgeQuery.Index1;
		Cache.Index2 = EdgeQuery.Index2;
		Cache.Separation = EdgeQuery.Separation;

		return;
		}

	// Build contact
	RkManifold OldManifold = Manifold;

	const float kRelEdgeTolerance = 0.90f;
	const float kRelFaceTolerance = 0.98f;
	const float kAbsTolerance = 0.5f * RK_LINEAR_SLOP;

	float FaceSeparation1 = FaceQuery1.Separation - 2.0f * RK_CONVEX_RADIUS;
	float FaceSeparation2 = FaceQuery2.Separation - 2.0f * RK_CONVEX_RADIUS;
	RK_ASSERT( FaceSeparation1 <= 0.0f && FaceSeparation2 <= 0.0f );

	if ( FaceSeparation2 > kRelFaceTolerance * FaceSeparation1 + kAbsTolerance )
		{
		// Face contact (2)
		rkBuildFaceContact( Manifold, Transform2, Hull2, Transform1, Hull1, FaceQuery2, true, Cache, Triangle );
		}
	else
		{
		// Face contact (1)
		rkBuildFaceContact( Manifold, Transform1, Hull1, Transform2, Hull2, FaceQuery1, false, Cache, Triangle );
		}
	
	// If ( Out.PointCount <= 1 ) -> Compare with unclipped separation 
	// If ( Out.PointCount > 1 ) -> Be aggresive and compare with clipped separation 
	float FaceSeparation = rkMax( FaceSeparation1, FaceSeparation2 );
	if ( Manifold.PointCount > 1 )
		{
		// Th manifold point separation contains the convex radius. 
		// We need to undo this here to correctly compare to the queries! 
		FaceSeparation = rkDeepestPointSeparation( Transform1, Transform2, Manifold ) - 2.0f * RK_CONVEX_RADIUS;
		}
	RK_ASSERT( FaceSeparation <= 0.0f );

	float EdgeSeparation = EdgeQuery.Separation - 2.0f * RK_CONVEX_RADIUS;
	RK_ASSERT( EdgeSeparation <= 0.0f );

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	if ( Manifold.Empty() || EdgeSeparation > kRelEdgeTolerance * FaceSeparation + kAbsTolerance )
		{
		// Edge contact - reset to old manifold so we don't loose warm-starting information!
		RkManifold Backup = Manifold;
		Manifold = OldManifold;

		rkBuildEdgeContact( Manifold, Transform1, Hull1, Transform2, Hull2, EdgeQuery, Cache, Triangle );
		if ( Manifold.Empty() )
			{
			// Use face manifold if we fail to create edge contact!
			Manifold = Backup;
			}
		}	
	}