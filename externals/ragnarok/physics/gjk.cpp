//--------------------------------------------------------------------------------------------------
// gjk.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "gjk.h"
#include "gjksimplex.h"
#include "hull.h"
#include "sat.h"


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static void rkClean( RkGJKCache& Cache )
	{
	for ( int VertexIndex = Cache.VertexCount - 1; VertexIndex >= 0; --VertexIndex )
		{
		RK_ASSERT( Cache.Lambdas[ VertexIndex ] >= 0.0f );
		if ( Cache.Lambdas[ VertexIndex ] < 1000.0f * RK_F32_EPSILON )
			{
			Cache.Lambdas[ VertexIndex ] = Cache.Lambdas[ Cache.VertexCount - 1 ];
			Cache.Vertices1[ VertexIndex ] = Cache.Vertices1[ Cache.VertexCount - 1 ];
			Cache.Vertices2[ VertexIndex ] = Cache.Vertices2[ Cache.VertexCount - 1 ];

			Cache.VertexCount--;
			}
		}
	}


//--------------------------------------------------------------------------------------------------
static int rkUniqueCount( int VertexCount, uint8* Vertices )
	{
	if ( VertexCount == 1 )
		{
		return VertexCount;
		}

	if ( VertexCount == 2 )
		{
		return Vertices[ 0 ] != Vertices[ 1 ] ? 2 : 1;
		}

	if ( VertexCount == 3 )
		{
		if ( Vertices[ 0 ] == Vertices[ 1 ] )
			{
			Vertices[ 1 ] = Vertices[ 2 ];

			return Vertices[ 0 ] != Vertices[ 1 ] ? 2 : 1;
			}
		if ( Vertices[ 0 ] == Vertices[ 2 ] )
			{
			return Vertices[ 0 ] != Vertices[ 1 ] ? 2 : 1;
			}
		if ( Vertices[ 1 ] == Vertices[ 2 ] )
			{
			return Vertices[ 0 ] != Vertices[ 1 ] ? 2 : 1;
			}
		}

	return 3;
	}


//--------------------------------------------------------------------------------------------------
static bool rkContainsVertex( int FaceIndex, int VertexIndex, const RkHull* Hull )
	{
	RK_ASSERT( Hull );
	const RkFace* Face = Hull->GetFace( FaceIndex );
	RK_ASSERT( Face );

	const RkHalfEdge* Edge = Hull->GetEdge( Face->Edge );
	do
		{
		if ( Edge->Origin == VertexIndex )
			{
			return true;
			}

		Edge = Hull->GetEdge( Edge->Next );
		} while ( Edge != Hull->GetEdge( Face->Edge ) );

	return false;
	}


//--------------------------------------------------------------------------------------------------
static int rkFindEdge( int VertexIndex1, int VertexIndex2, const RkHull* Hull )
	{
	RK_ASSERT( Hull );
	const RkVertex* Vertex = Hull->GetVertex( VertexIndex1 );
	RK_ASSERT( Vertex );

	const RkHalfEdge* Edge = Hull->GetEdge( Vertex->Edge );
	do
		{
		const RkHalfEdge* Twin = Hull->GetEdge( Edge->Twin );
		if ( Twin->Origin == VertexIndex2 )
			{
			return Hull->GetEdgeIndex( Edge );
			}

		Edge = Hull->GetEdge( Twin->Next );
		} while ( Edge != Hull->GetEdge( Vertex->Edge ) );

	// This is unlikely, but can happen if we have an edge across a face
	return -1;
	}


//--------------------------------------------------------------------------------------------------
static int rkFindFace( int VertexIndex1, int VertexIndex2, const RkHull* Hull )
	{
	RK_ASSERT( Hull );
	const RkVertex* Vertex = Hull->GetVertex( VertexIndex1 );
	RK_ASSERT( Vertex );

	const RkHalfEdge* Edge = Hull->GetEdge( Vertex->Edge );
	do
		{
		if ( rkContainsVertex( Edge->Face, VertexIndex2, Hull ) )
			{
			return Edge->Face;
			}

		const RkHalfEdge* Twin = Hull->GetEdge( Edge->Twin );
		Edge = Hull->GetEdge( Twin->Next );
		} while ( Edge != Hull->GetEdge( Vertex->Edge ) );

	return -1;
	}


//--------------------------------------------------------------------------------------------------
static int rkFindFace( int VertexIndex1, int VertexIndex2, int VertexIndex3, const RkHull* Hull )
	{
	RK_ASSERT( Hull );
	const RkVertex* Vertex = Hull->GetVertex( VertexIndex1 );
	RK_ASSERT( Vertex );

	const RkHalfEdge* Edge = Hull->GetEdge( Vertex->Edge );
	do
		{
		if ( rkContainsVertex( Edge->Face, VertexIndex2, Hull ) && rkContainsVertex( Edge->Face, VertexIndex3, Hull ) )
			{
			return Edge->Face;
			}

		const RkHalfEdge* Twin = Hull->GetEdge( Edge->Twin );
		Edge = Hull->GetEdge( Twin->Next );
		} while ( Edge != Hull->GetEdge( Vertex->Edge ) );

	return -1;
	}


//--------------------------------------------------------------------------------------------------
static RkGJKWitness rkResolveFeature( int VertexCount, const uint8* Vertices, const RkHull* Hull = nullptr )
	{
	if ( VertexCount == 1 )
		{
		// Can be everything (*sphere*, *capsule*, or *hull*)
		return { RK_GJK_VERTEX, Vertices[ 0 ] };
		}

	if ( VertexCount == 2 )
		{
		// Can be either a hull or capsule
		if ( Hull )
			{
			// *hull*
			int EdgeIndex = rkFindEdge( Vertices[ 0 ], Vertices[ 1 ], Hull );
			if ( EdgeIndex >= 0 )
				{
				return { RK_GJK_EDGE, EdgeIndex };
				}

			int FaceIndex = rkFindFace( Vertices[ 0 ], Vertices[ 1 ], Hull );
			return { RK_GJK_FACE, FaceIndex };
			}
		else
			{
			// *capsule*
			return { RK_GJK_EDGE, 0 };
			}
		}

	// Must be a *hull*
	RK_ASSERT( VertexCount == 3 );
	int FaceIndex = rkFindFace( Vertices[ 0 ], Vertices[ 1 ], Vertices[ 2 ], Hull );

	return { RK_GJK_FACE, FaceIndex };
	}


//--------------------------------------------------------------------------------------------------
// GJK proxy
//--------------------------------------------------------------------------------------------------
RkGJKProxy::RkGJKProxy( int VertexCount, const RkVector3* VertexBuffer )
	: mVertexCount( VertexCount )
	, mVertexBuffer( VertexBuffer )
	{

	}


//--------------------------------------------------------------------------------------------------
int RkGJKProxy::GetSupport( const RkVector3& Axis ) const
	{
	RK_ASSERT( mVertexCount > 0 );
	RK_ASSERT( mVertexBuffer );

	// We move the first vertex into the origin for improved precision.
	// This is necessary since we don't have shape transforms and 
	// vertices can potentially be far away from the origin (large).
	int MaxIndex = 0;
	float MaxProjection = 0.0f;

	for ( int Index = 1; Index < mVertexCount; ++Index )
		{
		// We subtract the first vertex since we are shifting into the origin.
		float Projection = rkDot( Axis, mVertexBuffer[ Index ] - mVertexBuffer[ 0 ] );
		if ( Projection > MaxProjection )
			{
			MaxIndex = Index;
			MaxProjection = Projection;
			}
		}

	RK_ASSERT( 0 <= MaxIndex && MaxIndex < mVertexCount );
	return MaxIndex;
	}


//--------------------------------------------------------------------------------------------------
// GJK (closest points)
//--------------------------------------------------------------------------------------------------
RkGJKQuery rkGJK( const RkTransform& Transform1, const RkGJKProxy& Proxy1, const RkTransform& Transform2, const RkGJKProxy& Proxy2, int MaxIterations )
	{
	RkGJKCache Cache;
	return rkGJK( Transform1, Proxy1, Transform2, Proxy2, Cache, MaxIterations );
	}


//--------------------------------------------------------------------------------------------------
RkGJKQuery rkGJK( const RkTransform& Transform1, const RkGJKProxy& Proxy1, const RkTransform& Transform2, const RkGJKProxy& Proxy2, RkGJKCache& Cache, int MaxIterations )
	{
	// Build initial simplex
	RkGJKSimplex Simplex, Backup;
	Simplex.ReadCache( Transform1, Proxy1, Transform2, Proxy2, Cache );
	
	// Keep track of squared distance to monitor progression
	float DistanceSq = RK_F32_MAX;

	// Run GJK
	for ( int Iteration = 0; Iteration < MaxIterations; ++Iteration )
		{
		// Solve simplex and check for overlap
		if ( !Simplex.Solve() )
			{
			// No progress - reconstruct last simplex
			RK_ASSERT( !Backup.IsEmpty() );
			Simplex = Backup;

			break;
			}

		if ( Simplex.GetVertexCount() == RK_MAX_SIMPLEX_VERTICES )
			{
			// Overlap!
			break;
			}

		// Assure distance progression
		float OldDistanceSq = DistanceSq;
		RkVector3 ClosestPoint = Simplex.GetClosestPoint();
		DistanceSq = rkDot( ClosestPoint, ClosestPoint );

		if ( DistanceSq >= OldDistanceSq )
			{
			// No progress - reconstruct last simplex
			RK_ASSERT( !Backup.IsEmpty() );
			Simplex = Backup;
			
			break;
			}

		// Build new tentative support point
		RkVector3 SearchDirection = Simplex.GetSearchDirection();
		if ( rkLengthSq( SearchDirection ) < 1000.0f * RK_F32_MIN )
			{
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
			}

		// Get new support points
		RkVector3 SearchDirection1 = rkTMul( Transform1.Rotation, SearchDirection );
		int Index1 = Proxy1.GetSupport( -SearchDirection1 );
		RkVector3 Support1 = Transform1 * Proxy1.GetVertex( Index1 );
		RkVector3 SearchDirection2 = rkTMul( Transform2.Rotation, SearchDirection );
		int Index2 = Proxy2.GetSupport( SearchDirection2 );
		RkVector3 Support2 = Transform2 * Proxy2.GetVertex( Index2 );

		// Save current simplex and add new vertex - this can fail if we detect cycling
		Backup = Simplex;

		if ( !Simplex.AddVertex( Index1, Support1, Index2, Support2 ) )
			{
			break;
			}
		}
	
	// Build witness points and safe cache
	RkVector3 Point1, Point2;
	Simplex.BuildWitnessPoints( Point1, Point2 );
	Simplex.WriteCache( Cache );

	RkGJKQuery Query;
	Query.Distance = rkDistance( Point1, Point2 );
	Query.Point1 = Point1;
	Query.Point2 = Point2;

	return Query;
	}


//--------------------------------------------------------------------------------------------------
RkGJKWitnessPair rkResolveCache( RkGJKCache Cache, const RkHull* Hull1, const RkHull* Hull2 )
	{
	// Phase 1: Remove vertices that do not contribute
	rkClean( Cache );

	// Phase 2: Remove duplicates and determine unique vertex counts
	int Count1 = rkUniqueCount( Cache.VertexCount, Cache.Vertices1 );
	int Count2 = rkUniqueCount( Cache.VertexCount, Cache.Vertices2 );

	// Phase 3: Resolve features
	RkGJKWitness Witness1 = rkResolveFeature( Count1, Cache.Vertices1, Hull1 );
	RkGJKWitness Witness2 = rkResolveFeature( Count2, Cache.Vertices2, Hull2 );

	return { Witness1, Witness2 };
	}
