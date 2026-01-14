//--------------------------------------------------------------------------------------------------
// toi.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "toi.h"
#include "constants.h"
#include "gjk.h"
#include "gjksimplex.h"
#include "hull.h"


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static int rkUniqueCount( int VertexCount, const uint8* Vertices )
	{
	RK_ASSERT( 1 <= VertexCount && VertexCount <= 3 );

	switch ( VertexCount )
		{
		case 1:
			{
			return 1;
			}
			break;

		case 2:
			{
			return Vertices[ 0 ] != Vertices[ 1 ] ? 2 : 1;
			}
			break;

		case 3:
			{
			if ( Vertices[ 0 ] != Vertices[ 1 ] && Vertices[ 0 ] != Vertices[ 2 ] && Vertices[ 1 ] != Vertices[ 2 ] )
				{
				// All different
				return 3;
				}
			else if ( Vertices[ 0 ] == Vertices[ 1 ] && Vertices[ 0 ] == Vertices[ 2 ] && Vertices[ 1 ] == Vertices[ 2 ] )
				{
				// All equal
				return 1;
				}
			else
				{
				return 2;
				}
			}
			break;

		default:
			RK_ASSERT( !"Should never get here!" );
			break;
		}

	return 0;
	}


//--------------------------------------------------------------------------------------------------
static inline bool rkCheckFastEdges( const RkTransform& Transform1, const RkVector3& LocalEdge1, const RkTransform& Transform2, const RkVector3& LocalEdge2, const RkVector3& Axis0 )
	{
	// By taking the local witness axes we make sure that we 
	// get the correct orientations (e.g. if one axis was flipped)!
	RkVector3 Edge1 = Transform1.Rotation * LocalEdge1;
	RkVector3 Edge2 = Transform2.Rotation * LocalEdge2;
	RkVector3 Axis = rkCross( Edge1, Edge2 );
	Axis = rkNormalize( Axis );

	return rkDot( Axis, Axis0 ) < 0.0f;
	}


//--------------------------------------------------------------------------------------------------
// RkSeparationType
//--------------------------------------------------------------------------------------------------
enum RkSeparationType
	{
	RK_SEPARATION_UNKNOWN,
	RK_SEPARATION_VERTICES,
	RK_SEPARATION_EDGES,
	RK_SEPARATION_FACE1,
	RK_SEPARATION_FACE2
	};


//--------------------------------------------------------------------------------------------------
// RkSeparationFunction 
//--------------------------------------------------------------------------------------------------
class RkSeparationFunction
	{
	public:
		RkSeparationFunction();
		RkSeparationFunction( const RkSweep& Sweep1, const RkTOIProxy* Proxy1, const RkSweep& Sweep2, const RkTOIProxy* Proxy2, const RkGJKQuery& Query, RkGJKCache Cache, float Beta );

		RkSeparationType GetType() const;

		float FindMinSeparation( int& Index1, int& Index2, float Beta ) const;
		float Evaluate( int Index1, int Index2, float Beta ) const;

		void ForceFixedAxis( float Beta );

	private:
		RkSeparationType ComputeType( const RkSweep& Sweep1, const RkTOIProxy* Proxy1, const RkSweep& Sweep2, const RkTOIProxy* Proxy2, const RkGJKQuery& Query, RkGJKCache Cache, float Beta ) const;

		RkSeparationType mType;
		RkSweep mSweep1;
		const RkTOIProxy* mProxy1;
		RkSweep mSweep2;
		const RkTOIProxy* mProxy2;

		RkVector3 mWitness1;
		RkVector3 mWitness2;
	};


//--------------------------------------------------------------------------------------------------
RkSeparationFunction::RkSeparationFunction()
	: mType( RK_SEPARATION_UNKNOWN )
	, mSweep1( RK_SWEEP_IDENTITY )
	, mProxy1( nullptr )
	, mSweep2( RK_SWEEP_IDENTITY )
	, mProxy2( nullptr )
	, mWitness1( RK_VEC3_ZERO )
	, mWitness2( RK_VEC3_ZERO )
	{

	}


//--------------------------------------------------------------------------------------------------
RkSeparationFunction::RkSeparationFunction( const RkSweep& Sweep1, const RkTOIProxy* Proxy1, const RkSweep& Sweep2, const RkTOIProxy* Proxy2, const RkGJKQuery& Query, RkGJKCache Cache, float Beta )
	: mType( RK_SEPARATION_UNKNOWN )
	, mSweep1( Sweep1 )
	, mProxy1( Proxy1 )
	, mSweep2( Sweep2 )
	, mProxy2( Proxy2 )
	, mWitness1( RK_VEC3_ZERO )
	, mWitness2( RK_VEC3_ZERO )
	{
	RkTransform Transform1 = Sweep1.Interpolate( Beta );
	RkTransform Transform2 = Sweep2.Interpolate( Beta );

	RK_ASSERT( 1 <= Cache.VertexCount && Cache.VertexCount <= 3 );
	int UniqueCount1 = rkUniqueCount( Cache.VertexCount, Cache.Vertices1 );
	int UniqueCount2 = rkUniqueCount( Cache.VertexCount, Cache.Vertices2 );

	switch ( Cache.VertexCount )
		{
		case 1:
			{
			RkVector3 Vertex1 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 0 ] );
			RkVector3 Vertex2 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 0 ] );

			mType = RK_SEPARATION_VERTICES;
			mWitness1 = rkNormalize( Vertex2 - Vertex1 );
			mWitness2 = RK_VEC3_ZERO;
			}
			break;

		case 2:
			{
			if ( UniqueCount1 == 2 && UniqueCount2 == 2 )
				{
				// Edge/Edge
				RkVector3 VertexA1 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 0 ] );
				RkVector3 VertexA2 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 1 ] );
				RkVector3 EdgeA = rkNormalize( VertexA2 - VertexA1 );
				
				RkVector3 VertexB1 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 0 ] );
				RkVector3 VertexB2 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 1 ] );
				RkVector3 EdgeB = rkNormalize( VertexB2 - VertexB1 );
				
				RkVector3 Axis = rkCross( EdgeA, EdgeB );
				float Length = rkLength( Axis );

				// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
				const float kTolerance = 0.005f;
				if ( Length < kTolerance )
					{
					// The axis is not safe to normalize so we use a world axis instead!
					mType = RK_SEPARATION_VERTICES;
					mWitness1 = rkNormalize( Query.Point2 - Query.Point1 );
					mWitness2 = RK_VEC3_ZERO;
					}
				else
					{
					Axis /= Length;
					if ( rkDot( VertexB1 - VertexA1, Axis ) < 0.0f )
						{
						Axis = -Axis;
						EdgeB = -EdgeB;
						}

					// Check for possible sign flip in edge/edge cross product
					if ( rkCheckFastEdges( Sweep1.Interpolate( 1.0f ), rkTMul( Transform1.Rotation, EdgeA ), Sweep2.Interpolate( 1.0f ), rkTMul( Transform2.Rotation, EdgeB ), Axis ) )
						{
						mType = RK_SEPARATION_VERTICES;
						mWitness1 = Axis;
						mWitness2 = RK_VEC3_ZERO;
						}
					else
						{
						mType = RK_SEPARATION_EDGES;
						mWitness1 = rkTMul( Transform1.Rotation, EdgeA );
						mWitness2 = rkTMul( Transform2.Rotation, EdgeB );
						}
					}
				}
			else if ( UniqueCount1 == 2 )
				{
				// Mimic the GJK search direction for vertex/edge
				RkVector3 A = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 0 ] );
				RkVector3 B = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 1 ] );
				RkVector3 Q = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 0 ] );
				
				RkVector3 AB = B - A;
				RkVector3 AQ = Q - A;
				RkVector3 N = rkCross( rkCross( AB, AQ ), AB );
				
				mType = RK_SEPARATION_VERTICES;
				mWitness1 = rkNormalize( N );
				mWitness2 = RK_VEC3_ZERO;
				}
			else
				{
				// Mimic the GJK search direction for vertex/edge
				RkVector3 A = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 0 ] );
				RkVector3 B = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 1 ] );
				RkVector3 Q = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 0 ] );
				
				RkVector3 AB = B - A;
				RkVector3 AQ = Q - A;
				RkVector3 N = rkCross( rkCross( AB, AQ ), AB );

				mType = RK_SEPARATION_VERTICES;
				mWitness1 = rkNormalize( -N );
				mWitness2 = RK_VEC3_ZERO;

				}
			}
			break;
		
		case 3:
			{
			if ( UniqueCount1 == 3 )
				{
				RkVector3 VertexA1 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 0 ] );
				RkVector3 VertexA2 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 1 ] );
				RkVector3 VertexA3 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 2 ] );
				RkVector3 Axis = rkCross( VertexA2 - VertexA1, VertexA3 - VertexA1 );
				Axis = rkNormalize( Axis );

				RkVector3 Point1 = ( VertexA1 + VertexA2 + VertexA3 ) / 3.0f;
				RkVector3 Point2 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 0 ] );

				if ( rkDot( Point2 - Point1, Axis ) < 0.0f )
					{
					Axis = -Axis;
					}

				mType = RK_SEPARATION_FACE1;
				mWitness1 = rkTMul( Transform1.Rotation, Axis );
				mWitness2 = rkTMul( Transform1, Point1 );
				}
			else if ( UniqueCount2 == 3 )
				{
				RkVector3 VertexB1 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 0 ] );
				RkVector3 VertexB2 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 1 ] );
				RkVector3 VertexB3 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 2 ] );
				RkVector3 Axis = rkCross( VertexB2 - VertexB1, VertexB3 - VertexB1 );
				Axis = rkNormalize( Axis );

				RkVector3 Point1 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 0 ] );
				RkVector3 Point2 = ( VertexB1 + VertexB2 + VertexB3 ) / 3.0f;

				if ( rkDot( Point1 - Point2, Axis ) < 0.0f )
					{
					Axis = -Axis;
					}

				mType = RK_SEPARATION_FACE2;
				mWitness1 = rkTMul( Transform2.Rotation, Axis );
				mWitness2 = rkTMul( Transform2, Point2);
				}
			else
				{
				RK_ASSERT( UniqueCount1 == 2 && UniqueCount2 == 2 );

				if ( Cache.Vertices1[ 0 ] == Cache.Vertices1[ 1 ] )
					{
					std::swap( Cache.Vertices1[ 1 ], Cache.Vertices1[ 2 ] );
					RK_ASSERT( Cache.Vertices1[ 0 ] != Cache.Vertices1[ 1 ] );
					}
				RkVector3 VertexA1 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 0 ] );
				RkVector3 VertexA2 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 1 ] );
				RkVector3 EdgeA = rkNormalize( VertexA2 - VertexA1 );

				if ( Cache.Vertices2[ 0 ] == Cache.Vertices2[ 1 ] )
					{
					std::swap( Cache.Vertices2[ 1 ], Cache.Vertices2[ 2 ] );
					RK_ASSERT( Cache.Vertices2[ 0 ] != Cache.Vertices2[ 1 ] );
					}
				RkVector3 VertexB1 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 0 ] );
				RkVector3 VertexB2 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 1 ] );
				RkVector3 EdgeB = rkNormalize( VertexB2 - VertexB1 );

				RkVector3 Axis = rkCross( EdgeA, EdgeB );
				float Length = rkLength( Axis );
				
				// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
				const float kTolerance = 0.005f;
				if ( Length < kTolerance )
					{
					// The axis is not safe to normalize so we use a world axis instead!
					mType = RK_SEPARATION_VERTICES;
					mWitness1 = rkNormalize( Query.Point2 - Query.Point1 );
					mWitness2 = RK_VEC3_ZERO;
					}
				else
					{
					Axis /= Length;
					if ( rkDot( VertexB1 - VertexA1, Axis ) < 0.0f )
						{
						Axis = -Axis;
						EdgeB = -EdgeB;
						}

					// Check for possible sign flip in edge/edge cross product
					if ( rkCheckFastEdges( Sweep1.Interpolate( 1.0f ), rkTMul( Transform1.Rotation, EdgeA ), Sweep2.Interpolate( 1.0f ), rkTMul( Transform2.Rotation, EdgeB ), Axis ) )
						{
						mType = RK_SEPARATION_VERTICES;
						mWitness1 = Axis;
						mWitness2 = RK_VEC3_ZERO;
						}
					else
						{
						mType = RK_SEPARATION_EDGES;
						mWitness1 = rkTMul( Transform1.Rotation, EdgeA );
						mWitness2 = rkTMul( Transform2.Rotation, EdgeB );
						}
					}
				}
			}
			break;

		default:
			RK_ASSERT( !"Should never get here!" );
			break;
		}
	}


//--------------------------------------------------------------------------------------------------
RkSeparationType RkSeparationFunction::GetType() const
	{
	return mType;
	}


//--------------------------------------------------------------------------------------------------
float RkSeparationFunction::FindMinSeparation( int& Index1, int& Index2, float Beta ) const
	{
	RK_ASSERT( mType != RK_SEPARATION_UNKNOWN );
	RkTransform Transform1 = mSweep1.Interpolate( Beta );
	RkTransform Transform2 = mSweep2.Interpolate( Beta );

	switch ( mType )
		{
		case RK_SEPARATION_VERTICES:
			{
			RkVector3 Axis = mWitness1;
			
			RkVector3 Axis1 = rkTMul( Transform1.Rotation, Axis );
			Index1 = mProxy1->GetSupport( Axis1 );
			RkVector3 Point1 = Transform1 * mProxy1->GetVertex( Index1 );

			RkVector3 Axis2 = rkTMul( Transform2.Rotation, Axis );
			Index2 = mProxy2->GetSupport( -Axis2 );
			RkVector3 Point2 = Transform2 * mProxy2->GetVertex( Index2 );

			return rkDot( Point2 - Point1, Axis );
			}
			break;

		case RK_SEPARATION_EDGES:
			{
			RkVector3 Edge1 = Transform1.Rotation * mWitness1;
			RkVector3 Edge2 = Transform2.Rotation * mWitness2;
			RkVector3 Axis = rkCross( Edge1, Edge2 );
			Axis = rkNormalize( Axis );
			RK_ASSERT( Axis != RK_VEC3_ZERO );

			RkVector3 Axis1 = rkTMul( Transform1.Rotation, Axis );
			Index1 = mProxy1->GetSupport( Axis1 );
			RkVector3 Point1 = Transform1 * mProxy1->GetVertex( Index1 );

			RkVector3 Axis2 = rkTMul( Transform2.Rotation, Axis );
			Index2 = mProxy2->GetSupport( -Axis2 );
			RkVector3 Point2 = Transform2 * mProxy2->GetVertex( Index2 );

			return rkDot( Point2 - Point1, Axis );
			}
			break;

		case RK_SEPARATION_FACE1:
			{
			RkVector3 Axis = Transform1.Rotation * mWitness1;

			Index1 = -1;
			RkVector3 Point1 = Transform1 * mWitness2;

			RkVector3 Axis2 = rkTMul( Transform2.Rotation, Axis );
			Index2 = mProxy2->GetSupport( -Axis2 );
			RkVector3 Point2 = Transform2 * mProxy2->GetVertex( Index2 );

			return rkDot( Point2 - Point1, Axis );
			}
			break;
		
		case RK_SEPARATION_FACE2:
			{
			RkVector3 Axis = Transform2.Rotation * mWitness1;

			RkVector3 Axis1 = rkTMul( Transform1.Rotation, Axis );
			Index1 = mProxy1->GetSupport( -Axis1 );
			RkVector3 Point1 = Transform1 * mProxy1->GetVertex( Index1 );

			Index2 = -1;
			RkVector3 Point2 = Transform2 * mWitness2;

			return rkDot( Point1 - Point2, Axis );
			}
			break;

		default:
			RK_ASSERT( !"Should never get here!" );
			break;
		}

	return 0.0f;
	}


//--------------------------------------------------------------------------------------------------
float RkSeparationFunction::Evaluate( int Index1, int Index2, float Beta ) const
	{
	RK_ASSERT( mType != RK_SEPARATION_UNKNOWN );
	RkTransform Transform1 = mSweep1.Interpolate( Beta );
	RkTransform Transform2 = mSweep2.Interpolate( Beta );

	switch ( mType )
		{
		case RK_SEPARATION_VERTICES:
			{
			RkVector3 Axis = mWitness1;

			RkVector3 Point1 = Transform1 * mProxy1->GetVertex( Index1 );
			RkVector3 Point2 = Transform2 * mProxy2->GetVertex( Index2 );

			return rkDot( Point2 - Point1, Axis );
			}
			break;

		case RK_SEPARATION_EDGES:
			{
			RkVector3 Edge1 = Transform1.Rotation * mWitness1;
			RkVector3 Edge2 = Transform2.Rotation * mWitness2;
			RkVector3 Axis = rkCross( Edge1, Edge2 );
			Axis = rkNormalize( Axis );

			RkVector3 Point1 = Transform1 * mProxy1->GetVertex( Index1 );
			RkVector3 Point2 = Transform2 * mProxy2->GetVertex( Index2 );

			return rkDot( Point2 - Point1, Axis );
			}
			break;

		case RK_SEPARATION_FACE1:
			{
			RkVector3 Axis = Transform1.Rotation * mWitness1;

			RkVector3 Point1 = Transform1 * mWitness2;
			RkVector3 Point2 = Transform2 * mProxy2->GetVertex( Index2 );

			return rkDot( Point2 - Point1, Axis );
			}
			break;

		case RK_SEPARATION_FACE2:
			{
			RkVector3 Axis = Transform2.Rotation * mWitness1;

			RkVector3 Point1 = Transform1 * mProxy1->GetVertex( Index1 );
			RkVector3 Point2 = Transform2 * mWitness2;

			return rkDot( Point1 - Point2, Axis );
			}
			break;

		default:
			RK_ASSERT( !"Should never get here!" );
			break;
		}

	return 0.0f;
	}


//--------------------------------------------------------------------------------------------------
void RkSeparationFunction::ForceFixedAxis( float Beta )
	{
	RK_ASSERT( mType == RK_SEPARATION_EDGES );

	RkTransform Transform1 = mSweep1.Interpolate( Beta );
	RkTransform Transform2 = mSweep2.Interpolate( Beta );

	RkVector3 Edge1 = Transform1.Rotation * mWitness1;
	RkVector3 Edge2 = Transform2.Rotation * mWitness2;
	RkVector3 Axis = rkCross( Edge1, Edge2 );
	Axis = rkNormalize( Axis );

	mType = RK_SEPARATION_VERTICES;
	mWitness1 = Axis;
	mWitness2 = RK_VEC3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RkSeparationType RkSeparationFunction::ComputeType( const RkSweep& Sweep1, const RkTOIProxy* Proxy1, const RkSweep& Sweep2, const RkTOIProxy* Proxy2, const RkGJKQuery& Query, RkGJKCache Cache, float Beta ) const
	{
	RkTransform Transform1 = Sweep1.Interpolate( Beta );
	RkHull* Hull1 = Proxy1->GetHull();
	RkTransform Transform2 = Sweep2.Interpolate( Beta );
	RkHull* Hull2 = Proxy2->GetHull();

	// Note: Don't promote here! EE is not save to promote in this context (e.g. deepest point can 
	// be penetrating) and VV promotion will break indexing from the cache into the vertex buffer. 
	auto [Witness1, Witness2] = rkResolveCache( Cache, Hull1, Hull2 );

	// Face separation
	if ( Witness1.Feature == RK_GJK_FACE )
		{
		return RK_SEPARATION_FACE1;
		}

	if ( Witness2.Feature == RK_GJK_FACE )
		{
		return RK_SEPARATION_FACE2;
		}

	// Edge separation
	if ( Witness1.Feature == RK_GJK_EDGE && Witness2.Feature == RK_GJK_EDGE )
		{
		// Edge/Edge
		RkVector3 VertexA1 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 0 ] );
		RkVector3 VertexA2 = Transform1 * Proxy1->GetVertex( Cache.Vertices1[ 1 ] );
		RkVector3 EdgeA = rkNormalize( VertexA2 - VertexA1 );

		RkVector3 VertexB1 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 0 ] );
		RkVector3 VertexB2 = Transform2 * Proxy2->GetVertex( Cache.Vertices2[ 1 ] );
		RkVector3 EdgeB = rkNormalize( VertexB2 - VertexB1 );

		RkVector3 Axis = rkCross( EdgeA, EdgeB );
		float Length = rkLength( Axis );

		// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
		const float kTolerance = 0.005f;
		if ( Length < kTolerance )
			{
			// The axis is not safe to normalize so we use a world axis instead!
			return RK_SEPARATION_VERTICES;
			}

		Axis /= Length;
		if ( rkDot( VertexB1 - VertexA1, Axis ) < 0.0f )
			{
			Axis = -Axis;
			EdgeB = -EdgeB;
			}

		// Check for possible sign flip in edge/edge cross product
		if ( rkCheckFastEdges( Sweep1.Interpolate( 1.0f ), rkTMul( Transform1.Rotation, EdgeA ), Sweep2.Interpolate( 1.0f ), rkTMul( Transform2.Rotation, EdgeB ), Axis ) )
			{
			return RK_SEPARATION_VERTICES;
			}

		return RK_SEPARATION_EDGES;
		}

	// Vertex separation
	RK_ASSERT( Witness1.Feature == RK_GJK_VERTEX || Witness2.Feature == RK_GJK_VERTEX );
	return RK_SEPARATION_VERTICES;
	}


//--------------------------------------------------------------------------------------------------
// RkTOIProxy
//--------------------------------------------------------------------------------------------------
RkTOIProxy::RkTOIProxy( int VertexCount, const RkVector3* VertexBuffer, float Radius, RkHull* Hull )
	: RkGJKProxy( VertexCount, VertexBuffer )
	, mRadius( Radius )
	, mHull( Hull )
	{

	}


//--------------------------------------------------------------------------------------------------
float RkTOIProxy::GetRadius() const
	{
	RK_ASSERT( mRadius >= 0.0f );
	return mRadius;
	}


//--------------------------------------------------------------------------------------------------
RkHull* RkTOIProxy::GetHull() const
	{
	return mHull;
	}


//--------------------------------------------------------------------------------------------------
// TOI computation
//--------------------------------------------------------------------------------------------------
RkTOIQuery rkTOI( const RkSweep& Sweep1, const RkTOIProxy& Proxy1, const RkSweep& Sweep2, const RkTOIProxy& Proxy2, float MaxAlpha, int MaxIterations )
	{
	// Setup target distance and tolerance
	float TotalRadius = Proxy1.GetRadius() + Proxy2.GetRadius();
	float Target = rkMax( RK_LINEAR_SLOP, TotalRadius - 3.0f * RK_LINEAR_SLOP );
	float Tolerance = 0.125f * RK_LINEAR_SLOP;
	RK_ASSERT( Target > Tolerance );


	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	float Alpha1 = 0.0f;
	RkGJKCache Cache;
	for ( int Iteration = 0; Iteration < MaxIterations; ++Iteration )
		{
		RkTransform Transform1 = Sweep1.Interpolate( Alpha1 );
		RkTransform Transform2 = Sweep2.Interpolate( Alpha1 );
		RkGJKQuery Query = rkGJK( Transform1, Proxy1, Transform2, Proxy2, Cache );

		// Are the shapes overlapping?
		if ( Query.Distance <= 0.0f )
			{
			// Failed!
			return { RK_TOI_OVERLAPPED, 0.0f };
			}

		if ( Query.Distance <= Target + Tolerance )
			{
			// Success!
			return { RK_TOI_TOUCHING, Alpha1, Query.Point1, Query.Point2 };
			}

		// Initialize the separating axis and keep track of local closest points
		RkSeparationFunction Function( Sweep1, &Proxy1, Sweep2, &Proxy2, Query, Cache, Alpha1 );

#if defined( DEBUG ) || defined( _DEBUG )
		// Validate separation function
// 			{
// 			int Index1, Index2;
// 			float MinSeparation = Function.FindMinSeparation( Index1, Index2, Alpha1 );
// 			RK_ASSERT( MinSeparation > Target - Tolerance && rkAbs( MinSeparation - Query.Distance ) < RK_LINEAR_SLOP );
// 			}
#endif

		// Compute the TOI on the separating axis. We do this by successively resolving the deepest point. 
		float Alpha2 = MaxAlpha;
		for ( int InnerIteration = 0; InnerIteration < MaxIterations; ++InnerIteration )
			{
			int Index1, Index2;
			float Separation2 = Function.FindMinSeparation( Index1, Index2, Alpha2 );

			// Dump the function seen by the root finder
// 			for ( int Step = 0; Step <= 100; ++Step )
// 				{
// 				float Alpha = 0.01f * Step;
// 				float Separation = Function.Evaluate( Index1, Index2, Alpha );
// 
// 				rnReport( "s(%4.2g) = %g\n", Alpha, Separation );
// 				}

			// Is the final configuration separated?
			if ( Separation2 - Target > Tolerance )
				{
				// Success!
				return { RK_TOI_SEPARATED, MaxAlpha };
				}

			// Has the separation reached tolerance?
			if ( Separation2 >= Target - Tolerance )
				{
				// Advance the sweeps
				Alpha1 = Alpha2;
				break;
				}

			// Compute the initial separation of the witness points
			float Separation1 = Function.Evaluate( Index1, Index2, Alpha1 );

			// Check for overlap. This might happen if  
			// the root finder runs out of iterations.
			if ( Separation1 < Target - Tolerance )
				{
				// Failed!
				return { RK_TOI_FAILED, Alpha1 };
				}

			// Has the separation reached tolerance?
			if ( Separation1 <= Target + Tolerance )
				{
				// Success! T1 should hold the TOI (could be 0.0)
				Transform1 = Sweep1.Interpolate( Alpha1 );
				Transform2 = Sweep2.Interpolate( Alpha1 );
				Query = rkGJK( Transform1, Proxy1, Transform2, Proxy2, Cache );

				return { RK_TOI_TOUCHING, Alpha1, Query.Point1, Query.Point2 };
				}

			// Compute 1D bracketed root of: s(t) - target = 0
			// s(t) = Dot( SupportB( -Axis ) - SupportA( Axis ), Axis ) 
			float Beta1 = Alpha1;
			float Beta2 = Alpha2;

			const int kMaxRootIterations = 64;
			for ( int RootIteration = 0; RootIteration < kMaxRootIterations; ++RootIteration )
				{
				// Use a mix of Regula-Falsi and bisection
				float Beta = ( RootIteration & 1 ) ? Beta1 + ( Target - Separation1 ) * ( Beta2 - Beta1 ) / ( Separation2 - Separation1 ) : 0.5f * ( Beta1 + Beta2 );
				float Separation = Function.Evaluate( Index1, Index2, Beta );

				// Has the separation reached tolerance?
				if ( rkAbs( Separation - Target ) <= 0.1f * Tolerance )
					{
					// Alpha2 holds a tentative value for Alpha1
					Alpha2 = Beta;
					break;
					}

				// Ensure we continue to bracket the root.
				if ( Separation > Target )
					{
					Beta1 = Beta;
					Separation1 = Separation;
					}
				else
					{
					Beta2 = Beta;
					Separation2 = Separation;
					}
				}

			// Restart the inner loop if we have a failing edge case.
			if ( InnerIteration == MaxIterations - 1 && Function.GetType() == RK_SEPARATION_EDGES )
				{
				InnerIteration = 0;

				Alpha2 = MaxAlpha;
				Function.ForceFixedAxis( Alpha1 );
				RK_ASSERT( Function.GetType() != RK_SEPARATION_EDGES );
				}
			}
		}

	// Failed!
	return { RK_TOI_OUT_OF_ITERATIONS, Alpha1 };
	}


//--------------------------------------------------------------------------------------------------
RkTOIQuery rkTOI( const RkTransform& Start1, const RkVector3& End1, const RkTOIProxy& Proxy1, const RkTransform& Start2, const RkVector3& End2, const RkTOIProxy& Proxy2, float MaxAlpha, int MaxIterations )
	{
	// Compute tolerance
	float TotalRadius = Proxy1.GetRadius() + Proxy2.GetRadius();
	float Target = rkMax( RK_LINEAR_SLOP, TotalRadius - 3.0f * RK_LINEAR_SLOP );
	float Tolerance = 0.125f * RK_LINEAR_SLOP;
	RK_ASSERT( Target > Tolerance );

	// Prepare input for distance query
	RkGJKCache Cache;

	float Alpha = 0.0f;
	RkTransform XForm1 = Start1;
	RkTransform XForm2 = Start2;
	
	RkVector3 Delta1 = End1 - Start1.Translation;
	RkVector3 Delta2 = End2 - Start2.Translation;

	for ( int Iteration = 0; Iteration < MaxIterations; ++Iteration )
		{
		// If the shapes are overlapped, we give up on continuous collision.
		RkGJKQuery Query = rkGJK( XForm1, Proxy1, XForm2, Proxy2, Cache );
		if ( Query.Distance < Target - Tolerance )
			{
			// We can start in initial overlap, but this should
			// *not* happen during subsequent iterations!
			RK_ASSERT( Iteration == 0 );

			return { RK_TOI_OVERLAPPED, 0.0f };
			}

		// Check if we reached the target distance
		if ( Query.Distance <= Target + Tolerance )
			{
			// Success!
			return { RK_TOI_TOUCHING, Alpha, Query.Point1, Query.Point2 };
			}

		// Separation function:
		// f(t) = (c2 + t * dp2 - c1 - t * dp1 ) * n 

		// Root finding : f(t) - target = 0
		// (c2 + t * dp2 - c1 - t * dp1 ) * n - target = 0
		// (c2 - c1) * n + t * (dp2 - dp1) * n - target = 0
		// t = [target - (c2 - c1) * n] / [(dp2 - dp1) * n]
		// t = (target - d) / [(dp2 - dp1) * n]
		float Distance = Query.Distance;
		RkVector3 Closest1 = Query.Point1;
		RkVector3 Closest2 = Query.Point2;
		RkVector3 Normal = rkNormalize( Closest2 - Closest1 );

		// Check if shapes are still approaching each other
		float Denominator = rkDot( Delta2 - Delta1, Normal );
		if ( Denominator >= 0.0f )
			{
			// Success!
			return { RK_TOI_SEPARATED, 1.0f };
			}

		// Advance sweep
		Alpha += ( Target - Distance ) / Denominator;
		if ( Alpha >= MaxAlpha )
			{
			// Success!
			return { RK_TOI_SEPARATED, MaxAlpha };
			}

		XForm1.Translation = Start1.Translation + Alpha * Delta1;
		XForm2.Translation = Start2.Translation + Alpha * Delta2;
		}

	// Failure!
	return { RK_TOI_OUT_OF_ITERATIONS, Alpha };
	}