//--------------------------------------------------------------------------------------------------
// gjksimplex.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "gjksimplex.h"
#include "gjk.h"


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static inline void rkBarycentricCoordinates( float Out[ 3 ], const RkVector3& A, const RkVector3& B )
	{
	RkVector3 AB = B - A;

	// Last element is divisor
	float Divisor = rkDot( AB, AB );

	Out[ 0 ] = rkDot( B, AB );
	Out[ 1 ] = -rkDot( A, AB );
	Out[ 2 ] = Divisor;
	}


//--------------------------------------------------------------------------------------------------
static inline void rkBarycentricCoordinates( float Out[ 4 ], const RkVector3& A, const RkVector3& B, const RkVector3& C )
	{
	RkVector3 AB = B - A;
	RkVector3 AC = C - A;

	RkVector3 B_x_C = rkCross( B, C );
	RkVector3 C_x_A = rkCross( C, A );
	RkVector3 A_x_B = rkCross( A, B );

	RkVector3 AB_x_AC = rkCross( AB, AC );

	// Last element is divisor
	float Divisor = rkDot( AB_x_AC, AB_x_AC );

	Out[ 0 ] = rkDot( B_x_C, AB_x_AC );
	Out[ 1 ] = rkDot( C_x_A, AB_x_AC );
	Out[ 2 ] = rkDot( A_x_B, AB_x_AC );
	Out[ 3 ] = Divisor;
	}


//--------------------------------------------------------------------------------------------------
static inline void rkBarycentricCoordinates( float Out[ 5 ], const RkVector3& A, const RkVector3& B, const RkVector3& C, const RkVector3& D )
	{
	RkVector3 AB = B - A;
	RkVector3 AC = C - A;
	RkVector3 AD = D - A;

	// Last element is divisor (forced to be positive)
	float Divisor = rkDet( AB, AC, AD );

	float Sign = rkSign( Divisor );
	Out[ 0 ] = Sign * rkDet( B, C, D );
	Out[ 1 ] = Sign * rkDet( A, D, C );
	Out[ 2 ] = Sign * rkDet( A, B, D );
	Out[ 3 ] = Sign * rkDet( A, C, B );
	Out[ 4 ] = Sign * Divisor;
	}


//--------------------------------------------------------------------------------------------------
// RkGJKCache
//--------------------------------------------------------------------------------------------------
void rkClearCache( RkGJKCache& Cache )
	{
	rkMemZero( &Cache, sizeof( Cache ) );
	}


//--------------------------------------------------------------------------------------------------
// Simplex 
//--------------------------------------------------------------------------------------------------
RkGJKSimplex::RkGJKSimplex()
	: mVertexCount( 0 )
	{
	
	}


//--------------------------------------------------------------------------------------------------
void RkGJKSimplex::ReadCache( const RkTransform& Transform1, const RkGJKProxy& Proxy1, const RkTransform& Transform2, const RkGJKProxy& Proxy2, const RkGJKCache& Cache )
	{
	RK_ASSERT( 0 <= Cache.VertexCount && Cache.VertexCount <= RK_MAX_SIMPLEX_VERTICES );

	// Copy data from cache
	mVertexCount = Cache.VertexCount;
	for ( int I = 0; I < Cache.VertexCount; ++I )
		{
		int Index1 = Cache.Vertices1[ I ];
		int Index2 = Cache.Vertices2[ I ];

		RkVector3 Vertex1 = Transform1 * Proxy1.GetVertex( Index1 );
		RkVector3 Vertex2 = Transform2 * Proxy2.GetVertex( Index2 );

		mVertices[ I ].Index1 = Index1;
		mVertices[ I ].Index2 = Index2;
		mVertices[ I ].Position1 = Vertex1;
		mVertices[ I ].Position2 = Vertex2;
		mVertices[ I ].Position = Vertex2 - Vertex1;
		mLambdas[ I ] = 0.0f;
		}

	// Compute the new simplex metric, if it is substantially 
	// different than the old metric flush the simplex.
	if ( mVertexCount > 0 )
		{
		float Metric1 = Cache.Metric;
		float Metric2 = GetMetric();

		if  ( 2.0f * Metric1 < Metric2 || Metric2 < 0.5f * Metric1 || Metric2 < RK_F32_EPSILON )
			{
			// Flush the simplex
			mVertexCount = 0;
			}
		}

	// If the cache is invalid or empty
	if ( mVertexCount == 0 )
		{
		RkVector3 Vertex1 = Transform1 * Proxy1.GetVertex( 0 );
		RkVector3 Vertex2 = Transform2 * Proxy2.GetVertex( 0 );

		mVertexCount = 1;
		mVertices[ 0 ].Index1 = 0;
		mVertices[ 0 ].Index2 = 0;
		mVertices[ 0 ].Position1 = Vertex1;
		mVertices[ 0 ].Position2 = Vertex2;
		mVertices[ 0 ].Position = Vertex2 - Vertex1;
		mLambdas[ 0 ] = 0.0f;
		}
	}


//--------------------------------------------------------------------------------------------------
void RkGJKSimplex::WriteCache( RkGJKCache& Cache ) const
	{
	Cache.Metric = GetMetric();
	Cache.VertexCount = mVertexCount;
	for ( int Index = 0; Index < mVertexCount; ++Index )
		{
		Cache.Vertices1[ Index ] = uint8( mVertices[ Index ].Index1 );
		Cache.Vertices2[ Index ] = uint8( mVertices[ Index ].Index2 );
		Cache.Lambdas[ Index ] = mLambdas[ Index ];
		}
	}


//--------------------------------------------------------------------------------------------------
int RkGJKSimplex::GetVertexCount() const
	{
	return mVertexCount;
	}


//--------------------------------------------------------------------------------------------------
RkGJKVertex& RkGJKSimplex::GetVertex( int Index )
	{
	RK_ASSERT( 0 <= Index && Index < RK_MAX_SIMPLEX_VERTICES );
	return mVertices[ Index ];
	}


//--------------------------------------------------------------------------------------------------
const RkGJKVertex& RkGJKSimplex::GetVertex( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < RK_MAX_SIMPLEX_VERTICES );
	return mVertices[ Index ];
	}


//--------------------------------------------------------------------------------------------------
bool RkGJKSimplex::IsEmpty() const
	{
	return mVertexCount == 0;
	}


//--------------------------------------------------------------------------------------------------
bool RkGJKSimplex::Solve()
	{
	// Save the current simplex
	RK_ASSERT( 1 <= mVertexCount && mVertexCount <= 4 );
	
	mCache.VertexCount = mVertexCount;
	for ( int I = 0; I < mVertexCount; ++I )
		{
		mCache.Vertices1[ I ] = uint8( mVertices[ I ].Index1 );
		mCache.Vertices2[ I ] = uint8( mVertices[ I ].Index2 );
		}

	// Solve simplex
	switch ( mVertexCount )
		{
		case 1: 
			return Solve1();
			break;

		case 2:
			return Solve2();
			break;

		case 3:
			return Solve3();
			break;

		case 4:
			return Solve4();
			break;

		default:
			RK_ASSERT( !"Should never get here!" );
			break;
		}

	return false;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkGJKSimplex::GetClosestPoint() const
	{
	RK_ASSERT( 1 <= mVertexCount && mVertexCount <= 4 );

	switch ( mVertexCount )
		{
		case 1: 
			return mVertices[ 0 ].Position;
			break;

		case 2:
			return mLambdas[ 0 ] * mVertices[ 0 ].Position + mLambdas[ 1 ] * mVertices[ 1 ].Position;
			break;

		case 3:
			return mLambdas[ 0 ] * mVertices[ 0 ].Position + mLambdas[ 1 ] * mVertices[ 1 ].Position + mLambdas[ 2 ] * mVertices[ 2 ].Position;
			break;

		case 4:
			return mLambdas[ 0 ] * mVertices[ 0 ].Position + mLambdas[ 1 ] * mVertices[ 1 ].Position + mLambdas[ 2 ] * mVertices[ 2 ].Position + mLambdas[ 3 ] * mVertices[ 3 ].Position;
			break;

		default:
			RK_ASSERT( !"Should never get here!" );
			break;
		}

	return RK_VEC3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RkVector3 RkGJKSimplex::GetSearchDirection() const
	{
	RK_ASSERT( 1 <= mVertexCount && mVertexCount <= 3 );

	switch ( mVertexCount )
		{
		case 1: 
			{
			// v = -A
			const RkVector3& A = mVertices[ 0 ].Position;

			return -A;
			}
			break;

		case 2:
			{
			// v = (AB x AO) x AB
			const RkVector3& A = mVertices[ 0 ].Position;
			const RkVector3& B = mVertices[ 1 ].Position;

			RkVector3 AB = B - A;

			return rkCross( rkCross( AB, -A ), AB );
			}
			break;

		case 3:
			{
			// v = AB x AC or v = AC x AB
			const RkVector3& A = mVertices[ 0 ].Position;
			const RkVector3& B = mVertices[ 1 ].Position;
			const RkVector3& C = mVertices[ 2 ].Position;

			RkVector3 AB = B - A;
			RkVector3 AC = C - A;
			
			RkVector3 N = rkCross( AB, AC );

			return rkDot( N, A ) < 0.0f ? N : -N;
			}
			break;

		default:
			RK_ASSERT( !"Should never get here!" );
			break;
		}

	return RK_VEC3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
bool RkGJKSimplex::AddVertex( int Index1, const RkVector3& Vertex1, int Index2, const RkVector3& Vertex2 )
	{
	if ( CheckCache( Index1, Index2 ) )
		{
		return false;
		}

	mVertices[ mVertexCount ].Index1 = Index1;
	mVertices[ mVertexCount ].Index2 = Index2;
	mVertices[ mVertexCount ].Position1 = Vertex1;
	mVertices[ mVertexCount ].Position2 = Vertex2;
	mVertices[ mVertexCount ].Position = Vertex2 - Vertex1;
	mVertexCount++;

	return true;
	}


//--------------------------------------------------------------------------------------------------
void RkGJKSimplex::BuildWitnessPoints( RkVector3& Vertex1, RkVector3& Vertex2 ) const
	{
	RK_ASSERT( 1 <= mVertexCount && mVertexCount <= 4 );

	switch ( mVertexCount )
		{
		case 1: 
			Vertex1 = mVertices[ 0 ].Position1;
			Vertex2 = mVertices[ 0 ].Position2;
			break;

		case 2:
			Vertex1 = mLambdas[ 0 ] * mVertices[ 0 ].Position1 + mLambdas[ 1 ] * mVertices[ 1 ].Position1;
			Vertex2 = mLambdas[ 0 ] * mVertices[ 0 ].Position2 + mLambdas[ 1 ] * mVertices[ 1 ].Position2;
			break;

		case 3:
			Vertex1 = mLambdas[ 0 ] * mVertices[ 0 ].Position1 + mLambdas[ 1 ] * mVertices[ 1 ].Position1 + mLambdas[ 2 ] * mVertices[ 2 ].Position1;
			Vertex2 = mLambdas[ 0 ] * mVertices[ 0 ].Position2 + mLambdas[ 1 ] * mVertices[ 1 ].Position2 + mLambdas[ 2 ] * mVertices[ 2 ].Position2;
			break;

		case 4:
			// Force indentical points and *zero* distance
			Vertex1 = Vertex2 = mLambdas[ 0 ] * mVertices[ 0 ].Position1 + mLambdas[ 1 ] * mVertices[ 1 ].Position1 + mLambdas[ 2 ] * mVertices[ 2 ].Position1 + mLambdas[ 3 ] * mVertices[ 3 ].Position1;
			break;

		default:
			RK_ASSERT( !"Should never get here!" );
			break;
		}
	}


//--------------------------------------------------------------------------------------------------
float RkGJKSimplex::GetMetric() const
	{
	RK_ASSERT( 1 <= mVertexCount && mVertexCount <= 4 );

	switch ( mVertexCount )
		{
		case 1: 
			{
			return 0.0f;
			}
			break;

		case 2:
			{
			const RkVector3& A = mVertices[ 0 ].Position;
			const RkVector3& B = mVertices[ 1 ].Position;

			return rkDistance( A, B );
			}
			break;

		case 3:
			{
			const RkVector3& A = mVertices[ 0 ].Position;
			const RkVector3& B = mVertices[ 1 ].Position;
			const RkVector3& C = mVertices[ 2 ].Position;

			return rkLength( rkCross( B - A, C - A ) ) / 2.0f;
			}
			break;

		case 4:
			{
			const RkVector3& A = mVertices[ 0 ].Position;
			const RkVector3& B = mVertices[ 1 ].Position;
			const RkVector3& C = mVertices[ 2 ].Position;
			const RkVector3& D = mVertices[ 3 ].Position;

			return rkDet( B - A, C - A, D - A ) / 6.0f;
			}
			break;

		default:
			RK_ASSERT( !"Should never get here!" );
			break;
		}

	return 0.0f;
	}


//--------------------------------------------------------------------------------------------------
bool RkGJKSimplex::CheckCache( int Index1, int Index2 ) const
	{
	// Check for duplicate support points. This is the main termination criteria.
	for ( int I = 0; I < mCache.VertexCount; ++I )
		{
		if ( mCache.Vertices1[ I ] == Index1 && mCache.Vertices2[ I ] == Index2 )
			{
			return true;
			}
		}

	return false;
	}


//--------------------------------------------------------------------------------------------------
bool RkGJKSimplex::Solve1()
	{
	// VR( A )
	RK_ASSERT( mVertexCount == 1 );
	mLambdas[ 0 ] = 1.0f;

	return true;
	}


//--------------------------------------------------------------------------------------------------
bool RkGJKSimplex::Solve2()
	{
	// Get simplex (be aware of aliasing here!)
	RK_ASSERT( mVertexCount == 2 );
	RkGJKVertex VertexA = mVertices[ 0 ];
	RkGJKVertex VertexB = mVertices[ 1 ];

	// Vertex regions
	float wAB[ 3 ];
	rkBarycentricCoordinates( wAB, VertexA.Position, VertexB.Position );
	
	// V( A )
	if ( wAB[ 1 ] <= 0.0f )
		{
		// Reduce simplex
		mVertexCount = 1;
		mVertices[ 0 ] = VertexA;

		mLambdas[ 0 ] = 1.0f;

		return true;
		}

	// V( B )
	if ( wAB[ 0 ] <= 0.0f )
		{
		// Reduce simplex
		mVertexCount = 1;
		mVertices[ 0 ] = VertexB;

		mLambdas[ 0 ] = 1.0f;

		return true;
		}

	// Edge region
	float Divisor = wAB[ 2 ];
	if ( Divisor <= 0.0f )
		{
		return false;
		}


	// VR( AB )
	mLambdas[ 0 ] = wAB[ 0 ] / Divisor;
	mLambdas[ 1 ] = wAB[ 1 ] / Divisor;

	return true;
	}


//--------------------------------------------------------------------------------------------------
bool RkGJKSimplex::Solve3()
	{
	// Get simplex (be aware of aliasing here!)
	RK_ASSERT( mVertexCount == 3 );
	RkGJKVertex VertexA = mVertices[ 0 ];
	RkGJKVertex VertexB = mVertices[ 1 ];
	RkGJKVertex VertexC = mVertices[ 2 ];

	// Vertex regions
	float wAB[ 3 ], wBC[ 3 ], wCA[ 3 ];
	rkBarycentricCoordinates( wAB, VertexA.Position, VertexB.Position );
	rkBarycentricCoordinates( wBC, VertexB.Position, VertexC.Position );
	rkBarycentricCoordinates( wCA, VertexC.Position, VertexA.Position );

	// VR( A )
	if ( wAB[ 1 ] <= 0.0f && wCA[ 0 ] <= 0.0f )
		{
		// Reduce simplex
		mVertexCount = 1;
		mVertices[ 0 ] = VertexA;

		mLambdas[ 0 ] = 1.0f;

		return true;
		}

	// VR( B )
	if ( wBC[ 1 ] <= 0.0f && wAB[ 0 ] <= 0.0f )
		{
		// Reduce simplex
		mVertexCount = 1;
		mVertices[ 0 ] = VertexB;

		mLambdas[ 0 ] = 1.0f;

		return true;
		}

	// VR( C )
	if ( wCA[ 1 ] <= 0.0f && wBC[ 0 ] <= 0.0f )
		{
		// Reduce simplex
		mVertexCount = 1;
		mVertices[ 0 ] = VertexC;

		mLambdas[ 0 ] = 1.0f;

		return true;
		}

	// Edge regions
	float wABC[ 4 ];
	rkBarycentricCoordinates( wABC, VertexA.Position, VertexB.Position, VertexC.Position );

	// VR( AB )
	if ( wABC[ 2 ] <= 0.0f && wAB[ 0 ] > 0.0f && wAB[ 1 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 2;
		mVertices[ 0 ] = VertexA;
		mVertices[ 1 ] = VertexB;

		// Normalize
		float Divisor = wAB[ 2 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wAB[ 0 ] / Divisor;
		mLambdas[ 1 ] = wAB[ 1 ] / Divisor;

		return true;
		}

	// VR( BC )
	if ( wABC[ 0 ] <= 0.0f && wBC[ 0 ] > 0.0f && wBC[ 1 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 2;
		mVertices[ 0 ] = VertexB;
		mVertices[ 1 ] = VertexC;

		// Normalize
		float Divisor = wBC[ 2 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wBC[ 0 ] / Divisor;
		mLambdas[ 1 ] = wBC[ 1 ] / Divisor;

		return true;
		}

	// VR( CA )
	if ( wABC[ 1 ] <= 0.0f && wCA[ 0 ] > 0.0f && wCA[ 1 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 2;
		mVertices[ 0 ] = VertexC;
		mVertices[ 1 ] = VertexA;

		// Normalize
		float Divisor = wCA[ 2 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wCA[ 0 ] / Divisor;
		mLambdas[ 1 ] = wCA[ 1 ] / Divisor;

		return true;
		}

	// Face region
	float Divisor = wABC[ 3 ];
	if ( Divisor <= 0.0f )
		{
		return false;
		}

	// VR( ABC )
	mLambdas[ 0 ] = wABC[ 0 ] / Divisor;
	mLambdas[ 1 ] = wABC[ 1 ] / Divisor;
	mLambdas[ 2 ] = wABC[ 2 ] / Divisor;

	return true;
	}


//--------------------------------------------------------------------------------------------------
bool RkGJKSimplex::Solve4()
	{
	// Get simplex (be aware of aliasing here!)
	RK_ASSERT( mVertexCount == 4 );
	RkGJKVertex VertexA = mVertices[ 0 ];
	RkGJKVertex VertexB = mVertices[ 1 ];
	RkGJKVertex VertexC = mVertices[ 2 ];
	RkGJKVertex VertexD = mVertices[ 3 ];

	// Vertex region
	float wAB[ 3 ], wAC[ 3 ], wAD[ 3 ], wBC[ 3 ], wCD[ 3 ], wDB[ 3 ];
	rkBarycentricCoordinates( wAB, VertexA.Position, VertexB.Position );
	rkBarycentricCoordinates( wAC, VertexA.Position, VertexC.Position );
	rkBarycentricCoordinates( wAD, VertexA.Position, VertexD.Position );
	rkBarycentricCoordinates( wBC, VertexB.Position, VertexC.Position );
	rkBarycentricCoordinates( wCD, VertexC.Position, VertexD.Position );
	rkBarycentricCoordinates( wDB, VertexD.Position, VertexB.Position );

	// VR( A )
	if ( wAB[ 1 ] <= 0.0f && wAC[ 1 ] <= 0.0f && wAD[ 1 ] <= 0.0f )
		{
		// Reduce simplex
		mVertexCount = 1;
		mVertices[ 0 ] = VertexA;

		mLambdas[ 0 ] = 1.0f;

		return true;
		}

	// VR( B )
	if ( wAB[ 0 ] <= 0.0f && wDB[ 0 ] <= 0.0f && wBC[ 1 ] <= 0.0f )
		{
		// Reduce simplex
		mVertexCount = 1;
		mVertices[ 0 ] = VertexB;

		mLambdas[ 0 ] = 1.0f;

		return true;
		}

	// VR( C )
	if ( wAC[ 0 ] <= 0.0f && wBC[ 0 ] <= 0.0f && wCD[ 1 ] <= 0.0f )
		{
		// Reduce simplex
		mVertexCount = 1;
		mVertices[ 0 ] = VertexC;

		mLambdas[ 0 ] = 1.0f;

		return true;
		}

	// VR( D )
	if ( wAD[ 0 ] <= 0.0f && wCD[ 0 ] <= 0.0f && wDB[ 1 ] <= 0.0f )
		{
		// Reduce simplex
		mVertexCount = 1;
		mVertices[ 0 ] = VertexD;

		mLambdas[ 0 ] = 1.0f;

		return true;
		}

	// Edge region
	float wACB[ 4 ], wABD[ 4 ], wADC[ 4 ], wBCD[ 4 ];
	rkBarycentricCoordinates( wACB, VertexA.Position, VertexC.Position, VertexB.Position );
	rkBarycentricCoordinates( wABD, VertexA.Position, VertexB.Position, VertexD.Position );
	rkBarycentricCoordinates( wADC, VertexA.Position, VertexD.Position, VertexC.Position );
	rkBarycentricCoordinates( wBCD, VertexB.Position, VertexC.Position, VertexD.Position );

	// VR( AB )
	if ( wABD[ 2 ] <= 0.0f && wACB[ 1 ] <= 0.0f && wAB[ 0 ] > 0.0f && wAB[ 1 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 2;
		mVertices[ 0 ] = VertexA;
		mVertices[ 1 ] = VertexB;

		// Normalize
		float Divisor = wAB[ 2 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wAB[ 0 ] / Divisor;
		mLambdas[ 1 ] = wAB[ 1 ] / Divisor;

		return true;
		}

	// VR( AC )
	if ( wACB[ 2 ] <= 0.0f && wADC[ 1 ] <= 0.0f && wAC[ 0 ] > 0.0f && wAC[ 1 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 2;
		mVertices[ 0 ] = VertexA;
		mVertices[ 1 ] = VertexC;

		// Normalize
		float Divisor = wAC[ 2 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wAC[ 0 ] / Divisor;
		mLambdas[ 1 ] = wAC[ 1 ] / Divisor;

		return true;
		}

	// VR( AD )
	if ( wADC[ 2 ] <= 0.0f && wABD[ 1 ] <= 0.0f && wAD[ 0 ] > 0.0f && wAD[ 1 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 2;
		mVertices[ 0 ] = VertexA;
		mVertices[ 1 ] = VertexD;

		// Normalize
		float Divisor = wAD[ 2 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wAD[ 0 ] / Divisor;
		mLambdas[ 1 ] = wAD[ 1 ] / Divisor;

		return true;
		}

	// VR( BC )
	if ( wACB[ 0 ] <= 0.0f && wBCD[ 2 ] <= 0.0f && wBC[ 0 ] > 0.0f && wBC[ 1 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 2;
		mVertices[ 0 ] = VertexB;
		mVertices[ 1 ] = VertexC;

		// Normalize
		float Divisor = wBC[ 2 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wBC[ 0 ] / Divisor;
		mLambdas[ 1 ] = wBC[ 1 ] / Divisor;

		return true;
		}

	// VR( CD )
	if ( wADC[ 0 ] <= 0.0f && wBCD[ 0 ] <= 0.0f && wCD[ 0 ] > 0.0f && wCD[ 1 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 2;
		mVertices[ 0 ] = VertexC;
		mVertices[ 1 ] = VertexD;

		// Normalize
		float Divisor = wCD[ 2 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wCD[ 0 ] / Divisor;
		mLambdas[ 1 ] = wCD[ 1 ] / Divisor;

		return true;
		}

	// VR( DB )
	if ( wABD[ 0 ] <= 0.0f && wBCD[ 1 ] <= 0.0f &&  wDB[ 0 ] > 0.0f && wDB[ 1 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 2;
		mVertices[ 0 ] = VertexD;
		mVertices[ 1 ] = VertexB;

		// Normalize
		float Divisor = wDB[ 2 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wDB[ 0 ] / Divisor;
		mLambdas[ 1 ] = wDB[ 1 ] / Divisor;

		return true;
		}

	// Face regions
	float wABCD[ 5 ];
	rkBarycentricCoordinates( wABCD, VertexA.Position, VertexB.Position, VertexC.Position, VertexD.Position );

	// VR( ACB )
	if ( wABCD[ 3 ] < 0.0f && wACB[ 0 ] > 0.0f && wACB[ 1 ] > 0.0f && wACB[ 2 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 3;
		mVertices[ 0 ] = VertexA;
		mVertices[ 1 ] = VertexC;
		mVertices[ 2 ] = VertexB;

		// Normalize
		float Divisor = wACB[ 3 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wACB[ 0 ] / Divisor;
		mLambdas[ 1 ] = wACB[ 1 ] / Divisor;
		mLambdas[ 2 ] = wACB[ 2 ] / Divisor;

		return true;
		}

	// VR( ABD )
	if ( wABCD[ 2 ] < 0.0f && wABD[ 0 ] > 0.0f && wABD[ 1 ] > 0.0f && wABD[ 2 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 3;
		mVertices[ 0 ] = VertexA;
		mVertices[ 1 ] = VertexB;
		mVertices[ 2 ] = VertexD;

		// Normalize
		float Divisor = wABD[ 3 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wABD[ 0 ] / Divisor;
		mLambdas[ 1 ] = wABD[ 1 ] / Divisor;
		mLambdas[ 2 ] = wABD[ 2 ] / Divisor;

		return true;
		}

	// VR( ADC )
	if ( wABCD[ 1 ] < 0.0f && wADC[ 0 ] > 0.0f && wADC[ 1 ] > 0.0f && wADC[ 2 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 3;
		mVertices[ 0 ] = VertexA;
		mVertices[ 1 ] = VertexD;
		mVertices[ 2 ] = VertexC;

		// Normalize
		float Divisor = wADC[ 3 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wADC[ 0 ] / Divisor;
		mLambdas[ 1 ] = wADC[ 1 ] / Divisor;
		mLambdas[ 2 ] = wADC[ 2 ] / Divisor;

		return true;
		}

	// VR( BCD )
	if ( wABCD[ 0 ] < 0.0f && wBCD[ 0 ] > 0.0f && wBCD[ 1 ] > 0.0f && wBCD[ 2 ] > 0.0f )
		{
		// Reduce simplex
		mVertexCount = 3;
		mVertices[ 0 ] = VertexB;
		mVertices[ 1 ] = VertexC;
		mVertices[ 2 ] = VertexD;

		// Normalize
		float Divisor = wBCD[ 3 ];
		if ( Divisor <= 0.0f )
			{
			return false;
			}

		mLambdas[ 0 ] = wBCD[ 0 ] / Divisor;
		mLambdas[ 1 ] = wBCD[ 1 ] / Divisor;
		mLambdas[ 2 ] = wBCD[ 2 ] / Divisor;

		return true;
		}

	// *** Inside tetrahedron ***
	float Divisor = wABCD[ 4 ];
	if ( Divisor <= 0.0f )
		{
		return false;
		}

	// VR( ABCD )
	mLambdas[ 0 ] = wABCD[ 0 ] / Divisor;
	mLambdas[ 1 ] = wABCD[ 1 ] / Divisor;
	mLambdas[ 2 ] = wABCD[ 2 ] / Divisor;
	mLambdas[ 3 ] = wABCD[ 3 ] / Divisor;

	return true;
	}

