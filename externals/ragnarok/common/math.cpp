//--------------------------------------------------------------------------------------------------
// math.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "math.h"
#include "memory.h"


//--------------------------------------------------------------------------------------------------
// Geometry utilities
//--------------------------------------------------------------------------------------------------
bool RkClosestApproachResult::IsNormalized() const
{
	return ( 0.0f <= Lambda1 && Lambda1 <= 1.0f ) && ( 0.0f <= Lambda2 && Lambda2 <= 1.0f );
}


//--------------------------------------------------------------------------------------------------
void rkClosestApproachLines( RkClosestApproachResult& Out, const RkVector3& P1, const RkVector3& D1, const RkVector3& P2, const RkVector3& D2 )
	{
	// Solve A*x = b
	float A11 = rkDot( D1, D1 ); float A12 = -rkDot( D1, D2 );
	float A21 = rkDot( D2, D1 ); float A22 = -rkDot( D2, D2 );

	RkVector3 W = P1 - P2;
	float B1 = -rkDot( D1, W );
	float B2 = -rkDot( D2, W );

	float Det = A11 * A22 - A12 * A21;
	if ( Det * Det < 1000.0f * RK_F32_MIN )
		{
		// Lines are parallel - project p2 onto line L1: x1 = p1 + s1 * d1
		float S1 = rkDot( P2 - P1, D1 ) / rkDot( D1, D1 );
		float S2 = 0.0f;

		Out.Point1 = P1 + S1 * D1;
		Out.Lambda1 = S1;
		Out.Point2 = P2 + S2 * D2;
		Out.Lambda2 = S2;

		return;
		}

	float S1 = ( A22 * B1 - A12 * B2 ) / Det;
	float S2 = ( A11 * B2 - A21 * B1 ) / Det;

	Out.Point1 = P1 + S1 * D1;
	Out.Lambda1 = S1;
	Out.Point2 = P2 + S2 * D2;
	Out.Lambda2 = S2;
	}


//--------------------------------------------------------------------------------------------------
void rkClosestApproachSegments( RkClosestApproachResult& Out, const RkVector3& P1, const RkVector3& Q1, const RkVector3& P2, const RkVector3& Q2 )
	{
	RkVector3 D1 = Q1 - P1;
	RkVector3 D2 = Q2 - P2;
	RkVector3 R = P1 - P2;

	float A = rkDot( D1, D1 );
	float B = rkDot( D1, D2 );
	float C = rkDot( D1, R );
	float E = rkDot( D2, D2 );
	float F = rkDot( D2, R );

	// Check if one of the segments degenerates into a point
	if ( A < 100.0f * RK_F32_EPSILON && E < 100.0f * RK_F32_EPSILON )
		{
		// Both segments degenerate into points
		Out.Point1 = P1;
		Out.Lambda1 = 0.0f;
		Out.Point2 = P2;
		Out.Lambda2 = 0.0f;

		return;
		}

	if ( A < 100.0f * RK_F32_EPSILON )
		{
		// First segment degenerates into a point
		float S2 = rkClamp( F / E, 0.0f, 1.0f );

		Out.Point1 = P1;
		Out.Lambda1 = 0.0f;
		Out.Point2 = P2 + S2 * D2;
		Out.Lambda2 = S2;

		return;
		}

	if ( E < 100.0f * RK_F32_EPSILON )
		{
		// Second segment degenerates into a point
		float S1 = rkClamp( -C / A, 0.0f, 1.0f );

		Out.Point1 = P1 + S1 * D1;
		Out.Lambda1 = S1;
		Out.Point2 = P2;
		Out.Lambda2 = 0.0f;

		return;
		}

	// Non-degenerate case
	float Denom = A * E - B * B;
	float S1 = Denom > 1000.0f * RK_F32_MIN ? rkClamp( ( B * F - C * E ) / Denom, 0.0f, 1.0f ) : 0.0f;
	float S2 = ( B * S1 + F ) / E;

	// Clamp lambda2 and recompute lambda1 if necessary
	if ( S2 < 0.0f )
		{
		S1 = rkClamp( -C / A, 0.0f, 1.0f );
		S2 = 0.0f;
		}
	else if ( S2 > 1.0f )
		{
		S1 = rkClamp( ( B - C ) / A, 0.0f, 1.0f );
		S2 = 1.0f;
		}

	Out.Point1 = P1 + S1 * D1;
	Out.Lambda1 = S1;
	Out.Point2 = P2 + S2 * D2;
	Out.Lambda2 = S2;
	}

//--------------------------------------------------------------------------------------------------
RkVector3 rkClosestPointOnSegment( const RkVector3& A, const RkVector3& B, const RkVector3& Q )
	{
	RkVector3 AB = B - A;
	RkVector3 AQ = Q - A;

	float Alpha = rkDot( AB, AQ );

	if ( Alpha <= 0.0f )
		{
		// q projects outside interval [a, b] on the side of a 
		return A;
		}
	else
		{
		float  Denominator = rkDot( AB, AB );
		if ( Alpha > Denominator )
			{
			// q projects outside interval [a, b] on the side of b 
			return B;
			}
		else
			{
			// q projects inside interval [a, b] 
			Alpha /= Denominator;
			return A + Alpha * AB;
			}
		}
	}


//--------------------------------------------------------------------------------------------------
RkVector3 rkClosestPointOnTriangle( const RkVector3& A, const RkVector3& B, const RkVector3& C, const RkVector3& Q )
	{
	// Check if P lies in vertex region of A
	RkVector3 AB = B - A;
	RkVector3 AC = C - A;
	RkVector3 AQ = Q - A;

	float D1 = rkDot( AB, AQ );
	float D2 = rkDot( AC, AQ );
	if ( D1 <= 0.0f && D2 <= 0.0f )
		{
		return A;
		}

	// Check if P lies in vertex region of B
	RkVector3 BQ = Q - B;

	float D3 = rkDot( AB, BQ );
	float D4 = rkDot( AC, BQ );
	if ( D3 >= 0.0f && D4 <= D3 )
		{
		return B;
		}

	// Check if P lies in edge region AB
	float VC = D1 * D4 - D3 * D2;
	if ( VC <= 0.0f && D1 >= 0.0f && D3 <= 0.0f )
		{
		float T = D1 / ( D1 - D3 );
		return A + T * AB;
		}

	// Check if P lies in vertex region of C
	RkVector3 CQ = Q - C;

	float D5 = rkDot( AB, CQ );
	float D6 = rkDot( AC, CQ );
	if ( D6 >= 0.0f && D5 <= D6 )
		{
		return C;
		}


	// Check if P lies in edge region AC
	float VB = D5 * D2 - D1 * D6;
	if ( VB <= 0.0f && D2 >= 0.0f && D6 <= 0.0f )
		{
		float T = D2 / ( D2 - D6 );
		return A + T * AC;
		}


	// Check if P lies in edge region of BC
	float VA = D3 * D6 - D5 * D4;
	if ( VA <= 0.0f && D4 >= D3 && D5 >= D6 )
		{
		RkVector3 BC = C - B;

		float T = ( D4 - D3 ) / ( ( D4 - D3 ) + ( D5 - D6 ) );
		return B + T * BC;
		}

	// P inside face region ABC
	float T1 = VB / ( VA + VB + VC );
	float T2 = VC / ( VA + VB + VC );

	return A + T1 * AB + T2 * AC;
	}


//--------------------------------------------------------------------------------------------------
float rkIntersectSegmentTriangle( const RkVector3& P, const RkVector3& Q, const RkVector3& A, const RkVector3& B, const RkVector3& C )
	{
	RkVector3 AB = B - A;
	RkVector3 AC = C - A;
	RkVector3 QP = P - Q;
	RkVector3 N = rkCross( AB, AC );

	float D = rkDot( QP, N );
	if ( D <= 0.0f )
		{
		return 1.0f;
		}

	RkVector3 AP = P - A;
	float T = rkDot( AP, N );
	if ( 0.0f > T || T > D )
		{
		return 1.0f;
		}

	RkVector3 E = rkCross( QP, AP );
	float V = rkDot( AC, E );
	if ( 0.0f > V || V > D )
		{
		return 1.0f;
		}

	float W = -rkDot( AB, E );
	if ( 0.0f > W || V + W > D )
		{
		return 1.0f;
		}

	return T / D;
	}


//--------------------------------------------------------------------------------------------------
float rkIntersectSegmentSphere( const RkVector3& P, const RkVector3& Q, const RkVector3& C, float R )
	{
	RkVector3 PQ = Q - P;
	RkVector3 CP = P - C;

	float K1 = rkDot( CP, CP ) - R * R;
	if ( K1 <= 0.0f )
		{
		// Initially intersecting
		return 0.0f;
		}

	float K2 = rkDot( PQ, CP );
	if ( K2 > 0.0f )
		{
		// Initially *not* intersecting and ray pointing away from sphere
		return 1.0f;
		}

	float K3 = rkDot( PQ, PQ );
	if ( K3 < 100.0f * RK_F32_EPSILON )
		{
		// Initially *not* intersecting and ray degenerates into point
		return 1.0f;
		}

	float Discriminant = K2 * K2 - K3 * K1;
	if ( Discriminant < 0.0f )
		{
		// No real roots -> no intersection
		return 1.0f;
		}

	float T = -( K2 + rkSqrt( Discriminant ) ) / K3;
	RK_ASSERT( T >= 0.0f );

	return T;
	}



//--------------------------------------------------------------------------------------------------
// General matrix inversion
//--------------------------------------------------------------------------------------------------
void rkInvert( float* Out, float* M, int N, int Lda )
	{
	float* pr_in = M;
	float* pr_out = Out;

	// Initialize identity matrix
		{  
		float* p = pr_out;
		rkMemZero( p, Lda * N * sizeof( float ) );
		
		for ( int k = N - 1; k >= 0; k-- )
			{
			p[ 0 ] = 1.0f;
			p += Lda + 1;
			}
		}

	// Forward step
		{
		for ( int i = 0; i < N; i++ )
			{
			float f = pr_in[ i ];

			f = 1.0f / f;
				{ 
				// scale this row
				int s = i;
				do
				{
					pr_in[ s ] *= f;
				}
				while ( ++s < N );

				int t = 0;
				do
				{
					pr_out[ t ] *= f;
				}
				while ( ++t <= i );
			}
	
			float* row_to_zero = pr_in;
			float* inverted_row = pr_out;

			for ( int row = i + 1; row < N; row++ )
				{
				row_to_zero += Lda;
				inverted_row += Lda;

				float factor = row_to_zero[ i ];
				if ( factor == 0.0f )
					{ 
					// matrix elem is already zero
					continue;
					}

				int k = 0;
				do
					{
					inverted_row[ k ] -= pr_out[ k ] * factor;
					}
				while ( ++k <= i );

				k = i;
				do
					{
					row_to_zero[ k ] -= pr_in[ k ] * factor;
					}
				while ( ++k < N );
				}

			pr_in += Lda;
			pr_out += Lda;
			}
		}

	// Reverse step (diagonal is already scaled)
		{
		for ( int i = N - 1; i >= 0; i-- )
			{
			pr_in -= Lda;
			pr_out -= Lda;

			float* row_to_zero = pr_in;
			float* inverted_row = pr_out;

			for ( int row = i - 1; row >= 0; row-- )
				{
				row_to_zero -= Lda;
				inverted_row -= Lda;

				float factor = row_to_zero[ i ];
				if ( factor == 0.0f )
					{ 
					// matrix elem is already zero
					continue;
					}

				int k = N - 1;
				do
					{
					inverted_row[ k ] -= pr_out[ k ] * factor;
					}
				while ( --k >= 0 );

				k = N - 1;
				do
					{
					row_to_zero[ k ] -= pr_in[ k ] * factor;
					}
				while ( --k >= i );
				}
			}
		}
	}