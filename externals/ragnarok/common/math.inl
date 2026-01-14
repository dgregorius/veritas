//--------------------------------------------------------------------------------------------------
// math.inl	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkMath
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkSign( float X )
	{
	return copysignf( 1.0f, X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkFloor( float X )
	{
	return floorf( X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkCeil( float X )
	{
	return ceilf( X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkRound( float X )
	{
	return roundf( X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkPow( float X, float Y )
	{
	return powf( X, Y );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkExp( float X )
	{
	return expf( X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkLog( float X )
	{
	return logf( X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkSqrt( float X )
	{
	return sqrtf( X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkSquare( float X )
	{
	return X * X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkCube( float X )
	{
	return X * X * X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkMod( float X, float Y )
	{
	return fmodf( X, Y );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkSin( float Rad )
	{
	return sinf( Rad );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkCos( float Rad )
	{
	return cosf( Rad );
	}

//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkTan( float Rad )
	{
	return tanf( Rad );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkASin( float X )
	{
	return asinf( X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkACos( float X )
	{
	return acosf( X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkATan( float X )
	{
	return atanf( X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkATan2( float Y, float X )
	{
	return atan2f( Y, X );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > 
RK_FORCEINLINE T rkAbs( T X )
	{
	return X > T() ? X : -X;
	}


//--------------------------------------------------------------------------------------------------
template< typename T > 
RK_FORCEINLINE T rkMin( T X, T Y )
	{
	return X < Y ? X : Y;
	}


//--------------------------------------------------------------------------------------------------
template< typename T > 
RK_FORCEINLINE T rkMax( T X, T Y )
	{
	return X < Y ? Y : X;
	}


//--------------------------------------------------------------------------------------------------
template< typename T > 
RK_FORCEINLINE T rkClamp( T X, T Min, T Max )
	{
	return rkMax( Min, rkMin( X, Max ) );
	}


//--------------------------------------------------------------------------------------------------
template< typename T >
RK_FORCEINLINE T rkNormalize( T X, T Min, T Max )
	{
	X = rkClamp( X, Min, Max );
	return ( X - Min ) / ( Max - Min );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool rkFuzzyEqual( float X, float Y, float Epsilon )
	{
	// Relative tolerance
	return rkAbs( X - Y ) < Epsilon * rkMax( 1.0f, rkMax( rkAbs( X ), rkAbs( Y ) ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool rkFuzzyZero( float X, float Epsilon )
	{
	// Absolute tolerance
	return rkAbs( X ) < Epsilon;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkBias( float T, float Bias )
	{
	RK_ASSERT( 0.0f <= T && T <= 1.0f );
	return T / ( ( ( ( 1.0f / Bias ) - 2.0f ) * ( 1.0f - T ) ) + 1.0f );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkGain( float T, float Gain )
	{
	if ( T < 0.5f )
		return rkBias( T * 2.0f, Gain ) / 2.0f;
	else
		return rkBias( T * 2.0f - 1.0f, 1.0f - Gain ) / 2.0f + 0.5f;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkLerp( float X1, float X2, float Alpha )
	{
	RK_ASSERT( 0.0f <= Alpha && Alpha <= 1.0f );
	return ( 1.0f - Alpha ) * X1 + Alpha * X2;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE int rkRandom( int Min, int Max )
	{
	int R = rand();
	R = R % ( Max - Min + 1 ) + Min;

	return R;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkRandom( float Min, float Max )
	{
	float R = float( rand() );
	R /= RAND_MAX;
	R = ( Max - Min ) * R + Min;

	return R;
	}


//--------------------------------------------------------------------------------------------------
// RkVector2
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2::RkVector2( float X, float Y )
	: X( X )
	, Y( Y )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2::RkVector2( const float* V )
	: X( V[ 0 ] )
	, Y( V[ 1 ] )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2::RkVector2( const double* V )
	: X( static_cast< float >( V[ 0 ] ) )
	, Y( static_cast< float >( V[ 1 ] ) )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2::operator float*()
	{
	return &X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2::operator const float*() const
	{
	return &X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2& RkVector2::operator*=( const RkVector2& V )
	{
	X *= V.X;
	Y *= V.Y;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2& RkVector2::operator+=( const RkVector2& V )
	{
	X += V.X;
	Y += V.Y;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2& RkVector2::operator-=( const RkVector2& V )
	{
	X -= V.X;
	Y -= V.Y;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2& RkVector2::operator*=( float F )
	{
	X *= F;
	Y *= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2& RkVector2::operator/=( float F )
	{
	X /= F;
	Y /= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 RkVector2::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 RkVector2::operator-() const
	{
	return RkVector2( -X, -Y );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 operator*( const RkMatrix2& M, const RkVector2& V )
	{
	return M.C1 * V.X + M.C2 * V.Y;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 operator*( const RkVector2& V1, const RkVector2& V2 )
	{
	RkVector2 Out;
	Out.X = V1.X * V2.X;
	Out.Y = V1.Y * V2.Y;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 operator/( const RkVector2& V1, const RkVector2& V2 )
	{
	RkVector2 Out;
	Out.X = V1.X / V2.X;
	Out.Y = V1.Y / V2.Y;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 operator+( const RkVector2& V1, const RkVector2& V2 )
	{
	RkVector2 Out;
	Out.X = V1.X + V2.X;
	Out.Y = V1.Y + V2.Y;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 operator-( const RkVector2& V1, const RkVector2& V2 )
	{
	RkVector2 Out;
	Out.X = V1.X - V2.X;
	Out.Y = V1.Y - V2.Y;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 operator*( float F, const RkVector2& V )
	{
	RkVector2 Out;
	Out.X = F * V.X;
	Out.Y = F * V.Y;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 operator*( const RkVector2& V, float F )
	{
	RkVector2 Out;
	Out.X = V.X * F;
	Out.Y = V.Y * F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 operator/( const RkVector2& V, float F )
	{
	RkVector2 Out;
	Out.X = V.X / F;
	Out.Y = V.Y / F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator<( const RkVector2& V1, const RkVector2& V2 )
	{
	if ( V1.X == V2.X )
		{
		return V1.Y < V2.Y;
		}

	return V1.X < V2.X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkVector2& V1, const RkVector2& V2 )
	{
	return V1.X == V2.X && V1.Y == V2.Y;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkVector2& V1, const RkVector2& V2 )
	{
	return V1.X != V2.X || V1.Y != V2.Y;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkMul( const RkMatrix2& M, const RkVector2& V )
	{
	return M.C1 * V.X + M.C2 * V.Y;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkTMul( const RkMatrix2& M, const RkVector2& V )
	{
	RkVector2 Out;
	Out.X = rkDot( M.C1, V );
	Out.Y = rkDot( M.C2, V );
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkNormalize( const RkVector2& V )
	{
	float LengthSq = rkLengthSq( V );
	if ( LengthSq > 1000.0f * RK_F32_MIN )
		{
		return V / rkSqrt( LengthSq );
		}
	
	return RK_VEC2_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkInvert( const RkVector2& V )
	{
	RkVector2 Out;
	Out.X = 1.0f / V.X;
	Out.Y = 1.0f / V.Y;
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkPerp( const RkVector2& V )
	{
	return RkVector2( -V.Y, V.X );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkAbs( const RkVector2& V )
	{
	RkVector2 Out;
	Out.X = rkAbs( V.X );
	Out.Y = rkAbs( V.Y );
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkCross( const RkVector2& V1, const RkVector2& V2 )
	{
	return V1.X * V2.Y - V1.Y * V2.X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDot( const RkVector2& V1, const RkVector2& V2 )
	{
	return V1.X * V2.X + V1.Y * V2.Y;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkLength( const RkVector2& V )
	{
	return rkSqrt( rkDot( V, V ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkLengthSq( const RkVector2& V )
	{
	return rkDot( V, V );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDistance( const RkVector2& V1, const RkVector2& V2 )
	{
	return rkLength( V1 - V2 );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDistanceSq( const RkVector2& V1, const RkVector2& V2 )
	{
	return rkLengthSq( V1 - V2 );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkMin( const RkVector2& V1, const RkVector2& V2 )
	{
	RkVector2 Out;
	Out.X = rkMin( V1.X, V2.X );
	Out.Y = rkMin( V1.Y, V2.Y );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkMax( const RkVector2& V1, const RkVector2& V2 )
	{
	RkVector2 Out;
	Out.X = rkMax( V1.X, V2.X );
	Out.Y = rkMax( V1.Y, V2.Y );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkClamp( const RkVector2& V, const RkVector2& Min, const RkVector2& Max )
	{
	RkVector2 Out;
	Out.X = rkClamp( V.X, Min.X, Max.X );
	Out.Y = rkClamp( V.Y, Min.Y, Max.Y );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkClamp( const RkVector2& V, float Max )
	{
	float LengthSq = rkLengthSq( V );
	if ( LengthSq <= Max * Max )
		{
		return V;
		}

	float Length = rkSqrt( LengthSq );
	return V * ( Max / Length );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 rkLerp( const RkVector2& V1, const RkVector2& V2, float Alpha )
	{
	RK_ASSERT( 0.0f <= Alpha && Alpha <= 1.0f );

	RkVector2 Out;
	Out.X = ( 1.0f - Alpha ) * V1.X + Alpha * V2.X;
	Out.Y = ( 1.0f - Alpha ) * V1.Y + Alpha * V2.Y;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
// RkVector3
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3::RkVector3( float F )
	: X( F )
	, Y( F )
	, Z( F )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3::RkVector3( float X, float Y, float Z )
	: X( X )
	, Y( Y )
	, Z( Z )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3::RkVector3( const float* V )
	: X( V[ 0 ] )
	, Y( V[ 1 ] )
	, Z( V[ 2 ] )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3::RkVector3( const double* V )
	: X( static_cast< float >( V[ 0 ] ) ) 
	, Y( static_cast< float >( V[ 1 ] ) )
	, Z( static_cast< float >( V[ 2 ] ) )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3::operator float*()
	{
	return &X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3::operator const float*() const
	{
	return &X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3& RkVector3::operator*=( const RkVector3& V )
	{
	X *= V.X;
	Y *= V.Y;
	Z *= V.Z;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3& RkVector3::operator+=( const RkVector3& V )
	{
	X += V.X;
	Y += V.Y;
	Z += V.Z;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3& RkVector3::operator-=( const RkVector3& V )
	{
	X -= V.X;
	Y -= V.Y;
	Z -= V.Z;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3& RkVector3::operator*=( float F )
	{
	X *= F;
	Y *= F;
	Z *= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3& RkVector3::operator/=( float F )
	{
	X /= F;
	Y /= F;
	Z /= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 RkVector3::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 RkVector3::operator-() const
	{
	RkVector3 Out;
	Out.X = -X;
	Out.Y = -Y;
	Out.Z = -Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 operator*( const RkMatrix3& M, const RkVector3& V )
	{
	return M.C1 * V.X + M.C2 * V.Y + M.C3 * V.Z;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 operator*( const RkQuaternion& Q, const RkVector3& V )
	{
	RK_ASSERT( rkAbs( 1.0f - rkLength( Q ) ) < 100.0f * RK_F32_EPSILON );
	return V + 2.0f * rkCross( Q.V, rkCross( Q.V, V ) + Q.S * V );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 operator*( const RkTransform& T, const RkVector3& V )
	{
	return T.Rotation * V + T.Translation;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 operator*( const RkVector3& V1, const RkVector3& V2 )
	{
	RkVector3 Out;
	Out.X = V1.X * V2.X;
	Out.Y = V1.Y * V2.Y;
	Out.Z = V1.Z * V2.Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 operator/( const RkVector3& V1, const RkVector3& V2 )
	{
	RkVector3 Out;
	Out.X = V1.X / V2.X;
	Out.Y = V1.Y / V2.Y;
	Out.Z = V1.Z / V2.Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 operator+( const RkVector3& V1, const RkVector3& V2 )
	{
	RkVector3 Out;
	Out.X = V1.X + V2.X;
	Out.Y = V1.Y + V2.Y;
	Out.Z = V1.Z + V2.Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 operator-( const RkVector3& V1, const RkVector3& V2 )
	{
	RkVector3 Out;
	Out.X = V1.X - V2.X;
	Out.Y = V1.Y - V2.Y;
	Out.Z = V1.Z - V2.Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 operator*( float F, const RkVector3& V )
	{
	RkVector3 Out;
	Out.X = F * V.X;
	Out.Y = F * V.Y;
	Out.Z = F * V.Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 operator*( const RkVector3& V, float F )
	{
	RkVector3 Out;
	Out.X = V.X * F;
	Out.Y = V.Y * F;
	Out.Z = V.Z * F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 operator/( const RkVector3& V, float F )
	{
	RkVector3 Out;
	Out.X = V.X / F;
	Out.Y = V.Y / F;
	Out.Z = V.Z / F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator<( const RkVector3& V1, const RkVector3& V2 )
	{
	if ( V1.X == V2.X )
		{
		if ( V1.Y == V2.Y )
			{
			return V1.Z < V2.Z;
			}

		return V1.Y < V2.Y;
		}

	return V1.X < V2.X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkVector3& V1, const RkVector3& V2 )
	{
	return V1.X == V2.X && V1.Y == V2.Y && V1.Z == V2.Z;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkVector3& V1, const RkVector3& V2 )
	{
	return V1.X != V2.X || V1.Y != V2.Y || V1.Z != V2.Z;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkMul( const RkMatrix3& M, const RkVector3& V )
	{
	return M.C1 * V.X + M.C2 * V.Y + M.C3 * V.Z;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkTMul( const RkMatrix3& M, const RkVector3& V )
	{
	RkVector3 Out;
	Out.X = rkDot( M.C1, V );
	Out.Y = rkDot( M.C2, V );
	Out.Z = rkDot( M.C3, V );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkMul( const RkQuaternion& Q, const RkVector3& V )
	{
	RK_ASSERT( rkAbs( 1.0f - rkLength( Q ) ) < 100.0f * RK_F32_EPSILON );
	return V + 2.0f * rkCross( Q.V, rkCross( Q.V, V ) + Q.S * V );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkTMul( const RkQuaternion& Q, const RkVector3& V )
	{
	RK_ASSERT( rkAbs( 1.0f - rkLength( Q ) ) < 100.0f * RK_F32_EPSILON );
	return V - 2.0f * rkCross( Q.V, Q.S * V - rkCross( Q.V, V ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkMul( const RkTransform& T, const RkVector3& V )
	{
	return rkMul( T.Rotation, V ) + T.Translation;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkTMul( const RkTransform& T, const RkVector3& V )
	{
	return rkTMul( T.Rotation, V - T.Translation );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkCross( const RkVector3& V1, const RkVector3& V2 )
	{
	RkVector3 Out;
	Out.X = V1.Y * V2.Z - V1.Z * V2.Y;
	Out.Y = V1.Z * V2.X - V1.X * V2.Z;
	Out.Z = V1.X * V2.Y - V1.Y * V2.X;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkNormalize( const RkVector3& V )
	{
	float LengthSq = rkLengthSq( V );
	if ( LengthSq > 1000.0f * RK_F32_MIN )
		{
		return V / rkSqrt( LengthSq );
		}

	return RK_VEC3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkInvert( const RkVector3& V )
	{
	RkVector3 Out;
	Out.X = 1.0f / V.X;
	Out.Y = 1.0f / V.Y;
	Out.Z = 1.0f / V.Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkPerp( const RkVector3& V )
	{
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	return rkNormalize( rkAbs( V.X ) >= 0.57735f ? RkVector3( V.Y, -V.X, 0.0f ) : RkVector3( 0.0f, V.Z, -V.Y ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkAbs( const RkVector3& V )
{
	RkVector3 Out;
	Out.X = rkAbs( V.X );
	Out.Y = rkAbs( V.Y );
	Out.Z = rkAbs( V.Z );

	return Out;
}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDot( const RkVector3& V1, const RkVector3& V2 )
	{
	return V1.X * V2.X + V1.Y * V2.Y + V1.Z * V2.Z;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkLength( const RkVector3& V )
	{
	return rkSqrt( rkDot( V, V ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkLengthSq( const RkVector3& V )
	{
	return rkDot( V, V );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDistance( const RkVector3& V1, const RkVector3& V2 )
	{
	return rkLength( V1 - V2 );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDistanceSq( const RkVector3& V1, const RkVector3& V2 )
	{
	return rkLengthSq( V1 - V2 );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDet( const RkVector3& V1, const RkVector3& V2, const RkVector3& V3 )
	{
	return rkDot( V1, rkCross( V2, V3 ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkMin( const RkVector3& V1, const RkVector3& V2 )
	{
	RkVector3 Out;
	Out.X = rkMin( V1.X, V2.X );
	Out.Y = rkMin( V1.Y, V2.Y );
	Out.Z = rkMin( V1.Z, V2.Z );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkMax( const RkVector3& V1, const RkVector3& V2 )
	{
	RkVector3 Out;
	Out.X = rkMax( V1.X, V2.X );
	Out.Y = rkMax( V1.Y, V2.Y );
	Out.Z = rkMax( V1.Z, V2.Z );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkClamp( const RkVector3& V, const RkVector3& Min, const RkVector3& Max )
	{
	RkVector3 Out;
	Out.X = rkClamp( V.X, Min.X, Max.X );
	Out.Y = rkClamp( V.Y, Min.Y, Max.Y );
	Out.Z = rkClamp( V.Z, Min.Z, Max.Z );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkClamp( const RkVector3& V, float Max )
	{
	float LengthSq = rkLengthSq( V );
	if ( LengthSq <= Max * Max )
		{
		return V;
		}

	float Length = rkSqrt( LengthSq );
	return V * ( Max / Length );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE int rkMinorAxis( const RkVector3& V )
	{
	return V.X < V.Y ? ( V.X < V.Z ? 0 : 2 ) : ( V.Y < V.Z ? 1 : 2 );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE int rkMajorAxis( const RkVector3& V )
	{
	return V.X < V.Y ? ( V.Y < V.Z ? 2 : 1 ) : ( V.X < V.Z ? 2 : 0 );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkMinElement( const RkVector3& V )
	{
	return rkMin( V.X, rkMin( V.Y, V.Z ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkMaxElement( const RkVector3& V )
	{
	return rkMax( V.X, rkMax( V.Y, V.Z ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkLerp( const RkVector3& V1, const RkVector3& V2, float Alpha )
	{
	RK_ASSERT( 0.0f <= Alpha && Alpha <= 1.0f );

	RkVector3 Out;
	Out.X = ( 1.0f - Alpha ) * V1.X + Alpha * V2.X;
	Out.Y = ( 1.0f - Alpha ) * V1.Y + Alpha * V2.Y;
	Out.Z = ( 1.0f - Alpha ) * V1.Z + Alpha * V2.Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkNLerp( const RkVector3& V1, const RkVector3& V2, float Alpha )
	{
	RK_ASSERT( 0.0f <= Alpha && Alpha <= 1.0f );

	return rkNormalize( rkLerp( V1, V2, Alpha ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkTransformPoint( const RkMatrix4& M, const RkVector3& P )
	{
	RkVector4 Out = M * RkVector4( P.X, P.Y, P.Z, 1.0f );
	return RkVector3( Out.X, Out.Y, Out.Z );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkTransformVector( const RkMatrix4& M, const RkVector3& V )
	{
	RkVector4 Out = M * RkVector4( V.X, V.Y, V.Z, 0.0f );
	return RkVector3( Out.X, Out.Y, Out.Z );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkProject( const RkVector3& Coord, RkMatrix4& ModelView, const RkMatrix4& Projection, const RkViewport& Viewport )
	{
	RkVector4 Point( Coord, 1.0f );
	Point = Projection * ModelView * Point;
	if ( rkAbs( Point.W ) < 100.0f * RK_F32_EPSILON )
		{
		Point.W = 1.0f;
		}
	Point /= Point.W;

	Point = 0.5f * Point + RkVector4( 0.5f, 0.5f, 0.5f, 0.5f );
	Point.X = Point.X * Viewport.Width + Viewport.X;
	Point.Y = Point.Y * Viewport.Height + Viewport.Y;

	return RkVector3( Point.X, Point.Y, Point.Z );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkUnproject( const RkVector3& Coord, RkMatrix4& ModelView, const RkMatrix4& Projection, const RkViewport& Viewport )
	{
	RkVector4 Point( Coord, 1.0f );
	Point.X = ( Point.X - Viewport.X ) / Viewport.Width;
	Point.Y = ( Point.Y - Viewport.Y ) / Viewport.Height;
	Point = 2.0f * Point - RkVector4( 1.0f, 1.0f, 1.0f, 1.0f );

	Point = rkInvert( Projection * ModelView ) * Point;
	if ( rkAbs( Point.W ) < 100.0f * RK_F32_EPSILON )
		{
		Point.W = 1.0f;
		}
	Point /= Point.W;

	return RkVector3( Point.X, Point.Y, Point.Z );
	}


//--------------------------------------------------------------------------------------------------
// RkVector4
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4::RkVector4( float X, float Y, float Z, float W )
	: X( X )
	, Y( Y )
	, Z( Z )
	, W( W )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4::RkVector4( const RkVector3& V, float W )
	: X( V.X )
	, Y( V.Y )
	, Z( V.Z )
	, W( W )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4::RkVector4( const float* V )
	: X( V[ 0 ] )
	, Y( V[ 1 ] )
	, Z( V[ 2 ] )
	, W( V[ 3 ] )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4::RkVector4( const double* V )
	: X( static_cast< float >( V[ 0 ] ) )
	, Y( static_cast< float >( V[ 1 ] ) )
	, Z( static_cast< float >( V[ 2 ] ) )
	, W( static_cast< float >( V[ 3 ] ) )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4::operator float*()
	{
	return &X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4::operator const float*() const
	{
	return &X;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4& RkVector4::operator*=( const RkVector4& V )
	{
	X *= V.X;
	Y *= V.Y;
	Z *= V.Z;
	W *= V.W;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4& RkVector4::operator+=( const RkVector4& V )
	{
	X += V.X;
	Y += V.Y;
	Z += V.Z;
	W += V.W;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4& RkVector4::operator-=( const RkVector4& V )
	{
	X -= V.X;
	Y -= V.Y;
	Z -= V.Z;
	W -= V.W;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4& RkVector4::operator*=( float F )
	{
	X *= F;
	Y *= F;
	Z *= F;
	W *= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4& RkVector4::operator/=( float F )
	{
	X /= F;
	Y /= F;
	Z /= F;
	W /= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 RkVector4::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 RkVector4::operator-() const
	{
	RkVector4 Out;
	Out.X = -X;
	Out.Y = -Y;
	Out.Z = -Z;
	Out.W = -W;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 operator*( const RkMatrix4& M, const RkVector4& V )
	{
	return M.C1 * V.X + M.C2 * V.Y + M.C3 * V.Z + M.C4 * V.W;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 operator*( const RkVector4& V1, const RkVector4& V2 )
	{
	RkVector4 Out;
	Out.X = V1.X * V2.X;
	Out.Y = V1.Y * V2.Y;
	Out.Z = V1.Z * V2.Z;
	Out.W = V1.W * V2.W;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 operator+( const RkVector4& V1, const RkVector4& V2 )
	{
	RkVector4 Out;
	Out.X = V1.X + V2.X;
	Out.Y = V1.Y + V2.Y;
	Out.Z = V1.Z + V2.Z;
	Out.W = V1.W + V2.W;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 operator-( const RkVector4& V1, const RkVector4& V2 )
	{
	RkVector4 Out;
	Out.X = V1.X - V2.X;
	Out.Y = V1.Y - V2.Y;
	Out.Z = V1.Z - V2.Z;
	Out.W = V1.W - V2.W;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 operator*( float S, const RkVector4& V )
	{
	RkVector4 Out;
	Out.X = S * V.X;
	Out.Y = S * V.Y;
	Out.Z = S * V.Z;
	Out.W = S * V.W;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 operator*( const RkVector4& V, float S )
	{
	RkVector4 Out;
	Out.X = V.X * S;
	Out.Y = V.Y * S;
	Out.Z = V.Z * S;
	Out.W = V.W * S;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 operator/( const RkVector4& V, float S )
	{
	RkVector4 Out;
	Out.X = V.X / S;
	Out.Y = V.Y / S;
	Out.Z = V.Z / S;
	Out.W = V.W / S;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkVector4& V1, const RkVector4& V2 )
	{
	return V1.X == V2.X && V1.Y == V2.Y && V1.Z == V2.Z && V1.W == V2.W;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkVector4& V1, const RkVector4& V2 )
	{
	return V1.X != V2.X || V1.Y != V2.Y || V1.Z != V2.Z || V1.W != V2.W;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 rkNormalize( const RkVector4& V )
	{
	float LengthSq = rkLengthSq( V );
	if ( LengthSq > 1000.0f *RK_F32_MIN )
		{
		return V / rkSqrt( LengthSq );
		}

	return RK_VEC4_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDot( const RkVector4& V1, const RkVector4& V2 )
	{
	return V1.X * V2.X + V1.Y * V2.Y + V1.Z * V2.Z + V1.W * V2.W;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkLength( const RkVector4& V )
	{
	return rkSqrt( V.X * V.X + V.Y * V.Y + V.Z * V.Z + V.W * V.W );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkLengthSq( const RkVector4& V )
	{
	return V.X * V.X + V.Y * V.Y + V.Z * V.Z + V.W * V.W;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDistance( const RkVector4& V1, const RkVector4& V2 )
	{
	return rkSqrt( ( V1.X - V2.X ) * ( V1.X - V2.X ) + ( V1.Y - V2.Y ) * ( V1.Y - V2.Y ) + ( V1.Z - V2.Z ) * ( V1.Z - V2.Z ) + ( V1.W - V2.W ) * ( V1.W - V2.W ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDistanceSq( const RkVector4& V1, const RkVector4& V2 )
	{
	return ( V1.X - V2.X ) * ( V1.X - V2.X ) + ( V1.Y - V2.Y ) * ( V1.Y - V2.Y ) + ( V1.Z - V2.Z ) * ( V1.Z - V2.Z ) + ( V1.W - V2.W ) * ( V1.W - V2.W );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 rkMin( const RkVector4& V1, const RkVector4& V2 )
	{
	RkVector4 Out;
	Out.X = rkMin( V1.X, V2.X );
	Out.Y = rkMin( V1.Y, V2.Y );
	Out.Z = rkMin( V1.Z, V2.Z );
	Out.W = rkMin( V1.W, V2.W );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 rkMax( const RkVector4& V1, const RkVector4& V2 )
	{
	RkVector4 Out;
	Out.X = rkMax( V1.X, V2.X );
	Out.Y = rkMax( V1.Y, V2.Y );
	Out.Z = rkMax( V1.Z, V2.Z );
	Out.W = rkMax( V1.W, V2.W );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 rkClamp( const RkVector4& V, const RkVector4& Min, const RkVector4& Max )
	{
	RkVector4 Out;
	Out.X = rkClamp( V.X, Min.X, Max.X );
	Out.Y = rkClamp( V.Y, Min.Y, Max.Y );
	Out.Z = rkClamp( V.Z, Min.Z, Max.Z );
	Out.W = rkClamp( V.W, Min.W, Max.W );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector4 rkClamp( const RkVector4& V, float Max )
	{
	float LengthSq = rkLengthSq( V );
	if ( LengthSq <= Max * Max )
		{
		return V;
		}

	float Length = rkSqrt( LengthSq );
	return V * ( Max / Length );
	}


//--------------------------------------------------------------------------------------------------
// RkMatrix2
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2::RkMatrix2( float A11, float A22 )
	: C1( A11, 0.0f )
	, C2( 0.0f, A22 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2::RkMatrix2( float A11, float A12, float A21, float A22 )
	: C1( A11, A21 )
	, C2( A12, A22 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2::RkMatrix2( const RkVector2& C1, const RkVector2& C2 )
	: C1( C1 )
	, C2( C2 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2::RkMatrix2( const float* M )
	: C1( M + 0 )
	, C2( M + 2 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2::RkMatrix2( const double* M )
	: C1( M + 0 )
	, C2( M + 2 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2::operator float*( )
	{
	return &A11;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2::operator const float*( ) const
	{
	return &A11;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2& RkMatrix2::operator*=( const RkMatrix2& M )
	{
	*this = *this * M;;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2& RkMatrix2::operator+=( const RkMatrix2& M )
	{
	C1 += M.C1;
	C2 += M.C2;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2& RkMatrix2::operator-=( const RkMatrix2& M )
	{
	C1 -= M.C1;
	C2 -= M.C2;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2& RkMatrix2::operator*=( float F )
	{
	C1 *= F;
	C2 *= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2& RkMatrix2::operator/=( float F )
	{
	C1 /= F;
	C2 /= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 RkMatrix2::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 RkMatrix2::operator-() const
	{
	return RkMatrix2( -C1, -C2 );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 operator*( const RkMatrix2& M1, const RkMatrix2& M2 )
	{
	RkMatrix2 Out;
	Out.C1 = M1 * M2.C1;
	Out.C2 = M1 * M2.C2;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 operator+( const RkMatrix2& M1, const RkMatrix2& M2 )
	{
	RkMatrix2 Out;
	Out.C1 = M1.C1 + M2.C1;
	Out.C2 = M1.C2 + M2.C2;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 operator-( const RkMatrix2& M1, const RkMatrix2& M2 )
	{
	RkMatrix2 Out;
	Out.C1 = M1.C1 - M2.C1;
	Out.C2 = M1.C2 - M2.C2;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 operator*( float F, const RkMatrix2& M )
	{
	RkMatrix2 Out;
	Out.C1 = F * M.C1;
	Out.C2 = F * M.C2;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 operator*( const RkMatrix2& M, float F )
	{
	RkMatrix2 Out;
	Out.C1 = M.C1 * F;
	Out.C2 = M.C2 * F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 operator/( const RkMatrix2& M, float F )
	{
	RkMatrix2 Out;
	Out.C1 = M.C1 / F;
	Out.C2 = M.C2 / F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkMatrix2& M1, const RkMatrix2& M2 )
	{
	return M1.C1 == M2.C1 && M1.C2 == M2.C2;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkMatrix2& M1, const RkMatrix2& M2 )
	{
	return M1.C1 != M2.C1 || M1.C2 != M2.C2;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 rkMul( const RkMatrix2& M1, const RkMatrix2& M2 )
	{
	RkMatrix2 Out;
	Out.C1 = rkMul( M1, M2.C1 );
	Out.C2 = rkMul( M1, M2.C2 );
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 rkTMul( const RkMatrix2& M1, const RkMatrix2& M2 )
	{
	RkMatrix2 Out;
	Out.C1 = rkTMul( M1, M2.C1 );
	Out.C2 = rkTMul( M1, M2.C2 );
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 rkTranspose( const RkMatrix2& M )
	{
	RkMatrix2 Out;
	Out.C1 = RkVector2( M.C1.X, M.C2.X );
	Out.C2 = RkVector2( M.C1.Y, M.C2.Y );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 rkInvert( const RkMatrix2& M )
	{
	float Det = rkDet( M );
	if ( rkAbs( Det ) > 1000.0f * RK_F32_MIN )
		{
		RkMatrix2 Out;
		Out.C1 = RkVector2( M.C2.Y, -M.C1.Y ) / Det;
		Out.C2 = RkVector2( -M.C2.X, M.C1.X ) / Det;

		return Out;
		}

	return RK_MAT2_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 rkInvertT( const RkMatrix2& M )
	{
	float Det = rkDet( M );
	if ( rkAbs( Det ) > 1000.0f * RK_F32_MIN )
		{
		RkMatrix2 Out;
		Out.C1 = RkVector2( M.C2.Y, -M.C2.X ) / Det;
		Out.C2 = RkVector2( -M.C1.Y, M.C1.X ) / Det;

		return Out;
		}

	return RK_MAT2_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix2 rkAbs( const RkMatrix2& M )
	{
	RkMatrix2 Out;
	Out.C1 = rkAbs( M.C1 );
	Out.C2 = rkAbs( M.C2 );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkTrace( const RkMatrix2& M )
	{
	return M.C1.X + M.C2.Y;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDet( const RkMatrix2& M )
	{
	return M.C1.X * M.C2.Y - M.C1.Y * M.C2.X;
	}


//--------------------------------------------------------------------------------------------------
// RkMatrix3
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3::RkMatrix3( float A11, float A22, float A33 )
	: C1(  A11, 0.0f, 0.0f )
	, C2( 0.0f,  A22, 0.0f )
	, C3( 0.0f, 0.0f,  A33 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3::RkMatrix3( const RkVector3& C1, const RkVector3& C2, const RkVector3& C3 )
	: C1( C1 )
	, C2( C2 )
	, C3( C3 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3::RkMatrix3( const RkQuaternion& Q )
	{
	float XX = Q.X * Q.X;
	float YY = Q.Y * Q.Y;
	float ZZ = Q.Z * Q.Z;

	float XY = Q.X * Q.Y;
	float XZ = Q.X * Q.Z;
	float XW = Q.X * Q.W;
	float YZ = Q.Y * Q.Z;
	float YW = Q.Y * Q.W;
	float ZW = Q.Z * Q.W;

	C1 = RkVector3( 1.0f - 2.0f * ( YY + ZZ ), 2.0f * ( XY + ZW ), 2.0f * ( XZ - YW ) );
	C2 = RkVector3( 2.0f * ( XY - ZW ), 1.0f - 2.0f * ( XX + ZZ ), 2.0f * ( YZ + XW ) );
	C3 = RkVector3( 2.0f * ( XZ + YW ), 2.0f * ( YZ - XW ), 1.0f - 2.0f * ( XX + YY ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3::RkMatrix3( const float* M )
	: C1( M + 0 )
	, C2( M + 3 )
	, C3( M + 6 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3::RkMatrix3( const double* M )
	: C1( M + 0 )
	, C2( M + 3 )
	, C3( M + 6 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3::operator float*( )
	{
	return &A11;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3::operator const float*( ) const
	{
	return &A11;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3& RkMatrix3::operator*=( const RkMatrix3& M )
	{
	*this = *this * M;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3& RkMatrix3::operator+=( const RkMatrix3& M )
	{
	C1 += M.C1;
	C2 += M.C2;
	C3 += M.C3;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3& RkMatrix3::operator-=( const RkMatrix3& M )
	{
	C1 -= M.C1;
	C2 -= M.C2;
	C3 -= M.C3;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3& RkMatrix3::operator*=( float F )
	{
	C1 *= F;
	C2 *= F;
	C3 *= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3& RkMatrix3::operator/=( float F )
	{
	C1 /= F;
	C2 /= F;
	C3 /= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 RkMatrix3::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 RkMatrix3::operator-() const
	{
	RkMatrix3 Out;
	Out.C1 = -C1;
	Out.C2 = -C2;
	Out.C3 = -C3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 operator*( const RkMatrix3& M1, const RkMatrix3& M2 )
	{
	RkMatrix3 Out;
	Out.C1 = M1 * M2.C1;
	Out.C2 = M1 * M2.C2;
	Out.C3 = M1 * M2.C3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 operator+( const RkMatrix3& M1, const RkMatrix3& M2 )
	{
	RkMatrix3 Out;
	Out.C1 = M1.C1 + M2.C1;
	Out.C2 = M1.C2 + M2.C2;
	Out.C3 = M1.C3 + M2.C3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 operator-( const RkMatrix3& M1, const RkMatrix3& M2 )
	{
	RkMatrix3 Out;
	Out.C1 = M1.C1 - M2.C1;
	Out.C2 = M1.C2 - M2.C2;
	Out.C3 = M1.C3 - M2.C3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 operator*( float F, const RkMatrix3& M )
	{
	RkMatrix3 Out;
	Out.C1 = F * M.C1;
	Out.C2 = F * M.C2;
	Out.C3 = F * M.C3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 operator*( const RkMatrix3& M, float F )
	{
	RkMatrix3 Out;
	Out.C1 = M.C1 * F;
	Out.C2 = M.C2 * F;
	Out.C3 = M.C3 * F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 operator/( const RkMatrix3& M, float F )
	{
	RkMatrix3 Out;
	Out.C1 = M.C1 / F;
	Out.C2 = M.C2 / F;
	Out.C3 = M.C3 / F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkMatrix3& M1, const RkMatrix3& M2 )
	{
	return M1.C1 == M2.C1 && M1.C2 == M2.C2 && M1.C3 == M2.C3;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkMatrix3& M1, const RkMatrix3& M2 )
	{
	return M1.C1 != M2.C1 || M1.C2 != M2.C2 || M1.C3 != M2.C3;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 rkMul( const RkMatrix3& M1, const RkMatrix3& M2 )
	{
	RkMatrix3 Out;
	Out.C1 = rkMul( M1, M2.C1 );
	Out.C2 = rkMul( M1, M2.C2 );
	Out.C3 = rkMul( M1, M2.C3 );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 rkTMul( const RkMatrix3& M1, const RkMatrix3& M2 )
	{
	RkMatrix3 Out;
	Out.C1 = rkTMul( M1, M2.C1 );
	Out.C2 = rkTMul( M1, M2.C2 );
	Out.C3 = rkTMul( M1, M2.C3 );

	return Out;
	}

//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 rkTranspose( const RkMatrix3& M )
	{
	RkMatrix3 Out;
	Out.C1 = RkVector3( M.C1.X, M.C2.X, M.C3.X );
	Out.C2 = RkVector3( M.C1.Y, M.C2.Y, M.C3.Y );
	Out.C3 = RkVector3( M.C1.Z, M.C2.Z, M.C3.Z );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 rkInvert( const RkMatrix3& M )
	{
	float Det = rkDet( M );
	if ( rkAbs( Det ) > 1000.0f * RK_F32_MIN )
		{
		RkMatrix3 Out;
		Out.C1 = rkCross( M.C2, M.C3 ) / Det;
		Out.C2 = rkCross( M.C3, M.C1 ) / Det;
		Out.C3 = rkCross( M.C1, M.C2 ) / Det;

		return rkTranspose( Out );
		}

	return RK_MAT3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 rkInvertT( const RkMatrix3& M )
	{
	float Det = rkDet( M );
	if ( rkAbs( Det ) > 1000.0f * RK_F32_MIN )
		{
		RkMatrix3 Out;
		Out.C1 = rkCross( M.C2, M.C3 ) / Det;
		Out.C2 = rkCross( M.C3, M.C1 ) / Det;
		Out.C3 = rkCross( M.C1, M.C2 ) / Det;

		return Out;
		}

	return RK_MAT3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 rkSkew( const RkVector3& V )
	{
	RkMatrix3 Out;
	Out.C1 = RkVector3( 0, V.Z, -V.Y );
	Out.C2 = RkVector3( -V.Z, 0, V.X );
	Out.C3 = RkVector3( V.Y, -V.X, 0 );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix3 rkAbs( const RkMatrix3& M )
	{
	RkMatrix3 Out;
	Out.C1 = rkAbs( M.C1 );
	Out.C2 = rkAbs( M.C2 );
	Out.C3 = rkAbs( M.C3 );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkTrace( const RkMatrix3& M )
	{
	return M.A11 + M.A22 + M.A33;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDet( const RkMatrix3& M )
	{
	return rkDot( M.C1, rkCross( M.C2, M.C3 ) );
	}


//--------------------------------------------------------------------------------------------------
// RkMatrix4
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4::RkMatrix4( float A11, float A22, float A33, float A44 )
	: C1(  A11, 0.0f, 0.0f, 0.0f )
	, C2( 0.0f,  A22, 0.0f, 0.0f )
	, C3( 0.0f, 0.0f,  A33, 0.0f )
	, C4( 0.0f, 0.0f, 0.0f,  A44 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4::RkMatrix4( const RkVector4& C1, const RkVector4& C2, const RkVector4& C3, const RkVector4& C4 )
	: C1( C1 )
	, C2( C2 )
	, C3( C3 )
	, C4( C4 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4::RkMatrix4( const RkVector3& Translation, const RkQuaternion& Rotation, const RkVector3& Scale )
	{
	*this = rkTranslation( Translation ) * rkRotation( Rotation ) * rkScale( Scale );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4::RkMatrix4( const float* M )
	: C1( M +  0 )
	, C2( M +  4 )
	, C3( M +  8 )
	, C4( M + 12 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4::RkMatrix4( const double* M )
	: C1( M + 0 )
	, C2( M + 4 )
	, C3( M + 8 )
	, C4( M + 12 )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4::RkMatrix4( const RkQuaternion& Q )
	{
	float XX = Q.X * Q.X;
	float YY = Q.Y * Q.Y;
	float ZZ = Q.Z * Q.Z;

	float XY = Q.X * Q.Y;
	float XZ = Q.X * Q.Z;
	float XW = Q.X * Q.W;
	float YZ = Q.Y * Q.Z;
	float YW = Q.Y * Q.W;
	float ZW = Q.Z * Q.W;

	C1 = RkVector4( 1.0f - 2.0f * ( YY + ZZ ), 2.0f * ( XY + ZW ), 2.0f * ( XZ - YW ), 0.0f );
	C2 = RkVector4( 2.0f * ( XY - ZW ), 1.0f - 2.0f * ( XX + ZZ ), 2.0f * ( YZ + XW ), 0.0f );
	C3 = RkVector4( 2.0f * ( XZ + YW ), 2.0f * ( YZ - XW ), 1.0f - 2.0f * ( XX + YY ), 0.0f );
	C4 = RkVector4( 0.0f, 0.0f, 0.0f, 1.0f );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4::RkMatrix4( const RkTransform& T )
	{
	RkQuaternion Q = T.Rotation;
	RkVector3 V = T.Translation;

	float XX = Q.X * Q.X;
	float YY = Q.Y * Q.Y;
	float ZZ = Q.Z * Q.Z;

	float XY = Q.X * Q.Y;
	float XZ = Q.X * Q.Z;
	float XW = Q.X * Q.W;
	float YZ = Q.Y * Q.Z;
	float YW = Q.Y * Q.W;
	float ZW = Q.Z * Q.W;

	C1 = RkVector4( 1.0f - 2.0f * ( YY + ZZ ), 2.0f * ( XY + ZW ), 2.0f * ( XZ - YW ), 0.0f );
	C2 = RkVector4( 2.0f * ( XY - ZW ), 1.0f - 2.0f * ( XX + ZZ ), 2.0f * ( YZ + XW ), 0.0f );
	C3 = RkVector4( 2.0f * ( XZ + YW ), 2.0f * ( YZ - XW ), 1.0f - 2.0f * ( XX + YY ), 0.0f );
	C4 = RkVector4( V.X, V.Y, V.Z, 1.0f );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4::operator float*()
	{
	return &A11;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4::operator const float*() const
	{
	return &A11;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4& RkMatrix4::operator*=( const RkMatrix4& M )
	{
	*this = *this * M;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4& RkMatrix4::operator+=( const RkMatrix4& M )
	{
	C1 += M.C1;
	C2 += M.C2;
	C3 += M.C3;
	C4 += M.C4;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4& RkMatrix4::operator-=( const RkMatrix4& M )
	{
	C1 -= M.C1;
	C2 -= M.C2;
	C3 -= M.C3;
	C4 -= M.C4;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4& RkMatrix4::operator*=( float F )
	{
	C1 *= F;
	C2 *= F;
	C3 *= F;
	C4 *= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4& RkMatrix4::operator/=( float F )
	{
	C1 /= F;
	C2 /= F;
	C3 /= F;
	C4 /= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 RkMatrix4::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 RkMatrix4::operator-() const
	{
	RkMatrix4 Out;
	Out.C1 = -C1;
	Out.C2 = -C2;
	Out.C3 = -C3;
	Out.C4 = -C4;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 operator*( const RkMatrix4& M1, const RkMatrix4& M2 )
	{
	RkMatrix4 Out;
	Out.C1 = M1 * M2.C1;
	Out.C2 = M1 * M2.C2;
	Out.C3 = M1 * M2.C3;
	Out.C4 = M1 * M2.C4;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 operator+( const RkMatrix4& M1, const RkMatrix4& M2 )
	{
	RkMatrix4 Out;
	Out.C1 = M1.C1 + M2.C1;
	Out.C2 = M1.C2 + M2.C2;
	Out.C3 = M1.C3 + M2.C3;
	Out.C4 = M1.C4 + M2.C4;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 operator-( const RkMatrix4& M1, const RkMatrix4& M2 )
	{
	RkMatrix4 Out;
	Out.C1 = M1.C1 - M2.C1;
	Out.C2 = M1.C2 - M2.C2;
	Out.C3 = M1.C3 - M2.C3;
	Out.C4 = M1.C4 - M2.C4;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 operator*( float S, const RkMatrix4& M )
	{
	RkMatrix4 Out;
	Out.C1 = S * M.C1;
	Out.C2 = S * M.C2;
	Out.C3 = S * M.C3;
	Out.C4 = S * M.C4;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 operator*( const RkMatrix4& M, float S )
	{
	RkMatrix4 Out;
	Out.C1 = M.C1 * S;
	Out.C2 = M.C2 * S;
	Out.C3 = M.C3 * S;
	Out.C4 = M.C4 * S;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 operator/( const RkMatrix4& M, float S )
	{
	RkMatrix4 Out;
	Out.C1 = M.C1 / S;
	Out.C2 = M.C2 / S;
	Out.C3 = M.C3 / S;
	Out.C4 = M.C4 / S;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkMatrix4& M1, const RkMatrix4& M2 )
	{
	return M1.C1 == M2.C1 && M1.C2 == M2.C2 && M1.C3 == M2.C3 && M1.C4 == M2.C4;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkMatrix4& M1, const RkMatrix4& M2 )
	{
	return M1.C1 != M2.C1 || M1.C2 != M2.C2 || M1.C3 != M2.C3 || M1.C4 != M2.C4;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkTranspose( const RkMatrix4& M )
	{
	RkMatrix4 Out;
	Out.A11 = M.A11;  Out.A12 = M.A21;  Out.A13 = M.A31;  Out.A14 = M.A41;
	Out.A21 = M.A12;  Out.A22 = M.A22;  Out.A23 = M.A32;  Out.A24 = M.A42;
	Out.A31 = M.A13;  Out.A32 = M.A23;  Out.A33 = M.A33;  Out.A34 = M.A43;
	Out.A41 = M.A14;  Out.A42 = M.A24;  Out.A43 = M.A34;  Out.A44 = M.A44;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkInvert( const RkMatrix4& M )
	{
	RkMatrix4 Out;
	rkInvert( Out, RkMatrix4( M ), 4, 4 );

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkInvertT( const RkMatrix4& M )
	{
	RkMatrix4 Out;
	rkInvert( Out, RkMatrix4( M ), 4, 4 );

	return rkTranspose( Out );
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkScale( float S )
	{
	return rkScale( RkVector3( S, S, S ) );
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkScale( const RkVector3& S )
	{
	RkMatrix4 Out;
	Out.A11 =  S.X;  Out.A12 = 0.0f;  Out.A13 = 0.0f;  Out.A14 = 0.0f;
	Out.A21 = 0.0f;  Out.A22 =  S.Y;  Out.A23 = 0.0f;  Out.A24 = 0.0f;
	Out.A31 = 0.0f;  Out.A32 = 0.0f;  Out.A33 =  S.Z;  Out.A34 = 0.0f;
	Out.A41 = 0.0f;  Out.A42 = 0.0f;  Out.A43 = 0.0f;  Out.A44 = 1.0f;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkRotation( const RkQuaternion& Q )
	{
	float XX = Q.X * Q.X;
	float YY = Q.Y * Q.Y;
	float ZZ = Q.Z * Q.Z;

	float XY = Q.X * Q.Y;
	float XZ = Q.X * Q.Z;
	float XW = Q.X * Q.W;
	float YZ = Q.Y * Q.Z;
	float YW = Q.Y * Q.W;
	float ZW = Q.Z * Q.W;

	RkMatrix4 Out;
	Out.C1 = RkVector4( 1.0f - 2.0f * ( YY + ZZ ), 2.0f * ( XY + ZW ), 2.0f * ( XZ - YW ), 0.0f );
	Out.C2 = RkVector4( 2.0f * ( XY - ZW ), 1.0f - 2.0f * ( XX + ZZ ), 2.0f * ( YZ + XW ), 0.0f );
	Out.C3 = RkVector4( 2.0f * ( XZ + YW ), 2.0f * ( YZ - XW ), 1.0f - 2.0f * ( XX + YY ), 0.0f );
	Out.C4 = RkVector4( 0.0f, 0.0f, 0.0f, 1.0f );

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkRotationX( float Rad )
	{
	float Sin = rkSin( Rad );
	float Cos = rkCos( Rad );

	RkMatrix4 Out;
	Out.A11 = 1.0f;  Out.A12 = 0.0f;  Out.A13 = 0.0f;  Out.A14 = 0.0f;
	Out.A21 = 0.0f;  Out.A22 =  Cos;  Out.A23 = -Sin;  Out.A24 = 0.0f;
	Out.A31 = 0.0f;  Out.A32 =  Sin;  Out.A33 =  Cos;  Out.A34 = 0.0f;
	Out.A41 = 0.0f;  Out.A42 = 0.0f;  Out.A43 = 0.0f;  Out.A44 = 1.0f;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkRotationY( float Rad )
	{
	float Sin = rkSin( Rad );
	float Cos = rkCos( Rad );

	RkMatrix4 Out;
	Out.A11 =  Cos;  Out.A12 = 0.0f;  Out.A13 =  Sin;  Out.A14 = 0.0f;
	Out.A21 = 0.0f;  Out.A22 = 1.0f;  Out.A23 = 0.0f;  Out.A24 = 0.0f;
	Out.A31 = -Sin;  Out.A32 = 0.0f;  Out.A33 =  Cos;  Out.A34 = 0.0f;
	Out.A41 = 0.0f;  Out.A42 = 0.0f;  Out.A43 = 0.0f;  Out.A44 = 1.0f;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkRotationZ( float Rad )
	{
	float Sin = rkSin( Rad );
	float Cos = rkCos( Rad );

	RkMatrix4 Out;
	Out.A11 =  Cos;  Out.A12 = -Sin;  Out.A13 = 0.0f;  Out.A14 = 0.0f;
	Out.A21 =  Sin;  Out.A22 =  Cos;  Out.A23 = 0.0f;  Out.A24 = 0.0f;
	Out.A31 = 0.0f;  Out.A32 = 0.0f;  Out.A33 = 1.0f;  Out.A34 = 0.0f;
	Out.A41 = 0.0f;  Out.A42 = 0.0f;  Out.A43 = 0.0f;  Out.A44 = 1.0f;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkTranslation( const RkVector3& T )
	{
	RkMatrix4 Out;
	Out.A11 = 1.0f;  Out.A12 = 0.0f;  Out.A13 = 0.0f;  Out.A14 = T.X;
	Out.A21 = 0.0f;  Out.A22 = 1.0f;  Out.A23 = 0.0f;  Out.A24 = T.Y;
	Out.A31 = 0.0f;  Out.A32 = 0.0f;  Out.A33 = 1.0f;  Out.A34 = T.Z;
	Out.A41 = 0.0f;  Out.A42 = 0.0f;  Out.A43 = 0.0f;  Out.A44 = 1.0f;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool rkIsAffine( const RkMatrix4& M )
	{
	return M.A41 == 0.0f && M.A42 == 0.0f && M.A43 == 0.0f && M.A44 == 1.0f;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkInvertAffine( const RkMatrix4& M )
	{
	//     [ R | t ]
	// M = [ --+-- ]    
	//     [ 0 | 1 ]    
	//
	//  y = M * x  ->  y = R * x + t  ->  x = R^-1 * (y - t)  ->  x = R^-1 * y - R^-1 * t
	// 
	//         [ R | t ]-1   [ R^-1 | -R^-1 * t ]
	//  M^-1 = [ --+-- ]   = [ -----+---------- ]
	//         [ 0 | 1 ]     [  0   |     1     ]
	RK_ASSERT( rkIsAffine( M ) );
	
	RkMatrix3 R;
	R.A11 = M.A11; R.A12 = M.A12; R.A13 = M.A13;
	R.A21 = M.A21; R.A22 = M.A22; R.A23 = M.A23;
	R.A31 = M.A31; R.A32 = M.A32; R.A33 = M.A33;
	RkMatrix3 InvR = rkInvert( R );
	
	RkVector3 T( M.A14, M.A24, M.A34 );
	RkVector3 InvR_x_T = InvR * T;

	RkMatrix4 Out;
	Out.A11 = InvR.A11;  Out.A12 = InvR.A12;  Out.A13 = InvR.A13;  Out.A14 = -InvR_x_T.X;
	Out.A21 = InvR.A21;  Out.A22 = InvR.A22;  Out.A23 = InvR.A23;  Out.A24 = -InvR_x_T.Y;
	Out.A31 = InvR.A31;  Out.A32 = InvR.A32;  Out.A33 = InvR.A33;  Out.A34 = -InvR_x_T.Z;
	Out.A41 = 0.0f;      Out.A42 = 0.0f;      Out.A43 = 0.0f;      Out.A44 = 1.0f;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkLookAt( const RkVector3& Eye, const RkVector3& Center, const RkVector3& Up )
	{
	// Build view transform
	RkVector3 AxisZ = rkNormalize( Center - Eye );
	RkVector3 AxisX = rkNormalize( rkCross( AxisZ, Up ) );
	RkVector3 AxisY = rkCross( AxisX, AxisZ );

	// Negate  
	AxisZ = -AxisZ;

	RkMatrix4 Out;
	Out.A11 = AxisX.X;  Out.A12 = AxisX.Y;  Out.A13 = AxisX.Z;  Out.A14 = -rkDot( AxisX, Eye );
	Out.A21 = AxisY.X;  Out.A22 = AxisY.Y;  Out.A23 = AxisY.Z;  Out.A24 = -rkDot( AxisY, Eye );
	Out.A31 = AxisZ.X;  Out.A32 = AxisZ.Y;  Out.A33 = AxisZ.Z;  Out.A34 = -rkDot( AxisZ, Eye );
	Out.A41 =    0.0f;  Out.A42 =    0.0f;  Out.A43 =    0.0f;  Out.A44 =                1.0f;
	
	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkPerspective( float FieldOfView, float AspectRatio, float NearPlane, float FarPlane )
	{
	RK_ASSERT( 0.0f < FieldOfView && FieldOfView < RK_PI_OVER_TWO );

	float Cot = 1.0f / rkTan( 0.5f * FieldOfView );
	float Depth = FarPlane - NearPlane;

	RkMatrix4 Out;
	Out.A11 = Cot / AspectRatio;  Out.A12 = 0.0f;  Out.A13 = 0.0f;                               Out.A14 = 0.0f;
	Out.A21 = 0.0f;               Out.A22 = Cot;   Out.A23 = 0.0f;                               Out.A24 = 0.0f;
	Out.A31 = 0.0f;               Out.A32 = 0.0f;  Out.A33 = -( NearPlane + FarPlane ) / Depth;  Out.A34 = -2.0f * NearPlane * FarPlane / Depth;
	Out.A41 = 0.0f;               Out.A42 = 0.0f;  Out.A43 = -1.0f;                              Out.A44 = 0.0f;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkMatrix4 rkOrtho( float Left, float Right, float Bottom, float Top, float Near, float Far )
	{
	RkMatrix4 Out;
	Out.A11 = 2.0f / ( Right - Left );	Out.A12 = 0.0f;						Out.A13 = 0.0f;					   Out.A14 = -( Right + Left ) / ( Right - Left );
	Out.A21 = 0.0f;						Out.A22 = 2.0f / ( Top - Bottom );	Out.A23 = 0.0f;					   Out.A24 = -( Top + Bottom ) / ( Top - Bottom );
	Out.A31 = 0.0f;						Out.A32 = 0.0f;						Out.A33 = -2.0f / ( Far - Near );  Out.A34 = -( Far + Near ) / ( Far - Near );
	Out.A41 = 0.0f;						Out.A42 = 0.0f;						Out.A43 = 0.0f;					   Out.A44 = 1.0f;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
// RkQuaternion
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion::RkQuaternion( float X, float Y, float Z, float W )
	: X( X )
	, Y( Y )
	, Z( Z )
	, W( W )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion::RkQuaternion( const RkVector3& Axis, float Rad )
	: V( rkSin( 0.5f * Rad ) * Axis )
	, S( rkCos( 0.5f * Rad ) )
	{
	
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion::RkQuaternion( const float* Q )
	: X( Q[ 0 ] )
	, Y( Q[ 1 ] )
	, Z( Q[ 2 ] )
	, W( Q[ 3 ] )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion::RkQuaternion( const double* Q )
	: X( static_cast< float >( Q[ 0 ] )	)
	, Y( static_cast< float >( Q[ 1 ] )	)
	, Z( static_cast< float >( Q[ 2 ] )	)
	, W( static_cast< float >( Q[ 3 ] )	)
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion::RkQuaternion( const RkVector3& Euler )
	{
	// Rotation order: ZYX (Z first, X last)
	*this = RkQuaternion( RK_VEC3_AXIS_X, Euler.X ) * RkQuaternion( RK_VEC3_AXIS_Y, Euler.Y ) * RkQuaternion( RK_VEC3_AXIS_Z, Euler.Z );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion::RkQuaternion( const RkMatrix3& M )
	{
	RkVector3 C1 = M.C1;
	RkVector3 C2 = M.C2;
	RkVector3 C3 = M.C3;

	float Trace = rkTrace( M );
	if ( Trace >= 0.0f )
		{
		X = C2.Z - C3.Y;
		Y = C3.X - C1.Z;
		Z = C1.Y - C2.X;
		W = Trace + 1.0f;
		}
	else
		{
		if ( C1.X > C2.Y && C1.X > C3.Z )
			{
			X = C1.X - C2.Y - C3.Z + 1.0f;
			Y = C2.X + C1.Y;
			Z = C3.X + C1.Z;
			W = C2.Z - C3.Y;
			}
		else if ( C2.Y > C3.Z )
			{
			X = C1.Y + C2.X;
			Y = C2.Y - C3.Z - C1.X + 1.0f;
			Z = C3.Y + C2.Z;
			W = C3.X - C1.Z;
			}
		else
			{
			X = C1.Z + C3.X;
			Y = C2.Z + C3.Y;
			Z = C3.Z - C1.X - C2.Y + 1.0f;
			W = C1.Y - C2.X;
			}
		}

	// The algorithm is simplified and made more accurate by normalizing at the end
	*this = rkNormalize( *this );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion::operator float*()
	{
	return V;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion::operator const float*() const
	{
	return V;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion& RkQuaternion::operator*=( const RkQuaternion& Q )
	{
	*this = rkMul( *this, Q );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion& RkQuaternion::operator+=( const RkQuaternion& Q )
	{
	V += Q.V;
	S += Q.S;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion& RkQuaternion::operator-=( const RkQuaternion& Q )
	{
	V -= Q.V;
	S -= Q.S;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion& RkQuaternion::operator*=( float F )
	{
	V *= F;
	S *= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion& RkQuaternion::operator/=( float F )
	{
	V /= F;
	S /= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion RkQuaternion::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion RkQuaternion::operator-() const
	{
	RkQuaternion Out;
	Out.V = -V;
	Out.S = -S;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion operator*( const RkQuaternion& Q1, const RkQuaternion& Q2 )
	{
	RkQuaternion Out;
	Out.V = rkCross( Q1.V, Q2.V ) + Q2.V * Q1.S + Q1.V * Q2.S;
	Out.S = Q1.S * Q2.S - rkDot( Q1.V, Q2.V );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion operator+( const RkQuaternion& Q1, const RkQuaternion& Q2 )
	{
	RkQuaternion Out;
	Out.V = Q1.V + Q2.V;
	Out.S = Q1.S + Q2.S;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion operator-( const RkQuaternion& Q1, const RkQuaternion& Q2 )
	{
	RkQuaternion Out;
	Out.V = Q1.V - Q2.V;
	Out.S = Q1.S - Q2.S;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion operator*( float F, const RkQuaternion& Q )
	{
	RkQuaternion Out;
	Out.V = F * Q.V;
	Out.S = F * Q.S;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion operator*( const RkQuaternion& Q, float F )
	{
	RkQuaternion Out;
	Out.V = Q.V * F;
	Out.S = Q.S * F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion operator/( const RkQuaternion& Q, float F )
	{
	RkQuaternion Out;
	Out.V = Q.V / F;
	Out.S = Q.S / F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkQuaternion& Q1, const RkQuaternion& Q2 )
	{
	return Q1.V == Q2.V && Q1.S == Q2.S;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkQuaternion& Q1, const RkQuaternion& Q2 )
	{
	return Q1.V != Q2.V || Q1.S != Q2.S;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion rkMul( const RkQuaternion& Q1, const RkQuaternion& Q2 )
	{
	RkQuaternion Out;
	Out.V = rkCross( Q1.V, Q2.V ) + Q2.V * Q1.S + Q1.V * Q2.S;
	Out.S = Q1.S * Q2.S - rkDot( Q1.V, Q2.V );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion rkCMul( const RkQuaternion& Q1, const RkQuaternion& Q2 )
	{
	RkQuaternion Out;
	Out.V = rkCross( Q2.V, Q1.V ) + Q2.V * Q1.S - Q1.V * Q2.S; 
	Out.S = Q1.S * Q2.S + rkDot( Q1.V, Q2.V ); 

	return Out;	
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion rkConjugate( const RkQuaternion& Q )
	{
	RkQuaternion Out;
	Out.V = -Q.V ;
	Out.S = Q.S;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion rkInvert( const RkQuaternion& Q )
	{
	float LengthSq = rkDot( Q, Q );
	if ( LengthSq * LengthSq > 1000.0f * RK_F32_MIN )
		{
		RkQuaternion Out;
		Out.V = -Q.V / LengthSq;
		Out.S = Q.S / LengthSq;

		return Out;
		}

	return RK_QUAT_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion rkNormalize( const RkQuaternion& Q )
	{
	float LengthSq = rkLengthSq( Q );
	if ( LengthSq > 1000.0f * RK_F32_MIN )
		{
		return Q / rkSqrt( LengthSq );
		}

	return RK_QUAT_ZERO;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion rkExp( const RkVector3& V )
	{
	// Exponential map (Grassia)
	const float kThreshold = 0.018581361f;

	float Angle = rkLength( V );
	if ( Angle < kThreshold )
		{
		// Taylor expansion
		RkQuaternion Out;
		Out.V = ( 0.5f + Angle * Angle / 48.0f ) * V;
		Out.S = rkCos( 0.5f * Angle );

		return Out;
		}
	else
		{
		return RkQuaternion( V / Angle, Angle );
		}
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion rkShortestArc( const RkVector3& V1, const RkVector3& V2 )
	{
	RK_ASSERT( rkAbs( 1.0f - rkLength( V1 ) ) < 100.0f * RK_F32_EPSILON );
	RK_ASSERT( rkAbs( 1.0f - rkLength( V2 ) ) < 100.0f * RK_F32_EPSILON );

	RkQuaternion Out;
	
	RkVector3 M = 0.5f * ( V1 + V2 );
	if ( rkLengthSq( M ) > 1000.0f * RK_F32_MIN )
		{
		Out.V = rkCross( V1, M );
		Out.S = rkDot( V1, M );
		}
	else
		{
		// Anti-parallel: Use a perpendicular vector
		if ( rkAbs( V1.X ) > 0.5f )
			{
			Out.X = V1.Y;
			Out.Y = -V1.X;
			Out.Z = 0.0f;
			}
		else
			{
			Out.X = 0.0f;
			Out.Y = V1.Z;
			Out.Z = -V1.Y;
			}

		Out.W = 0.0f;
		}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return rkNormalize( Out );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDot( const RkQuaternion& Q1, const RkQuaternion& Q2 )
	{
	return Q1.X * Q2.X + Q1.Y * Q2.Y + Q1.Z * Q2.Z + Q1.W * Q2.W;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkLength( const RkQuaternion& Q )
	{
	return rkSqrt( Q.X * Q.X + Q.Y * Q.Y + Q.Z * Q.Z + Q.W * Q.W );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkLengthSq( const RkQuaternion& Q )
	{
	return Q.X * Q.X + Q.Y * Q.Y + Q.Z * Q.Z + Q.W * Q.W;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkQuaternion rkNLerp( RkQuaternion Q1, RkQuaternion Q2, float Alpha )
	{
	RK_ASSERT( rkAbs( 1.0f - rkLength( Q1 ) ) < 100.0f * RK_F32_EPSILON );
	RK_ASSERT( rkAbs( 1.0f - rkLength( Q2 ) ) < 100.0f * RK_F32_EPSILON );
	RK_ASSERT( 0.0f <= Alpha && Alpha <= 1.0f );

	if ( rkDot( Q1, Q2 ) < 0.0f )
		{
		Q2 = -Q2;
		}

	RkQuaternion Out;
	Out.X = ( 1.0f - Alpha ) * Q1.X + Alpha * Q2.X;
	Out.Y = ( 1.0f - Alpha ) * Q1.Y + Alpha * Q2.Y;
	Out.Z = ( 1.0f - Alpha ) * Q1.Z + Alpha * Q2.Z;
	Out.W = ( 1.0f - Alpha ) * Q1.W + Alpha * Q2.W;

	return rkNormalize( Out );
	}


//--------------------------------------------------------------------------------------------------
// RkTransform
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkTransform::RkTransform( const RkVector3& Translation, const RkQuaternion& Rotation )
	: Translation( Translation )
	, Rotation( Rotation )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkTransform operator*( const RkTransform& T1, const RkTransform& T2 )
	{
	RkTransform Out;
	Out.Translation = T1 * T2.Translation;
	Out.Rotation = T1.Rotation * T2.Rotation;
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkTransform& T1, const RkTransform& T2 )
	{
	return T1.Translation == T2.Translation && T1.Rotation == T2.Rotation;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkTransform& T1, const RkTransform& T2 )
	{
	return T1.Translation != T2.Translation || T1.Rotation != T2.Rotation;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkTransform rkMul( const RkTransform& T1, const RkTransform& T2 )
	{
	RkTransform Out;
	Out.Translation = rkMul( T1, T2.Translation );
	Out.Rotation = rkMul( T1.Rotation, T2.Rotation );
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkTransform rkTMul( const RkTransform& T1, const RkTransform& T2 )
	{
	RkTransform Out;
	Out.Translation = rkTMul( T1.Rotation, T2.Translation - T1.Translation );
	Out.Rotation = rkCMul( T1.Rotation, T2.Rotation );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkTransform rkInvert( const RkTransform& T )
	{
	RkTransform Out;
	Out.Translation = rkTMul( T.Rotation, -T.Translation );
	Out.Rotation = rkConjugate( T.Rotation );
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkTransform rkLerp( const RkTransform& T1, const RkTransform& T2, float Alpha )
	{
	RkTransform Out;
	Out.Translation = rkLerp( T1.Translation, T2.Translation, Alpha );
	Out.Rotation = rkNLerp( T1.Rotation, T2.Rotation, Alpha );
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkTransform rkNormalize( const RkTransform& T )
	{
	RkTransform Out;
	Out.Translation = T.Translation;
	Out.Rotation = rkNormalize( T.Rotation );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
// RkSweep
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkSweep::RkSweep( const RkVector3 & LocalCenter, const RkTransform & Transform0, const RkTransform & Transform )
	: Alpha0( 0.0f )
	, LocalCenter( LocalCenter )
	, Center0( Transform0* LocalCenter )
	, Center( Transform* LocalCenter )
	, Orientation0( RkQuaternion( Transform0.Rotation ) )
	, Orientation( RkQuaternion( Transform.Rotation ) )
	{
	Normalize();
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkSweep::RkSweep( const RkVector3& LocalCenter, const RkVector3& Center0, const RkVector3& Center, const RkQuaternion& Orientation0, const RkQuaternion& Orientation )
	: Alpha0( 0.0f )
	, LocalCenter( LocalCenter )
	, Center0( Orientation0* LocalCenter + Center0 )
	, Center( Orientation* LocalCenter + Center )
	, Orientation0( Orientation0 )
	, Orientation( Orientation )
	{
	Normalize();
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void RkSweep::Normalize()
	{
	// Check polarity
	if ( rkDot( Orientation0, Orientation ) < 0.0f )
		{
		Orientation = -Orientation;
		}
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool RkSweep::IsNormalized() const
	{
	// Validate polarity
	return rkDot( Orientation0, Orientation ) >= 0.0f;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void RkSweep::Advance( float Alpha )
	{
	RK_ASSERT( 0.0f <= Alpha0 && Alpha0 < 1.0f );
	float Beta = (Alpha - Alpha0) / (1.0f - Alpha0);

	Alpha0 = Alpha;
	Center0 = rkLerp( Center0, Center, Beta );
	Orientation0 = rkNLerp( Orientation0, Orientation, Beta );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkTransform RkSweep::Interpolate( float Beta ) const
	{
	RkTransform Out;
	Out.Rotation = rkNLerp( Orientation0, Orientation, Beta );
	Out.Translation = rkLerp( Center0, Center, Beta ) - Out.Rotation * LocalCenter;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkSweep& Sweep1, const RkSweep& Sweep2 )
	{
	return Sweep1.Alpha0 == Sweep2.Alpha0 && Sweep1.Center0 == Sweep2.Center0 && Sweep1.Center == Sweep2.Center && Sweep1.Orientation0 == Sweep2.Orientation0 && Sweep1.Orientation == Sweep2.Orientation;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkSweep& Sweep1, const RkSweep& Sweep2 )
	{
	return Sweep1.Alpha0 != Sweep2.Alpha0 || Sweep1.Center0 != Sweep2.Center0 || Sweep1.Center != Sweep2.Center || Sweep1.Orientation0 != Sweep2.Orientation0 || Sweep1.Orientation != Sweep2.Orientation;
	}


//--------------------------------------------------------------------------------------------------
// RkPlane2
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane2::RkPlane2( const RkVector2& Normal, float Offset )
	: Normal( Normal )
	, Offset( Offset )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane2::RkPlane2( const RkVector2& Normal, const RkVector2& Point )
	: Normal( Normal )
	, Offset( rkDot( Normal, Point ) )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane2 RkPlane2::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane2 RkPlane2::operator-() const
	{
	return RkPlane2( -Normal, -Offset );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane2 rkNormalize( const RkPlane2& Plane )
	{
	float Length = rkLength( Plane.Normal );
	return RkPlane2( Plane.Normal / Length, Plane.Offset / Length );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDistance( const RkPlane2& Plane, const RkVector2& Point )
	{
	return rkDot( Plane.Normal, Point ) - Plane.Offset;
	}


//--------------------------------------------------------------------------------------------------
// RkPlane3
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane3::RkPlane3( const RkVector3& Normal, float Offset )
	: Normal( Normal )
	, Offset( Offset )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane3::RkPlane3( const RkVector3& Normal, const RkVector3& Point )
	: Normal( Normal )
	, Offset( rkDot( Normal, Point ) )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane3::RkPlane3( const RkVector3& Point1, const RkVector3& Point2, const RkVector3& Point3 )
	{
	Normal = rkCross( Point2 - Point1, Point3 - Point1 );
	Offset = rkDot( Normal, Point1 );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane3 RkPlane3::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane3 RkPlane3::operator-() const
	{
	return RkPlane3( -Normal, -Offset );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane3 operator*( const RkTransform& Transform, const RkPlane3& Plane )
	{
	RkVector3 Normal = Transform.Rotation * Plane.Normal;
	return RkPlane3( Normal, Plane.Offset + rkDot( Normal, Transform.Translation ) );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane3 operator*( float Scale, const RkPlane3& Plane )
	{
	return RkPlane3( Plane.Normal, Scale * Plane.Offset );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane3 operator*( const RkPlane3& Plane, float Scale )
	{
	return RkPlane3( Plane.Normal, Plane.Offset * Scale );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkPlane3& Plane1, const RkPlane3& Plane2 )
	{
	return Plane1.Normal == Plane2.Normal && Plane1.Offset == Plane2.Offset;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkPlane3& Plane1, const RkPlane3& Plane2 )
	{
	return Plane1.Normal != Plane2.Normal || Plane1.Offset != Plane2.Offset;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkPlane3 rkNormalize( const RkPlane3& Plane )
	{
	float Length = rkLength( Plane.Normal );
	return RkPlane3( Plane.Normal / Length, Plane.Offset / Length );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkDistance( const RkPlane3& Plane, const RkVector3& Point )
	{
	return rkDot( Plane.Normal, Point ) - Plane.Offset;
	}


//--------------------------------------------------------------------------------------------------
// RkBounds2
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds2::RkBounds2( const RkVector2& Min, const RkVector2& Max )
	: Min( Min )
	, Max( Max )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds2& RkBounds2::operator+=( const RkVector2& Point )
	{
	Min = rkMin( Min, Point );
	Max = rkMax( Max, Point );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds2& RkBounds2::operator+=( const RkBounds2& Bounds )
	{
	Min = rkMin( Min, Bounds.Min );
	Max = rkMax( Max, Bounds.Max );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 RkBounds2::GetCenter() const
	{
	return 0.5f * ( Max + Min );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector2 RkBounds2::GetExtent() const
	{
	return 0.5f * ( Max - Min );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool RkBounds2::Contains( const RkVector2& Point ) const
	{
	if ( Min.X > Point.X || Point.X > Max.X ) return false;
	if ( Min.Y > Point.Y || Point.Y > Max.Y ) return false;

	return true;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool RkBounds2::Contains( const RkBounds2& Bounds ) const
	{
	if ( Min.X > Bounds.Min.X || Bounds.Max.X > Max.X ) return false;
	if ( Min.Y > Bounds.Min.Y || Bounds.Max.Y > Max.Y ) return false;

	return true;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool RkBounds2::IsValid() const
	{
	// Check for negative volume
	return Max.X >= Min.X && Max.Y >= Min.Y;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds2 operator+( const RkBounds2& Bounds1, const RkBounds2& Bounds2 )
	{
	RkBounds2 Out;
	Out.Min = rkMin( Bounds1.Min, Bounds2.Min );
	Out.Max = rkMax( Bounds1.Max, Bounds2.Max );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkBounds2& Bounds1, const RkBounds2& Bounds2 )
	{
	return Bounds1.Min == Bounds2.Min && Bounds1.Max == Bounds2.Max;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkBounds2& Bounds1, const RkBounds2& Bounds2 )
	{
	return Bounds1.Min != Bounds2.Min || Bounds1.Max != Bounds2.Max;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool rkOverlap( const RkBounds2& Bounds1, const RkBounds2& Bounds2 )
	{
	// No intersection if separated along one axis
	if ( Bounds1.Max.X < Bounds2.Min.X || Bounds1.Min.X > Bounds2.Max.X ) return false;
	if ( Bounds1.Max.Y < Bounds2.Min.Y || Bounds1.Min.Y > Bounds2.Max.Y ) return false;

	// Overlapping on all axis means bounds are intersecting
	return true;
	}


//--------------------------------------------------------------------------------------------------
// RkBounds3
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3::RkBounds3( const RkVector3& Min, const RkVector3& Max )
	: Min( Min )
	, Max( Max )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3& RkBounds3::operator+=( const RkVector3& Point )
	{
	Min = rkMin( Min, Point );
	Max = rkMax( Max, Point );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3& RkBounds3::operator+=( const RkBounds3& Bounds )
	{
	Min = rkMin( Min, Bounds.Min );
	Max = rkMax( Max, Bounds.Max );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3& RkBounds3::operator*=( const RkVector3& Scale )
	{
	Min *= Scale;
	Max *= Scale;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3& RkBounds3::operator*=( float Scale )
	{
	Min *= Scale;
	Max *= Scale;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 RkBounds3::GetCenter() const
	{
	return 0.5f * ( Max + Min );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 RkBounds3::GetExtent() const
	{
	return 0.5f * ( Max - Min );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float RkBounds3::GetPerimeter() const
	{
	// Return the surface area (perimeter)
	float WidthX = Max.X - Min.X;
	float WidthY = Max.Y - Min.Y;
	float WidthZ = Max.Z - Min.Z;

	return 2.0f * ( WidthX * WidthZ + WidthY * WidthX + WidthZ * WidthY );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float RkBounds3::GetVolume() const
	{
	RkVector3 Delta = Max - Min;
	return Delta.X * Delta.Y * Delta.Z;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool RkBounds3::Contains( const RkVector3& Point ) const
	{
	if ( Min.X > Point.X || Point.X > Max.X ) return false;
	if ( Min.Y > Point.Y || Point.Y > Max.Y ) return false;
	if ( Min.Z > Point.Z || Point.Z > Max.Z ) return false;

	return true;
	}

//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool RkBounds3::Contains( const RkBounds3& Bounds ) const
	{
	if ( Min.X > Bounds.Min.X || Bounds.Max.X > Max.X ) return false;
	if ( Min.Y > Bounds.Min.Y || Bounds.Max.Y > Max.Y ) return false;
	if ( Min.Z > Bounds.Min.Z || Bounds.Max.Z > Max.Z ) return false;

	return true;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool RkBounds3::IsValid() const
	{
	// Check for negative volume
	return Max.X >= Min.X && Max.Y >= Min.Y && Max.Z >= Min.Z;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3 operator*( const RkTransform& Transform, const RkBounds3& Bounds )
	{
	RkVector3 Center = Transform * Bounds.GetCenter();
	RkVector3 Extent = rkAbs( RkMatrix3( Transform.Rotation ) ) * Bounds.GetExtent();

	RkBounds3 Out;
	Out.Min = Center - Extent;
	Out.Max = Center + Extent;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3 operator*( const RkBounds3& Bounds, const RkVector3& Scale )
	{
	RkBounds3 Out;
	Out.Min = Bounds.Min * Scale;
	Out.Max = Bounds.Max * Scale;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3 operator*( const RkBounds3& Bounds, float Scale )
	{
	RkBounds3 Out;
	Out.Min = Bounds.Min * Scale;
	Out.Max = Bounds.Max * Scale;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3 operator*( const RkVector3& Scale, const RkBounds3& Bounds )
	{
	RkBounds3 Out;
	Out.Min = Scale * Bounds.Min;
	Out.Max = Scale * Bounds.Max;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3 operator*( float Scale, const RkBounds3& Bounds )
	{
	RkBounds3 Out;
	Out.Min = Scale * Bounds.Min;
	Out.Max = Scale * Bounds.Max;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3 operator+( const RkBounds3& Bounds1, const RkBounds3& Bounds2 )
	{
	RkBounds3 Out;
	Out.Min = rkMin( Bounds1.Min, Bounds2.Min );
	Out.Max = rkMax( Bounds1.Max, Bounds2.Max );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkBounds3& Bounds1, const RkBounds3& Bounds2 )
	{
	return Bounds1.Min == Bounds2.Min && Bounds1.Max == Bounds2.Max;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkBounds3& Bounds1, const RkBounds3& Bounds2 )
	{
	return Bounds1.Min != Bounds2.Min || Bounds1.Max != Bounds2.Max;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkPerimeter( const RkBounds3& Bounds )
	{
	// Return the surface area (perimeter)
	float WidthX = Bounds.Max.X - Bounds.Min.X;
	float WidthY = Bounds.Max.Y - Bounds.Min.Y;
	float WidthZ = Bounds.Max.Z - Bounds.Min.Z;

	return 2.0f * ( WidthX * WidthZ + WidthY * WidthX + WidthZ * WidthY );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkCenter( const RkBounds3& Bounds )
	{
	return 0.5f * ( Bounds.Max + Bounds.Min );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkVector3 rkExtent( const RkBounds3& Bounds )
	{
	return 0.5f * ( Bounds.Max - Bounds.Min );
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3 rkUnion( const RkBounds3& Bounds1, const RkBounds3& Bounds2 )
	{
	RkBounds3 Out;
	Out.Min = rkMin( Bounds1.Min, Bounds2.Min );
	Out.Max = rkMax( Bounds1.Max, Bounds2.Max );
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkBounds3 rkInflate( const RkBounds3& Bounds, float Extension )
	{
	RkVector3 Radius( Extension, Extension, Extension );

	RkBounds3 Out;
	Out.Min = Bounds.Min - Radius;
	Out.Max = Bounds.Max + Radius;
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool rkOverlap( const RkBounds3& Bounds1, const RkBounds3& Bounds2 )
	{
	// No intersection if separated along one axis
	if ( Bounds1.Max.X < Bounds2.Min.X || Bounds1.Min.X > Bounds2.Max.X ) return false;
	if ( Bounds1.Max.Y < Bounds2.Min.Y || Bounds1.Min.Y > Bounds2.Max.Y ) return false;
	if ( Bounds1.Max.Z < Bounds2.Min.Z || Bounds1.Min.Z > Bounds2.Max.Z ) return false;

	// Overlapping on all axis means bounds are intersecting
	return true;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool rkContains( const RkBounds3& Bounds1, const RkBounds3& Bounds2 )
	{
	if ( Bounds1.Min.X > Bounds2.Min.X || Bounds2.Max.X > Bounds1.Max.X ) return false;
	if ( Bounds1.Min.Y > Bounds2.Min.Y || Bounds2.Max.Y > Bounds1.Max.Y ) return false;
	if ( Bounds1.Min.Z > Bounds2.Min.Z || Bounds2.Max.Z > Bounds1.Max.Z ) return false;

	return true;
	}


//--------------------------------------------------------------------------------------------------
// RkColor
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkColor::RkColor( float R, float G, float B, float A )
	: R( R )
	, G( G )
	, B( B )
	, A( A )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkColor::operator float*()
	{
	return &R;
	}

//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkColor::operator const float*() const
	{
	return &R;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkColor& C1, const RkColor& C2 )
	{
	return C1.R == C2.R && C1.G == C2.G && C1.B == C2.B && C1.A == C2.A;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkColor& C1, const RkColor& C2 )
	{
	return C1.R != C2.R || C1.G != C2.G || C1.B != C2.B || C1.A != C2.A;
	}


//--------------------------------------------------------------------------------------------------
// RkViewport
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE RkViewport::RkViewport( int X, int Y, int Width, int Height )
	: X( X )
	, Y( Y )
	, Width( Width )
	, Height( Height )
	{

	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator==( const RkViewport& Lhs, const RkViewport& Rhs )
	{
	return Lhs.X == Rhs.X && Lhs.Y == Rhs.Y && Lhs.Width == Rhs.Width && Lhs.Height == Rhs.Height;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE bool operator!=( const RkViewport& Lhs, const RkViewport& Rhs )
	{
	return Lhs.X != Rhs.X || Lhs.Y != Rhs.Y || Lhs.Width != Rhs.Width || Lhs.Height != Rhs.Height;
	}