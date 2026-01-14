//--------------------------------------------------------------------------------------------------
// qhMath.inl
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// qhMath	
//--------------------------------------------------------------------------------------------------
inline qhReal qhSin( qhReal Rad )
	{
	return sin( Rad );
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhCos( qhReal Rad )
	{
	return cos( Rad );
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhTan( qhReal Rad )
	{
	return tan( Rad );
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhAbs( qhReal X )
	{
	return X >= 0 ? X : -X;
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhSqrt( qhReal X )
	{
	return sqrt( X );
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline T qhMin( T X, T Y )
	{
	return X < Y ? X : Y;
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline T qhMax( T X, T Y )
	{
	return X > Y ? X : Y;
	}


//--------------------------------------------------------------------------------------------------
template< typename T > inline T qhClamp( T X, T Min, T Max )
	{
	return qhMax( Min, qhMin( X, Max ) );
	}


//--------------------------------------------------------------------------------------------------
// qhVector	
//--------------------------------------------------------------------------------------------------
inline qhVector3::qhVector3( qhReal X, qhReal Y, qhReal Z )
	: X( X )  
	, Y( Y )  
	, Z( Z )  
	{

	}


//--------------------------------------------------------------------------------------------------
inline qhVector3::qhVector3( const qhReal* V )
	: X( V[ 0 ] )  
	, Y( V[ 1 ] )  
	, Z( V[ 2 ] )  
	{

	}


//--------------------------------------------------------------------------------------------------
inline qhVector3::operator qhReal*()
	{
	return &X;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3::operator const qhReal*() const
	{
	return &X;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3& qhVector3::operator*=( const qhVector3& V )
	{
	X *= V.X;
	Y *= V.Y;
	Z *= V.Z;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3& qhVector3::operator+=( const qhVector3& V )
	{
	X += V.X;
	Y += V.Y;
	Z += V.Z;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3& qhVector3::operator-=( const qhVector3& V )
	{
	X -= V.X;
	Y -= V.Y;
	Z -= V.Z;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3& qhVector3::operator*=( qhReal S )
	{
	X *= S;
	Y *= S;
	Z *= S;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3& qhVector3::operator/=( qhReal S )
	{
	X /= S;
	Y /= S;
	Z /= S;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhVector3::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhVector3::operator-() const
	{
	return qhVector3( -X, -Y, -Z );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 operator*( const qhMatrix3& M, const qhVector3& V )
	{
	return M.C1 * V.X + M.C2 * V.Y + M.C3 * V.Z;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 operator*( const qhQuaternion& Q, const qhVector3& V )
	{
	QH_ASSERT( qhAbs( 1.0f - qhLength( Q ) ) < 100.0f * QH_REAL_EPSILON );
	return V + 2 * qhCross( Q.V, qhCross( Q.V, V ) + Q.S * V );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 operator*( const qhVector3& V1, const qhVector3& V2 )
	{
	return qhVector3( V1.X * V2.X, V1.Y * V2.Y, V1.Z * V2.Z );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 operator+( const qhVector3& V1, const qhVector3& V2 )
	{
	return qhVector3( V1.X + V2.X, V1.Y + V2.Y, V1.Z + V2.Z );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 operator-( const qhVector3& V1, const qhVector3& V2 )
	{
	return qhVector3( V1.X - V2.X, V1.Y - V2.Y, V1.Z - V2.Z );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 operator*( qhReal S, const qhVector3& V )
	{
	return qhVector3( S * V.X, S * V.Y, S * V.Z );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 operator*( const qhVector3& V, qhReal S )
	{
	return qhVector3( V.X * S, V.Y * S, V.Z * S );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 operator/( const qhVector3& V, qhReal S )
	{
	return qhVector3( V.X / S, V.Y / S, V.Z / S );
	}


//--------------------------------------------------------------------------------------------------
inline bool operator==( const qhVector3& V1, const qhVector3& V2 )
	{
	return V1.X == V2.X && V1.Y == V2.Y && V1.Z == V2.Z;
	}


//--------------------------------------------------------------------------------------------------
inline bool operator!=( const qhVector3& V1, const qhVector3& V2 )
	{
	return V1.X != V2.X || V1.Y != V2.Y || V1.Z != V2.Z;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhMul( const qhMatrix3& M, const qhVector3& V )
	{
	return M.C1 * V.X + M.C2 * V.Y + M.C3 * V.Z;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhTMul( const qhMatrix3& M, const qhVector3& V )
	{
	qhVector3 Out;
	Out.X = qhDot( M.C1, V );
	Out.Y = qhDot( M.C2, V );
	Out.Z = qhDot( M.C3, V );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhMul( const qhQuaternion& Q, const qhVector3& V )
	{
	QH_ASSERT( qhAbs( qhReal( 1 ) - qhLength( Q ) ) < qhReal( 100 ) * QH_REAL_EPSILON );
	return V + qhReal( 2 ) * qhCross( Q.V, qhCross( Q.V, V ) + Q.S * V );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhTMul( const qhQuaternion& Q, const qhVector3& V )
	{
	QH_ASSERT( qhAbs( qhReal( 1 ) - qhLength( Q ) ) < qhReal( 100 ) * QH_REAL_EPSILON );
	return V - qhReal( 2 ) * qhCross( Q.V, Q.S * V - qhCross( Q.V, V ) );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhCross( const qhVector3& V1, const qhVector3& V2 )
	{
	qhReal X = V1.Y * V2.Z - V1.Z * V2.Y;
	qhReal Y = V1.Z * V2.X - V1.X * V2.Z;
	qhReal Z = V1.X * V2.Y - V1.Y * V2.X;

	return qhVector3( X, Y, Z );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhNormalize( const qhVector3& V )
	{
	qhReal Length = qhLength( V );
	if ( Length > 1000.0f * QH_REAL_MIN )
		{
		qhVector3 Out;
		Out.X = V.X / Length;
		Out.Y = V.Y / Length;
		Out.Z = V.Z / Length;
		
		return Out;
		}

	return QH_VEC3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhAbs( const qhVector3& V )
	{
	qhVector3 Out;
	Out.X = qhAbs( V.X );
	Out.Y = qhAbs( V.Y );
	Out.Z = qhAbs( V.Z );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhMin( const qhVector3& V1, const qhVector3& V2 )
	{
	qhVector3 Out;
	Out.X = qhMin( V1.X, V2.X );
	Out.Y = qhMin( V1.Y, V2.Y );
	Out.Z = qhMin( V1.Z, V2.Z );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhMax( const qhVector3& V1, const qhVector3& V2 )
	{
	qhVector3 Out;
	Out.X = qhMax( V1.X, V2.X );
	Out.Y = qhMax( V1.Y, V2.Y );
	Out.Z = qhMax( V1.Z, V2.Z );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhClamp( const qhVector3& V, const qhVector3& Min, const qhVector3& Max )
	{
	qhVector3 Out;
	Out.X = qhClamp( V.X, Min.X, Max.X );
	Out.Y = qhClamp( V.Y, Min.Y, Max.Y );
	Out.Z = qhClamp( V.Z, Min.Z, Max.Z );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhDot( const qhVector3& V1, const qhVector3& V2 )
	{
	return V1.X * V2.X + V1.Y * V2.Y + V1.Z * V2.Z;
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhLength( const qhVector3& V )
	{
	return sqrt( qhDot( V, V ) );
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhLengthSq( const qhVector3& V )
	{
	return qhDot( V, V );
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhDistance( const qhVector3& V1, const qhVector3& V2 )
	{
	return qhLength( V1 - V2 );
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhDistanceSq( const qhVector3& V1, const qhVector3& V2 )
	{
	return qhLengthSq( V1 - V2 );
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhDet( const qhVector3& V1, const qhVector3& V2, const qhVector3& V3 )
{
	return qhDot( V1, qhCross( V2, V3 ) );
}


//--------------------------------------------------------------------------------------------------
inline int qhMinElement( const qhVector3& V )
	{
	return V.X < V.Y ? ( V.X < V.Z ? 0 : 2 ) : ( V.Y < V.Z ? 1 : 2 );
	}


//--------------------------------------------------------------------------------------------------
inline int qhMaxElement( const qhVector3& V )
	{
	return V.X < V.Y ? ( V.Y < V.Z ? 2 : 1 ) : ( V.X < V.Z ? 2 : 0 );
	}


//--------------------------------------------------------------------------------------------------
// qhMatrix3
//--------------------------------------------------------------------------------------------------
inline qhMatrix3::qhMatrix3( qhReal A11, qhReal A22, qhReal A33 )
	: C1(  A11, 0.0f, 0.0f )
	, C2( 0.0f,  A22, 0.0f )
	, C3( 0.0f, 0.0f,  A33 )
	{

	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3::qhMatrix3( const qhVector3& C1, const qhVector3& C2, const qhVector3& C3 )
	: C1( C1 )
	, C2( C2 )
	, C3( C3 )
	{

	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3::qhMatrix3( const qhQuaternion& Q )
	{
	qhReal XX = Q.X * Q.X;
	qhReal YY = Q.Y * Q.Y;
	qhReal ZZ = Q.Z * Q.Z;

	qhReal XY = Q.X * Q.Y;
	qhReal XZ = Q.X * Q.Z;
	qhReal XW = Q.X * Q.W;
	qhReal YZ = Q.Y * Q.Z;
	qhReal YW = Q.Y * Q.W;
	qhReal ZW = Q.Z * Q.W;

	C1 = qhVector3( qhReal( 1 ) - qhReal( 2 ) * ( YY + ZZ ), qhReal( 2 ) * ( XY + ZW ), qhReal( 2 ) * ( XZ - YW ) );
	C2 = qhVector3( qhReal( 2 ) * ( XY - ZW ), qhReal( 1 ) - qhReal( 2 ) * ( XX + ZZ ), qhReal( 2 ) * ( YZ + XW ) );
	C3 = qhVector3( qhReal( 2 ) * ( XZ + YW ), qhReal( 2 ) * ( YZ - XW ), qhReal( 1 ) - qhReal( 2 ) * ( XX + YY ) );
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3::qhMatrix3( const qhReal* M )
	: C1( M + 0 )
	, C2( M + 3 )
	, C3( M + 6 )
	{

	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3::operator qhReal*( )
	{
	return &A11;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3::operator const qhReal*( ) const
	{
	return &A11;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3& qhMatrix3::operator*=( const qhMatrix3& M )
	{
	*this = *this * M;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3& qhMatrix3::operator+=( const qhMatrix3& M )
	{
	C1 += M.C1;
	C2 += M.C2;
	C3 += M.C3;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3& qhMatrix3::operator-=( const qhMatrix3& M )
	{
	C1 -= M.C1;
	C2 -= M.C2;
	C3 -= M.C3;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3& qhMatrix3::operator*=( qhReal F )
	{
	C1 *= F;
	C2 *= F;
	C3 *= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3& qhMatrix3::operator/=( qhReal F )
	{
	C1 /= F;
	C2 /= F;
	C3 /= F;

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 qhMatrix3::operator+() const
	{
	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 qhMatrix3::operator-() const
	{
	qhMatrix3 Out;
	Out.C1 = -C1;
	Out.C2 = -C2;
	Out.C3 = -C3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 operator*( const qhMatrix3& M1, const qhMatrix3& M2 )
	{
	qhMatrix3 Out;
	Out.C1 = M1 * M2.C1;
	Out.C2 = M1 * M2.C2;
	Out.C3 = M1 * M2.C3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 operator+( const qhMatrix3& M1, const qhMatrix3& M2 )
	{
	qhMatrix3 Out;
	Out.C1 = M1.C1 + M2.C1;
	Out.C2 = M1.C2 + M2.C2;
	Out.C3 = M1.C3 + M2.C3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 operator-( const qhMatrix3& M1, const qhMatrix3& M2 )
	{
	qhMatrix3 Out;
	Out.C1 = M1.C1 - M2.C1;
	Out.C2 = M1.C2 - M2.C2;
	Out.C3 = M1.C3 - M2.C3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 operator*( qhReal F, const qhMatrix3& M )
	{
	qhMatrix3 Out;
	Out.C1 = F * M.C1;
	Out.C2 = F * M.C2;
	Out.C3 = F * M.C3;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 operator*( const qhMatrix3& M, qhReal F )
	{
	qhMatrix3 Out;
	Out.C1 = M.C1 * F;
	Out.C2 = M.C2 * F;
	Out.C3 = M.C3 * F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 operator/( const qhMatrix3& M, qhReal F )
	{
	qhMatrix3 Out;
	Out.C1 = M.C1 / F;
	Out.C2 = M.C2 / F;
	Out.C3 = M.C3 / F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline bool operator==( const qhMatrix3& M1, const qhMatrix3& M2 )
	{
	return M1.C1 == M2.C1 && M1.C2 == M2.C2 && M1.C3 == M2.C3;
	}


//--------------------------------------------------------------------------------------------------
inline bool operator!=( const qhMatrix3& M1, const qhMatrix3& M2 )
	{
	return M1.C1 != M2.C1 || M1.C2 != M2.C2 || M1.C3 != M2.C3;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 qhMul( const qhMatrix3& M1, const qhMatrix3& M2 )
	{
	qhMatrix3 Out;
	Out.C1 = qhMul( M1, M2.C1 );
	Out.C2 = qhMul( M1, M2.C2 );
	Out.C3 = qhMul( M1, M2.C3 );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 qhTMul( const qhMatrix3& M1, const qhMatrix3& M2 )
	{
	qhMatrix3 Out;
	Out.C1 = qhTMul( M1, M2.C1 );
	Out.C2 = qhTMul( M1, M2.C2 );
	Out.C3 = qhTMul( M1, M2.C3 );

	return Out;
	}

//--------------------------------------------------------------------------------------------------
inline qhMatrix3 qhTranspose( const qhMatrix3& M )
	{
	qhMatrix3 Out;
	Out.C1 = qhVector3( M.C1.X, M.C2.X, M.C3.X );
	Out.C2 = qhVector3( M.C1.Y, M.C2.Y, M.C3.Y );
	Out.C3 = qhVector3( M.C1.Z, M.C2.Z, M.C3.Z );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 qhInvert( const qhMatrix3& M )
	{
	qhReal Det = qhDet( M );
	if ( qhAbs( Det ) > qhReal( 1000 ) * QH_REAL_MIN )
		{
		qhMatrix3 Out;
		Out.C1 = qhCross( M.C2, M.C3 ) / Det;
		Out.C2 = qhCross( M.C3, M.C1 ) / Det;
		Out.C3 = qhCross( M.C1, M.C2 ) / Det;

		return qhTranspose( Out );
		}

	return QH_MAT3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
inline qhMatrix3 qhInvertT( const qhMatrix3& M )
	{
	qhReal Det = qhDet( M );
	if ( qhAbs( Det ) > qhReal( 1000 ) * QH_REAL_MIN )
		{
		qhMatrix3 Out;
		Out.C1 = qhCross( M.C2, M.C3 ) / Det;
		Out.C2 = qhCross( M.C3, M.C1 ) / Det;
		Out.C3 = qhCross( M.C1, M.C2 ) / Det;

		return Out;
		}

	return QH_MAT3_ZERO;
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhTrace( const qhMatrix3& M )
	{
	return M.C1.X + M.C2.Y + M.C3.Z;
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhDet( const qhMatrix3& M )
	{
	return qhDot( M.C1, qhCross( M.C2, M.C3 ) );
	}


//--------------------------------------------------------------------------------------------------
// qhQuaternion
//--------------------------------------------------------------------------------------------------
inline qhQuaternion::qhQuaternion( qhReal X, qhReal Y, qhReal Z, qhReal W )
	: X( X )
	, Y( Y )
	, Z( Z )
	, W( W )
	{

	}


//--------------------------------------------------------------------------------------------------
inline qhQuaternion::qhQuaternion( const qhVector3& V, qhReal S )
	: V( V )
	, S( S )
	{

	}


//--------------------------------------------------------------------------------------------------
inline qhQuaternion::qhQuaternion( const qhReal* Q )
	: X( Q[ 0 ] )
	, Y( Q[ 1 ] )
	, Z( Q[ 2 ] )
	, W( Q[ 3 ] )
	{

	}


//--------------------------------------------------------------------------------------------------
inline qhQuaternion operator*( const qhQuaternion& Q1, const qhQuaternion& Q2 )
	{
	qhQuaternion Out;
	Out.V = qhCross( Q1.V, Q2.V ) + Q2.V * Q1.S + Q1.V * Q2.S;
	Out.S = Q1.S * Q2.S - qhDot( Q1.V, Q2.V );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhQuaternion qhRotationX( qhReal Rad )
	{
	qhReal Sin = qhSin( 0.5f * Rad );
	qhReal Cos = qhCos( 0.5f * Rad );
	
	qhQuaternion Out;
	Out.V = qhVector3( Sin, 0.0f, 0.0f );
	Out.S = Cos;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhQuaternion qhRotationY( qhReal Rad )
	{
	qhReal Sin = qhSin( 0.5f * Rad );
	qhReal Cos = qhCos( 0.5f * Rad );

	qhQuaternion Out;
	Out.V = qhVector3( 0.0f, Sin, 0.0f );
	Out.S = Cos;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhQuaternion qhRotationZ( qhReal Rad )
	{
	qhReal Sin = qhSin( 0.5f * Rad );
	qhReal Cos = qhCos( 0.5f * Rad );

	qhQuaternion Out;
	Out.V = qhVector3( 0.0f, 0.0f, Sin );
	Out.S = Cos;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhQuaternion qhShortestArc( const qhVector3& V1, const qhVector3& V2 )
	{
	qhQuaternion Out;

	qhVector3 M = 0.5f * ( V1 + V2 );
	if ( qhLengthSq( M ) > QH_REAL_EPSILON * QH_REAL_EPSILON )
		{
		Out.V = qhCross( V1, M );
		Out.S = qhDot( V1, M );
		}
	else
		{
		// Anti-parallel: Use a perpendicular vector
		if ( qhAbs( V1.X ) > 0.5f )
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
	return qhNormalize( Out );
	}


//--------------------------------------------------------------------------------------------------
inline qhQuaternion qhConjugate( const qhQuaternion& Q )
	{
	qhQuaternion Out;
	Out.V = -Q.V ;
	Out.S = Q.S;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline qhQuaternion qhNormalize( const qhQuaternion& Q )
	{
	qhReal Length = qhLength( Q );
	if ( Length > 1000.0f * QH_REAL_MIN )
		{
		qhQuaternion Out;
		Out.X = Q.X / Length;
		Out.Y = Q.Y / Length;
		Out.Z = Q.Z / Length;
		Out.W = Q.W / Length;

		return Out;
		}

	return QH_QUAT_ZERO;
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhDot( const qhQuaternion& Q1, const qhQuaternion& Q2 )
	{
	return Q1.X * Q2.X + Q1.Y * Q2.Y + Q1.Z * Q2.Z + Q1.W * Q2.W;
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhLength( const qhQuaternion& Q )
	{
	return qhSqrt( Q.X * Q.X + Q.Y * Q.Y + Q.Z * Q.Z + Q.W * Q.W );
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhLengthSq( const qhQuaternion& Q )
	{
	return Q.X * Q.X + Q.Y * Q.Y + Q.Z * Q.Z + Q.W * Q.W;
	}


//--------------------------------------------------------------------------------------------------
// qhPlane
//--------------------------------------------------------------------------------------------------
inline qhPlane::qhPlane( const qhVector3& Normal, qhReal Offset )
	: Normal( Normal )
	, Offset( Offset )
	{
	
	}


//--------------------------------------------------------------------------------------------------
inline qhPlane::qhPlane( const qhVector3& Normal, const qhVector3& Point )
	: Normal( Normal )
	, Offset( qhDot( Normal, Point ) )
	{

	}


//--------------------------------------------------------------------------------------------------
inline qhPlane::qhPlane( const qhVector3& Point1, const qhVector3& Point2, const qhVector3& Point3 )
	{
	qhVector3 Edge1 = Point2 - Point1;
	qhVector3 Edge2 = Point3 - Point1;

	Normal = qhCross( Edge1, Edge2 );
	Offset = qhDot( Normal , Point1 ); 
	}


//--------------------------------------------------------------------------------------------------
inline void qhPlane::Negate()
	{
	Normal = -Normal;
	Offset = -Offset;
	}


//--------------------------------------------------------------------------------------------------
inline void qhPlane::Normalize()
	{
	qhReal Length = qhLength( Normal );
	Normal /= Length;
	Offset /= Length;
	}


//--------------------------------------------------------------------------------------------------
inline void qhPlane::Translate( const qhVector3& Translation )
	{
	Offset += qhDot( Normal, Translation );
	}

	
//--------------------------------------------------------------------------------------------------
inline qhReal qhPlane::Distance( const qhVector3& Point ) const
	{
	return qhDot( Normal, Point ) - Offset;
	}


//--------------------------------------------------------------------------------------------------
// qhBounds3
//--------------------------------------------------------------------------------------------------
inline qhBounds3::qhBounds3( const qhVector3& Min, const qhVector3& Max )
	: Min( Min )
	, Max( Max )
	{

	}


//--------------------------------------------------------------------------------------------------
inline qhBounds3& qhBounds3::operator+=( const qhVector3& Point )
	{
	Min = qhMin( Min, Point );
	Max = qhMax( Max, Point );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhBounds3& qhBounds3::operator+=( const qhBounds3& Bounds )
	{
	Min = qhMin( Min, Bounds.Min );
	Max = qhMax( Max, Bounds.Max );

	return *this;
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhBounds3::GetCenter() const
	{
	return 0.5f * ( Max + Min );
	}


//--------------------------------------------------------------------------------------------------
inline qhVector3 qhBounds3::GetExtent() const
	{
	return 0.5f * ( Max - Min );
	}


//--------------------------------------------------------------------------------------------------
inline qhReal qhBounds3::GetVolume() const
	{
	qhVector3 Diagonal = Max - Min;
	return Diagonal.X * Diagonal.Y * Diagonal.Z;
	}


//--------------------------------------------------------------------------------------------------
inline qhBounds3 operator+( const qhBounds3& Bounds1, const qhBounds3& Bounds2 )
	{
	qhBounds3 Out;
	Out.Min = qhMin( Bounds1.Min, Bounds2.Min );
	Out.Max = qhMax( Bounds1.Max, Bounds2.Max );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
inline bool operator==( const qhBounds3& Bounds1, const qhBounds3& Bounds2 )
	{
	return Bounds1.Min == Bounds2.Min && Bounds1.Max == Bounds2.Max;
	}	


//--------------------------------------------------------------------------------------------------
inline bool operator!=( const qhBounds3& Bounds1, const qhBounds3& Bounds2 )
	{
	return Bounds1.Min != Bounds2.Min || Bounds1.Max != Bounds2.Max;
	}



//--------------------------------------------------------------------------------------------------
// qhMass	
//--------------------------------------------------------------------------------------------------
inline qhMatrix3 qhSteiner( qhReal Mass, const qhVector3& Offset )
	{
	// Usage: Io = Ic + Is and Ic = Io - Is where Is = -m * skew( d )^2
	qhReal Ixx = Mass * ( Offset.Y * Offset.Y + Offset.Z * Offset.Z );
	qhReal Iyy = Mass * ( Offset.X * Offset.X + Offset.Z * Offset.Z );
	qhReal Izz = Mass * ( Offset.X * Offset.X + Offset.Y * Offset.Y );
	qhReal Ixy = -Mass * Offset.X * Offset.Y;
	qhReal Ixz = -Mass * Offset.X * Offset.Z;
	qhReal Iyz = -Mass * Offset.Y * Offset.Z;

	// Write
	qhMatrix3 Out;
	Out.C1.X = Ixx; Out.C2.X = Ixy; Out.C3.X = Ixz;
	Out.C1.Y = Ixy;	Out.C2.Y = Iyy;	Out.C3.Y = Iyz;
	Out.C1.Z = Ixz;	Out.C2.Z = Iyz;	Out.C3.Z = Izz;

	return Out;
	}