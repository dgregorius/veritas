//--------------------------------------------------------------------------------------------------
/*
	@file		qhMath.h

	@author		Dirk Gregorius
	@version	0.1
	@date		30/11/2011

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "qhTypes.h"
#include "qhArray.h"

#include <cmath>

struct qhVector3;
struct qhMatrix3;
struct qhQuaternion;
struct qhPlane;
struct qhBounds3;


//--------------------------------------------------------------------------------------------------
// qhConstants	
//--------------------------------------------------------------------------------------------------
#define QH_PI				qhReal( 3.14159265358979323846 )
#define QH_SQRT2			qhReal( 1.41421356237309504880 )	
#define QH_SQRT3			qhReal( 1.73205080756887729353 )	
#define QH_DEG2RAD			( QH_PI / qhReal( 180 ) )
#define QH_RAD2DEG			( qhReal( 180 ) / QH_PI )


//--------------------------------------------------------------------------------------------------
// qhMath	
//--------------------------------------------------------------------------------------------------
qhReal qhSin( qhReal Rad );
qhReal qhCos( qhReal Rad );
qhReal qhTan( qhReal Rad );

qhReal qhAbs( qhReal X );
qhReal qhSqrt( qhReal X );

template< typename T > T qhMin( T X, T Y );
template< typename T > T qhMax( T X, T Y );
template< typename T > T qhClamp( T X, T Min, T Max );


//--------------------------------------------------------------------------------------------------
// qhVector	
//--------------------------------------------------------------------------------------------------
struct qhVector3
	{
	// Attributes
	qhReal X, Y, Z;

	// Construction
	qhVector3() = default;
	qhVector3( qhReal X, qhReal Y, qhReal Z );
	qhVector3( const qhReal* V );

	// Conversion
	operator qhReal*();
	operator const qhReal*() const;

	// Assignment
	qhVector3& operator*=( const qhVector3& V );
	qhVector3& operator+=( const qhVector3& V );
	qhVector3& operator-=( const qhVector3& V );
	qhVector3& operator*=( qhReal S );
	qhVector3& operator/=( qhReal S );

	// Unary operators
	qhVector3 operator+() const;
	qhVector3 operator-() const;
	};

// Binary operators
qhVector3 operator*( const qhMatrix3& M, const qhVector3& V );
qhVector3 operator*( const qhQuaternion& Q, const qhVector3& V );

qhVector3 operator*( const qhVector3& V1, const qhVector3& V2 );
qhVector3 operator+( const qhVector3& V1, const qhVector3& V2 );
qhVector3 operator-( const qhVector3& V1, const qhVector3& V2 );
qhVector3 operator*( qhReal S, const qhVector3& V );
qhVector3 operator*( const qhVector3& V, qhReal S );
qhVector3 operator/( const qhVector3& V, qhReal S );

bool operator==( const qhVector3& V1, const qhVector3& V2 );
bool operator!=( const qhVector3& V1, const qhVector3& V2 );

// Standard vector operations
qhVector3 qhMul( const qhMatrix3& M, const qhVector3& V );
qhVector3 qhTMul( const qhMatrix3& M, const qhVector3& V );
qhVector3 qhMul( const qhQuaternion& Q, const qhVector3& V );
qhVector3 qhTMul( const qhQuaternion& Q, const qhVector3& V );

qhVector3 qhCross( const qhVector3& V1, const qhVector3& V2 );
qhVector3 qhNormalize( const qhVector3& V );
qhVector3 qhAbs( const qhVector3& V );
qhVector3 qhMin( const qhVector3& V1, const qhVector3& V2 );
qhVector3 qhMax( const qhVector3& V1, const qhVector3& V2 );
qhVector3 qhClamp( const qhVector3& V, const qhVector3& Min, const qhVector3& Max );

qhReal qhDot( const qhVector3& V1, const qhVector3& V2 );
qhReal qhLength( const qhVector3& V );
qhReal qhLengthSq( const qhVector3& V );
qhReal qhDistance( const qhVector3& V1, const qhVector3& V2 );
qhReal qhDistanceSq( const qhVector3& V1, const qhVector3& V2 );
qhReal qhDet( const qhVector3& V1, const qhVector3& V2, const qhVector3& V3 );

int qhMinElement( const qhVector3& V );
int qhMaxElement( const qhVector3& V );

// Constants
QH_GLOBAL_CONSTANT const qhVector3 QH_VEC3_ZERO	= qhVector3( 0, 0, 0 ); 
QH_GLOBAL_CONSTANT const qhVector3 QH_VEC3_AXIS_X = qhVector3( 1, 0, 0 ); 
QH_GLOBAL_CONSTANT const qhVector3 QH_VEC3_AXIS_Y = qhVector3( 0, 1, 0 ); 
QH_GLOBAL_CONSTANT const qhVector3 QH_VEC3_AXIS_Z = qhVector3( 0, 0, 1 ); 


//--------------------------------------------------------------------------------------------------
// qhMatrix3
//--------------------------------------------------------------------------------------------------
struct qhMatrix3
	{
	// Attributes
	union
		{
		struct
			{
			qhReal A11, A21, A31;
			qhReal A12, A22, A32;
			qhReal A13, A23, A33;
			};

		struct
			{
			qhVector3 C1, C2, C3;
			};
		};

	// Construction
	qhMatrix3() = default;
	qhMatrix3( qhReal A11, qhReal A22, qhReal A33 );
	qhMatrix3( const qhVector3& C1, const qhVector3& C2, const qhVector3& C3 );
	explicit qhMatrix3( const qhQuaternion& Q );
	qhMatrix3( const qhReal* M );

	// Conversion
	operator qhReal*( );
	operator const qhReal*( ) const;

	// Assignment
	qhMatrix3& operator*=( const qhMatrix3& M );
	qhMatrix3& operator+=( const qhMatrix3& M );
	qhMatrix3& operator-=( const qhMatrix3& M );
	qhMatrix3& operator*=( qhReal F );
	qhMatrix3& operator/=( qhReal F );

	// Unary operators
	qhMatrix3 operator+() const;
	qhMatrix3 operator-() const;
	};

// Binary arithmetic operators
qhMatrix3 operator*( const qhMatrix3& M1, const qhMatrix3& M2 );
qhMatrix3 operator+( const qhMatrix3& M1, const qhMatrix3& M2 );
qhMatrix3 operator-( const qhMatrix3& M1, const qhMatrix3& M2 );
qhMatrix3 operator*( qhReal F, const qhMatrix3& M );
qhMatrix3 operator*( const qhMatrix3& M, qhReal F );
qhMatrix3 operator/( const qhMatrix3& M, qhReal F );

// Comparison operators
bool operator==( const qhMatrix3& M1, const qhMatrix3& M2 );
bool operator!=( const qhMatrix3& M1, const qhMatrix3& M2 );

// Standard matrix operations
qhMatrix3 qhMul( const qhMatrix3& M1, const qhMatrix3& M2 );
qhMatrix3 qhTMul( const qhMatrix3& M1, const qhMatrix3& M2 );
qhMatrix3 qhTranspose( const qhMatrix3& M );
qhMatrix3 qhInvert( const qhMatrix3& M );
qhMatrix3 qhInvertT( const qhMatrix3& M );

qhReal qhTrace( const qhMatrix3& M );
qhReal qhDet( const qhMatrix3& M );

// Constants
QH_GLOBAL_CONSTANT const qhMatrix3 QH_MAT3_ZERO = qhMatrix3( 0, 0, 0 );
QH_GLOBAL_CONSTANT const qhMatrix3 QH_MAT3_IDENTITY = qhMatrix3( 1, 1, 1 );


//--------------------------------------------------------------------------------------------------
// qhQuaternion
//--------------------------------------------------------------------------------------------------
struct qhQuaternion
	{
	// Attributes
	union
		{
		struct 
			{
			qhReal X, Y, Z, W;
			};

		struct 
			{
			qhVector3 V;
			qhReal S;
			};
		};

	// Construction
	qhQuaternion() = default;
	qhQuaternion( qhReal X, qhReal Y, qhReal Z, qhReal W );
	qhQuaternion( const qhVector3& V, qhReal S );
	qhQuaternion( const qhReal* Q );

	};

// Binary arithmetic operators
qhQuaternion operator*( const qhQuaternion& Q1, const qhQuaternion& Q2 );

// Standard quaternion operations
qhQuaternion qhRotationX( qhReal Rad );
qhQuaternion qhRotationY( qhReal Rad );
qhQuaternion qhRotationZ( qhReal Rad );
qhQuaternion qhShortestArc( const qhVector3& V1, const qhVector3& V2 );

qhQuaternion qhConjugate( const qhQuaternion& Q );
qhQuaternion qhNormalize( const qhQuaternion& Q );

qhReal qhDot( const qhQuaternion& Q1, const qhQuaternion& Q2 );
qhReal qhLength( const qhQuaternion& Q );
qhReal qhLengthSq( const qhQuaternion& Q );

// Constants
QH_GLOBAL_CONSTANT const qhQuaternion QH_QUAT_ZERO = qhQuaternion( 0, 0, 0, 0 );
QH_GLOBAL_CONSTANT const qhQuaternion QH_QUAT_IDENTITY = qhQuaternion( 0, 0, 0, 1 );


//--------------------------------------------------------------------------------------------------
// qhPlane
//--------------------------------------------------------------------------------------------------
struct qhPlane
	{
	// Attributes
	qhVector3 Normal;
	qhReal Offset;

	// Construction
	qhPlane() = default;
	qhPlane( const qhVector3& Normal, qhReal Offset );
	qhPlane( const qhVector3& Normal, const qhVector3& Point );
	qhPlane( const qhVector3& Point1, const qhVector3& Point2, const qhVector3& Point3 );

	void Negate();
	void Normalize();
	void Translate( const qhVector3& Translation );
		
	qhReal Distance( const qhVector3& Point ) const;
	};


//--------------------------------------------------------------------------------------------------
// qhBounds3
//--------------------------------------------------------------------------------------------------
struct qhBounds3
	{
	// Attributes
	qhVector3 Min, Max;

	// Construction
	qhBounds3() = default;
	qhBounds3( const qhVector3& Min, const qhVector3& Max );

	// Assignment
	qhBounds3& operator+=( const qhVector3& Point );
	qhBounds3& operator+=( const qhBounds3& Bounds );
	
	// Standard bounds operations
	qhVector3 GetCenter() const;
	qhVector3 GetExtent() const;

	qhReal GetVolume() const;
	};

// Binary arithmetic operators
qhBounds3 operator+( const qhBounds3& Bounds1, const qhBounds3& Bounds2 );

// Comparison operators
bool operator==( const qhBounds3& Bounds1, const qhBounds3& Bounds2 );
bool operator!=( const qhBounds3& Bounds1, const qhBounds3& Bounds2 );

// Standard bounds operations
qhBounds3 qhBuildBounds( qhArray< qhVector3 >& Vertices );
	
// Constants
QH_GLOBAL_CONSTANT const qhBounds3 QH_BOUNDS3_EMPTY	= qhBounds3( qhVector3( QH_REAL_MAX, QH_REAL_MAX, QH_REAL_MAX ), qhVector3( -QH_REAL_MAX, -QH_REAL_MAX, -QH_REAL_MAX ) );
QH_GLOBAL_CONSTANT const qhBounds3 QH_BOUNDS3_INFINITE = qhBounds3( qhVector3( -QH_REAL_MAX, -QH_REAL_MAX, -QH_REAL_MAX ), qhVector3( QH_REAL_MAX, QH_REAL_MAX, QH_REAL_MAX ) );


//--------------------------------------------------------------------------------------------------
// qhColor
//--------------------------------------------------------------------------------------------------
struct qhColor
	{
	float R, G, B, A;

	qhColor() = default;
	qhColor( float R, float G, float B, float A = 1.0f )
		: R( R )
		, G( G )
		, B( B )
		, A( A )
		{

		}

	// Conversion
	operator float* ( )
		{
		return &R;
		}

	operator const float* ( ) const
		{
		return &R;
		}
	};


//--------------------------------------------------------------------------------------------------
// qhMass	
//--------------------------------------------------------------------------------------------------
struct qhMassProperties
	{
	qhReal Mass;
	qhVector3 Center;
	qhMatrix3 Inertia;
	};

qhMatrix3 qhSteiner( qhReal Mass, const qhVector3& Offset );


//--------------------------------------------------------------------------------------------------
// Pre-processing
//--------------------------------------------------------------------------------------------------
qhVector3 qhComputeCentroid( const qhArray< qhVector3 >& Vertices );
void qhShiftVertices( qhArray< qhVector3 >& Vertices, const qhVector3& Translation );
void qhWeldVertices( qhArray< qhVector3 >& Vertices, const qhVector3& Tolerance = QH_VEC3_ZERO );


#include "qhMath.inl"