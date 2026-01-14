//--------------------------------------------------------------------------------------------------
/*
	@file		math.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/assert.h"
#include "ragnarok/common/types.h"

// CRT
#include <math.h>
#include <float.h>
#include <stdlib.h>

struct RkVector2;
struct RkVector3;
struct RkVector4;
struct RkMatrix2;
struct RkMatrix3;
struct RkMatrix4;
struct RkQuaternion;
struct RkTransform;
struct RkSweep;
struct RkPlane2;
struct RkPlane3;
struct RkBounds2;
struct RkBounds3;
struct RkColor;
struct RkViewport;


//--------------------------------------------------------------------------------------------------
// Constants
//--------------------------------------------------------------------------------------------------
#define RK_PI				3.141592654f
#define RK_2PI				6.283185307f
#define RK_ONE_OVER_PI		0.318309886f
#define RK_ONE_OVER_2PI		0.159154943f
#define RK_PI_OVER_TWO		1.570796327f
#define RK_PI_OVER_FOUR		0.785398163f
#define RK_SQRT2			1.414213562f
#define RK_ONE_OVER_SQRT2	0.707106781f
#define RK_SQRT3			1.732050808f
#define RK_ONE_OVER_SQRT3	0.577350269f

#define RK_DEG2RAD			0.01745329251f
#define RK_RAD2DEG			57.2957795131f


//--------------------------------------------------------------------------------------------------
// Math
//--------------------------------------------------------------------------------------------------
float rkSign( float X );
float rkFloor( float X );
float rkCeil( float X );
float rkRound( float X );
float rkPow( float X, float Y );
float rkExp( float X );
float rkLog( float X );
float rkSqrt( float X );
float rkSquare( float X );
float rkCube( float X );
float rkMod( float X, float Y );

float rkSin( float Rad );
float rkCos( float Rad );
float rkTan( float Rad );
float rkASin( float X );
float rkACos( float X );
float rkATan( float X );
float rkATan2( float Y, float X );

template< typename T > T rkAbs( T X );
template< typename T > T rkMin( T X, T Y );
template< typename T > T rkMax( T X, T Y );
template< typename T > T rkClamp( T X, T Min, T Max );
template< typename T > T rkNormalize( T X, T Min, T Max );

bool rkFuzzyZero( float X, float Epsilon = RK_F32_EPSILON );
bool rkFuzzyEqual( float X, float Y, float Epsilon = RK_F32_EPSILON );

float rkBias( float T, float Bias );
float rkGain( float T, float Gain );
float rkLerp( float X1, float X2, float Alpha );

int rkRandom( int Min, int Max );
float rkRandom( float Min, float Max );


//--------------------------------------------------------------------------------------------------
// RkVector2
//--------------------------------------------------------------------------------------------------
struct RkVector2
	{
	// Attributes
	float X, Y;

	// Construction
	RkVector2() = default;
	RkVector2( float X, float Y );
	RkVector2( const float* V );
	RkVector2( const double* V );

	// Conversion
	operator float*();
	operator const float*() const;

	// Assignment
	RkVector2& operator*=( const RkVector2 & V );
	RkVector2& operator+=( const RkVector2 & V );
	RkVector2& operator-=( const RkVector2 & V );
	RkVector2& operator*=( float F );
	RkVector2& operator/=( float F );

	// Unary operators
	RkVector2 operator+() const;
	RkVector2 operator-() const;
	};

// Binary arithmetic operators
RkVector2 operator*( const RkMatrix2& M, const RkVector2& V );

RkVector2 operator*( const RkVector2& V1, const RkVector2& V2 );
RkVector2 operator/( const RkVector2& V1, const RkVector2& V2 );
RkVector2 operator+( const RkVector2& V1, const RkVector2& V2 );
RkVector2 operator-( const RkVector2& V1, const RkVector2& V2 );
RkVector2 operator*( float F, const RkVector2& V );
RkVector2 operator*( const RkVector2& V, float F );
RkVector2 operator/( const RkVector2& V, float F );

bool operator<( const RkVector2& V1, const RkVector2& V2 );
bool operator==( const RkVector2& V1, const RkVector2& V2 );
bool operator!=( const RkVector2& V1, const RkVector2& V2 );

// Standard vector operations
RkVector2 rkMul( const RkMatrix2& M, const RkVector2& V );
RkVector2 rkTMul( const RkMatrix2& M, const RkVector2& V );

RkVector2 rkNormalize( const RkVector2& V );
RkVector2 rkInvert( const RkVector2& V );
RkVector2 rkPerp( const RkVector2& V );
RkVector2 rkAbs( const RkVector2& V );

float rkCross( const RkVector2& V1, const RkVector2& V2 );
float rkDot( const RkVector2& V1, const RkVector2& V2 );
float rkLength( const RkVector2& V );
float rkLengthSq( const RkVector2& V );
float rkDistance( const RkVector2& V1, const RkVector2& V2 );
float rkDistanceSq( const RkVector2& V1, const RkVector2& V2 );

RkVector2 rkMin( const RkVector2& V1, const RkVector2& V2 );
RkVector2 rkMax( const RkVector2& V1, const RkVector2& V2 );
RkVector2 rkClamp( const RkVector2& V, const RkVector2& Min, const RkVector2& Max );
RkVector2 rkClamp( const RkVector2& V, float Max );

// Interpolation
RkVector2 rkLerp( const RkVector2& V1, const RkVector2& V2, float Alpha );

// Constants
RK_GLOBAL_CONSTANT const RkVector2 RK_VEC2_ONE = RkVector2( 1.0f, 1.0f );
RK_GLOBAL_CONSTANT const RkVector2 RK_VEC2_ZERO = RkVector2( 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector2 RK_VEC2_AXIS_X = RkVector2( 1.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector2 RK_VEC2_AXIS_Y = RkVector2( 0.0f, 1.0f );


//--------------------------------------------------------------------------------------------------
// RkVector3
//--------------------------------------------------------------------------------------------------
struct RkVector3
	{
	// Attributes
	float X, Y, Z;

	// Construction
	RkVector3() = default;
	explicit RkVector3( float F );
	RkVector3( float X, float Y, float Z );
	RkVector3( const float* V );
	RkVector3( const double* V );
		
	// Conversion
	operator float*();
	operator const float*() const;

	// Assignment
	RkVector3& operator*=( const RkVector3& V );
	RkVector3& operator+=( const RkVector3& V );
	RkVector3& operator-=( const RkVector3& V );
	RkVector3& operator*=( float F );
	RkVector3& operator/=( float F );

	// Unary operators
	RkVector3 operator+() const;
	RkVector3 operator-() const;
	};

// Binary arithmetic operators
RkVector3 operator*( const RkMatrix3& M, const RkVector3& V );
RkVector3 operator*( const RkQuaternion& Q, const RkVector3& V );
RkVector3 operator*( const RkTransform& T, const RkVector3& V );

RkVector3 operator*( const RkVector3& V1, const RkVector3& V2 );
RkVector3 operator/( const RkVector3& V1, const RkVector3& V2 );
RkVector3 operator+( const RkVector3& V1, const RkVector3& V2 );
RkVector3 operator-( const RkVector3& V1, const RkVector3& V2 );
RkVector3 operator*( float F, const RkVector3& V );
RkVector3 operator*( const RkVector3& V, float F );
RkVector3 operator/( const RkVector3& V, float F );

bool operator<( const RkVector3& V1, const RkVector3& V2 );
bool operator==( const RkVector3& V1, const RkVector3& V2 );
bool operator!=( const RkVector3& V1, const RkVector3& V2 );

// Standard vector operations
RkVector3 rkMul( const RkMatrix3& M, const RkVector3& V );
RkVector3 rkTMul( const RkMatrix3& M, const RkVector3& V );
RkVector3 rkMul( const RkQuaternion& Q, const RkVector3& V );
RkVector3 rkTMul( const RkQuaternion& Q, const RkVector3& V );
RkVector3 rkMul( const RkTransform& T, const RkVector3& V );
RkVector3 rkTMul( const RkTransform& F, const RkVector3& V );

RkVector3 rkCross( const RkVector3& V1, const RkVector3& V2 );
RkVector3 rkNormalize( const RkVector3& V );
RkVector3 rkInvert( const RkVector3& V );
RkVector3 rkPerp( const RkVector3& V );
RkVector3 rkAbs( const RkVector3& V );

float rkDot( const RkVector3& V1, const RkVector3& V2 );
float rkLength( const RkVector3& V );
float rkLengthSq( const RkVector3& V );
float rkDistance( const RkVector3& V1, const RkVector3& V2 );
float rkDistanceSq( const RkVector3& V1, const RkVector3& V2 );
float rkDet( const RkVector3& V1, const RkVector3& V2, const RkVector3& V3 );

RkVector3 rkMin( const RkVector3& V1, const RkVector3& V2 );
RkVector3 rkMax( const RkVector3& V1, const RkVector3& V2 );
RkVector3 rkClamp( const RkVector3& V, const RkVector3& Min, const RkVector3& Max );
RkVector3 rkClamp( const RkVector3& V, float Max );

int rkMinorAxis( const RkVector3& V );
int rkMajorAxis( const RkVector3& V );
float rkMinElement( const RkVector3& V );
float rkMaxElement( const RkVector3& V );

// Interpolation
RkVector3 rkLerp( const RkVector3& V1, const RkVector3& V2, float Alpha );
RkVector3 rkNLerp( const RkVector3& V1, const RkVector3& V2, float Alpha );

// Transformation
RkVector3 rkTransformPoint( const RkMatrix4& M, const RkVector3& P );
RkVector3 rkTransformVector( const RkMatrix4& M, const RkVector3& V );

RkVector3 rkProject( const RkVector3& Coord, RkMatrix4& ModelView, const RkMatrix4& Projection, const RkViewport& Viewport );
RkVector3 rkUnproject( const RkVector3& Coord, RkMatrix4& ModelView, const RkMatrix4& Projection, const RkViewport& Viewport );

// Constants
RK_GLOBAL_CONSTANT const RkVector3 RK_VEC3_ONE = RkVector3( 1.0f, 1.0f, 1.0f );
RK_GLOBAL_CONSTANT const RkVector3 RK_VEC3_ZERO = RkVector3( 0.0f, 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector3 RK_VEC3_AXIS_X = RkVector3( 1.0f, 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector3 RK_VEC3_AXIS_Y = RkVector3( 0.0f, 1.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector3 RK_VEC3_AXIS_Z = RkVector3( 0.0f, 0.0f, 1.0f );
								  
RK_GLOBAL_CONSTANT const RkVector3 RK_VEC3_LEFT = RkVector3( 1.0f, 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector3 RK_VEC3_UP = RkVector3( 0.0f, 1.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector3 RK_VEC3_FORWARD = RkVector3( 0.0f, 0.0f, 1.0f );


//--------------------------------------------------------------------------------------------------
// RkVector4
//--------------------------------------------------------------------------------------------------
struct RkVector4
	{
	// Attributes
	float X, Y, Z, W;

	// Construction
	RkVector4() = default;
	RkVector4( float X, float Y, float Z, float W );
	RkVector4( const RkVector3& V, float W );
	RkVector4( const float* V );
	RkVector4( const double* V );

	// Conversion
	operator float*();
	operator const float*() const;

	// Assignment
	RkVector4& operator*=( const RkVector4& V );
	RkVector4& operator+=( const RkVector4& V );
	RkVector4& operator-=( const RkVector4& V );
	RkVector4& operator*=( float F );
	RkVector4& operator/=( float F );

	// Unary operators
	RkVector4 operator+() const;
	RkVector4 operator-() const;
	};

// Binary arithmetic operators
RkVector4 operator*( const RkMatrix4& M, const RkVector4& V );
RkVector4 operator*( const RkVector4& V1, const RkVector4& V2 );
RkVector4 operator+( const RkVector4& V1, const RkVector4& V2 );
RkVector4 operator-( const RkVector4& V1, const RkVector4& V2 );
RkVector4 operator*( float S, const RkVector4& V );
RkVector4 operator*( const RkVector4& V, float S );
RkVector4 operator/( const RkVector4& V, float S );

// Comparison operators
bool operator==( const RkVector4& V1, const RkVector4& V2 );
bool operator!=( const RkVector4& V1, const RkVector4& V2 );

// Standard vector operations
RkVector4 rkNormalize( const RkVector4& V );

float rkDot( const RkVector4& V1, const RkVector4& V2 );
float rkLength( const RkVector4& V );
float rkLengthSq( const RkVector4& V );
float rkDistance( const RkVector4& V1, const RkVector4& V2 );
float rkDistanceSq( const RkVector4& V1, const RkVector4& V2 );

RkVector4 rkMin( const RkVector4& V1, const RkVector4& V2 );
RkVector4 rkMax( const RkVector4& V1, const RkVector4& V2 );
RkVector4 rkClamp( const RkVector4& V, const RkVector4& Min, const RkVector4& Max );
RkVector4 rkClamp( const RkVector4& V, float Max );

// Constants
RK_GLOBAL_CONSTANT const RkVector4 RK_VEC4_ONE = RkVector4( 1.0f, 1.0f, 1.0f, 1.0f );
RK_GLOBAL_CONSTANT const RkVector4 RK_VEC4_ZERO = RkVector4( 0.0f, 0.0f, 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector4 RK_VEC4_AXIS_X = RkVector4( 1.0f, 0.0f, 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector4 RK_VEC4_AXIS_Y = RkVector4( 0.0f, 1.0f, 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector4 RK_VEC4_AXIS_Z = RkVector4( 0.0f, 0.0f, 1.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkVector4 RK_VEC4_AXIS_W = RkVector4( 0.0f, 0.0f, 0.0f, 1.0f );


//--------------------------------------------------------------------------------------------------
// RkMatrix2
//--------------------------------------------------------------------------------------------------
struct RkMatrix2
	{
	// Attributes
	union
		{
		struct
			{
			float A11, A21;
			float A12, A22;
			};

		struct
			{
			RkVector2 C1, C2;
			};
		};

	// Construction
	RkMatrix2() = default;
	RkMatrix2( float A11, float A22 );
	RkMatrix2( float A11, float A12, float A21, float A22 );
	RkMatrix2( const RkVector2& C1, const RkVector2& C2 );
	RkMatrix2( const float* M );
	RkMatrix2( const double* M );

	// Conversion
	operator float*();
	operator const float*() const;

	// Assignment
	RkMatrix2& operator*=( const RkMatrix2& M );
	RkMatrix2& operator+=( const RkMatrix2& M );
	RkMatrix2& operator-=( const RkMatrix2& M );
	RkMatrix2& operator*=( float F );
	RkMatrix2& operator/=( float F );

	// Unary operators
	RkMatrix2 operator+() const;
	RkMatrix2 operator-() const;
	};

// Binary arithmetic operators
RkMatrix2 operator*( const RkMatrix2& M1, const RkMatrix2& M2 );
RkMatrix2 operator+( const RkMatrix2& M1, const RkMatrix2& M2 );
RkMatrix2 operator-( const RkMatrix2& M1, const RkMatrix2& M2 );
RkMatrix2 operator*( float F, const RkMatrix2& M );
RkMatrix2 operator*( const RkMatrix2& M, float F );
RkMatrix2 operator/( const RkMatrix2& M, float F );

// Comparison operators
bool operator==( const RkMatrix2& M1, const RkMatrix2& M2 );
bool operator!=( const RkMatrix2& M1, const RkMatrix2& M2 );

// Standard matrix operations
RkMatrix2 rkMul( const RkMatrix2& M1, const RkMatrix2& M2 );
RkMatrix2 rkTMul( const RkMatrix2& M1, const RkMatrix2& M2 );
RkMatrix2 rkTranspose( const RkMatrix2& M );
RkMatrix2 rkInvert( const RkMatrix2& M );
RkMatrix2 rkInvertT( const RkMatrix2& M );
RkMatrix2 rkAbs( const RkMatrix2& M );

float rkTrace( const RkMatrix2& M );
float rkDet( const RkMatrix2& M );

// Constants
RK_GLOBAL_CONSTANT const RkMatrix2 RK_MAT2_ZERO = RkMatrix2( 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkMatrix2 RK_MAT2_IDENTITY = RkMatrix2( 1.0f, 1.0f );


//--------------------------------------------------------------------------------------------------
// RkMatrix3
//--------------------------------------------------------------------------------------------------
struct RkMatrix3
	{
	// Attributes
	union
		{
		struct
			{
			float A11, A21, A31;
			float A12, A22, A32;
			float A13, A23, A33;
			};

		struct
			{
			RkVector3 C1, C2, C3;
			};
		};

	// Construction
	RkMatrix3() = default;
	RkMatrix3( float A11, float A22, float A33 );
	RkMatrix3( const RkVector3 & C1, const RkVector3 & C2, const RkVector3 & C3 );
	explicit RkMatrix3( const RkQuaternion & Q );
	RkMatrix3( const float* M );
	RkMatrix3( const double* M );

	// Conversion
	operator float*();
	operator const float*() const;

	// Assignment
	RkMatrix3& operator*=( const RkMatrix3 & M );
	RkMatrix3& operator+=( const RkMatrix3 & M );
	RkMatrix3& operator-=( const RkMatrix3 & M );
	RkMatrix3& operator*=( float F );
	RkMatrix3& operator/=( float F );

	// Unary operators
	RkMatrix3 operator+() const;
	RkMatrix3 operator-() const;
	};

// Binary arithmetic operators
RkMatrix3 operator*( const RkMatrix3& M1, const RkMatrix3& M2 );
RkMatrix3 operator+( const RkMatrix3& M1, const RkMatrix3& M2 );
RkMatrix3 operator-( const RkMatrix3& M1, const RkMatrix3& M2 );
RkMatrix3 operator*( float F, const RkMatrix3& M );
RkMatrix3 operator*( const RkMatrix3& M, float F );
RkMatrix3 operator/( const RkMatrix3& M, float F );

// Comparison operators
bool operator==( const RkMatrix3& M1, const RkMatrix3& M2 );
bool operator!=( const RkMatrix3& M1, const RkMatrix3& M2 );

// Standard matrix operations
RkMatrix3 rkMul( const RkMatrix3& M1, const RkMatrix3& M2 );
RkMatrix3 rkTMul( const RkMatrix3& M1, const RkMatrix3& M2 );
RkMatrix3 rkTranspose( const RkMatrix3& M );
RkMatrix3 rkInvert( const RkMatrix3& M );
RkMatrix3 rkInvertT( const RkMatrix3& M );
RkMatrix3 rkSkew( const RkVector3& V );
RkMatrix3 rkAbs( const RkMatrix3& M );

float rkTrace( const RkMatrix3& M );
float rkDet( const RkMatrix3& M );

// Constants
RK_GLOBAL_CONSTANT const RkMatrix3 RK_MAT3_ZERO = RkMatrix3( 0.0f, 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkMatrix3 RK_MAT3_IDENTITY = RkMatrix3( 1.0f, 1.0f, 1.0f );


//--------------------------------------------------------------------------------------------------
// RkMatrix4
//--------------------------------------------------------------------------------------------------
struct RkMatrix4
	{
	// Attributes
	union 
		{
		struct
			{
			float A11, A21, A31, A41;
			float A12, A22, A32, A42;
			float A13, A23, A33, A43;
			float A14, A24, A34, A44;
			};

		struct
			{
			RkVector4 C1, C2, C3, C4;
			};
		};

	// Construction
	RkMatrix4() = default;
	RkMatrix4( float A11, float A22, float A33, float A44 );
	RkMatrix4( const RkVector4& C1, const RkVector4& C2, const RkVector4& C3, const RkVector4& C4 );
	RkMatrix4( const RkVector3& Translation, const RkQuaternion& Rotation, const RkVector3& Scale = RK_VEC3_ONE );
	RkMatrix4( const float* M );
	RkMatrix4( const double* M );

	explicit RkMatrix4( const RkMatrix3& M );
	explicit RkMatrix4( const RkQuaternion& Q );
	explicit RkMatrix4( const RkTransform& T );

	// Conversion
	operator float*();
	operator const float*() const;
	
	// Assignment
	RkMatrix4& operator*=( const RkMatrix4& M );
	RkMatrix4& operator+=( const RkMatrix4& M );
	RkMatrix4& operator-=( const RkMatrix4& M );
	RkMatrix4& operator*=( float F );
	RkMatrix4& operator/=( float F );

	// Unary operators
	RkMatrix4 operator+() const;
	RkMatrix4 operator-() const;
	};

// Binary arithmetic operators
RkMatrix4 operator*( const RkMatrix4& M1, const RkMatrix4& M2 );
RkMatrix4 operator+( const RkMatrix4& M1, const RkMatrix4& M2 );
RkMatrix4 operator-( const RkMatrix4& M1, const RkMatrix4& M2 );

RkMatrix4 operator*( float S, const RkMatrix4& M );
RkMatrix4 operator*( const RkMatrix4& M, float S );
RkMatrix4 operator/( const RkMatrix4& M, float S );

// Comparison operators
bool operator==( const RkMatrix4& M1, const RkMatrix4& M2 );
bool operator!=( const RkMatrix4& M1, const RkMatrix4& M2 );

// Standard matrix operations
RkMatrix4 rkTranspose( const RkMatrix4& M );
RkMatrix4 rkInvert( const RkMatrix4& M );
RkMatrix4 rkInvertT( const RkMatrix4& M );

// Affine transformations
RkMatrix4 rkScale( float S );
RkMatrix4 rkScale( const RkVector3& S );
RkMatrix4 rkRotation( const RkQuaternion& Q );
RkMatrix4 rkRotationX( float Rad );
RkMatrix4 rkRotationY( float Rad );
RkMatrix4 rkRotationZ( float Rad );
RkMatrix4 rkTranslation( const RkVector3& T );

bool rkIsAffine( const RkMatrix4& M );
RkMatrix4 rkInvertAffine( const RkMatrix4& M );
RkMatrix4 rkLookAt( const RkVector3& Eye, const RkVector3& Center, const RkVector3& Up = RK_VEC3_UP );
RkMatrix4 rkPerspective( float FieldOfView, float AspectRatio, float NearPlane, float FarPlane );
RkMatrix4 rkOrtho( float Left, float Right, float Bottom, float Top, float Near = -1.0f, float Far = 1.0f );

// Constants
RK_GLOBAL_CONSTANT const RkMatrix4 RK_MAT4_ZERO = RkMatrix4( 0.0f, 0.0f, 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkMatrix4 RK_MAT4_IDENTITY = RkMatrix4( 1.0f, 1.0f, 1.0f, 1.0f );


//--------------------------------------------------------------------------------------------------
// RkQuaternion
//--------------------------------------------------------------------------------------------------
struct RkQuaternion
	{
	// Attributes
	union
		{
		struct 
			{
			float X, Y, Z, W;
			};

		struct 
			{
			RkVector3 V;
			float S;
			};
		};

	// Construction
	RkQuaternion() = default;
	RkQuaternion( float X, float Y, float Z, float W );
	RkQuaternion( const RkVector3& Axis, float Rad );
	RkQuaternion( const float* Q );
	RkQuaternion( const double* Q );

	explicit RkQuaternion( const RkVector3& Euler );
	explicit RkQuaternion( const RkMatrix3& M );

	// Conversion
	operator float*();
	operator const float*() const;

	// Assignment
	RkQuaternion& operator*=( const RkQuaternion& Q );
	RkQuaternion& operator+=( const RkQuaternion& Q );
	RkQuaternion& operator-=( const RkQuaternion& Q );
	RkQuaternion& operator*=( float F );
	RkQuaternion& operator/=( float F );

	// Unary operators
	RkQuaternion operator+() const;
	RkQuaternion operator-() const;
	};

// Binary arithmetic operators
RkQuaternion operator*( const RkQuaternion& Q1, const RkQuaternion& Q2 );
RkQuaternion operator+( const RkQuaternion& Q1, const RkQuaternion& Q2 );
RkQuaternion operator-( const RkQuaternion& Q1, const RkQuaternion& Q2 );
RkQuaternion operator*( float F, const RkQuaternion& Q );
RkQuaternion operator*( const RkQuaternion& Q, float F );
RkQuaternion operator/( const RkQuaternion& Q, float F );

// Comparison operators
bool operator==( const RkQuaternion& Q1, const RkQuaternion& Q2 );
bool operator!=( const RkQuaternion& Q1, const RkQuaternion& Q2 );

// Standard quaternion operations
RkQuaternion rkMul( const RkQuaternion& Q1, const RkQuaternion& Q2 );
RkQuaternion rkCMul( const RkQuaternion& Q1, const RkQuaternion& Q2 );
RkQuaternion rkConjugate( const RkQuaternion& Q );
RkQuaternion rkInvert( const RkQuaternion& Q );
RkQuaternion rkNormalize( const RkQuaternion& Q );
RkQuaternion rkExp( const RkVector3& V );
RkQuaternion rkShortestArc( const RkVector3& V1, const RkVector3& V2 );

float rkDot( const RkQuaternion& Q1, const RkQuaternion& Q2 );
float rkLength( const RkQuaternion& Q );
float rkLengthSq( const RkQuaternion& Q );

// Quaternion interpolation
RkQuaternion rkNLerp( RkQuaternion Q1, RkQuaternion Q2, float Alpha );

// Constants
RK_GLOBAL_CONSTANT const RkQuaternion RK_QUAT_ZERO = RkQuaternion( 0.0f, 0.0f, 0.0f, 0.0f );
RK_GLOBAL_CONSTANT const RkQuaternion RK_QUAT_IDENTITY = RkQuaternion( 0.0f, 0.0f, 0.0f, 1.0f );


//--------------------------------------------------------------------------------------------------
// RkTransform
//--------------------------------------------------------------------------------------------------
struct RkTransform
	{
	// Attributes
	RkVector3 Translation;
	RkQuaternion Rotation;
	
	// Construction
	RkTransform() = default;
	RkTransform( const RkVector3& Translation, const RkQuaternion& Rotation );
	};

// Binary arithmetic operators
RkTransform operator*( const RkTransform& T1, const RkTransform& T2 );

// Comparison operators
bool operator==( const RkTransform& T1, const RkTransform& T2 );
bool operator!=( const RkTransform& T1, const RkTransform& T2 );

// Standard transform operations
RkTransform rkMul( const RkTransform& T1, const RkTransform& T2 );
RkTransform rkTMul( const RkTransform& T1, const RkTransform& T2 );
RkTransform rkInvert( const RkTransform& T );
RkTransform rkNormalize( const RkTransform& T );

// Interpolation
RkTransform rkLerp( const RkTransform& T1, const RkTransform& T2, float Alpha );

// Constants 
RK_GLOBAL_CONSTANT const RkTransform RK_TRANSFORM_ZERO = RkTransform( RkVector3( 0.0f, 0.0f, 0.0f ), RkQuaternion( 0.0f, 0.0f, 0.0f, 0.0f ) );
RK_GLOBAL_CONSTANT const RkTransform RK_TRANSFORM_IDENTITY = RkTransform( RkVector3( 0.0f, 0.0f, 0.0f ), RkQuaternion( 0.0f, 0.0f, 0.0f, 1.0f ) );


//--------------------------------------------------------------------------------------------------
// RkSweep
//--------------------------------------------------------------------------------------------------
struct RkSweep
	{
	float Alpha0;
	RkVector3 LocalCenter;
	RkVector3 Center0, Center;
	RkQuaternion Orientation0, Orientation;

	RkSweep() = default;
	RkSweep( const RkVector3& LocalCenter, const RkTransform& Transform0, const RkTransform& Transform );
	RkSweep( const RkVector3& LocalCenter, const RkVector3& Center0, const RkVector3& Center, const RkQuaternion& Orientation0, const RkQuaternion& Orientation );

	void Normalize();
	bool IsNormalized() const;

	void Advance( float Beta );
	RkTransform Interpolate( float Beta ) const;
	};

// Comparison operators
bool operator==( const RkSweep& Sweep1, const RkSweep& Sweep2 );
bool operator!=( const RkSweep& Sweep1, const RkSweep& Sweep2 );

// Constants 
RK_GLOBAL_CONSTANT const RkSweep RK_SWEEP_IDENTITY = RkSweep( RkVector3( 0.0f, 0.0, 0.0f ), RkVector3( 0.0f, 0.0, 0.0f ), RkVector3( 0.0f, 0.0, 0.0f ), RkQuaternion( 0.0f, 0.0f, 0.0f, 1.0f ), RkQuaternion( 0.0f, 0.0f, 0.0f, 1.0f ) );



//--------------------------------------------------------------------------------------------------
// RkPlane2
//--------------------------------------------------------------------------------------------------
struct RkPlane2
	{
	// Attributes
	RkVector2 Normal;
	float Offset;

	// Construction
	RkPlane2() = default;
	RkPlane2( const RkVector2& Normal, float Offset );
	RkPlane2( const RkVector2& Normal, const RkVector2& Point );

	// Unary arithmetic operators
	RkPlane2 operator+() const;
	RkPlane2 operator-() const;
	};

// Standard plane operations
RkPlane2 rkNormalize( const RkPlane2& Plane );
float rkDistance( const RkPlane2& Plane, const RkVector2& Point );


//--------------------------------------------------------------------------------------------------
// RkPlane3
//--------------------------------------------------------------------------------------------------
struct RkPlane3
	{
	// Attributes
	RkVector3 Normal;
	float Offset;

	// Construction
	RkPlane3() = default;
	RkPlane3( const RkVector3& Normal, float Offset );
	RkPlane3( const RkVector3& Normal, const RkVector3& Point );
	RkPlane3( const RkVector3& Point1, const RkVector3& Point2, const RkVector3& Point3 );

	// Unary arithmetic operators
	RkPlane3 operator+() const;
	RkPlane3 operator-() const;
	};

// Binary arithmetic operators
RkPlane3 operator*( const RkTransform& Transform, const RkPlane3& Plane );
RkPlane3 operator*( float Scale, const RkPlane3& Plane );
RkPlane3 operator*( const RkPlane3& Plane, float Scale );

bool operator==( const RkPlane3& Plane1, const RkPlane3& Plane2 );
bool operator!=( const RkPlane3& Plane1, const RkPlane3& Plane2 );

// Standard plane operations
RkPlane3 rkNormalize( const RkPlane3& Plane );
float rkDistance( const RkPlane3& Plane, const RkVector3& Point );


//--------------------------------------------------------------------------------------------------
// RkBounds2
//--------------------------------------------------------------------------------------------------
struct RkBounds2
	{
	// Attributes
	RkVector2 Min, Max;

	// Construction
	RkBounds2() = default;
	RkBounds2( const RkVector2& Min, const RkVector2& Max );

	// Assignment
	RkBounds2& operator+=( const RkVector2& Point );
	RkBounds2& operator+=( const RkBounds2& Bounds );

	// Standard bounds operations
	RkVector2 GetCenter() const;
	RkVector2 GetExtent() const;

	bool Contains( const RkVector2& Point ) const;
	bool Contains( const RkBounds2& Bounds ) const;

	bool IsValid() const;
	};

// Binary arithmetic operators
RkBounds2 operator+( const RkBounds2& Bounds1, const RkBounds2& Bounds2 );

// Comparison operators
bool operator==( const RkBounds2& Bounds1, const RkBounds2& Bounds2 );
bool operator!=( const RkBounds2& Bounds1, const RkBounds2& Bounds2 );

// Standard bounds operations
bool rkOverlap( const RkBounds2& Bounds1, const RkBounds2& Bounds2 );

// Constants
RK_GLOBAL_CONSTANT const RkBounds2 RK_BOUNDS2_EMPTY = RkBounds2( RkVector2( RK_F32_MAX, RK_F32_MAX ), RkVector2( -RK_F32_MAX, -RK_F32_MAX ) );
RK_GLOBAL_CONSTANT const RkBounds2 RK_BOUNDS2_INFINITE = RkBounds2( RkVector2( -RK_F32_MAX, -RK_F32_MAX ), RkVector2( RK_F32_MAX, RK_F32_MAX ) );


//--------------------------------------------------------------------------------------------------
// RkBounds3
//--------------------------------------------------------------------------------------------------
struct RkBounds3
	{
	// Attributes
	RkVector3 Min, Max;

	// Construction
	RkBounds3() = default;
	RkBounds3( const RkVector3& Min, const RkVector3& Max );

	// Assignment
	RkBounds3& operator+=( const RkVector3& Point );
	RkBounds3& operator+=( const RkBounds3& Bounds );
	RkBounds3& operator*=( const RkVector3& Scale );
	RkBounds3& operator*=( float Scale );

	// Standard bounds operations
	RkVector3 GetCenter() const;
	RkVector3 GetExtent() const;
	float GetPerimeter() const;
	float GetVolume() const;

	bool Contains( const RkVector3& Point ) const;
	bool Contains( const RkBounds3& Bounds ) const;

	bool IsValid() const;
	};

// Binary arithmetic operators
RkBounds3 operator*( const RkTransform& Transform, const RkBounds3& Bounds );
RkBounds3 operator*( const RkBounds3& Bounds, const RkVector3& Scale );
RkBounds3 operator*( const RkVector3& Scale, const RkBounds3& Bounds );
RkBounds3 operator*( const RkBounds3& Bounds, float Scale );
RkBounds3 operator*( float Scale, const RkBounds3& Bounds );
RkBounds3 operator+( const RkBounds3& Bounds1, const RkBounds3& Bounds2 );

// Comparison operators
bool operator==( const RkBounds3& Bounds1, const RkBounds3& Bounds2 );
bool operator!=( const RkBounds3& Bounds1, const RkBounds3& Bounds2 );

// Standard bounds operations
float rkPerimeter( const RkBounds3& Bounds );
RkVector3 rkCenter( const RkBounds3& Bounds );
RkVector3 rkExtent( const RkBounds3& Bounds );
RkBounds3 rkUnion( const RkBounds3& Bounds1, const RkBounds3& Bounds2 );
RkBounds3 rkInflate( const RkBounds3& Bounds, float Extension );

bool rkOverlap( const RkBounds3& Bounds1, const RkBounds3& Bounds2 );
bool rkContains( const RkBounds3& Bounds1, const RkBounds3& Bounds2 );

// Constants
RK_GLOBAL_CONSTANT const RkBounds3 RK_BOUNDS3_EMPTY = RkBounds3( RkVector3( RK_F32_MAX, RK_F32_MAX, RK_F32_MAX ), RkVector3( -RK_F32_MAX, -RK_F32_MAX, -RK_F32_MAX ) );
RK_GLOBAL_CONSTANT const RkBounds3 RK_BOUNDS3_INFINITE = RkBounds3( RkVector3( -RK_F32_MAX, -RK_F32_MAX, -RK_F32_MAX ), RkVector3( RK_F32_MAX, RK_F32_MAX, RK_F32_MAX ) );


//--------------------------------------------------------------------------------------------------
// Geometry
//--------------------------------------------------------------------------------------------------
struct RkClosestApproachResult
	{
	RkVector3 Point1;
	float Lambda1;
	RkVector3 Point2;
	float Lambda2;

	bool IsNormalized() const;
	};

void rkClosestApproachLines( RkClosestApproachResult& Out, const RkVector3& P1, const RkVector3& D1, const RkVector3& P2, const RkVector3& D2 );
void rkClosestApproachSegments( RkClosestApproachResult& Out, const RkVector3& P1, const RkVector3& Q1, const RkVector3& P2, const RkVector3& Q2 );

RkVector3 rkClosestPointOnSegment( const RkVector3& A, const RkVector3& B, const RkVector3& Q );
RkVector3 rkClosestPointOnTriangle( const RkVector3& A, const RkVector3& B, const RkVector3& C, const RkVector3& Q );

float rkIntersectSegmentTriangle( const RkVector3& P, const RkVector3& Q, const RkVector3& A, const RkVector3& B, const RkVector3& C );
float rkIntersectSegmentSphere( const RkVector3& P, const RkVector3& Q, const RkVector3& C, float R );


//--------------------------------------------------------------------------------------------------
// RkColor
//--------------------------------------------------------------------------------------------------
struct RkColor
	{
	// Attributes
	float R, G, B, A;

	// Construction
	RkColor() = default;
	RkColor( float R, float G, float B, float A = 1.0f );

	// Conversion
	operator float*();
	operator const float*() const;
	};

// Binary arithmetic operators
bool operator==( const RkColor& C1, const RkColor& C2 );
bool operator!=( const RkColor& C1, const RkColor& C2 );

// Constants
RK_GLOBAL_CONSTANT RkColor RK_COLOR_TRANSPARENT = RkColor( 0.0f, 0.0f, 0.0f, 0.0f );

RK_GLOBAL_CONSTANT RkColor RK_COLOR_WHITE = RkColor( 1.0f, 1.0f, 1.0f );
RK_GLOBAL_CONSTANT RkColor RK_COLOR_LIGHTGRAY = RkColor( 0.75f, 0.75f, 0.75f );
RK_GLOBAL_CONSTANT RkColor RK_COLOR_GRAY = RkColor( 0.5f, 0.5f, 0.5f );
RK_GLOBAL_CONSTANT RkColor RK_COLOR_DARKGRAY = RkColor( 0.25f, 0.25f, 0.25f );
RK_GLOBAL_CONSTANT RkColor RK_COLOR_BLACK = RkColor( 0.0f, 0.0f, 0.0f );

RK_GLOBAL_CONSTANT RkColor RK_COLOR_RED = RkColor( 0.9f, 0.4f, 0.4f );
RK_GLOBAL_CONSTANT RkColor RK_COLOR_GREEN = RkColor( 0.4f, 0.9f, 0.4f );
RK_GLOBAL_CONSTANT RkColor RK_COLOR_BLUE = RkColor( 0.4f, 0.4f, 0.9f );
RK_GLOBAL_CONSTANT RkColor RK_COLOR_YELLOW = RkColor( 0.9f, 0.9f, 0.4f );
RK_GLOBAL_CONSTANT RkColor RK_COLOR_CYAN = RkColor( 0.4f, 0.9f, 0.9f );
RK_GLOBAL_CONSTANT RkColor RK_COLOR_PURPLE = RkColor( 0.9f, 0.4f, 0.9f );


#define RK_COL32(R,G,B,A)    (((uint32)(A)<<24) | ((uint32)(B)<<16) | ((uint32)(G)<<8) | ((uint32)(R)<<0))
#define RK_COL32_WHITE       RK_COL32(255,255,255,255)  // Opaque white = 0xFFFFFFFF
#define RK_COL32_BLACK       RK_COL32(0,0,0,255)        // Opaque black


//--------------------------------------------------------------------------------------------------
// RkViewport
//--------------------------------------------------------------------------------------------------
struct RkViewport
	{
	// Attributes
	int X = 0;
	int Y = 0;
	int Width = 0;
	int Height = 0;

	// Construction
	RkViewport() = default;
	RkViewport( int X, int Y, int Width, int Height );
	};

// Binary arithmetic operators
bool operator==( const RkViewport& Lhs, const RkViewport& Rhs );
bool operator!=( const RkViewport& Lhs, const RkViewport& Rhs );


//--------------------------------------------------------------------------------------------------
// General matrix inversion
//--------------------------------------------------------------------------------------------------
void rkInvert( float* Out, float* M, int N, int Lda );


#include "math.inl"