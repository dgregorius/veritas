//-------------------------------------------------------------------------------------------------
/**
	@file		vMath.h

	@author		Dirk Gregorius
	@version	0.1
	@date		09/02/2011

	Math system using SSE/SSE2

	Copyright(C) 2012 by D. Gregorius. All rights reserved.
*/
//-------------------------------------------------------------------------------------------------
#pragma once

#include <xmmintrin.h>
#include <emmintrin.h>

struct m32;


//-------------------------------------------------------------------------------------------------
// Macros
//-------------------------------------------------------------------------------------------------
#define VM_FORCEINLINE	__forceinline
#define VM_CALLCONV		__vectorcall
#define VM_ALIGN16		__declspec( align( 16 ) )
#define VM_RESTRICT		__restrict

#define VM_CONSTANT		extern const __declspec(selectany)


//-------------------------------------------------------------------------------------------------
// Vector type
//-------------------------------------------------------------------------------------------------
typedef __m128 v32;

v32 VM_CALLCONV operator*( m32 M, v32 V );
v32 VM_CALLCONV operator+( v32 V1, v32 V2 );
v32 VM_CALLCONV operator-( v32 V1, v32 V2 );
v32 VM_CALLCONV operator*( v32 V1, v32 V2 );
v32 VM_CALLCONV operator/( v32 V1, v32 V2 );

v32 VM_CALLCONV operator+( v32 V );
v32 VM_CALLCONV operator-( v32 V );


//-------------------------------------------------------------------------------------------------
// Vector load/store operations
//-------------------------------------------------------------------------------------------------
v32 VM_CALLCONV vmLoad3( const float* Src );
v32 VM_CALLCONV vmLoad3A( const float* Src );
v32 VM_CALLCONV vmLoad4( const float* Src );
v32 VM_CALLCONV vmLoad4A( const float* Src );

void VM_CALLCONV vmStore3( float* Dst, v32 V );
void VM_CALLCONV vmStore3A( float* Dst, v32 V );
void VM_CALLCONV vmStore4( float* Dst, v32 V );
void VM_CALLCONV vmStore4A( float* Dst, v32 V );


//-------------------------------------------------------------------------------------------------
// General vector operations
//-------------------------------------------------------------------------------------------------
v32 VM_CALLCONV vmZero();
v32 VM_CALLCONV vmSet( float X, float Y, float Z, float W = 0.0f );

float VM_CALLCONV vmGetX( v32 V );
float VM_CALLCONV vmGetY( v32 V );
float VM_CALLCONV vmGetZ( v32 V );
float VM_CALLCONV vmGetW( v32 V );
float VM_CALLCONV vmGet( v32 V, int Index );

v32 VM_CALLCONV vmSplat( float X );
v32 VM_CALLCONV vmSplatX( v32 V );
v32 VM_CALLCONV vmSplatY( v32 V );
v32 VM_CALLCONV vmSplatZ( v32 V );
v32 VM_CALLCONV vmSplatW( v32 V );

v32 VM_CALLCONV vmMul( m32 M, v32 V );
v32 VM_CALLCONV vmTMul( m32 M, v32 V );

v32 VM_CALLCONV vmAdd( v32 V1, v32 V2 );
v32 VM_CALLCONV vmSub( v32 V1, v32 V2 );
v32 VM_CALLCONV vmMul( v32 V1, v32 V2 );
v32 VM_CALLCONV vmDiv( v32 V1, v32 V2 );
v32 VM_CALLCONV vmNegate( v32 V );
v32 VM_CALLCONV vmAbs( v32 V );

v32 VM_CALLCONV vmMin( v32 V1, v32 V2 );
v32 VM_CALLCONV vmMax( v32 V1, v32 V2 );
v32 VM_CALLCONV vmClamp( v32 V, v32 Min, v32 Max );

v32 VM_CALLCONV vmEqual( v32 V1, v32 V2 );
v32 VM_CALLCONV vmLess( v32 V1, v32 V2 );
v32 VM_CALLCONV vmLessEq( v32 V1, v32 V2 );
v32 VM_CALLCONV vmGreater( v32 V1, v32 V2 );
v32 VM_CALLCONV vmGreaterEq( v32 V1, v32 V2 );
v32 VM_CALLCONV vmSelect( v32 C, v32 V1, v32 V2 );


//-------------------------------------------------------------------------------------------------
// 3d vector operations
//-------------------------------------------------------------------------------------------------
v32 VM_CALLCONV vmCross3( v32 V1, v32 V2 );
v32 VM_CALLCONV vmModifiedCross3( v32 V1, v32 V2 );

v32 VM_CALLCONV vmDot3( v32 V1, v32 V2 );
v32 VM_CALLCONV vmLength3( v32 V );
v32 VM_CALLCONV vmNormalize3( v32 V );

bool VM_CALLCONV vmAnyEqual3( v32 V1, v32 V2 );
bool VM_CALLCONV vmAnyLess3( v32 V1, v32 V2 );
bool VM_CALLCONV vmAnyLessEq3( v32 V1, v32 V2 );
bool VM_CALLCONV vmAnyGreater3( v32 V1, v32 V2 );
bool VM_CALLCONV vmAnyGreaterEq3( v32 V1, v32 V2 );

bool VM_CALLCONV vmAllEqual3( v32 V1, v32 V2 );
bool VM_CALLCONV vmAllLess3( v32 V1, v32 V2 );
bool VM_CALLCONV vmAllLessEq3( v32 V1, v32 V2 );
bool VM_CALLCONV vmAllGreater3( v32 V1, v32 V2 );
bool VM_CALLCONV vmAllGreaterEq3( v32 V1, v32 V2 );


//-------------------------------------------------------------------------------------------------
// Matrix type
//-------------------------------------------------------------------------------------------------
struct m32
	{
	v32 C1, C2, C3;
	};

m32 VM_CALLCONV operator+( m32 M1, m32 M2 );
m32 VM_CALLCONV operator-( m32 M1, m32 M2 );
m32 VM_CALLCONV operator*( m32 M1, m32 M2 );

m32 VM_CALLCONV operator+( m32 M );
m32 VM_CALLCONV operator-( m32 M );


//-------------------------------------------------------------------------------------------------
// 3d matrix operations
//-------------------------------------------------------------------------------------------------
m32 VM_CALLCONV vmTranspose( m32 M );
m32 VM_CALLCONV vmAbs( m32 M );
m32 VM_CALLCONV vmSkew( v32 V );


//--------------------------------------------------------------------------------------------------
// AABB overlap tests
//--------------------------------------------------------------------------------------------------
bool VM_CALLCONV vmTestBoundsOverlap( v32 NodeMin1, v32 NodeMax1, v32 NodeMin2, v32 NodeMax2 );
bool VM_CALLCONV vmTestBoundsRayOverlap( v32 NodeMin, v32 NodeMax, v32 RayStart, v32 RayDelta );
bool VM_CALLCONV vmTestBoundsRayOverlap( v32 NodeMin, v32 NodeMax, v32 RayStart, v32 RayDelta, v32 Lambda );
bool VM_CALLCONV vmTestBoundsTriangleOverlap( v32 NodeCenter, v32 NodeExtent, v32 Vertex1, v32 Vertex2, v32 Vertex3 );

bool VM_CALLCONV vmContains( v32 ParentMin, v32 ParentMax, v32 ChildMin, v32 ChildMax );
void VM_CALLCONV vmUnion( v32& VM_RESTRICT ParentMin, v32& VM_RESTRICT ParentMax, v32 ChildMin, v32 ChildMax );


//--------------------------------------------------------------------------------------------------
// Ray intersection tests
//--------------------------------------------------------------------------------------------------
float VM_CALLCONV vmIntersectRayTriangle( v32 RayStart, v32 RayDelta, v32 Vertex1, v32 Vertex2, v32 Vertex3 );


//-------------------------------------------------------------------------------------------------
// Conversion types for constants
//-------------------------------------------------------------------------------------------------
VM_ALIGN16 struct VM_VECTORF32
	{
	union
		{
		float F[4];
		v32 V;
		};

	VM_FORCEINLINE operator v32() const { return V; }
	VM_FORCEINLINE operator const float*() const { return F; }
	VM_FORCEINLINE operator __m128i() const { return _mm_castps_si128( V ); }
	VM_FORCEINLINE operator __m128d() const { return _mm_castps_pd( V ); }
	};

VM_CONSTANT VM_VECTORF32 VM_ZERO = { 0.0f, 0.0f, 0.0f, 0.0f };
VM_CONSTANT VM_VECTORF32 VM_HALF = { 0.5f, 0.5f, 0.5f, 0.5f };
VM_CONSTANT VM_VECTORF32 VM_ONE = { 1.0f, 1.0f, 1.0f, 1.0f };
VM_CONSTANT VM_VECTORF32 VM_TWO = { 2.0f, 2.0f, 2.0f, 2.0f };
VM_CONSTANT VM_VECTORF32 VM_THREE = { 3.0f, 3.0f, 3.0f, 3.0f };
VM_CONSTANT VM_VECTORF32 VM_SIX = { 6.0f, 6.0f, 6.0f, 6.0f };

VM_CONSTANT VM_VECTORF32 VM_CROSS1 = {  0.0f,  1.0f, -1.0f, 0.0f };
VM_CONSTANT VM_VECTORF32 VM_CROSS2 = { -1.0f,  0.0f,  1.0f, 0.0f };
VM_CONSTANT VM_VECTORF32 VM_CROSS3 = {  1.0f, -1.0f,  0.0f, 0.0f };


//-------------------------------------------------------------------------------------------------
VM_ALIGN16 struct VM_VECTORI32
	{
	union
		{
		int I[4];
		v32 V;
		};

	VM_FORCEINLINE operator v32() const { return V; }
	VM_FORCEINLINE operator const int*() const { return I; }
	VM_FORCEINLINE operator __m128i() const { return _mm_castps_si128( V ); }
	VM_FORCEINLINE operator __m128d() const { return _mm_castps_pd( V ); }
	};

VM_CONSTANT VM_VECTORI32 VM_MASKX = { 0xFFFFFFFF, 0x00000000, 0x00000000, 0x00000000 };
VM_CONSTANT VM_VECTORI32 VM_MASKY = { 0x00000000, 0xFFFFFFFF, 0x00000000, 0x00000000 };
VM_CONSTANT VM_VECTORI32 VM_MASKZ = { 0x00000000, 0x00000000, 0xFFFFFFFF, 0x00000000 };
VM_CONSTANT VM_VECTORI32 VM_MASKW = { 0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF };
VM_CONSTANT VM_VECTORI32 VM_MASK0 = { 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
VM_CONSTANT VM_VECTORI32 VM_MASK1 = { 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF };
VM_CONSTANT VM_VECTORI32 VM_MASK2 = { 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF };
VM_CONSTANT VM_VECTORI32 VM_MASK3 = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000 };

VM_CONSTANT VM_VECTORI32 VM_MIN3 = { 0x0000FFFF, 0x0000FFFF, 0x0000FFFF, 0x00000000 };
VM_CONSTANT VM_VECTORI32 VM_MAX3 = { 0xFFFF0000, 0xFFFF0000, 0xFFFF0000, 0x00000000 };



//-------------------------------------------------------------------------------------------------
// Utilities
//-------------------------------------------------------------------------------------------------
#define VM_SHUFFLE( FP1, FP2, FP3, FP4 )	_MM_SHUFFLE( FP4, FP3, FP2, FP1 )

//-------------------------------------------------------------------------------------------------
#define VM_TRANSPOSE3( C1, C2, C3 )											\
	{																		\
	v32 T1 = _mm_unpacklo_ps( ( C1 ), ( C2 ) );								\
	v32 T2 = _mm_unpackhi_ps( ( C1 ), ( C2 ) );								\
	( C1 ) = _mm_shuffle_ps( ( T1 ), ( C3 ), _MM_SHUFFLE( 0, 0, 1, 0 ) );	\
	( C2 ) = _mm_shuffle_ps( ( T1 ), ( C3 ), _MM_SHUFFLE( 1, 1, 3, 2 ) );	\
	( C3 ) = _mm_shuffle_ps( ( T2 ), ( C3 ), _MM_SHUFFLE( 2, 2, 1, 0 ) );	\
	}
	

//-------------------------------------------------------------------------------------------------
#define VM_TRANSPOSE4( C1, C2, C3, C4 )										\
	{																		\
	v32 T1 = _mm_unpacklo_ps( ( C1 ), ( C2 ) );								\
	v32 T2 = _mm_unpacklo_ps( ( C3 ), ( C4 ) );								\
	v32 T3 = _mm_unpacklo_ps( ( C1 ), ( C2 ) );								\
	v32 T4 = _mm_unpacklo_ps( ( C3 ), ( C4 ) );								\
	( C1 ) = _mm_unpacklo_ps( ( T1 ), ( T2 ) );								\
	( C2 ) = _mm_unpackhi_ps( ( T1 ), ( T2 ) );								\
	( C3 ) = _mm_unpacklo_ps( ( T3 ), ( T4 ) );								\
	( C4 ) = _mm_unpackhi_ps( ( T3 ), ( T4 ) );								\
	}																											



#include "simd.inl"
