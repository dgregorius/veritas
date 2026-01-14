// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

struct b3Sweep;
struct b3Plane;

#define B3_TWO_PI 6.283185307f
#define B3_PI_OVER_TWO 1.570796327f
#define B3_PI_OVER_FOUR 0.785398163f
#define B3_SQRT3 1.732050808f

// todo eliminate this
static const b3AABB B3_BOUNDS3_EMPTY = { { FLT_MAX, FLT_MAX, FLT_MAX }, { -FLT_MAX, -FLT_MAX, -FLT_MAX } };

typedef struct b3Matrix2
{
	b3Vec2 cx, cy;
} b3Matrix2;

typedef struct b3Triangle
{
	b3Vec3 vertices[3];
	int i1, i2, i3;
	int flags;
} b3Triangle;

bool b3IsSweepNormalized( b3Sweep* sweep );

void b3GeneralInverse( float* out, float* m, int n, int lda );

B3_INLINE float b3Dot2( b3Vec2 v1, b3Vec2 v2 )
{
	return v1.x * v2.x + v1.y * v2.y;
}

B3_INLINE float b3Length2( b3Vec2 v )
{
	return sqrtf( b3Dot2( v, v ) );
}

B3_INLINE float b3LengthSquared2( b3Vec2 v )
{
	return b3Dot2( v, v );
}

B3_INLINE b3Vec2 b3MinVec2( b3Vec2 v1, b3Vec2 v2 )
{
	b3Vec2 v;
	v.x = b3MinFloat( v1.x, v2.x );
	v.y = b3MinFloat( v1.y, v2.y );
	return v;
}

B3_INLINE b3Vec2 b3MaxVec2( b3Vec2 v1, b3Vec2 v2 )
{
	b3Vec2 v;
	v.x = b3MaxFloat( v1.x, v2.x );
	v.y = b3MaxFloat( v1.y, v2.y );
	return v;
}

B3_INLINE void b3Store( float* dst, b3Vec3 src )
{
	dst[0] = src.x;
	dst[1] = src.y;
	dst[2] = src.z;
}

B3_INLINE b3Vec3 b3ClampLength( b3Vec3 v, float maxLength )
{
	float lengthSq = b3LengthSquared( v );
	if ( lengthSq <= maxLength * maxLength )
	{
		return v;
	}

	float length = sqrtf( lengthSq );
	return b3MulSV( maxLength / length, v );
}

B3_INLINE b3Quat b3QuatFromExponentialMap( b3Vec3 v )
{
	// Exponential map (Grassia)
	float threshold = 0.018581361f;

	float angle = b3Length( v );
	if ( angle < threshold )
	{
		// Taylor expansion
		b3Quat out;
		out.v = b3MulSV( 0.5f + angle * angle / 48.0f, v );
		out.s = b3Cos( 0.5f * angle );

		return out;
	}

	return b3MakeQuatFromAxisAngle( b3MulSV( 1.0f / angle, v ), angle );
}

/// Integrate rotation from angular velocity
/// @param q1 initial rotation
/// @param deltaRotation the angular displacement vector in radians (angular velocity multiplied by the time step)
B3_INLINE b3Quat b3IntegrateRotation( b3Quat q1, b3Vec3 deltaRotation )
{
	// https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
	b3Quat qd = { b3MulSV( 0.5f, deltaRotation ), 0.0f };
	qd = b3MulQuat( qd, q1 );
	b3Quat q2 = { b3Add( q1.v, qd.v ), qd.s + q1.s };
	q2 = b3NormalizeQuat( q2 );
	return q2;

	// return b3NormalizeQuat( b3MulQuat(b3QuatFromExponentialMap( deltaRotation ), q1) );
}

B3_INLINE float b3ScalarTripleProduct( b3Vec3 a, b3Vec3 b, b3Vec3 c )
{
	b3Vec3 d;
	d.x = b.y * c.z - b.z * c.y;
	d.y = b.z * c.x - b.x * c.z;
	d.z = b.x * c.y - b.y * c.x;
	return a.x * d.x + a.y * d.y + a.z * d.z;
}

// Get a value by index. Avoid undefined behavior of code like (&v.x)[2].
B3_INLINE float b3GetByIndex( b3Vec3 v, int index )
{
	B3_ASSERT( 0 <= index && index < 3 );
	float temp[3] = { v.x, v.y, v.z };
	return temp[index];
}

B3_INLINE int b3MinorAxis( b3Vec3 v )
{
	return v.x < v.y ? ( v.x < v.z ? 0 : 2 ) : ( v.y < v.z ? 1 : 2 );
}

B3_INLINE int b3MajorAxis( b3Vec3 v )
{
	return v.x < v.y ? ( v.y < v.z ? 2 : 1 ) : ( v.x < v.z ? 2 : 0 );
}

B3_INLINE float b3MinElement( b3Vec3 v )
{
	return b3MinFloat( v.x, b3MinFloat( v.y, v.z ) );
}

B3_INLINE float b3MaxElement( b3Vec3 v )
{
	return b3MaxFloat( v.x, b3MaxFloat( v.y, v.z ) );
}

B3_INLINE int b3MaxElementIndex( b3Vec3 v )
{
	return v.x < v.y ? ( v.y < v.z ? 2 : 1 ) : ( v.x < v.z ? 2 : 0 );
}

B3_INLINE b3Vec2 b3MulMV2( b3Matrix2 m, b3Vec2 a )
{
	b3Vec2 b = { m.cx.x * a.x + m.cy.x * a.y, m.cx.y * a.x + m.cy.y * a.y };
	return b;
}

B3_INLINE b3Matrix2 b3MulMM2( b3Matrix2 m1, b3Matrix2 m2 )
{
	b3Matrix2 out;
	out.cx = b3MulMV2( m1, m2.cx );
	out.cy = b3MulMV2( m1, m2.cy );
	return out;
}

B3_INLINE float b3Det2( b3Matrix2 m )
{
	return m.cx.x * m.cy.y - m.cx.y * m.cy.x;
}

B3_INLINE b3Matrix2 b3Invert2( b3Matrix2 m )
{
	float det = b3Det2( m );
	if ( b3AbsFloat( det ) > 1000.0f * FLT_MIN )
	{
		float invDet = 1.0f / det;
		return B3_LITERAL( b3Matrix2 ){
			{ invDet * m.cy.y, -invDet * m.cx.y },
			{ -invDet * m.cy.x, invDet * m.cx.x },
		};
	}

	return B3_LITERAL( b3Matrix2 ){ { 0.0f, 0.0f }, { 0.0f, 0.0f } };
}

// Assumes positive semi-definite
B3_INLINE b3Vec2 b3Solve2( b3Matrix2 m, b3Vec2 b )
{
	float det = b3Det2( m );
	if ( det > 1000.0f * FLT_MIN )
	{
		float invDet = 1.0f / det;
		return B3_LITERAL( b3Vec2 ){
			invDet * m.cy.y * b.x - invDet * m.cy.x * b.y,
			-invDet * m.cx.y * b.x + invDet * m.cx.x * b.y,
		};
	}

	return B3_LITERAL( b3Vec2 ){ 0.0f, 0.0f };
}

B3_INLINE b3Plane b3NormalizePlane( b3Plane plane )
{
	float invLength = 1.0f / b3Length( plane.normal );
	return B3_LITERAL( b3Plane ){ b3MulSV( invLength, plane.normal ), invLength * plane.offset };
}

// Negative if p is below the triangle v1-v2-v3
B3_INLINE float b3SignedVolume( b3Vec3 v1, b3Vec3 v2, b3Vec3 v3, b3Vec3 p )
{
	b3Vec3 e1 = b3Sub( v2, v1 );
	b3Vec3 e2 = b3Sub( v3, v1 );
	b3Vec3 n = b3Cross( e1, e2 );
	return b3Dot( n, b3Sub( p, v1 ) );
}

// todo eliminate this
B3_INLINE bool b3IsWithinSegments( const b3ClosestApproachResult* result )
{
	return ( 0.0f <= result->lambda1 && result->lambda1 <= 1.0f ) && ( 0.0f <= result->lambda2 && result->lambda2 <= 1.0f );
}

typedef struct
{
	b3Vec3 point;
	b3TriangleFeature feature;
} b3TrianglePoint;

b3TrianglePoint b3ClosestPointOnTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, b3Vec3 q );

float b3IntersectSegmentTriangle( b3Vec3 p, b3Vec3 q, b3Vec3 a, b3Vec3 b, b3Vec3 c );
float b3IntersectSegmentSphere( b3Vec3 p, b3Vec3 q, b3Vec3 c, float r );

typedef struct b3ShapeExtent
{
	float minExtent;
	float maxExtent;
} b3ShapeExtent;

b3MassData b3ComputeMassProperties( int triangleCount, const int* triangles, int vertexCount, const b3Vec3* vertices,
									float density );

bool b3IsValidMassData( const b3MassData* massData );

b3Matrix3 b3SphereInertia( float mass, float radius );
b3Matrix3 b3CylinderInertia( float mass, float radius, float height );
b3Matrix3 b3BoxInertia( float mass, b3Vec3 min, b3Vec3 max );

// Inertia helper (Io = Ic + Is and Ic = Io - Is)
b3Matrix3 b3Steiner( float mass, b3Vec3 origin );

B3_INLINE b3Matrix3 b3TransformInertia( b3Transform transform, b3Matrix3 centralInertia, float mass )
{
	b3Matrix3 rotationMatrix = b3MakeMatrixFromQuat( transform.q );
	b3Matrix3 inertia = b3MulMM( rotationMatrix, b3MulMM( centralInertia, b3Transpose( rotationMatrix ) ) );
	inertia = b3AddMM( inertia, b3Steiner( mass, transform.p ) );
	return inertia;
}

int b3GetProxySupport( const b3ShapeProxy* proxy, b3Vec3 axis );
int b3GetPointSupport( const b3Vec3* points, int count, b3Vec3 axis );

bool b3IntersectRayAndAABB( b3Vec3 lowerBound, b3Vec3 upperBound, b3Vec3 p1, b3Vec3 p2, float* minFraction, float* maxFraction );

#ifdef __cplusplus

B3_INLINE b3Vec2& operator+=( b3Vec2& a, b3Vec2 b )
{
	a.x += b.x;
	a.y += b.y;
	return a;
}

B3_INLINE b3Vec2& operator*=( b3Vec2& a, float s )
{
	a.x *= s;
	a.y *= s;
	return a;
}

B3_INLINE b3Vec2 operator-( b3Vec2 v )
{
	return { -v.x, -v.y };
}

B3_INLINE b3Vec2 operator+( b3Vec2 v1, b3Vec2 v2 )
{
	b3Vec2 out;
	out.x = v1.x + v2.x;
	out.y = v1.y + v2.y;

	return out;
}

B3_INLINE b3Vec2 operator-( b3Vec2 v1, b3Vec2 v2 )
{
	b3Vec2 out;
	out.x = v1.x - v2.x;
	out.y = v1.y - v2.y;

	return out;
}

B3_INLINE b3Vec2 operator*( float f, b3Vec2 v )
{
	b3Vec2 out;
	out.x = f * v.x;
	out.y = f * v.y;

	return out;
}

B3_INLINE b3Vec2 operator*( b3Vec2 v, float f )
{
	b3Vec2 out;
	out.x = v.x * f;
	out.y = v.y * f;

	return out;
}

B3_INLINE b3Matrix2 operator+( b3Matrix2 m1, b3Matrix2 m2 )
{
	b3Matrix2 out;
	out.cx = m1.cx + m2.cx;
	out.cy = m1.cy + m2.cy;

	return out;
}

B3_INLINE b3Matrix2 operator-( b3Matrix2 m1, b3Matrix2 m2 )
{
	b3Matrix2 out;
	out.cx = m1.cx - m2.cx;
	out.cy = m1.cy - m2.cy;

	return out;
}

B3_INLINE b3Matrix2 operator*( float f, b3Matrix2 m )
{
	b3Matrix2 out;
	out.cx = f * m.cx;
	out.cy = f * m.cy;

	return out;
}

B3_INLINE b3Matrix2 operator*( b3Matrix2 m, float f )
{
	b3Matrix2 out;
	out.cx = m.cx * f;
	out.cy = m.cy * f;

	return out;
}

#endif
