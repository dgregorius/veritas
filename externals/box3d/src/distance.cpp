// SPDX-FileCopyrightText: 2025 Dirk Gregorius
// SPDX-License-Identifier: MIT

#include "math_internal.h"

#include "box3d/collision.h"

#define B3_MAX_SIMPLEX_VERTICES 4

int b3GetProxySupport( const b3ShapeProxy* proxy, b3Vec3 axis )
{
	int count = proxy->count;
	const b3Vec3* points = proxy->points;

	B3_ASSERT( count > 0 );
	B3_ASSERT( points != nullptr );

	// We move the first vertex into the origin for improved precision.
	// This is necessary since we don't have shape transforms and
	// vertices can potentially be far away from the origin (large).
	b3Vec3 origin = points[0];
	int maxIndex = 0;
	float maxProjection = 0.0f;

	for ( int index = 1; index < count; ++index )
	{
		// We subtract the first vertex since we are shifting into the origin.
		float projection = b3Dot( axis, points[index] - origin );
		if ( projection > maxProjection )
		{
			maxIndex = index;
			maxProjection = projection;
		}
	}

	return maxIndex;
}

int b3GetPointSupport( const b3Vec3* points, int count, b3Vec3 axis )
{
	B3_ASSERT( count > 0 );
	B3_ASSERT( points != nullptr );

	// We move the first vertex into the origin for improved precision.
	// This is necessary since we don't have shape transforms and
	// vertices can potentially be far away from the origin (large).
	b3Vec3 origin = points[0];
	int maxIndex = 0;
	float maxProjection = 0.0f;

	for ( int index = 1; index < count; ++index )
	{
		// We subtract the first vertex since we are shifting into the origin.
		float projection = b3Dot( axis, points[index] - origin );
		if ( projection > maxProjection )
		{
			maxIndex = index;
			maxProjection = projection;
		}
	}

	return maxIndex;
}

// Similar to Real-time Collision Detection, p179.
// todo try https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
bool b3IntersectRayAndAABB( b3Vec3 lowerBound, b3Vec3 upperBound, b3Vec3 p1, b3Vec3 p2, float* minFraction,
							float* maxFraction )
{
	// Ray direction and length
	b3Vec3 d = b3Sub( p2, p1 );
	float rayLength = b3Length( d );

	// Handle degenerate ray
	if ( rayLength < FLT_EPSILON )
	{
		// Check if point is inside AABB
		if ( p1.x >= lowerBound.x && p1.x <= upperBound.x && p1.y >= lowerBound.y && p1.y <= upperBound.y &&
			 p1.z >= lowerBound.z && p1.z <= upperBound.z )
		{
			*minFraction = 0.0f;
			*maxFraction = 0.0f;
			return true;
		}
		else
		{
			return false;
		}
	}

	b3Vec3 rayDir = b3MulSV( 1.0f / rayLength, d );

	// Slab method for ray-AABB intersection
	float tMin = 0.0f;
	float tMax = rayLength;

	// Test each axis
	for ( int i = 0; i < 3; ++i )
	{
		float rayComponent = b3GetByIndex( rayDir, i );
		float rayStart = b3GetByIndex( p1, i );
		float boxMin = b3GetByIndex( lowerBound, i );
		float boxMax = b3GetByIndex( upperBound, i );

		if ( b3AbsFloat( rayComponent ) < FLT_EPSILON )
		{
			// Ray is parallel to slab, check if ray origin is within slab
			if ( rayStart < boxMin || rayStart > boxMax )
			{
				return false;
			}
		}
		else
		{
			// Compute intersection distances
			float t1 = ( boxMin - rayStart ) / rayComponent;
			float t2 = ( boxMax - rayStart ) / rayComponent;

			// Ensure t1 <= t2
			if ( t1 > t2 )
			{
				float temp = t1;
				t1 = t2;
				t2 = temp;
			}

			// Update intersection interval
			tMin = b3MaxFloat( tMin, t1 );
			tMax = b3MinFloat( tMax, t2 );

			// Check for no intersection
			if ( tMin > tMax )
			{
				return false;
			}
		}
	}

	// Check if intersection is behind ray start
	if ( tMax < 0.0f )
	{
		return false;
	}

	// Convert distances to fractions
	*minFraction = b3ClampFloat( tMin / rayLength, 0.0f, 1.0f );
	*maxFraction = b3ClampFloat( tMax / rayLength, 0.0f, 1.0f );

	return true;
}

#if 0

// From Real-time Collision Detection, p179.
bool b3IntersectRayAndAABB( b3Vec3 lowerBound, b3Vec3 upperBound, b3Vec3 p1, b3Vec3 p2, float* minFraction,
							 float* maxFraction )
{
	float tmin = 0.0f;
	float tmax = FLT_MAX;

	float lower[3] = { lowerBound.x, lowerBound.y, lowerBound.z };
	float upper[3] = { upperBound.x, upperBound.y, upperBound.z };
	float p[3] = { p1.x, p1.y, p1.z };
	float d[3] = { p2.x - p1.x, p2.y - p1.y, p2.z - p1.z };

	for (int i = 0; i < 3; ++i)
	{
		if ( b3AbsFloat(d[i]) < FLT_EPSILON )
		{
			// parallel
			if ( p[i] < lower[i] || upper[i] < p[i] )
			{
				return false;
			}
		}
		else
		{
			float invD = 1.0f / d[i];
			float t1 = ( lower[i] - p[i] ) * invD;
			float t2 = ( upper[i] - p[i] ) * invD;

			if ( t1 > t2 )
			{
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
			}

			// Push the min up
			if ( t1 > tmin )
			{
				tmin = t1;
			}

			// Pull the max down
			if (t2 < tmax)
			{
				tmax = t2;
			}

			if ( tmin > tmax )
			{
				return false;
			}
		}
	}

	*minFraction = tmin;
	*maxFraction = tmax;
	return true;
}
#endif

static void b3BarycentricCoordinates( float out[3], b3Vec3 a, b3Vec3 b )
{
	b3Vec3 ab = b - a;

	// Last element is divisor
	float divisor = b3Dot( ab, ab );

	out[0] = b3Dot( b, ab );
	out[1] = -b3Dot( a, ab );
	out[2] = divisor;
}

static void b3BarycentricCoordinates( float out[4], b3Vec3 a, b3Vec3 b, b3Vec3 c )
{
	b3Vec3 ab = b - a;
	b3Vec3 ac = c - a;

	b3Vec3 bXC = b3Cross( b, c );
	b3Vec3 cXA = b3Cross( c, a );
	b3Vec3 aXB = b3Cross( a, b );

	b3Vec3 abXAc = b3Cross( ab, ac );

	// Last element is divisor
	float divisor = b3Dot( abXAc, abXAc );

	out[0] = b3Dot( bXC, abXAc );
	out[1] = b3Dot( cXA, abXAc );
	out[2] = b3Dot( aXB, abXAc );
	out[3] = divisor;
}

static void b3BarycentricCoordinates( float out[5], b3Vec3 a, b3Vec3 b, b3Vec3 c, b3Vec3 d )
{
	b3Vec3 ab = b - a;
	b3Vec3 ac = c - a;
	b3Vec3 ad = d - a;

	// Last element is divisor (forced to be positive)
	float divisor = b3ScalarTripleProduct( ab, ac, ad );

	float sign = divisor < 0.0f ? -1.0f : 1.0f;
	out[0] = sign * b3ScalarTripleProduct( b, c, d );
	out[1] = sign * b3ScalarTripleProduct( a, d, c );
	out[2] = sign * b3ScalarTripleProduct( a, b, d );
	out[3] = sign * b3ScalarTripleProduct( a, c, b );
	out[4] = sign * divisor;
}

static float b3GetMetric( const b3Simplex* simplex )
{
	int count = simplex->count;
	B3_ASSERT( 1 <= count && count <= 4 );

	const b3SimplexVertex* vertices = simplex->vertices;

	switch ( count )
	{
		case 1:
		{
			return 0.0f;
		}

		case 2:
		{
			b3Vec3 a = vertices[0].w;
			b3Vec3 b = vertices[1].w;
			return b3Distance( a, b );
		}

		case 3:
		{
			b3Vec3 a = vertices[0].w;
			b3Vec3 b = vertices[1].w;
			b3Vec3 c = vertices[2].w;
			return b3Length( b3Cross( b - a, c - a ) ) / 2.0f;
		}

		case 4:
		{
			b3Vec3 a = vertices[0].w;
			b3Vec3 b = vertices[1].w;
			b3Vec3 c = vertices[2].w;
			b3Vec3 d = vertices[3].w;
			return b3ScalarTripleProduct( b - a, c - a, d - a ) / 6.0f;
		}

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return 0.0f;
}

#if 0
static b3Simplex b3MakeSimplexFromCache( const b3Transform& transform1, const b3ShapeProxy& proxy1, const b3Transform& transform2,
										 const b3ShapeProxy& proxy2, const b3SimplexCache* cache )
{
	B3_ASSERT( cache->count <= B3_MAX_SIMPLEX_VERTICES );

	b3Simplex simplex = {};

	// Copy data from cache
	b3SimplexVertex* vertices = simplex.vertices;

	simplex.count = cache->count;
	for ( int i = 0; i < cache->count; ++i )
	{
		int index1 = cache->indexA[i];
		int index2 = cache->indexB[i];

		B3_ASSERT( 0 <= index1 && index1 < proxy1.count );
		B3_ASSERT( 0 <= index2 && index2 < proxy2.count );

		b3Vec3 vertex1 = b3TransformPoint( transform1, proxy1.points[index1] );
		b3Vec3 vertex2 = b3TransformPoint( transform2, proxy2.points[index2] );

		vertices[i].indexA = index1;
		vertices[i].indexB = index2;
		vertices[i].wA = vertex1;
		vertices[i].wB = vertex2;
		vertices[i].w = vertex2 - vertex1;
		vertices[i].a = 0.0f;
	}

	// Compute the new simplex metric, if it is substantially
	// different than the old metric flush the simplex.
	if ( simplex.count > 0 )
	{
		float metric1 = cache->metric;
		float metric2 = b3GetMetric( &simplex );

		// todo the tetrahedron metric can be negative
		if ( 2.0f * metric1 < metric2 || metric2 < 0.5f * metric1 || metric2 < FLT_EPSILON )
		{
			// Flush the simplex
			simplex.count = 0;
		}
	}

	// If the cache is invalid or empty
	if ( simplex.count == 0 )
	{
		b3Vec3 vertex1 = b3TransformPoint( transform1, proxy1.points[0] );
		b3Vec3 vertex2 = b3TransformPoint( transform2, proxy2.points[0] );

		simplex.count = 1;
		simplex.vertices[0].indexA = 0;
		simplex.vertices[0].indexB = 0;
		simplex.vertices[0].wA = vertex1;
		simplex.vertices[0].wB = vertex2;
		simplex.vertices[0].w = vertex2 - vertex1;
		simplex.vertices[0].a = 0.0f;
	}

	return simplex;
}
#endif

static void b3WriteCache( b3SimplexCache* cache, const b3Simplex* simplex )
{
	int count = simplex->count;
	cache->metric = b3GetMetric( simplex );
	cache->count = uint16_t( count );
	for ( int index = 0; index < count; ++index )
	{
		cache->indexA[index] = uint8_t( simplex->vertices[index].indexA );
		cache->indexB[index] = uint8_t( simplex->vertices[index].indexB );
	}
}

#if 0
static bool b3Solve1( b3Simplex* simplex )
{
	// VR( A )
	B3_ASSERT( simplex->count == 1 );
	simplex->vertices[0].a = 1.0f;

	return true;
}
#endif

static bool b3Solve2( b3Simplex* simplex )
{
	b3SimplexVertex* vs = simplex->vertices;
	B3_ASSERT( simplex->count == 2 );

	// Vertex regions
	//float wAB[3];

#if 0
	b3BarycentricCoordinates( wAB, vs[0].w, vs[1].w );
#else
	b3Vec3 a = vs[0].w;
	b3Vec3 b = vs[1].w;
	b3Vec3 ab = b - a;

	// Last element is divisor
	float divisor = b3Dot( ab, ab );

	float u = b3Dot( b, ab );
	float v = -b3Dot( a, ab );
	//wAB[2] = divisor;
#endif

	// V( A )
	if ( v <= 0.0f )
	{
		// Reduce simplex
		simplex->count = 1;
		vs[0].a = 1.0f;

		return true;
	}

	// V( B )
	if ( u <= 0.0f )
	{
		// Reduce simplex
		simplex->count = 1;
		vs[0] = vs[1];
		vs[0].a = 1.0f;

		return true;
	}

	// Edge region
	if ( divisor <= 0.0f )
	{
		return false;
	}

	// VR( AB )
	float denominator = 1.0f / divisor;
	vs[0].a = denominator * u;
	vs[1].a = denominator * v;

	return true;
}

static bool b3Solve3( b3Simplex* simplex )
{
	b3SimplexVertex* vs = simplex->vertices;
	B3_ASSERT( simplex->count == 3 );

	// Get simplex (be aware of aliasing here!)
	b3SimplexVertex v1 = vs[0];
	b3SimplexVertex v2 = vs[1];
	b3SimplexVertex v3 = vs[2];

	// Vertex regions
	float wAB[3], wBC[3], wCA[3];
	b3BarycentricCoordinates( wAB, v1.w, v2.w );
	b3BarycentricCoordinates( wBC, v2.w, v3.w );
	b3BarycentricCoordinates( wCA, v3.w, v1.w );

	// VR( A )
	if ( wAB[1] <= 0.0f && wCA[0] <= 0.0f )
	{
		// Reduce simplex
		simplex->count = 1;
		vs[0] = v1;
		vs[0].a = 1.0f;

		return true;
	}

	// VR( B )
	if ( wBC[1] <= 0.0f && wAB[0] <= 0.0f )
	{
		// Reduce simplex
		simplex->count = 1;
		vs[0] = v2;
		vs[0].a = 1.0f;

		return true;
	}

	// VR( C )
	if ( wCA[1] <= 0.0f && wBC[0] <= 0.0f )
	{
		// Reduce simplex
		simplex->count = 1;
		vs[0] = v3;
		vs[0].a = 1.0f;

		return true;
	}

	// Edge regions
	float wABC[4];
	b3BarycentricCoordinates( wABC, v1.w, v2.w, v3.w );

	// VR( AB )
	if ( wABC[2] <= 0.0f && wAB[0] > 0.0f && wAB[1] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 2;
		vs[0] = v1;
		vs[1] = v2;

		// Normalize
		float divisor = wAB[2];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wAB[0] / divisor;
		vs[1].a = wAB[1] / divisor;

		return true;
	}

	// VR( BC )
	if ( wABC[0] <= 0.0f && wBC[0] > 0.0f && wBC[1] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 2;
		vs[0] = v2;
		vs[1] = v3;

		// Normalize
		float divisor = wBC[2];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wBC[0] / divisor;
		vs[1].a = wBC[1] / divisor;

		return true;
	}

	// VR( CA )
	if ( wABC[1] <= 0.0f && wCA[0] > 0.0f && wCA[1] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 2;
		vs[0] = v3;
		vs[1] = v1;

		// Normalize
		float divisor = wCA[2];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wCA[0] / divisor;
		vs[1].a = wCA[1] / divisor;

		return true;
	}

	// Face region
	float divisor = wABC[3];
	if ( divisor <= 0.0f )
	{
		return false;
	}

	// VR( ABC )
	vs[0].a = wABC[0] / divisor;
	vs[1].a = wABC[1] / divisor;
	vs[2].a = wABC[2] / divisor;

	return true;
}

static bool b3Solve4( b3Simplex* simplex )
{
	b3SimplexVertex* vs = simplex->vertices;

	// Get simplex (be aware of aliasing here!)
	B3_ASSERT( simplex->count == 4 );
	b3SimplexVertex vertexA = vs[0];
	b3SimplexVertex vertexB = vs[1];
	b3SimplexVertex vertexC = vs[2];
	b3SimplexVertex vertexD = vs[3];

	// Vertex region
	float wAB[3], wAC[3], wAD[3], wBC[3], wCD[3], wDB[3];
	b3BarycentricCoordinates( wAB, vertexA.w, vertexB.w );
	b3BarycentricCoordinates( wAC, vertexA.w, vertexC.w );
	b3BarycentricCoordinates( wAD, vertexA.w, vertexD.w );
	b3BarycentricCoordinates( wBC, vertexB.w, vertexC.w );
	b3BarycentricCoordinates( wCD, vertexC.w, vertexD.w );
	b3BarycentricCoordinates( wDB, vertexD.w, vertexB.w );

	// VR( A )
	if ( wAB[1] <= 0.0f && wAC[1] <= 0.0f && wAD[1] <= 0.0f )
	{
		// Reduce simplex
		simplex->count = 1;
		vs[0] = vertexA;

		vs[0].a = 1.0f;

		return true;
	}

	// VR( B )
	if ( wAB[0] <= 0.0f && wDB[0] <= 0.0f && wBC[1] <= 0.0f )
	{
		// Reduce simplex
		simplex->count = 1;
		vs[0] = vertexB;

		vs[0].a = 1.0f;

		return true;
	}

	// VR( C )
	if ( wAC[0] <= 0.0f && wBC[0] <= 0.0f && wCD[1] <= 0.0f )
	{
		// Reduce simplex
		simplex->count = 1;
		vs[0] = vertexC;

		vs[0].a = 1.0f;

		return true;
	}

	// VR( D )
	if ( wAD[0] <= 0.0f && wCD[0] <= 0.0f && wDB[1] <= 0.0f )
	{
		// Reduce simplex
		simplex->count = 1;
		vs[0] = vertexD;

		vs[0].a = 1.0f;

		return true;
	}

	// Edge region
	float wACB[4], wABD[4], wADC[4], wBCD[4];
	b3BarycentricCoordinates( wACB, vertexA.w, vertexC.w, vertexB.w );
	b3BarycentricCoordinates( wABD, vertexA.w, vertexB.w, vertexD.w );
	b3BarycentricCoordinates( wADC, vertexA.w, vertexD.w, vertexC.w );
	b3BarycentricCoordinates( wBCD, vertexB.w, vertexC.w, vertexD.w );

	// VR( AB )
	if ( wABD[2] <= 0.0f && wACB[1] <= 0.0f && wAB[0] > 0.0f && wAB[1] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 2;
		vs[0] = vertexA;
		vs[1] = vertexB;

		// Normalize
		float divisor = wAB[2];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wAB[0] / divisor;
		vs[1].a = wAB[1] / divisor;

		return true;
	}

	// VR( AC )
	if ( wACB[2] <= 0.0f && wADC[1] <= 0.0f && wAC[0] > 0.0f && wAC[1] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 2;
		vs[0] = vertexA;
		vs[1] = vertexC;

		// Normalize
		float divisor = wAC[2];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wAC[0] / divisor;
		vs[1].a = wAC[1] / divisor;

		return true;
	}

	// VR( AD )
	if ( wADC[2] <= 0.0f && wABD[1] <= 0.0f && wAD[0] > 0.0f && wAD[1] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 2;
		vs[0] = vertexA;
		vs[1] = vertexD;

		// Normalize
		float divisor = wAD[2];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wAD[0] / divisor;
		vs[1].a = wAD[1] / divisor;

		return true;
	}

	// VR( BC )
	if ( wACB[0] <= 0.0f && wBCD[2] <= 0.0f && wBC[0] > 0.0f && wBC[1] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 2;
		vs[0] = vertexB;
		vs[1] = vertexC;

		// Normalize
		float divisor = wBC[2];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wBC[0] / divisor;
		vs[1].a = wBC[1] / divisor;

		return true;
	}

	// VR( CD )
	if ( wADC[0] <= 0.0f && wBCD[0] <= 0.0f && wCD[0] > 0.0f && wCD[1] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 2;
		vs[0] = vertexC;
		vs[1] = vertexD;

		// Normalize
		float divisor = wCD[2];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wCD[0] / divisor;
		vs[1].a = wCD[1] / divisor;

		return true;
	}

	// VR( DB )
	if ( wABD[0] <= 0.0f && wBCD[1] <= 0.0f && wDB[0] > 0.0f && wDB[1] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 2;
		vs[0] = vertexD;
		vs[1] = vertexB;

		// Normalize
		float divisor = wDB[2];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wDB[0] / divisor;
		vs[1].a = wDB[1] / divisor;

		return true;
	}

	// Face regions
	float wABCD[5];
	b3BarycentricCoordinates( wABCD, vertexA.w, vertexB.w, vertexC.w, vertexD.w );

	// VR( ACB )
	if ( wABCD[3] < 0.0f && wACB[0] > 0.0f && wACB[1] > 0.0f && wACB[2] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 3;
		vs[0] = vertexA;
		vs[1] = vertexC;
		vs[2] = vertexB;

		// Normalize
		float divisor = wACB[3];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wACB[0] / divisor;
		vs[1].a = wACB[1] / divisor;
		vs[2].a = wACB[2] / divisor;

		return true;
	}

	// VR( ABD )
	if ( wABCD[2] < 0.0f && wABD[0] > 0.0f && wABD[1] > 0.0f && wABD[2] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 3;
		vs[0] = vertexA;
		vs[1] = vertexB;
		vs[2] = vertexD;

		// Normalize
		float divisor = wABD[3];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wABD[0] / divisor;
		vs[1].a = wABD[1] / divisor;
		vs[2].a = wABD[2] / divisor;

		return true;
	}

	// VR( ADC )
	if ( wABCD[1] < 0.0f && wADC[0] > 0.0f && wADC[1] > 0.0f && wADC[2] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 3;
		vs[0] = vertexA;
		vs[1] = vertexD;
		vs[2] = vertexC;

		// Normalize
		float divisor = wADC[3];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wADC[0] / divisor;
		vs[1].a = wADC[1] / divisor;
		vs[2].a = wADC[2] / divisor;

		return true;
	}

	// VR( BCD )
	if ( wABCD[0] < 0.0f && wBCD[0] > 0.0f && wBCD[1] > 0.0f && wBCD[2] > 0.0f )
	{
		// Reduce simplex
		simplex->count = 3;
		vs[0] = vertexB;
		vs[1] = vertexC;
		vs[2] = vertexD;

		// Normalize
		float divisor = wBCD[3];
		if ( divisor <= 0.0f )
		{
			return false;
		}

		vs[0].a = wBCD[0] / divisor;
		vs[1].a = wBCD[1] / divisor;
		vs[2].a = wBCD[2] / divisor;

		return true;
	}

	// *** Inside tetrahedron ***
	float divisor = wABCD[4];
	if ( divisor <= 0.0f )
	{
		return false;
	}

	// VR( ABCD )
	vs[0].a = wABCD[0] / divisor;
	vs[1].a = wABCD[1] / divisor;
	vs[2].a = wABCD[2] / divisor;
	vs[3].a = wABCD[3] / divisor;

	return true;
}

#if 0
static bool b3SolveSimplex( b3Simplex* simplex )
{
	int count = simplex->count;

	B3_ASSERT( 1 <= count && count <= 4 );

	//// Save the current simplex
	// mCache.VertexCount = mVertexCount;
	// for ( int i = 0; i < mVertexCount; ++i )
	//{
	//	mCache.Vertices1[i] = uint8_t( mVertices[i].Index1 );
	//	mCache.Vertices2[i] = uint8_t( mVertices[i].Index2 );
	// }

	switch ( count )
	{
		case 1:
			return b3Solve1( simplex );

		case 2:
			return b3Solve2( simplex );

		case 3:
			return b3Solve3( simplex );

		case 4:
			return b3Solve4( simplex );

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return false;
}

static b3Vec3 b3GetClosestPoint( const b3Simplex* simplex )
{
	int count = simplex->count;
	B3_ASSERT( 1 <= count && count <= 4 );

	const b3SimplexVertex* vs = simplex->vertices;

	switch ( count )
	{
		case 1:
			return simplex->vertices[0].w;

		case 2:
			return vs[0].a * vs[0].w + vs[1].a * vs[1].w;

		case 3:
			return vs[0].a * vs[0].w + vs[1].a * vs[1].w + vs[2].a * vs[2].w;

		case 4:
			return vs[0].a * vs[0].w + vs[1].a * vs[1].w + vs[2].a * vs[2].w + vs[3].a * vs[3].w;

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return b3Vec3_zero;
}

static b3Vec3 b3GetSearchDirection( const b3Simplex* simplex )
{
	const b3SimplexVertex* vs = simplex->vertices;
	int count = simplex->count;
	B3_ASSERT( 1 <= count && count <= 4 );

	switch ( count )
	{
		case 1:
		{
			// v = -A
			b3Vec3 a = vs[0].w;

			return -a;
		}

		case 2:
		{
			// v = (AB x AO) x AB
			b3Vec3 a = vs[0].w;
			b3Vec3 b = vs[1].w;

			b3Vec3 ab = b - a;

			return b3Cross( b3Cross( ab, -a ), ab );
		}

		case 3:
		{
			// v = AB x AC or v = AC x AB
			b3Vec3 a = vs[0].w;
			b3Vec3 b = vs[1].w;
			b3Vec3 c = vs[2].w;

			b3Vec3 ab = b - a;
			b3Vec3 ac = c - a;

			b3Vec3 n = b3Cross( ab, ac );

			return b3Dot( n, a ) < 0.0f ? n : -n;
		}

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return b3Vec3_zero;
}

static bool b3AddVertex( b3Simplex* simplex, int indexA, b3Vec3 vertexA, int indexB, b3Vec3 vertexB )
{
	b3SimplexVertex* vs = simplex->vertices;
	int count = simplex->count;

	// Check for duplicate support points. This is the main termination criteria.
	for ( int i = 0; i < count; ++i )
	{
		if ( vs[i].indexA == indexA && vs[i].indexB == indexB )
		{
			return false;
		}
	}

	vs[count].indexA = indexA;
	vs[count].indexB = indexB;
	vs[count].wA = vertexA;
	vs[count].wB = vertexB;
	vs[count].w = vertexB - vertexA;
	simplex->count += 1;

	return true;
}
#endif

static void b3ComputeWitnessPoints( const b3Simplex* simplex, b3Vec3* vertexA, b3Vec3* vertexB )
{
	const b3SimplexVertex* vs = simplex->vertices;
	int count = simplex->count;
	B3_ASSERT( 1 <= count && count <= 4 );

	switch ( count )
	{
		case 1:
			*vertexA = vs[0].wA;
			*vertexB = vs[0].wB;
			break;

		case 2:
			*vertexA = vs[0].a * vs[0].wA + vs[1].a * vs[1].wA;
			*vertexB = vs[0].a * vs[0].wB + vs[1].a * vs[1].wB;
			break;

		case 3:
			*vertexA = vs[0].a * vs[0].wA + vs[1].a * vs[1].wA + vs[2].a * vs[2].wA;
			*vertexB = vs[0].a * vs[0].wB + vs[1].a * vs[1].wB + vs[2].a * vs[2].wB;
			break;

		case 4:
			// Force identical points and *zero* distance
			*vertexA = *vertexB = vs[0].a * vs[0].wA + vs[1].a * vs[1].wA + vs[2].a * vs[2].wA + vs[3].a * vs[3].wA;
			break;

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}
}

// todo_erin remove old code after more testing
b3DistanceOutput b3ShapeDistance( const b3DistanceInput* input, b3SimplexCache* cache, b3Simplex* simplexes, int simplexCapacity )
{
	// Build initial simplex
	b3Transform transformA = input->transformA;
	b3Transform transformB = input->transformB;

	// Work in relative space
	b3Transform xf = b3InvMulTransforms( transformA, transformB );

	// Use matrices for faster math
	b3Matrix3 m = b3MakeMatrixFromQuat( xf.q );
	b3Matrix3 mt = b3Transpose( m );
	b3Matrix3 mA = b3MakeMatrixFromQuat( transformA.q );

	const b3ShapeProxy* proxyA = &input->proxyA;
	const b3ShapeProxy* proxyB = &input->proxyB;

#if 0
	b3Simplex simplex = b3MakeSimplexFromCache( input->proxyA, input->proxyB, m, xf.p, cache );
#else
	// Compute initial simplex from cache
	B3_ASSERT( cache->count <= B3_MAX_SIMPLEX_VERTICES );

	b3Simplex simplex = {};
	b3SimplexVertex* vs = simplex.vertices;

	simplex.count = cache->count;
	for ( int i = 0; i < cache->count; ++i )
	{
		int index1 = cache->indexA[i];
		int index2 = cache->indexB[i];

		B3_ASSERT( 0 <= index1 && index1 < proxyA->count );
		B3_ASSERT( 0 <= index2 && index2 < proxyB->count );

		b3Vec3 vertex1 = proxyA->points[index1];
		b3Vec3 vertex2 = b3Add( b3MulMV( m, proxyB->points[index2] ), xf.p );

		vs[i].indexA = index1;
		vs[i].indexB = index2;
		vs[i].wA = vertex1;
		vs[i].wB = vertex2;
		vs[i].w = vertex2 - vertex1;
		vs[i].a = 0.0f;
	}

	// Compute the new simplex metric, if it is substantially
	// different than the old metric flush the simplex.
	if ( simplex.count > 0 )
	{
		float metric1 = cache->metric;
		float metric2 = b3GetMetric( &simplex );

		// todo the tetrahedron metric can be negative
		if ( 2.0f * metric1 < metric2 || metric2 < 0.5f * metric1 || metric2 < FLT_EPSILON )
		{
			// Flush the simplex
			simplex.count = 0;
		}
	}

	// If the cache is invalid or empty
	if ( simplex.count == 0 )
	{
		b3Vec3 vertex1 = proxyA->points[0];
		b3Vec3 vertex2 = b3Add( b3MulMV( m, proxyB->points[0] ), xf.p );

		simplex.count = 1;
		simplex.vertices[0].indexA = 0;
		simplex.vertices[0].indexB = 0;
		simplex.vertices[0].wA = vertex1;
		simplex.vertices[0].wB = vertex2;
		simplex.vertices[0].w = vertex2 - vertex1;
		simplex.vertices[0].a = 0.0f;
	}
#endif

	b3Simplex backup = {};

	int simplexIndex = 0;
	if ( simplexes != nullptr && simplexIndex < simplexCapacity )
	{
		simplexes[simplexIndex] = simplex;
		simplexIndex += 1;
	}

	b3DistanceOutput distanceOutput = {};

	// Keep track of squared distance
	float distanceSq = FLT_MAX;

	b3Vec3 normal = b3Vec3_zero;

	// Run GJK
	constexpr int maxIterations = 32;
	int iteration = 0;
	for ( ; iteration < maxIterations; ++iteration )
	{
#if 0
		if ( b3SolveSimplex( &simplex ) == false )
		{
			// No progress - reconstruct last simplex
			B3_ASSERT( backup.count != 0 );
			simplex = backup;
			break;
		}
#else
		// Solve simplex
		bool solved = false;
		switch ( simplex.count )
		{
			case 1:
				simplex.vertices[0].a = 1.0f;
				solved = true;
				break;

			case 2:
				solved = b3Solve2( &simplex );
				break;

			case 3:
				solved = b3Solve3( &simplex );
				break;

			case 4:
				solved = b3Solve4( &simplex );
				break;

			default:
				B3_ASSERT( !"Should never get here!" );
				break;
		}

		if ( solved == false )
		{
			// No progress - reconstruct last simplex
			B3_ASSERT( backup.count != 0 );
			simplex = backup;
			break;
		}
#endif
		if ( simplexes != nullptr && simplexIndex < simplexCapacity )
		{
			simplexes[simplexIndex] = simplex;
			simplexIndex += 1;
			distanceOutput.iterations = iteration;
			distanceOutput.simplexCount = simplexIndex;
		}

		if ( simplex.count == B3_MAX_SIMPLEX_VERTICES )
		{
			// Overlap
			b3Vec3 localPointA, localPointB;
			b3ComputeWitnessPoints(&simplex, & localPointA, &localPointB );
			distanceOutput.pointA = b3Add( b3MulMV( mA, localPointA ), transformA.p );
			distanceOutput.pointB = b3Add( b3MulMV( mA, localPointB ), transformA.p );
			return distanceOutput;
		}

		// Assure distance progression
		float oldDistanceSq = distanceSq;
#if 0
		b3Vec3 closestPoint = b3GetClosestPoint( &simplex );
#else
		// Compute closest point
		b3Vec3 closestPoint = {};

		switch ( simplex.count )
		{
			case 1:
				closestPoint = vs[0].w;
				break;

			case 2:
				closestPoint = vs[0].a * vs[0].w + vs[1].a * vs[1].w;
				break;

			case 3:
				closestPoint = vs[0].a * vs[0].w + vs[1].a * vs[1].w + vs[2].a * vs[2].w;
				break;

			case 4:
				closestPoint = vs[0].a * vs[0].w + vs[1].a * vs[1].w + vs[2].a * vs[2].w + vs[3].a * vs[3].w;
				break;

			default:
				B3_ASSERT( !"Should never get here!" );
				break;
		}
#endif

		distanceSq = b3Dot( closestPoint, closestPoint );

		if ( distanceSq >= oldDistanceSq )
		{
			// No progress - reconstruct last simplex
			B3_ASSERT( backup.count != 0 );
			simplex = backup;
			break;
		}

		// Build new tentative support point
#if 0
		b3Vec3 searchDirection = b3GetSearchDirection( &simplex );
#else
		b3Vec3 searchDirection = {};

		switch ( simplex.count )
		{
			case 1:
			{
				// v = -A
				searchDirection = b3Neg( vs[0].w );
			}
			break;

			case 2:
			{
				// v = (AB x AO) x AB
				b3Vec3 a = vs[0].w;
				b3Vec3 b = vs[1].w;

				b3Vec3 ab = b - a;

				searchDirection = b3Cross( b3Cross( ab, -a ), ab );
			}
			break;

			case 3:
			{
				// v = AB x AC or v = AC x AB
				b3Vec3 a = vs[0].w;
				b3Vec3 b = vs[1].w;
				b3Vec3 c = vs[2].w;

				b3Vec3 ab = b - a;
				b3Vec3 ac = c - a;

				b3Vec3 n = b3Cross( ab, ac );

				searchDirection = b3Dot( n, a ) < 0.0f ? n : -n;
			}
			break;

			default:
				B3_ASSERT( !"Should never get here!" );
				break;
		}
#endif

		if ( b3LengthSquared( searchDirection ) < FLT_EPSILON )
		{
			// The origin is probably contained by a line segment or triangle.
			// Thus the shapes are overlapped.
			b3Vec3 localPointA, localPointB;
			b3ComputeWitnessPoints( &simplex, &localPointA, &localPointB );
			distanceOutput.pointA = b3Add( b3MulMV( mA, localPointA ), transformA.p );
			distanceOutput.pointB = b3Add( b3MulMV( mA, localPointB ), transformA.p );
			return distanceOutput;
		}

		normal = -searchDirection;

		// Get new support points
		b3Vec3 searchDirection1 = searchDirection;
		int indexA = b3GetProxySupport( &input->proxyA, -searchDirection1 );
		b3Vec3 supportA = input->proxyA.points[indexA];
		b3Vec3 searchDirection2 = b3MulMV( mt, searchDirection );
		int indexB = b3GetProxySupport( &input->proxyB, searchDirection2 );
		b3Vec3 supportB = b3Add( b3MulMV( m, input->proxyB.points[indexB] ), xf.p );

		// Save current simplex and add new vertex - this can fail if we detect cycling
		backup = simplex;

#if 0
		if ( !b3AddVertex( &simplex, index1, support1, index2, support2 ) )
		{
			break;
		}
#else
		// b3SimplexVertex* vs = simplex.vertices;

		// Check for duplicate support points. This is the main termination criteria.
		bool duplicate = false;
		for ( int i = 0; i < simplex.count; ++i )
		{
			if ( vs[i].indexA == indexA && vs[i].indexB == indexB )
			{
				duplicate = true;
				break;
			}
		}

		if ( duplicate )
		{
			break;
		}

		vs[simplex.count].indexA = indexA;
		vs[simplex.count].indexB = indexB;
		vs[simplex.count].wA = supportA;
		vs[simplex.count].wB = supportB;
		vs[simplex.count].w = supportB - supportA;
		simplex.count += 1;
#endif
	}

	normal = b3Normalize( normal );
	if ( b3IsNormalized( normal ) == false )
	{
		// Treat as overlap
		return distanceOutput;
	}

	// Build witness points and safe cache
	b3Vec3 localPointA, localPointB;
	b3ComputeWitnessPoints( &simplex, &localPointA, &localPointB );
	b3WriteCache( cache, &simplex );

	// Convert results from frame A into world space
	normal = b3MulMV( mA, normal );
	distanceOutput.pointA = b3Add( b3MulMV( mA, localPointA ), transformA.p );
	distanceOutput.pointB = b3Add( b3MulMV( mA, localPointB ), transformA.p );
	distanceOutput.distance = b3Distance( localPointA, localPointB );
	distanceOutput.normal = normal;
	distanceOutput.iterations = iteration;
	distanceOutput.simplexCount = simplexIndex;

	// Apply radii if requested
	if ( input->useRadii )
	{
		float rA = input->proxyA.radius;
		float rB = input->proxyB.radius;
		distanceOutput.distance = b3MaxFloat( 0.0f, distanceOutput.distance - rA - rB );

		// Keep closest points on perimeter even if overlapped, this way the points move smoothly.
		distanceOutput.pointA = distanceOutput.pointA + rA * normal;
		distanceOutput.pointB = distanceOutput.pointB - rB * normal;
	}

	return distanceOutput;
}

#if 0
b3GJKResult b3GJK( const b3GJKProxy& proxy1, const b3GJKProxy& proxy2, b3Vec3 translation, b3GJKSimplexCache& cache,
				   int maxIterations )
{
	// Build initial simplex
	b3GJKSimplex simplex, backup;
	simplex.ReadCache( proxy1, proxy2, translation, cache );

	int simplexIndex = 0;

	// Keep track of squared distance
	float distanceSq = FLT_MAX;

	b3Vec3 normal = b3Vec3_zero;

	// Run GJK
	int iteration = 0;
	for ( ; iteration < maxIterations; ++iteration )
	{
		// Solve simplex and check for overlap
		if ( !simplex.Solve() )
		{
			// No progress - reconstruct last simplex
			B3_ASSERT( !backup.IsEmpty() );
			simplex = backup;

			break;
		}

		if ( simplex.GetVertexCount() == B3_MAX_SIMPLEX_VERTICES )
		{
			// Overlap!
			break;
		}

		// Assure distance progression
		float oldDistanceSq = distanceSq;
		b3Vec3 closestPoint = simplex.GetClosestPoint();
		distanceSq = b3Dot( closestPoint, closestPoint );

		if ( distanceSq >= oldDistanceSq )
		{
			// No progress - reconstruct last simplex
			B3_ASSERT( !backup.IsEmpty() );
			simplex = backup;

			break;
		}

		// Build new tentative support point
		b3Vec3 searchDirection = simplex.GetSearchDirection();
		if ( b3LengthSquared( searchDirection ) < 1000.0f * FLT_MIN )
		{
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
		}

		normal = -searchDirection;

		// Get new support points
		b3Vec3 searchDirection1 = searchDirection;
		int index1 = proxy1.GetSupport( -searchDirection1 );
		b3Vec3 support1 = proxy1.mVertexBuffer[index1];
		b3Vec3 searchDirection2 = searchDirection;
		int index2 = proxy2.GetSupport( searchDirection2 );
		b3Vec3 support2 = proxy2.mVertexBuffer[index2] + translation;

		// Check for duplicate support points. This is the main termination criteria.
		int vertexCount = simplex.mCache.VertexCount;
		for ( int i = 0; i < vertexCount; ++i )
		{
			if ( simplex.mCache.Vertices1[i] == index1 && simplex.mCache.Vertices2[i] == index2 )
			{
				goto DuplicateFound;
			}
		}

		// Save current simplex and add new vertex - this can fail if we detect cycling
		backup = simplex;

		b3GJKSimplexVertex* vertex = simplex.mVertices + simplex.mVertexCount;
		vertex->Index1 = index1;
		vertex->Index2 = index2;
		vertex->Position1 = support1;
		vertex->Position2 = support2;
		vertex->Position = support2 - support1;
		simplex.mVertexCount += 1;
	}

DuplicateFound:

	// Build witness points and safe cache
	b3Vec3 point1, point2;
	simplex.BuildWitnessPoints( point1, point2 );
	simplex.WriteCache( cache );

	b3GJKResult result;
	result.Distance = b3Distance( point1, point2 );
	result.Normal = b3Normalize( normal );
	result.Point1 = point1;
	result.Point2 = point2;
	result.IterationCount = iteration;
	result.SimplexCount = simplexIndex;

	// todo I think there are cases where the normal is zero at this point
	// B3_ASSERT(b3Dot(Normal, Point2 - Point1) >= 0.0f);

	return result;
}
#endif
