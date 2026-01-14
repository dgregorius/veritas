// SPDX-FileCopyrightText: 2025 Dirk Gregorius and Erin Catto
// SPDX-License-Identifier: MIT

#include "constants.h"
#include "math_internal.h"

#include "box3d/collision.h"

// Separation function:
// f(t) = (c2 + t * dp2 - c1 - t * dp1 ) * n

// Root finding : f(t) - target = 0
// (c2 + t * dp2 - c1 - t * dp1 ) * n - target = 0
// (c2 - c1) * n + t * (dp2 - dp1) * n - target = 0
// t = [target - (c2 - c1) * n] / [(dp2 - dp1) * n]
// t = (target - d) / [(dp2 - dp1) * n]

b3CastOutput b3ShapeCast( const b3ShapeCastPairInput* input )
{
	// Compute tolerance
	float linearSlop = B3_LINEAR_SLOP;
	float totalRadius = input->proxyA.radius + input->proxyB.radius;
	float target = b3MaxFloat( linearSlop, totalRadius - linearSlop );
	float tolerance = 0.25f * linearSlop;

	B3_ASSERT( target > tolerance );

	// Prepare input for distance query
	b3SimplexCache cache = {};

	float alpha = 0.0f;

	b3DistanceInput distanceInput = {};
	distanceInput.proxyA = input->proxyA;
	distanceInput.proxyB = input->proxyB;
	distanceInput.transformA = input->transformA;
	distanceInput.transformB = input->transformB;
	distanceInput.useRadii = false;

	b3Vec3 delta2 = input->translationB;
	b3DistanceOutput distanceOutput = {};
	b3CastOutput output = {};
	output.triangleIndex = B3_NULL_INDEX;

	int iteration = 0;
	constexpr int maxIterations = 20;

	for ( ; iteration < maxIterations; ++iteration )
	{
		output.iterations += 1;

		distanceOutput = b3ShapeDistance( &distanceInput, &cache, nullptr, 0 );

		if ( distanceOutput.distance < target + tolerance )
		{
			if ( iteration == 0 )
			{
				if ( input->canEncroach && distanceOutput.distance > 2.0f * linearSlop )
				{
					target = distanceOutput.distance - linearSlop;
				}
				else
				{
					// Initial overlap
					output.hit = true;

					// Compute a common point
					b3Vec3 c1 = b3MulAdd( distanceOutput.pointA, input->proxyA.radius, distanceOutput.normal );
					b3Vec3 c2 = b3MulAdd( distanceOutput.pointB, -input->proxyB.radius, distanceOutput.normal );
					output.point = b3Lerp( c1, c2, 0.5f );
					return output;
				}
			}
			else
			{
				// Logging for bad input data
				if ( distanceOutput.distance > 0.0f && b3IsNormalized( distanceOutput.normal ) == false )
				{
					for (int i = 0; i < input->proxyA.count; ++i)
					{
						b3Vec3 p = input->proxyA.points[i];
						b3Log( "pointA[%d] = {%.9f, %.9f, %.9f}", i, p.x, p.y, p.z );
					}
					b3Log( "radiusA = %.9f", input->proxyA.radius );

					for (int i = 0; i < input->proxyB.count; ++i)
					{
						b3Vec3 p = input->proxyB.points[i];
						b3Log( "pointB[%d] = {%.9f, %.9f, %.9f}", i, p.x, p.y, p.z );
					}
					b3Log( "radiusB = %.9f", input->proxyB.radius );

					{
						b3Transform xfA = input->transformA;
						b3Log( "xfA = {{%.9f, %.9f, %.9f}, {{%.9f, %.9f, %.9f}, %.9f}", xfA.p.x, xfA.p.y, xfA.p.z, xfA.q.v.x,
							   xfA.q.v.y, xfA.q.v.z, xfA.q.s );
					}
					{
						b3Transform xfB = input->transformB;
						b3Log( "xfB = {{%.9f, %.9f, %.9f}, {{%.9f, %.9f, %.9f}, %.9f}", xfB.p.x, xfB.p.y, xfB.p.z, xfB.q.v.x,
							   xfB.q.v.y, xfB.q.v.z, xfB.q.s );
					}

					{
						b3Vec3 t = input->translationB;
						b3Log( "t = {%.9f, %.9f, %.9f}", t.x, t.y, t.z);
					}

					b3Log( "maxFraction = %.9f, canEncroach = %d", input->maxFraction, input->canEncroach );

					// Numerical problem. Likely extreme input.
					return output;
				}

				// Hitting this assert implies that the algorithm brought the shapes too close.
				// B3_ASSERT( distanceOutput.distance > 0.0f && b3IsNormalized( distanceOutput.normal ) );

				output.fraction = alpha;
				output.point = distanceOutput.pointA + input->proxyA.radius * distanceOutput.normal;
				output.normal = distanceOutput.normal;
				output.hit = true;
				return output;
			}
		}

		B3_ASSERT( distanceOutput.distance > 0.0f );
		B3_ASSERT( b3IsNormalized( distanceOutput.normal ) );

		// Check if shapes are approaching each other
		float denominator = b3Dot( delta2, distanceOutput.normal );
		if ( denominator >= 0.0f )
		{
			// Miss
			return output;
		}

		// Advance sweep
		alpha += ( target - distanceOutput.distance ) / denominator;
		if ( alpha >= input->maxFraction )
		{
			// Success!
			return output;
		}

		distanceInput.transformB.p = input->transformB.p + alpha * delta2;
	}

	// Failure!
	return output;
}

b3Transform b3GetSweepTransform( const b3Sweep* sweep, float time )
{
	b3Transform transform;
	transform.q = b3NLerp( sweep->q1, sweep->q2, time );
	transform.p = b3Lerp( sweep->c1, sweep->c2, time ) - b3RotateVector( transform.q, sweep->localCenter );
	return transform;
}

static int b3UniqueCount( int vertexCount, const uint8_t* vertices )
{
	B3_ASSERT( 1 <= vertexCount && vertexCount <= 3 );

	switch ( vertexCount )
	{
		case 1:
		{
			return 1;
		}

		case 2:
		{
			return vertices[0] != vertices[1] ? 2 : 1;
		}

		case 3:
		{
			if ( vertices[0] != vertices[1] && vertices[0] != vertices[2] && vertices[1] != vertices[2] )
			{
				// All different
				return 3;
			}
			else if ( vertices[0] == vertices[1] && vertices[0] == vertices[2] && vertices[1] == vertices[2] )
			{
				// All equal
				return 1;
			}
			else
			{
				return 2;
			}
		}

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return 0;
}

static bool b3CheckFastEdges( const b3Transform& transform1, b3Vec3 localEdge1, const b3Transform& transform2, b3Vec3 localEdge2,
							  b3Vec3 axis0 )
{
	// By taking the local witness axes we make sure that we
	// get the correct orientations (e.g. if one axis was flipped)!
	b3Vec3 edge1 = b3RotateVector( transform1.q, localEdge1 );
	b3Vec3 edge2 = b3RotateVector( transform2.q, localEdge2 );
	b3Vec3 axis = b3Cross( edge1, edge2 );
	axis = b3Normalize( axis );

	return b3Dot( axis, axis0 ) < 0.0f;
}

enum class b3SeparationType
{
	unknown = 0,
	vertices,
	edges,
	face1,
	face2
};

struct b3SeparationFunction
{
	const b3ShapeProxy* proxyA;
	const b3ShapeProxy* proxyB;
	b3Sweep sweepA;
	b3Sweep sweep2;
	b3Vec3 witnessA;
	b3Vec3 witnessB;
	b3SeparationType type;
};

static b3SeparationFunction b3MakeSeparationFunction( const b3Sweep& sweep1, const b3ShapeProxy* proxy1, const b3Sweep& sweep2,
													  const b3ShapeProxy* proxy2, const b3DistanceOutput& query,
													  b3SimplexCache cache, float beta )
{
	B3_ASSERT( 1 <= cache.count && cache.count <= 3 );

	b3SeparationFunction fcn = {};
	fcn.proxyA = proxy1;
	fcn.proxyB = proxy2;
	fcn.sweepA = sweep1;
	fcn.sweep2 = sweep2;
	fcn.type = b3SeparationType::unknown;

	int uniqueCount1 = b3UniqueCount( cache.count, cache.indexA );
	int uniqueCount2 = b3UniqueCount( cache.count, cache.indexB );

	b3Transform transform1 = b3GetSweepTransform( &sweep1, beta );
	b3Transform transform2 = b3GetSweepTransform( &sweep2, beta );

	switch ( cache.count )
	{
		case 1:
		{
			b3Vec3 vertex1 = b3TransformPoint( transform1, proxy1->points[cache.indexA[0]] );
			b3Vec3 vertex2 = b3TransformPoint( transform2, proxy2->points[cache.indexB[0]] );

			fcn.type = b3SeparationType::vertices;
			fcn.witnessA = b3Normalize( vertex2 - vertex1 );
			fcn.witnessB = b3Vec3_zero;
		}
		break;

		case 2:
		{
			if ( uniqueCount1 == 2 && uniqueCount2 == 2 )
			{
				// Edge/Edge
				b3Vec3 vertexA1 = b3TransformPoint( transform1, proxy1->points[cache.indexA[0]] );
				b3Vec3 vertexA2 = b3TransformPoint( transform1, proxy1->points[cache.indexA[1]] );
				b3Vec3 edgeA = b3Normalize( vertexA2 - vertexA1 );

				b3Vec3 vertexB1 = b3TransformPoint( transform2, proxy2->points[cache.indexB[0]] );
				b3Vec3 vertexB2 = b3TransformPoint( transform2, proxy2->points[cache.indexB[1]] );
				b3Vec3 edgeB = b3Normalize( vertexB2 - vertexB1 );

				b3Vec3 axis = b3Cross( edgeA, edgeB );
				float length = b3Length( axis );

				// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
				const float kTolerance = 0.005f;
				if ( length < kTolerance )
				{
					// The axis is not safe to normalize so we use a world axis instead!
					fcn.type = b3SeparationType::vertices;
					fcn.witnessA = b3Normalize( query.pointB - query.pointA );
					fcn.witnessB = b3Vec3_zero;
				}
				else
				{
					axis *= 1.0f / length;
					if ( b3Dot( vertexB1 - vertexA1, axis ) < 0.0f )
					{
						axis = -axis;
						edgeB = -edgeB;
					}

					// Check for possible sign flip in edge/edge cross product
					if ( b3CheckFastEdges( b3GetSweepTransform( &sweep1, 1.0f ), b3InvRotateVector( transform1.q, edgeA ),
										   b3GetSweepTransform( &sweep2, 1.0f ), b3InvRotateVector( transform2.q, edgeB ),
										   axis ) )
					{
						fcn.type = b3SeparationType::vertices;
						fcn.witnessA = axis;
						fcn.witnessB = b3Vec3_zero;
					}
					else
					{
						fcn.type = b3SeparationType::edges;
						fcn.witnessA = b3InvRotateVector( transform1.q, edgeA );
						fcn.witnessB = b3InvRotateVector( transform2.q, edgeB );
					}
				}
			}
			else
			{
				// Vertex/Edge
				fcn.type = b3SeparationType::vertices;
				fcn.witnessA = b3Normalize( query.pointB - query.pointA );
				fcn.witnessB = b3Vec3_zero;
			}
		}
		break;

		case 3:
		{
			if ( uniqueCount1 == 3 )
			{
				b3Vec3 vertexA1 = b3TransformPoint( transform1, proxy1->points[cache.indexA[0]] );
				b3Vec3 vertexA2 = b3TransformPoint( transform1, proxy1->points[cache.indexA[1]] );
				b3Vec3 vertexA3 = b3TransformPoint( transform1, proxy1->points[cache.indexA[2]] );
				b3Vec3 axis = b3Cross( vertexA2 - vertexA1, vertexA3 - vertexA1 );
				axis = b3Normalize( axis );

				b3Vec3 point1 = ( 1.0f / 3.0f ) * ( vertexA1 + vertexA2 + vertexA3 );
				b3Vec3 point2 = b3TransformPoint( transform2, proxy2->points[cache.indexB[0]] );

				if ( b3Dot( point2 - point1, axis ) < 0.0f )
				{
					axis = -axis;
				}

				fcn.type = b3SeparationType::face1;
				fcn.witnessA = b3InvRotateVector( transform1.q, axis );
				fcn.witnessB = b3InvTransformPoint( transform1, point1 );
			}
			else if ( uniqueCount2 == 3 )
			{
				b3Vec3 vertexB1 = b3TransformPoint( transform2, proxy2->points[cache.indexB[0]] );
				b3Vec3 vertexB2 = b3TransformPoint( transform2, proxy2->points[cache.indexB[1]] );
				b3Vec3 vertexB3 = b3TransformPoint( transform2, proxy2->points[cache.indexB[2]] );
				b3Vec3 axis = b3Cross( vertexB2 - vertexB1, vertexB3 - vertexB1 );
				axis = b3Normalize( axis );

				b3Vec3 point1 = b3TransformPoint( transform1, proxy1->points[cache.indexA[0]] );
				b3Vec3 point2 = ( 1.0f / 3.0f ) * ( vertexB1 + vertexB2 + vertexB3 );

				if ( b3Dot( point1 - point2, axis ) < 0.0f )
				{
					axis = -axis;
				}

				fcn.type = b3SeparationType::face2;
				fcn.witnessA = b3InvRotateVector( transform2.q, axis );
				fcn.witnessB = b3InvTransformPoint( transform2, point2 );
			}
			else
			{
				B3_ASSERT( uniqueCount1 == 2 && uniqueCount2 == 2 );

				if ( cache.indexA[0] == cache.indexA[1] )
				{
					uint8_t temp = cache.indexA[1];
					cache.indexA[1] = cache.indexA[2];
					cache.indexA[2] = temp;

					B3_ASSERT( cache.indexA[0] != cache.indexA[1] );
				}
				b3Vec3 vertexA1 = b3TransformPoint( transform1, proxy1->points[cache.indexA[0]] );
				b3Vec3 vertexA2 = b3TransformPoint( transform1, proxy1->points[cache.indexA[1]] );
				b3Vec3 edgeA = b3Normalize( vertexA2 - vertexA1 );

				if ( cache.indexB[0] == cache.indexB[1] )
				{
					uint8_t temp = cache.indexB[1];
					cache.indexB[1] = cache.indexB[2];
					cache.indexB[2] = temp;

					// std::swap(Cache.indexB[1], Cache.indexB[2]);

					B3_ASSERT( cache.indexB[0] != cache.indexB[1] );
				}
				b3Vec3 vertexB1 = b3TransformPoint( transform2, proxy2->points[cache.indexB[0]] );
				b3Vec3 vertexB2 = b3TransformPoint( transform2, proxy2->points[cache.indexB[1]] );
				b3Vec3 edgeB = b3Normalize( vertexB2 - vertexB1 );

				b3Vec3 axis = b3Cross( edgeA, edgeB );
				float length = b3Length( axis );

				// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
				const float kTolerance = 0.005f;
				if ( length < kTolerance )
				{
					// The axis is not safe to normalize so we use a world axis instead!
					fcn.type = b3SeparationType::vertices;
					fcn.witnessA = b3Normalize( query.pointB - query.pointA );
					fcn.witnessB = b3Vec3_zero;
				}
				else
				{
					axis *= 1.0f / length;
					if ( b3Dot( vertexB1 - vertexA1, axis ) < 0.0f )
					{
						axis = -axis;
						edgeB = -edgeB;
					}

					// Check for possible sign flip in edge/edge cross product
					// todo_erin get final transform directly
					if ( b3CheckFastEdges( b3GetSweepTransform( &sweep1, 1.0f ), b3InvRotateVector( transform1.q, edgeA ),
										   b3GetSweepTransform( &sweep2, 1.0f ), b3InvRotateVector( transform2.q, edgeB ),
										   axis ) )
					{
						fcn.type = b3SeparationType::vertices;
						fcn.witnessA = axis;
						fcn.witnessB = b3Vec3_zero;
					}
					else
					{
						fcn.type = b3SeparationType::edges;
						fcn.witnessA = b3InvRotateVector( transform1.q, edgeA );
						fcn.witnessB = b3InvRotateVector( transform2.q, edgeB );
					}
				}
			}
		}
		break;

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return fcn;
}

static float b3FindMinSeparation( b3SeparationFunction* fcn, int& index1, int& index2, float beta )
{
	b3Transform transform1 = b3GetSweepTransform( &fcn->sweepA, beta );
	b3Transform transform2 = b3GetSweepTransform( &fcn->sweep2, beta );

	switch ( fcn->type )
	{
		case b3SeparationType::vertices:
		{
			b3Vec3 axis = fcn->witnessA;

			b3Vec3 axis1 = b3InvRotateVector( transform1.q, axis );
			index1 = b3GetPointSupport( fcn->proxyA->points, fcn->proxyA->count, axis1 );
			b3Vec3 point1 = b3TransformPoint( transform1, fcn->proxyA->points[index1] );

			b3Vec3 axis2 = b3InvRotateVector( transform2.q, axis );
			index2 = b3GetPointSupport( fcn->proxyB->points, fcn->proxyB->count, -axis2 );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->proxyB->points[index2] );

			return b3Dot( point2 - point1, axis );
		}
		break;

		case b3SeparationType::edges:
		{
			b3Vec3 edge1 = b3RotateVector( transform1.q, fcn->witnessA );
			b3Vec3 edge2 = b3RotateVector( transform2.q, fcn->witnessB );
			b3Vec3 axis = b3Cross( edge1, edge2 );
			B3_ASSERT( axis.x != 0.0f || axis.y != 0.0f || axis.z != 0.0f );
			axis = b3Normalize( axis );

			b3Vec3 axis1 = b3InvRotateVector( transform1.q, axis );
			index1 = b3GetPointSupport( fcn->proxyA->points, fcn->proxyA->count, axis1 );
			b3Vec3 point1 = b3TransformPoint( transform1, fcn->proxyA->points[index1] );

			b3Vec3 axis2 = b3InvRotateVector( transform2.q, axis );
			index2 = b3GetPointSupport( fcn->proxyB->points, fcn->proxyB->count, -axis2 );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->proxyB->points[index2] );

			return b3Dot( point2 - point1, axis );
		}
		break;

		case b3SeparationType::face1:
		{
			b3Vec3 axis = b3RotateVector( transform1.q, fcn->witnessA );

			index1 = -1;
			b3Vec3 point1 = b3TransformPoint( transform1, fcn->witnessB );

			b3Vec3 axis2 = b3InvRotateVector( transform2.q, axis );
			index2 = b3GetPointSupport( fcn->proxyB->points, fcn->proxyB->count, -axis2 );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->proxyB->points[index2] );

			return b3Dot( point2 - point1, axis );
		}
		break;

		case b3SeparationType::face2:
		{
			b3Vec3 axis = b3RotateVector( transform2.q, fcn->witnessA );

			b3Vec3 axis1 = b3InvRotateVector( transform1.q, axis );
			index1 = b3GetPointSupport( fcn->proxyA->points, fcn->proxyA->count, -axis1 );
			b3Vec3 point1 = b3TransformPoint( transform1, fcn->proxyA->points[index1] );

			index2 = -1;
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->witnessB );

			return b3Dot( point1 - point2, axis );
		}
		break;

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return 0.0f;
}

static float b3Evaluate( b3SeparationFunction* fcn, int index1, int index2, float beta )
{
	b3Transform transform1 = b3GetSweepTransform( &fcn->sweepA, beta );
	b3Transform transform2 = b3GetSweepTransform( &fcn->sweep2, beta );

	switch ( fcn->type )
	{
		case b3SeparationType::vertices:
		{
			b3Vec3 axis = fcn->witnessA;

			b3Vec3 point1 = b3TransformPoint( transform1, fcn->proxyA->points[index1] );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->proxyB->points[index2] );

			return b3Dot( point2 - point1, axis );
		}
		break;

		case b3SeparationType::edges:
		{
			b3Vec3 edge1 = b3RotateVector( transform1.q, fcn->witnessA );
			b3Vec3 edge2 = b3RotateVector( transform2.q, fcn->witnessB );
			b3Vec3 axis = b3Cross( edge1, edge2 );
			axis = b3Normalize( axis );

			b3Vec3 point1 = b3TransformPoint( transform1, fcn->proxyA->points[index1] );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->proxyB->points[index2] );

			return b3Dot( point2 - point1, axis );
		}
		break;

		case b3SeparationType::face1:
		{
			b3Vec3 axis = b3RotateVector( transform1.q, fcn->witnessA );

			b3Vec3 point1 = b3TransformPoint( transform1, fcn->witnessB );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->proxyB->points[index2] );

			return b3Dot( point2 - point1, axis );
		}
		break;

		case b3SeparationType::face2:
		{
			b3Vec3 axis = b3RotateVector( transform2.q, fcn->witnessA );

			b3Vec3 point1 = b3TransformPoint( transform1, fcn->proxyA->points[index1] );
			b3Vec3 point2 = b3TransformPoint( transform2, fcn->witnessB );

			return b3Dot( point1 - point2, axis );
		}
		break;

		default:
			B3_ASSERT( !"Should never get here!" );
			break;
	}

	return 0.0f;
}

static void b3ForceFixedAxis( b3SeparationFunction* fcn, float beta )
{
	B3_ASSERT( fcn->type == b3SeparationType::edges );

	b3Transform transform1 = b3GetSweepTransform( &fcn->sweepA, beta );
	b3Transform transform2 = b3GetSweepTransform( &fcn->sweep2, beta );

	b3Vec3 edge1 = b3RotateVector( transform1.q, fcn->witnessA );
	b3Vec3 edge2 = b3RotateVector( transform2.q, fcn->witnessB );
	b3Vec3 axis = b3Cross( edge1, edge2 );
	axis = b3Normalize( axis );

	fcn->type = b3SeparationType::vertices;
	fcn->witnessA = axis;
	fcn->witnessB = b3Vec3_zero;
}

// Time of Impact using root finding
// todo this should be done in relative space
b3TOIOutput b3TimeOfImpact( const b3TOIInput* input )
{
	// Setup target distance and tolerance
	float totalRadius = input->proxyA.radius + input->proxyB.radius;
	float target = b3MaxFloat( B3_LINEAR_SLOP, totalRadius - B3_LINEAR_SLOP );
	float tolerance = 0.25f * B3_LINEAR_SLOP;
	B3_ASSERT( target > tolerance );

	// Initialize simplex cache
	b3SimplexCache cache = {};
	b3DistanceInput distanceInput = {};
	b3DistanceOutput distanceOutput = {};
	b3TOIOutput output = {};

	distanceInput.proxyA = input->proxyA;
	distanceInput.proxyB = input->proxyB;
	distanceInput.useRadii = false;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	float alpha1 = 0.0f;
	constexpr int maxIterations = 20;
	int iteration = 0;
	for ( ; iteration < maxIterations; ++iteration )
	{
		distanceInput.transformA = b3GetSweepTransform( &input->sweepA, alpha1 );
		distanceInput.transformB = b3GetSweepTransform( &input->sweepB, alpha1 );

		distanceOutput = b3ShapeDistance( &distanceInput, &cache, nullptr, 0 );

		// Are the shapes overlapping?
		if ( distanceOutput.distance <= 0.0f )
		{
			// Failed!
			output.state = b3_toiStateOverlapped;
			output.fraction = 0.0f;
			break;
			// return { B3_TOI_OVERLAPPED, 0.0f, query.pointA, query.pointB, query.Normal, query.Distance, iteration };
		}

		if ( distanceOutput.distance <= target + tolerance )
		{
			// Success!
			output.state = b3_toiStateHit;

			// Averaged hit point
			b3Vec3 pA = b3MulAdd( distanceOutput.pointA, input->proxyA.radius, distanceOutput.normal );
			b3Vec3 pB = b3MulAdd( distanceOutput.pointB, -input->proxyB.radius, distanceOutput.normal );
			output.point = b3Lerp( pA, pB, 0.5f );
			output.normal = distanceOutput.normal;
			output.fraction = alpha1;
			break;
			// return { B3_TOI_TOUCHING, alpha1, query.pointA, query.pointB, query.Normal, query.Distance, iteration };
		}

		// Initialize the separating axis.
		b3SeparationFunction function = b3MakeSeparationFunction( input->sweepA, &input->proxyA, input->sweepB, &input->proxyB,
																  distanceOutput, cache, alpha1 );

#ifndef NDEBUG
		// Validate separation function
		//{
		//	int Index1, Index2;
		//	float MinSeparation = Function.FindMinSeparation(Index1, Index2, Alpha1);
		//	B3_ASSERT(MinSeparation > Target - Tolerance && b3Abs(MinSeparation - Query.Distance) < B3_LINEAR_SLOP);
		//}
#endif

		// Compute the TOI on the separating axis. We do this by successively resolving the deepest point.
		float alpha2 = input->maxFraction;
		for ( int innerIteration = 0; innerIteration < maxIterations; ++innerIteration )
		{
			int index1, index2;
			float separation2 = b3FindMinSeparation( &function, index1, index2, alpha2 );

			// Dump the function seen by the root finder
			// 			for ( int Step = 0; Step <= 100; ++Step )
			// 				{
			// 				float Alpha = 0.01f * Step;
			// 				float Separation = Function.Evaluate( Index1, Index2, Alpha );
			//
			// 				b3Report( "s(%4.2g) = %g\n", Alpha, Separation );
			// 				}

			// Is the final configuration separated?
			if ( separation2 - target > tolerance )
			{
				// Success!
				output.state = b3_toiStateSeparated;
				output.fraction = input->maxFraction;
				break;

				// return { B3_TOI_SEPARATED, maxAlpha, query.pointA, query.pointB, query.Normal, query.Distance, iteration };
			}

			// Has the separation reached tolerance?
			if ( separation2 >= target - tolerance )
			{
				// Advance the sweeps
				alpha1 = alpha2;
				break;
			}

			// Compute the initial separation of the witness points
			float separation1 = b3Evaluate( &function, index1, index2, alpha1 );

			// Check for overlap. This might happen if
			// the root finder runs out of iterations.
			if ( separation1 < target - tolerance )
			{
				// Failed!
				output.state = b3_toiStateFailed;
				output.fraction = alpha1;
				break;
				// return { B3_TOI_FAILED, alpha1, query.pointA, query.pointB, query.Normal, query.Distance, iteration };
			}

			// Has the separation reached tolerance?
			if ( separation1 <= target + tolerance )
			{
				// Success! T1 should hold the TOI (could be 0.0)
				// distanceInput.transformA = b3GetSweepTransform( &input->sweepA, alpha1 );
				// distanceInput.transformB = b3GetSweepTransform( &input->sweepB, alpha1 );
				// distanceOutput = b3ShapeDistance( &cache, &distanceInput, nullptr, 0 );

				output.state = b3_toiStateHit;

				// Averaged hit point
				b3Vec3 pA = b3MulAdd( distanceOutput.pointA, input->proxyA.radius, distanceOutput.normal );
				b3Vec3 pB = b3MulAdd( distanceOutput.pointB, -input->proxyB.radius, distanceOutput.normal );
				output.point = b3Lerp( pA, pB, 0.5f );
				output.normal = distanceOutput.normal;
				output.fraction = alpha1;
				break;

				// return { B3_TOI_TOUCHING, alpha1, query.pointA, query.pointB, query.Normal, query.Distance, iteration };
			}

			// Compute 1D bracketed root of: s(t) - target = 0
			// s(t) = Dot( SupportB( -Axis ) - SupportA( Axis ), Axis )
			float beta1 = alpha1;
			float beta2 = alpha2;

			const int kMaxRootIterations = 64;
			for ( int rootIteration = 0; rootIteration < kMaxRootIterations; ++rootIteration )
			{
				// Use a mix of Regula-Falsi and bisection
				float beta = ( rootIteration & 1 )
								 ? beta1 + ( target - separation1 ) * ( beta2 - beta1 ) / ( separation2 - separation1 )
								 : 0.5f * ( beta1 + beta2 );
				float separation = b3Evaluate( &function, index1, index2, beta );

				// Has the separation reached tolerance?
				if ( b3AbsFloat( separation - target ) <= 0.1f * tolerance )
				{
					// Alpha2 holds a tentative value for Alpha1
					alpha2 = beta;
					break;
				}

				// Ensure we continue to bracket the root.
				if ( separation > target )
				{
					beta1 = beta;
					separation1 = separation;
				}
				else
				{
					beta2 = beta;
					separation2 = separation;
				}
			}

			// Restart the inner loop if we have a failing edge case.
			if ( innerIteration == maxIterations - 1 && function.type == b3SeparationType::edges )
			{
				innerIteration = 0;

				alpha2 = input->maxFraction;
				b3ForceFixedAxis( &function, alpha1 );
				B3_ASSERT( function.type != b3SeparationType::edges );
			}
		}
	}

	if ( iteration == maxIterations )
	{
		output.state = b3_toiStateFailed;

		// Averaged hit point
		b3Vec3 pA = b3MulAdd( distanceOutput.pointA, input->proxyA.radius, distanceOutput.normal );
		b3Vec3 pB = b3MulAdd( distanceOutput.pointB, -input->proxyB.radius, distanceOutput.normal );
		output.point = b3Lerp( pA, pB, 0.5f );
		output.normal = distanceOutput.normal;
		output.fraction = alpha1;
	}

	return output;
}
