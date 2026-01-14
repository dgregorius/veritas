// SPDX-FileCopyrightText: 2025 Dirk Gregorius and Erin Catto
// SPDX-License-Identifier: MIT

#include "algorithm.h"
#include "constants.h"
#include "core.h"
#include "platform.h"
#include "sat.h"
#include "shape.h"

#include "box3d/base.h"
#include "box3d/collision.h"

#include <stddef.h>

static bool b3AreParallel( const b3Capsule* capsule1, const b3Capsule* capsule2 )
{
	b3Vec3 segment1 = capsule1->center2 - capsule1->center1;
	float length1 = b3Length( segment1 );
	if ( length1 < B3_LINEAR_SLOP )
	{
		return false;
	}

	b3Vec3 segment2 = capsule2->center2 - capsule2->center1;
	float length2 = b3Length( segment2 );
	if ( length2 < B3_LINEAR_SLOP )
	{
		return false;
	}

	// Parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
	const float kTolerance = 0.005f;
	b3Vec3 axis = b3Cross( segment1, segment2 );

	return b3Length( axis ) < kTolerance * length1 * length2;
}

static void b3BuildClipPlanes( b3Plane out[2], const b3Capsule* capsule )
{
	b3Vec3 normal = b3Normalize( capsule->center2 - capsule->center1 );

	out[0].normal = -normal;
	out[0].offset = -b3Dot( normal, capsule->center1 );
	out[1].normal = normal;
	out[1].offset = b3Dot( normal, capsule->center2 );
}

b3Manifold b3CollideSpheres( const b3Sphere* sphereA, b3Transform xfA, const b3Sphere* sphereB, b3Transform xfB )
{
	b3Manifold manifold = {};

	b3Transform xf = b3InvMulTransforms( xfA, xfB );

	b3Vec3 center1 = sphereA->center;
	b3Vec3 center2 = b3TransformPoint( xf, sphereB->center );

	float totalRadius = sphereA->radius + sphereB->radius;
	b3Vec3 offset = center2 - center1;
	float distanceSq = b3LengthSquared( offset );

	if ( distanceSq > totalRadius * totalRadius )
	{
		// We found a separating axis
		return manifold;
	}

	b3Vec3 normal = { 0.0f, 1.0f, 0.0f };
	float distance = sqrtf( distanceSq );
	if ( distance * distance > 1000.0f * FLT_MIN )
	{
		normal = ( 1.0f / distance ) * offset;
	}

	// contact point is mid-way
	b3Vec3 point = 0.5f * ( center1 + sphereA->radius * normal + center2 - sphereB->radius * normal );

	manifold.normal = b3RotateVector( xfA.q, normal );

	b3ManifoldPoint* mp = manifold.points + 0;
	mp->anchorA = b3RotateVector( xfA.q, point );
	mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
	mp->point = mp->anchorA + xfA.p;
	mp->separation = distance - totalRadius;
	mp->id = b3MakeFeatureKeyFull( b3_featureShapeA, 0, b3_featureShapeA, 0 );

	manifold.pointCount = 1;

	return manifold;
}

// todo_erin fixme
b3Manifold b3CollideCapsuleAndSphere( const b3Capsule* capsule, b3Transform xfA, const b3Sphere* sphere, b3Transform xfB )
{
	b3Manifold manifold = {};

	b3Transform xf = b3InvMulTransforms( xfA, xfB );

	b3Vec3 center = b3TransformPoint( xf, sphere->center );

	b3Vec3 center1 = capsule->center1;
	b3Vec3 center2 = capsule->center2;

	float totalRadius = sphere->radius + capsule->radius;

	b3Vec3 closestPoint = b3ClosestPointOnSegment( center1, center2, center );
	b3Vec3 offset = center - closestPoint;
	float distanceSq = b3LengthSquared( offset );

	if ( distanceSq > totalRadius * totalRadius )
	{
		// We found a separating axis
		return manifold;
	}

	b3Vec3 normal = { 0.0f, 1.0f, 0.0f };
	float distance = sqrtf( distanceSq );
	if ( distance * distance > 1000.0f * FLT_MIN )
	{
		normal = ( 1.0f / distance ) * offset;
	}

	// contact point is mid-way
	b3Vec3 point = 0.5f * ( center - sphere->radius * normal + closestPoint + capsule->radius * normal );

	manifold.normal = b3RotateVector( xfA.q, normal );

	b3ManifoldPoint* mp = manifold.points + 0;
	mp->anchorA = b3RotateVector( xfA.q, point );
	mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
	mp->point = mp->anchorA + xfA.p;
	mp->separation = distance - totalRadius;
	mp->id = b3MakeFeatureKeyFull( b3_featureShapeA, 0, b3_featureShapeA, 0 );
	manifold.pointCount = 1;

	return manifold;
}

// todo this could be optimized by iterating the hull faces for the maximum separation then using GJK versus that face if
// separated
b3Manifold b3CollideHullAndSphere( const b3Hull* hull, b3Transform xfA, const b3Sphere* sphere, b3Transform xfB,
								   b3SimplexCache* cache )
{
	b3Manifold manifold = {};
	const float speculativeDistance = B3_SPECULATIVE_DISTANCE;

	b3Transform xf = b3InvMulTransforms( xfA, xfB );

	b3DistanceInput distanceInput;
	distanceInput.proxyA = { b3GetHullPoints( hull ), hull->vertexCount, 0.0f };
	distanceInput.proxyB = { &sphere->center, 1, 0.0f };
	distanceInput.transformA = b3Transform_identity;
	distanceInput.transformB = xf;
	distanceInput.useRadii = false;

	float radiusA = 0.0f;
	float radiusB = sphere->radius;
	float radius = radiusA + radiusB;

	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, cache, nullptr, 0 );

	if ( distanceOutput.distance > radius + speculativeDistance )
	{
		// We found a separating axis
		*cache = {};
		return manifold;
	}

	b3Vec3 center = b3TransformPoint( xf, sphere->center );

	if ( distanceOutput.distance > 100.0f * FLT_EPSILON )
	{
		// Shallow penetration
		b3Vec3 normal = b3Normalize( distanceOutput.pointB - distanceOutput.pointA );

		// cA is the projection of the sphere center onto to the hull
		b3Vec3 cA = b3MulAdd( center, radiusA - b3Dot( b3Sub( center, distanceOutput.pointA ), normal ), normal );

		// cB is the deepest point on the sphere with respect to the reference f
		b3Vec3 cB = b3MulSub( center, radiusB, normal );

		b3Vec3 contactPoint = b3Lerp( cA, cB, 0.5f );

		manifold.normal = b3RotateVector( xfA.q, normal );

		b3ManifoldPoint* mp = manifold.points + 0;
		mp->anchorA = b3RotateVector( xfA.q, contactPoint );
		mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
		mp->point = mp->anchorA + xfA.p;
		mp->separation = distanceOutput.distance - radius;
		mp->id = b3MakeFeatureKeyFull( b3_featureShapeA, 0, b3_featureShapeA, 0 );

		manifold.pointCount = 1;
	}
	else
	{
		// Deep penetration
		int bestIndex = -1;
		float bestDistance = -FLT_MAX;
		const b3Plane* planes = b3GetHullPlanes( hull );

		for ( int index = 0; index < hull->faceCount; ++index )
		{
			b3Plane plane = planes[index];

			float distance = b3PlaneSeparation( plane, center );
			if ( distance > bestDistance )
			{
				bestIndex = index;
				bestDistance = distance;
			}
		}
		B3_ASSERT( bestIndex >= 0 );

		b3Vec3 normal = planes[bestIndex].normal;

		// cA is the projection of the sphere center onto to the hull
		b3Vec3 cA = b3MulAdd( center, radiusA - b3Dot( b3Sub( center, distanceOutput.pointA ), normal ), normal );

		// cB is the deepest point on the sphere with respect to the reference f
		b3Vec3 cB = b3MulSub( center, radiusB, normal );

		b3Vec3 contactPoint = b3Lerp( cA, cB, 0.5f );

		manifold.normal = b3RotateVector( xfA.q, normal );

		b3ManifoldPoint* mp = manifold.points + 0;
		mp->anchorA = b3RotateVector( xfA.q, contactPoint );
		mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
		mp->point = mp->anchorA + xfA.p;
		mp->separation = bestDistance - radius;
		mp->id = b3MakeFeatureKeyFull( b3_featureShapeA, 0, b3_featureShapeA, 0 );

		manifold.pointCount = 1;
	}

	return manifold;
}

b3Manifold b3CollideCapsules( const b3Capsule* capsuleA, b3Transform xfA, const b3Capsule* capsuleB, b3Transform xfB )
{
	b3Manifold manifold = {};

	b3Transform xf = b3InvMulTransforms( xfA, xfB );

	b3Capsule localCapsuleB;
	localCapsuleB.center1 = b3TransformPoint( xf, capsuleB->center1 );
	localCapsuleB.center2 = b3TransformPoint( xf, capsuleB->center2 );
	localCapsuleB.radius = capsuleB->radius;

	float radius = capsuleA->radius + capsuleB->radius;
	float maxDistance = radius + B3_SPECULATIVE_DISTANCE;

	b3ClosestApproachResult result =
		b3ClosestApproachSegments( capsuleA->center1, capsuleA->center2, localCapsuleB.center1, localCapsuleB.center2 );
	b3Vec3 offset = result.point2 - result.point1;
	float distanceSq = b3LengthSquared( offset );

	if ( distanceSq > maxDistance * maxDistance )
	{
		// We found a separating axis
		return manifold;
	}

	// Try to create two contact points if the capsules are nearly parallel
	if ( b3AreParallel( capsuleA, &localCapsuleB ) )
	{
		// Clip first segment against side planes of second segment
		// todo this is backwards
		b3ClipVertex segment[2];
		b3BuildCapsuleSegment( segment, capsuleA, b3InvertTransform( xf ) );

		b3Plane clipPlanes[2];
		b3BuildClipPlanes( clipPlanes, &localCapsuleB );

		if ( b3ClipSegment( segment, clipPlanes[0] ) && b3ClipSegment( segment, clipPlanes[1] ) )
		{
			b3Vec3 closestPoint1 = b3ClosestPointOnSegment( localCapsuleB.center1, localCapsuleB.center2, segment[0].position );
			b3Vec3 closestPoint2 = b3ClosestPointOnSegment( localCapsuleB.center1, localCapsuleB.center2, segment[1].position );

			float distance1 = b3Distance( closestPoint1, segment[0].position );
			float distance2 = b3Distance( closestPoint2, segment[1].position );
			if ( distance1 <= radius && distance2 <= radius )
			{
				if (distance1 < 0.1f * B3_LINEAR_SLOP || distance2 < 0.1f * B3_LINEAR_SLOP)
				{
					// Avoid divide by zero
					return manifold;
				}

				b3Vec3 normal1 = ( 1.0f / distance1 ) * ( closestPoint1 - segment[0].position );
				b3Vec3 normal2 = ( 1.0f / distance2 ) * ( closestPoint2 - segment[1].position );
				b3Vec3 normal = b3Normalize( normal1 + normal2 );

				b3Vec3 point1 =
					0.5f * ( segment[0].position + capsuleA->radius * normal1 + closestPoint1 - capsuleB->radius * normal );
				b3Vec3 point2 =
					0.5f * ( segment[1].position + capsuleA->radius * normal2 + closestPoint2 - capsuleB->radius * normal );

				manifold.normal = b3RotateVector( xfA.q, normal );

				b3ManifoldPoint* mp = manifold.points + 0;
				mp->anchorA = b3RotateVector( xfA.q, point1 );
				mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
				mp->point = mp->anchorA + xfA.p;
				mp->separation = distance1 - radius;
				mp->id = b3MakeFeatureKey( segment[0].pair );

				mp = manifold.points + 1;
				mp->anchorA = b3RotateVector( xfA.q, point2 );
				mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
				mp->point = mp->anchorA + xfA.p;
				mp->separation = distance2 - radius;
				mp->id = b3MakeFeatureKey( segment[1].pair );

				manifold.pointCount = 2;
				return manifold;
			}
		}
	}

	b3Vec3 normal = { 0.0f, 1.0f, 0.0f };
	float distance = sqrtf( distanceSq );
	if ( distance * distance > 1000.0f * FLT_MIN )
	{
		normal = ( 1.0f / distance ) * offset;
	}

	b3Vec3 point = 0.5f * ( result.point1 + capsuleA->radius * normal + result.point2 - capsuleB->radius * normal );

	b3FeaturePair pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );

	manifold.normal = b3RotateVector( xfA.q, normal );

	b3ManifoldPoint* mp = manifold.points + 0;
	mp->anchorA = b3RotateVector( xfA.q, point );
	mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
	mp->point = mp->anchorA + xfA.p;
	mp->separation = distance - radius;
	mp->id = b3MakeFeatureKey( pair );

	manifold.pointCount = 1;
	return manifold;
}

static void b3BuildSegment( b3ClipVertex out[2], const b3Capsule* capsule, const b3Transform& transform )
{
	out[0].position = b3TransformPoint( transform, capsule->center1 );
	out[0].separation = 0.0f;
	out[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
	out[1].position = b3TransformPoint( transform, capsule->center2 );
	out[1].separation = 0.0f;
	out[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );
}

static bool b3BuildHullAndCapsuleFaceContact( b3Manifold& manifold, const b3Hull* hull, b3Transform xfA, const b3Capsule* capsule,
											  b3Transform xfB, b3FaceQuery query )
{
	manifold.pointCount = 0;

	b3Transform xf = b3InvMulTransforms( xfA, xfB );

	const b3Plane* planes = b3GetHullPlanes( hull );

	// Clip the capsule edge against the side planes of the reference face
	int refFace = query.faceIndex;
	b3Plane refPlane = planes[refFace];

	b3ClipVertex segment[2];
	b3BuildSegment( segment, capsule, xf );

	// todo get rid of identity transform
	if ( b3ClipSegmentToHullFace( segment, hull, b3Transform_identity, refFace ) == false )
	{
		return false;
	}

	float distance1 = b3PlaneSeparation( refPlane, segment[0].position );
	float distance2 = b3PlaneSeparation( refPlane, segment[1].position );

	if ( distance1 <= B3_SPECULATIVE_DISTANCE && distance2 <= B3_SPECULATIVE_DISTANCE )
	{
		b3Vec3 normal = refPlane.normal;
		b3Vec3 point1 = segment[0].position - 0.5f * ( distance1 + capsule->radius ) * normal;
		b3Vec3 point2 = segment[1].position - 0.5f * ( distance2 + capsule->radius ) * normal;

		manifold.normal = b3RotateVector( xfA.q, normal );

		b3ManifoldPoint* mp = manifold.points + 0;
		mp->anchorA = b3RotateVector( xfA.q, point1 );
		mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
		mp->point = mp->anchorA + xfA.p;
		mp->separation = distance1 - capsule->radius;
		mp->id = b3MakeFeatureKey( segment[0].pair );

		mp = manifold.points + 1;
		mp->anchorA = b3RotateVector( xfA.q, point2 );
		mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
		mp->point = mp->anchorA + xfA.p;
		mp->separation = distance2 - capsule->radius;
		mp->id = b3MakeFeatureKey( segment[1].pair );

		manifold.pointCount = 2;
		return true;
	}

	if ( distance1 <= B3_SPECULATIVE_DISTANCE )
	{
		B3_VALIDATE( distance2 > B3_SPECULATIVE_DISTANCE );

		b3Vec3 normal = refPlane.normal;
		b3Vec3 point = segment[0].position + 0.5f * ( capsule->radius + distance1 ) * normal;

		manifold.normal = b3RotateVector( xfA.q, normal );

		b3ManifoldPoint* mp = manifold.points + 0;
		mp->anchorA = b3RotateVector( xfA.q, point );
		mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
		mp->point = mp->anchorA + xfA.p;
		mp->separation = distance1 - capsule->radius;
		mp->id = b3MakeFeatureKey( segment[0].pair );

		manifold.pointCount = 1;
		return true;
	}

	if ( distance2 <= B3_SPECULATIVE_DISTANCE )
	{
		B3_VALIDATE( distance1 > B3_SPECULATIVE_DISTANCE );

		b3Vec3 normal = refPlane.normal;
		b3Vec3 point = segment[1].position + 0.5f * ( capsule->radius + distance2 ) * normal;

		manifold.normal = b3RotateVector( xfA.q, normal );

		b3ManifoldPoint* mp = manifold.points + 0;
		mp->anchorA = b3RotateVector( xfA.q, point );
		mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
		mp->point = mp->anchorA + xfA.p;
		mp->separation = distance2 - capsule->radius;
		mp->id = b3MakeFeatureKey( segment[1].pair );

		manifold.pointCount = 1;
		return true;
	}

	return false;
}

static float b3DeepestPointSeparation( const b3Manifold& manifold )
{
	// Deepest point
	float minSeparation = FLT_MAX;
	for ( int index = 0; index < manifold.pointCount; ++index )
	{
		minSeparation = b3MinFloat( minSeparation, manifold.points[index].separation );
	}

	return minSeparation;
}

static bool b3BuildHullAndCapsuleEdgeContact( b3Manifold& manifold, const b3Hull* hull, b3Transform xfA, const b3Capsule* capsule,
											  b3Transform xfB, b3EdgeQuery query )
{
	manifold.pointCount = 0;

	b3Transform xf = b3InvMulTransforms( xfA, xfB );

	b3Vec3 pc = b3TransformPoint( xf, capsule->center1 );
	b3Vec3 qc = b3TransformPoint( xf, capsule->center2 );
	b3Vec3 ec = qc - pc;

	// this assertion is invalid, we arrive here because the distance is small or zero.
	// B3_ASSERT(Query.Separation <= 0.0f);

	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	const b3HullHalfEdge* edge2 = edges + query.indexB;
	const b3HullHalfEdge* twin2 = edges + edge2->twin;
	b3Vec3 ch = hull->center;
	b3Vec3 ph = points[edge2->origin];
	b3Vec3 qh = points[twin2->origin];
	b3Vec3 eh = qh - ph;

	b3Vec3 normal = b3Cross( ec, eh );
	normal = b3Normalize( normal );

	// Normal should point outward from hull
	if ( b3Dot( normal, ph - ch ) < 0.0f )
	{
		normal = -normal;
	}

	b3ClosestApproachResult result = b3ClosestApproachLines( ph, eh, pc, ec );

	if ( b3IsWithinSegments( &result ) == false )
	{
		// closest point beyond end points
		return false;
	}

	b3Vec3 point = 0.5f * ( result.point1 - capsule->radius * normal + result.point2 );

	float separation = b3Dot( normal, result.point2 - result.point1 );
	B3_VALIDATE( b3AbsFloat( separation - query.separation ) < B3_LINEAR_SLOP );

	manifold.normal = b3RotateVector( xfA.q, normal );

	b3ManifoldPoint* mp = manifold.points + 0;
	mp->anchorA = b3RotateVector( xfA.q, point );
	mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
	mp->point = mp->anchorA + xfA.p;
	mp->separation = separation - capsule->radius;
	mp->id = b3MakeFeatureKeyFull( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );

	manifold.pointCount = 1;
	return true;
}

b3Manifold b3CollideHullAndCapsule( const b3Hull* hull, b3Transform xfA, const b3Capsule* capsule, b3Transform xfB,
									b3SimplexCache* cache )
{
	b3Manifold manifold = {};

	b3Transform xf = b3InvMulTransforms( xfA, xfB );

	b3DistanceInput distanceInput;
	distanceInput.proxyA = { b3GetHullPoints( hull ), hull->vertexCount, 0.0f };
	distanceInput.proxyB = { &capsule->center1, 2, 0.0f };
	distanceInput.transformA = b3Transform_identity;
	distanceInput.transformB = xf;
	distanceInput.useRadii = false;

	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, cache, nullptr, 0 );

	if ( distanceOutput.distance > capsule->radius + B3_SPECULATIVE_DISTANCE )
	{
		// We found a separating axis
		cache = {};
		return manifold;
	}

	if ( distanceOutput.distance > 100.0f * FLT_EPSILON )
	{
		const b3Plane* planes = b3GetHullPlanes( hull );

		// Shallow penetration
		b3Vec3 delta = b3Normalize( distanceOutput.pointB - distanceOutput.pointA );
		int refFace = b3FindHullSupportFace( hull, delta );
		b3Plane refPlane = planes[refFace];

		// Try to create two contact points if closest
		// points difference is nearly parallel to face normal
		const float kTolerance = 0.998f;
		if ( b3AbsFloat( b3Dot( refPlane.normal, delta ) ) > kTolerance )
		{
			// Clip capsule segment against side planes of reference face
			b3ClipVertex segment[2];
			b3BuildCapsuleSegment( segment, capsule, xf );

			// todo get rid of identity transform
			if ( b3ClipSegmentToHullFace( segment, hull, b3Transform_identity, refFace ) )
			{
				float distance1 = b3PlaneSeparation( refPlane, segment[0].position );
				float distance2 = b3PlaneSeparation( refPlane, segment[1].position );
				if ( distance1 <= capsule->radius + B3_SPECULATIVE_DISTANCE ||
					 distance2 <= capsule->radius + B3_SPECULATIVE_DISTANCE )
				{
					b3Vec3 normal = refPlane.normal;
					b3Vec3 point1 = segment[0].position - 0.5f * ( capsule->radius + distance1 ) * normal;
					b3Vec3 point2 = segment[1].position - 0.5f * ( capsule->radius + distance2 ) * normal;

					manifold.normal = b3RotateVector( xfA.q, normal );

					b3ManifoldPoint* mp = manifold.points + 0;
					mp->anchorA = b3RotateVector( xfA.q, point1 );
					mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
					mp->point = mp->anchorA + xfA.p;
					mp->separation = distance1 - capsule->radius;
					mp->id = b3MakeFeatureKey( segment[0].pair );

					mp = manifold.points + 1;
					mp->anchorA = b3RotateVector( xfA.q, point2 );
					mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
					mp->point = mp->anchorA + xfA.p;
					mp->separation = distance2 - capsule->radius;
					mp->id = b3MakeFeatureKey( segment[1].pair );

					manifold.pointCount = 2;
					return manifold;
				}
			}
		}

		// Create contact from closest points
		b3Vec3 point = 0.5f * ( distanceOutput.pointA - capsule->radius * delta + distanceOutput.pointB );

		manifold.normal = b3RotateVector( xfA.q, delta );

		b3ManifoldPoint* mp = manifold.points + 0;
		mp->anchorA = b3RotateVector( xfA.q, point );
		mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
		mp->point = mp->anchorA + xfA.p;
		mp->separation = distanceOutput.distance - capsule->radius;
		mp->id = b3MakeFeatureKeyFull( b3_featureShapeA, 0, b3_featureShapeA, 0 );

		manifold.pointCount = 1;
		return manifold;
	}

	// Deep penetration

	b3FaceQuery faceQuery = b3QueryFaceDirectionHullAndCapsule( hull, capsule, xf );
	if ( faceQuery.separation > capsule->radius )
	{
		// We found a separating axis
		return manifold;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeDirectionHullAndCapsule( hull, capsule, xf );
	if ( edgeQuery.separation > capsule->radius )
	{
		// We found a separating axis
		return manifold;
	}

	// Create face contact
	float faceSeparation = faceQuery.separation - capsule->radius;
	b3BuildHullAndCapsuleFaceContact( manifold, hull, xfA, capsule, xfB, faceQuery );
	if ( manifold.pointCount > 1 )
	{
		// If ( Out.PointCount <= 1 ) -> Compare with unclipped separation
		// If ( Out.PointCount > 1 ) -> Be aggressive and compare with clipped separation
		// Face contact can be empty if it does not realize the axis of minimum penetration
		faceSeparation = b3DeepestPointSeparation( manifold );
	}
	B3_VALIDATE( faceSeparation <= 0.0f );

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	const float kRelEdgeTolerance = 0.90f;
	const float kAbsTolerance = 0.5f * B3_LINEAR_SLOP;
	float edgeSeparation = edgeQuery.separation - capsule->radius;
	if ( manifold.pointCount == 0 || edgeSeparation > kRelEdgeTolerance * faceSeparation + kAbsTolerance )
	{
		// Edge contact
		b3BuildHullAndCapsuleEdgeContact( manifold, hull, xfA, capsule, xfB, edgeQuery );
	}

	return manifold;
}

static bool b3BuildFaceContact( b3Manifold& manifold, const b3Hull* hullA, b3Transform xfA, const b3Hull* hullB, b3Transform xfB,
								b3FaceQuery query, bool flipNormal, b3SATCache* cache )
{
	const b3HullFace* facesA = b3GetHullFaces( hullA );
	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	int refFace = query.faceIndex;
	b3Plane refPlane = planesA[refFace];

	b3Transform xf = b3InvMulTransforms( xfA, xfB );
	b3Vec3 refNormalInB = b3InvRotateVector( xf.q, refPlane.normal );

	// Find incident face
	int incFace = b3FindIncidentFace( hullB, refNormalInB, query.vertexIndex );

	// Build clip polygon from incident face
	b3ClipVertex buffer1[2 * B3_MAX_CLIP_POINTS], buffer2[2 * B3_MAX_CLIP_POINTS];
	int pointCount = b3BuildPolygon( buffer1, xf, hullB, incFace, refPlane );

	// Clip incident face against side planes of reference face
	b3ClipVertex* input = buffer1;
	b3ClipVertex* output = buffer2;

	const b3HullFace* face = facesA + refFace;
	int edgeIndex = face->edge;

	do
	{
		const b3HullHalfEdge* edge = edgesA + edgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edgesA + nextEdgeIndex;
		b3Vec3 vertex1 = pointsA[edge->origin];
		b3Vec3 vertex2 = pointsA[next->origin];
		b3Vec3 tangent = b3Normalize( vertex2 - vertex1 );
		b3Vec3 binormal = b3Cross( tangent, refPlane.normal );

		b3Plane clipPlane = b3MakePlaneFromNormalAndPoint( binormal, vertex1 );

		pointCount = b3ClipPolygon( output, input, pointCount, clipPlane, edgeIndex, refPlane );
		B3_ASSERT( pointCount <= 2 * B3_MAX_CLIP_POINTS );

		B3_SWAP( output, input );

		if ( pointCount < 3 )
		{
			manifold.pointCount = 0;
			*cache = {};
			return false;
		}

		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != face->edge );

	pointCount = b3MinInt( pointCount, B3_MAX_CLIP_POINTS );

	b3ManifoldPoint points[B3_MAX_CLIP_POINTS];
	float minSeparation = FLT_MAX;

	for ( int i = 0; i < pointCount; ++i )
	{
		b3ClipVertex* clipPoint = input + i;
		b3ManifoldPoint* mp = points + i;
		*mp = {};

		b3Vec3 point = clipPoint->position - clipPoint->separation * refPlane.normal;

		if ( flipNormal )
		{
			mp->anchorB = b3RotateVector( xfA.q, point );
			mp->anchorA = mp->anchorB + ( xfA.p - xfB.p );
			mp->point = mp->anchorB + xfA.p;
		}
		else
		{
			mp->anchorA = b3RotateVector( xfA.q, point );
			mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
			mp->point = mp->anchorA + xfA.p;
		}

		mp->separation = clipPoint->separation;

		b3FeaturePair pair = flipNormal ? b3FlipPair( clipPoint->pair ) : clipPoint->pair;
		mp->id = b3MakeFeatureKey( pair );

		minSeparation = b3MinFloat( minSeparation, clipPoint->separation );
	}

	if ( minSeparation >= B3_SPECULATIVE_DISTANCE )
	{
		manifold.pointCount = 0;
		*cache = {};
		return false;
	}

	manifold.normal = flipNormal ? b3RotateVector( xfA.q, -refPlane.normal ) : b3RotateVector( xfA.q, refPlane.normal );

	b3ReduceManifoldPoints( &manifold, points, pointCount );

	// Save cache
	cache->separation = query.separation;
	cache->type = flipNormal ? (uint8_t)b3_faceNormalB : (uint8_t)b3_faceNormalA;
	cache->indexA = flipNormal ? (uint8_t)query.vertexIndex : ( uint8_t ) query.faceIndex;
	cache->indexB = flipNormal ? (uint8_t)query.faceIndex : (uint8_t)query.vertexIndex;

	return true;
}

static bool b3BuildEdgeContact( b3Manifold& manifold, const b3Hull* hullA, b3Transform xfA, const b3Hull* hullB, b3Transform xfB,
								b3EdgeQuery query, b3SATCache* cache )
{
	manifold.pointCount = 0;

	b3Transform xf = b3InvMulTransforms( xfA, xfB );

	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	B3_VALIDATE( query.separation <= 2.0f * B3_SPECULATIVE_DISTANCE );

	const b3HullHalfEdge* edge1 = edgesA + query.indexA;
	const b3HullHalfEdge* twin1 = edgesA + edge1->twin;
	b3Vec3 c1 = hullA->center;
	b3Vec3 p1 = pointsA[edge1->origin];
	b3Vec3 q1 = pointsA[twin1->origin];
	b3Vec3 e1 = q1 - p1;

	const b3HullHalfEdge* edge2 = edgesB + query.indexB;
	const b3HullHalfEdge* twin2 = edgesB + edge2->twin;
	b3Vec3 p2 = b3TransformPoint( xf, pointsB[edge2->origin] );
	b3Vec3 q2 = b3TransformPoint( xf, pointsB[twin2->origin] );
	b3Vec3 e2 = q2 - p2;

	b3Vec3 normal = b3Cross( e1, e2 );
	normal = b3Normalize( normal );

	if ( b3Dot( normal, p1 - c1 ) < 0.0f )
	{
		normal = -normal;
	}

	b3ClosestApproachResult result = b3ClosestApproachLines( p1, e1, p2, e2 );

	if ( !b3IsWithinSegments( &result ) )
	{
		*cache = {};
		return false;
	}

	// This can slide off the end from caching
	float separation = b3Dot( normal, result.point2 - result.point1 );
	B3_VALIDATE( b3AbsFloat( separation - query.separation ) < B3_LINEAR_SLOP );

	b3Vec3 point = 0.5f * ( result.point1 + result.point2 );

	manifold.normal = b3RotateVector( xfA.q, normal );

	b3ManifoldPoint* mp = manifold.points + 0;
	mp->anchorA = b3RotateVector( xfA.q, point );
	mp->anchorB = mp->anchorA + ( xfA.p - xfB.p );
	mp->separation = separation;
	mp->point = mp->anchorA + xfA.p;
	mp->id = b3MakeFeatureKeyFull( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );

	manifold.pointCount = 1;

	// Save cache
	cache->separation = separation;
	cache->type = (uint8_t)b3_edgePairAxis;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;

	return true;
}

static bool b3QueryLastFeatures( b3Manifold& out, const b3Hull* hullA, b3Transform xfA, const b3Hull* hullB, b3Transform xfB,
								 b3SATCache* cache )
{
	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Plane* planesB = b3GetHullPlanes( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	b3Transform xf = b3InvMulTransforms( xfA, xfB );
	float linearSlop = B3_LINEAR_SLOP;
	float speculativeDistance = B3_SPECULATIVE_DISTANCE;

	switch ( cache->type )
	{
		case b3_faceNormalA:
		{
			B3_ASSERT( cache->indexA < hullA->faceCount );

			// We perform all computations in local space of the second hull
			b3Plane plane = b3InvTransformPlane( xf, planesA[cache->indexA] );
			int vertexIndex = b3FindHullSupportVertex( hullB, -plane.normal );
			b3Vec3 support = pointsB[vertexIndex];
			float separation = b3PlaneSeparation( plane, support );

			// DIRK_TODO: With GJK based manifolds we could be more *strict* and
			// just measure the separation of the last face and deepest vertex. This
			// might help with creating false contacts from the cache. E.g.
			// float Separation = rnDistance( Plane, Hull2->GetPosition( Cache.Index2 );

			if ( separation >= speculativeDistance )
			{
				// We found a separating axis
				out.pointCount = 0;

				// Update cache
				cache->separation = separation;
				cache->indexB = (uint8_t)vertexIndex;

				return true;
			}

			if ( cache->separation >= speculativeDistance )
			{
				// Objects were separated and are now potentially penetrating - cache is invalidated
				cache = {};

				return false;
			}

			if ( b3AbsFloat( cache->separation - separation ) < linearSlop )
			{
				// Try to rebuild contact from last separating plane and new deepest point.
				// The deepest point can change when we roll over the surface (e.g. cylinder).
				b3FaceQuery faceQuery;
				faceQuery.separation = separation;
				faceQuery.faceIndex = cache->indexA;
				faceQuery.vertexIndex = vertexIndex;

				// We need to run a full test if we clipped all contact points away
				bool flip = false;
				return b3BuildFaceContact( out, hullA, xfA, hullB, xfB, faceQuery, flip, cache );
			}
		}
		break;

		case b3_faceNormalB:
		{
			B3_ASSERT( cache->indexB < hullB->faceCount );

			// We perform all computations in local space of the first hull
			b3Plane plane = b3TransformPlane( xf, planesB[cache->indexB] );
			int vertexIndex = b3FindHullSupportVertex( hullA, -plane.normal );
			b3Vec3 support = pointsA[vertexIndex];
			float separation = b3PlaneSeparation( plane, support );

			if ( separation >= speculativeDistance )
			{
				// We found a separating axis
				out.pointCount = 0;

				// Update cache
				cache->separation = separation;
				cache->indexA = (uint8_t)vertexIndex;

				return true;
			}

			if ( cache->separation >= speculativeDistance )
			{
				// Objects were separated and are now potentially penetrating - cache is invalidated
				cache = {};

				return false;
			}

			if ( b3AbsFloat( cache->separation - separation ) < linearSlop )
			{
				// Try to rebuild contact from last features
				b3FaceQuery faceQuery;
				faceQuery.separation = separation;
				faceQuery.faceIndex = cache->indexB;
				faceQuery.vertexIndex = vertexIndex;

				// We need to run a full test if we clipped all contact points away
				bool flip = true;
				b3BuildFaceContact( out, hullB, xfB, hullA, xfA, faceQuery, flip, cache );
			}
		}
		break;

		case b3_edgePairAxis:
		{
			// We perform all computations in local space of the second hull
			b3Transform transform = b3InvertTransform( xf );

			int index1 = cache->indexA;
			const b3HullHalfEdge* edge1 = edgesA + index1;
			const b3HullHalfEdge* twin1 = edgesA + index1 + 1;
			B3_ASSERT( edge1->twin == index1 + 1 && twin1->twin == index1 );

			b3Vec3 p1 = b3TransformPoint( transform, pointsA[edge1->origin] );
			b3Vec3 q1 = b3TransformPoint( transform, pointsA[twin1->origin] );
			b3Vec3 e1 = q1 - p1;

			b3Vec3 u1 = b3RotateVector( transform.q, planesA[edge1->face].normal );
			b3Vec3 v1 = b3RotateVector( transform.q, planesA[twin1->face].normal );

			int index2 = cache->indexB;
			const b3HullHalfEdge* edge2 = edgesB + index2;
			const b3HullHalfEdge* twin2 = edgesB + index2 + 1;
			B3_ASSERT( edge2->twin == index2 + 1 && twin2->twin == index2 );

			b3Vec3 p2 = pointsB[edge2->origin];
			b3Vec3 q2 = pointsB[twin2->origin];
			b3Vec3 e2 = q2 - p2;

			b3Vec3 u2 = planesB[edge2->face].normal;
			b3Vec3 v2 = planesB[twin2->face].normal;

			// flipping the signs of u2 and v2
			// cross(v2, u2) == cross(-v2, -u2)
			// so we still use -e2
			// but we can also use e1 = cross(u1, v1) and e2 = cross(u2, v2)
			if ( b3IsMinkowskiFace( u1, v1, e1, -u2, -v2, e2 ) )
			{
				// Transform reference center of the first hull into local space of the second hull
				b3Vec3 c1 = b3TransformPoint( transform, hullA->center );
				b3Vec3 c2 = hullB->center;

				float separation = b3EdgeEdgeSeparation( p1, e1, c1, p2, e2, c2 );
				if ( separation > speculativeDistance )
				{
					// We found a separating axis
					out.pointCount = 0;

					cache->separation = separation;

					return true;
				}

				if ( cache->separation > speculativeDistance )
				{
					// Objects were separated and are now potentially penetrating
					cache = {};

					return false;
				}

				if ( b3AbsFloat( cache->separation - separation ) < linearSlop )
				{
					// Try to rebuild contact from last features
					b3EdgeQuery edgeQuery;
					edgeQuery.indexA = cache->indexA;
					edgeQuery.indexB = cache->indexB;
					edgeQuery.separation = separation;

					// We need to run a full test if we clipped all contact points away
					return b3BuildEdgeContact( out, hullA, xfA, hullB, xfB, edgeQuery, cache );
				}
			}
		}
		break;

		default:
			break;
	}

	return false;
}

b3Manifold b3CollideHulls( const b3Hull* hullA, b3Transform xfA, const b3Hull* hullB, b3Transform xfB, b3SATCache* cache )
{
	b3Manifold manifold = {};

	// Query last features
	cache->hit = 1;
	if ( b3QueryLastFeatures( manifold, hullA, xfA, hullB, xfB, cache ) )
	{
		// SUCCESS: We rebuilt the manifold from the feature cache and can exit early!
		return manifold;
	}

	cache->hit = 0;
	const float speculativeDistance = B3_SPECULATIVE_DISTANCE;

	b3Transform xf = b3InvMulTransforms( xfA, xfB );

	// Find axis of minimum penetration
	b3FaceQuery faceQueryA = b3QueryFaceDirections( hullA, hullB, xf );
	if ( faceQueryA.separation > speculativeDistance )
	{
		B3_ASSERT( faceQueryA.faceIndex < hullA->faceCount );
		B3_ASSERT( faceQueryA.vertexIndex < hullB->vertexCount );

		// We found a separating axis
		cache->separation = faceQueryA.separation;
		cache->type = (uint8_t)b3_faceNormalA;
		cache->indexA = (uint8_t)faceQueryA.faceIndex;
		cache->indexB = (uint8_t)faceQueryA.vertexIndex;
		return manifold;
	}

	b3FaceQuery faceQueryB = b3QueryFaceDirections( hullB, hullA, b3InvertTransform( xf ) );
	if ( faceQueryB.separation > speculativeDistance )
	{
		B3_ASSERT( faceQueryB.faceIndex < hullB->faceCount );
		B3_ASSERT( faceQueryB.vertexIndex < hullA->vertexCount );

		// We found a separating axis
		cache->separation = faceQueryB.separation;
		cache->type = (uint8_t)b3_faceNormalB;
		cache->indexA = (uint8_t)faceQueryB.vertexIndex;
		cache->indexB = (uint8_t)faceQueryB.faceIndex;
		return manifold;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeDirections( hullA, hullB, xf );
	if ( edgeQuery.separation > speculativeDistance )
	{
		// We found a separating axis
		cache->separation = edgeQuery.separation;
		cache->type = (uint8_t)b3_edgePairAxis;
		cache->indexA = (uint8_t)edgeQuery.indexA;
		cache->indexB = (uint8_t)edgeQuery.indexB;
		return manifold;
	}

	const float kRelEdgeTolerance = 0.90f;
	const float kRelFaceTolerance = 0.98f;
	const float kAbsTolerance = 0.5f * B3_LINEAR_SLOP;

	// Always build a face contact (e.g. Jenga problem)
	float faceSeparationA = faceQueryA.separation;
	float faceSeparationB = faceQueryB.separation;
	B3_VALIDATE( faceSeparationA <= speculativeDistance && faceSeparationB <= speculativeDistance );

	if ( faceSeparationB > kRelFaceTolerance * faceSeparationA + kAbsTolerance )
	{
		// Face contact B
		bool flip = true;
		b3BuildFaceContact( manifold, hullB, xfB, hullA, xfA, faceQueryB, flip, cache );
	}
	else
	{
		// Face contact A
		bool flip = false;
		b3BuildFaceContact( manifold, hullA, xfA, hullB, xfB, faceQueryA, flip, cache );
	}

	// todo debugging
	b3Manifold temp = manifold;
	(void)temp;

	if (edgeQuery.indexA == B3_NULL_INDEX)
	{
		// There are no valid edge pairs (all edges parallel)
		return manifold;
	}

	// If ( Out.PointCount <= 1 ) -> Compare with unclipped separation
	// If ( Out.PointCount > 1 ) -> Be aggressive and compare with clipped separation
	float faceSeparation = b3MaxFloat( faceSeparationA, faceSeparationB );
	if ( manifold.pointCount > 1 )
	{
		// Deepest point
		faceSeparation = FLT_MAX;
		for ( int index = 0; index < manifold.pointCount; ++index )
		{
			faceSeparation = b3MinFloat( faceSeparation, manifold.points[index].separation );
		}
	}

	// todo this can fail because I'm using SAT in the separating case for speculative.
	// Face contact can get positive separation after clipping if edge contact is the true separating axis.
	//B3_ASSERT( faceSeparation <= speculativeDistance ||
	//		   (edgeQuery.Separation >= faceQuery1.Separation && edgeQuery.Separation >= faceQuery2.Separation) );

	float edgeSeparation = edgeQuery.separation;
	B3_VALIDATE( edgeSeparation <= speculativeDistance );

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	if ( manifold.pointCount == 0 || edgeSeparation > kRelEdgeTolerance * faceSeparation + kAbsTolerance )
	{
		// Edge contact
		b3Manifold backup = manifold;

		b3BuildEdgeContact( manifold, hullA, xfA, hullB, xfB, edgeQuery, cache );
		if ( manifold.pointCount == 0 )
		{
			// Use face manifold if we fail to create an edge contact!
			manifold = backup;
		}
	}

	return manifold;
}
