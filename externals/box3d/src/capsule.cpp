// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "constants.h"
#include "math_internal.h"
#include "shape.h"

b3MassData b3ComputeCapsuleMass( const b3Capsule* shape, float density )
{
	b3Vec3 c1 = shape->center1;
	b3Vec3 c2 = shape->center2;
	float r = shape->radius;

	// Cylinder
	float cylinderHeight = b3Distance( c1, c2 );
	float cylinderVolume = B3_PI * r * r * cylinderHeight;
	float cylinderMass = cylinderVolume * density;

	// Sphere
	float sphereVolume = ( 4.0f / 3.0f ) * B3_PI * r * r * r;
	float sphereMass = sphereVolume * density;

	// Local accumulated inertia
	b3Matrix3 inertia = b3CylinderInertia( cylinderMass, r, cylinderHeight ) + b3SphereInertia( sphereMass, r );

	float steiner = 0.125f * sphereMass * ( 3.0f * r + 2.0f * cylinderHeight ) * cylinderHeight;
	inertia.cx.x += steiner;
	inertia.cz.z += steiner;

	// Align capsule axis with chosen up-axis
	b3Matrix3 rotation = b3Mat3_identity;
	if ( cylinderHeight * cylinderHeight > 1000.0f * FLT_MIN )
	{
		b3Vec3 direction = b3Normalize( c2 - c1 );
		b3Quat q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, direction );
		rotation = b3MakeMatrixFromQuat( q );
	}

	// Parallel Axis Theorem capsule
	float mass = sphereMass + cylinderMass;
	b3Vec3 center = 0.5f * ( c1 + c2 );

	b3MassData out;
	out.mass = mass;
	out.center = center;
	out.inertia = b3MulMM( rotation, b3MulMM( inertia, b3Transpose( rotation ) ) ) + b3Steiner( mass, center );

	return out;
}

b3AABB b3ComputeCapsuleAABB( const b3Capsule* shape, b3Transform transform )
{
	float r = shape->radius;

	b3Vec3 center1 = b3TransformPoint( transform, shape->center1 );
	b3Vec3 center2 = b3TransformPoint( transform, shape->center2 );
	b3Vec3 extent = { r, r, r };

	b3AABB aabb;
	aabb.lowerBound = b3Min( center1, center2 ) - extent;
	aabb.upperBound = b3Max( center1, center2 ) + extent;
	return aabb;
}

b3AABB b3ComputeSweptCapsuleAABB( const b3Capsule* shape, b3Transform xf1, b3Transform xf2 )
{
	b3Vec3 r = { shape->radius, shape->radius, shape->radius };
	b3Vec3 a = b3TransformPoint( xf1, shape->center1 );
	b3Vec3 b = b3TransformPoint( xf1, shape->center2 );
	b3Vec3 c = b3TransformPoint( xf2, shape->center1 );
	b3Vec3 d = b3TransformPoint( xf2, shape->center2 );

	b3AABB aabb = {
		.lowerBound = b3Min( b3Min( a, b ), b3Min( c, d ) ) - r,
		.upperBound = b3Max( b3Max( a, b ), b3Max( c, d ) ) + r,
	};
	return aabb;
}

static b3CastOutput b3CastRayAgainstCap( b3Vec3 p, b3Vec3 q, b3Vec3 c, float r, float maxFraction )
{
	b3Sphere sphere = { c, r };
	b3RayCastInput input = {
		.origin = p,
		.translation = q - p,
		.maxFraction = maxFraction,
	};

	b3CastOutput output = b3RayCastSphere( &sphere, &input );
	return output;
}

static b3CastOutput b3TransformCapsuleResult( const b3CastOutput& result, const b3Quat& rotation, b3Vec3 translation )
{
	b3CastOutput out = result;
	out.point = b3RotateVector( rotation, result.point ) + translation;
	out.normal = b3RotateVector( rotation, result.normal );
	return out;
}

bool b3OverlapCapsule( const b3Capsule* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy )
{
	b3DistanceInput input;
	input.proxyA = b3ShapeProxy{ &shape->center1, 2, shape->radius };
	input.proxyB = *proxy;
	input.transformA = shapeTransform;
	input.transformB = b3Transform_identity;
	input.useRadii = true;

	b3SimplexCache cache = {};
	b3DistanceOutput output = b3ShapeDistance( &input, &cache, nullptr, 0 );
	return output.distance < B3_OVERLAP_SLOP;
}

// todo_erin implement precision improvement
// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
b3CastOutput b3RayCastCapsule( const b3Capsule* shape, const b3RayCastInput* input )
{
	b3Vec3 c1 = shape->center1;
	b3Vec3 c2 = shape->center2;
	float r = shape->radius;

	// Initialize result structure
	b3CastOutput output = {};

	// Compute height and handle degenerate capsules
	float height = b3Distance( c1, c2 );
	if ( height < 1000.0f * FLT_MIN )
	{
		b3Sphere sphere = { 0.5f * ( shape->center1 + shape->center2 ), shape->radius };
		return b3RayCastSphere( &sphere, input );
	}

	// Transform ray and capsule into local space capsule space
	b3Quat rotation = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, ( 1.0f / height ) * ( c2 - c1 ) );
	b3Vec3 offset = c1;

	// Capsule starts at the origin and is along the y-axis
	b3Vec3 a = b3Vec3_zero;
	b3Vec3 b = { 0.0f, height, 0.0f };
	b3Vec3 ab = b - a;

	// Ray expressed relative to capsule space (capsule along y-axis)
	b3Vec3 p = b3InvRotateVector( rotation, input->origin - offset );
	b3Vec3 q = p + b3InvRotateVector( rotation, input->translation );
	b3Vec3 pq = q - p;

	// Ray 2D translation length squared
	float k1 = pq.x * pq.x + pq.z * pq.z;

	// Ray start point 2D separation squared from circle
	float k3 = p.x * p.x + p.z * p.z - r * r;

	// Parallel case (2D ray translation is zero)
	if ( k1 < 1000.0f * FLT_MIN )
	{
		if ( k3 > 0.0f )
		{
			// Parallel and outside
			return output;
		}

		if ( 0.0f <= p.y && p.y <= height )
		{
			// Parallel and inside
			output.hit = true;
			output.point = input->origin;
			return output;
		}

		// Below cylinder and casting upwards
		if ( p.y < 0.0f && pq.y > 0.0f )
		{
			output = b3CastRayAgainstCap( p, q, a, r, input->maxFraction );
			return b3TransformCapsuleResult( output, rotation, offset );
		}

		// Above cylinder and casting downwards
		if ( p.y > height && pq.y < 0.0f )
		{
			output = b3CastRayAgainstCap( p, q, b, r, input->maxFraction );
			return b3TransformCapsuleResult( output, rotation, offset );
		}

		// Above or below and casting away from cylinder
		return output;
	}

	// Non-parallel case
	float k2 = pq.x * p.x + pq.z * p.z;

	float discriminant = k2 * k2 - k1 * k3;
	if ( discriminant < 0.0f )
	{
		// No real roots - no intersection
		return output;
	}

	float t = ( -k2 - sqrtf( discriminant ) ) / k1;
	if ( t > input->maxFraction )
	{
		// Segment approaching cylinder, but not quite getting there.
		return output;
	}

	// Don't skip t < 0. This means that we start in the *infinite* cylinder and still might hit a cap

	// This is the point on the ray that hits the infinite cylinder
	b3Vec3 c = p + t * pq;

	// This is the cylinder hit point relative to the capsule base
	b3Vec3 ac = c - a;

	// Fraction of the cylinder hit point along the capsule axis
	float s = b3Dot( ac, ab ) / ( height * height );

	if ( s < 0.0f )
	{
		// X projects outside A, run test through sphere at A
		output = b3CastRayAgainstCap( p, q, a, r, input->maxFraction );
		return b3TransformCapsuleResult( output, rotation, offset );
	}

	if ( s > 1.0f )
	{
		// X projects outside B, run test through sphere at B
		output = b3CastRayAgainstCap( p, q, b, r, input->maxFraction );
		return b3TransformCapsuleResult( output, rotation, offset );
	}

	if ( t < 0.0f )
	{
		// Ray starts inside
		output.hit = true;
		output.point = input->origin;
		return output;
	}

	// Ray hits cylinder inside segment AB
	output.fraction = t;
	output.point = c;
	output.normal = b3Normalize( { c.x, 0.0f, c.z } );
	output.hit = true;

	return b3TransformCapsuleResult( output, rotation, offset );
}

b3CastOutput b3ShapeCastCapsule( const b3Capsule* capsule, const b3ShapeCastInput* input )
{
	b3ShapeCastPairInput pairInput;
	pairInput.proxyA = { &capsule->center1, 2, capsule->radius };
	pairInput.proxyB = input->proxy;
	pairInput.transformA = b3Transform_identity;
	pairInput.transformB = b3Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.maxFraction = input->maxFraction;
	pairInput.canEncroach = input->canEncroach;

	b3CastOutput output = b3ShapeCast( &pairInput );
	return output;
}

int b3CollideMoverAndCapsule( b3PlaneResult* result, const b3Capsule* shape, const b3Capsule* mover )
{
	b3DistanceInput distanceInput;
	distanceInput.proxyA = { &shape->center1, 2, 0.0f };
	distanceInput.proxyB = { &mover->center1, 2, mover->radius };
	distanceInput.transformA = b3Transform_identity;
	distanceInput.transformB = b3Transform_identity;
	distanceInput.useRadii = false;

	float totalRadius = mover->radius + shape->radius;

	b3SimplexCache cache = {};
	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, &cache, nullptr, 0 );

	if ( distanceOutput.distance <= totalRadius )
	{
		b3Plane plane = { distanceOutput.normal, totalRadius - distanceOutput.distance };
		*result = b3PlaneResult{ plane, distanceOutput.pointA };
		return 1;
	}

	return 0;
}
