// SPDX-FileCopyrightText: 2025 Dirk Gregorius and Erin Catto
// SPDX-License-Identifier: MIT

#include "contact.h"
#include "physics_world.h"
#include "qsort.h"
#include "sat.h"
#include "shape.h"
#include "stack_array.h"

#include "box3d/types.h"

#include <stdio.h>

// mesh contact ideas:
// - prefer face contact when reducing?

// The maximum number of k-means clustering iteration for building mesh contact clusters.
#define B3_MAX_K_MEANS_ITERATIONS 8

#define B3_NORMAL_THRESHOLD 0.995f

#if B3_ENABLE_VALIDATION
static bool b3IsSorted( const int* array, int count )
{
	for ( int i = 0; i < count - 1; ++i )
	{
		if ( array[i] >= array[i + 1] )
		{
			return false;
		}
	}

	return true;
}
#endif

struct b3MeshTriangleIndexContext
{
	b3StackArray<int, 128>* triangleIndices;
};

static bool b3CollectTriangleIndicesCallback( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	B3_UNUSED( a, b, c );
	b3MeshTriangleIndexContext* triangleContext = static_cast<b3MeshTriangleIndexContext*>( context );
	triangleContext->triangleIndices->PushBack( triangleIndex );
	return true;
}

static void b3QueryMeshTriangles( b3StackArray<int, 128>& triangleIndices, const b3Mesh* mesh, const b3AABB& bounds )
{
	triangleIndices.Clear();

	b3MeshTriangleIndexContext context = { .triangleIndices = &triangleIndices };
	b3QueryMesh( mesh, bounds, b3CollectTriangleIndicesCallback, &context );
}

struct b3HeightFieldTriangleIndexContext
{
	b3StackArray<int, 128>* triangleIndices;
};

static bool b3CollectHeightFieldTriangleIndicesCallback( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	B3_UNUSED( a, b, c );
	b3HeightFieldTriangleIndexContext* triangleContext = static_cast<b3HeightFieldTriangleIndexContext*>( context );
	triangleContext->triangleIndices->PushBack( triangleIndex );
	return true;
}

static void b3QueryHeightFieldTriangles( b3StackArray<int, 128>& triangleIndices, const b3HeightField* heightField,
								  const b3AABB& bounds )
{
	triangleIndices.Clear();

	b3HeightFieldTriangleIndexContext context = { .triangleIndices = &triangleIndices };
	b3QueryHeightField( heightField, bounds, b3CollectHeightFieldTriangleIndicesCallback, &context );
}

static void b3RefreshCache( b3ContactSim* sim, const b3Shape* shapeA, b3Transform xfA, const b3AABB& bounds )
{
	B3_ASSERT( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape );

	// If the dynamic body didn't move out of the cached query bounds we are done!
	if ( b3AABB_Contains( sim->meshQueryBounds, bounds ) )
	{
		if ( shapeA->type == b3_meshShape )
		{
			for ( int i = 0; i < sim->triangleCache.count; ++i )
			{
				B3_ASSERT( 0 <= sim->triangleCache[i].triangleIndex &&
						   sim->triangleCache[i].triangleIndex < shapeA->mesh.data->triangleCount );
			}
		}

		return;
	}

	// Inflate query bounds to account for convex radius (the mesh BVH does not contain
	// any inflation in its nodes to keep it tight). We additionally inflate the query
	// bounds to avoid constant re-traversal of the tree when objects come to rest. We
	// add both here for explicitness and readability. Practically the bounds extension
	// alone should be sufficient, but we might potentially overlook that two unrelated
	// things are happening here.
	float radius = B3_AABB_MARGIN + B3_SPECULATIVE_DISTANCE;
	b3Vec3 extension = { radius, radius, radius };
	sim->meshQueryBounds.lowerBound = bounds.lowerBound - extension;
	sim->meshQueryBounds.upperBound = bounds.upperBound + extension;

	// Re-query triangles
	b3StackArray<int, 128> triangleIndices;

	// Bounds are in world space. Convert to the local mesh frame.
	b3AABB localBounds = b3AABB_Transform( b3InvertTransform( xfA ), sim->meshQueryBounds );
	if ( shapeA->type == b3_meshShape )
	{
		b3QueryMeshTriangles( triangleIndices, &shapeA->mesh, localBounds );
	}
	else
	{
		B3_ASSERT( shapeA->type == b3_heightShape );
		b3QueryHeightFieldTriangles( triangleIndices, shapeA->heightField, localBounds );
	}

	// Triangle indices must be sorted to match caches.
#if B3_ENABLE_VALIDATION
	B3_ASSERT( b3IsSorted( triangleIndices.m_data, triangleIndices.m_count ) );
#endif

	// Create new contact cache and match with old one
	b3StackArray<b3ContactCache, 128> contactCache;
	contactCache.Resize( triangleIndices.m_count );

	int index2 = 0;
	for ( int index1 = 0; index1 < triangleIndices.m_count; ++index1 )
	{
		contactCache[index1] = {};

		while ( index2 < sim->triangleCache.count && sim->triangleCache[index2].triangleIndex < triangleIndices[index1] )
		{
			index2 += 1;
		}

		if ( index2 < sim->triangleCache.count && sim->triangleCache[index2].triangleIndex == triangleIndices[index1] )
		{
			contactCache[index1] = sim->triangleCache[index2].cache;
		}
	}

	// Save new cache
	sim->triangleCache.Resize( triangleIndices.m_count );
	for ( int i = 0; i < triangleIndices.m_count; ++i )
	{
		sim->triangleCache[i] = { triangleIndices[i], contactCache[i] };

		if ( shapeA->type == b3_meshShape )
		{
			B3_ASSERT( 0 <= sim->triangleCache[i].triangleIndex &&
					   sim->triangleCache[i].triangleIndex < shapeA->mesh.data->triangleCount );
		}
	}
}

static int b3BuildTriangleClusters( b3Vec3* clusterNormals, int* memberships, const b3TriangleManifold* manifolds,
									int manifoldCount )
{
	static_assert( B3_MAX_MANIFOLDS == 3, "max manifolds" );

	if ( manifoldCount == 0 )
	{
		return 0;
	}

	for ( int i = 0; i < manifoldCount; ++i )
	{
		memberships[i] = B3_NULL_INDEX;
	}

	clusterNormals[0] = manifolds[0].base.normal;
	memberships[0] = 0;
	int clusterCount = 1;
	for (int i = 1; i < manifoldCount; ++i)
	{
		b3Vec3 normal = manifolds[i].base.normal;
		bool foundMatch = false;
		for (int j = 0; j < clusterCount; ++j)
		{
			float dot = b3Dot( normal, clusterNormals[j] );
			if (dot > B3_NORMAL_THRESHOLD)
			{
				// assign membership to speed up k-means convergence
				memberships[i] = j;
				foundMatch = true;
				break;
			}
		}

		if (foundMatch == false)
		{
			clusterNormals[clusterCount] = normal;
			clusterCount += 1;
			if (clusterCount == B3_MAX_MANIFOLDS)
			{
				break;
			}
		}
	}

	if (clusterCount == 0)
	{
		// memberships should all be assigned to cluster 0
		return 1;
	}

	// Run k-means clustering where k == 2 or 3
	for ( int iteration = 0; iteration < B3_MAX_K_MEANS_ITERATIONS; ++iteration )
	{
		b3Vec3 normalAccumulators[B3_MAX_MANIFOLDS] = {};

		// Phase 1: Assignment step
		bool converged = true;
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			const b3Manifold& manifold = manifolds[manifoldIndex].base;
			B3_ASSERT( manifold.pointCount != 0 );

			float bestDot = -FLT_MAX;
			int bestClusterIndex = B3_NULL_INDEX;

			for ( int clusterIndex = 0; clusterIndex < clusterCount; ++clusterIndex )
			{
				b3Vec3 clusterNormal = clusterNormals[clusterIndex];

				float dot = b3Dot( clusterNormal, manifold.normal );
				if ( dot > bestDot )
				{
					bestDot = dot;
					bestClusterIndex = clusterIndex;
				}
			}

			B3_ASSERT( 0 <= bestClusterIndex && bestClusterIndex < clusterCount );

			if ( memberships[manifoldIndex] != bestClusterIndex )
			{
				converged = false;
			}

			normalAccumulators[bestClusterIndex] += manifold.normal;
			memberships[manifoldIndex] = bestClusterIndex;
		}

		if ( converged )
		{
			break;
		}

		// Phase 2: Update step
		for ( int i = 0; i < clusterCount; ++i )
		{
			clusterNormals[i] = b3Normalize( normalAccumulators[i] );
		}
	}

	return clusterCount;
}

static float b3FindImpulse( const b3ManifoldPoint& point, const b3Manifold& manifold )
{
	for ( int pointIndex = 0; pointIndex < manifold.pointCount; ++pointIndex )
	{
		if ( manifold.points[pointIndex].triangleIndex == point.triangleIndex && manifold.points[pointIndex].id == point.id )
		{
			return manifold.points[pointIndex].normalImpulse;
		}
	}

	return -1.0f;
}

static void b3MatchManifolds( b3Manifold* newManifolds, int newCount, b3Manifold* oldManifolds, int oldCount )
{
	// Match manifolds by normal
	bool consumed[B3_MAX_MANIFOLDS] = { false };
	for ( int newManifoldIndex = 0; newManifoldIndex < newCount; ++newManifoldIndex )
	{
		b3Manifold& newManifold = newManifolds[newManifoldIndex];

		for ( int oldManifoldIndex = 0; oldManifoldIndex < oldCount; ++oldManifoldIndex )
		{
			const b3Manifold& oldManifold = oldManifolds[oldManifoldIndex];
			if ( !consumed[oldManifoldIndex] && b3Dot( oldManifold.normal, newManifold.normal ) >= B3_NORMAL_THRESHOLD )
			{
				// Match normal impulses
				for ( int newPointIndex = 0; newPointIndex < newManifold.pointCount; ++newPointIndex )
				{
					b3ManifoldPoint& newPoint = newManifold.points[newPointIndex];
					B3_ASSERT( newPoint.normalImpulse == 0.0f );

					float impulse = b3FindImpulse( newPoint, oldManifold );
					if ( impulse >= 0.0f )
					{
						newPoint.normalImpulse = impulse;
						newPoint.persisted = true;
					}
				}

				newManifold.frictionImpulse = oldManifold.frictionImpulse;
				newManifold.rollingImpulse = oldManifold.rollingImpulse;
				newManifold.frictionImpulse = oldManifold.frictionImpulse;

				// Mark this manifold as consumed so we don't match it again!
				consumed[oldManifoldIndex] = true;
				break;
			}
		}
	}
}

struct b3TentativeTriangle
{
	float squaredDistance;
	int index;
};

#define B3_MAX_EDGE_COUNT 64

struct b3FoundEdges
{
	uint64_t keys[B3_MAX_EDGE_COUNT];
	int count;
};

static inline bool b3AddEdge( b3FoundEdges* edges, int vertex1, int vertex2 )
{
	uint64_t i1 = (uint64_t)b3MinInt( vertex1, vertex2 );
	uint64_t i2 = (uint64_t)b3MaxInt( vertex1, vertex2 );
	uint64_t key = i1 << 32 | i2;

	int count = edges->count;
	for ( int i = 0; i < count; ++i )
	{
		if ( edges->keys[i] == key )
		{
			return false;
		}
	}

	if ( count == B3_MAX_EDGE_COUNT )
	{
		// This will lead to a potential ghost collision
		return true;
	}

	edges->keys[count] = key;
	edges->count += 1;

	return true;
}

static inline bool b3FindEdge( b3FoundEdges* edges, int vertex1, int vertex2 )
{
	uint64_t i1 = (uint64_t)b3MinInt( vertex1, vertex2 );
	uint64_t i2 = (uint64_t)b3MaxInt( vertex1, vertex2 );
	uint64_t key = i1 << 32 | i2;

	int count = edges->count;
	for ( int i = 0; i < count; ++i )
	{
		if ( edges->keys[i] == key )
		{
			return true;
		}
	}

	return false;
}

#define B3_MAX_VERTEX_COUNT 64

struct b3FoundVertices
{
	int keys[B3_MAX_VERTEX_COUNT];
	int count;
};

static inline bool b3AddVertex( b3FoundVertices* vertices, int vertex )
{
	int key = vertex;

	int count = vertices->count;
	for ( int i = 0; i < count; ++i )
	{
		if ( vertices->keys[i] == key )
		{
			return false;
		}
	}

	if ( count == B3_MAX_VERTEX_COUNT )
	{
		// This will lead to a potential ghost collision
		return true;
	}

	vertices->keys[count] = key;
	vertices->count += 1;

	return true;
}

// todo testing
#if 0
b3Manifold b3CollideTriangleAndHullOld( const b3Vec3* triangle, int flags, const b3Hull* hull, b3SATCache* cache,
										b3TriangleFeature* feature, int triangleIndex )
{
	B3_UNUSED( flags, triangleIndex );
	*feature = b3_featureTriangleFace;

	// Delegate to general convex/convex collision (SAT)
	b3TriangleHull triangleHull = b3MakeTriangleHull( triangle[0], triangle[1], triangle[2] );
	return b3CollideHulls( &triangleHull.base, b3Transform_identity, hull, b3Transform_identity, cache );
}
#endif

bool b3ComputeMeshManifolds( b3World* world, int workerIndex, b3ContactSim* sim, const b3Shape* shapeA, b3Transform xfA,
							 const int* materialMap, const b3Shape* shapeB, b3Transform xfB )
{
	B3_ASSERT( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape );
	B3_UNUSED( workerIndex );

	b3Manifold oldManifolds[3];
	const int oldManifoldCount = sim->manifoldCount;
	if (oldManifoldCount > 0)
	{
		memcpy( oldManifolds, sim->manifolds, oldManifoldCount * sizeof( b3Manifold ) );
	}

	b3Manifold newManifolds[3];
	int newManifoldCount = 0;

	b3RefreshCache( sim, shapeA, xfA, shapeB->aabb );

	b3StackArray<b3TriangleManifold, 16> acceptedManifolds;
	b3StackArray<b3TriangleManifold, 16> tentativeManifolds;
	b3StackArray<b3TentativeTriangle, 16> tentativeTriangles;
	b3FoundEdges foundEdges;
	b3FoundVertices foundVertices;
	foundEdges.count = 0;
	foundVertices.count = 0;

	// Collide with triangles and build manifolds
	int triangleCount = sim->triangleCache.count;

	// This transform converts from mesh frame into the shapeB frame
	b3Transform xf = b3InvMulTransforms( xfB, xfA );
	b3Matrix3 relativeMatrix = b3MakeMatrixFromQuat( xf.q );
	b3Matrix3 matrixB = b3MakeMatrixFromQuat( xfB.q );

	for ( int index = 0; index < triangleCount; ++index )
	{
		int triangleIndex = sim->triangleCache[index].triangleIndex;

		b3Triangle triangle;
		if ( shapeA->type == b3_meshShape )
		{
			triangle = b3GetMeshTriangle( &shapeA->mesh, triangleIndex );
		}
		else
		{
			B3_ASSERT( shapeA->type == b3_heightShape );
			triangle = b3GetHeightFieldTriangle( shapeA->heightField, triangleIndex );
		}

		// Transform triangle into the shape frame
		triangle.vertices[0] = b3MulMV( relativeMatrix, triangle.vertices[0] ) + xf.p;
		triangle.vertices[1] = b3MulMV( relativeMatrix, triangle.vertices[1] ) + xf.p;
		triangle.vertices[2] = b3MulMV( relativeMatrix, triangle.vertices[2] ) + xf.p;

		// Skip triangle if the shape centroid is behind triangle plane (one-sided collision only)
		b3Vec3 centroid = b3GetShapeCentroid( shapeB );
		b3Plane trianglePlane = b3MakePlaneFromPoints( triangle.vertices[0], triangle.vertices[1], triangle.vertices[2] );
		float centroidSeparation = b3PlaneSeparation( trianglePlane, centroid );
		if ( centroidSeparation < 0.0f )
		{
			continue;
		}

		b3TaskContext* context = world->taskContexts.Get( workerIndex );
		b3ContactCache* cache = &sim->triangleCache[index].cache;
		b3TriangleManifold manifold = {};
		manifold.triangleFlags = triangle.flags;
		switch ( shapeB->type )
		{
			case b3_sphereShape:
				manifold = b3CollideTriangleAndSphere( triangle.vertices, triangle.flags, &shapeB->sphere );
				break;

			case b3_capsuleShape:
				manifold.base =
					b3CollideTriangleAndCapsule( triangle.vertices, triangle.flags, &shapeB->capsule, &cache->simplexCache );
				manifold.feature = b3_featureTriangleFace;
				break;

			case b3_hullShape:
#if 0
				manifold.base = b3CollideTriangleAndHullOld( triangle.vertices, triangle.flags, shapeB->hull, &cache->satCache,
														  &manifold.feature, triangleIndex );
#else
				manifold.base = b3CollideTriangleAndHull( triangle.vertices, triangle.flags, shapeB->hull, &cache->satCache,
														  &manifold.feature, triangleIndex );
#endif
				context->satCallCount += 1;
				context->satCacheHitCount += cache->satCache.hit;
				break;

			default:
				B3_ASSERT( false );
				return false;
		}

		if ( manifold.base.pointCount > 0 )
		{
			b3Vec3 normalB = manifold.base.normal;
			manifold.base.normal = b3MulMV( matrixB, normalB );

			for ( int pointIndex = 0; pointIndex < manifold.base.pointCount; ++pointIndex )
			{
				b3ManifoldPoint* mp = manifold.base.points + pointIndex;
				mp->triangleIndex = triangleIndex;

				// The manifolds are computed in local space B.
				// Anchor vectors are from the body origin to the contact point in world space.
				mp->anchorB = b3MulMV( matrixB, mp->point );
				mp->anchorA = mp->anchorB + ( xfB.p - xfA.p );
				mp->point = mp->anchorB + xfB.p;
			}

			manifold.i1 = triangle.i1;
			manifold.i2 = triangle.i2;
			manifold.i3 = triangle.i3;

			if ( manifold.feature == b3_featureTriangleFace || B3_FORCE_GHOST_COLLISIONS )
			{
				(void)b3AddEdge( &foundEdges, triangle.i1, triangle.i2 );
				(void)b3AddEdge( &foundEdges, triangle.i2, triangle.i3 );
				(void)b3AddEdge( &foundEdges, triangle.i3, triangle.i1 );
				(void)b3AddVertex( &foundVertices, triangle.i1 );
				(void)b3AddVertex( &foundVertices, triangle.i2 );
				(void)b3AddVertex( &foundVertices, triangle.i3 );

				acceptedManifolds.PushBack( manifold );
			}
			else if ( manifold.feature == b3_featureHullFace )
			{
				float cosNormalAngle = b3Dot( trianglePlane.normal, normalB );
				if ( cosNormalAngle > 0.8f )
				{
					(void)b3AddEdge( &foundEdges, triangle.i1, triangle.i2 );
					(void)b3AddEdge( &foundEdges, triangle.i2, triangle.i3 );
					(void)b3AddEdge( &foundEdges, triangle.i3, triangle.i1 );
					(void)b3AddVertex( &foundVertices, triangle.i1 );
					(void)b3AddVertex( &foundVertices, triangle.i2 );
					(void)b3AddVertex( &foundVertices, triangle.i3 );

					acceptedManifolds.PushBack( manifold );
				}
				else
				{
					b3TentativeTriangle tentativeTriangle = { .squaredDistance = manifold.squaredDistance,
															  .index = tentativeManifolds.m_count };
					tentativeTriangles.PushBack( tentativeTriangle );
					tentativeManifolds.PushBack( manifold );
				}
			}
			else
			{
				b3TentativeTriangle tentativeTriangle = { .squaredDistance = manifold.squaredDistance,
														  .index = tentativeManifolds.m_count };
				tentativeTriangles.PushBack( tentativeTriangle );
				tentativeManifolds.PushBack( manifold );
			}
		}
	}

	if ( shapeB->type == b3_sphereShape )
	{
		// Sort triangles so the closest triangles are processed first
		{
			b3TentativeTriangle tmp;
#define LESS( i, j ) tentativeTriangles[(int)i].squaredDistance < tentativeTriangles[(int)j].squaredDistance
#define SWAP( i, j )                                                                                                             \
	tmp = tentativeTriangles[(int)i], tentativeTriangles[(int)i] = tentativeTriangles[(int)j], tentativeTriangles[(int)j] = tmp
			QSORT( tentativeTriangles.m_count, LESS, SWAP );
#undef LESS
#undef SWAP
		}

		// Add tentative manifolds in sorted order. Avoid adding manifolds that generate ghost collisions.
		for ( int i = 0; i < tentativeTriangles.m_count; ++i )
		{
			b3TriangleManifold* m = tentativeManifolds.Get( tentativeTriangles[i].index );

			bool addedEdge1 = b3AddEdge( &foundEdges, m->i1, m->i2 );
			bool addedEdge2 = b3AddEdge( &foundEdges, m->i2, m->i3 );
			bool addedEdge3 = b3AddEdge( &foundEdges, m->i3, m->i1 );
			bool addedVertex1 = b3AddVertex( &foundVertices, m->i1 );
			bool addedVertex2 = b3AddVertex( &foundVertices, m->i2 );
			bool addedVertex3 = b3AddVertex( &foundVertices, m->i3 );

			b3TriangleFeature feature = m->feature;
			bool shouldCollide = false;
			switch ( feature )
			{
				case b3_featureNone:
				case b3_featureTriangleFace:
					B3_ASSERT( false );
					break;

				case b3_featureEdge1:
					shouldCollide = addedEdge1;
					break;

				case b3_featureEdge2:
					shouldCollide = addedEdge2;
					break;

				case b3_featureEdge3:
					shouldCollide = addedEdge3;
					break;

				case b3_featureVertex1:
					shouldCollide = addedVertex1;
					break;

				case b3_featureVertex2:
					shouldCollide = addedVertex2;
					break;

				case b3_featureVertex3:
					shouldCollide = addedVertex3;
					break;

				default:
					B3_ASSERT( false );
					break;
			}

			if ( shouldCollide == true )
			{
				acceptedManifolds.PushBack( tentativeManifolds[i] );
			}
		}
	}
	else if ( shapeB->type == b3_hullShape )
	{
		for ( int i = 0; i < tentativeManifolds.m_count; ++i )
		{
			b3TriangleManifold* m = tentativeManifolds.m_data + i;
			int triangleFlags = m->triangleFlags;

			if ( triangleFlags & b3_concaveEdge1 )
			{
				if ( b3FindEdge( &foundEdges, m->i1, m->i2 ) )
				{
					continue;
				}
			}

			if ( triangleFlags & b3_concaveEdge2 )
			{
				if ( b3FindEdge( &foundEdges, m->i2, m->i3 ) )
				{
					continue;
				}
			}

			if ( triangleFlags & b3_concaveEdge3 )
			{
				if ( b3FindEdge( &foundEdges, m->i3, m->i1 ) )
				{
					continue;
				}
			}

			acceptedManifolds.PushBack( tentativeManifolds[i] );
		}
	}

	if ( acceptedManifolds.m_count == 0 )
	{
		if (sim->manifoldCount > 0)
		{
			b3FreeManifolds( world, sim->manifolds, sim->manifoldCount );
			sim->manifolds = nullptr;
			sim->manifoldCount = 0;
		}

		return false;
	}

	// todo handle pre-solve

	// Cluster member ship for each triangle manifold. Will be 0, 1, or 2.
	b3StackArray<int, 16> clusterMembership;
	clusterMembership.Resize( acceptedManifolds.m_count );

#if B3_ENABLE_VALIDATION && 0
	constexpr int bufferSize = 256;
	char buffer[bufferSize] = {};
	int offset = 0;
	for ( int i = 0; i < acceptedManifolds.m_count && offset < bufferSize - 1; ++i )
	{
		offset += snprintf( buffer + offset, bufferSize - offset, "%d ", acceptedManifolds.m_data[i].base.pointCount );
	}
	b3Log( "%s", buffer );

	b3HexColor colors[16] = {
		b3_colorRed,	b3_colorOrange, b3_colorYellow,	   b3_colorGreen,	  b3_colorCyan,	 b3_colorBlue,
		b3_colorViolet, b3_colorPink,	b3_colorChocolate, b3_colorGoldenRod, b3_colorCoral, b3_colorRosyBrown,
		b3_colorAqua,	b3_colorPeru,	b3_colorLime,	   b3_colorGold,
	};

	b3TaskContext* context = world->taskContexts.Get( workerIndex );
	for ( int i = 0; i < acceptedManifolds.m_count && context->pointCount < 64; ++i )
	{
		b3Manifold* m = &acceptedManifolds.m_data[i].base;
		int np = m->pointCount;
		for ( int j = 0; j < np && context->pointCount < 64; ++j )
		{
			b3Vec3 p = m->points[j].point;
			p.y += 0.005f * i + 0.005f;
			b3HexColor color = colors[i & 0xF];
			context->points[context->pointCount] = { .p = p, .label = j, .color = color };
			context->pointCount += 1;
		}
	}
#endif

	b3Vec3 clusterNormals[B3_MAX_MANIFOLDS];
	int clusterCount =
		b3BuildTriangleClusters( clusterNormals, clusterMembership.m_data, acceptedManifolds.m_data, acceptedManifolds.m_count );

	// b3Log( "clusterCount = %d", clusterCount );

	b3StackArray<b3ManifoldPoint, 32> manifoldPoints[B3_MAX_MANIFOLDS];

	// Distribute all the points to the clusters (up to 3 clusters)
	for ( int i = 0; i < acceptedManifolds.m_count; ++i )
	{
		int clusterIndex = clusterMembership[i];
		B3_ASSERT( 0 <= clusterIndex && clusterIndex < clusterCount );

		b3ManifoldPoint* mps = acceptedManifolds[i].base.points;
		int pointCount = acceptedManifolds[i].base.pointCount;
		manifoldPoints[clusterIndex].Append( mps, pointCount );
	}

	// Reduce the points and build the manifolds
	for ( int clusterIndex = 0; clusterIndex < clusterCount; ++clusterIndex )
	{
		if ( manifoldPoints[clusterIndex].m_count == 0 )
		{
			continue;
		}

		B3_ASSERT( newManifoldCount < 3 );
		b3Manifold& manifold = newManifolds[newManifoldCount];
		manifold.normal = clusterNormals[clusterIndex];

		// Reduce the points into the final manifolds.
		// Warning: this modifies the manifoldPoints array.
		b3ReduceManifoldPoints( &manifold, manifoldPoints[clusterIndex].m_data, manifoldPoints[clusterIndex].m_count );

		int pointCount = manifold.pointCount;
		B3_ASSERT( 0 < pointCount && pointCount <= B3_MAX_MANIFOLD_POINTS );

		manifold.twistImpulse = 0.0f;
		manifold.frictionImpulse = b3Vec3_zero;
		manifold.rollingImpulse = b3Vec3_zero;
		manifold.pointCount = pointCount;

		newManifoldCount += 1;
	}

	// Allocate manifolds if the number changed
	if (newManifoldCount != oldManifoldCount)
	{
		if (oldManifoldCount > 0)
		{
			b3FreeManifolds( world, sim->manifolds, oldManifoldCount );
		}

		if (newManifoldCount > 0)
		{
			sim->manifolds = b3AllocateManifolds( world, newManifoldCount );
		}
		else
		{
			sim->manifolds = nullptr;
		}
	}

	if (newManifoldCount > 0)
	{
		B3_ASSERT( sim->manifolds != nullptr );
		memcpy( sim->manifolds, newManifolds, newManifoldCount * sizeof( b3Manifold ) );
	}

	sim->manifoldCount = newManifoldCount;

	// Match old manifolds with new one. This requires the normal vectors to be in world space.
	b3MatchManifolds( sim->manifolds, sim->manifoldCount, oldManifolds, oldManifoldCount );

	const b3SurfaceMaterial* materialB = shapeB->materials + 0;
	b3Vec3 tangentVelocityA = b3Vec3_zero;

	// Update friction and restitution if the mesh has per triangle material
	if ( shapeA->materialCount > 0 && sim->manifoldCount > 0 )
	{
		float friction = 0.0f;
		float restitution = 0.0f;
		float sampleCount = 0.0f;

		const uint8_t* materialIndices;
		if ( shapeA->type == b3_meshShape )
		{
			materialIndices = b3GetMeshMaterialIndices( shapeA->mesh.data );
		}
		else
		{
			materialIndices = shapeA->heightField->materialIndices;
		}

		float speculativeDistance = B3_SPECULATIVE_DISTANCE;

		for ( int i = 0; i < sim->manifoldCount; ++i )
		{
			b3Manifold& manifold = sim->manifolds[i];
			int pointCount = manifold.pointCount;
			for ( int j = 0; j < pointCount; ++j )
			{
				if ( manifold.points[j].separation > speculativeDistance )
				{
					continue;
				}

				int triangleIndex = manifold.points[j].triangleIndex;
				int materialIndex;
				if ( shapeA->type == b3_meshShape )
				{
					materialIndex = materialIndices[triangleIndex];

					if (materialMap != nullptr)
					{
						materialIndex = materialMap[materialIndex];
					}
				}
				else
				{
					materialIndex = materialIndices[triangleIndex >> 1];
				}

				materialIndex = b3ClampInt( materialIndex, 0, shapeA->materialCount - 1 );
				b3SurfaceMaterial material = shapeA->materials[materialIndex];
				friction += world->frictionCallback( material.friction, material.userMaterialId, materialB->friction,
													 materialB->userMaterialId );
				restitution += world->restitutionCallback( material.restitution, material.userMaterialId, materialB->restitution,
														   materialB->userMaterialId );

				tangentVelocityA = b3Add( tangentVelocityA, material.tangentVelocity );

				sampleCount += 1.0f;
			}
		}

		if ( sampleCount > 0.0f )
		{
			float invCount = 1.0f / sampleCount;
			sim->friction = invCount * friction;
			sim->restitution = invCount * restitution;
			tangentVelocityA = b3MulSV( invCount, tangentVelocityA );
		}

		B3_ASSERT( b3IsValidFloat( sim->friction ) && sim->friction >= 0.0f );
		B3_ASSERT( b3IsValidFloat( sim->restitution ) && sim->restitution >= 0.0f );
	}
	else
	{
		// Keep these updated in case the values on the shapes are modified
		sim->friction = world->frictionCallback( shapeA->materials[0].friction, shapeA->materials[0].userMaterialId,
												 materialB->friction, materialB->userMaterialId );
		sim->restitution = world->restitutionCallback( shapeA->materials[0].restitution, shapeA->materials[0].userMaterialId,
													   materialB->restitution, materialB->userMaterialId );

		tangentVelocityA = shapeA->materials[0].tangentVelocity;
	}

	tangentVelocityA = b3RotateVector( xfA.q, tangentVelocityA );

	float radiusB = 0.0f;
	if ( shapeB->type == b3_sphereShape )
	{
		radiusB = shapeB->sphere.radius;
	}
	else if ( shapeB->type == b3_capsuleShape )
	{
		radiusB = shapeB->capsule.radius;
	}
	sim->rollingResistance = shapeB->materials[0].rollingResistance * radiusB;

	b3Vec3 tangentVelocityB = b3RotateVector( xfB.q, materialB->tangentVelocity );
	sim->tangentVelocity = b3Sub( tangentVelocityA, tangentVelocityB );

	return true;
}
