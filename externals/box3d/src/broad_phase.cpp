// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "broad_phase.h"

#include "aabb.h"
#include "arena_allocator.h"
#include "array.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "physics_world.h"
#include "platform.h"
#include "shape.h"

#include <string.h>

// #include <stdio.h>

// static FILE* s_file = nullptr;

void b3CreateBroadPhase( b3BroadPhase* bp, const b3Capacity* capacity )
{
	static_assert( b3_bodyTypeCount == 3, "must be three body types" );

	// if (s_file == nullptr)
	//{
	//	s_file = fopen("pairs01.txt", "a");
	//	fprintf(s_file, "============\n\n");
	// }

	bp->moveSet = b3CreateSet( 2 * capacity->dynamicShapeCount );
	bp->moveArray.Create( capacity->dynamicShapeCount );
	bp->moveResults = nullptr;
	bp->movePairs = nullptr;
	bp->movePairCapacity = 0;
	b3AtomicStoreInt( &bp->movePairIndex, 0 );
	bp->pairSet = b3CreateSet( 2 * capacity->contactCount );

	int staticCapacity = b3MaxInt( 16, capacity->staticShapeCount );
	bp->trees[b3_staticBody] = b3DynamicTree_Create( staticCapacity );

	int kinematicCapacity = 16;
	bp->trees[b3_kinematicBody] = b3DynamicTree_Create( kinematicCapacity );

	int dynamicCapacity = b3MaxInt( 16, capacity->dynamicShapeCount );
	bp->trees[b3_dynamicBody] = b3DynamicTree_Create( dynamicCapacity );
}

void b3DestroyBroadPhase( b3BroadPhase* bp )
{
	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3DynamicTree_Destroy( bp->trees + i );
	}

	b3DestroySet( &bp->moveSet );
	bp->moveArray.Destroy();
	b3DestroySet( &bp->pairSet );

	*bp = {};

	memset( bp, 0, sizeof( b3BroadPhase ) );

	// if (s_file != nullptr)
	//{
	//	fclose(s_file);
	//	s_file = nullptr;
	// }
}

static void b3UnBufferMove( b3BroadPhase* bp, int proxyKey )
{
	bool found = b3RemoveKey( &bp->moveSet, proxyKey + 1 );

	if ( found )
	{
		// Purge from move buffer. Linear search.
		// todo if I can iterate the move set then I don't need the moveArray
		int count = bp->moveArray.count;
		for ( int i = 0; i < count; ++i )
		{
			if ( bp->moveArray.data[i] == proxyKey )
			{
				bp->moveArray.RemoveSwap( i );
				break;
			}
		}
	}
}

int b3BroadPhase_CreateProxy( b3BroadPhase* bp, b3BodyType proxyType, b3AABB aabb, uint64_t categoryBits, int shapeIndex,
							  bool forcePairCreation )
{
	B3_ASSERT( 0 <= proxyType && proxyType < b3_bodyTypeCount );
	int proxyId = b3DynamicTree_CreateProxy( bp->trees + proxyType, aabb, categoryBits, shapeIndex );
	int proxyKey = B3_PROXY_KEY( proxyId, proxyType );
	if ( proxyType != b3_staticBody || forcePairCreation )
	{
		b3BufferMove( bp, proxyKey );
	}
	return proxyKey;
}

void b3BroadPhase_DestroyProxy( b3BroadPhase* bp, int proxyKey )
{
	B3_ASSERT( bp->moveArray.count == (int)bp->moveSet.count );
	b3UnBufferMove( bp, proxyKey );

	b3BodyType proxyType = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );

	B3_ASSERT( 0 <= proxyType && proxyType <= b3_bodyTypeCount );
	b3DynamicTree_DestroyProxy( bp->trees + proxyType, proxyId );
}

void b3BroadPhase_MoveProxy( b3BroadPhase* bp, int proxyKey, b3AABB aabb )
{
	b3BodyType proxyType = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );

	b3DynamicTree_MoveProxy( bp->trees + proxyType, proxyId, aabb );
	b3BufferMove( bp, proxyKey );
}

void b3BroadPhase_EnlargeProxy( b3BroadPhase* bp, int proxyKey, b3AABB aabb )
{
	B3_ASSERT( proxyKey != B3_NULL_INDEX );
	int typeIndex = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );

	B3_ASSERT( typeIndex != b3_staticBody );

	b3DynamicTree_EnlargeProxy( bp->trees + typeIndex, proxyId, aabb );
	b3BufferMove( bp, proxyKey );
}

struct b3MovePair
{
	int shapeIndexA;
	int shapeIndexB;
	int childIndex;
	b3MovePair* next;
	bool heap;
};

struct b3MoveResult
{
	b3MovePair* pairList;
};

struct b3QueryPairContext
{
	b3World* world;
	b3MoveResult* moveResult;
	b3AABB aabb;
	b3BodyType queryTreeType;
	int queryProxyKey;
	int queryShapeIndex;

	int compoundProxyId;
	int compoundShapeIndex;
};

// This is called from b3DynamicTree::Query when we are gathering pairs.
static bool b3PairQueryCallback( int proxyId, uint64_t userData, void* context )
{
	b3QueryPairContext* queryContext = (b3QueryPairContext*)context;

	int shapeIndex = int( userData );

	if (shapeIndex == queryContext->queryShapeIndex)
	{
		return true;
	}

	b3World* world = queryContext->world;
	int childIndex = 0;

	if (queryContext->compoundShapeIndex == B3_NULL_INDEX)
	{
		b3Shape* shape = world->shapes.Get( shapeIndex );
		if (shape->type == b3_compoundShape)
		{
			// recurse
			queryContext->compoundShapeIndex = shapeIndex;
			queryContext->compoundProxyId = proxyId;
			b3DynamicTree_Query( &shape->compound->tree, queryContext->aabb, B3_DEFAULT_MASK_BITS, false, b3PairQueryCallback,
								 context );
			queryContext->compoundShapeIndex = B3_NULL_INDEX;
			queryContext->compoundProxyId = B3_NULL_INDEX;
			return true;
		}
	}
	else
	{
		// The query is going into a compound shape
		shapeIndex = queryContext->compoundShapeIndex;
		proxyId = queryContext->compoundProxyId;
		childIndex = int( userData );
	}

	b3BroadPhase* broadPhase = &queryContext->world->broadPhase;

	int proxyKey = B3_PROXY_KEY( proxyId, queryContext->queryTreeType );
	int queryProxyKey = queryContext->queryProxyKey;

	// A proxy cannot form a pair with itself.
	B3_ASSERT( proxyKey != queryContext->queryProxyKey );

	b3BodyType treeType = queryContext->queryTreeType;
	b3BodyType queryProxyType = B3_PROXY_TYPE( queryProxyKey );

	// De-duplication
	// It is important to prevent duplicate contacts from being created. Ideally I can prevent duplicates
	// early and in the worker. Most of the time the moveSet contains dynamic and kinematic proxies, but
	// sometimes it has static proxies.

	// I had an optimization here to skip checking the move set if this is a query into
	// the static tree. The assumption is that the static proxies are never in the move set
	// so there is no risk of duplication. However, this is not true with
	// b3ShapeDef::invokeContactCreation or when a static shape is modified.
	// There can easily be scenarios where the static proxy is in the moveSet but the dynamic proxy is not.
	// I could have some flag to indicate that there are any static bodies in the moveSet.

	// Is this proxy also moving?
	if ( queryProxyType == b3_dynamicBody )
	{
		if ( treeType == b3_dynamicBody && proxyKey < queryProxyKey )
		{
			bool moved = b3ContainsKey( &broadPhase->moveSet, proxyKey + 1 );
			if ( moved )
			{
				// Both proxies are moving. Avoid duplicate pairs.
				return true;
			}
		}
	}
	else
	{
		B3_ASSERT( treeType == b3_dynamicBody );
		bool moved = b3ContainsKey( &broadPhase->moveSet, proxyKey + 1 );
		if ( moved )
		{
			// Both proxies are moving. Avoid duplicate pairs.
			return true;
		}
	}

	uint64_t pairKey = b3ShapePairKey( shapeIndex, queryContext->queryShapeIndex, childIndex );
	if ( b3ContainsKey( &broadPhase->pairSet, pairKey ) )
	{
		// contact exists
		return true;
	}

	// Order shapes so that B3_SHAPE_PAIR_KEY works correctly
	int shapeIdA = shapeIndex;
	int shapeIdB = queryContext->queryShapeIndex;
	b3Shape* shapeA = world->shapes.Get( shapeIdA );
	b3Shape* shapeB = world->shapes.Get( shapeIdB );
	int bodyIdA = shapeA->bodyId;
	int bodyIdB = shapeB->bodyId;

	// Are the shapes on the same body?
	if ( bodyIdA == bodyIdB )
	{
		return true;
	}

	// Sensors are handled elsewhere
	if ( shapeA->sensorIndex != B3_NULL_INDEX || shapeB->sensorIndex != B3_NULL_INDEX )
	{
		return true;
	}

	if ( b3ShouldShapesCollide( shapeA->filter, shapeB->filter ) == false )
	{
		return true;
	}

	// Does a joint override collision?
	b3Body* bodyA = world->bodies.Get( bodyIdA );
	b3Body* bodyB = world->bodies.Get( bodyIdB );
	if ( b3ShouldBodiesCollide( world, bodyA, bodyB ) == false )
	{
		return true;
	}

	// Custom user filter
	if ( shapeA->enableCustomFiltering || shapeB->enableCustomFiltering )
	{
		b3CustomFilterFcn* customFilterFcn = queryContext->world->customFilterFcn;
		if ( customFilterFcn != nullptr )
		{
			b3ShapeId idA = { shapeIdA + 1, world->worldId, shapeA->generation };
			b3ShapeId idB = { shapeIdB + 1, world->worldId, shapeB->generation };
			bool shouldCollide = customFilterFcn( idA, idB, queryContext->world->customFilterContext );
			if ( shouldCollide == false )
			{
				return true;
			}
		}
	}

	// todo per thread to eliminate atomic?
	int pairIndex = b3AtomicFetchAddInt( &broadPhase->movePairIndex, 1 );

	b3MovePair* pair;
	if ( pairIndex < broadPhase->movePairCapacity )
	{
		pair = broadPhase->movePairs + pairIndex;
		pair->heap = false;
	}
	else
	{
		// todo experimenting with ignoring this pair if we ran out of space
		return true;
		//pair = (b3MovePair*)b3Alloc( sizeof( b3MovePair ) );
		//pair->heap = true;
	}

	pair->shapeIndexA = shapeIdA;
	pair->shapeIndexB = shapeIdB;
	pair->childIndex = childIndex;
	pair->next = queryContext->moveResult->pairList;
	queryContext->moveResult->pairList = pair;

	// continue the query
	return true;
}

static void b3FindPairsTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b3TracyCZoneNC( pair_task, "Pair Task", b3_colorAquamarine, true );

	B3_UNUSED( threadIndex );

	b3World* world = (b3World*)context;
	b3BroadPhase* bp = &world->broadPhase;

	b3QueryPairContext queryContext = {};
	queryContext.world = world;
	queryContext.compoundShapeIndex = B3_NULL_INDEX;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		// Initialize move result for this moved proxy
		queryContext.moveResult = bp->moveResults + i;
		queryContext.moveResult->pairList = nullptr;

		int proxyKey = bp->moveArray.data[i];
		if ( proxyKey == B3_NULL_INDEX )
		{
			// proxy was destroyed after it moved
			continue;
		}

		b3BodyType proxyType = B3_PROXY_TYPE( proxyKey );

		int proxyId = B3_PROXY_ID( proxyKey );
		queryContext.queryProxyKey = proxyKey;

		const b3DynamicTree* baseTree = bp->trees + proxyType;

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a contact that may touch later.
		b3AABB fatAABB = b3DynamicTree_GetAABB( baseTree, proxyId );
		queryContext.queryShapeIndex = (int)b3DynamicTree_GetUserData( baseTree, proxyId );
		queryContext.aabb = fatAABB;

		// Compound shape collision invocation is not supported
		B3_VALIDATE( world->shapes.data[queryContext.queryShapeIndex].type != b3_compoundShape );

		// Query trees. Only dynamic proxies collide with kinematic and static proxies.
		// Using B3_DEFAULT_MASK_BITS so that b3Filter::groupIndex works.
		// consider using bits = groupIndex > 0 ? B3_DEFAULT_MASK_BITS : maskBits
		bool requireAllBits = false;
		if ( proxyType == b3_dynamicBody )
		{
			queryContext.queryTreeType = b3_kinematicBody;
			b3DynamicTree_Query( bp->trees + b3_kinematicBody, fatAABB, B3_DEFAULT_MASK_BITS, requireAllBits, b3PairQueryCallback,
								 &queryContext );

			queryContext.queryTreeType = b3_staticBody;
			b3DynamicTree_Query( bp->trees + b3_staticBody, fatAABB, B3_DEFAULT_MASK_BITS, requireAllBits, b3PairQueryCallback,
								 &queryContext );
		}

		// All proxies collide with dynamic proxies
		// Using B3_DEFAULT_MASK_BITS so that b3Filter::groupIndex works.
		queryContext.queryTreeType = b3_dynamicBody;
		b3DynamicTree_Query( bp->trees + b3_dynamicBody, fatAABB, B3_DEFAULT_MASK_BITS, requireAllBits, b3PairQueryCallback,
							 &queryContext );
	}

	b3TracyCZoneEnd( pair_task );
}

void b3UpdateBroadPhasePairs( b3World* world )
{
	b3BroadPhase* bp = &world->broadPhase;

	int moveCount = bp->moveArray.count;
	B3_ASSERT( moveCount == (int)bp->moveSet.count );

	if ( moveCount == 0 )
	{
		return;
	}

	b3TracyCZoneNC( update_pairs, "Pairs", b3_colorMediumSlateBlue, true );

	b3ArenaAllocator* alloc = &world->arena;

	// todo these could be in the step context
	bp->moveResults = (b3MoveResult*)b3AllocateArenaItem( alloc, moveCount * sizeof( b3MoveResult ), "move results" );
	bp->movePairCapacity = 16 * moveCount;
	bp->movePairs = (b3MovePair*)b3AllocateArenaItem( alloc, bp->movePairCapacity * sizeof( b3MovePair ), "move pairs" );

	b3AtomicStoreInt( &bp->movePairIndex, 0 );

#ifndef NDEBUG
	extern b3AtomicInt b3_probeCount;
	b3AtomicStoreInt( &b3_probeCount, 0 );
#endif

	int minRange = 64;
	void* userPairTask = world->enqueueTaskFcn( &b3FindPairsTask, moveCount, minRange, world, world->userTaskContext );
	if ( userPairTask != nullptr )
	{
		world->finishTaskFcn( userPairTask, world->userTaskContext );
		world->taskCount += 1;
	}

	// todo_erin could start tree rebuild here

	b3TracyCZoneNC( create_contacts, "Create Contacts", b3_colorCoral, true );

	// Single-threaded work
	// - Clear move flags
	// - Create contacts in deterministic order
	for ( int i = 0; i < moveCount; ++i )
	{
		b3MoveResult* result = bp->moveResults + i;
		b3MovePair* pair = result->pairList;
		while ( pair != nullptr )
		{
			int shapeIdA = pair->shapeIndexA;
			int shapeIdB = pair->shapeIndexB;
			int childIndex = pair->childIndex;

			// if (s_file != nullptr)
			//{
			//	fprintf(s_file, "%d %d\n", shapeIdA, shapeIdB);
			// }

			b3Shape* shapeA = world->shapes.Get( shapeIdA );
			b3Shape* shapeB = world->shapes.Get( shapeIdB );

			b3CreateContact( world, shapeA, shapeB, childIndex );

			if ( pair->heap )
			{
				b3MovePair* temp = pair;
				pair = pair->next;
				b3Free( temp, sizeof( b3MovePair ) );
			}
			else
			{
				pair = pair->next;
			}
		}

		// if (s_file != nullptr)
		//{
		//	fprintf(s_file, "\n");
		// }
	}

	// if (s_file != nullptr)
	//{
	//	fprintf(s_file, "count = %d\n\n", pairCount);
	// }

	// Reset move buffer
	bp->moveArray.Clear();
	b3ClearSet( &bp->moveSet );

	b3FreeArenaItem( alloc, bp->movePairs );
	bp->movePairs = nullptr;
	b3FreeArenaItem( alloc, bp->moveResults );
	bp->moveResults = nullptr;

	b3ValidateSolverSets( world );

	b3TracyCZoneEnd( create_contacts );

	b3TracyCZoneEnd( update_pairs );
}

bool b3BroadPhase_TestOverlap( const b3BroadPhase* bp, int proxyKeyA, int proxyKeyB )
{
	int typeIndexA = B3_PROXY_TYPE( proxyKeyA );
	int proxyIdA = B3_PROXY_ID( proxyKeyA );
	int typeIndexB = B3_PROXY_TYPE( proxyKeyB );
	int proxyIdB = B3_PROXY_ID( proxyKeyB );

	b3AABB aabbA = b3DynamicTree_GetAABB( bp->trees + typeIndexA, proxyIdA );
	b3AABB aabbB = b3DynamicTree_GetAABB( bp->trees + typeIndexB, proxyIdB );
	return b3AABB_Overlaps( aabbA, aabbB );
}

void b3BroadPhase_RebuildTrees( b3BroadPhase* bp )
{
	b3DynamicTree_Rebuild( bp->trees + b3_dynamicBody, false );
	b3DynamicTree_Rebuild( bp->trees + b3_kinematicBody, false );
}

int b3BroadPhase_GetShapeIndex( b3BroadPhase* bp, int proxyKey )
{
	int typeIndex = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );

	return (int)b3DynamicTree_GetUserData( bp->trees + typeIndex, proxyId );
}

void b3ValidateBroadphase( const b3BroadPhase* bp )
{
	b3DynamicTree_Validate( bp->trees + b3_dynamicBody );
	b3DynamicTree_Validate( bp->trees + b3_kinematicBody );

	// TODO_ERIN validate every shape AABB is contained in tree AABB
}

void b3ValidateNoEnlarged( const b3BroadPhase* bp )
{
#if B3_ENABLE_VALIDATION == 1
	for ( int j = 0; j < b3_bodyTypeCount; ++j )
	{
		const b3DynamicTree* tree = bp->trees + j;
		b3DynamicTree_ValidateNoEnlarged( tree );
	}
#else
	B3_UNUSED( bp );
#endif
}
