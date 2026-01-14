// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

struct b3Contact;
struct b3Joint;
struct b3World;

// Deterministic solver
//
// Collide all awake contacts
// Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are
// created in a deterministic order. bit-wise OR together bit arrays and issue changes:
// - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root
// - stop touching: increment constraintRemoveCount

// Persistent island for awake bodies, joints, and contacts
// https://en.wikipedia.org/wiki/Component_(graph_theory)
// https://en.wikipedia.org/wiki/Dynamic_connectivity
// map from int to solver set and index

struct b3Island
{
	// index of solver set stored in b3World
	// may be B3_NULL_INDEX
	int setIndex;

	// island index within set
	// may be B3_NULL_INDEX
	int localIndex;

	int islandId;

	int headBody;
	int tailBody;
	int bodyCount;

	int headContact;
	int tailContact;
	int contactCount;

	int headJoint;
	int tailJoint;
	int jointCount;

	// Keeps track of how many contacts have been removed from this island.
	// This is used to determine if an island is a candidate for splitting.
	int constraintRemoveCount;
};

// This is used to move islands across solver sets
struct b3IslandSim
{
	int islandId;
};

b3Island* b3CreateIsland( b3World* world, int setIndex );
void b3DestroyIsland( b3World* world, int islandId );

// Link contacts into the island graph when it starts having contact points
void b3LinkContact( b3World* world, b3Contact* contact );

// Unlink contact from the island graph when it stops having contact points
void b3UnlinkContact( b3World* world, b3Contact* contact );

// Link a joint into the island graph when it is created
void b3LinkJoint( b3World* world, b3Joint* joint );

// Unlink a joint from the island graph when it is destroyed
void b3UnlinkJoint( b3World* world, b3Joint* joint );

void b3SplitIsland( b3World* world, int baseId );
void b3SplitIslandTask( int startIndex, int endIndex, uint32_t threadIndex, void* context );

void b3ValidateIsland( b3World* world, int islandId );
