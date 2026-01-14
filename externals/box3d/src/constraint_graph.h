// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "bitset.h"
#include "constants.h"

struct b3Body;
struct b3ContactSim;
struct b3Contact;
struct b3ContactConstraint;
struct b3ContactConstraintSIMD;
struct b3JointSim;
struct b3Joint;
struct b3StepContext;
struct b3World;

// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other bodies.
#define B3_OVERFLOW_INDEX (B3_GRAPH_COLOR_COUNT - 1)

// This keeps constraints involving two dynamic bodies at a lower solver priority than constraints
// involving a dynamic and static bodies. This reduces tunneling due to push through.
#define B3_DYNAMIC_COLOR_COUNT ( B3_GRAPH_COLOR_COUNT - 4 )

struct b3GraphColor
{
	// This bitset is indexed by bodyId so this is over-sized to encompass static bodies
	// however I never traverse these bits or use the bit count for anything
	// This bitset is unused on the overflow color.
	//
	// Dirk suggested having a uint64_t per body that tracks the graph color membership
	// but I think this would make debugging harder and be less flexible. With the bitset
	// I can trivially increase the number of graph colors beyond 64. See usage of b2CountSetBits
	// for validation.
	b3BitSet bodySet;

	// cache friendly arrays
	b3Array<b3ContactSim> contactSims;
	b3Array<b3JointSim> jointSims;

	// transient data
	b3ContactConstraint* overflowConstraints;

	// Contact constraints with strides 1, 2, and 3 to support contact sims with that number of manifolds.
	b3ContactConstraintSIMD* simdConstraints[3];

	// This is the total number of active manifolds that generate contact constraints for this graph.
	int overflowManifoldCount;
};

struct b3ConstraintGraph
{
	// including overflow at the end
	b3GraphColor colors[B3_GRAPH_COLOR_COUNT];
};

void b3CreateGraph( b3ConstraintGraph* graph, int bodyCapacity );
void b3DestroyGraph( b3ConstraintGraph* graph );

void b3AddContactToGraph( b3World* world, b3ContactSim* contactSim, b3Contact* contact );
void b3RemoveContactFromGraph( b3World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex, bool destroy );

b3JointSim* b3CreateJointInGraph( b3World* world, b3Joint* joint );
void b3AddJointToGraph( b3World* world, b3JointSim* jointSim, b3Joint* joint );
void b3RemoveJointFromGraph( b3World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex );
