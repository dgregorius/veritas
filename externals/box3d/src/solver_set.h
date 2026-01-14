// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

struct b3Body;
struct b3BodySim;
struct b3BodyState;
struct b3ContactSim;
struct b3IslandSim;
struct b3Joint;
struct b3JointSim;
struct b3World;

// This holds solver set data. The following sets are used:
// - static set for all static bodies and joints between static bodies
// - active set for all active bodies with body states (no
// contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
// The purpose of solver sets is to achieve high memory locality.
// https://www.youtube.com/watch?v=nZNd5FjSquk
struct b3SolverSet
{
	// Body array. Empty for unused set.
	b3Array<b3BodySim> bodySims;

	// Body state only exists for active set
	b3Array<b3BodyState> bodyStates;

	// This holds sleeping/disabled joints. Empty for static/active set.
	b3Array<b3JointSim> jointSims;

	// This holds all contacts for sleeping sets.
	// This holds non-touching contacts for the awake set.
	b3Array<b3ContactSim> contactSims;

	// The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
	// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
	// islands will be naturally merged with the set is woken.
	// The static and disabled sets have no islands.
	// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
	b3Array<b3IslandSim> islandSims;

	// Aligns with b3World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
	int setIndex;
};

void b3DestroySolverSet( b3World* world, int setIndex );

void b3WakeSolverSet( b3World* world, int setIndex );
void b3TrySleepIsland( b3World* world, int islandId );

// Merge set 2 into set 1 then destroy set 2.
// Warning: any pointers into these sets will be orphaned.
void b3MergeSolverSets( b3World* world, int setIndex1, int setIndex2 );

void b3TransferBody( b3World* world, b3SolverSet* targetSet, b3SolverSet* sourceSet, b3Body* body );
void b3TransferJoint( b3World* world, b3SolverSet* targetSet, b3SolverSet* sourceSet, b3Joint* joint );
