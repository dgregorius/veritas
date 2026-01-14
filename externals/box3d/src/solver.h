// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box3d/math_functions.h"

#include <stdint.h>

struct b3BodySim;
struct b3BodyState;
struct b3ContactSim;
struct b3JointSim;
struct b3Manifold;
struct b3World;

enum b3SolverStageType
{
	b3_stagePrepareJoints,
	b3_stagePrepareContacts1,
	b3_stagePrepareContacts2,
	b3_stagePrepareContacts3,
	b3_stageIntegrateVelocities,
	b3_stageWarmStart,
	b3_stageSolve,
	b3_stageIntegratePositions,
	b3_stageRelax,
	b3_stageRestitution,
	b3_stageStoreImpulses1,
	b3_stageStoreImpulses2,
	b3_stageStoreImpulses3,
};

enum b3SolverBlockType : int16_t
{
	// Block for iterating across awake bodies
	b3_bodyBlock,

	// Block for iterating across awake joints. For prepare.
	b3_jointBlock,

	// Block for iterating across awake contacts. For prepare and store of contacts with 1 manifold.
	b3_contact1Block,

	// Block for iterating across awake contacts. For prepare and store of contacts with 2 manifolds.
	b3_contact2Block,

	// Block for iterating across awake contacts. For prepare and store of contacts with 3 manifolds.
	b3_contact3Block,

	// Block for iterating across joints of a single graph color.
	b3_graphJointBlock,

	// Block for iterating across contacts with 1 manifold of a single graph color
	b3_graphContact1Block,

	// Block for iterating across contacts with 2 manifold2 of a single graph color
	b3_graphContact2Block,

	// Block for iterating across contacts with 3 manifolds of a single graph color
	b3_graphContact3Block,
};

struct b3Softness
{
	float biasRate;
	float massScale;
	float impulseScale;
};

// Look up manifold for each constraint index.
// Could be color, contact index, and manifold index
struct b3ManifoldLookup
{
	b3ContactSim* contactSim;
};

// Each block of work has a sync index that gets incremented when a worker claims the block. This ensures only a single worker
// claims a block, yet this lets work be distributed dynamically across multiple workers (work stealing). This also reduces
// contention on a single block index atomic. For non-iterative stages the sync index is simply set to one. For iterative stages
// (solver iteration) the same block of work is executed once per iteration and the atomic sync index is shared across iterations,
// so it increases monotonically.
struct b3SolverBlock
{
	int startIndex;
	int16_t count;
	b3SolverBlockType blockType;
	b3AtomicInt syncIndex;
};

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
struct b3SolverStage
{
	b3SolverStageType type;
	b3SolverBlock* blocks;
	int blockCount;
	int colorIndex;
	// todo consider false sharing of this atomic
	b3AtomicInt completionCount;
};

// Context for a time step. Recreated each time step.
struct b3StepContext
{
	// time step
	float dt;

	// inverse time step (0 if dt == 0).
	float inv_dt;

	// sub-step
	float h;
	float inv_h;

	int subStepCount;

	b3Softness contactSoftness;
	b3Softness staticSoftness;

	float restitutionThreshold;
	float maxLinearVelocity;

	struct b3World* world;
	struct b3ConstraintGraph* graph;

	// shortcut to body states from awake set
	b3BodyState* states;

	// shortcut to body sims from awake set
	b3BodySim* sims;

	// array of all shape ids for shapes that have enlarged AABBs
	int* enlargedShapes;
	int enlargedShapeCount;

	// Array of bullet bodies that need continuous collision handling
	int* bulletBodies;
	b3AtomicInt bulletBodyCount;

	// joint pointers for simplified parallel-for access.
	b3JointSim** joints;

	// Contact pointers for simplified parallel-for access. Used in narrow-phase.
	b3ContactSim** contactSims;

	// Contact manifolds for SIMD contact constraint preparation
	// Parallel-for prepare contacts with NULL gaps for SIMD remainders
	// There is one entry for each SIMD lane.
	// There are separate arrays for contacts with 1/2/3 manifolds
	b3ManifoldLookup* manifoldLookups[3];

	// All contact manifold constraints across all colors. These are distributed to each color
	// in contiguous chunks according to manifold count.
	// There are separate arrays for contacts with 1/2/3 manifolds
	struct b3ContactConstraintSIMD* contactConstraints[3];

	int activeColorCount;
	int workerCount;

	b3SolverStage* stages;
	int stageCount;
	bool enableWarmStarting;

	// todo padding to prevent false sharing
	char dummy1[64];

	// sync index (16-bits) | stage type (16-bits)
	b3AtomicU32 atomicSyncBits;

	char dummy2[64];
};

inline b3Softness b3MakeSoft( float hertz, float zeta, float h )
{
	if ( hertz == 0.0f )
	{
		return b3Softness{
			.biasRate = 0.0f,
			.massScale = 0.0f,
			.impulseScale = 0.0f,
		};
	}

	float omega = 2.0f * B3_PI * hertz;
	float a1 = 2.0f * zeta + h * omega;
	float a2 = h * omega * a1;
	float a3 = 1.0f / ( 1.0f + a2 );

	return b3Softness{
		.biasRate = omega / a1,
		.massScale = a2 * a3,
		.impulseScale = a3,
	};
}

void b3Solve( b3World* world, b3StepContext* stepContext );
