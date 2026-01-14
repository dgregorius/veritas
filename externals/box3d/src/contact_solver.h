// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "solver.h"
#include "math_internal.h"

struct b3ContactSim;

struct b3ContactConstraintPoint
{
	b3Vec3 anchorA, anchorB;
	float baseSeparation;
	float relativeVelocity;
	float normalImpulse;
	float totalNormalImpulse;
	float normalMass;
};

struct b3ContactConstraint
{
	b3ContactConstraintPoint points[4];
	int pointCount;
	int indexA;
	int indexB;
	float invMassA, invMassB;
	b3Matrix3 invIA, invIB;
	b3Vec3 normal;
	b3Vec3 tangent1;
	b3Vec3 tangent2;
	b3Vec3 originA, originB;
	float twistMass;
	float twistImpulse;
	b3Matrix2 tangentMass;
	b3Vec2 frictionImpulse;
	b3Matrix3 rollingMass;
	b3Vec3 rollingImpulse;
	float leverArms[4];
	float friction;
	float rollingResistance;
	float tangentVelocity1;
	float tangentVelocity2;
	float restitution;
	b3Softness softness;

	// Back pointer for storing results
	b3Manifold* manifold;
};

int b3GetContactConstraintSIMDByteCount();

// Overflow contacts don't fit into the constraint graph coloring
void b3PrepareOverflowContacts( b3StepContext* context );
void b3WarmStartOverflowContacts( b3StepContext* context );
void b3SolveOverflowContacts( b3StepContext* context, bool useBias );
void b3ApplyOverflowRestitution( b3StepContext* context );
void b3StoreOverflowImpulses( b3StepContext* context );

// Contacts that live within the constraint graph coloring
void b3PrepareContactsTask( int startIndex, int endIndex, b3StepContext* context, int manifoldCount);

void b3WarmStartManifoldTask( int startIndex, int endIndex, b3StepContext* context, int colorIndex, int manifoldCount );
void b3SolveManifoldTask( int startIndex, int endIndex, b3StepContext* context, int colorIndex, int manifoldCount, bool useBias );
void b3ApplyRestitutionTask( int startIndex, int endIndex, b3StepContext* context, int colorIndex, int manifoldCount );

void b3StoreImpulsesTask( int startIndex, int endIndex, b3StepContext* context, int manifoldCount );
