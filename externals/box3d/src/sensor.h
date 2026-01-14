// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "bitset.h"

struct b3Shape;
struct b3World;

// Used to track shapes that hit sensors using time of impact
typedef struct b3SensorHit
{
	int sensorId;
	int visitorId;
} b3SensorHit;

struct b3Visitor
{
	int shapeId;
	uint16_t generation;
};

struct b3Sensor
{
	b3Array<b3Visitor> hits;
	b3Array<b3Visitor> overlaps1;
	b3Array<b3Visitor> overlaps2;
	int shapeId;
};

struct b3SensorTaskContext
{
	b3BitSet eventBits;
};

void b3OverlapSensors( b3World* world );

void b3DestroySensor( b3World* world, b3Shape* sensorShape );
