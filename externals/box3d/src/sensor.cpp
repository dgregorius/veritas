// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "sensor.h"

#include "array.h"
#include "body.h"
#include "contact.h"
#include "ctz.h"
#include "physics_world.h"
#include "shape.h"

#include "box3d/collision.h"

// for qsort
#include <stdlib.h>

struct b3SensorQueryContext
{
	b3World* world;
	b3SensorTaskContext* taskContext;
	b3Sensor* sensor;
	b3Shape* sensorShape;
	b3Transform transform;
};

static bool b3OverlapSensor( b3Shape* sensorShape, b3Transform sensorTransform, b3Shape* visitorShape,
							 b3Transform visitorTransform )
{
	b3ShapeType type = sensorShape->type;

	b3ShapeProxy proxy = b3MakeShapeProxy( visitorShape );

	// Get the visitor shape in the frame of the sensor
	b3Transform relativeTransform = b3InvMulTransforms( sensorTransform, visitorTransform );

	b3Vec3 localPoints[B3_MAX_SHAPE_CAST_POINTS];
	b3ShapeProxy localProxy;

	localProxy.count = b3MinInt( proxy.count, B3_MAX_SHAPE_CAST_POINTS );
	for ( int i = 0; i < localProxy.count; ++i )
	{
		localPoints[i] = b3TransformPoint( relativeTransform, proxy.points[i] );
	}

	localProxy.points = localPoints;
	localProxy.radius = proxy.radius;

	switch ( type )
	{
		case b3_capsuleShape:
			return b3OverlapCapsule( &sensorShape->capsule, b3Transform_identity, &localProxy );

		case b3_compoundShape:
			return b3OverlapCompound( sensorShape->compound , b3Transform_identity, &localProxy );

		case b3_heightShape:
			return b3OverlapHeightField( sensorShape->heightField, b3Transform_identity, &localProxy );

		case b3_hullShape:
			return b3OverlapHull( sensorShape->hull, b3Transform_identity, &localProxy );

		case b3_meshShape:
			return b3OverlapMesh( &sensorShape->mesh, b3Transform_identity, &localProxy );

		case b3_sphereShape:
			return b3OverlapSphere( &sensorShape->sphere, b3Transform_identity, &localProxy );

		default:
			B3_ASSERT( false );
			return false;
	}
}

// Sensor shapes need to
// - detect begin and end overlap events
// - events must be reported in deterministic order
// - maintain an active list of overlaps for query

// Assumption
// - sensors don't detect shapes on the same body

// Algorithm
// Query all sensors for overlaps
// Check against previous overlaps

// Data structures
// Each sensor has an double buffered array of overlaps
// These overlaps use a shape reference with index and generation

static bool b3SensorQueryCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = (int)userData;

	b3SensorQueryContext* queryContext = (b3SensorQueryContext*)context;
	b3Shape* sensorShape = queryContext->sensorShape;
	int sensorShapeId = sensorShape->id;

	if ( shapeId == sensorShapeId )
	{
		return true;
	}

	b3World* world = queryContext->world;
	b3Shape* otherShape = world->shapes.Get( shapeId );

	// Mesh vs mesh is not supported
	if ( ( otherShape->type == b3_meshShape || otherShape->type == b3_heightShape ) &&
		 ( sensorShape->type == b3_meshShape || sensorShape->type == b3_heightShape ) )
	{
		return true;
	}

	// Are sensor events enabled on the other shape?
	if ( otherShape->enableSensorEvents == false )
	{
		return true;
	}

	// Skip shapes on the same body
	if ( otherShape->bodyId == sensorShape->bodyId )
	{
		return true;
	}

	// Check filter
	if ( b3ShouldShapesCollide( sensorShape->filter, otherShape->filter ) == false )
	{
		return true;
	}

	// Custom user filter
	if ( sensorShape->enableCustomFiltering || otherShape->enableCustomFiltering )
	{
		b3CustomFilterFcn* customFilterFcn = queryContext->world->customFilterFcn;
		if ( customFilterFcn != nullptr )
		{
			b3ShapeId idA = { sensorShapeId + 1, world->worldId, sensorShape->generation };
			b3ShapeId idB = { shapeId + 1, world->worldId, otherShape->generation };
			bool shouldCollide = customFilterFcn( idA, idB, queryContext->world->customFilterContext );
			if ( shouldCollide == false )
			{
				return true;
			}
		}
	}

	b3Transform otherTransform = b3GetBodyTransform( world, otherShape->bodyId );

	bool overlap = b3OverlapSensor( sensorShape, queryContext->transform, otherShape, otherTransform );
	if ( overlap == false )
	{
		return true;
	}

	// Record the overlap
	b3Sensor* sensor = queryContext->sensor;
	b3Visitor* shapeRef = sensor->overlaps2.Add();
	shapeRef->shapeId = shapeId;
	shapeRef->generation = otherShape->generation;

	return true;
}

static int b3CompareVisitors( const void* a, const void* b )
{
	const b3Visitor* sa = (const b3Visitor*)a;
	const b3Visitor* sb = (const b3Visitor*)b;

	if ( sa->shapeId < sb->shapeId )
	{
		return -1;
	}

	return 1;
}

static void b3SensorTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b3TracyCZoneNC( sensor_task, "Overlap", b3_colorBrown, true );

	b3World* world = (b3World*)context;
	B3_ASSERT( (int)threadIndex < world->workerCount );
	b3SensorTaskContext* taskContext = world->sensorTaskContexts.data + threadIndex;

	B3_ASSERT( startIndex < endIndex );

	b3DynamicTree* trees = world->broadPhase.trees;
	for ( int sensorIndex = startIndex; sensorIndex < endIndex; ++sensorIndex )
	{
		b3Sensor* sensor = world->sensors.Get( sensorIndex );
		b3Shape* sensorShape = world->shapes.Get( sensor->shapeId );

		// Swap overlap arrays
		b3Array<b3Visitor> temp = sensor->overlaps1;
		sensor->overlaps1 = sensor->overlaps2;
		sensor->overlaps2 = temp;
		sensor->overlaps2.Clear();

		// Append sensor hits
		sensor->overlaps2.Append( sensor->hits.data, sensor->hits.count );

		// Clear the hits
		sensor->hits.Clear();

		b3Body* body = world->bodies.Get( sensorShape->bodyId );
		if ( body->setIndex == b3_disabledSet || sensorShape->enableSensorEvents == false )
		{
			if ( sensor->overlaps1.count != 0 )
			{
				// This sensor is dropping all overlaps because it has been disabled.
				b3SetBit( &taskContext->eventBits, sensorIndex );
			}
			continue;
		}

		b3Transform transform = b3GetBodyTransformQuick( world, body );

		b3SensorQueryContext queryContext = {
			.world = world,
			.taskContext = taskContext,
			.sensor = sensor,
			.sensorShape = sensorShape,
			.transform = transform,
		};

		B3_ASSERT( sensorShape->sensorIndex == sensorIndex );
		b3AABB queryBounds = sensorShape->aabb;

		// Query all trees
		b3DynamicTree_Query( trees + 0, queryBounds, sensorShape->filter.maskBits, false, b3SensorQueryCallback, &queryContext );
		b3DynamicTree_Query( trees + 1, queryBounds, sensorShape->filter.maskBits, false, b3SensorQueryCallback, &queryContext );
		b3DynamicTree_Query( trees + 2, queryBounds, sensorShape->filter.maskBits, false, b3SensorQueryCallback, &queryContext );

		// Sort the overlaps to enable finding begin and end events.
		qsort( sensor->overlaps2.data, sensor->overlaps2.count, sizeof( b3Visitor ), b3CompareVisitors );

		// Remove duplicates from overlaps2 (sorted). Duplicates are possible due to the hit events appended earlier.
		int uniqueCount = 0;
		int overlapCount = sensor->overlaps2.count;
		b3Visitor* overlapData = sensor->overlaps2.data;
		for ( int i = 0; i < overlapCount; ++i )
		{
			if ( uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount - 1].shapeId )
			{
				overlapData[uniqueCount] = overlapData[i];
				uniqueCount += 1;
			}
		}
		sensor->overlaps2.count = uniqueCount;

		int count1 = sensor->overlaps1.count;
		int count2 = sensor->overlaps2.count;
		if ( count1 != count2 )
		{
			// something changed
			b3SetBit( &taskContext->eventBits, sensorIndex );
		}
		else
		{
			for ( int i = 0; i < count1; ++i )
			{
				b3Visitor* s1 = sensor->overlaps1.data + i;
				b3Visitor* s2 = sensor->overlaps2.data + i;

				if ( s1->shapeId != s2->shapeId || s1->generation != s2->generation )
				{
					// something changed
					b3SetBit( &taskContext->eventBits, sensorIndex );
					break;
				}
			}
		}
	}

	b3TracyCZoneEnd( sensor_task );
}

void b3OverlapSensors( b3World* world )
{
	int sensorCount = world->sensors.count;
	if ( sensorCount == 0 )
	{
		return;
	}

	B3_ASSERT( world->workerCount > 0 );

	b3TracyCZoneNC( overlap_sensors, "Sensors", b3_colorMediumPurple, true );

	for ( int i = 0; i < world->workerCount; ++i )
	{
		b3SetBitCountAndClear( &world->sensorTaskContexts.data[i].eventBits, sensorCount );
	}

	// Parallel-for sensors overlaps
	int minRange = 16;
	void* userSensorTask = world->enqueueTaskFcn( &b3SensorTask, sensorCount, minRange, world, world->userTaskContext );
	world->taskCount += 1;
	if ( userSensorTask != nullptr )
	{
		world->finishTaskFcn( userSensorTask, world->userTaskContext );
	}

	b3TracyCZoneNC( sensor_state, "Events", b3_colorLightSlateGray, true );

	b3BitSet* bitSet = &world->sensorTaskContexts.data[0].eventBits;
	for ( int i = 1; i < world->workerCount; ++i )
	{
		b3InPlaceUnion( bitSet, &world->sensorTaskContexts.data[i].eventBits );
	}

	// Iterate sensors bits and publish events
	// Process sensor state changes. Iterate over set bits
	uint64_t* bits = bitSet->bits;
	uint32_t blockCount = bitSet->blockCount;

	for ( uint32_t k = 0; k < blockCount; ++k )
	{
		uint64_t word = bits[k];
		while ( word != 0 )
		{
			uint32_t ctz = b3CTZ64( word );
			int sensorIndex = (int)( 64 * k + ctz );

			b3Sensor* sensor = world->sensors.Get( sensorIndex );
			b3Shape* sensorShape = world->shapes.Get( sensor->shapeId );
			b3ShapeId sensorId = { sensor->shapeId + 1, world->worldId, sensorShape->generation };

			int count1 = sensor->overlaps1.count;
			int count2 = sensor->overlaps2.count;
			const b3Visitor* refs1 = sensor->overlaps1.data;
			const b3Visitor* refs2 = sensor->overlaps2.data;

			// overlaps1 can have overlaps that end
			// overlaps2 can have overlaps that begin
			int index1 = 0, index2 = 0;
			while ( index1 < count1 && index2 < count2 )
			{
				const b3Visitor* r1 = refs1 + index1;
				const b3Visitor* r2 = refs2 + index2;
				if ( r1->shapeId == r2->shapeId )
				{
					if ( r1->generation < r2->generation )
					{
						// end
						b3ShapeId visitorId = { r1->shapeId + 1, world->worldId, r1->generation };
						b3SensorEndTouchEvent event = {
							.sensorShapeId = sensorId,
							.visitorShapeId = visitorId,
						};
						world->sensorEndEvents[world->endEventArrayIndex].PushBack( event );
						index1 += 1;
					}
					else if ( r1->generation > r2->generation )
					{
						// begin
						b3ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
						b3SensorBeginTouchEvent event = { sensorId, visitorId };
						world->sensorBeginEvents.PushBack( event );
						index2 += 1;
					}
					else
					{
						// persisted
						index1 += 1;
						index2 += 1;
					}
				}
				else if ( r1->shapeId < r2->shapeId )
				{
					// end
					b3ShapeId visitorId = { r1->shapeId + 1, world->worldId, r1->generation };
					b3SensorEndTouchEvent event = { sensorId, visitorId };
					world->sensorEndEvents[world->endEventArrayIndex].PushBack( event );
					index1 += 1;
				}
				else
				{
					// begin
					b3ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
					b3SensorBeginTouchEvent event = { sensorId, visitorId };
					world->sensorBeginEvents.PushBack( event );
					index2 += 1;
				}
			}

			while ( index1 < count1 )
			{
				// end
				const b3Visitor* r1 = refs1 + index1;
				b3ShapeId visitorId = { r1->shapeId + 1, world->worldId, r1->generation };
				b3SensorEndTouchEvent event = { sensorId, visitorId };
				world->sensorEndEvents[world->endEventArrayIndex].PushBack( event );
				index1 += 1;
			}

			while ( index2 < count2 )
			{
				// begin
				const b3Visitor* r2 = refs2 + index2;
				b3ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
				b3SensorBeginTouchEvent event = { sensorId, visitorId };
				world->sensorBeginEvents.PushBack( event );
				index2 += 1;
			}

			// Clear the smallest set bit
			word = word & ( word - 1 );
		}
	}

	b3TracyCZoneEnd( sensor_state );
	b3TracyCZoneEnd( overlap_sensors );
}

void b3DestroySensor( b3World* world, b3Shape* sensorShape )
{
	b3Sensor* sensor = world->sensors.Get( sensorShape->sensorIndex );
	for ( int i = 0; i < sensor->overlaps2.count; ++i )
	{
		b3Visitor* ref = sensor->overlaps2.data + i;
		b3SensorEndTouchEvent event = {
			.sensorShapeId =
				{
					.index1 = sensorShape->id + 1,
					.world0 = world->worldId,
					.generation = sensorShape->generation,
				},
			.visitorShapeId =
				{
					.index1 = ref->shapeId + 1,
					.world0 = world->worldId,
					.generation = ref->generation,
				},
		};

		world->sensorEndEvents[world->endEventArrayIndex].PushBack( event );
	}

	// Destroy sensor
	sensor->hits.Destroy();
	sensor->overlaps1.Destroy();
	sensor->overlaps2.Destroy();

	int movedIndex = world->sensors.RemoveSwap( sensorShape->sensorIndex );
	if ( movedIndex != B3_NULL_INDEX )
	{
		// Fixup moved sensor
		b3Sensor* movedSensor = world->sensors.Get( sensorShape->sensorIndex );
		b3Shape* otherSensorShape = world->shapes.Get( movedSensor->shapeId );
		otherSensorShape->sensorIndex = sensorShape->sensorIndex;
	}
}
