//--------------------------------------------------------------------------------------------------
/*
	@file		sat.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/math.h"

struct RkCapsule;
struct RkHull;
struct RkManifold;


//--------------------------------------------------------------------------------------------------
// RkSATCache
//--------------------------------------------------------------------------------------------------
#define RK_SAT_UNKNOWN		0 
#define RK_SAT_FACE1		1
#define RK_SAT_FACE2		2
#define RK_SAT_EDGES		3

struct RkSATCache
	{
	int Type = RK_SAT_UNKNOWN;
	int Index1 = -1;
	int Index2 = -1;

	float Separation = 0.0f;
	};

void rkClearCache( RkSATCache& Cache );


//--------------------------------------------------------------------------------------------------
// RkFaceQuery
//--------------------------------------------------------------------------------------------------
struct RkFaceQuery
	{
	int FaceIndex;
	int VertexIndex;
	float Separation;

	bool IsValid() const;
	};

float rkProject( const RkPlane3& Plane, const RkHull* Hull );
void rkQueryFaceDirections( RkFaceQuery& Out, const RkTransform& CapsuleTransform, const RkCapsule& Capsule, const RkTransform& HullTransform, const RkHull* Hull );
void rkQueryFaceDirections( RkFaceQuery& Out, const RkTransform& Transform1, const RkHull* Hull1, const RkTransform& Transform2, const RkHull* Hull2 );


//--------------------------------------------------------------------------------------------------
// RkEdgeQuery
//--------------------------------------------------------------------------------------------------
struct RkEdgeQuery
	{
	int Index1;
	int Index2;
	float Separation;

	bool IsValid() const;
	};

bool rkIsMinkowskiFace( const RkVector3& A, const RkVector3& B, const RkVector3& N );
bool rkIsMinkowskiFace( const RkVector3& A, const RkVector3& B, const RkVector3& B_x_A, const RkVector3& C, const RkVector3& D, const RkVector3& D_x_C );

float rkProject( const RkVector3& P1, const RkVector3& E1, const RkVector3& P2, const RkVector3& E2, const RkVector3& C1 );
void rkQueryEdgeDirections( RkEdgeQuery& Out, const RkTransform& CapsuleTransform, const RkCapsule& Capsule, const RkTransform& HullTransform, const RkHull* Hull );
void rkQueryEdgeDirections( RkEdgeQuery& Out, const RkTransform& Transform1, const RkHull* Hull1, const RkTransform& Transform2, const RkHull* Hull2 );


//--------------------------------------------------------------------------------------------------
// SAT Collision
//--------------------------------------------------------------------------------------------------
void rkCollide( RkManifold& Manifold, const RkTransform& CapsuleTransform, const RkCapsule& Capsule, const RkTransform& HullTransform, const RkHull* Hull, int Triangle = -1 );
void rkCollide( RkManifold& Manifold, const RkTransform& Transform1, const RkHull* Hull1, const RkTransform& Transform2, const RkHull* Hull2, RkSATCache& Cache, int Triangle = -1 );