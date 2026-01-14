//--------------------------------------------------------------------------------------------------
/*
	@file		gjk.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/math.h"
#include "ragnarok/common/types.h"

struct RkGJKCache;
struct RkHull;


//--------------------------------------------------------------------------------------------------
// RkGJKFeature
//--------------------------------------------------------------------------------------------------
enum RkGJKFeature
	{
	RK_GJK_VERTEX,
	RK_GJK_EDGE,
	RK_GJK_FACE,
	};


//--------------------------------------------------------------------------------------------------
// RkGJKWitness
//--------------------------------------------------------------------------------------------------
struct RkGJKWitness
	{
	RkGJKFeature Feature;
	int Index;
	};


//--------------------------------------------------------------------------------------------------
// RkGJKWitnessPair
//--------------------------------------------------------------------------------------------------
struct RkGJKWitnessPair
	{
	RkGJKWitness Witness1;
	RkGJKWitness Witness2;
	};


//--------------------------------------------------------------------------------------------------
// RkGJKQuery
//--------------------------------------------------------------------------------------------------
struct RkGJKQuery
	{
	float Distance;
	RkVector3 Point1;
	RkVector3 Point2;
	};


//--------------------------------------------------------------------------------------------------
// RkGJKProxy
//--------------------------------------------------------------------------------------------------
class RkGJKProxy
	{
	public:
		RkGJKProxy( int VertexCount, const RkVector3* VertexBuffer );

		int GetVertexCount() const;
		const RkVector3* GetVertexBuffer() const;
		const RkVector3& GetVertex( int Index ) const;

		int GetSupport( const RkVector3& Axis ) const;

	protected:
		int mVertexCount;
		const RkVector3* mVertexBuffer;
	};


//--------------------------------------------------------------------------------------------------
// GJK (closest points)
//--------------------------------------------------------------------------------------------------
RkGJKQuery rkGJK( const RkTransform& Transform1, const RkGJKProxy& Proxy1, const RkTransform& Transform2, const RkGJKProxy& Proxy2, int MaxIterations = 32 );
RkGJKQuery rkGJK( const RkTransform& Transform1, const RkGJKProxy& Proxy1, const RkTransform& Transform2, const RkGJKProxy& Proxy2, RkGJKCache& Cache, int MaxIterations = 32 );
RkGJKWitnessPair rkResolveCache( RkGJKCache Cache, const RkHull* Hull1 = nullptr, const RkHull* Hull2 = nullptr );


#include "gjk.inl"