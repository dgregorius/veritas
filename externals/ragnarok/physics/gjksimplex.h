//--------------------------------------------------------------------------------------------------
/*
	@file		gjksimplex.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/math.h"
#include "ragnarok/common/memory.h"

class RkGJKProxy;


//--------------------------------------------------------------------------------------------------
// RkGJKCache
//--------------------------------------------------------------------------------------------------
#define RK_MAX_SIMPLEX_VERTICES	4 

struct RkGJKCache
	{
	float Metric = 0.0f;
	int VertexCount = 0;
	uint8 Vertices1[ RK_MAX_SIMPLEX_VERTICES ] = { 0 };
	uint8 Vertices2[ RK_MAX_SIMPLEX_VERTICES ] = { 0 };
	float Lambdas[ RK_MAX_SIMPLEX_VERTICES ] = { 0.0f };
	};


void rkClearCache( RkGJKCache& Cache );


//--------------------------------------------------------------------------------------------------
// RkGJKVertex
//--------------------------------------------------------------------------------------------------
struct RkGJKVertex
	{
	int Index1;
	int Index2;

	RkVector3 Position1;
	RkVector3 Position2;
	RkVector3 Position;
	};


//--------------------------------------------------------------------------------------------------
// RkGJKSimplex 
//--------------------------------------------------------------------------------------------------
class RkGJKSimplex
	{
	public:
		RkGJKSimplex();

		void ReadCache( const RkTransform& Transform1, const RkGJKProxy& Proxy1, const RkTransform& Transform2, const RkGJKProxy& Proxy2, const RkGJKCache& Cache );
		void WriteCache( RkGJKCache& Cache ) const;

		int GetVertexCount() const;
		RkGJKVertex& GetVertex( int Index );
		const RkGJKVertex& GetVertex( int Index ) const;
		bool IsEmpty() const;

		bool Solve();
		RkVector3 GetClosestPoint() const;
		RkVector3 GetSearchDirection() const;
		bool AddVertex( int Index1, const RkVector3& Vertex1, int Index2, const RkVector3& Vertex2 );

		void BuildWitnessPoints( RkVector3& Vertex1, RkVector3& Vertex2 ) const;

	private:
		float GetMetric() const;
		bool CheckCache( int Index1, int Index2 ) const;
		
		bool Solve1();
		bool Solve2();
		bool Solve3();
		bool Solve4();

		int mVertexCount;
		float mLambdas[ RK_MAX_SIMPLEX_VERTICES ];
		RkGJKVertex mVertices[ RK_MAX_SIMPLEX_VERTICES ];
		RkGJKCache mCache;
	};