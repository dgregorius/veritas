//--------------------------------------------------------------------------------------------------
/*
	@file		qhConvex.h

	@author		Dirk Gregorius
	@version	0.1
	@date		30/11/2011

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "qhTypes.h"
#include "qhMath.h"
#include "qhArray.h"
#include "qhList.h"
#include "qhHalfEdge.h"


//--------------------------------------------------------------------------------------------------
// qhConvex
//--------------------------------------------------------------------------------------------------
class qhConvex
	{
	public:
		// Construction / Destruction
		qhConvex();
		qhConvex( const qhConvex& ) = delete;
		qhConvex( qhConvex&& ) = delete;
		~qhConvex();

		qhConvex& operator=( const qhConvex& ) = delete;
		qhConvex& operator=( qhConvex&& ) = delete;
		
		void Construct( const qhArray< qhVector3 >& Vertices );
		void Construct( int VertexCount, const qhReal32* VertexList );
		void Construct( int VertexCount, const qhReal64* VertexList );
		bool IsConsistent() const;
		
		// Accessors / Mutators
		int GetVertexCount() const;
		int GetHalfEdgeCount() const;
		int GetFaceCount() const;

		const qhList< qhVertex >& GetVertexList() const;
		const qhList< qhFace >& GetFaceList() const;

		qhBounds3 ComputeBounds() const;
		qhMassProperties ComputeMass( qhReal Density ) const;

		void ShiftHull( const qhVector3& Translation );
		void ScaleHull( qhReal Scale );

	private:
		// Memory management
		void AllocateMemory( int VertexCount );
		qhVertex* CreateVertex( const qhVector3& Position, int Index );
		void DestroyVertex( qhVertex* Vertex );
		qhFace* CreateFace( qhVertex* Vertex1, qhVertex* Vertex2, qhVertex* Vertex3 );
		void DestroyFace( qhFace* Face );

		// Implementation
		void ComputeTolerance( int VertexCount, const qhVector3* VertexBase ); 
		bool BuildInitialHull( int VertexCount, const qhVector3* VertexBase );
		qhVertex* NextConflictVertex();
		void AddVertexToHull( qhVertex* Vertex );
		void CleanHull();
		
		void BuildHorizon( qhArray< qhHalfEdge* >& Horizon, qhVertex* Apex, qhFace* Seed, qhHalfEdge* Edge1 = nullptr );
		void BuildCone( qhArray< qhFace* >& Cone, const qhArray< qhHalfEdge* >& Horizon, qhVertex* Apex );
		void MergeFaces( const qhArray< qhFace* >& Cone );
		void ResolveVertices( const qhArray< qhFace* >& Cone );
		void ResolveFaces( const qhArray< qhFace* >& Cone );
		
		bool MergeConcave( qhFace* Face );
		bool MergeCoplanar( qhFace* Face  );
		void ConnectFaces( qhHalfEdge* Edge );
		void ConnectEdges( qhHalfEdge* Prev, qhHalfEdge* Next, qhArray< qhFace* >& MergedFaces );
		void DestroyEdges( qhHalfEdge* Begin, qhHalfEdge* End );
		void AbsorbFaces( qhFace* Face, qhArray< qhFace* >& MergedFaces );
		
		// Data members
		qhReal mTolerance;
		qhReal mMinRadius;
		qhReal mMinOutside;
	
		qhVector3 mInteriorPoint;
		qhList< qhVertex > mOrphanedList;
		qhList< qhVertex > mVertexList;
		qhList< qhFace > mFaceList;

		qhPool< qhVertex > mVertexPool;
		qhPool< qhHalfEdge > mEdgePool;
		qhPool< qhFace > mFacePool;
	};


#include "qhConvex.inl"
