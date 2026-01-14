//--------------------------------------------------------------------------------------------------
// qhConvex.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "qhConvex.h"

#include <algorithm>
#include <limits>

#define QH_AXIS_X	0
#define QH_AXIS_Y	1
#define QH_AXIS_Z	2


//--------------------------------------------------------------------------------------------------
// Local construction utilities
//--------------------------------------------------------------------------------------------------
static inline qhBounds3 qhBuildBounds( int VertexCount, const qhVector3* Vertices )
	{
	qhBounds3 Bounds = QH_BOUNDS3_EMPTY;
	for ( int i = 0; i < VertexCount; ++i )
		{
		Bounds += Vertices[ i ];
		}

	return Bounds;
	}


//--------------------------------------------------------------------------------------------------
static void qhFindFarthestPointsAlongCardinalAxes( int& Index1, int& Index2, qhReal Tolerance, int VertexCount, const qhVector3* VertexBase ) 
	{
	Index1 = Index2 = -1;
	
	qhVector3 V0 = VertexBase[ 0 ];
	qhVector3 Min[ 3 ] = { V0, V0, V0 };
	qhVector3 Max[ 3 ] = { V0, V0, V0 };

	int MinIndex[ 3 ] = { 0, 0, 0 };
	int MaxIndex[ 3 ] = { 0, 0, 0 };

	for ( int i = 1; i < VertexCount; ++i )
		{
		qhVector3 V = VertexBase[ i ];

		// X-Axis
		if ( V.X < Min[ QH_AXIS_X ].X  )
			{
			Min[ QH_AXIS_X ] = V;
			MinIndex[ QH_AXIS_X ] = i;
			}
		else if ( V.X > Max[ QH_AXIS_X ].X )
			{
			Max[ QH_AXIS_X ] = V;
			MaxIndex[ QH_AXIS_X ] = i;
			}

		// Y-Axis
		if ( V.Y < Min[ QH_AXIS_Y ].Y )
			{
			Min[ QH_AXIS_Y ] = V;
			MinIndex[ QH_AXIS_Y ] = i;
			}
		else if ( V.Y > Max[ QH_AXIS_Y ].Y )
			{
			Max[ QH_AXIS_Y ] = V;
			MaxIndex[ QH_AXIS_Y ] = i;
			}

		// Z-Axis
		if ( V.Z < Min[ QH_AXIS_Z ].Z )
			{
			Min[ QH_AXIS_Z ] = V;
			MinIndex[ QH_AXIS_Z ] = i;
			}
		else if ( V.Z > Max[ QH_AXIS_Z ].Z )
			{
			Max[ QH_AXIS_Z ] = V;
			MaxIndex[ QH_AXIS_Z ] = i;
			}
		}

	qhVector3 Distance;
	Distance[ QH_AXIS_X ] = Max[ QH_AXIS_X ].X - Min[ QH_AXIS_X ].X; 
	Distance[ QH_AXIS_Y ] = Max[ QH_AXIS_Y ].Y - Min[ QH_AXIS_Y ].Y; 
	Distance[ QH_AXIS_Z ] = Max[ QH_AXIS_Z ].Z - Min[ QH_AXIS_Z ].Z; 

	int MaxElement = qhMaxElement( Distance );
	if ( Distance[ MaxElement ] > qhReal( 2 ) * Tolerance )
		{
		Index1 = MinIndex[ MaxElement ];
		Index2 = MaxIndex[ MaxElement ];
		}
	}


//--------------------------------------------------------------------------------------------------
static int qhFindFarthestPointFromLine( int Index1, int Index2, qhReal Tolerance, int VertexCount, const qhVector3* VertexBase ) 
	{
	qhVector3 A = VertexBase[ Index1 ];
	qhVector3 B = VertexBase[ Index2 ];

	qhVector3 AB = B - A;
	qhReal MaxDistance = qhReal( 2 ) * Tolerance;
	int MaxIndex = -1;

	for ( int i = 0; i < VertexCount; ++i )
		{
		if ( i == Index1 || i == Index2 )
			{
			continue;
			}

		const qhVector3& P = VertexBase[ i ];

		qhVector3 AP = P - A;
		qhReal s = qhDot( AP, AB ) / qhDot( AB, AB );
		qhVector3 Q = A + s * AB;

		qhReal Distance = qhDistance( P, Q );
		if ( Distance > MaxDistance )
			{
			MaxDistance = Distance;
			MaxIndex = i;
			}
		}

	return MaxIndex;
	}


//--------------------------------------------------------------------------------------------------
static int qhFindFarthestPointFromPlane( int Index1, int Index2, int Index3, qhReal Tolerance, int VertexCount, const qhVector3* VertexBase ) 
	{
	qhVector3 A = VertexBase[ Index1 ];
	qhVector3 B = VertexBase[ Index2 ];
	qhVector3 C = VertexBase[ Index3 ];
	
	qhPlane Plane = qhPlane( A, B, C );
	Plane.Normalize();
	
	qhReal MaxDistance = qhReal( 2 ) * Tolerance;
	int MaxIndex = -1;

	for ( int i = 0; i < VertexCount; ++i )
		{
		if ( i == Index1 || i == Index2 || i == Index3 )
			{
			continue;
			}

		qhReal Distance = qhAbs( Plane.Distance( VertexBase[ i ] ) );
		if ( Distance > MaxDistance )
			{
			MaxDistance = Distance;
			MaxIndex = i;
			}
		}

	return MaxIndex;
	}


//--------------------------------------------------------------------------------------------------
// qhConvex
//--------------------------------------------------------------------------------------------------
qhConvex::qhConvex()
	: mTolerance( 0 )
	, mMinRadius( 0 )
	, mMinOutside( 0 )
	, mInteriorPoint( QH_VEC3_ZERO )
	{

	}


//--------------------------------------------------------------------------------------------------
qhConvex::~qhConvex()
	{
	// Destroy faces
	qhFace* Face = mFaceList.Begin();
	while ( Face != mFaceList.End() )
		{
		qhFace* Nuke = Face;
		Face = Face->Next;

		qhRemove( Nuke );
		DestroyFace( Nuke );
		}

	// Destroy vertices
	qhVertex* Vertex = mVertexList.Begin();
	while ( Vertex != mVertexList.End() )
		{
		qhVertex* Nuke = Vertex;
		Vertex = Vertex->Next;

		qhRemove( Nuke );
		DestroyVertex( Nuke );
		}
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::Construct( const qhArray< qhVector3 >& Vertices )
	{
	// Validate passed arguments
	if ( Vertices.Size() < 4 )
		{
		return;
		}

	// Try to build an initial hull 
	AllocateMemory( Vertices.Size() );
	ComputeTolerance( Vertices.Size(), Vertices.Begin() );
	if ( !BuildInitialHull( Vertices.Size(), Vertices.Begin() ) )
		{
		return;
		}

	// Construct hull
	qhVertex* Vertex = NextConflictVertex();
	while ( Vertex )
		{
		AddVertexToHull( Vertex );
		Vertex = NextConflictVertex();
		}

	CleanHull();
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::Construct( int VertexCount, const qhReal32* VertexList )
	{
	qhArray< qhVector3 > Vertices;
	Vertices.Resize( VertexCount );

	for ( int Index = 0; Index < VertexCount; ++Index )
		{
		qhReal X( VertexList[ 3 * Index + 0 ] );
		qhReal Y( VertexList[ 3 * Index + 1 ] );
		qhReal Z( VertexList[ 3 * Index + 2 ] );

		Vertices[ Index ] = qhVector3( X, Y, Z );
		}

	Construct( Vertices );
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::Construct( int VertexCount, const qhReal64* VertexList )
	{
	qhArray< qhVector3 > Vertices;
	Vertices.Resize( VertexCount );

	for ( int Index = 0; Index < VertexCount; ++Index )
		{
		qhReal X = static_cast< qhReal >( VertexList[ 3 * Index + 0 ] );
		qhReal Y = static_cast< qhReal >( VertexList[ 3 * Index + 1 ] );
		qhReal Z = static_cast< qhReal >( VertexList[ 3 * Index + 2 ] );

		Vertices[ Index ] = qhVector3( X, Y, Z );
		}

	Construct( Vertices );
	}


//--------------------------------------------------------------------------------------------------
bool qhConvex::IsConsistent() const
	{
	// Convex polyhedron invariants
	int V = GetVertexCount();
	int E = GetHalfEdgeCount() / 2;
	int F = GetFaceCount();

	// Euler's identity 
	if ( V - E + F != 2 )
		{
		return false;
		}

	// Edge and face invariants
	for ( const qhFace* Face = mFaceList.Begin(); Face != mFaceList.End(); Face = Face->Next )
		{
		// Face invariants (Topology)
		if ( Face->Edge->Face != Face )
			{
			return false;
			}

		// Face invariants (Geometry)
		if ( !qhCheckConsistency( Face ) )
			{
			return false;
			}

// 		if ( !qhIsConvex( Face, mMinRadius ) )
// 			{
// 			return false;
// 			}

		if ( Face->Plane.Distance( mInteriorPoint ) > 0 )
			{
			return false;
			}

		if ( Face->Mark != QH_MARK_VISIBLE )
			{
			return false;
			}

		const qhHalfEdge* Edge = Face->Edge;

		do 
			{
			// DIRK_TODO: Vertex invariant -> each vertex must be connected to > 2 edges...

			// Edge invariants (Topology)
			if ( Edge->Next->Origin != Edge->Twin->Origin )
				{
				return false;
				}
			if ( Edge->Prev->Next != Edge )
				{
				return false;
				}

			if ( Edge->Next->Prev != Edge )
				{
				return false;
				}

			if ( Edge->Twin->Twin != Edge )
				{
				return false;
				}

			if ( Edge->Face != Face )
				{
				return false;
				}

			// Edge invariants (Geometry)
// 			if ( !Edge->IsConvex( mMinRadius ) )
// 				{
// 				return false;
// 				}

			if ( qhDistanceSq( Edge->Origin->Position, Edge->Twin->Origin->Position ) < qhReal( 1000 ) * QH_REAL_MIN )
				{
				return false;
				}

			Edge = Edge->Next;
			} 
		while ( Edge != Face->Edge );
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
qhBounds3 qhConvex::ComputeBounds() const
	{
	qhBounds3 Bounds = QH_BOUNDS3_EMPTY;
	
	for ( const qhVertex* Vertex = mVertexList.Begin(); Vertex != mVertexList.End(); Vertex = Vertex->Next )
		{
		Bounds += Vertex->Position;
		}

	return Bounds;
	}


//--------------------------------------------------------------------------------------------------
qhMassProperties qhConvex::ComputeMass( qhReal Density ) const
	{
	// M. Kallay - "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
	qhReal Volume = 0;
	qhVector3 Center = QH_VEC3_ZERO;

	qhReal XX = 0;  qhReal XY = 0;
	qhReal YY = 0;  qhReal XZ = 0;
	qhReal ZZ = 0;  qhReal YZ = 0;

	for ( const qhFace* Face = mFaceList.Begin(); Face != mFaceList.End(); Face = Face->Next )
		{
		const qhHalfEdge* Edge1 = Face->Edge;
		const qhHalfEdge* Edge2 = Edge1->Next;
		const qhHalfEdge* Edge3 = Edge2->Next;
		QH_ASSERT( Edge1 != Edge3 );

		qhVector3 V1 = Edge1->Origin->Position;

		do
			{
			qhVector3 V2 = Edge2->Origin->Position;
			qhVector3 V3 = Edge3->Origin->Position;

			// Signed volume of this tetrahedron
			qhReal Det = qhDet( V1, V2, V3 );

			// Contribution to mass
			Volume += Det;

			// Contribution to centroid
			qhVector3 V4 = V1 + V2 + V3;
			Center += Det * V4;

			// Contribution to inertia monomials
			XX += Det * ( V1.X * V1.X + V2.X * V2.X + V3.X * V3.X + V4.X * V4.X );
			YY += Det * ( V1.Y * V1.Y + V2.Y * V2.Y + V3.Y * V3.Y + V4.Y * V4.Y );
			ZZ += Det * ( V1.Z * V1.Z + V2.Z * V2.Z + V3.Z * V3.Z + V4.Z * V4.Z );
			XY += Det * ( V1.X * V1.Y + V2.X * V2.Y + V3.X * V3.Y + V4.X * V4.Y );
			XZ += Det * ( V1.X * V1.Z + V2.X * V2.Z + V3.X * V3.Z + V4.X * V4.Z );
			YZ += Det * ( V1.Y * V1.Z + V2.Y * V2.Z + V3.Y * V3.Z + V4.Y * V4.Z );

			Edge2 = Edge3;
			Edge3 = Edge3->Next;
			}
		while ( Edge1 != Edge3 );
		}
	QH_ASSERT( Volume > 0 );

	// Fetch result
	qhMatrix3 Inertia;
	Inertia.C1.X = YY + ZZ;  Inertia.C2.X =     -XY;  Inertia.C3.X =     -XZ;
	Inertia.C1.Y =     -XY;  Inertia.C2.Y = XX + ZZ;  Inertia.C3.Y =     -YZ;
	Inertia.C1.Z =     -XZ;  Inertia.C2.Z =     -YZ;  Inertia.C3.Z = XX + YY;

	qhMassProperties Out;
	Out.Mass = Density * Volume / qhReal( 6 );
	Out.Center = Center / ( qhReal( 4 ) * Volume );
	Out.Inertia = ( Density / qhReal( 120 ) ) * Inertia - qhSteiner( Out.Mass, Out.Center );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::ShiftHull( const qhVector3& Translation )
	{
	// Transform vertices
	for ( qhVertex* Vertex = mVertexList.Begin(); Vertex != mVertexList.End(); Vertex = Vertex->Next )
		{
		Vertex->Position += Translation;
		}

	// Transform planes
	for ( qhFace* Face = mFaceList.Begin(); Face != mFaceList.End(); Face = Face->Next )
		{
		Face->Plane.Translate( Translation );
		}

	// Shift interior point
	mInteriorPoint += Translation;
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::ScaleHull( qhReal Scale )
	{
	// Transform vertices
	for ( qhVertex* Vertex = mVertexList.Begin(); Vertex != mVertexList.End(); Vertex = Vertex->Next )
		{
		Vertex->Position *= Scale;
		}

	// Transform planes
	for ( qhFace* Face = mFaceList.Begin(); Face != mFaceList.End(); Face = Face->Next )
		{
		Face->Plane.Offset *= Scale;
		}

	// Shift interior point
	mInteriorPoint *= Scale;
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::AllocateMemory( int VertexCount )
	{
	// Compute maximum feature counts 
	int MaxVertexCount = VertexCount;
	int MaxEdgeCount = 3 * VertexCount - 6;
	int MaxFaceCount = 2 * VertexCount - 4;

	// During construction we can temporarily allocate additional features. 
	// Also remember we are actually allocating half-edges where H = 2 * E.
	mVertexPool.Resize( 2 * MaxVertexCount );
	mEdgePool.Resize( 4 * MaxEdgeCount );
	mFacePool.Resize( 2 * MaxFaceCount );
	}


//--------------------------------------------------------------------------------------------------
qhVertex* qhConvex::CreateVertex( const qhVector3& Position, int Index )
	{
// 	qhVertex* Vertex = (qhVertex*)qhAlloc( sizeof( qhVertex ), __alignof( qhVertex ) );
// 	new ( Vertex ) qhVertex;

	qhVertex* Vertex = mVertexPool.Allocate();
	new ( Vertex ) qhVertex;

	Vertex->Prev = nullptr;
	Vertex->Next = nullptr;
	Vertex->Mark = QH_MARK_CONFIRM;
	Vertex->Position = Position;
	Vertex->ConflictFace = nullptr;
	Vertex->UserIndex = Index;
	
	return Vertex;
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::DestroyVertex( qhVertex* Vertex )
	{
	QH_ASSERT( !qhInList( Vertex ) );

// 	Vertex->~qhVertex();
// 	qhFree( Vertex );

	Vertex->~qhVertex();
	mVertexPool.Free( Vertex );
	}


//--------------------------------------------------------------------------------------------------
qhFace* qhConvex::CreateFace( qhVertex* Vertex1, qhVertex* Vertex2, qhVertex* Vertex3 )
	{
// 	qhFace* Face = (qhFace*)qhAlloc( sizeof( qhFace ), __alignof( qhFace ) );
// 	new ( Face ) qhFace;
// 
// 	qhHalfEdge* Edge1 = (qhHalfEdge*)qhAlloc( sizeof( qhHalfEdge ), __alignof( qhHalfEdge ) );
// 	qhHalfEdge* Edge2 = (qhHalfEdge*)qhAlloc( sizeof( qhHalfEdge ), __alignof( qhHalfEdge ) );
// 	qhHalfEdge* Edge3 = (qhHalfEdge*)qhAlloc( sizeof( qhHalfEdge ), __alignof( qhHalfEdge ) );

	qhFace* Face = mFacePool.Allocate();
	new ( Face ) qhFace;

	qhHalfEdge* Edge1 = mEdgePool.Allocate();
	qhHalfEdge* Edge2 = mEdgePool.Allocate();
	qhHalfEdge* Edge3 = mEdgePool.Allocate();

	qhPlane Plane = qhPlane( Vertex1->Position, Vertex2->Position, Vertex3->Position );
	qhReal Area = qhLength( Plane.Normal ) / qhReal( 2 );
	Plane.Normalize();

	// Initialize face
	Face->Prev = nullptr;
	Face->Next = nullptr;

	Face->Edge = Edge1;

	Face->Mark = QH_MARK_VISIBLE;
	Face->Area = Area;
	Face->Centroid = ( Vertex1->Position + Vertex2->Position + Vertex3->Position ) / qhReal( 3 ); 
	Face->Plane = Plane;
	Face->Flipped = Plane.Distance( mInteriorPoint ) > 0;

	// Initialize edges
	Edge1->Prev = Edge3;
	Edge1->Next = Edge2;
	Edge1->Origin = Vertex1;
	Edge1->Face = Face;
	Edge1->Twin = nullptr;

	Edge2->Prev = Edge1;
	Edge2->Next = Edge3;
	Edge2->Origin = Vertex2;
	Edge2->Face = Face;
	Edge2->Twin = nullptr;

	Edge3->Prev = Edge2;
	Edge3->Next = Edge1;
	Edge3->Origin = Vertex3;
	Edge3->Face = Face;
	Edge3->Twin = nullptr;

	return Face;
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::DestroyFace( qhFace* Face )
	{
	QH_ASSERT( !qhInList( Face ) );
	
	// Edge can be null if face was merged
	qhHalfEdge* Edge = Face->Edge;
	if ( Edge != nullptr )
		{
		do 
			{
			qhHalfEdge* Nuke = Edge;
			Edge = Edge->Next;

			// qhFree( Nuke );
			mEdgePool.Free( Nuke );
			} 
		while ( Edge != Face->Edge );
		}

// 	Face->~qhFace();
// 	qhFree( Face );

	Face->~qhFace();
	mFacePool.Free( Face );
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::ComputeTolerance( int VertexCount, const qhVector3* VertexBase )
{
	// Compute the tolerance relative to the object size 
	qhBounds3 Bounds = qhBuildBounds( VertexCount, VertexBase );
	qhVector3 Max = qhMax( qhAbs( Bounds.Min ), qhAbs( Bounds.Max ) );

	qhReal MaxSum = Max.X + Max.Y + Max.Z;
	qhReal MaxCoord = qhMax( Max.X, qhMax( Max.Y, Max.Z ) );
	qhReal MaxDistance = qhMin( QH_SQRT3 * MaxCoord, MaxSum );

	qhReal Tolerance = ( qhReal( 3 ) * MaxDistance * qhReal( 1.01 ) + MaxCoord ) * QH_REAL_EPSILON;

	mTolerance = Tolerance;
	mMinRadius = qhReal( 2 ) * mTolerance;
	mMinOutside = qhReal( 2 ) * mMinRadius;
	QH_ASSERT( mMinRadius < mMinOutside + 3 * QH_REAL_EPSILON );
}


//--------------------------------------------------------------------------------------------------
bool qhConvex::BuildInitialHull( int VertexCount, const qhVector3* VertexBase )
	{
	int Index1, Index2;
	qhFindFarthestPointsAlongCardinalAxes( Index1, Index2, mTolerance, VertexCount, VertexBase );
	if ( Index1 < 0 || Index2 < 0 )
		{
		return false;
		}

	int Index3 = qhFindFarthestPointFromLine( Index1, Index2, mTolerance, VertexCount, VertexBase );
	if ( Index3 < 0 )
		{
		return false;
		}

	int Index4 = qhFindFarthestPointFromPlane( Index1, Index2, Index3, mTolerance, VertexCount, VertexBase );
	if ( Index4 < 0 )
		{
		return false;
		}

	// Check winding order 
	qhVector3 V1 = VertexBase[ Index1 ] - VertexBase[ Index4 ];
	qhVector3 V2 = VertexBase[ Index2 ] - VertexBase[ Index4 ];
	qhVector3 V3 = VertexBase[ Index3 ] - VertexBase[ Index4 ];

	if ( qhDet( V1, V2, V3 ) < qhReal( 0 ) )
		{
		std::swap( Index2, Index3 );
		}

	// Compute an interior point to detect flipped faces
	mInteriorPoint = QH_VEC3_ZERO;
	mInteriorPoint += VertexBase[ Index1 ];
	mInteriorPoint += VertexBase[ Index2 ];
	mInteriorPoint += VertexBase[ Index3 ];
	mInteriorPoint += VertexBase[ Index4 ];
	mInteriorPoint /= qhReal( 4 );

	// Allocate initial vertices and save them in the vertex list
	qhVertex* Vertex1 = CreateVertex( VertexBase[ Index1 ], Index1 );
	mVertexList.PushBack( Vertex1 );
	qhVertex* Vertex2 = CreateVertex( VertexBase[ Index2 ], Index2 );
	mVertexList.PushBack( Vertex2 );
	qhVertex* Vertex3 = CreateVertex( VertexBase[ Index3 ], Index3 );
	mVertexList.PushBack( Vertex3 );
	qhVertex* Vertex4 = CreateVertex( VertexBase[ Index4 ], Index4 );
	mVertexList.PushBack( Vertex4 );

	// Allocate initial faces and save them in the face list
	qhFace* Face1 = CreateFace( Vertex1, Vertex2, Vertex3 );
	mFaceList.PushBack( Face1 );
	qhFace* Face2 = CreateFace( Vertex4, Vertex2, Vertex1 );
	mFaceList.PushBack( Face2 );
	qhFace* Face3 = CreateFace( Vertex4, Vertex3, Vertex2 );
	mFaceList.PushBack( Face3 );
	qhFace* Face4 = CreateFace( Vertex4, Vertex1, Vertex3 );
	mFaceList.PushBack( Face4 );

	// Link faces
	qhLinkFaces( Face1, 0, Face2, 1 );
	qhLinkFaces( Face1, 1, Face3, 1 );
	qhLinkFaces( Face1, 2, Face4, 1 );
	
	qhLinkFaces( Face2, 0, Face3, 2 );
	qhLinkFaces( Face3, 0, Face4, 2 );
	qhLinkFaces( Face4, 0, Face2, 2 );

	QH_ASSERT( qhCheckConsistency( Face1 ) );
	QH_ASSERT( qhCheckConsistency( Face2 ) );
	QH_ASSERT( qhCheckConsistency( Face3 ) );
	QH_ASSERT( qhCheckConsistency( Face4 ) );

	// Partition vertices into conflict lists
	for ( int Index = 0; Index < VertexCount; ++Index )
		{
		if ( Index == Index1 || Index == Index2 || Index == Index3 || Index == Index4 )
			{
			continue;
			}

		const qhVector3& Point = VertexBase[ Index ];

		qhReal MaxDistance = mMinOutside;
		qhFace* MaxFace = nullptr;

		for ( qhFace* Face = mFaceList.Begin(); Face != mFaceList.End(); Face = Face->Next )
			{
			qhReal Distance = Face->Plane.Distance( Point );
			if ( Distance > MaxDistance )
				{
				MaxDistance = Distance;
				MaxFace = Face;
				}
			}

		if ( MaxFace != nullptr )
			{
			qhVertex* Vertex = CreateVertex( Point, Index );

			Vertex->ConflictFace = MaxFace;
			MaxFace->ConflictList.PushBack( Vertex );
			}
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
qhVertex* qhConvex::NextConflictVertex()
	{
	// Choose the farthest point to avoid adding a lot of nearly coplanar vertices
	qhVertex* MaxVertex = nullptr;
	qhReal MaxDistance = mMinOutside;

	// DIRK_TODO: This can be optimized easily by storing the farthest vertex of a face at the end/beginning of the conflict list
	for ( qhFace* Face = mFaceList.Begin(); Face != mFaceList.End(); Face = Face->Next )
		{
		if ( !Face->ConflictList.Empty() )
			{
			for ( qhVertex* Vertex = Face->ConflictList.Begin(); Vertex != Face->ConflictList.End(); Vertex = Vertex->Next )
				{
				QH_ASSERT( Vertex->ConflictFace == Face );
				qhReal Distance = Face->Plane.Distance( Vertex->Position );
				
				if ( Distance > MaxDistance )
					{
					MaxDistance = Distance;
					MaxVertex = Vertex;
					}
				}
			}
		}

	return MaxVertex;
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::AddVertexToHull( qhVertex* Vertex )
	{
	// Remove vertex from conflict face
	qhFace* Face = Vertex->ConflictFace;
	Vertex->ConflictFace = nullptr;
	Face->ConflictList.Remove( Vertex );
	mVertexList.PushBack( Vertex );

	// Find the horizon edges
	qhArray< qhHalfEdge* > Horizon;
	BuildHorizon( Horizon, Vertex, Face );
	QH_ASSERT( Horizon.Size() >= 3 );

	// Create new cone faces
	qhArray< qhFace* > Cone;
	BuildCone( Cone, Horizon, Vertex );
	QH_ASSERT( Cone.Size() >= 3 );

#ifdef QH_DEBUG
	// Push iteration before merging faces
	AddIteration( Vertex, Horizon, Cone, mFaceList );
	int Iteration = mIterations.Size() - 1;
#endif

	// Merge coplanar faces
	MergeFaces( Cone );
	
	// Resolve orphaned vertices
	ResolveVertices( Cone );

	// Remove hidden faces and add new ones
	ResolveFaces( Cone );
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::CleanHull()
	{
	// Mark all vertices on the hull as visible
	for ( qhFace* Face = mFaceList.Begin(); Face != mFaceList.End(); Face = Face->Next )
		{
		qhHalfEdge* Edge = Face->Edge;

		do 
			{
			Edge->Origin->Mark = QH_MARK_VISIBLE;
			Edge = Edge->Next;
			}
		while ( Edge != Face->Edge );
		}

	// Remove unconfirmed vertices
	qhVertex* Vertex = mVertexList.Begin();
	while ( Vertex != mVertexList.End() )
		{
		qhVertex* Next = Vertex->Next;
		if ( Vertex->Mark != QH_MARK_VISIBLE )
			{
			mVertexList.Remove( Vertex );
			DestroyVertex( Vertex );
			}

		Vertex = Next;
		}
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::BuildHorizon( qhArray< qhHalfEdge* >& Horizon, qhVertex* Apex, qhFace* Seed, qhHalfEdge* Edge1 )
	{
	// Move vertices to orphaned list
	Seed->Mark = QH_MARK_DELETE;

	qhVertex* Vertex = Seed->ConflictList.Begin();
	while ( Vertex != Seed->ConflictList.End() )
		{
		qhVertex* Orphan = Vertex;
		Vertex = Vertex->Next;

		Orphan->ConflictFace = nullptr;
		Seed->ConflictList.Remove( Orphan );

		mOrphanedList.PushBack( Orphan );
		}
	QH_ASSERT( Seed->ConflictList.Empty() );

	qhHalfEdge* Edge;
	if ( Edge1 != nullptr )
		{
		Edge = Edge1->Next;
		}
	else
		{
		Edge1 = Seed->Edge;
		Edge = Edge1;
		}

	do 
		{
		qhHalfEdge* Twin = Edge->Twin;
		if ( Twin->Face->Mark == QH_MARK_VISIBLE )
			{
			qhReal Distance = Twin->Face->Plane.Distance( Apex->Position );
			if ( Distance > mMinRadius )
				{
				BuildHorizon( Horizon, Apex, Twin->Face, Twin );
				}
			else
				{
				Horizon.PushBack( Edge );
				}
			}
		
		Edge = Edge->Next;
		}
	while ( Edge != Edge1 );
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::BuildCone( qhArray< qhFace* >& Cone, const qhArray< qhHalfEdge* >& Horizon, qhVertex* Apex )
	{
	// Create cone faces and link bottom edges to horizon
	for ( int i = 0; i < Horizon.Size(); ++i )
		{
		qhHalfEdge* Edge = Horizon[ i ];
		QH_ASSERT( Edge->Twin->Twin == Edge );

		qhFace* Face = CreateFace( Apex, Edge->Origin, Edge->Twin->Origin );
		Cone.PushBack( Face );
		
		// Link face to bottom edge
		qhLinkFace( Face, 1, Edge->Twin );
		}

	// Link new cone faces with each other
	qhFace* Face1 = Cone.Back();
	for ( int i = 0; i < Cone.Size(); ++i )
		{
		qhFace* Face2 = Cone[ i ];
		qhLinkFaces( Face1, 2, Face2, 0 );
		Face1 = Face2;
		}
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::MergeFaces( const qhArray< qhFace* >& Cone )
	{
	// Merge flipped faces
	for ( int i = 0; i < Cone.Size(); ++i )
		{
		qhFace* Face = Cone[ i ];
		if ( Face->Mark == QH_MARK_VISIBLE )
			{
			if ( Face->Flipped )
				{
				Face->Flipped = false;

				qhReal BestArea = 0;
				qhHalfEdge* BestEdge = nullptr;

				qhHalfEdge* Edge = Face->Edge;

				do 
					{
					qhHalfEdge* Twin = Edge->Twin;

					qhReal Area = Twin->Face->Area;
					if ( Area > BestArea )
						{
						BestArea = Area;
						BestEdge = Edge; 
						}

					Edge = Edge->Next;
					} 
				while ( Edge != Face->Edge );

				QH_ASSERT( BestEdge != nullptr );
				ConnectFaces( BestEdge );
				}
			}
		}

	// Merge all concave
	for ( int i = 0; i < Cone.Size(); ++i )
		{
		qhFace* Face = Cone[ i ];
		if ( Face->Mark == QH_MARK_VISIBLE )
			{
			// Merge concave face
			while ( MergeConcave( Face ) ) {}
			}
		}

	// Merge all coplanar
	for ( int i = 0; i < Cone.Size(); ++i )
		{
		qhFace* Face = Cone[ i ];
		if ( Face->Mark == QH_MARK_VISIBLE )
			{
			// Merge coplanar faces
			while ( MergeCoplanar( Face ) ) {}
			}
		}
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::ResolveVertices( const qhArray< qhFace* >& Cone )
	{
	// Resolve orphaned vertices
	qhVertex* Vertex = mOrphanedList.Begin();
	while ( Vertex != mOrphanedList.End() )
		{
		qhVertex* Next = Vertex->Next;
		mOrphanedList.Remove( Vertex );
	
		qhReal MaxDistance = mMinOutside;
		qhFace* MaxFace = nullptr;
	
		for ( int i = 0; i < Cone.Size(); ++i )
			{
			// Skip faces that got merged
			if ( Cone[ i ]->Mark == QH_MARK_VISIBLE )
				{
				qhReal Distance = Cone[ i ]->Plane.Distance( Vertex->Position );
				if ( Distance > MaxDistance )
					{
					MaxDistance = Distance;
					MaxFace = Cone[ i ];
					}
				}
			}
	
		if ( MaxFace != nullptr )
			{
			QH_ASSERT( MaxFace->Mark == QH_MARK_VISIBLE );
			MaxFace->ConflictList.PushBack( Vertex );
			Vertex->ConflictFace = MaxFace;
			}
		else
			{
			// Vertex has been already removed from the orphaned list 
			// and can be destroyed
			DestroyVertex( Vertex );
			Vertex = nullptr;
			}
	
		Vertex = Next;
		}

	QH_ASSERT( mOrphanedList.Empty() );
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::ResolveFaces( const qhArray< qhFace* >& Cone )
	{
	// Delete hidden faces
		{
			qhFace* Face = mFaceList.Begin();
			while (Face != mFaceList.End())
			{
				qhFace* Nuke = Face;
				Face = Face->Next;

				if (Nuke->Mark == QH_MARK_DELETE)
				{
					QH_ASSERT(Nuke->ConflictList.Empty());

					mFaceList.Remove(Nuke);
					DestroyFace(Nuke);
				}
			}
		}

	// Add new faces
		{
			for (int i = 0; i < Cone.Size(); ++i)
			{
				qhFace* Face = Cone[i];

				// Assert that all concave faces were merged!
				if (Face->Mark == QH_MARK_DELETE)
				{
					DestroyFace(Face);
					continue;
				}

				mFaceList.PushBack(Cone[i]);
			}
		}
	}


//--------------------------------------------------------------------------------------------------
bool qhConvex::MergeConcave( qhFace* Face )
	{
	qhHalfEdge* Edge = Face->Edge;

	do 
		{
		qhHalfEdge* Twin = Edge->Twin;

		if ( Edge->IsConcave( mMinRadius ) || Twin->IsConcave( mMinRadius ) )
			{
			// Merge 
			ConnectFaces( Edge );
			return true;
			}

		Edge = Edge->Next;
		} 
	while ( Edge != Face->Edge );

	return false;
	}


//--------------------------------------------------------------------------------------------------
bool qhConvex::MergeCoplanar( qhFace* Face )
	{
	qhHalfEdge* Edge = Face->Edge;

	do 
		{
		qhHalfEdge* Twin = Edge->Twin;

		if ( !Edge->IsConvex( mMinRadius ) || !Twin->IsConvex( mMinRadius ) )
			{	
			ConnectFaces( Edge );
			return true;
			}

		Edge = Edge->Next;
		} 
	while ( Edge != Face->Edge );

	return false;
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::ConnectFaces( qhHalfEdge* Edge )
	{
	// The absorbing face
	qhFace* Face = Edge->Face;
	QH_ASSERT( qhCheckConsistency( Face ) );

	// Find the strip of shared edges
	qhHalfEdge* Twin = Edge->Twin;
	QH_ASSERT( qhCheckConsistency( Twin->Face ) );

	qhHalfEdge* EdgePrev = Edge->Prev;
	qhHalfEdge* EdgeNext = Edge->Next;
	qhHalfEdge* TwinPrev = Twin->Prev;
	qhHalfEdge* TwinNext = Twin->Next;
	
	while ( EdgePrev->Twin->Face == Twin->Face )
		{
		QH_ASSERT( EdgePrev->Twin == TwinNext );
		QH_ASSERT( TwinNext->Twin == EdgePrev );
		
		EdgePrev = EdgePrev->Prev;
		TwinNext = TwinNext->Next;
		}
	QH_ASSERT( EdgePrev->Face != TwinNext->Face );

	while ( EdgeNext->Twin->Face == Twin->Face )
		{
		QH_ASSERT( EdgeNext->Twin == TwinPrev );
		QH_ASSERT( TwinPrev->Twin == EdgeNext );

		EdgeNext = EdgeNext->Next;
		TwinPrev = TwinPrev->Prev;
		}
	QH_ASSERT( EdgeNext->Face != TwinPrev->Face );

	// Make sure we don't reference a shared edge
	Face->Edge = EdgePrev;
		
	// Discard opposing face and absorb non-shared edges
	qhArray< qhFace* > MergedFaces;
	MergedFaces.PushBack( Twin->Face );
	Twin->Face->Mark = QH_MARK_DELETE;
	Twin->Face->Edge = nullptr;

	for ( qhHalfEdge* Absorbed = TwinNext; Absorbed != TwinPrev->Next; Absorbed = Absorbed->Next )
		{
		Absorbed->Face = Face;
		}

	// Delete shared edges (before connection)
	DestroyEdges( EdgePrev->Next, EdgeNext );
	DestroyEdges( TwinPrev->Next, TwinNext );

	// Connect half edges (this can have side effects)
	ConnectEdges( EdgePrev, TwinNext, MergedFaces );
	ConnectEdges( TwinPrev, EdgeNext, MergedFaces );

	// Rebuild geometry for the merges face
	qhNewellPlane( Face );
	QH_ASSERT( qhCheckConsistency( Face ) );

	// Absorb conflict vertices
	AbsorbFaces( Face, MergedFaces );	
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::ConnectEdges( qhHalfEdge* Prev, qhHalfEdge* Next, qhArray< qhFace* >& MergedFaces )
	{
	QH_ASSERT( Prev != Next );
	QH_ASSERT( Prev->Face == Next->Face );
	
	// Check for redundant edges (this has side effects)
	// If this condition holds true both faces are in the same 
	// plane since they share three vertices.
	if ( Prev->Twin->Face == Next->Twin->Face )
		{
		// Next is redundant and will be removed. 
		// It should not be referenced by its associated face!
		if ( Next->Face->Edge == Next )
			{
			Next->Face->Edge = Prev;
			}

		qhHalfEdge* Twin;
		if ( qhVertexCount( Prev->Twin->Face ) == 3 )
			{
			Twin = Next->Twin->Prev->Twin;
			QH_ASSERT( Twin->Face->Mark != QH_MARK_DELETE );

			// If the opposing face is a triangle. We will    
			// get rid of it *and* its associated edges 
			// (Don't set OpposingFace->Edge = nullptr!)
			qhFace* OpposingFace = Prev->Twin->Face;
			OpposingFace->Mark = QH_MARK_DELETE;
			MergedFaces.PushBack( OpposingFace );
			}
		else
			{
			Twin = Next->Twin;
			
			// Prev->Twin is redundant and will be removed.
			// It should not be referenced by its associated face!
			if ( Twin->Face->Edge == Prev->Twin )
				{
				Twin->Face->Edge = Twin;
				}

			Twin->Next = Prev->Twin->Next;
			Twin->Next->Prev = Twin;  

			//qhFree( Prev->Twin );
			mEdgePool.Free( Prev->Twin );
			}
		
		Prev->Next = Next->Next;
		Prev->Next->Prev = Prev;

		Prev->Twin = Twin;
		Twin->Twin = Prev;

		// Destroy the redundant edge and its associated vertex
		mVertexList.Remove( Next->Origin );
		DestroyVertex( Next->Origin );

		//qhFree( Next );
		mEdgePool.Free( Next );

		// Twin->Face was modified, so recompute its plane
		qhNewellPlane( Twin->Face );	
		}
	else
		{
		Prev->Next = Next;
		Next->Prev = Prev;
		}
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::DestroyEdges( qhHalfEdge* Begin, qhHalfEdge* End )
	{
	qhHalfEdge* Edge = Begin;
	while ( Edge != End )
		{
		qhHalfEdge* Nuke = Edge;
		Edge = Edge->Next;

		// Delete vertex if there is more than one shared edge
		// DIRK_TODO: Since we run over the twin edges as well this would delete the vertex twice!
// 		if ( Nuke != Begin )
// 			{
// 			mVertexList.Remove( Nuke->Origin );
// 			DestroyVertex( Nuke->Origin );
// 			}
		
		// qhFree( Nuke );
		mEdgePool.Free( Nuke );
		}
	}


//--------------------------------------------------------------------------------------------------
void qhConvex::AbsorbFaces( qhFace* Face, qhArray< qhFace* >& MergedFaces )
	{
	for ( int i = 0; i < MergedFaces.Size(); ++i )
		{
		QH_ASSERT( MergedFaces[ i ]->Mark == QH_MARK_DELETE );
		qhList< qhVertex >& ConflictList = MergedFaces[ i ]->ConflictList;

		qhVertex* Vertex = ConflictList.Begin();
		while ( Vertex != ConflictList.End() )
			{
			qhVertex* Next = Vertex->Next;
			ConflictList.Remove( Vertex );

			if ( Face->Plane.Distance( Vertex->Position ) > mMinOutside )
				{
				Face->ConflictList.PushBack( Vertex );
				Vertex->ConflictFace = Face;
				}
			else
				{
				mOrphanedList.PushBack( Vertex );
				}

			Vertex = Next;
			}

		QH_ASSERT( ConflictList.Empty() );
		}
	}



