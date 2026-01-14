//--------------------------------------------------------------------------------------------------
// hull.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "hull.h"

// QHull
#include <qhConvex.h>


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static void rkSortEdges( qhArray< const qhHalfEdge* >& edges )
	{
	for ( int EdgeIndex = 0; EdgeIndex < edges.Size(); EdgeIndex += 2 )
		{
		const qhHalfEdge* Edge = edges[ EdgeIndex ];
		for ( int TwinIndex = EdgeIndex + 1; TwinIndex < edges.Size(); TwinIndex += 1 )
			{
			const qhHalfEdge* Twin = edges[ TwinIndex ];
			if ( Edge->Twin == Twin )
				{
				RK_ASSERT( Twin->Twin == Edge );
				std::swap( edges[ EdgeIndex + 1 ], edges[ TwinIndex ] );
				}
			}
		}
	}


//--------------------------------------------------------------------------------------------------
static bool rkValidateEdges( const qhArray< const qhHalfEdge* >& Edges )
	{
	for ( int i = 0; i < Edges.Size(); i += 2 )
		{
		const qhHalfEdge* Edge = Edges[ i ];
		const qhHalfEdge* Twin = Edges[ i + 1 ];

		if ( Edge->Twin != Twin || Twin->Twin != Edge )
			{
			return false;
			}
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
static inline void rkStore( RkVector3& Dst, const qhVector3& Src )
	{
	Dst.X = static_cast< float >( Src.X );
	Dst.Y = static_cast< float >( Src.Y );
	Dst.Z = static_cast< float >( Src.Z );
	}


//--------------------------------------------------------------------------------------------------
static inline RkVector3 rkAsVector3( const qhVector3& V )
	{
	RkVector3 Out;
	rkStore( Out, V );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
static inline void rkStore( RkMatrix3& Dst, const qhMatrix3& Src )
	{
	Dst.C1 = rkAsVector3( Src.C1 );
	Dst.C2 = rkAsVector3( Src.C2 );
	Dst.C3 = rkAsVector3( Src.C3 );
	}


//--------------------------------------------------------------------------------------------------
static inline RkMatrix3 rkAsMatrix3( const qhMatrix3& M )
	{
	RkMatrix3 Out;
	rkStore( Out, M );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
static inline void rkStore( RkPlane3& Dst, const qhPlane& Src )
	{
	Dst.Normal = rkAsVector3( Src.Normal );
	Dst.Offset = static_cast< float >( Src.Offset );
	}


//--------------------------------------------------------------------------------------------------
static inline RkPlane3 rkAsPlane( const qhPlane& P )
	{
	RkPlane3 Out;
	rkStore( Out, P );
	
	return Out;
	}


//--------------------------------------------------------------------------------------------------
static inline void rkStore( RkBounds3& Dst, const qhBounds3& Src )
	{
	Dst.Min = rkAsVector3( Src.Min );
	Dst.Max = rkAsVector3( Src.Max );
	}


//--------------------------------------------------------------------------------------------------
static inline RkBounds3 rkAsBounds3( const qhBounds3& Bounds )
	{
	RkBounds3 Out;
	rkStore( Out, Bounds );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
static inline RkHalfEdge rkMakeEdge( int Next, int Twin, int Origin, int Face )
	{
	RkHalfEdge Edge;
	Edge.Next = uint8( Next );
	Edge.Twin = uint8( Twin );
	Edge.Origin = uint8( Origin );
	Edge.Face = uint8( Face );

	return Edge;
	}


//--------------------------------------------------------------------------------------------------
// RkHull
//--------------------------------------------------------------------------------------------------
int RkHull::FindSupportFace( const RkVector3& Direction ) const
	{
	int BestIndex = -1;
	float BestDot = -RK_F32_MAX;

	for ( int Index = 0; Index < FaceCount; ++Index )
		{
		float Dot = rkDot( PlaneList[ Index ].Normal, Direction );
		if ( Dot > BestDot )
			{
			BestDot = Dot;
			BestIndex = Index;
			}
		}
	RK_ASSERT( BestIndex >= 0 );

	return BestIndex;
	}


//--------------------------------------------------------------------------------------------------
int RkHull::FindSupportVertex( const RkVector3& Direction ) const
	{
	int BestIndex = -1;
	float BestDot = -RK_F32_MAX;

	for ( int Index = 0; Index < VertexCount; ++Index )
		{
		float Dot = rkDot( Direction, PositionList[ Index ] );
		if ( Dot > BestDot )
			{
			BestIndex = Index;
			BestDot = Dot;
			}
		}
	RK_ASSERT( BestIndex >= 0 );

	return BestIndex;
	}


//--------------------------------------------------------------------------------------------------
size_t RkHull::GetSize() const
	{
	size_t ByteCount = sizeof( RkHull );
	ByteCount = rkAlign( ByteCount, alignof( RkVertex ) );
	ByteCount += VertexCount * sizeof( RkVertex );
	ByteCount = rkAlign( ByteCount, alignof( RkVector3 ) );
	ByteCount += VertexCount * sizeof( RkVector3 );
	ByteCount = rkAlign( ByteCount, alignof( RkHalfEdge ) );
	ByteCount += EdgeCount * sizeof( RkHalfEdge );
	ByteCount = rkAlign( ByteCount, alignof( RkFace )  );
	ByteCount += FaceCount * sizeof( RkFace );
	ByteCount = rkAlign( ByteCount, alignof( RkPlane3 )  );
	ByteCount += FaceCount * sizeof( RkPlane3 );
	ByteCount = rkAlign( ByteCount, 16 );
	RK_ASSERT( ( ByteCount & 15 ) == 0 );

	return ByteCount;
	}


//--------------------------------------------------------------------------------------------------
bool RkHull::IsConsistent() const
	{
	// Alignment tests
	if ( !VertexList.IsAligned( alignof( RkVertex ) ) )
		{
		return false;
		}

	if ( !PositionList.IsAligned( alignof( RkVector3 ) ) )
		{
		return false;
		}

	if ( !EdgeList.IsAligned( alignof( RkHalfEdge ) ) )
		{
		return false;
		}

	if ( !FaceList.IsAligned( alignof( RkFace ) ) )
		{
		return false;
		}

	if ( !PlaneList.IsAligned( alignof( RkPlane3 ) ) )
		{
		return false;
		}

	// // Euler's identity for convex polyhedron
	int V = VertexCount;
	int E = EdgeCount / 2;
	int F = FaceCount;

	if ( V - E + F != 2 )
		{
		return false;
		}

	// Vertex invariants
	for ( int Index = 0; Index < VertexCount; ++Index )
		{
		const RkVertex* Vertex = GetVertex( Index );
		const RkHalfEdge* Edge = GetEdge( Vertex->Edge );

		// Connectivity tests
		if ( GetVertex( Edge->Origin ) != Vertex )
			{
			return false;
			}
		}

	// Edge invariants
	for ( int Index = 0; Index < EdgeCount; Index += 2 )
		{
		const RkHalfEdge* Edge = GetEdge( Index + 0 );
		const RkHalfEdge* Twin = GetEdge( Index + 1 );

		// Connectivity tests
		if ( GetEdge( Edge->Twin ) != Twin )
			{
			return false;
			}

		if ( GetEdge( Twin->Twin ) != Edge )
			{
			return false;
			}
		}

	// Face invariants
	for ( int Index = 0; Index < FaceCount; ++Index )
		{
		const RkFace* Face = GetFace( Index );
		const RkHalfEdge* Edge = GetEdge( Face->Edge );

		// Geometry tests
		RkPlane3 Plane = GetPlane( Index );
		if ( rkDistance( Plane, Center ) >= 0.0f )
			{
			return false;
			}

		do 
			{
			// Connectivity tests
			const RkHalfEdge* Next = GetEdge( Edge->Next );
			const RkHalfEdge* Twin = GetEdge( Edge->Twin );

			if ( GetFace( Edge->Face ) != Face )
				{
				return false;
				}

			if ( GetEdge( Twin->Twin ) != Edge )
				{
				return false;
				}

			if ( Next->Origin != Twin->Origin )
				{
				return false;
				}

			Edge = Next;
			} 
		while ( Edge != GetEdge( Face->Edge ) );
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
// RkHull utilities
//--------------------------------------------------------------------------------------------------
RkHull* rkCreateBox( const RkVector3& Extent )
	{
	int VertexCount = 8;
	RkVector3 VertexBase[] = 
		{
		RkVector3(  Extent.X,  Extent.Y,  Extent.Z ),
		RkVector3( -Extent.X,  Extent.Y,  Extent.Z ),
		RkVector3( -Extent.X, -Extent.Y,  Extent.Z ),
		RkVector3(  Extent.X, -Extent.Y,  Extent.Z ),
		RkVector3(  Extent.X,  Extent.Y, -Extent.Z ),
		RkVector3( -Extent.X,  Extent.Y, -Extent.Z ),
		RkVector3( -Extent.X, -Extent.Y, -Extent.Z ),
		RkVector3(  Extent.X, -Extent.Y, -Extent.Z )
		};

	RkHull* Hull = rkCreateHull( VertexCount, VertexBase );
	RK_ASSERT( Hull->VertexCount == 8 );
	RK_ASSERT( Hull->EdgeCount == 24 );
	RK_ASSERT( Hull->FaceCount == 6 );

	return Hull;
	}


//--------------------------------------------------------------------------------------------------
RkHull* rkCreateBox( const RkVector3& Center, const RkVector3& Extent )
	{
	int VertexCount = 8;
	RkVector3 VertexBase[] = 
		{
		Center + RkVector3(  Extent.X,  Extent.Y,  Extent.Z ),
		Center + RkVector3( -Extent.X,  Extent.Y,  Extent.Z ),
		Center + RkVector3( -Extent.X, -Extent.Y,  Extent.Z ),
		Center + RkVector3(  Extent.X, -Extent.Y,  Extent.Z ),
		Center + RkVector3(  Extent.X,  Extent.Y, -Extent.Z ),
		Center + RkVector3( -Extent.X,  Extent.Y, -Extent.Z ),
		Center + RkVector3( -Extent.X, -Extent.Y, -Extent.Z ),
		Center + RkVector3(  Extent.X, -Extent.Y, -Extent.Z )
		};

	RkHull* Hull = rkCreateHull( VertexCount, VertexBase );
	RK_ASSERT( Hull->VertexCount == 8 );
	RK_ASSERT( Hull->EdgeCount == 24 );
	RK_ASSERT( Hull->FaceCount == 6 );

	return Hull;
	}


//--------------------------------------------------------------------------------------------------
RkHull* rkCreateCylinder( float Height, float Radius, int Slices )
	{
	RK_ASSERT( Height > 0.0f );
	RK_ASSERT( Radius > 0.0f );
	RK_ASSERT( 4 <= Slices && Slices <= 32 );

	int VertexCount = 2 * Slices;
	RkVector3* VertexBase = (RkVector3*)alloca( VertexCount * sizeof( RkVector3 ) );
	RK_ASSERT( VertexBase );

	float Alpha = 0.0f;
	float DeltaAlpha = RK_2PI / Slices;

	for ( int Index = 0; Index < Slices; ++Index )
		{
		float SinAlpha = rkSin( Alpha );
		float CosAlpha = rkCos( Alpha );

		VertexBase[ 2*Index + 0 ] = RkVector3( Radius * CosAlpha, 0.0f, Radius * SinAlpha );
		VertexBase[ 2*Index + 1 ] = RkVector3( Radius * CosAlpha, Height, Radius * SinAlpha );

		Alpha += DeltaAlpha;
		}

	RkHull* Hull = rkCreateHull( VertexCount, VertexBase );
	RK_ASSERT( Hull->VertexCount == VertexCount );
	RK_ASSERT( Hull->EdgeCount == 6 * Slices );
	RK_ASSERT( Hull->FaceCount == Slices + 2 );

	return Hull;
	}


//--------------------------------------------------------------------------------------------------
RkHull* rkCreateCone( float Height, float Radius1, float Radius2, int Slices )
	{
	RK_ASSERT( Height > 0.0f );
	RK_ASSERT( Radius1 > 0.0f );
	RK_ASSERT( Radius2 > 0.0f );
	RK_ASSERT( 4 <= Slices && Slices <= 32 );

	int VertexCount = 2 * Slices;
	RkVector3* VertexBase = (RkVector3*)alloca( VertexCount * sizeof( RkVector3 ) );
	RK_ASSERT( VertexBase );

	float Alpha = 0.0f;
	float DeltaAlpha = RK_2PI / Slices;

	for ( int Index = 0; Index < Slices; ++Index )
		{
		float SinAlpha = rkSin( Alpha );
		float CosAlpha = rkCos( Alpha );

		VertexBase[ 2 * Index + 0 ] = RkVector3( Radius1 * CosAlpha, 0.0f, Radius1 * SinAlpha );
		VertexBase[ 2 * Index + 1 ] = RkVector3( Radius2 * CosAlpha, Height, Radius2 * SinAlpha );

		Alpha += DeltaAlpha;
		}

	RkHull* Hull = rkCreateHull( VertexCount, VertexBase );
	RK_ASSERT( Hull->VertexCount == VertexCount );
	RK_ASSERT( Hull->EdgeCount == 6 * Slices );
	RK_ASSERT( Hull->FaceCount == Slices + 2 );

	return Hull;
	}


//--------------------------------------------------------------------------------------------------
RkHull* rkCreateConvex( float Radius, int PointCount )
	{
	// Golden ratio
	const float Phi = ( 1.0f + rkSqrt( 5.0f ) ) / 2.0f;  

	// Random points on sphere (Fibonacci lattice)
	RkStackArray< RkVector3, 64 > Points( PointCount );
	for ( int PointIndex = 0; PointIndex < PointCount; ++PointIndex )
		{
		float Theta = RK_2PI * PointIndex / Phi;						// Azimuthal angle
		float Z = 1.0f - ( 2.0f * PointIndex + 1.0f ) / PointCount;		// Z coordinate
		float Radius_XY = rkSqrt( 1.0f - Z * Z );						// Radius in xy-plane

		Points[ PointIndex ].X = Radius * Radius_XY * rkCos( Theta );
		Points[ PointIndex ].Y = Radius * Radius_XY * rkSin( Theta );
		Points[ PointIndex ].Z = Radius * Z;
		}

	return rkCreateHull( Points.Size(), Points.Data() );
	}


//--------------------------------------------------------------------------------------------------
RkHull* rkCreateHull( int VertexCount, const RkVector3* VertexBase )
	{
	qhConvex Convex;
	Convex.Construct( VertexCount, reinterpret_cast< const qhReal32* >( VertexBase ) );
	if ( !Convex.IsConsistent() )
		{
		return nullptr;
		}

	// Copy vertices, edges and faces into array to transform pointers into indices
	qhArray< const qhVertex* > Vertices;
	Vertices.Reserve( VertexCount );
	
	const qhList< qhVertex >& VertexList = Convex.GetVertexList();
	for ( const qhVertex* Vertex = VertexList.Begin(); Vertex != VertexList.End(); Vertex = Vertex->Next )
		{
		Vertices.PushBack( Vertex );
		}
	
	if ( Vertices.Size() > 256 )
		{
		return nullptr;
		}

	qhArray< const qhFace* > Faces;
	qhArray< const qhHalfEdge* > Edges;

	const qhList< qhFace >& FaceList = Convex.GetFaceList();
	for ( const qhFace* Face = FaceList.Begin(); Face != FaceList.End(); Face = Face->Next )
		{
		Faces.PushBack( Face );

		const qhHalfEdge* Edge = Face->Edge;

		do 
			{
			Edges.PushBack( Edge );
			Edge = Edge->Next;
			} 
		while ( Edge != Face->Edge );
		}

	if ( Edges.Size() > 256 || Faces.Size() > 256 )
		{
		return nullptr;
		}

	rkSortEdges( Edges );
	RK_ASSERT( rkValidateEdges( Edges ) );

	// Create hull resource
	qhBounds3 Bounds = Convex.ComputeBounds();
	qhMassProperties MassProperties = Convex.ComputeMass( 1.0f );

	// Allocate the hull
	size_t ByteCount = sizeof( RkHull );
	size_t VertexOffset = ByteCount = rkAlign( ByteCount, alignof( RkVertex ) );
	ByteCount += Vertices.Size() * sizeof( RkVertex );
	size_t PositionOffset = ByteCount = rkAlign( ByteCount, alignof( RkVector3 ) );
	ByteCount += Vertices.Size() * sizeof( RkVector3 );
	size_t EdgeOffset = ByteCount = rkAlign( ByteCount, alignof( RkHalfEdge ) );
	ByteCount += Edges.Size() * sizeof( RkHalfEdge );
	size_t FaceOffset = ByteCount = rkAlign( ByteCount, alignof( RkFace ) );
	ByteCount += Faces.Size() * sizeof( RkFace );
	size_t PlaneOffset = ByteCount = rkAlign( ByteCount, alignof( RkPlane3 ) );
	ByteCount += Faces.Size() * sizeof( RkPlane3 );
	ByteCount = rkAlign( ByteCount, 16 );
	RK_ASSERT( ( ByteCount & 15 ) == 0 );

	RkHull* Hull = (RkHull*)rkAlignedAlloc( ByteCount, 16 );
	Hull->VertexList = (RkVertex*)rkAddByteOffset( Hull, VertexOffset );
	Hull->PositionList = (RkVector3*)rkAddByteOffset( Hull, PositionOffset );
	Hull->EdgeList = (RkHalfEdge*)rkAddByteOffset( Hull, EdgeOffset );
	Hull->FaceList = (RkFace*)rkAddByteOffset( Hull, FaceOffset );
	Hull->PlaneList = (RkPlane3*)rkAddByteOffset( Hull, PlaneOffset );

	// Fill hull
	Hull->Bounds = rkAsBounds3( Bounds );

	Hull->Mass = MassProperties.Mass ;
	Hull->Center = rkAsVector3( MassProperties.Center );
	Hull->Inertia = rkAsMatrix3( MassProperties.Inertia );

	Hull->VertexCount = Vertices.Size();
	for ( int Index = 0; Index < Vertices.Size(); ++Index )
		{
		Hull->VertexList[ Index ].Edge = 0;

		const qhVertex* Vertex = Vertices[ Index ];
		rkStore( Hull->PositionList[ Index ], Vertex->Position );
		}

	Hull->EdgeCount = Edges.Size();
	for ( int Index = 0; Index < Edges.Size(); ++Index )
		{
		const qhHalfEdge* Edge = Edges[ Index ];
		
		int Next = Edges.IndexOf( Edge->Next );
		RK_ASSERT( 0 <= Next && Next <= RK_U8_MAX );
		Hull->EdgeList[ Index ].Next = uint8( Next  );
		
		int Twin = Edges.IndexOf( Edge->Twin );
		RK_ASSERT( 0 <= Twin && Twin <= RK_U8_MAX );
		Hull->EdgeList[ Index ].Twin = uint8( Twin );
		
		int Face = Faces.IndexOf( Edge->Face );
		RK_ASSERT( 0 <= Face && Face <= RK_U8_MAX );
		Hull->EdgeList[ Index ].Face = uint8( Face );

		int Origin = Vertices.IndexOf( Edge->Origin );
		RK_ASSERT( 0 <= Origin && Origin <= RK_U8_MAX );
		Hull->EdgeList[ Index ].Origin = uint8( Origin );

		Hull->VertexList[ Origin ].Edge = uint8( Index );
		}

	Hull->FaceCount = Faces.Size();
	for ( int Index = 0; Index < Faces.Size(); ++Index )
		{
		const qhFace* Face = Faces[ Index ];

		int Edge = Edges.IndexOf( Face->Edge );
		RK_ASSERT( 0 <= Edge && Edge <= RK_U8_MAX );
		Hull->FaceList[ Index ].Edge = uint8( Edge );
		rkStore( Hull->PlaneList[ Index ], Face->Plane );
		}

	RK_ASSERT( Hull->IsConsistent() && Hull->GetSize() == ByteCount );
	return Hull;
	}


//--------------------------------------------------------------------------------------------------
RkHull* rkCloneHull( const RkHull* Hull )
	{
	if ( !Hull )
		{
		return nullptr;
		}

	RkHull* Clone = static_cast< RkHull* >( rkAlignedAlloc( Hull->GetSize(), 16 ) );
	rkMemCpy( Clone, Hull, Hull->GetSize() );

	return Clone;
	}


//--------------------------------------------------------------------------------------------------
void rkDestroyHull( RkHull*& Hull )
	{
	if ( Hull )
		{
		rkAlignedFree( Hull );
		Hull = nullptr;
		}	
	}


//--------------------------------------------------------------------------------------------------
// RkTriangleHull
//--------------------------------------------------------------------------------------------------
RkTriangleHull::RkTriangleHull( const RkVector3& A, const RkVector3& B, const RkVector3& C )
	{
	// Bounding volume 
	RkVector3 Min = rkMin( A, rkMin( B, C ) );
	RkVector3 Max = rkMax( A, rkMax( B, C ) );
	Bounds = RkBounds3( Min, Max );
		
	// Mass properties
	Mass = 0.0f;
	Center = ( A + B + C ) / 3.0f;
	Inertia = RK_MAT3_ZERO;
		
	// Vertices
	VertexCount = 3;
	VertexList = TriangleVertices;

	TriangleVertices[ 0 ].Edge = 0;
	TriangleVertices[ 1 ].Edge = 2;
	TriangleVertices[ 2 ].Edge = 4;

	PositionList = TrianglePositions;

	TrianglePositions[ 0 ] = A;
	TrianglePositions[ 1 ] = B;
	TrianglePositions[ 2 ] = C;

	// Edges (remember that each edge *must* be followed by its twin!)
	EdgeCount = 6;
	EdgeList = TriangleEdges;

	TriangleEdges[ 0 ] = rkMakeEdge( 2, 1, 0, 0 ); // Face 0 - Edge 0
	TriangleEdges[ 2 ] = rkMakeEdge( 4, 3, 1, 0 ); // Face 0 - Edge 1
	TriangleEdges[ 4 ] = rkMakeEdge( 0, 5, 2, 0 ); // Face 0 - Edge 2

	TriangleEdges[ 1 ] = rkMakeEdge( 5, 0, 1, 1 ); // Face 1 - Edge 0
	TriangleEdges[ 3 ] = rkMakeEdge( 1, 2, 2, 1 ); // Face 1 - Edge 1
	TriangleEdges[ 5 ] = rkMakeEdge( 3, 4, 0, 1 ); // Face 1 - Edge 2

	// Faces
	FaceCount = 2;
	FaceList = TriangleFaces;

	TriangleFaces[ 0 ].Edge = 0;
	TriangleFaces[ 1 ].Edge = 1;

	// Planes
	PlaneList = TrianglePlanes;

	RkVector3 Normal = rkCross( B - A, C - A );
	Normal = rkNormalize( Normal );
	RK_ASSERT( rkAbs( rkLength( Normal ) - 1.0f ) < 100.0f * RK_F32_EPSILON );

	TrianglePlanes[ 0 ] = RkPlane3( Normal, Center );
	TrianglePlanes[ 1 ] = RkPlane3( -Normal, Center );
	}