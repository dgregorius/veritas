//--------------------------------------------------------------------------------------------------
// bvh.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "bvh.h"


//--------------------------------------------------------------------------------------------------
// Local constants
//--------------------------------------------------------------------------------------------------
#define RK_BUCKET_COUNT	16
#define RK_DESIRED_TRIANGLES_PER_LEAF 4
#define RK_MAXIMUM_TRIANGLES_PER_LEAF 8


//--------------------------------------------------------------------------------------------------
// RkBVHNode
//--------------------------------------------------------------------------------------------------
int RkBVHNode::GetHeight() const
	{
	if ( IsLeaf() )
		{
		return 0;
		}

	const RkBVHNode* pLeftChild = GetLeftChild();
	int LeftHeight = pLeftChild->GetHeight();
	const RkBVHNode* pRightChild = GetRightChild();
	int RightHeight = pRightChild->GetHeight();

	return 1 + rkMax( LeftHeight, RightHeight );
	}


//--------------------------------------------------------------------------------------------------
// RkBVH
//--------------------------------------------------------------------------------------------------
size_t RkBVH::GetSize() const
	{
	size_t ByteCount = sizeof( RkBVH );
	ByteCount = rkAlign( ByteCount, alignof( RkBVHNode ) );
	ByteCount += NodeCount * sizeof( RkBVHNode );
	ByteCount = rkAlign( ByteCount, alignof( RkBVHTriangle ) );
	ByteCount += TriangleCount * sizeof( RkBVHTriangle );
	ByteCount = rkAlign( ByteCount, alignof( RkVector3 ) );
	ByteCount += VertexCount * sizeof( RkVector3 );
	ByteCount = rkAlign( ByteCount, 16 );
	RK_ASSERT( ( ByteCount & 15 ) == 0 );

	return ByteCount;
	}


//--------------------------------------------------------------------------------------------------
bool RkBVH::IsConsistent() const
	{
	// Check alignment
	if ( !rkIsAligned( this, 16 ) )
		{
		return false;
		}

	if ( !NodeList.IsAligned( alignof( RkBVHNode ) ) )
		{
		return false;
		}

	if ( !TriangleList.IsAligned( alignof( RkBVHTriangle ) ) )
		{
		return false;
		}

	if ( !VertexList.IsAligned( alignof( RkVector3 ) ) )
		{
		return false;
		}

	// Check nodes
	int Count = 0;
	const RkBVHNode* Stack[ 512 ];
	Stack[ Count++ ] = GetRoot();

	while ( Count > 0 )
		{
		const RkBVHNode* Node = Stack[ --Count ];
		RkBounds3 NodeBounds = Node->GetBounds();

		if ( !Node->IsLeaf() )
			{
			const RkBVHNode* Child1 = Node->GetLeftChild();
			RkBounds3 Bounds1 = Child1->GetBounds();
			const RkBVHNode* Child2 = Node->GetRightChild();
			RkBounds3 Bounds2 = Child2->GetBounds();

			if ( !NodeBounds.Contains( Bounds1 ) )
				{
				return false;
				}

			if ( !NodeBounds.Contains( Bounds2 ) )
				{
				return false;
				}

			Stack[ Count++ ] = Child2;
			Stack[ Count++ ] = Child1;
			}	
		else
			{
			RkBounds3 TriangleBounds = RK_BOUNDS3_EMPTY;
			for ( uint32 Index = 0; Index < Node->TriangleCount; ++Index )
				{
				RkBVHTriangle Triangle = GetTriangle( Node->TriangleOffset + Index );
				
				RkBounds3 VertexBounds = RK_BOUNDS3_EMPTY;
				VertexBounds += GetVertex( Triangle.Index1 );
				VertexBounds += GetVertex( Triangle.Index2 );
				VertexBounds += GetVertex( Triangle.Index3 );
				
				TriangleBounds += VertexBounds;
				}

			if ( !NodeBounds.Contains( TriangleBounds ) )
				{
				return false;
				}
			}
		}	

	// Check triangles
	for ( int Index = 0; Index < TriangleCount; ++Index )
		{
		// Index range 
		RkBVHTriangle Triangle = TriangleList[ Index ];
		if ( Triangle.Index1 >= VertexCount )
			{
			return false;
			}

		if ( Triangle.Index2 >= VertexCount )
			{
			return false;
			}

		if ( Triangle.Index3 >= VertexCount )
			{
			return false;
			}

		// Degenerate topology
		if ( Triangle.Index1 == Triangle.Index2 )
			{
			return false;
			}
		if ( Triangle.Index1 == Triangle.Index3 )
			{
			return false;
			}
		if ( Triangle.Index2 == Triangle.Index3 )
			{
			return false;
			}

		// Degenerate geometry
		RkVector3 Vertex1 = VertexList[ Triangle.Index1 ];
		RkVector3 Vertex2 = VertexList[ Triangle.Index2 ];
		RkVector3 Vertex3 = VertexList[ Triangle.Index3 ];
		if ( Vertex1 == Vertex2 )
			{
			return false;
			}
		if ( Vertex1 == Vertex3 )
			{
			return false;
			}
		if ( Vertex2 == Vertex3 )
			{
			return false;
			}

		RkVector3 Normal = rkCross( Vertex2 - Vertex1, Vertex3 - Vertex1 );
		float LengthSq = rkLengthSq( Normal );

		if ( LengthSq < 1000.0f * RK_F32_MIN )
			{
			return false;
			}
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static inline void rkStore( float* Dst, const RkVector3& Src )
	{
	RK_ASSERT( Dst );
	Dst[ 0 ] = Src.X;
	Dst[ 1 ] = Src.Y;
	Dst[ 2 ] = Src.Z;
	}

//--------------------------------------------------------------------------------------------------
static inline void rkStoreInternal( RkBVHNode& Node, int Axis, const RkBounds3& Bounds, int ChildOffset )
	{
	Node.Axis = Axis;
	Node.ChildOffset = ChildOffset;
	rkStore( Node.BoundsMin, Bounds.Min );
	rkStore( Node.BoundsMax, Bounds.Max );
	}


//--------------------------------------------------------------------------------------------------
static inline void rkStoreLeaf( RkBVHNode& Node, const RkBounds3& Bounds, int TriangleCount, int TriangleOffset )
	{
	Node.Type = RkBVHNode::kLeaf;
	Node.TriangleCount = TriangleCount;
	Node.TriangleOffset = TriangleOffset;
	rkStore( Node.BoundsMin, Bounds.Min );
	rkStore( Node.BoundsMax, Bounds.Max );
	}


//--------------------------------------------------------------------------------------------------
struct RkPrimitve
	{
	int Index;
	RkBounds3 Bounds;
	};


//--------------------------------------------------------------------------------------------------
struct RkBucket
	{
	int Count;
	RkBounds3 Bounds;
	};


//--------------------------------------------------------------------------------------------------
struct RkSplit
	{
	int Axis;
	int Index;

	RkBounds3 LeftBounds;
	RkBounds3 RightBounds;

	RK_FORCEINLINE bool Succeeded() const { return Axis >= 0; }
	RK_FORCEINLINE bool Failed() const { return Axis < 0; }
	};


//--------------------------------------------------------------------------------------------------
class RkBVHBuilder
	{
	public:
		void Run( RkArray< RkBVHNode >& Nodes, int PrimitiveCount, RkPrimitve* PrimitiveList );

	private:
		// Splitting
		RkSplit SplitOptimal( int PrimitiveCount, RkPrimitve* PrimitiveList ) const;
		RkSplit SplitHalf( int PrimitiveCount, RkPrimitve* PrimitiveList ) const;
		bool ValidateSplit( int PrimitiveCount, RkPrimitve* PrimitiveList, const RkSplit& Split ) const;

		// Tree builder (recursive)
		int BuildTree( RkArray< RkBVHNode >& Nodes, int PrimitiveCount, RkPrimitve* PrimitiveList ) const;

		RkPrimitve* mPrimitiveBase;
	};


//--------------------------------------------------------------------------------------------------
void RkBVHBuilder::Run( RkArray< RkBVHNode >& Nodes, int PrimitiveCount, RkPrimitve* PrimitiveList )
	{
	// Build tree recursively
	mPrimitiveBase = PrimitiveList;
	BuildTree( Nodes, PrimitiveCount, PrimitiveList );
	}


//--------------------------------------------------------------------------------------------------
RkSplit RkBVHBuilder::SplitOptimal( int PrimitiveCount, RkPrimitve* PrimitiveList ) const
	{
	RkSplit Split;
	Split.Axis = -1;
	Split.Index = -1;

	// Compute bounds of primitive centroids and choose split axis
	RkBounds3 Bounds = RK_BOUNDS3_EMPTY;
	for ( int I = 0; I < PrimitiveCount; ++I )
		{
		RkVector3 Center = PrimitiveList[ I ].Bounds.GetCenter();
		Bounds += Center;
		}

	// Compute costs for splitting after each bucket and keep track of best split
	// This is a small O(n^2) loop. This can be further optimized, but it is already 
	// very fast and is kept for simplicity right now.
	int BestBucket = -1;
	float BestCost = RK_F32_MAX;

	for ( int Axis = 0; Axis < 3; ++Axis )
		{
		RkVector3 Extent = Bounds.GetExtent();
		if ( Extent[ Axis ] < 1000.0f * RK_F32_EPSILON )
			{
			continue;
			}

		// Initialize buckets
		RkBucket Buckets[ RK_BUCKET_COUNT ];
		for ( int I = 0; I < RK_BUCKET_COUNT; ++I )
			{
			Buckets[ I ].Count = 0;
			Buckets[ I ].Bounds = RK_BOUNDS3_EMPTY;
			}

		// Fill buckets
		float Factor = RK_BUCKET_COUNT * ( 1.0f - RK_F32_EPSILON ) / ( Bounds.Max[ Axis ] - Bounds.Min[ Axis ] );
		for ( int I = 0; I < PrimitiveCount; ++I )
			{
			RkVector3 Center = PrimitiveList[ I ].Bounds.GetCenter();
			int Index = int( Factor * ( Center[ Axis ] - Bounds.Min[ Axis ] ) );
			RK_ASSERT( 0 <= Index && Index < RK_BUCKET_COUNT );

			Buckets[ Index ].Count++;
			Buckets[ Index ].Bounds += PrimitiveList[ I ].Bounds;
			}

		// Evaluate splits 
		for ( int I = 0; I < RK_BUCKET_COUNT - 1; ++I )
			{
			int LeftCount = 0;
			RkBounds3 LeftBounds = RK_BOUNDS3_EMPTY;
			for ( int K = 0; K <= I; ++K )
				{
				LeftCount += Buckets[ K ].Count;
				LeftBounds += Buckets[ K ].Bounds;
				}

			int RightCount = 0;
			RkBounds3 RightBounds = RK_BOUNDS3_EMPTY;
			for ( int K = I + 1; K < RK_BUCKET_COUNT; ++K )
				{
				RightCount += Buckets[ K ].Count;
				RightBounds += Buckets[ K ].Bounds;
				}

			RK_ASSERT( LeftCount + RightCount == PrimitiveCount );
			if ( LeftCount > 0 && RightCount > 0 )
				{
				float Cost = LeftCount * LeftBounds.GetPerimeter() + RightCount * RightBounds.GetPerimeter();

				if ( Cost < BestCost )
					{
					BestBucket = I;
					BestCost = Cost;

					Split.Axis = Axis;
					Split.Index = LeftCount;
					Split.LeftBounds = LeftBounds;
					Split.RightBounds = RightBounds;
					}
				}
			}
		}

	// Partition
	if ( BestBucket >= 0 )
		{
		int Axis = Split.Axis;
		float Factor = RK_BUCKET_COUNT * ( 1.0f - RK_F32_EPSILON ) / ( Bounds.Max[ Axis ] - Bounds.Min[ Axis ] );

		int SplitIndex = 0;
		for ( int I = 0; I < PrimitiveCount; ++I )
			{
			RkVector3 Center = PrimitiveList[ I ].Bounds.GetCenter();
			int Index = int( Factor * ( Center[ Axis ] - Bounds.Min[ Axis ] ) );

			if ( Index <= BestBucket )
				{
				std::swap( PrimitiveList[ I ], PrimitiveList[ SplitIndex ] );
				SplitIndex++;
				}
			}
		RK_ASSERT( SplitIndex == Split.Index );
		}

	return Split;
	}


//--------------------------------------------------------------------------------------------------
RkSplit RkBVHBuilder::SplitHalf( int PrimitiveCount, RkPrimitve* PrimitiveList ) const
	{
	// Split in the middle
	int SplitIndex = PrimitiveCount / 2;

	RkBounds3 LeftBounds = RK_BOUNDS3_EMPTY;
	for ( int I = 0; I < SplitIndex; ++I )
		{
		LeftBounds += PrimitiveList[ I ].Bounds;
		}

	RkBounds3 RightBounds = RK_BOUNDS3_EMPTY;
	for ( int I = SplitIndex; I < PrimitiveCount; ++I )
		{
		RightBounds += PrimitiveList[ I ].Bounds;
		}

	RkBounds3 Bounds = LeftBounds + RightBounds;
	int Axis = rkMajorAxis( Bounds.GetExtent() );

	RkSplit Split;
	Split.Axis = Axis;
	Split.Index = SplitIndex;
	Split.LeftBounds = LeftBounds;
	Split.RightBounds = RightBounds;

	return Split;
	}


//--------------------------------------------------------------------------------------------------
bool RkBVHBuilder::ValidateSplit( int PrimitiveCount, RkPrimitve* PrimitiveList, const RkSplit& Split ) const
	{
	if ( Split.Axis < 0 )
		{
		return false;
		}

	for ( int I = 0; I < Split.Index; ++I )
		{
		if ( !Split.LeftBounds.Contains( PrimitiveList[ I ].Bounds ) )
			{
			return false;
			}
		}

	for ( int I = Split.Index; I < PrimitiveCount; ++I )
		{
		if ( !Split.RightBounds.Contains( PrimitiveList[ I ].Bounds ) )
			{
			return false;
			}
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
int RkBVHBuilder::BuildTree( RkArray< RkBVHNode >& Nodes, int PrimitiveCount, RkPrimitve* PrimitiveList ) const
	{
	if ( PrimitiveCount > RK_DESIRED_TRIANGLES_PER_LEAF )
		{
		// Try to split the input set using the SAH
		RkSplit Split = SplitOptimal( PrimitiveCount, PrimitiveList );
		if ( Split.Failed() )
			{
			if ( PrimitiveCount > RK_MAXIMUM_TRIANGLES_PER_LEAF )
				{
				// Re-split. This is a less optimal split and can create more false positives!
				Split = SplitHalf( PrimitiveCount, PrimitiveList );
				}
			else
				{	
				RkBounds3 Bounds = RK_BOUNDS3_EMPTY;
				for ( int I = 0; I < PrimitiveCount; ++I )
					{
					Bounds += PrimitiveList[ I ].Bounds;
					}

				// We have only a few triangles left. Create a leaf.
				int Index = Nodes.PushBack();
				rkStoreLeaf( Nodes[ Index ], Bounds, PrimitiveCount, int( PrimitiveList - mPrimitiveBase ) );

				return Index;
				}
			}
		RK_ASSERT( ValidateSplit( PrimitiveCount, PrimitiveList, Split ) );

		// Allocate node and recurse
		int Index = Nodes.PushBack();
		int LeftIndex = BuildTree( Nodes, Split.Index, PrimitiveList );
		int RightIndex = BuildTree( Nodes, PrimitiveCount - Split.Index, PrimitiveList + Split.Index );
		RK_ASSERT( LeftIndex - Index == 1 && RightIndex - Index > 1 );

		RkBounds3 Bounds = Split.LeftBounds + Split.RightBounds;
		rkStoreInternal( Nodes[ Index ], Split.Axis, Bounds, RightIndex - Index );
		
		return Index;
		}
	else
		{
		RkBounds3 Bounds = RK_BOUNDS3_EMPTY;
		for ( int I = 0; I < PrimitiveCount; ++I )
			{
			Bounds += PrimitiveList[ I ].Bounds;
			}

		int Index = Nodes.PushBack();
		rkStoreLeaf( Nodes[ Index ], Bounds, PrimitiveCount, int( PrimitiveList - mPrimitiveBase ) );

		return Index;
		}
	}


//--------------------------------------------------------------------------------------------------
static void rkSortBVHTriangles( RkBVH* BVH )
	{
	// Sort triangles in depth-first-order 
	int Offset = 0;
	RkArray< RkBVHTriangle > Triangles;
	Triangles.Reserve( BVH->TriangleCount );

	int Count = 0;
	RkBVHNode* Stack[ 64 ];
	Stack[ Count++ ] = BVH->GetRoot();

	while ( Count > 0 )
		{
		RkBVHNode* Node = Stack[ --Count ];

		if ( !Node->IsLeaf() )
			{
			Stack[ Count++ ] = Node->GetRightChild();
			Stack[ Count++ ] = Node->GetLeftChild();
			}	
		else
			{
			int TriangleCount = Node->TriangleCount;
			int TriangleOffset = Node->TriangleOffset;

			for ( int Triangle = 0; Triangle < TriangleCount; ++Triangle )
				{
				int Index = TriangleOffset + Triangle;
				Triangles.PushBack( BVH->GetTriangle( Index ) );
				}

			Node->TriangleOffset = Offset;
			Offset += TriangleCount;
			}
		}	

	RK_ASSERT( Offset == Triangles.Size() );
	RK_ASSERT( Triangles.Size() == BVH->TriangleCount );
	
	// Copy sorted triangle array back to tree. We cannot  
	// just swap pointers since the BVH triangles are aligned.
	rkMemCpy( BVH->TriangleList, Triangles.Begin(), BVH->TriangleCount * sizeof( RkBVHTriangle ) );
	}


//--------------------------------------------------------------------------------------------------
// BVH utilities
//--------------------------------------------------------------------------------------------------
RkBVH* rkCreateGrid( int Size, float Scale )
	{
	// Create vertices
	int VertexCount = ( 2 * Size + 1 ) * ( 2 * Size + 1 );

	RkArray< RkVector3 > VertexList;
	VertexList.Resize( VertexCount );

	RkVector3* VertexIterator = VertexList.Begin();
	for ( int X = -Size; X <= Size; ++X )
		{
		for ( int Z = -Size; Z <= Size; ++Z )
			{
			*VertexIterator++ = RkVector3( X * Scale, 0.0f, Z * Scale );
			}
		}
	RK_ASSERT( VertexIterator - VertexList.Begin() == VertexCount );

	// Triangles
	int TriangleCount = 2 * ( 2 * Size ) * ( 2 * Size );

	RkArray< int > IndexList;
	IndexList.Resize( 3 * TriangleCount );

	int* IndexIterator = IndexList.Begin();
	for ( int Row = 0; Row < 2 * Size; ++Row )
		{
		for ( int Col = 0; Col < 2 * Size; ++Col )
			{
			int Index1 = Col + ( 2 * Size + 1 ) * Row;
			int Index2 = Index1 + 1;
			int Index3 = Index2 + ( 2 * Size + 1 );
			int Index4 = Index3 - 1;

			*IndexIterator++ = Index1;
			*IndexIterator++ = Index2;
			*IndexIterator++ = Index3;

			*IndexIterator++ = Index3;
			*IndexIterator++ = Index4;
			*IndexIterator++ = Index1;
			}
		}
	RK_ASSERT( IndexIterator - IndexList.Begin() == 3 * TriangleCount );

	return rkCreateBVH( VertexCount, VertexList.Data(), TriangleCount, IndexList.Data() );
	}


//--------------------------------------------------------------------------------------------------
RkBVH* rkCreateTorus( int RadialResoltion, int TubularResolution, float Radius, float Thickness )
	{
	// Create vertices
	RkArray< RkVector3 > VertexList;

	for ( int RadialIndex = 0; RadialIndex < RadialResoltion; RadialIndex++ )
		{
		for ( int TubularIndex = 0; TubularIndex < TubularResolution; TubularIndex++ )
			{
			float U = (float)TubularIndex / TubularResolution * RK_2PI;
			float V = (float)RadialIndex / RadialResoltion * RK_2PI;
			
			float X = ( Radius + Thickness * rkCos( V ) ) * rkCos( U );
			float Y = ( Radius + Thickness * rkCos( V ) ) * rkSin( U );
			float Z = Thickness * rkSin( V );
			
			VertexList.PushBack( RkVector3( X, Y, Z ) );
			}
		}

	// Triangles 
	RkArray< int > IndexList;
	for ( int RadialIndex1 = 0; RadialIndex1 < RadialResoltion; RadialIndex1++ )
		{
		int RadialIndex2 = ( RadialIndex1 + 1 ) % RadialResoltion;
		for ( int TubularIndex1 = 0; TubularIndex1 < TubularResolution; TubularIndex1++ )
			{
			int TubularIndex2 = ( TubularIndex1 + 1 ) % TubularResolution;
			int Index1 = RadialIndex1 * TubularResolution + TubularIndex1;
			int Index2 = RadialIndex1 * TubularResolution + TubularIndex2;
			int Index3 = RadialIndex2 * TubularResolution + TubularIndex2;
			int Index4 = RadialIndex2 * TubularResolution + TubularIndex1;

			IndexList.PushBack( { Index1, Index2, Index3 } );
			IndexList.PushBack( { Index3, Index4, Index1 } );
			}
		}

	return rkCreateBVH( VertexList.Size(), VertexList.Data(), IndexList.Size() / 3, IndexList.Data() );
	}


//--------------------------------------------------------------------------------------------------
RkBVH* rkCreateBVH( int VertexCount, const RkVector3* VertexBase, int TriangleCount, const int* IndexBase )
	{
	// Validate input parameters
	if ( VertexCount < 3 || !VertexBase || TriangleCount <= 0 || !IndexBase )
		{
		return nullptr;
		}

	// Create primitive list
	RkBounds3 BVHBounds = RK_BOUNDS3_EMPTY;
	RkArray< RkPrimitve > PrimitiveList;
	PrimitiveList.Resize( TriangleCount );

	for ( int Index = 0; Index < TriangleCount; ++Index )
		{
		int Index1 = IndexBase[ 3 * Index + 0 ];
		int Index2 = IndexBase[ 3 * Index + 1 ];
		int Index3 = IndexBase[ 3 * Index + 2 ];

		RkVector3 Vertex1 = VertexBase[ Index1 ];
		RkVector3 Vertex2 = VertexBase[ Index2 ];
		RkVector3 Vertex3 = VertexBase[ Index3 ];

		RkBounds3 Bounds;
		Bounds.Min = rkMin( Vertex1, rkMin( Vertex2, Vertex3 ) );
		Bounds.Max = rkMax( Vertex1, rkMax( Vertex2, Vertex3 ) );

		PrimitiveList[ Index ].Index = Index;
		PrimitiveList[ Index ].Bounds = Bounds;
		
		BVHBounds += Bounds;
		}

	// Build the tree (this reorders the builder triangles)
	RkArray< RkBVHNode > Nodes;
	Nodes.Reserve( 2 * TriangleCount - 1 );

	RkBVHBuilder Builder;
	Builder.Run( Nodes, PrimitiveList.Size(), PrimitiveList.Begin() );

	// Allocate the BVH
	size_t ByteCount = sizeof( RkBVH );
	size_t NodeOffset = ByteCount = rkAlign( ByteCount, alignof( RkBVHNode ) );
	ByteCount += Nodes.Size() * sizeof( RkBVHNode );
	size_t TriangleOffset = ByteCount = rkAlign( ByteCount, alignof( RkBVHTriangle ) );
	ByteCount += TriangleCount * sizeof( RkBVHTriangle );
	size_t VertexOffset = ByteCount = rkAlign( ByteCount, alignof( RkVector3 ) );
	ByteCount += VertexCount * sizeof( RkVector3 );
	ByteCount = rkAlign( ByteCount, 16 );
	RK_ASSERT( ( ByteCount & 15 ) == 0 );

	// Fill the BVH
	RkBVH* BVH = (RkBVH*)rkAlignedAlloc( ByteCount, alignof( RkBVH ) );
	BVH->NodeList = (RkBVHNode*)rkAddByteOffset( BVH, NodeOffset );
	BVH->TriangleList = (RkBVHTriangle*)rkAddByteOffset( BVH, TriangleOffset );
	BVH->VertexList = (RkVector3*)rkAddByteOffset( BVH, VertexOffset );
	
	BVH->Bounds = BVHBounds;
	BVH->NodeCount = Nodes.Size();
	rkMemCpy( BVH->NodeList, Nodes.Data(), Nodes.Size() * sizeof( RkBVHNode ) );
	BVH->TriangleCount = TriangleCount;
	for ( int Index = 0; Index < TriangleCount; ++Index )
		{
		const RkPrimitve& Primitive = PrimitiveList[ Index ];
		BVH->TriangleList[ Index ].Index1 = IndexBase[ 3 * Primitive.Index + 0 ];
		BVH->TriangleList[ Index ].Index2 = IndexBase[ 3 * Primitive.Index + 1 ];
		BVH->TriangleList[ Index ].Index3 = IndexBase[ 3 * Primitive.Index + 2 ];
		}
	BVH->VertexCount = VertexCount;
	rkMemCpy( BVH->VertexList, VertexBase, VertexCount * sizeof( RkVector3 ) );
	
	// Sort triangle in DFS order. Casts and volume queries will return sorted arrays.
	rkSortBVHTriangles( BVH );
	
	RK_ASSERT( BVH->GetSize() == ByteCount && BVH->IsConsistent() );
	return BVH;
	}


//--------------------------------------------------------------------------------------------------
RkBVH* rkCloneBVH( const RkBVH* BVH )
	{
	if ( !BVH )
		{
		return nullptr;
		}

	RkBVH* Clone = static_cast< RkBVH* >( rkAlignedAlloc( BVH->GetSize(), alignof( RkBVHNode ) ) );
	rkMemCpy( Clone, BVH, BVH->GetSize() );

	return Clone;
	}


//--------------------------------------------------------------------------------------------------
void rkDestroyBVH( RkBVH*& BVH )
	{
	if ( BVH )
		{
		rkAlignedFree( BVH );
		BVH = nullptr;
		}
	}