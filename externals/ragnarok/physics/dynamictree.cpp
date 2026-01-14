//--------------------------------------------------------------------------------------------------
// dynamictree.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "dynamictree.h"


//--------------------------------------------------------------------------------------------------
// RkNodePool
//--------------------------------------------------------------------------------------------------
RkDynamicTreeNodePool::~RkDynamicTreeNodePool()
	{
	rkAlignedFree( mNodes );
	}


//--------------------------------------------------------------------------------------------------
int RkDynamicTreeNodePool::Size() const
	{
	return mSize;
	}


//--------------------------------------------------------------------------------------------------
int RkDynamicTreeNodePool::Capactity() const
	{
	return mCapacity;
	}


//--------------------------------------------------------------------------------------------------
bool RkDynamicTreeNodePool::Empty() const
	{
	return mSize == 0;
	}


//--------------------------------------------------------------------------------------------------
void RkDynamicTreeNodePool::Reserve( int Capacity )
	{
	if ( Capacity > mCapacity )
		{
		RkDynamicTreeNode* Nodes = mNodes;
		mNodes = static_cast< RkDynamicTreeNode* >( rkAlignedAlloc( Capacity * sizeof( RkDynamicTreeNode ), alignof( RkDynamicTreeNode ) ) );
		rkMemCpy( mNodes, Nodes, mCapacity * sizeof( RkDynamicTreeNode ) );

		rkAlignedFree( Nodes );
		Nodes = nullptr;

		// DIRK_TODO: Use bump pointer over new memory to avoid this!
		for ( int Index = mCapacity; Index < Capacity - 1; ++Index )
			{
			int* nNext = (int*)( mNodes + Index );
			*nNext = Index + 1;
			}
		int* nNext = (int*)( mNodes + Capacity - 1 );
		*nNext = -1;

		mNext = mCapacity;
		mCapacity = Capacity;
		}
	}


//--------------------------------------------------------------------------------------------------
int RkDynamicTreeNodePool::Alloc()
	{
	// Grow the pool if the free list is empty
	if ( mNext < 0 )
		{
		Reserve( rkMax( 2, 2 * mCapacity ) );
		}

	// Peel a node from the free list
	mSize++;
	int Index = mNext;
	mNext = *(int*)( mNodes + Index );

	return Index;
	}


//--------------------------------------------------------------------------------------------------
void RkDynamicTreeNodePool::Free( int Index )
	{
	// Re-insert object into free list
	RK_ASSERT( 0 <= Index && Index < mCapacity );

	mSize--;
	*(int*)( mNodes + Index ) = mNext;
	mNext = Index;
	}


//--------------------------------------------------------------------------------------------------
RkDynamicTreeNode& RkDynamicTreeNodePool::operator[]( int Index )
	{
	RK_ASSERT( 0 <= Index && Index < mCapacity );
	return mNodes[ Index ];
	}


//--------------------------------------------------------------------------------------------------
const RkDynamicTreeNode& RkDynamicTreeNodePool::operator[]( int Index ) const
	{
	RK_ASSERT( 0 <= Index && Index < mCapacity );
	return mNodes[ Index ];
	}


//--------------------------------------------------------------------------------------------------
// RkDynamicTree
//--------------------------------------------------------------------------------------------------
RkDynamicTree::RkDynamicTree()
	: mRoot( RK_NULL_NODE )
	, mProxyCount( 0 )
	, mDirty( false )
	{
	
	}


//--------------------------------------------------------------------------------------------------
int RkDynamicTree::CreateProxy( const RkBounds3& Bounds, void* UserData )
	{
	RK_ASSERT( Bounds.IsValid() );

	int Proxy = mNodes.Alloc();
	mNodes[ Proxy ].BoundsMin = Bounds.Min;
	mNodes[ Proxy ].Child1 = RK_NULL_NODE;
	mNodes[ Proxy ].BoundsMax = Bounds.Max;
	mNodes[ Proxy ].Child2 = RK_NULL_NODE;
	mNodes[ Proxy ].Parent = RK_NULL_NODE;
	mNodes[ Proxy ].Height = 0;
	mNodes[ Proxy ].Dirty = false;
	mNodes[ Proxy ].UserData = UserData;
	InsertLeaf( Proxy, true );

	mProxyCount++;
	RK_ASSERT( mNodes.Size() == 2 * mProxyCount - 1 );

	return Proxy;
	}


//--------------------------------------------------------------------------------------------------
void RkDynamicTree::MoveProxy( int Proxy, const RkBounds3& Bounds )
	{
	RK_ASSERT( mNodes[ Proxy ].IsLeaf() );
	RK_ASSERT( Bounds.IsValid() );
	
	RemoveLeaf( Proxy );
	mNodes[ Proxy ].BoundsMin = Bounds.Min;
	mNodes[ Proxy ].Child1 = RK_NULL_NODE;
	mNodes[ Proxy ].BoundsMax = Bounds.Max;
	mNodes[ Proxy ].Child2 = RK_NULL_NODE;
	mNodes[ Proxy ].Parent = RK_NULL_NODE;
	mNodes[ Proxy ].Height = 0;
	mNodes[ Proxy ].Dirty = false;
	InsertLeaf( Proxy, false );

	RK_ASSERT( mNodes.Size() == 2 * mProxyCount - 1 );
	}


//--------------------------------------------------------------------------------------------------
void* RkDynamicTree::DestroyProxy( int Proxy )
	{
	RK_ASSERT( mNodes[ Proxy ].IsLeaf() );

	// Grab user data from the node to return it
	void* pUserData = mNodes[ Proxy ].UserData;

	// Remove node from tree and free it
	RemoveLeaf( Proxy );
	mNodes.Free( Proxy );

	mProxyCount--;
	RK_ASSERT( mNodes.Size() == rkMax( 0, 2 * mProxyCount - 1 ) );

	return pUserData;
	}


//--------------------------------------------------------------------------------------------------
int RkDynamicTree::GetHeight() const
	{
	if ( mRoot == RK_NULL_NODE )
		{
		return 0;
		}

	return mNodes[ mRoot ].Height;
	}


//--------------------------------------------------------------------------------------------------
int RkDynamicTree::GetMaxBalance() const
	{
	int MaxBalance = 0;

	int Count = 0;
	int Stack[ RK_STACK_SIZE ];
	Stack[ Count++ ] = mRoot;

	while ( Count > 0 )
		{
		int Node = Stack[ --Count ];
		RK_ASSERT( Node != RK_NULL_NODE );

		if ( !mNodes[ Node ].IsLeaf() )
			{
			int Child1 = mNodes[ Node ].Child1;
			int Height1 = mNodes[ Child1 ].Height;
			int Child2 = mNodes[ Node ].Child2;
			int Height2 = mNodes[ Child2 ].Height;

			int Balance = rkAbs( Height1 - Height2 );
			MaxBalance = rkMax( MaxBalance, Balance );

			// Recurse
			RK_ASSERT( Count <= RK_STACK_SIZE - 2 );
			Stack[ Count++ ] = Child2;
			Stack[ Count++ ] = Child1;
			}
		}

	return MaxBalance;
	}


//--------------------------------------------------------------------------------------------------
float RkDynamicTree::GetArea() const
	{
	// Area of internal nodes 
	float TotalArea = 0.0f;
	if ( mRoot == RK_NULL_NODE )
		{
		return TotalArea;
		}

	int Count = 0;
	int Stack[ RK_STACK_SIZE ];
	Stack[ Count++ ] = mRoot;

	while ( Count > 0 )
		{
		int Node = Stack[ --Count ];
		RK_ASSERT( Node != RK_NULL_NODE );

		// Skip root and leaves
		if ( Node != mRoot && !mNodes[ Node ].IsLeaf() )
			{
			RkBounds3 Bounds = mNodes[ Node ].GetBounds();
			float Area = Bounds.GetPerimeter();
			TotalArea += Area;
			}

		if ( !mNodes[ Node ].IsLeaf() )
			{
			RK_ASSERT( Count <= RK_STACK_SIZE - 2 );
			Stack[ Count++ ] = mNodes[ Node ].Child2;
			Stack[ Count++ ] = mNodes[ Node ].Child1;
			}
		}

	return TotalArea;
	}

//--------------------------------------------------------------------------------------------------
float RkDynamicTree::GetAreaRatio() const
	{
	return mRoot != RK_NULL_NODE ? GetArea() / rkPerimeter( mNodes[ mRoot ].GetBounds() ) : 0.0f;
	}


//--------------------------------------------------------------------------------------------------
RkBounds3 RkDynamicTree::GetBounds() const
	{
	return mRoot != RK_NULL_NODE ? mNodes[ mRoot ].GetBounds() : RK_BOUNDS3_EMPTY;
	}


//--------------------------------------------------------------------------------------------------
bool RkDynamicTree::IsDirty() const
	{
	return mDirty;
	}


//--------------------------------------------------------------------------------------------------
void RkDynamicTree::RefitProxy( int Proxy, const RkBounds3& Bounds )
	{
	RK_ASSERT( mNodes[ Proxy ].IsLeaf() );
	RK_ASSERT( Bounds.IsValid() );

	// Must be ensured by caller (e.g. RkBody/RkShape)
	RK_ASSERT( !rkContains( mNodes[ Proxy ].GetBounds(), Bounds ) );
	mNodes[ Proxy ].BoundsMin = Bounds.Min;
	mNodes[ Proxy ].BoundsMax = Bounds.Max;
	mDirty = true;

	// Pass 1: Grow and mark nodes until node movement is absorbed by parent
	v32 BoundsMin = vmLoad3( Bounds.Min );
	v32 BoundsMax = vmLoad3( Bounds.Max );

	int Parent = mNodes[ Proxy ].Parent;
	while ( Parent != RK_NULL_NODE )
		{
		v32 NodeMin = vmLoad3A( mNodes[ Parent ].BoundsMin );
		v32 NodeMax = vmLoad3A( mNodes[ Parent ].BoundsMax );
		if ( vmContains( NodeMin, NodeMax, BoundsMin, BoundsMax ) )
			{
			break;
			}

		vmUnion( NodeMin, NodeMax, BoundsMin, BoundsMax );
		vmStore3A( mNodes[ Parent ].BoundsMin, NodeMin );
		vmStore3A( mNodes[ Parent ].BoundsMax, NodeMax );

		mNodes[ Parent ].Dirty = true;
		Parent = mNodes[ Parent ].Parent;
		}

	// Pass 2: Continue marking remaining nodes without growing
	while ( Parent != RK_NULL_NODE )
		{
		// Early out if already dirty - all ancestors must be dirty too!
		if ( mNodes[ Parent ].Dirty )
			{
			break;
			}

		mNodes[ Parent ].Dirty = true;
		Parent = mNodes[ Parent ].Parent;
		}

	Validate( mRoot );
	}


//--------------------------------------------------------------------------------------------------
void RkDynamicTree::CollectLeaves( RkArray< int >& Leaves )
	{
	RK_ASSERT( mNodes.Size() == 2 * mProxyCount - 1 );

	int Count = 0;
	int Stack[ RK_STACK_SIZE ];
	int Node = mRoot;

	while ( true )
		{
		if ( mNodes[ Node ].IsLeaf() || !mNodes[ Node ].Dirty )
			{
			// Detach and add leaves *and* roots of unchanged subtrees for rebuild
			RK_ASSERT( !mNodes[ Node ].Dirty );
			mNodes[ Node ].Parent = RK_NULL_NODE;
			Leaves.PushBack( Node );
			}
		else
			{
			// Delete all internal nodes that have grown
			int Nuke = Node;

			// Recurse
			Stack[ Count++ ] = mNodes[ Node ].Child2;
			Node = mNodes[ Node ].Child1;

			// Free internal node
			mNodes.Free( Nuke );

			continue;
			}

		if ( Count == 0 )
			{
			break;
			}

		Node = Stack[ --Count ];
		}
	}


//--------------------------------------------------------------------------------------------------
void RkDynamicTree::Rebuild()
	{
	if ( !mDirty )
		{
		return;
		}
	mDirty = false;

	RkStackArray< int, 512 > Leaves;
	CollectLeaves( Leaves );
	
	mRoot = RebuildRecursive( Leaves.Begin(), Leaves.End() );
	RK_ASSERT( mNodes.Size() == 2 * mProxyCount - 1 );

	Validate( mRoot );
	}


//--------------------------------------------------------------------------------------------------
int RkDynamicTree::RebuildRecursive( int* First, int* Last, int Parent )
	{
	int LeafCount = static_cast< int >( Last - First );
	RK_ASSERT( LeafCount > 0 );

	if ( LeafCount == 1 )
		{
		// Hang leaf into tree. Remember that the leaf might 
		// be an old internal node in the rebuild context here!
		int Leaf = First[ 0 ];
		RK_ASSERT( mNodes[ Leaf ].Parent == RK_NULL_NODE );
		mNodes[ Leaf ].Parent = Parent;
		mNodes[ Leaf ].Dirty = false;

		return Leaf;
		}
	else
		{
		// Partition leaf nodes using the 'Median Split' 
		// method and create a new internal node.
		RkBounds3 CenterBounds = BuildBounds( First, Last );
		RkVector3 CenterExtent = CenterBounds.Max - CenterBounds.Min;

		int* Mid = First + LeafCount / 2;
		int SplitAxis = rkMajorAxis( CenterExtent );
		std::nth_element( First, Mid, Last, [&]( int Leaf1, int Leaf2 )
			{
			float Center1 = 0.5f * ( mNodes[ Leaf1 ].BoundsMin[ SplitAxis ] + mNodes[ Leaf1 ].BoundsMax[ SplitAxis ] );
			float Center2 = 0.5f * ( mNodes[ Leaf2 ].BoundsMin[ SplitAxis ] + mNodes[ Leaf2 ].BoundsMax[ SplitAxis ] );

			return Center1 < Center2;
			} );
			
		int Node = mNodes.Alloc();
		int Child1 = RebuildRecursive( First, Mid, Node );
		int Child2 = RebuildRecursive( Mid, Last, Node );
		RkBounds3 Bounds = rkUnion( mNodes[ Child1 ].GetBounds(), mNodes[ Child2 ].GetBounds() );

		mNodes[ Node ].BoundsMin = Bounds.Min;
		mNodes[ Node ].Child1 = Child1;
		mNodes[ Node ].BoundsMax = Bounds.Max;
		mNodes[ Node ].Child2 = Child2;
		mNodes[ Node ].Parent = Parent;
		mNodes[ Node ].Height = 1 + rkMax( mNodes[ Child1 ].Height, mNodes[ Child2 ].Height );
		mNodes[ Node ].Dirty = false;
		mNodes[ Node ].UserData = nullptr;

		return Node;
		}
	}


//--------------------------------------------------------------------------------------------------
RkBounds3 RkDynamicTree::BuildBounds( const int* First, const int* Last ) const
	{
	// Build bounds of node *centers*
	RkBounds3 Bounds = RK_BOUNDS3_EMPTY;
	while ( First != Last )
		{
		int Node = *First++;
		Bounds += 0.5f * ( mNodes[ Node ].BoundsMin + mNodes[ Node ].BoundsMax );
		}

	return Bounds;
	}


//--------------------------------------------------------------------------------------------------
int RkDynamicTree::FindBestSibling( const RkBounds3& BoxD ) const
	{
	// Greedy algorithm for sibling selection using the SAH
	//
	// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
	// 1: make a new parent for A and D : E-(A-(B,C), D)
	// 2: associate D with B
	//     a: B is a leaf : A-(E-(B,D), C)
	//     b: B is an internal node: A-(B{D},C)
	// 3: associate D with C
	//     a: C is a leaf : A-(B, E-(C,D))
	//     b: C is an internal node: A-(B, C{D})
	// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.
	//
	// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
	// cost of sibling H = area(union(H, D)) + increased are of ancestors
	//
	// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
	// case1: D becomes a sibling of B
	// case2: D becomes a decedent of B along with a new internal node of area(D).

	RkVector3 CenterD = rkCenter( BoxD );
	float AreaD = rkPerimeter( BoxD );

	// Area of current node
	RkBounds3 RootBox = mNodes[ mRoot ].GetBounds();
	float AreaBase = rkPerimeter( RootBox );

	// Area of inflated node
	float DirectCost = rkPerimeter( RootBox + BoxD );
	float InheritedCost = 0.0f;

	int BestSibling = mRoot;
	float BestCost = DirectCost;

	// Descend the tree from root, following a single greedy path.
	int Index = mRoot;
	while ( mNodes[ Index ].IsLeaf() == false )
		{
		int Child1 = mNodes[ Index ].Child1;
		int Child2 = mNodes[ Index ].Child2;

		// Cost of creating a new parent for this node and the new leaf
		float Cost = DirectCost + InheritedCost;

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if ( Cost < BestCost )
			{
			BestSibling = Index;
			BestCost = Cost;
			}

		// Inheritance cost seen by children
		InheritedCost += DirectCost - AreaBase;

		bool Leaf1 = mNodes[ Child1 ].IsLeaf();
		bool Leaf2 = mNodes[ Child2 ].IsLeaf();

		// Cost of descending into child 1
		float LowerCost1 = RK_F32_MAX;
		RkBounds3 Box1 = mNodes[ Child1 ].GetBounds();
		float DirectCost1 = rkPerimeter( Box1 + BoxD );
		float Area1 = 0.0f;
		if ( Leaf1 )
			{
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			float Cost1 = DirectCost1 + InheritedCost;

			// Need this here due to while condition above
			if ( Cost1 < BestCost )
				{
				BestSibling = Child1;
				BestCost = Cost1;
				}
			}
		else
			{
			// Child 1 is an internal node
			Area1 = rkPerimeter( Box1 );

			// Lower bound cost of inserting under child 1.
			LowerCost1 = InheritedCost + DirectCost1 + rkMin( AreaD - Area1, 0.0f );
			}

			// Cost of descending into child 2
		float LowerCost2 = RK_F32_MAX;
		RkBounds3 Box2 = mNodes[ Child2 ].GetBounds();
		float DirectCost2 = rkPerimeter( Box2 + BoxD );
		float Area2 = 0.0f;
		if ( Leaf2 )
			{
			// Child 2 is a leaf
			// Cost of creating new node and increasing area of node P
			float Cost2 = DirectCost2 + InheritedCost;

			// Need this here due to while condition above
			if ( Cost2 < BestCost )
				{
				BestSibling = Child2;
				BestCost = Cost2;
				}
			}
		else
			{
			// Child 2 is an internal node
			Area2 = rkPerimeter( Box2 );

			// Lower bound cost of inserting under child 2. This is not the cost
			// of child 2, it is the best we can hope for under child 2.
			LowerCost2 = InheritedCost + DirectCost2 + rkMin( AreaD - Area2, 0.0f );
			}

		if ( Leaf1 && Leaf2 )
			{
			break;
			}

		// Can the cost possibly be decreased?
		if ( BestCost <= LowerCost1 && BestCost <= LowerCost2 )
			{
			break;
			}

		if ( LowerCost1 == LowerCost2 && Leaf1 == false )
			{
			RK_ASSERT( LowerCost1 < RK_F32_MAX );
			RK_ASSERT( LowerCost2 < RK_F32_MAX );

			// No clear choice based on lower bound surface area. This can happen when both
			// children fully contain L. Fall back to node distance.
			RkVector3 Offset1 = rkCenter( Box1 ) - CenterD;
			RkVector3 Offset2 = rkCenter( Box2 ) - CenterD;
			LowerCost1 = rkDot( Offset1, Offset1 );
			LowerCost2 = rkDot( Offset2, Offset2 );
			}

		// Descend
		if ( LowerCost1 < LowerCost2 && Leaf1 == false )
			{
			Index = Child1;
			AreaBase = Area1;
			DirectCost = DirectCost1;
			}
		else
			{
			Index = Child2;
			AreaBase = Area2;
			DirectCost = DirectCost2;
			}

		RK_ASSERT( mNodes[ Index ].IsLeaf() == false );
		}

	return BestSibling;
	}


//--------------------------------------------------------------------------------------------------
void RkDynamicTree::InsertLeaf( int Leaf, bool ShouldRotate )
	{
	if ( mRoot == RK_NULL_NODE )
		{
		mRoot = Leaf;
		mNodes[ Leaf ].Parent = RK_NULL_NODE;

		return;
		}

	Validate( mRoot );

	RkBounds3 LeafBounds = mNodes[ Leaf ].GetBounds();
	int Sibling = FindBestSibling( LeafBounds );

	// Create and insert new Parent
	RkBounds3 SiblingBounds = mNodes[ Sibling ].GetBounds();
	RkBounds3 NewBounds = SiblingBounds + LeafBounds;

	int NewParent = mNodes.Alloc();
	mNodes[ NewParent ].BoundsMin = NewBounds.Min;
	mNodes[ NewParent ].Child1 = Sibling;
	mNodes[ NewParent ].BoundsMax = NewBounds.Max;
	mNodes[ NewParent ].Child2 = Leaf;
	mNodes[ NewParent ].Parent = mNodes[ Sibling ].Parent;
	mNodes[ NewParent ].Height = mNodes[ Sibling ].Height + 1;
	mNodes[ NewParent ].Dirty = false;
	mNodes[ NewParent ].UserData = nullptr;

	int OldParent = mNodes[ Sibling ].Parent;
	if ( OldParent != RK_NULL_NODE )
		{
			// We are not inserting at the root
		if ( mNodes[ OldParent ].Child1 == Sibling )
			{
			mNodes[ OldParent ].Child1 = NewParent;
			}
		else
			{
			mNodes[ OldParent ].Child2 = NewParent;
			}
		}
	else
		{
		// Inserting at the root
		mRoot = NewParent;
		}

	mNodes[ Sibling ].Parent = NewParent;
	mNodes[ Leaf ].Parent = NewParent;

	// Walk back up the tree and fix heights and AABBs
	AdjustAncestors( mNodes[ Leaf ].Parent, ShouldRotate );

	// This means you likely have many overlapping objects (e.g. at the origin)
	RK_ASSERT( mNodes[ mRoot ].Height < RK_STACK_SIZE );

	Validate( mRoot );
	}


//--------------------------------------------------------------------------------------------------
void RkDynamicTree::RemoveLeaf( int Leaf )
	{
	RK_ASSERT( mNodes[ Leaf ].IsLeaf() );

	Validate( mRoot );

	if ( Leaf == mRoot )
		{
		mRoot = RK_NULL_NODE;
		return;
		}

	int Parent = mNodes[ Leaf ].Parent;

	int Sibling = RK_NULL_NODE;
	if ( mNodes[ Parent ].Child1 == Leaf )
		{
		Sibling = mNodes[ Parent ].Child2;
		}
	else
		{
		Sibling = mNodes[ Parent ].Child1;
		}

	int GrandParent = mNodes[ Parent ].Parent;
	if ( GrandParent != RK_NULL_NODE )
		{
		// Destroy parent and connect sibling to grandparent
		mNodes[ Sibling ].Parent = GrandParent;

		if ( mNodes[ GrandParent ].Child1 == Parent )
			{
			mNodes[ GrandParent ].Child1 = Sibling;
			}
		else
			{
			mNodes[ GrandParent ].Child2 = Sibling;
			}

		// Walk back up the tree and fix heights and AABBs
		AdjustAncestors( GrandParent, false );	
		}
	else
		{
		mNodes[ Sibling ].Parent = RK_NULL_NODE;
		mRoot = Sibling;
		}

	mNodes.Free( Parent );

	Validate( mRoot );
	}



//--------------------------------------------------------------------------------------------------
void RkDynamicTree::AdjustAncestors( int Node, bool ShouldRotate )
	{
	while ( Node != RK_NULL_NODE )
		{
		int Child1 = mNodes[ Node ].Child1;
		RK_ASSERT( Child1 != RK_NULL_NODE );
		int Child2 = mNodes[ Node ].Child2;
		RK_ASSERT( Child2 != RK_NULL_NODE );

		RkBounds3 Bounds = mNodes[ Child1 ].GetBounds() + mNodes[ Child2 ].GetBounds();
		mNodes[ Node ].BoundsMin = Bounds.Min;
		mNodes[ Node ].BoundsMax = Bounds.Max;
		mNodes[ Node ].Height = 1 + rkMax( mNodes[ Child1 ].Height, mNodes[ Child2 ].Height );
		
		if ( ShouldRotate )
			{
			Rotate( Node );
			}

		Node = mNodes[ Node ].Parent;
		}
	}


//--------------------------------------------------------------------------------------------------
void RkDynamicTree::Rotate( int IndexA )
	{
	RK_ASSERT( IndexA != RK_NULL_NODE );

	RkDynamicTreeNode& NodeA = mNodes[ IndexA ];
	if ( NodeA.Height < 2 )
		{
		return;
		}

	int IndexB = NodeA.Child1;
	RkDynamicTreeNode& NodeB = mNodes[ IndexB ];
	int IndexC = NodeA.Child2;
	RkDynamicTreeNode& NodeC = mNodes[ IndexC ];

	if ( NodeB.Height == 0 )
		{
		// B is a leaf
		RK_ASSERT( NodeC.Height > 0 );

		int IndexF = NodeC.Child1;
		RkDynamicTreeNode& NodeF = mNodes[ IndexF ];
		int IndexG = NodeC.Child2;
		RkDynamicTreeNode& NodeG = mNodes[ IndexG ];

		// Base cost
		float CostBase = rkPerimeter( NodeC.GetBounds() );

		// Cost of swapping B and F
		RkBounds3 BoundsBG = rkUnion( NodeB.GetBounds(), NodeG.GetBounds() );
		float CostBF = rkPerimeter( BoundsBG );

		// Cost of swapping B and G
		RkBounds3 BoundsBF = rkUnion( NodeB.GetBounds(), NodeF.GetBounds() );
		float CostBG = rkPerimeter( BoundsBF );

		if ( CostBase < CostBF && CostBase < CostBG )
			{
			// Rotation does not improve cost
			return;
			}

		if ( CostBF < CostBG )
			{
			// Swap B and F
			NodeA.Child1 = IndexF;
			NodeC.Child1 = IndexB;

			NodeB.Parent = IndexC;
			NodeF.Parent = IndexA;

			NodeC.BoundsMin = BoundsBG.Min;
			NodeC.BoundsMax = BoundsBG.Max;
			NodeC.Height = 1 + rkMax( NodeB.Height, NodeG.Height );
			NodeA.Height = 1 + rkMax( NodeC.Height, NodeF.Height );
			}
		else
			{
			// Swap B and G
			NodeA.Child1 = IndexG;
			NodeC.Child2 = IndexB;

			NodeB.Parent = IndexC;
			NodeG.Parent = IndexA;

			NodeC.BoundsMin = BoundsBF.Min;
			NodeC.BoundsMax = BoundsBF.Max;
			NodeC.Height = 1 + rkMax( NodeB.Height, NodeF.Height );
			NodeA.Height = 1 + rkMax( NodeC.Height, NodeG.Height );
			}
		}
	else if ( NodeC.Height == 0 )
		{
		// C is a leaf
		RK_ASSERT( NodeB.Height > 0 );

		int IndexD = NodeB.Child1;
		RkDynamicTreeNode& NodeD = mNodes[ IndexD ];
		int IndexE = NodeB.Child2;
		RkDynamicTreeNode& NodeE = mNodes[ IndexE ];

		// Base cost
		float CostBase = rkPerimeter( NodeB.GetBounds() );

		// Cost of swapping C and D
		RkBounds3 BoundsCE = rkUnion( NodeC.GetBounds(), NodeE.GetBounds() );
		float CostCD = rkPerimeter( BoundsCE );

		// Cost of swapping C and E
		RkBounds3 BoundsCD = rkUnion( NodeC.GetBounds(), NodeD.GetBounds() );
		float CostCE = rkPerimeter( BoundsCD );

		if ( CostBase < CostCD && CostBase < CostCE )
			{
			// Rotation does not improve cost
			return;
			}

		if ( CostCD < CostCE )
			{
			// Swap C and D
			NodeA.Child2 = IndexD;
			NodeB.Child1 = IndexC;

			NodeC.Parent = IndexB;
			NodeD.Parent = IndexA;

			NodeB.BoundsMin = BoundsCE.Min;
			NodeB.BoundsMax = BoundsCE.Max;
			NodeB.Height = 1 + rkMax( NodeC.Height, NodeE.Height );
			NodeA.Height = 1 + rkMax( NodeB.Height, NodeD.Height );
			}
		else
			{
			// Swap C and E
			NodeA.Child2 = IndexE;
			NodeB.Child2 = IndexC;

			NodeC.Parent = IndexB;
			NodeE.Parent = IndexA;

			NodeB.BoundsMin = BoundsCD.Min;
			NodeB.BoundsMax = BoundsCD.Max;
			NodeB.Height = 1 + rkMax( NodeC.Height, NodeD.Height );
			NodeA.Height = 1 + rkMax( NodeB.Height, NodeE.Height );
			}
		}
	else
		{
		int IndexD = NodeB.Child1;
		RkDynamicTreeNode& NodeD = mNodes[ IndexD ];
		int IndexE = NodeB.Child2;
		RkDynamicTreeNode& NodeE = mNodes[ IndexE ];
		int IndexF = NodeC.Child1;
		RkDynamicTreeNode& NodeF = mNodes[ IndexF ];
		int IndexG = NodeC.Child2;
		RkDynamicTreeNode& NodeG = mNodes[ IndexG ];

		// Base cost
		float AreaB = rkPerimeter( NodeB.GetBounds() );
		float AreaC = rkPerimeter( NodeC.GetBounds() );
		float CostBase = AreaB + AreaC;

		enum RkRotation { None, BF, BG, CD, CE };
		RkRotation BestRotation = None;
		float BestCost = CostBase;

		// Cost of swapping B and F
		RkBounds3 BoundsBG = rkUnion( NodeB.GetBounds(), NodeG.GetBounds() );
		float CostBF = AreaB + rkPerimeter( BoundsBG );
		if ( CostBF < BestCost )
			{
			BestRotation = BF;
			BestCost = CostBF;
			}

		// Cost of swapping B and G
		RkBounds3 BoundsBF = rkUnion( NodeB.GetBounds(), NodeF.GetBounds() );
		float CostBG = AreaB + rkPerimeter( BoundsBF );
		if ( CostBG < BestCost )
			{
			BestRotation = BG;
			BestCost = CostBG;
			}

		// Cost of swapping C and D
		RkBounds3 BoundsCE = rkUnion( NodeC.GetBounds(), NodeE.GetBounds() );
		float CostCD = AreaC + rkPerimeter( BoundsCE );
		if ( CostCD < BestCost )
			{
			BestRotation = CD;
			BestCost = CostCD;
			}

		// Cost of swapping C and E
		RkBounds3 BoundsCD = rkUnion( NodeC.GetBounds(), NodeD.GetBounds() );
		float CostCE = AreaC + rkPerimeter( BoundsCD );
		if ( CostCE < BestCost )
			{
			BestRotation = CE;
			BestCost = CostCE;
			}

		switch ( BestRotation )
			{
			case None:
				break;

			case BF:
				NodeA.Child1 = IndexF;
				NodeC.Child1 = IndexB;

				NodeB.Parent = IndexC;
				NodeF.Parent = IndexA;

				NodeC.BoundsMin = BoundsBG.Min;
				NodeC.BoundsMax = BoundsBG.Max;
				NodeC.Height = 1 + rkMax( NodeB.Height, NodeG.Height );
				NodeA.Height = 1 + rkMax( NodeC.Height, NodeF.Height );
				break;

			case BG:
				NodeA.Child1 = IndexG;
				NodeC.Child2 = IndexB;

				NodeB.Parent = IndexC;
				NodeG.Parent = IndexA;

				NodeC.BoundsMin = BoundsBF.Min;
				NodeC.BoundsMax = BoundsBF.Max;
				NodeC.Height = 1 + rkMax( NodeB.Height, NodeF.Height );
				NodeA.Height = 1 + rkMax( NodeC.Height, NodeG.Height );
				break;

			case CD:
				NodeA.Child2 = IndexD;
				NodeB.Child1 = IndexC;

				NodeC.Parent = IndexB;
				NodeD.Parent = IndexA;

				NodeB.BoundsMin = BoundsCE.Min;
				NodeB.BoundsMax = BoundsCE.Max;
				NodeB.Height = 1 + rkMax( NodeC.Height, NodeE.Height );
				NodeA.Height = 1 + rkMax( NodeB.Height, NodeD.Height );
				break;

			case CE:
				NodeA.Child2 = IndexE;
				NodeB.Child2 = IndexC;

				NodeC.Parent = IndexB;
				NodeE.Parent = IndexA;

				NodeB.BoundsMin = BoundsCD.Min;
				NodeB.BoundsMax = BoundsCD.Max;
				NodeB.Height = 1 + rkMax( NodeC.Height, NodeD.Height );
				NodeA.Height = 1 + rkMax( NodeB.Height, NodeE.Height );
				break;

			default:
				RK_ASSERT( false );
				break;
			}
		}

	Validate( IndexA );
	}


//--------------------------------------------------------------------------------------------------
void RkDynamicTree::Validate( int Index ) const
	{
//#define RK_VALIDATE_TREE
#ifdef RK_VALIDATE_TREE
	RK_ASSERT( Index != RK_NULL_NODE );
	const RkDynamicTreeNode& Node = mNodes[ Index ];

	if ( Index == mRoot )
		{
		RK_ASSERT( Node.Parent == RK_NULL_NODE );
		}

	if ( Node.IsLeaf() )
		{
		RK_ASSERT( Node.Height == 0 );
		RK_ASSERT( Node.Child1 == RK_NULL_NODE );
		RK_ASSERT( Node.Child2 == RK_NULL_NODE );
		
		return;
		}

	RK_ASSERT( Node.Child1 != Node.Child2 );

	RK_ASSERT( Node.Child1 != RK_NULL_NODE );
	const RkDynamicTreeNode& Child1 = mNodes[ Node.Child1 ];
	RK_ASSERT( Child1.Parent == Index );

	RK_ASSERT( Node.Child2 != RK_NULL_NODE );
	const RkDynamicTreeNode& Child2 = mNodes[ Node.Child2 ];
	RK_ASSERT( Child2.Parent == Index );

	int Height1 = Child1.Height;
	int Height2 = Child2.Height;
	int Height = 1 + rkMax( Height1, Height2 );
	RK_ASSERT( Node.Height == Height );

	// If the tree is dirty we can only guarentee integrity, but not exact match (s. RefitProxy())
	RkBounds3 Bounds = Child1.GetBounds() + Child2.GetBounds();
	RK_ASSERT( mDirty ? rkContains( Node.GetBounds(), Bounds ) : Node.GetBounds() == Bounds );

	// Recurse
	Validate( Node.Child1 );
	Validate( Node.Child2 );
#endif
	}