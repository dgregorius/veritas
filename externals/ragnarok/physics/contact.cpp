//--------------------------------------------------------------------------------------------------
// contact.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "contact.h"
#include "body.h"
#include "island.h"
#include "shape.h"
#include "world.h"

// Internal 
#include "solver.h"
#include "contactsolver.h"

// STL
#include <algorithm>


//--------------------------------------------------------------------------------------------------
// RkContact
//--------------------------------------------------------------------------------------------------
RkContact::RkContact( RkContactType Type, RkWorld* World, RkShape* Shape1, RkShape* Shape2 )
	{
	mType = Type;
	
	RK_ASSERT( World );
	mWorld = World;

	RK_ASSERT( Shape1 );
	RK_ASSERT( Shape2 );
	RK_ASSERT( Shape1 != Shape2 );
	mShape1 = Shape1;
	mShape2 = Shape2;
	mBody1 = Shape1->GetBody();
	RK_ASSERT( mBody1 && mBody1->GetWorld() == mWorld );
	mBody2 = Shape2->GetBody();
	RK_ASSERT( mBody2 && mBody2->GetWorld() == mWorld );
	RK_ASSERT( mBody1 != mBody2 );

	mManifoldCount = 0;
	mCollideFlags = 0;
	
	// Register this pair with the associated bodies
	mEdge1.Other = mBody2;
	mEdge1.Contact = this;
	mEdge1.Prev = nullptr;
	mEdge1.Next = nullptr;
	mBody1->AddContact( &mEdge1 );

	mEdge2.Other = mBody1;
	mEdge2.Contact = this;
	mEdge2.Prev = nullptr;
	mEdge2.Next = nullptr;
	mBody2->AddContact( &mEdge2 );
	
	// Contacts must be explicitly added to islands when start touching!
	mIsland = nullptr;
	mIslandIndex = -1;
	
	// User data
	mUserData = nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkContact::~RkContact()
	{
	// Unregister this joint from the associated bodies
	mBody2->RemoveContact( &mEdge2 );
	mBody1->RemoveContact( &mEdge1 );
	}


//--------------------------------------------------------------------------------------------------
bool RkContact::IsTouching() const
	{
	RK_ASSERT( std::all_of( mManifolds, mManifolds + mManifoldCount, []( const RkManifold& Manifold ) { return Manifold.PointCount > 0; } ) );
	return mManifoldCount > 0;
	}


//--------------------------------------------------------------------------------------------------
bool RkContact::IsStatic() const
	{
	return mBody1->GetType() != RK_DYNAMIC_BODY || mBody2->GetType() != RK_DYNAMIC_BODY;
	}


//--------------------------------------------------------------------------------------------------
void* RkContact::GetUserData() const
	{
	return mUserData;
	}


//--------------------------------------------------------------------------------------------------
void RkContact::SetUserData( void* UserData ) const
	{
	mUserData = UserData;
	}


//--------------------------------------------------------------------------------------------------
bool RkContact::IsActive() const
	{
	return CollisionIndex >= 0;
	}


//--------------------------------------------------------------------------------------------------
void RkContact::SetCollideFlag( uint32 Flag )
	{
	mCollideFlags |= Flag;
	}


//--------------------------------------------------------------------------------------------------
uint32 RkContact::GetCollideFlags() const
	{
	return mCollideFlags;
	}


//--------------------------------------------------------------------------------------------------
bool RkContact::TestCollideFlag( uint32 Flag ) const
	{
	return ( mCollideFlags & Flag ) == Flag;
	}


//--------------------------------------------------------------------------------------------------
void RkContact::ClearCollideFlags()
	{
	mCollideFlags = 0;
	}


//--------------------------------------------------------------------------------------------------
RkIsland* RkContact::GetIsland() const
	{
	return mIsland;
	}


//--------------------------------------------------------------------------------------------------
void RkContact::SetIsland( RkIsland* Island )
	{
	mIsland = Island;
	}


//--------------------------------------------------------------------------------------------------
int RkContact::GetIslandIndex() const
	{
	return mIslandIndex;
	}


//--------------------------------------------------------------------------------------------------
void RkContact::SetIslandIndex( int IslandIndex )
	{
	RK_ASSERT( IslandIndex < 0 || mIsland->GetContact( IslandIndex ) == this );
	mIslandIndex = IslandIndex;
	}


//--------------------------------------------------------------------------------------------------
void RkContact::LoadConstraints( RkContactConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep )
	{
	// Load velocities
	int BodyIndex1 = mBody1->SolverIndex + 1;
	RK_ASSERT( BodyIndex1 > 0 || BodyIndex1 == 0 && mBody1->GetType() == RK_STATIC_BODY );
	int BodyIndex2 = mBody2->SolverIndex + 1;
	RK_ASSERT( BodyIndex2 > 0 || BodyIndex2 == 0 && mBody2->GetType() == RK_STATIC_BODY );

	RkVector3 V1 = BodyBuffer[ BodyIndex1 ].LinearVelocity;
	RkVector3 W1 = BodyBuffer[ BodyIndex1 ].AngularVelocity;
	RkVector3 V2 = BodyBuffer[ BodyIndex2 ].LinearVelocity;
	RkVector3 W2 = BodyBuffer[ BodyIndex2 ].AngularVelocity;

	// Resolve materials
	float Friction = rkSqrt( mShape1->GetFriction() * mShape2->GetFriction() );
	float ConvexRadius1 = mShape1->GetType() < RK_HULL_SHAPE ? 0.0f : RK_CONVEX_RADIUS;
	float ConvexRadius2 = mShape2->GetType() < RK_HULL_SHAPE ? 0.0f : RK_CONVEX_RADIUS;

	// Load contact constraints
	for ( int ManifoldIndex = 0; ManifoldIndex < mManifoldCount; ++ManifoldIndex )
		{
		// Fetch next manifold 
		const RkManifold& Manifold = mManifolds[ ManifoldIndex ];
		RK_ASSERT( !Manifold.Empty() );

		// Fetch contact constraint 
		RkContactConstraint* Constraint = ConstraintBuffer + ManifoldIndex;

		// Fill contact header
		Constraint->BodyIndex1 = BodyIndex1;
		Constraint->BodyIndex2 = BodyIndex2;
		Constraint->Friction = Friction;

		// Fill and warm-start non-penetration constraints
		float TotalLinearImpulse = 0.0f;
		float TotalAngularImpulse = 0.0f;

		Constraint->PointCount = Manifold.PointCount;
		Constraint->Compliance = Compliance;
		for ( int PointIndex = 0; PointIndex < Manifold.PointCount; ++PointIndex )
			{
			const RkManifoldPoint& ManifoldPoint = Manifold.Points[ PointIndex ];
			RK_ASSERT( ManifoldPoint.Impulse >= 0.0f );

			RkVector3 Position1 = mBody1->TransformPointToWorld( ManifoldPoint.LocalPosition1 );
			RkVector3 R1 = Position1 - mBody1->GetMassCenter();
			RkVector3 R1_x_N = rkCross( R1, Manifold.Normal );
			RkVector3 Position2 = mBody2->TransformPointToWorld( ManifoldPoint.LocalPosition2 );
			RkVector3 R2 = Position2 - mBody2->GetMassCenter();
			RkVector3 R2_x_N = rkCross( R2, Manifold.Normal );

			float Separation = rkDot( Position2 - Position1, Manifold.Normal ) - ConvexRadius2 - ConvexRadius1;
			float Bias = rkMin( Separation + RK_LINEAR_SLOP, 0.0f );
			
			rkCreateConstraint( Constraint->NonPenetrationConstraints[ PointIndex ], mBody1, R1, mBody2, R2, Manifold.Normal, Bias, ManifoldPoint.Impulse );
			rkWarmstartConstraint( Constraint->NonPenetrationConstraints[ PointIndex ], V1, W1, V2, W2 );

			float LeverArm = rkDistance( Manifold.Center, Position1 );
			Constraint->LeverArms[ PointIndex ] = LeverArm;

			TotalLinearImpulse += ManifoldPoint.Impulse;
			TotalAngularImpulse += LeverArm * ManifoldPoint.Impulse;
			}

		// Fill and warm-start tangent constraint 
		RkVector3 R1 = Manifold.Center - mBody1->GetMassCenter();
		RkVector3 R2 = Manifold.Center - mBody2->GetMassCenter();

		float MaxLinearLambda = Friction * TotalLinearImpulse;
		RkVector3 Tangent[] = { Manifold.Tangent1, Manifold.Tangent2 };
		rkCreateConstraint( Constraint->LinearFrictionConstraint, mBody1, R1, mBody2, R2, Tangent, Manifold.LinearFrictionImpulse, MaxLinearLambda );
		rkWarmstartConstraint( Constraint->LinearFrictionConstraint, V1, W1, V2, W2 );
		
		// Fill and warm-start angular friction constraint 
		float MaxAngularLambda = Friction * TotalAngularImpulse;
		rkCreateConstraint( Constraint->AngularFrictionConstraint, mBody1, mBody2, Manifold.Normal, Manifold.AngularFrictionImpulse, MaxAngularLambda );
		rkWarmstartConstraint( Constraint->AngularFrictionConstraint, W1, W2 );
		}

	// Store velocities
	BodyBuffer[ BodyIndex1 ].LinearVelocity = V1;
	BodyBuffer[ BodyIndex1 ].AngularVelocity = W1;
	BodyBuffer[ BodyIndex2 ].LinearVelocity = V2;
	BodyBuffer[ BodyIndex2 ].AngularVelocity = W2;
	}


//--------------------------------------------------------------------------------------------------
void RkContact::SolveConstraints( RkContactConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer )
	{
	// Fetch constraint buffer
	for ( int ManifoldIndex = 0; ManifoldIndex < mManifoldCount; ++ManifoldIndex )
		{
		// Fetch next manifold 
		const RkManifold& Manifold = mManifolds[ ManifoldIndex ];
		RK_ASSERT( !Manifold.Empty() );

		// Fetch contact constraint 
		RkContactConstraint* Constraint = ConstraintBuffer + ManifoldIndex;

		// Load body indices
		int BodyIndex1 = Constraint->BodyIndex1;
		int BodyIndex2 = Constraint->BodyIndex2;

		// Load velocities
		RkVector3 V1 = BodyBuffer[ BodyIndex1 ].LinearVelocity;
		RkVector3 W1 = BodyBuffer[ BodyIndex1 ].AngularVelocity;
		RkVector3 V2 = BodyBuffer[ BodyIndex2 ].LinearVelocity;
		RkVector3 W2 = BodyBuffer[ BodyIndex2 ].AngularVelocity;

		// Solve linear friction constraint
		RkAngularFrictionConstraint& AngularFrictionConstraint = Constraint->AngularFrictionConstraint;
		rkSolveConstraint( AngularFrictionConstraint, W1, W2 );

		// Solve angular friction constraints
		RkLinearFrictionConstraint& LinearFrictionConstraint = Constraint->LinearFrictionConstraint;
		rkSolveConstraint( LinearFrictionConstraint, V1, W1, V2, W2 );

		// Solve non-penetration constraints and accumulate impulse for max friction impulses
		float TotalLinearImpulse = 0.0f;
		float TotalAngularImpulse = 0.0f;

		RkCompliance Compliance = Constraint->Compliance;
		for ( int Index = 0; Index < Constraint->PointCount; ++Index )
			{
			RkNonPenetrationConstraint& NonPenetrationConstraint = Constraint->NonPenetrationConstraints[ Index ];
			rkSolveConstraint( NonPenetrationConstraint, V1, W1, V2, W2, Compliance );

			TotalLinearImpulse += NonPenetrationConstraint.Lambda;
			TotalAngularImpulse += Constraint->LeverArms[ Index ] * NonPenetrationConstraint.Lambda;
			}

		// Update max friction impulses (Coulomb friction)
		LinearFrictionConstraint.MaxLambda = Constraint->Friction * TotalLinearImpulse;
		AngularFrictionConstraint.MaxLambda = Constraint->Friction * TotalAngularImpulse;

		// Store velocities
		BodyBuffer[ BodyIndex1 ].LinearVelocity = V1;
		BodyBuffer[ BodyIndex1 ].AngularVelocity = W1;
		BodyBuffer[ BodyIndex2 ].LinearVelocity = V2;
		BodyBuffer[ BodyIndex2 ].AngularVelocity = W2;
		}
	}


//--------------------------------------------------------------------------------------------------
void RkContact::SaveConstraints( RkContactConstraint* ConstraintBuffer )
	{
	for ( int ManifoldIndex = 0; ManifoldIndex < mManifoldCount; ++ManifoldIndex )
		{
		// Fetch next manifold 
		RkManifold& Manifold = mManifolds[ ManifoldIndex ];
		RK_ASSERT( !Manifold.Empty() );

		// Fetch contact constraint 
		RkContactConstraint* Constraint = ConstraintBuffer + ManifoldIndex;
		
		// Save normal impulses
		for ( int PointIndex = 0; PointIndex < Constraint->PointCount; ++PointIndex )
			{
			RK_ASSERT( Constraint->NonPenetrationConstraints[ PointIndex ].Lambda >= 0.0f );
			Manifold.Points[ PointIndex ].Impulse = Constraint->NonPenetrationConstraints[ PointIndex ].Lambda;
			}

		// Save friction impulses
		Manifold.LinearFrictionImpulse = Constraint->LinearFrictionConstraint.Lambda;
		Manifold.AngularFrictionImpulse = Constraint->AngularFrictionConstraint.Lambda;
		}
	}


//--------------------------------------------------------------------------------------------------
// RkContactMap
//--------------------------------------------------------------------------------------------------
RkContactMap::RkContactMap()
	{
	RkContactMapImpl_init( &mImpl );
	}


//--------------------------------------------------------------------------------------------------
RkContactMap::~RkContactMap()
	{
	RkContactMapImpl_cleanup( &mImpl );
	}


//--------------------------------------------------------------------------------------------------
int RkContactMap::Size() const
	{
	return static_cast< int >( RkContactMapImpl_size( &mImpl ) );
	}


//--------------------------------------------------------------------------------------------------
void RkContactMap::Clear()
	{
	RkContactMapImpl_clear( &mImpl );
	}


//--------------------------------------------------------------------------------------------------
void RkContactMap::Insert( const RkShapePair& Pair, RkContact* Contact )
	{
	RkContactMapImpl_insert( &mImpl, Pair, Contact );
	}


//--------------------------------------------------------------------------------------------------
bool RkContactMap::Remove( const RkShapePair& Pair )
	{
	return RkContactMapImpl_erase( &mImpl, Pair );
	}


//--------------------------------------------------------------------------------------------------
bool RkContactMap::Contains( const RkShapePair& Pair ) const
	{
	RkContactMapImpl_itr Iterator = RkContactMapImpl_get( &mImpl, Pair );
	return !RkContactMapImpl_is_end( Iterator );
	}


//--------------------------------------------------------------------------------------------------
RkContact* RkContactMap::Find( RkShapePair& Pair )
	{
	RkContactMapImpl_itr Iterator = RkContactMapImpl_get( &mImpl, Pair );
	return !RkContactMapImpl_is_end( Iterator ) ? Iterator.data->val : nullptr;
	}
