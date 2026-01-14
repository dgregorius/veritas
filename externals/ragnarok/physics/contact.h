//--------------------------------------------------------------------------------------------------
/*
	@file		contact.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/contacttype.h"
#include "ragnarok/physics/gjk.h"
#include "ragnarok/physics/gjksimplex.h"
#include "ragnarok/physics/manifold.h"
#include "ragnarok/physics/sat.h"
#include "ragnarok/physics/shape.h"

#include "ragnarok/common/array.h"
#include "ragnarok/common/math.h"

class RkBody;
class RkContact;
class RkIsland;
class RkShape;
class RkWorld;

struct RkCompliance;
struct RkContactConstraint;
struct RkSolverBody;

// Verstable
#define NAME RkContactMapImpl
#define KEY_TY RkShapePair
#define VAL_TY RkContact*
#define HASH_FN rkHashPair
#define CMPR_FN rkCmprPair
#include "verstable.h"


//--------------------------------------------------------------------------------------------------
// RkContactFlags
//--------------------------------------------------------------------------------------------------
enum RkContactFlags
	{
	RK_DESTROY_CONTACT = 1 << 0,
	RK_CONTACT_BEGIN_TOUCH = 1 << 1,
	RK_CONTACT_END_TOUCH = 1 << 2
	};


//--------------------------------------------------------------------------------------------------
// RkContactEdge
//--------------------------------------------------------------------------------------------------
struct RkContactEdge
	{
	RkBody* Other;
	RkContact* Contact;
	RkContactEdge* Prev;
	RkContactEdge* Next;
	};


//--------------------------------------------------------------------------------------------------
// RkContactCache
//--------------------------------------------------------------------------------------------------
struct RkContactCache
	{
	RkSATCache SAT;
	RkGJKCache GJK;
	};

void rkClearCache( RkContactCache& Cache );


//--------------------------------------------------------------------------------------------------
// RkContact
//--------------------------------------------------------------------------------------------------
class RkContact
	{
	public:
		// Construction / Destruction
		RkContact( RkContactType Type, RkWorld* World, RkShape* Shape1, RkShape* Shape2 );
		virtual ~RkContact();

		// Type
		RkContactType GetType() const;

		// World
		RkWorld* GetWorld() const;

		// Bodies
		RkBody* GetBody1() const;
		RkBody* GetBody2() const;
		RkBody* GetOtherBody( RkBody* Body ) const;

		// Shapes
		RkShape* GetShape1() const;
		RkShape* GetShape2() const;
		RkShape* GetOtherShape( RkShape* Shape ) const;

		// Manifolds
		int GetManifoldCount() const;
		RkManifold& GetManifold( int ManifoldIndex );
		const RkManifold& GetManifold( int ManifoldIndex ) const;

		bool IsTouching() const;
		bool IsStatic() const;

		// Collision
		virtual void Collide() = 0;

		// User data
		void* GetUserData() const;
		void SetUserData( void* UserData ) const;

	ragnarok:
		// World
		int WorldIndex = -1;
		int CollisionIndex = -1;
		int SolverIndex = -1;
		int GraphIndex = -1;
		int ColorIndex = -1;

		// Collision
		bool IsActive() const;

		void SetCollideFlag( uint32 Flag );
		uint32 GetCollideFlags() const;
		bool TestCollideFlag( uint32 Flag ) const;
		void ClearCollideFlags();

		// Island
		RkIsland* GetIsland() const;
		void SetIsland( RkIsland* Island );
		int GetIslandIndex() const;
		void SetIslandIndex( int IslandIndex );
	
		// Solver
		void LoadConstraints( RkContactConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer, const RkCompliance& Compliance, float Timestep );
		void SolveConstraints( RkContactConstraint* ConstraintBuffer, RkSolverBody* BodyBuffer );
		void SaveConstraints( RkContactConstraint* ConstraintBuffer );

	protected:
		RkContactType mType;

		RkWorld* mWorld;
		RkBody* mBody1;
		RkBody* mBody2;
		RkShape* mShape1;
		RkShape* mShape2;

		int mManifoldCount;
		RkManifold mManifolds[ 3 ];
		uint32 mCollideFlags;
		
		RkContactEdge mEdge1;
		RkContactEdge mEdge2;

		RkIsland* mIsland;
		int mIslandIndex;

		mutable void* mUserData;
	};


//--------------------------------------------------------------------------------------------------
// RkContactMap
//--------------------------------------------------------------------------------------------------
class RkContactMap
	{
	public:
		// Construction / Destruction
		RkContactMap();
		~RkContactMap();

		int Size() const;
		void Clear();

		void Insert( const RkShapePair& Pair, RkContact* Contact );
		bool Remove( const RkShapePair& Pair );
		bool Contains( const RkShapePair& Pair ) const;

		RkContact* Find( RkShapePair& Pair );

	private:
		RkContactMapImpl mImpl;
	};

#include "contact.inl"