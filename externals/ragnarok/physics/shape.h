//--------------------------------------------------------------------------------------------------
/*
	@file		shape.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/filter.h"
#include "ragnarok/physics/proxy.h"
#include "ragnarok/physics/shapetype.h"

#include "ragnarok/physics/sphere.h"
#include "ragnarok/physics/capsule.h"
#include "ragnarok/physics/hull.h"
#include "ragnarok/physics/bvh.h"

class RkBody;
class RkBroadphase;
struct RkMassProperties;


//--------------------------------------------------------------------------------------------------
// RkShapeCastResult
//--------------------------------------------------------------------------------------------------
struct RkShapeCastResult
	{
	float HitTime = 1.0f;
	RkVector3 HitPoint = RK_VEC3_ZERO;
	RkVector3 HitNormal = RK_VEC3_ZERO;
	int Triangle = -1;

	bool DidHit() const
		{
		return HitTime < 1.0f;
		}
	};


//--------------------------------------------------------------------------------------------------
// RkShape
//--------------------------------------------------------------------------------------------------
class RkShape
	{
	public:
		// Construction / Destruction
		RkShape( RkShapeType Type, RkBody* Body );
		virtual ~RkShape() = default;

		// Type
		RkShapeType GetType() const;

		// Body
		RkBody* GetBody() const;

		// Filter
		RkFilter GetFilter() const;
		void SetFilter( RkFilter Filter );

		// Material
		float GetDensity() const;
		void SetDensity( float Density );
		float GetFriction() const;
		void SetFriction( float Friction );
		float GetRestitution() const;
		void SetRestitution( float Restitution );

		// Casting (assumes ray in local space of parent body) 
		virtual RkShapeCastResult CastRay( const RkVector3& RayStart, const RkVector3& RayEnd, float MaxAlpha ) const = 0;

		// Geometry
		virtual RkBounds3 ComputeBounds( const RkTransform& Transform ) const = 0;
		virtual RkMassProperties ComputeMassProperties() const = 0;
		virtual float GetMinMotionRadius() const = 0;
		virtual float GetMaxMotionRadius( const RkVector3& Center ) const = 0;

		// User data
		RkColor GetUserColor() const;
		void SetUserColor( const RkColor& UserColor ) const;
		void* GetUserData() const;
		void SetUserData( void* UserData ) const;

	ragnarok:
		// Body
		int BodyIndex = -1;

		// Broadphase
		RkProxy GetProxy() const;
		RkBounds3 GetProxyBounds() const;

		void Insert( RkBroadphase* Broadphase, const RkTransform& Transform );
		void Move( RkBroadphase* Broadphase, const RkTransform& Transform );
		void Remove( RkBroadphase* Broadphase );
		
		bool Update( const RkTransform& Transform );
		bool IsDirty() const;
		void SetDirty( bool Dirty );

	protected:
		RkShapeType mType;

		RkBody* mBody;
		
		RkFilter mFilter;
		float mDensity;
		float mFriction;
		float mRestitution;

		RkProxy mProxy;
		RkBounds3 mProxyBounds;
		bool mDirty;

		mutable RkColor mUserColor;
		mutable void* mUserData;
	};


//--------------------------------------------------------------------------------------------------
// RkShapePair
//--------------------------------------------------------------------------------------------------
struct RkShapePair
	{
	RkShapePair() = default;
	RkShapePair( RkShape* Shape1, RkShape* Shape2 )
		: Shape1( rkMin( Shape1, Shape2 ) )
		, Shape2( rkMax( Shape1, Shape2 ) )
		{ 
		}

	const RkShape* Shape1 = nullptr;
	const RkShape* Shape2 = nullptr;
	};

// Verstable adapters
uint64_t rkHashPair( const RkShapePair& ShapePair );
bool rkCmprPair( const RkShapePair& Lhs, const RkShapePair& Rhs );

