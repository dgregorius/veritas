//--------------------------------------------------------------------------------------------------
/**
	@file		plugin.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

// Veritas
#include <veritas/veritas.h>

// Jolt
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
using namespace JPH;

// STL includes
#include <algorithm>
#include <thread>

// Forward 
class VsJoltSphereShape;
class VsJoltCapsuleShape;
class VsJoltHull;
class VsJoltHullShape;
class VsJoltMesh;
class VsJoltMeshShape;
class VsJoltBody;
class VsJoltWorld;
class VsJoltPlugin;


//--------------------------------------------------------------------------------------------------
// VsLayers
//--------------------------------------------------------------------------------------------------
namespace VsLayers
	{
	static constexpr ObjectLayer NON_MOVING = 0;
	static constexpr ObjectLayer MOVING = 1;
	static constexpr ObjectLayer NUM_LAYERS = 2;
	};


//--------------------------------------------------------------------------------------------------
// VsObjectLayerPairFilter
//--------------------------------------------------------------------------------------------------
class VsObjectLayerPairFilter : public ObjectLayerPairFilter
	{
	public:
		virtual bool ShouldCollide( ObjectLayer inObject1, ObjectLayer inObject2 ) const override
			{
			switch ( inObject1 )
				{
				case VsLayers::NON_MOVING:
					return inObject2 == VsLayers::MOVING; // Non moving only collides with moving
				case VsLayers::MOVING:
					return true; // Moving collides with everything
				default:
					JPH_ASSERT( false );
					return false;
				}
			}
	};


//--------------------------------------------------------------------------------------------------
// VsBroadPhaseLayers
//--------------------------------------------------------------------------------------------------
namespace VsBroadPhaseLayers
	{
	static constexpr BroadPhaseLayer NON_MOVING( 0 );
	static constexpr BroadPhaseLayer MOVING( 1 );
	static constexpr uint NUM_LAYERS( 2 );
	};


//--------------------------------------------------------------------------------------------------
// VsLayerInterface
//--------------------------------------------------------------------------------------------------
class VsLayerInterface final : public BroadPhaseLayerInterface
	{
	public:
		VsLayerInterface()
			{
			// Create a mapping table from object to broad phase layer
			mObjectToBroadPhase[ VsLayers::NON_MOVING ] = VsBroadPhaseLayers::NON_MOVING;
			mObjectToBroadPhase[ VsLayers::MOVING ] = VsBroadPhaseLayers::MOVING;
			}

		virtual uint GetNumBroadPhaseLayers() const override
			{
			return VsBroadPhaseLayers::NUM_LAYERS;
			}

		virtual BroadPhaseLayer	GetBroadPhaseLayer( ObjectLayer inLayer ) const override
			{
			JPH_ASSERT( inLayer < VsLayers::NUM_LAYERS );
			return mObjectToBroadPhase[ inLayer ];
			}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
		virtual const char* GetBroadPhaseLayerName( BroadPhaseLayer inLayer ) const override
			{
			switch ( (BroadPhaseLayer::Type)inLayer )
				{
				case (BroadPhaseLayer::Type)VsBroadPhaseLayers::NON_MOVING: return "NON_MOVING";
				case (BroadPhaseLayer::Type)VsBroadPhaseLayers::MOVING: return "MOVING";
				default: JPH_ASSERT( false ); return "INVALID";
				}
			}
#endif 

	private:
		BroadPhaseLayer	mObjectToBroadPhase[ VsLayers::NUM_LAYERS ];
	};


//--------------------------------------------------------------------------------------------------
// VsObjectVsBroadPhaseLayerFilter
//--------------------------------------------------------------------------------------------------
class VsObjectVsBroadPhaseLayerFilter : public ObjectVsBroadPhaseLayerFilter
	{
	public:
		virtual bool ShouldCollide( ObjectLayer inLayer1, BroadPhaseLayer inLayer2 ) const override
			{
			switch ( inLayer1 )
				{
				case VsLayers::NON_MOVING:
					return inLayer2 == VsBroadPhaseLayers::MOVING;
				case VsLayers::MOVING:
					return true;

				default:
					JPH_ASSERT( false );
					return false;
				}
			}
	};


//--------------------------------------------------------------------------------------------------
// VsJoltHull
//--------------------------------------------------------------------------------------------------
class VsJoltHull : public IVsHull
	{
	public:
		// Construction / Destruction
		explicit VsJoltHull( ShapeRefC Hull );
		virtual ~VsJoltHull() = default;

		// IVsHull
		virtual int GetVertexCount() const override;
		virtual const VsVector3* GetVertexPositions() const override;
		virtual const VsVector3* GetVertexNormals() const override;

		virtual int GetEdgeCount() const override;
		virtual const VsVector3* GetEdgePositions() const override;

		ShapeRefC GetNative() const;

	private:
		std::vector< VsVector3 > mVertexPositions;
		std::vector< VsVector3 > mVertexNormals;
		std::vector< VsVector3 > mEdgePositions;
		ShapeRefC mNative;
	};


//--------------------------------------------------------------------------------------------------
// VsJoltHullShape
//--------------------------------------------------------------------------------------------------
class VsJoltHullShape : public IVsHullShape
	{
	public:
		// Construction / Destruction
		explicit VsJoltHullShape( VsJoltBody* Body, const VsJoltHull* Hull );
		virtual ~VsJoltHullShape();

		// IVsShape
		virtual VsShapeType GetType() const override;
		virtual IVsBody* GetBody() const override;
		virtual VsColor GetColor() const override;
		virtual void SetColor( const VsColor& Color ) override;

		// IVsHullShape
		virtual const IVsHull* GetHull() const override;

	private:
		VsJoltBody* mBody = nullptr;
		VsColor mColor = VS_COLOR_TRANSPARENT;
		const VsJoltHull* mHull = nullptr;	
		uint mNative;
	};


//--------------------------------------------------------------------------------------------------
// VsJoltBody
//--------------------------------------------------------------------------------------------------
class VsJoltBody : public IVsBody
	{
	public:
		// Construction / Destruction
		explicit VsJoltBody( VsJoltWorld* World, VsBodyType Type );
		virtual ~VsJoltBody() override;

		// World
		virtual IVsWorld* GetWorld() const override;

		// Type
		virtual VsBodyType GetType() const override;

		// Transform
		virtual VsVector3 GetPosition() const override;
		virtual void SetPosition( const VsVector3& Position ) override;
		virtual VsQuaternion GetOrientation() const override;
		virtual void SetOrientation( const VsQuaternion& Orientation ) override;

		// Shapes
		virtual IVsSphereShape* CreateSphere( const VsVector3& Center, float Radius ) override;
		virtual IVsCapsuleShape* CreateCapulse( const VsVector3& Center1, const VsVector3& Center2, float Radius ) override;
		virtual IVsHullShape* CreateHull( const IVsHull* Hull ) override;
		virtual IVsMeshShape* CreateMesh( const IVsMesh* Mesh ) override;
		virtual void DestroyShape( IVsShape* Shape ) override;

		virtual int GetShapeCount() const override;
		virtual IVsShape* GetShape( int ShapeIndex ) override;
		virtual const IVsShape* GetShape( int ShapeIndex ) const override;

		// Native
		BodyID GetNative() const;

	private:
		VsJoltWorld* mWorld = nullptr;
		std::vector< IVsShape* > mShapes;
		
		BodyID mNative;
	};


//--------------------------------------------------------------------------------------------------
// VsJoltWorld
//--------------------------------------------------------------------------------------------------
class VsJoltWorld : public IVsWorld
	{
	public:
		// Construction / Destruction
		explicit VsJoltWorld( VsJoltPlugin* Plugin );
		virtual ~VsJoltWorld() override;

		// Plugin
		virtual IVsPlugin* GetPlugin() const override;

		// Color
		virtual VsColor GetColor() const override;
		virtual void SetColor( const VsColor& Color ) override;

		// Events
		virtual void AddListener( IVsWorldListener* Listener ) override;
		virtual void RemoveListener( IVsWorldListener* Listener ) override;

		void NotifyBodyAdded( IVsBody* Body );
		void NotifyBodyRemoved( IVsBody* Body );
		void NotifyShapeAdded( IVsBody* Body, IVsShape* Shape );
		void NotifyShapeRemoved( IVsBody* Body, IVsShape* Shape );

		// Gravity
		virtual VsVector3 GetGravity() const override;
		virtual void SetGravity( const VsVector3& Gravity ) override;

		// Bodies
		virtual IVsBody* CreateBody( VsBodyType Type ) override;
		virtual void DestroyBody( IVsBody* Body ) override;
		virtual int GetBodyCount() const override;
		virtual IVsBody* GetBody( int BodyIndex ) override;
		virtual const IVsBody* GetBody( int BodyIndex ) const override;

		// Simulation
		virtual void Step( float Timestep ) override;

		// Native
		PhysicsSystem& GetNative();

	private:
		VsJoltPlugin* mPlugin = nullptr;
		VsColor mColor = { 0.5f, 0.9f, 0.8f, 1.00f };
		std::vector< IVsWorldListener* > mListeners;
		std::vector< VsJoltBody* > mBodies;

		VsLayerInterface mBroadphaseLayerInterface;
		VsObjectVsBroadPhaseLayerFilter mObjectVsBroadphaseLayerFilter;
		VsObjectLayerPairFilter mObjectVsObjectLayerFilter;
		PhysicsSystem mNative;
	};


//--------------------------------------------------------------------------------------------------
// VsJoltPlugin
//--------------------------------------------------------------------------------------------------
class VsJoltPlugin : public IVsPlugin
	{
	public:
		// Construction / Destruction
		VsJoltPlugin();
		virtual ~VsJoltPlugin() override;

		// Module
		virtual void Release() override;
		virtual const char* GetName() const override;
		virtual const char* GetVersion() const override;

		// Hulls
		virtual IVsHull* CreateHull( int VertexCount, const VsVector3* Vertices ) override;
		virtual void DestroyHull( IVsHull* Hull ) override;

		virtual int GetHullCount() const override;
		virtual IVsHull* GetHull( int HullIndex ) override;
		virtual const IVsHull* GetHull( int HullIndex ) const override;

		// Meshes
		virtual IVsMesh* CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices ) override;
		virtual void DestroyMesh( IVsMesh* Mesh ) override;

		virtual int GetMeshCount() const override;
		virtual IVsMesh* GetMesh( int MeshIndex ) override;
		virtual const IVsMesh* GetMesh( int MeshIndex ) const override;

		// Worlds
		virtual IVsWorld* CreateWorld() override;
		virtual void DestroyWorld( IVsWorld* World ) override;

		virtual int GetWorldCount() const override;
		virtual IVsWorld* GetWorld( int WorldIndex ) override;
		virtual const IVsWorld* GetWorld( int WorldIndex ) const override;

		// Shared
		TempAllocatorImpl* GetTempAllocator() const;
		JobSystemThreadPool* GetThreadPool() const;

	private:
		std::vector< IVsHull* > mHulls;
		std::vector< IVsMesh* > mMeshes;
		std::vector< IVsWorld* > mWorlds;

		TempAllocatorImpl* mTempAllocator = nullptr;
		JobSystemThreadPool* mThreadPool = nullptr;
	};