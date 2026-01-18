//--------------------------------------------------------------------------------------------------
// physxplugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "physxplugin.h"

// ImGUI
#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>
#include <implot_internal.h>

// Windows
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>


//--------------------------------------------------------------------------------------------------
// VsPhysXEdge
//--------------------------------------------------------------------------------------------------
struct VsPhysXEdge
	{
	int Index1;
	int Index2;
	};


//--------------------------------------------------------------------------------------------------
static inline bool operator<( VsPhysXEdge Lhs, VsPhysXEdge Rhs )
	{
	return ( Lhs.Index1 == Rhs.Index1 ? Lhs.Index2 < Rhs.Index2 : Lhs.Index1 < Rhs.Index1 );
	}


//--------------------------------------------------------------------------------------------------
static inline bool operator==( VsPhysXEdge Lhs, VsPhysXEdge Rhs )
	{
	return Lhs.Index1 == Rhs.Index1 && Lhs.Index2 == Rhs.Index2;
	}


//--------------------------------------------------------------------------------------------------
// VsPhysXHull
//--------------------------------------------------------------------------------------------------
VsPhysXHull::VsPhysXHull( PxConvexMesh* ConvexMesh )
	{
	VS_ASSERT( ConvexMesh );
	mNative = ConvexMesh;

	int PolygonCount = ConvexMesh->getNbPolygons();
	const PxU8* IndexBuffer = ConvexMesh->getIndexBuffer();
	const PxVec3* VertexBuffer = ConvexMesh->getVertices();
	for ( int PolygonIndex = 0; PolygonIndex < PolygonCount; ++PolygonIndex )
		{
		PxHullPolygon Polygon;
		ConvexMesh->getPolygonData( PolygonIndex, Polygon );
		
		int IndexCount = Polygon.mNbVerts;
		const PxU8* Indices = IndexBuffer + Polygon.mIndexBase;
		PxVec3 FaceNormal( Polygon.mPlane[ 0 ], Polygon.mPlane[ 1 ], Polygon.mPlane[ 2 ] );

		int PivotIndex = Indices[ 0 ];
		PxVec3 Pivot = VertexBuffer[ PivotIndex ];
		for ( int Index = 1; Index < IndexCount - 1; ++Index )
			{
			mVertexPositions.push_back( VsVector3( Pivot.x, Pivot.y, Pivot.z ) );
			mVertexNormals.push_back( VsVector3( FaceNormal.x, FaceNormal.y, FaceNormal.z ) );

			int VertexIndex1 = Indices[ Index + 0 ];
			PxVec3 Vertex1 = VertexBuffer[ VertexIndex1 ];
			mVertexPositions.push_back( VsVector3( Vertex1.x, Vertex1.y, Vertex1.z ) );
			mVertexNormals.push_back( VsVector3( FaceNormal.x, FaceNormal.y, FaceNormal.z ) );

			int VertexIndex2 = Indices[ Index + 1 ];
			PxVec3 Vertex2 = VertexBuffer[ VertexIndex2 ];
			mVertexPositions.push_back( VsVector3( Vertex2.x, Vertex2.y, Vertex2.z ) );
			mVertexNormals.push_back( VsVector3( FaceNormal.x, FaceNormal.y, FaceNormal.z ) );
			}
		}

	// Extract (unique) edges
	std::vector< VsPhysXEdge > Edges;
	for ( int FaceIndex = 0; FaceIndex < PolygonCount; ++FaceIndex )
		{
		for ( int PolygonIndex = 0; PolygonIndex < PolygonCount; ++PolygonIndex )
			{
			PxHullPolygon Polygon;
			ConvexMesh->getPolygonData( PolygonIndex, Polygon );

			int IndexCount = Polygon.mNbVerts;
			const PxU8* Indices = IndexBuffer + Polygon.mIndexBase;

			int VertexIndex1 = Indices[ IndexCount - 1 ];
			for ( int Index = 0; Index < IndexCount; ++Index )
				{
				int VertexIndex2 = Indices[ Index ];

				VsPhysXEdge Edge;
				Edge.Index1 = std::min( VertexIndex1, VertexIndex2 );
				Edge.Index2 = std::max( VertexIndex1, VertexIndex2 );
				Edges.push_back( Edge );

				VertexIndex1 = VertexIndex2;
				}
			}
		}

	std::sort( Edges.begin(), Edges.end() );
	Edges.resize( std::unique( Edges.begin(), Edges.end() ) - Edges.begin() );
	for ( VsPhysXEdge Edge : Edges )
		{
		PxVec3 Vertex1 = VertexBuffer[ Edge.Index1 ];
		mEdgePositions.push_back( VsVector3( Vertex1.x, Vertex1.y, Vertex1.z ) );
		PxVec3 Vertex2 = VertexBuffer[ Edge.Index2 ];
		mEdgePositions.push_back( VsVector3( Vertex2.x, Vertex2.y, Vertex2.z ) );
		}
	}


//--------------------------------------------------------------------------------------------------
VsPhysXHull::~VsPhysXHull()
	{
	VS_ASSERT( mNative->getReferenceCount() == 1 );
	PX_RELEASE( mNative );
	}


//--------------------------------------------------------------------------------------------------
int VsPhysXHull::GetVertexCount() const
	{
	return static_cast<int>( mVertexPositions.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsPhysXHull::GetVertexPositions() const
	{
	return mVertexPositions.data();
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsPhysXHull::GetVertexNormals() const
	{
	return mVertexNormals.data();
	}


//--------------------------------------------------------------------------------------------------
int VsPhysXHull::GetEdgeCount() const
	{
	return static_cast<int>( mEdgePositions.size() / 2 );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsPhysXHull::GetEdgePositions() const
	{
	return mEdgePositions.data();
	}


//--------------------------------------------------------------------------------------------------
PxConvexMesh* VsPhysXHull::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsPhysXHullShape
//--------------------------------------------------------------------------------------------------
VsPhysXHullShape::VsPhysXHullShape( VsPhysXBody* Body, const VsPhysXHull* Hull )
	: mBody( Body )
	, mHull( Hull )
	{
	VsPhysXWorld* World = static_cast< VsPhysXWorld* >( Body->GetWorld() );
	VsPhysXPlugin* Plugin = static_cast< VsPhysXPlugin* >( World->GetPlugin() );

	PxPhysics* Physics = Plugin->GetPhysics();
	mMaterial = Physics->createMaterial( 0.6f, 0.6f, 0.0f );
	mMaterial->setFrictionCombineMode( PxCombineMode::eAVERAGE );
	mMaterial->setRestitutionCombineMode( PxCombineMode::eMAX );
	VS_ASSERT( mMaterial );

	PxRigidActor* RigidActor = Body->GetNative();
	mNative = PxRigidActorExt::createExclusiveShape( *RigidActor, PxConvexMeshGeometry( Hull->GetNative() ), *mMaterial );
	if ( RigidActor->getType() == PxActorType::eRIGID_DYNAMIC )
		{
		PxRigidDynamic* RigidDynamic = static_cast< PxRigidDynamic* >( RigidActor );
		if ( !( RigidDynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC ) )
			{
			PxRigidBodyExt::updateMassAndInertia( *RigidDynamic, 1000.0f );
			}
		}
	}


//--------------------------------------------------------------------------------------------------
VsPhysXHullShape::~VsPhysXHullShape()
	{
	PX_RELEASE( mNative );
	PX_RELEASE( mMaterial );
	}


//--------------------------------------------------------------------------------------------------
VsShapeType VsPhysXHullShape::GetType() const
	{
	return VS_HULL_SHAPE;
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsPhysXHullShape::GetBody() const
	{
	return mBody;
	}


//--------------------------------------------------------------------------------------------------
VsColor VsPhysXHullShape::GetColor() const
	{
	return mColor;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXHullShape::SetColor( const VsColor& Color )
	{
	mColor = Color;
	}


//--------------------------------------------------------------------------------------------------
const IVsHull* VsPhysXHullShape::GetHull() const
	{
	return mHull;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXHullShape::SetFriction( float Friction )
	{
	mMaterial->setStaticFriction( Friction );
	mMaterial->setDynamicFriction( Friction );
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXHullShape::SetRestitution( float Restitution )
	{
	mMaterial->setRestitution( Restitution );
	}


//--------------------------------------------------------------------------------------------------
PxShape* VsPhysXHullShape::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsPhysXMesh
//--------------------------------------------------------------------------------------------------
VsPhysXMesh::VsPhysXMesh()
	{

	}


//--------------------------------------------------------------------------------------------------
VsPhysXMesh::~VsPhysXMesh()
	{

	}


//--------------------------------------------------------------------------------------------------
int VsPhysXMesh::GetVertexCount() const
	{
	return static_cast< int >( mVertexPositions.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsPhysXMesh::GetVertexPositions() const
	{
	return mVertexPositions.data();
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsPhysXMesh::GetVertexNormals() const
	{
	return mVertexNormals.data();
	}


//--------------------------------------------------------------------------------------------------
// VsPhysXBody
//--------------------------------------------------------------------------------------------------
VsPhysXBody::VsPhysXBody( VsPhysXWorld* World, VsBodyType Type )
	: mWorld( World )
	{
	VsPhysXPlugin* Plugin = static_cast< VsPhysXPlugin* >( World->GetPlugin() );
	PxPhysics* Physics = Plugin->GetPhysics();
	if ( Type == VS_STATIC_BODY )
		{
		PxRigidStatic* RigidStatic = Physics->createRigidStatic( PxTransform( PxIdentity ) );
		mNative = RigidStatic;
		}
	else
		{
		PxRigidDynamic* RigidDynamic = Physics->createRigidDynamic( PxTransform( PxIdentity ) );
		RigidDynamic->setRigidBodyFlag( PxRigidBodyFlag::eKINEMATIC, Type == VS_KEYFRAMED_BODY ? true : false );
		RigidDynamic->setRigidBodyFlag( PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES, true );
		RigidDynamic->setLinearDamping( 0.0f );
		RigidDynamic->setAngularDamping( 0.0f );
		if ( !mWorld->IsAutoSleepingEnabled() )
			{
			RigidDynamic->setWakeCounter( PX_MAX_F32 );
			}

		mNative = RigidDynamic;
		}

	VS_ASSERT( mNative );
	PxScene* Scene = World->GetNative();
	Scene->addActor( *mNative );
	}


//--------------------------------------------------------------------------------------------------
VsPhysXBody::~VsPhysXBody()
	{
	while ( !mShapes.empty() )
		{
		IVsShape* Shape = mShapes.back();
		mWorld->NotifyShapeRemoved( this, Shape );
		mShapes.pop_back();
		DeleteShape( Shape );
		}

	PX_RELEASE( mNative );
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsPhysXBody::GetWorld() const
	{
	return mWorld;
	}


//--------------------------------------------------------------------------------------------------
VsBodyType VsPhysXBody::GetType() const
	{
	if ( mNative->getType() == PxActorType::eRIGID_STATIC )
		{
		return VS_STATIC_BODY;
		}

	VS_ASSERT( mNative->getType() == PxActorType::eRIGID_DYNAMIC );
	PxRigidDynamic* RigidDynamic = static_cast< PxRigidDynamic*>( mNative );
	return RigidDynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC ? VS_KEYFRAMED_BODY : VS_DYNAMIC_BODY;
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsPhysXBody::GetPosition() const
	{
	PxTransform Pose = mNative->getGlobalPose();
	return { Pose.p.x, Pose.p.y, Pose.p.z };
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXBody::SetPosition( const VsVector3& Position )
	{
	PxTransform Pose = mNative->getGlobalPose();
	Pose.p = PxVec3( Position.X, Position.Y, Position.Z );
	mNative->setGlobalPose( Pose );
	}


//--------------------------------------------------------------------------------------------------
VsQuaternion VsPhysXBody::GetOrientation() const
	{
	PxTransform Pose = mNative->getGlobalPose();
	return { Pose.q.x, Pose.q.y, Pose.q.z, Pose.q.w };
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXBody::SetOrientation( const VsQuaternion& Orientation )
	{
	PxTransform Pose = mNative->getGlobalPose();
	Pose.q = PxQuat( Orientation.X, Orientation.Y, Orientation.Z, Orientation.W );
	mNative->setGlobalPose( Pose ); 
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsPhysXBody::GetLinearVelocity() const
	{
	if ( mNative->getType() == PxActorType::eRIGID_STATIC )
		{
		return { 0.0f, 0.0f, 0.0f };
		}

	const PxRigidDynamic* RigidDynamic = static_cast< const PxRigidDynamic* >( mNative );
	PxVec3 LinearVelocity = RigidDynamic->getLinearVelocity();
	return { LinearVelocity.x, LinearVelocity.y, LinearVelocity.z };
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXBody::SetLinearVelocity( const VsVector3& LinearVelocity )
	{
	if ( mNative->getType() == PxActorType::eRIGID_STATIC )
		{
		return;
		}

	PxRigidDynamic* RigidDynamic = static_cast< PxRigidDynamic* >( mNative );
	RigidDynamic->setLinearVelocity( { LinearVelocity.X, LinearVelocity.Y, LinearVelocity.Z }, true );
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsPhysXBody::GetAngularVelocity() const
	{
	if ( mNative->getType() == PxActorType::eRIGID_STATIC )
		{
		return { 0.0f, 0.0f, 0.0f };
		}

	const PxRigidDynamic* RigidDynamic = static_cast< const PxRigidDynamic* >( mNative );
	PxVec3 AngularVelocity = RigidDynamic->getAngularVelocity();
	return { AngularVelocity.x, AngularVelocity.y, AngularVelocity.z };
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXBody::SetAngularVelocity( const VsVector3& AngularVelocity )
	{
	if ( mNative->getType() == PxActorType::eRIGID_STATIC )
		{
		return;
		}

	PxRigidDynamic* RigidDynamic = static_cast< PxRigidDynamic* > ( mNative );
	RigidDynamic->setAngularVelocity( { AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z }, true );
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXBody::SetVelocityFromKeyframe( const VsFrame& Keyframe, float )
	{
	if ( mNative->getType() == PxActorType::eRIGID_STATIC )
		{
		return;
		}

	PxTransform Destination;
	Destination.p = { Keyframe.Origin.X, Keyframe.Origin.Y, Keyframe.Origin.Z };
	Destination.q = { Keyframe.Basis.X, Keyframe.Basis.Y, Keyframe.Basis.Z, Keyframe.Basis.W };

	PxRigidDynamic* RigidDynamic = static_cast< PxRigidDynamic* > ( mNative );
	RigidDynamic->setKinematicTarget( Destination );
	}


//--------------------------------------------------------------------------------------------------
float VsPhysXBody::GetFriction() const
	{
	return mFriction;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXBody::SetFriction( float Friction )
	{
	if ( mFriction != Friction )
		{
		mFriction = Friction;

		// Propagate to shapes
		for ( IVsShape* Shape : mShapes )
			{
			switch ( Shape->GetType() )
				{
				case VS_HULL_SHAPE:
					static_cast< VsPhysXHullShape* >( Shape )->SetFriction( Friction );
					break;

				default:
					VS_ASSERT( 0 );
					break;
				}
			}
		}
	}


//--------------------------------------------------------------------------------------------------
float VsPhysXBody::GetRestitution() const
	{
	return mRestitution;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXBody::SetRestitution( float Restitution )
	{
	if ( mRestitution != Restitution )
		{
		mRestitution = Restitution;

		// Propagate to shapes
		for ( IVsShape* Shape : mShapes )
			{
			switch ( Shape->GetType() )
				{
				case VS_HULL_SHAPE:
					static_cast< VsPhysXHullShape* >( Shape )->SetRestitution( Restitution );
					break;

				default:
					VS_ASSERT( 0 );
					break;
				}
			}
		}
	}


//--------------------------------------------------------------------------------------------------
IVsSphereShape* VsPhysXBody::CreateSphere( const VsVector3& Center, float Radius )
	{
	// DIRK_TODO: ...
	VS_ASSERT( 0 );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsCapsuleShape* VsPhysXBody::CreateCapulse( const VsVector3& Center1, const VsVector3& Center2, float Radius )
	{
	// DIRK_TODO: ...
	VS_ASSERT( 0 );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsHullShape* VsPhysXBody::CreateHull( const IVsHull* Hull )
	{
	if ( !Hull )
		{
		return nullptr;
		}

	VsPhysXHullShape* Shape = new VsPhysXHullShape( this, static_cast< const VsPhysXHull* >( Hull ) );
	Shape->SetFriction( mFriction );
	Shape->SetRestitution( mRestitution );
	mShapes.push_back( Shape );
	mWorld->NotifyShapeAdded( this, Shape );

	return Shape;
	}


//--------------------------------------------------------------------------------------------------
IVsMeshShape* VsPhysXBody::CreateMesh( const IVsMesh* Mesh )
	{
	if ( !Mesh )
		{
		return nullptr;
		}

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXBody::DestroyShape( IVsShape* Shape )
	{
	if ( !Shape )
		{
		return;
		}

	mWorld->NotifyShapeRemoved( this, Shape );
	std::erase( mShapes, Shape );
	DeleteShape( Shape );
	}


//--------------------------------------------------------------------------------------------------
int VsPhysXBody::GetShapeCount() const
	{
	return static_cast< int >( mShapes.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsShape* VsPhysXBody::GetShape( int ShapeIndex )
	{
	return ( 0 <= ShapeIndex && ShapeIndex < GetShapeCount() ) ? mShapes[ ShapeIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsShape* VsPhysXBody::GetShape( int ShapeIndex ) const
	{
	return ( 0 <= ShapeIndex && ShapeIndex < GetShapeCount() ) ? mShapes[ ShapeIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
bool VsPhysXBody::IsSleeping() const
	{
	if ( mNative->getType() == PxActorType::eRIGID_STATIC )
		{
		return true;
		}

	const PxRigidDynamic* RigidDynamic = static_cast< const PxRigidDynamic* > ( mNative );
	return RigidDynamic->isSleeping();
	}


//--------------------------------------------------------------------------------------------------
PxRigidActor* VsPhysXBody::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsPhysXWorld
//--------------------------------------------------------------------------------------------------
VsPhysXWorld::VsPhysXWorld( VsPhysXPlugin* Plugin )
	: mPlugin( Plugin )
	{
	PxPhysics* Physics = Plugin->GetPhysics();
	PxCpuDispatcher* Dispatcher = Plugin->GetDispatcher();

	PxSceneDesc SceneDesc( Physics->getTolerancesScale() );
	SceneDesc.gravity = PxVec3( 0.0f, -10.0f, 0.0f );
	SceneDesc.cpuDispatcher = Dispatcher;
	SceneDesc.filterShader = PxDefaultSimulationFilterShader;
	SceneDesc.solverType = PxSolverType::ePGS;
	SceneDesc.flags.raise( PxSceneFlag::eENABLE_STABILIZATION );
	VS_ASSERT( SceneDesc.isValid() );
	
	mNative = Physics->createScene( SceneDesc );
	VS_ASSERT( mNative );
	}


//--------------------------------------------------------------------------------------------------
VsPhysXWorld::~VsPhysXWorld()
	{
	while ( !mBodies.empty() )
		{
		VsPhysXBody* Body = mBodies.back();
		NotifyBodyRemoved( Body );
		mBodies.pop_back();
		delete Body;
		}

	PX_RELEASE( mNative );
	}


//--------------------------------------------------------------------------------------------------
IVsPlugin* VsPhysXWorld::GetPlugin() const
	{
	return mPlugin;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::AddListener( IVsWorldListener* Listener )
	{
	// Add 
	if ( !Listener )
		{
		return;
		}

	if ( std::find( mListeners.begin(), mListeners.end(), Listener ) != mListeners.end() )
		{
		return;
		}
	mListeners.push_back( Listener );

	// Sync
	for ( VsPhysXBody* Body : mBodies )
		{
		Listener->OnBodyAdded( Body );
		for ( int ShapeIndex = 0; ShapeIndex < Body->GetShapeCount(); ++ShapeIndex )
			{
			IVsShape* Shape = Body->GetShape( ShapeIndex );
			Listener->OnShapeAdded( Body, Shape );
			}
		}
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::RemoveListener( IVsWorldListener* Listener )
	{
	if ( !Listener )
		{
		return;
		}

	std::erase( mListeners, Listener );
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::NotifyBodyAdded( IVsBody* Body )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnBodyAdded( Body ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::NotifyBodyRemoved( IVsBody* Body )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnBodyRemoved( Body ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::NotifyShapeAdded( IVsBody* Body, IVsShape* Shape )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnShapeAdded( Body, Shape ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::NotifyShapeRemoved( IVsBody* Body, IVsShape* Shape )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnShapeRemoved( Body, Shape ); } );
	}


//--------------------------------------------------------------------------------------------------
VsColor VsPhysXWorld::GetColor() const
	{
	return mColor;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::SetColor( const VsColor& Color )
	{
	mColor = Color;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::SetAutoSleeping( bool Enable )
	{
	if ( mAutoSleeping && !Enable )
		{
		// Disable sleeping
		mAutoSleeping = false;
		for ( VsPhysXBody* Body : mBodies )
			{
			PxRigidActor* Native = Body->GetNative();
			if ( Native->getType() == PxActorType::eRIGID_STATIC )
				{
				continue;
				}
			PxRigidDynamic* RigidDynamic = static_cast< PxRigidDynamic* >( Native );
			if ( RigidDynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC )
				{
				continue;
				}
			
			RigidDynamic->wakeUp();
			RigidDynamic->setWakeCounter( PX_MAX_F32 );
			}
		}
	if ( !mAutoSleeping && Enable )
		{
		// Enable sleeping
		mAutoSleeping = true;
		for ( VsPhysXBody* Body : mBodies )
			{
			PxRigidActor* Native = Body->GetNative();
			if ( Native->getType() == PxActorType::eRIGID_STATIC )
				{
				continue;
				}
			PxRigidDynamic* RigidDynamic = static_cast< PxRigidDynamic* >( Native );
			if ( RigidDynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC )
				{
				continue;
				}

			const float Default = 0.4f;
			RigidDynamic->setWakeCounter( Default );
			}
		}
	}


//--------------------------------------------------------------------------------------------------
bool VsPhysXWorld::IsAutoSleepingEnabled() const
	{
	return mAutoSleeping;
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsPhysXWorld::GetGravity() const
	{
	PxVec3 Gravity = mNative->getGravity();
	return { Gravity.x, Gravity.y, Gravity.z };
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::SetGravity( const VsVector3& Gravity )
	{
	mNative->setGravity( PxVec3( Gravity.X, Gravity.Y, Gravity.Z ) );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsPhysXWorld::CreateBody( VsBodyType Type )
	{
	VsPhysXBody* Body = new VsPhysXBody( this, Type );
	mBodies.push_back( Body );
	NotifyBodyAdded( Body );

	return Body;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::DestroyBody( IVsBody* Body )
	{
	if ( !Body )
		{
		return;
		}

	NotifyBodyRemoved( Body );
	std::erase( mBodies, Body );
	delete static_cast< VsPhysXBody* >( Body );
	}


//--------------------------------------------------------------------------------------------------
int VsPhysXWorld::GetBodyCount() const
	{
	return static_cast< int >( mBodies.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsPhysXWorld::GetBody( int BodyIndex )
	{
	return ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsBody* VsPhysXWorld::GetBody( int BodyIndex ) const
	{
	return ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::Step( float Timestep )
	{
	// DIRK_TODO: Pass scratch buffer...
	mNative->simulate( Timestep );
	mNative->fetchResults( true );
	}


//--------------------------------------------------------------------------------------------------
physx::PxScene* VsPhysXWorld::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsPhysXPlugin
//--------------------------------------------------------------------------------------------------
VsPhysXPlugin::VsPhysXPlugin( ImGuiContext* Context )
	{
	VS_ASSERT( Context );
	ImGui::SetCurrentContext( Context );

	snprintf( mVersion, std::size( mVersion ), "%d.%d.%d", PX_PHYSICS_VERSION_MAJOR, PX_PHYSICS_VERSION_MINOR, PX_PHYSICS_VERSION_BUGFIX );

	fs::path ModulePath = fs::current_path() / "plugins/physx";
	SetDllDirectoryW( ModulePath.c_str() );
	mFoundation = PxCreateFoundation( PX_PHYSICS_VERSION, mAllocator, mErrorCallback );
	mPhysics = PxCreatePhysics( PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale() );
	PxInitExtensions( *mPhysics, NULL );
	mDispatcher = PxDefaultCpuDispatcherCreate( std::min( 8, (int)std::thread::hardware_concurrency() / 2 ) );
	SetDllDirectoryW( NULL );
	}


//--------------------------------------------------------------------------------------------------
VsPhysXPlugin::~VsPhysXPlugin()
	{
	while ( !mWorlds.empty() )
		{
		VsPhysXWorld* World = mWorlds.back();
		mWorlds.pop_back();
		delete World;
		}

	while ( !mMeshes.empty() )
		{
		VsPhysXMesh* Mesh = mMeshes.back();
		mMeshes.pop_back();
		delete Mesh;
		}

	while ( !mHulls.empty() )
		{
		VsPhysXHull* Hull = mHulls.back();
		mHulls.pop_back();
		delete Hull;
		}

	PX_RELEASE( mDispatcher );
	PxCloseExtensions();
	PX_RELEASE( mPhysics );
	PX_RELEASE( mFoundation );

	ImGui::SetCurrentContext( NULL );
	}

//--------------------------------------------------------------------------------------------------
void VsPhysXPlugin::Release()
	{
	delete this;
	}


//--------------------------------------------------------------------------------------------------
const char* VsPhysXPlugin::GetName() const
	{
	return "PhysX";
	}


//--------------------------------------------------------------------------------------------------
const char* VsPhysXPlugin::GetVersion() const
	{
	return mVersion;
	}


//--------------------------------------------------------------------------------------------------
bool VsPhysXPlugin::IsEnabled() const
	{
	return mEnabled;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXPlugin::SetEnabled( bool Enabled )
	{
	mEnabled = Enabled;
	}


//--------------------------------------------------------------------------------------------------
bool VsPhysXPlugin::OnInspectorGUI()
	{
	ImGui::Text( "PhysX %s", mVersion );
	return false;
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsPhysXPlugin::CreateHull( int VertexCount, const VsVector3* Vertices )
	{
	// Create params without cooking interface
	PxTolerancesScale ToleranceScale;
	PxCookingParams CookingParams( ToleranceScale );

	PxConvexMeshDesc ConvexMeshDesc;
	ConvexMeshDesc.points.count = VertexCount;
	ConvexMeshDesc.points.stride = sizeof( VsVector3 );
	ConvexMeshDesc.points.data = Vertices;
	ConvexMeshDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

	if ( PxConvexMesh* ConvexMesh = PxCreateConvexMesh( CookingParams, ConvexMeshDesc, mPhysics->getPhysicsInsertionCallback() ) )
		{
		mHulls.push_back( new VsPhysXHull( ConvexMesh ) );
		return mHulls.back();
		}

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXPlugin::DestroyHull( IVsHull* Hull )
	{
	if ( !Hull )
		{
		return;
		}

	std::erase( mHulls, Hull );
	delete static_cast< VsPhysXHull* >( Hull );
	}


//--------------------------------------------------------------------------------------------------
int VsPhysXPlugin::GetHullCount() const
	{
	return static_cast< int >( mHulls.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsPhysXPlugin::GetHull( int HullIndex )
	{
	return ( 0 <= HullIndex && HullIndex < GetHullCount() ) ? mHulls[ HullIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsHull* VsPhysXPlugin::GetHull( int HullIndex ) const
	{
	return ( 0 <= HullIndex && HullIndex < GetHullCount() ) ? mHulls[ HullIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsPhysXPlugin::CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices )
	{
	// DIRK_TODO: ...
	VS_ASSERT( 0 );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXPlugin::DestroyMesh( IVsMesh* Mesh )
	{
	if ( !Mesh )
		{
		return;
		}

	std::erase( mMeshes, Mesh );
	delete static_cast< VsPhysXMesh* >( Mesh );
	}


//--------------------------------------------------------------------------------------------------
int VsPhysXPlugin::GetMeshCount() const
	{
	return static_cast< int >( mMeshes.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsPhysXPlugin::GetMesh( int MeshIndex )
	{
	return ( 0 <= MeshIndex && MeshIndex < GetMeshCount() ) ? mMeshes[ MeshIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsMesh* VsPhysXPlugin::GetMesh( int MeshIndex ) const
	{
	return ( 0 <= MeshIndex && MeshIndex < GetMeshCount() ) ? mMeshes[ MeshIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsPhysXPlugin::CreateWorld()
	{
	VsPhysXWorld* World = new VsPhysXWorld( this );
	mWorlds.push_back( World );

	return World;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXPlugin::DestroyWorld( IVsWorld* World )
	{
	if ( !World )
		{
		return;
		}

	std::erase( mWorlds, World );
	delete static_cast< VsPhysXWorld* >( World );
	}


//--------------------------------------------------------------------------------------------------
int VsPhysXPlugin::GetWorldCount() const
	{
	return static_cast< int >( mWorlds.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsPhysXPlugin::GetWorld( int WorldIndex )
	{
	return ( 0 <= WorldIndex && WorldIndex < GetWorldCount() ) ? mWorlds[ WorldIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsWorld* VsPhysXPlugin::GetWorld( int WorldIndex ) const
	{
	return ( 0 <= WorldIndex && WorldIndex < GetWorldCount() ) ? mWorlds[ WorldIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
physx::PxFoundation* VsPhysXPlugin::GetFoundation() const
	{
	return mFoundation;
	}


//--------------------------------------------------------------------------------------------------
physx::PxPhysics* VsPhysXPlugin::GetPhysics() const
	{
	return mPhysics;
	}


//--------------------------------------------------------------------------------------------------
physx::PxCpuDispatcher* VsPhysXPlugin::GetDispatcher() const
	{
	return mDispatcher;
	}


// Export
VS_EXPORT_PLUGIN( VsPhysXPlugin );
