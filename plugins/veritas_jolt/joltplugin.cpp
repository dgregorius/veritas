//--------------------------------------------------------------------------------------------------
// joltplugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "joltplugin.h"

// STL includes
#include <cstdarg>
#include <iostream>
#include <thread>

static inline constexpr uint cMaxBodies = 1024;
static inline constexpr uint cNumBodyMutexes = 0;
static inline constexpr uint cMaxBodyPairs = 1024;
static inline constexpr uint cMaxContactConstraints = 1024;


//--------------------------------------------------------------------------------------------------
// TraceImpl
//--------------------------------------------------------------------------------------------------
static void TraceImpl( const char* Format, ... )
	{
	// Format the message
	va_list List;
	va_start( List, Format );
	char Buffer[ 1024 ] = {};
	vsnprintf( Buffer, sizeof( Buffer ), Format, List );
	va_end( List );

	// Print to the TTY
	std::cout << Buffer << std::endl;
	}


//--------------------------------------------------------------------------------------------------
// VsJoltEdge
//--------------------------------------------------------------------------------------------------
struct VsJoltEdge
	{
	uint Index1;
	uint Index2;
	};


//--------------------------------------------------------------------------------------------------
static inline bool operator<( VsJoltEdge Lhs, VsJoltEdge Rhs )
	{
	return ( Lhs.Index1 == Rhs.Index1 ? Lhs.Index2 < Rhs.Index2 : Lhs.Index1 < Rhs.Index1 );
	}


//--------------------------------------------------------------------------------------------------
static inline bool operator==( VsJoltEdge Lhs, VsJoltEdge Rhs )
	{
	return Lhs.Index1 == Rhs.Index1 && Lhs.Index2 == Rhs.Index2;
	}


//--------------------------------------------------------------------------------------------------
// VsJoltHull
//--------------------------------------------------------------------------------------------------
VsJoltHull::VsJoltHull( ShapeRefC Hull )
	{
	VS_ASSERT(Hull->GetSubType() == EShapeSubType::ConvexHull );
	mNative = Hull;

	// Extract faces
	const ConvexHullShape* HullShape = static_cast< const ConvexHullShape* >( Hull.GetPtr() );
	VS_ASSERT( HullShape );

	int FaceCount = HullShape->GetNumFaces();
	const Array<Plane>& Planes = HullShape->GetPlanes();
	for ( int FaceIndex = 0; FaceIndex < FaceCount; ++FaceIndex )
		{
		Plane FacePlane = Planes[ FaceIndex ];
		Vec3 FaceNormal = FacePlane.GetNormal();

		uint VertexIndices[ 64 ];
		int VertexCount = HullShape->GetNumVerticesInFace( FaceIndex );
		HullShape->GetFaceVertices( FaceIndex, std::size( VertexIndices ), VertexIndices );
		
		uint PivotIndex = VertexIndices[ 0 ];
		Vec3 Pivot = HullShape->GetPoint( PivotIndex );
		for ( int VertexIndex = 1; VertexIndex < VertexCount - 1; ++VertexIndex )
			{
			mVertexPositions.push_back( VsVector3( Pivot.GetX(), Pivot.GetY(), Pivot.GetZ() ) );
			mVertexNormals.push_back( VsVector3( FaceNormal.GetX(), FaceNormal.GetY(), FaceNormal.GetZ() ) );
			
			Vec3 Vertex1 = HullShape->GetPoint( VertexIndices[ VertexIndex + 0 ] );
			mVertexPositions.push_back( VsVector3( Vertex1.GetX(), Vertex1.GetY(), Vertex1.GetZ() ) );
			mVertexNormals.push_back( VsVector3( FaceNormal.GetX(), FaceNormal.GetY(), FaceNormal.GetZ() ) );
			
			Vec3 Vertex2 = HullShape->GetPoint( VertexIndices[ VertexIndex + 1 ] );
			mVertexPositions.push_back( VsVector3( Vertex2.GetX(), Vertex2.GetY(), Vertex2.GetZ() ) );
			mVertexNormals.push_back( VsVector3( FaceNormal.GetX(), FaceNormal.GetY(), FaceNormal.GetZ() ) );
			}
		}

	// Extract (unique) edges
	std::vector< VsJoltEdge > Edges;
	for ( int FaceIndex = 0; FaceIndex < FaceCount; ++FaceIndex )
		{
		uint VertexIndices[ 64 ];
		uint VertexCount = HullShape->GetNumVerticesInFace( FaceIndex );
		HullShape->GetFaceVertices( FaceIndex, std::size( VertexIndices ), VertexIndices);

		uint VertexIndex1 = VertexIndices[ VertexCount - 1 ];
		for ( uint Index = 0; Index < VertexCount; ++Index )
			{
			uint VertexIndex2 = VertexIndices[ Index ];

			VsJoltEdge Edge;
			Edge.Index1 = std::min( VertexIndex1, VertexIndex2 );
			Edge.Index2 = std::max( VertexIndex1, VertexIndex2 );
			Edges.push_back( Edge );

			VertexIndex1 = VertexIndex2;
			}
		}

	std::sort( Edges.begin(), Edges.end() );
	Edges.resize( std::unique( Edges.begin(), Edges.end() ) - Edges.begin() );
	for ( VsJoltEdge Edge : Edges )
		{
		Vec3 Vertex1 = HullShape->GetPoint( Edge.Index1 );
		mEdgePositions.push_back( VsVector3( Vertex1.GetX(), Vertex1.GetY(), Vertex1.GetZ() ) );
		Vec3 Vertex2 = HullShape->GetPoint( Edge.Index2 );
		mEdgePositions.push_back( VsVector3( Vertex2.GetX(), Vertex2.GetY(), Vertex2.GetZ() ) );
		}
	}


//--------------------------------------------------------------------------------------------------
int VsJoltHull::GetVertexCount() const
	{
	return static_cast<int>( mVertexPositions.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsJoltHull::GetVertexPositions() const
	{
	return mVertexPositions.data();
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsJoltHull::GetVertexNormals() const
	{
	return mVertexNormals.data();
	}


//--------------------------------------------------------------------------------------------------
int VsJoltHull::GetEdgeCount() const
	{
	return static_cast< int >( mEdgePositions.size() / 2 );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsJoltHull::GetEdgePositions() const
	{
	return mEdgePositions.data();
	}


//--------------------------------------------------------------------------------------------------
JPH::ShapeRefC VsJoltHull::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsJoltHullShape
//--------------------------------------------------------------------------------------------------
VsJoltHullShape::VsJoltHullShape( VsJoltBody* Body, const VsJoltHull* Hull )
	: mBody( Body )
	, mHull( Hull )
	{
	VsJoltWorld* World = static_cast< VsJoltWorld* >( Body->GetWorld() );
	PhysicsSystem& PhysicsSystem = World->GetNative();
	BodyInterface& BodyInterface = PhysicsSystem.GetBodyInterfaceNoLock();

	ShapeRefC Shape = BodyInterface.GetShape( Body->GetNative() );
	VS_ASSERT( Shape->GetSubType() == EShapeSubType::MutableCompound );
	MutableCompoundShape* CompoundShape = static_cast< MutableCompoundShape* >( const_cast< JPH::Shape* >( Shape.GetPtr() ) );
	Vec3 PrevMassCenter = CompoundShape->GetCenterOfMass();

	ShapeRefC HullShape = Hull->GetNative();
	mNative = CompoundShape->AddShape( Vec3::sZero(), Quat::sIdentity(), HullShape );
	CompoundShape->AdjustCenterOfMass();

	BodyInterface.NotifyShapeChanged( Body->GetNative(), PrevMassCenter, true, EActivation::Activate);
	
	// DIRK_TODO: Cache all constraints attached to the parent body
// 	// Notify the constraints that the shape has changed (this could be done more efficient as we know which constraints are affected)
// 	Vec3 delta_com = s->GetCenterOfMass() - PrevMassCenter;
// 	for ( Constraint* c : mConstraints )
// 		c->NotifyShapeChanged( b->GetID(), delta_com );
	}


//--------------------------------------------------------------------------------------------------
VsJoltHullShape::~VsJoltHullShape()
	{
	VsJoltWorld* World = static_cast<VsJoltWorld*>( mBody->GetWorld() );
	PhysicsSystem& PhysicsSystem = World->GetNative();
	BodyInterface& BodyInterface = PhysicsSystem.GetBodyInterfaceNoLock();

	ShapeRefC Shape = BodyInterface.GetShape( mBody->GetNative() );
	VS_ASSERT( Shape->GetSubType() == EShapeSubType::MutableCompound );
	MutableCompoundShape* CompoundShape = static_cast<MutableCompoundShape*>( const_cast<JPH::Shape*>( Shape.GetPtr() ) );
	CompoundShape->RemoveShape( mNative );
	}


//--------------------------------------------------------------------------------------------------
VsShapeType VsJoltHullShape::GetType() const
	{
	return VS_HULL_SHAPE;
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsJoltHullShape::GetBody() const
	{
	return mBody;
	}


//--------------------------------------------------------------------------------------------------
VsColor VsJoltHullShape::GetColor() const
	{
	return mColor;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltHullShape::SetColor( const VsColor& Color )
	{
	mColor = Color;
	}


//--------------------------------------------------------------------------------------------------
const IVsHull* VsJoltHullShape::GetHull() const
	{
	return mHull;
	}


//--------------------------------------------------------------------------------------------------
// VsJoltMesh
//--------------------------------------------------------------------------------------------------
VsJoltMesh::VsJoltMesh()
	{

	}


//--------------------------------------------------------------------------------------------------
VsJoltMesh::~VsJoltMesh()
	{

	}


//--------------------------------------------------------------------------------------------------
int VsJoltMesh::GetVertexCount() const
	{
	return static_cast< int >( mVertexPositions.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsJoltMesh::GetVertexPositions() const
	{
	return mVertexPositions.data();
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsJoltMesh::GetVertexNormals() const
	{
	return mVertexNormals.data();
	}


//--------------------------------------------------------------------------------------------------
// VsJoltBody
//--------------------------------------------------------------------------------------------------
VsJoltBody::VsJoltBody( VsJoltWorld* World, VsBodyType Type )
	: mWorld( World )
	{
	EmptyShapeSettings EmptySettings;
	EmptySettings.SetEmbedded();
	ShapeSettings::ShapeResult EmptyResult = EmptySettings.Create();
	VS_ASSERT( !EmptyResult.HasError() );
	ShapeRefC EmptyShape = EmptyResult.Get();

	MutableCompoundShapeSettings CompountSettings;
	CompountSettings.SetEmbedded();
	CompountSettings.AddShape( Vec3::sZero(), Quat::sIdentity(), EmptyShape );
	ShapeSettings::ShapeResult CompoundResult = CompountSettings.Create();
	VS_ASSERT( !CompoundResult.HasError() );
	ShapeRefC Shape = CompoundResult.Get();

	const EMotionType TypeMap[] = { EMotionType::Static, EMotionType::Kinematic, EMotionType::Dynamic };
	BodyCreationSettings BodySettings( Shape, Vec3::sZero(), Quat::sIdentity(), TypeMap[ Type ], Type == VS_STATIC_BODY ? VsLayers::NON_MOVING : VsLayers::MOVING);
	
	PhysicsSystem& PhysicsSystem = mWorld->GetNative();
	BodyInterface& BodyInterface = PhysicsSystem.GetBodyInterfaceNoLock();
	mNative = BodyInterface.CreateAndAddBody( BodySettings, EActivation::Activate );
	VS_ASSERT( !mNative.IsInvalid() );
	}


//--------------------------------------------------------------------------------------------------
VsJoltBody::~VsJoltBody()
	{
	while ( !mShapes.empty() )
		{
		IVsShape* Shape = mShapes.back();
		mWorld->NotifyShapeRemoved( this, Shape );
		mShapes.pop_back();
		DeleteShape( Shape );
		}

	PhysicsSystem& PhysicsSystem = mWorld->GetNative();
	BodyInterface& BodyInterface = PhysicsSystem.GetBodyInterfaceNoLock();
	BodyInterface.RemoveBody( mNative );
	BodyInterface.DestroyBody( mNative );
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsJoltBody::GetWorld() const
	{
	return mWorld;
	}


//--------------------------------------------------------------------------------------------------
VsBodyType VsJoltBody::GetType() const
	{
	PhysicsSystem& PhysicsSystem = mWorld->GetNative();
	BodyInterface& BodyInterface = PhysicsSystem.GetBodyInterfaceNoLock();
	EMotionType Type = BodyInterface.GetMotionType( mNative );
	VsBodyType TypeMap[] = { VS_STATIC_BODY, VS_KEYFRAMED_BODY, VS_STATIC_BODY };
	return TypeMap[ int( Type ) ];
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsJoltBody::GetPosition() const
	{
	PhysicsSystem& PhysicsSystem = mWorld->GetNative();
	BodyInterface& BodyInterface = PhysicsSystem.GetBodyInterfaceNoLock();
	Vec3 Position = BodyInterface.GetPosition( mNative );
	return { Position.GetX(), Position.GetY(), Position.GetZ() };
	}


//--------------------------------------------------------------------------------------------------
void VsJoltBody::SetPosition( const VsVector3& Position )
	{
	PhysicsSystem& PhysicsSystem = mWorld->GetNative();
	BodyInterface& BodyInterface = PhysicsSystem.GetBodyInterfaceNoLock();
	BodyInterface.SetPosition( mNative, Vec3( Position.X, Position.Y, Position.Z ), EActivation::Activate );
	}


//--------------------------------------------------------------------------------------------------
VsQuaternion VsJoltBody::GetOrientation() const
	{
	PhysicsSystem& PhysicsSystem = mWorld->GetNative();
	BodyInterface& BodyInterface = PhysicsSystem.GetBodyInterfaceNoLock();
	Quat Orientation = BodyInterface.GetRotation( mNative );
	return { Orientation.GetX(), Orientation.GetY(), Orientation.GetZ(), Orientation.GetW() };
	}


//--------------------------------------------------------------------------------------------------
void VsJoltBody::SetOrientation( const VsQuaternion& Orientation )
	{
	PhysicsSystem& PhysicsSystem = mWorld->GetNative();
	BodyInterface& BodyInterface = PhysicsSystem.GetBodyInterfaceNoLock();
	BodyInterface.SetRotation( mNative, Quat( Orientation.X, Orientation.Y, Orientation.Z, Orientation.W ), EActivation::Activate );
	}


//--------------------------------------------------------------------------------------------------
IVsSphereShape* VsJoltBody::CreateSphere( const VsVector3& Center, float Radius )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsCapsuleShape* VsJoltBody::CreateCapulse( const VsVector3& Center1, const VsVector3& Center2, float Radius )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsHullShape* VsJoltBody::CreateHull( const IVsHull* Hull )
	{
	if ( !Hull )
		{
		return nullptr;
		}
	
	VsJoltHullShape* Shape = new VsJoltHullShape( this, static_cast< const VsJoltHull* >( Hull ) );
	mShapes.push_back( Shape );
	mWorld->NotifyShapeAdded( this, Shape );

	return Shape;
	}


//--------------------------------------------------------------------------------------------------
IVsMeshShape* VsJoltBody::CreateMesh( const IVsMesh* Mesh )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltBody::DestroyShape( IVsShape* Shape )
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
int VsJoltBody::GetShapeCount() const
	{
	return static_cast< int >( mShapes.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsShape* VsJoltBody::GetShape( int ShapeIndex )
	{
	return ( 0 <= ShapeIndex && ShapeIndex < GetShapeCount() ) ? mShapes[ ShapeIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsShape* VsJoltBody::GetShape( int ShapeIndex ) const
	{
	return ( 0 <= ShapeIndex && ShapeIndex < GetShapeCount() ) ? mShapes[ ShapeIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
BodyID VsJoltBody::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsJoltWorld
//--------------------------------------------------------------------------------------------------
VsJoltWorld::VsJoltWorld( VsJoltPlugin* Plugin )
	: mPlugin( Plugin )
	{
	mNative.Init( cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, mBroadphaseLayerInterface, mObjectVsBroadphaseLayerFilter, mObjectVsObjectLayerFilter );
	}


//--------------------------------------------------------------------------------------------------
VsJoltWorld::~VsJoltWorld()
	{
	while ( !mBodies.empty() )
		{
		VsJoltBody* Body = mBodies.back();
		NotifyBodyRemoved( Body );
		mBodies.pop_back();
		delete Body;
		}
	}


//--------------------------------------------------------------------------------------------------
IVsPlugin* VsJoltWorld::GetPlugin() const
	{
	return mPlugin;
	}


//--------------------------------------------------------------------------------------------------
VsColor VsJoltWorld::GetColor() const
	{
	return mColor;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::SetColor( const VsColor& Color )
	{
	mColor = Color;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::AddListener( IVsWorldListener* Listener )
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
	for ( VsJoltBody* Body : mBodies )
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
void VsJoltWorld::RemoveListener( IVsWorldListener* Listener )
	{
	if ( !Listener )
		{
		return;
		}

	std::erase( mListeners, Listener );
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::NotifyBodyAdded( IVsBody* Body )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnBodyAdded( Body ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::NotifyBodyRemoved( IVsBody* Body )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnBodyRemoved( Body ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::NotifyShapeAdded( IVsBody* Body, IVsShape* Shape )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnShapeAdded( Body, Shape ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::NotifyShapeRemoved( IVsBody* Body, IVsShape* Shape )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnShapeRemoved( Body, Shape ); } );
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsJoltWorld::GetGravity() const
	{
	Vec3 Gravity = mNative.GetGravity();
	return { Gravity.GetX(), Gravity.GetY(), Gravity.GetZ() };
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::SetGravity( const VsVector3& Gravity )
	{
	mNative.SetGravity( Vec3( Gravity.X, Gravity.Y, Gravity.Z ) );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsJoltWorld::CreateBody( VsBodyType Type )
	{
	VsJoltBody* Body = new VsJoltBody( this, Type );
	mBodies.push_back( Body );
	NotifyBodyAdded( Body );

	return Body;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::DestroyBody( IVsBody* Body )
	{
	if ( !Body )
		{
		return;
		}

	NotifyBodyRemoved( Body );
	std::erase( mBodies, Body );
	delete static_cast< VsJoltBody* >( Body );
	}


//--------------------------------------------------------------------------------------------------
int VsJoltWorld::GetBodyCount() const
	{
	return static_cast< int >( mBodies.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsJoltWorld::GetBody( int BodyIndex )
	{
	return ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsBody* VsJoltWorld::GetBody( int BodyIndex ) const
	{
	return ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::Step( float Timestep )
	{
	mNative.Update( Timestep, 1, mPlugin->GetTempAllocator(), mPlugin->GetThreadPool() );
	}


//--------------------------------------------------------------------------------------------------
PhysicsSystem& VsJoltWorld::GetNative()
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsJoltPlugin
//--------------------------------------------------------------------------------------------------
VsJoltPlugin::VsJoltPlugin()
	{
	Trace = TraceImpl;
	RegisterDefaultAllocator();
	Factory::sInstance = new Factory();
	RegisterTypes();

	mTempAllocator = new TempAllocatorImpl( 64 * 1024 * 1024 );
	mThreadPool = new JobSystemThreadPool( cMaxPhysicsJobs, cMaxPhysicsBarriers, std::min( 8, (int)std::thread::hardware_concurrency() / 2 ) );
	}


//--------------------------------------------------------------------------------------------------
VsJoltPlugin::~VsJoltPlugin()
	{
	while ( !mWorlds.empty() )
		{
		VsJoltWorld* World = mWorlds.back();
		mWorlds.pop_back();
		delete World;
		}

	while ( !mMeshes.empty() )
		{
		VsJoltMesh* Mesh = mMeshes.back();
		mMeshes.pop_back();
		delete Mesh;
		}

	while ( !mHulls.empty() )
		{
		VsJoltHull* Hull = mHulls.back();
		mHulls.pop_back();
		delete Hull;
		}

	delete mThreadPool;
	delete mTempAllocator;

	UnregisterTypes();
	delete Factory::sInstance;
	Factory::sInstance = nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltPlugin::Release()
	{
	delete this;
	}


//--------------------------------------------------------------------------------------------------
const char* VsJoltPlugin::GetName() const
	{
	return "Jolt";
	}


//--------------------------------------------------------------------------------------------------
const char* VsJoltPlugin::GetVersion() const
	{
	return "5.2.0";
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsJoltPlugin::CreateHull( int VertexCount, const VsVector3* Vertices )
	{
	Array< Vec3 > HullPoints( VertexCount );
	for ( int VertexIndex = 0; VertexIndex < VertexCount; ++VertexIndex )
		{
		VsVector3 Vertex = Vertices[ VertexIndex ];
		HullPoints[ VertexIndex ] = Vec3( Vertex.X, Vertex.Y, Vertex.Z );
		}

	ConvexHullShapeSettings HullSettings( HullPoints );
	HullSettings.SetEmbedded();
	ShapeSettings::ShapeResult Result = HullSettings.Create();
	if ( !Result.HasError() )
		{
		mHulls.push_back( new VsJoltHull( Result.Get() ) );
		return mHulls.back();
		}

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltPlugin::DestroyHull( IVsHull* Hull )
	{
	if ( !Hull )
		{
		return;
		}

	std::erase( mHulls, Hull );
	delete static_cast< VsJoltHull* >( Hull );
	}


//--------------------------------------------------------------------------------------------------
int VsJoltPlugin::GetHullCount() const
	{
	return static_cast< int >( mHulls.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsJoltPlugin::GetHull( int HullIndex )
	{
	return ( 0 <= HullIndex && HullIndex < GetHullCount() ) ? mHulls[ HullIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsHull* VsJoltPlugin::GetHull( int HullIndex ) const
	{
	return ( 0 <= HullIndex && HullIndex < GetHullCount() ) ? mHulls[ HullIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsJoltPlugin::CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltPlugin::DestroyMesh( IVsMesh* Mesh )
	{
	// DIRK_TODO: ...
	VS_ASSERT( 0 );
	}


//--------------------------------------------------------------------------------------------------
int VsJoltPlugin::GetMeshCount() const
	{
	return static_cast< int >( mMeshes.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsJoltPlugin::GetMesh( int MeshIndex )
	{
	return ( 0 <= MeshIndex && MeshIndex < GetMeshCount() ) ? mMeshes[ MeshIndex ] : nullptr;
	}



//--------------------------------------------------------------------------------------------------
const IVsMesh* VsJoltPlugin::GetMesh( int MeshIndex ) const
	{
	return ( 0 <= MeshIndex && MeshIndex < GetMeshCount() ) ? mMeshes[ MeshIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsJoltPlugin::CreateWorld()
	{
	VsJoltWorld* World = new VsJoltWorld( this );
	mWorlds.push_back( World );

	return World;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltPlugin::DestroyWorld( IVsWorld* World )
	{
	if ( !World )
		{
		return;
		}

	std::erase( mWorlds, World );
	delete static_cast< VsJoltWorld* >( World );
	}


//--------------------------------------------------------------------------------------------------
int VsJoltPlugin::GetWorldCount() const
	{
	return static_cast<int>( mWorlds.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsJoltPlugin::GetWorld( int WorldIndex )
	{
	return ( 0 <= WorldIndex && WorldIndex < GetWorldCount() ) ? mWorlds[ WorldIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsWorld* VsJoltPlugin::GetWorld( int WorldIndex ) const
	{
	return ( 0 <= WorldIndex && WorldIndex < GetWorldCount() ) ? mWorlds[ WorldIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
JPH::TempAllocatorImpl* VsJoltPlugin::GetTempAllocator() const
	{
	return mTempAllocator;
	}


//--------------------------------------------------------------------------------------------------
JPH::JobSystemThreadPool* VsJoltPlugin::GetThreadPool() const
	{
	return mThreadPool;
	}


// Export
VS_EXPORT_PLUGIN( VsJoltPlugin );