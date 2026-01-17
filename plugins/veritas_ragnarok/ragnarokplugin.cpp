//--------------------------------------------------------------------------------------------------
// ragnarokplugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "ragnarokplugin.h"

// ImGUI
#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>
#include <implot_internal.h>


//--------------------------------------------------------------------------------------------------
// VsRagnarokHull
//--------------------------------------------------------------------------------------------------
VsRagnarokHull::VsRagnarokHull( RkHull* Hull )
	{
	VS_ASSERT( Hull );
	mNative = Hull;

	// Vertices
	for ( int FaceIndex = 0; FaceIndex < Hull->FaceCount; ++FaceIndex )
		{
		const RkFace* Face = Hull->GetFace( FaceIndex );
		const RkPlane3 FacePlane = Hull->GetPlane( FaceIndex );
		const RkVector3 FaceNormal = FacePlane.Normal;

		const RkHalfEdge* Edge1 = Hull->GetEdge( Face->Edge );
		const RkHalfEdge* Edge2 = Hull->GetEdge( Edge1->Next );
		const RkHalfEdge* Edge3 = Hull->GetEdge( Edge2->Next );
		RK_ASSERT( Edge1 != Edge3 );

		do
			{
			int VertexIndex1 = Edge1->Origin;
			RkVector3 Vertex1 = Hull->GetPosition( VertexIndex1 );
			mVertexPositions.push_back( { Vertex1.X, Vertex1.Y, Vertex1.Z } );
			mVertexNormals.push_back( { FaceNormal.X, FaceNormal.Y, FaceNormal.Z } );

			int VertexIndex2 = Edge2->Origin;
			RkVector3 Vertex2 = Hull->GetPosition( VertexIndex2 );
			mVertexPositions.push_back( { Vertex2.X, Vertex2.Y, Vertex2.Z } );
			mVertexNormals.push_back( { FaceNormal.X, FaceNormal.Y, FaceNormal.Z } );

			int VertexIndex3 = Edge3->Origin;
			RkVector3 Vertex3 = Hull->GetPosition( VertexIndex3 );
			mVertexPositions.push_back( { Vertex3.X, Vertex3.Y, Vertex3.Z } );
			mVertexNormals.push_back( { FaceNormal.X, FaceNormal.Y, FaceNormal.Z } );
			

			Edge2 = Edge3;
			Edge3 = Hull->GetEdge( Edge3->Next );
			}
		while ( Edge1 != Edge3 );
		}

	// Edges (unique)
	int EdgeCount = Hull->EdgeCount / 2;
	mEdgePositions.resize( 2 * EdgeCount );
	for ( int EdgeIndex = 0; EdgeIndex < EdgeCount; EdgeIndex++ )
		{
		const RkHalfEdge* Edge = Hull->GetEdge( 2 * EdgeIndex + 0 );
		RkVector3 EdgeVertex = Hull->GetPosition( Edge->Origin );
		const RkHalfEdge* Twin = Hull->GetEdge( 2 * EdgeIndex + 1 );
		RkVector3 TwinVertex = Hull->GetPosition( Twin->Origin );

		mEdgePositions[ 2 * EdgeIndex + 0 ] = { EdgeVertex.X, EdgeVertex.Y, EdgeVertex.Z };
		mEdgePositions[ 2 * EdgeIndex + 1 ] = { TwinVertex.X, TwinVertex.Y, TwinVertex.Z };
		}
	}


//--------------------------------------------------------------------------------------------------
VsRagnarokHull::~VsRagnarokHull()
	{
	rkDestroyHull( mNative );
	}


//--------------------------------------------------------------------------------------------------
int VsRagnarokHull::GetVertexCount() const
	{
	return static_cast< int >( mVertexPositions.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsRagnarokHull::GetVertexPositions() const
	{
	return mVertexPositions.data();
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsRagnarokHull::GetVertexNormals() const
	{
	return mVertexNormals.data();
	}


//--------------------------------------------------------------------------------------------------
int VsRagnarokHull::GetEdgeCount() const
	{
	return static_cast< int >( mEdgePositions.size() / 2 );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsRagnarokHull::GetEdgePositions() const
	{
	return mEdgePositions.data();
	}


//--------------------------------------------------------------------------------------------------
RkHull* VsRagnarokHull::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsRagnarokHullShape
//--------------------------------------------------------------------------------------------------
VsRagnarokHullShape::VsRagnarokHullShape( VsRagnarokBody* Body, const VsRagnarokHull* Hull )
	: mBody( Body )
	, mHull( Hull )
	{
	mNative = Body->GetNative()->AddHull( Hull->GetNative() );
	}


//--------------------------------------------------------------------------------------------------
VsRagnarokHullShape::~VsRagnarokHullShape()
	{
	mBody->GetNative()->RemoveShape( mNative );
	mNative = nullptr;
	}


//--------------------------------------------------------------------------------------------------
VsShapeType VsRagnarokHullShape::GetType() const
	{
	return VS_HULL_SHAPE;
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsRagnarokHullShape::GetBody() const
	{
	return mBody;
	}


//--------------------------------------------------------------------------------------------------
VsColor VsRagnarokHullShape::GetColor() const
	{
	return mColor;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokHullShape::SetColor( const VsColor& Color )
	{
	mColor = Color;
	}


//--------------------------------------------------------------------------------------------------
const IVsHull* VsRagnarokHullShape::GetHull() const
	{
	return mHull;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokHullShape::SetFriction( float Friction )
	{
	mNative->SetFriction( Friction );
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokHullShape::SetRestitution( float Restitution )
	{
	mNative->SetRestitution( Restitution );
	}


//--------------------------------------------------------------------------------------------------
RkHullShape* VsRagnarokHullShape::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsRagnarokMesh
//--------------------------------------------------------------------------------------------------
VsRagnarokMesh::VsRagnarokMesh()
	{

	}


//--------------------------------------------------------------------------------------------------
VsRagnarokMesh::~VsRagnarokMesh()
	{

	}


//--------------------------------------------------------------------------------------------------
int VsRagnarokMesh::GetVertexCount() const
	{
	return static_cast< int >( mVertexPositions.size() );
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsRagnarokMesh::GetVertexPositions() const
	{
	return mVertexPositions.data();
	}


//--------------------------------------------------------------------------------------------------
const VsVector3* VsRagnarokMesh::GetVertexNormals() const
	{
	return mVertexNormals.data();
	}


//--------------------------------------------------------------------------------------------------
// VsRagnarokBody
//--------------------------------------------------------------------------------------------------
VsRagnarokBody::VsRagnarokBody( VsRagnarokWorld* World, VsBodyType Type )
	: mWorld( World )
	{
	mNative = World->GetNative()->AddBody();

	const RkBodyType TypeMap[] = { RK_STATIC_BODY, RK_KEYFRAMED_BODY, RK_DYNAMIC_BODY };
	mNative->SetType( TypeMap[ Type ] );
	}


//--------------------------------------------------------------------------------------------------
VsRagnarokBody::~VsRagnarokBody()
	{
	while ( !mShapes.empty() )
		{
		IVsShape* Shape = mShapes.back();
		mWorld->NotifyShapeRemoved( this, Shape );
		mShapes.pop_back();
		DeleteShape( Shape );
		}
	RK_ASSERT( mNative->GetShapeCount() == 0 );

	mWorld->GetNative()->RemoveBody( mNative );
	mNative = nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsRagnarokBody::GetWorld() const
	{
	return mWorld;
	}


//--------------------------------------------------------------------------------------------------
VsBodyType VsRagnarokBody::GetType() const
	{
	RkBodyType Type = mNative->GetType();
	VsBodyType TypeMap[] = { VS_STATIC_BODY, VS_KEYFRAMED_BODY, VS_STATIC_BODY };
	return TypeMap[ Type ];
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsRagnarokBody::GetPosition() const
	{
	RkVector3 Position = mNative->GetPosition();
	return { Position.X, Position.Y, Position.Z };
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokBody::SetPosition( const VsVector3& Position )
	{
	mNative->SetPosition( { Position.X, Position.Y, Position.Z } );
	}


//--------------------------------------------------------------------------------------------------
VsQuaternion VsRagnarokBody::GetOrientation() const
	{
	RkQuaternion Orientation = mNative->GetOrientation();
	return { Orientation.X, Orientation.Y, Orientation.Z, Orientation.W };
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokBody::SetOrientation( const VsQuaternion& Orientation )
	{
	mNative->SetOrientation( { Orientation.X, Orientation.Y, Orientation.Z, Orientation.W } );
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsRagnarokBody::GetLinearVelocity() const
	{
	RkVector3 LinearVelocity = mNative->GetLinearVelocity();
	return { LinearVelocity.X, LinearVelocity.Y, LinearVelocity.Z };
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokBody::SetLinearVelocity( const VsVector3& LinearVelocity )
	{
	mNative->SetLinearVelocity( { LinearVelocity.X, LinearVelocity.Y, LinearVelocity.Z } );
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsRagnarokBody::GetAngularVelocity() const
	{
	RkVector3 AngularVelocity = mNative->GetAngularVelocity();
	return { AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z };
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokBody::SetAngularVelocity( const VsVector3& AngularVelocity )
	{
	mNative->SetAngularVelocity( { AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z } );
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokBody::SetVelocityFromKeyframe( const VsFrame& Keyframe, float Timestep )
	{
	RkTransform TargetTransform;
	TargetTransform.Translation = { Keyframe.Origin.X, Keyframe.Origin.Y, Keyframe.Origin.Z };
	TargetTransform.Rotation = { Keyframe.Basis.X, Keyframe.Basis.Y, Keyframe.Basis.Z, Keyframe.Basis.W };

	mNative->SetVelocityFromKeyframe( TargetTransform, Timestep );
	}


//--------------------------------------------------------------------------------------------------
float VsRagnarokBody::GetFriction() const
	{
	return mFriction;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokBody::SetFriction( float Friction )
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
					static_cast< VsRagnarokHullShape* >( Shape )->SetFriction( Friction );
					break;

				default:
					VS_ASSERT( 0 );
					break;
				}
			}
		}
	}


//--------------------------------------------------------------------------------------------------
float VsRagnarokBody::GetRestitution() const
	{
	return mRestitution;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokBody::SetRestitution( float Restitution )
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
					static_cast< VsRagnarokHullShape* >( Shape )->SetRestitution( Restitution );
					break;

				default:
					VS_ASSERT( 0 );
					break;
				}
			}
		}
	}


//--------------------------------------------------------------------------------------------------
IVsSphereShape* VsRagnarokBody::CreateSphere( const VsVector3& Center, float Radius )
	{
	// DIRK_TODO: ...
	VS_ASSERT( 0 );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsCapsuleShape* VsRagnarokBody::CreateCapulse( const VsVector3& Center1, const VsVector3& Center2, float Radius )
	{
	// DIRK_TODO: ...
	VS_ASSERT( 0 );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsHullShape* VsRagnarokBody::CreateHull( const IVsHull* Hull )
	{
	if ( !Hull )
		{
		return nullptr;
		}

	VsRagnarokHullShape* Shape = new VsRagnarokHullShape( this, static_cast< const VsRagnarokHull* >( Hull ) );
	Shape->SetFriction( mFriction );
	Shape->SetRestitution( mRestitution );
	mShapes.push_back( Shape );
	mWorld->NotifyShapeAdded( this, Shape );

	return Shape;
	}


//--------------------------------------------------------------------------------------------------
IVsMeshShape* VsRagnarokBody::CreateMesh( const IVsMesh* Mesh )
	{
	if ( !Mesh )
		{
		return nullptr;
		}

	// DIRK_TODO: ...
	VS_ASSERT( 0 );
	
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokBody::DestroyShape( IVsShape* Shape )
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
int VsRagnarokBody::GetShapeCount() const
	{
	return static_cast< int >( mShapes.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsShape* VsRagnarokBody::GetShape( int ShapeIndex )
	{
	return ( 0 <= ShapeIndex && ShapeIndex < GetShapeCount() ) ? mShapes[ ShapeIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsShape* VsRagnarokBody::GetShape( int ShapeIndex ) const
	{
	return ( 0 <= ShapeIndex && ShapeIndex < GetShapeCount() ) ? mShapes[ ShapeIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkBody* VsRagnarokBody::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsRagnarokWorld
//--------------------------------------------------------------------------------------------------
VsRagnarokWorld::VsRagnarokWorld( VsRagnarokPlugin* Plugin )
	: mPlugin( Plugin )
	{
	tf::Executor* Executor = Plugin->GetExecutor();
	mNative = new RkWorld( Executor );
	mNative->SetGravity( { 0.0f, -10.0f, 0.0f } );
	}


//--------------------------------------------------------------------------------------------------
VsRagnarokWorld::~VsRagnarokWorld()
	{
	while ( !mBodies.empty() )
		{
		VsRagnarokBody* Body = mBodies.back();
		NotifyBodyRemoved( Body );
		mBodies.pop_back();
		delete Body;
		}
	VS_ASSERT( mNative->GetBodyCount() == 0 );

	delete mNative;
	mNative = nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsPlugin* VsRagnarokWorld::GetPlugin() const
	{
	return mPlugin;
	}


//--------------------------------------------------------------------------------------------------
VsColor VsRagnarokWorld::GetColor() const
	{
	return mColor;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokWorld::SetColor( const VsColor& Color )
	{
	mColor = Color;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokWorld::AddListener( IVsWorldListener* Listener )
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
	for ( VsRagnarokBody* Body : mBodies )
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
void VsRagnarokWorld::RemoveListener( IVsWorldListener* Listener )
	{
	if ( !Listener )
		{
		return;
		}

	std::erase( mListeners, Listener );
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokWorld::NotifyBodyAdded( IVsBody* Body )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnBodyAdded( Body ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokWorld::NotifyBodyRemoved( IVsBody* Body )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnBodyRemoved( Body ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokWorld::NotifyShapeAdded( IVsBody* Body, IVsShape* Shape )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnShapeAdded( Body, Shape ); } );
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokWorld::NotifyShapeRemoved( IVsBody* Body, IVsShape* Shape )
	{
	std::for_each( mListeners.begin(), mListeners.end(), [ = ]( IVsWorldListener* Listener ) { Listener->OnShapeRemoved( Body, Shape ); } );
	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsRagnarokWorld::GetGravity() const
	{
	RkVector3 Gravity = mNative->GetGravity();
	return { Gravity.X, Gravity.Y, Gravity.Z };
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokWorld::SetGravity( const VsVector3& Gravity )
	{
	mNative->SetGravity( { Gravity.X, Gravity.Y, Gravity.Z } );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsRagnarokWorld::CreateBody( VsBodyType Type )
	{
	VsRagnarokBody* Body = new VsRagnarokBody( this, Type );
	mBodies.push_back( Body );
	NotifyBodyAdded( Body );

	return Body;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokWorld::DestroyBody( IVsBody* Body )
	{
	if ( !Body )
		{
		return;
		}

	NotifyBodyRemoved( Body );
	std::erase( mBodies, Body );
	delete static_cast< VsRagnarokBody* >( Body );
	}


//--------------------------------------------------------------------------------------------------
int VsRagnarokWorld::GetBodyCount() const
	{
	return static_cast< int >( mBodies.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsRagnarokWorld::GetBody( int BodyIndex )
	{
	return ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsBody* VsRagnarokWorld::GetBody( int BodyIndex ) const
	{
	return ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokWorld::Step( float Timestep )
	{
	mNative->Step( 8, Timestep );
	}


//--------------------------------------------------------------------------------------------------
RkWorld* VsRagnarokWorld::GetNative() const
	{
	return mNative;
	}


//--------------------------------------------------------------------------------------------------
// VsRagnarokPlugin
//--------------------------------------------------------------------------------------------------
VsRagnarokPlugin::VsRagnarokPlugin( ImGuiContext* Context )
	{
	VS_ASSERT( Context );
	ImGui::SetCurrentContext( Context );

	int WorkerCount = std::min( 8, static_cast< int >( std::thread::hardware_concurrency() / 2 ) );
	mExecutor = new tf::Executor( WorkerCount );
	}


//--------------------------------------------------------------------------------------------------
VsRagnarokPlugin::~VsRagnarokPlugin()
	{
	while ( !mWorlds.empty() )
		{
		VsRagnarokWorld* World = mWorlds.back();
		mWorlds.pop_back();
		delete World;
		}

	while ( !mMeshes.empty() )
		{
		VsRagnarokMesh* Mesh = mMeshes.back();
		mMeshes.pop_back();
		delete Mesh;
		}

	while ( !mHulls.empty() )
		{
		VsRagnarokHull* Hull = mHulls.back();
		mHulls.pop_back();
		delete Hull;
		}

	delete mExecutor;
	mExecutor = nullptr;

	ImGui::SetCurrentContext( NULL );
	}

//--------------------------------------------------------------------------------------------------
void VsRagnarokPlugin::Release()
	{
	delete this;
	}


//--------------------------------------------------------------------------------------------------
const char* VsRagnarokPlugin::GetName() const
	{
	return "Ragnarok";
	}


//--------------------------------------------------------------------------------------------------
const char* VsRagnarokPlugin::GetVersion() const
	{
	return "0.1";
	}


//--------------------------------------------------------------------------------------------------
bool VsRagnarokPlugin::IsEnabled() const
	{
	return mEnabled;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokPlugin::SetEnabled( bool Enabled )
	{
	mEnabled = Enabled;
	}


//--------------------------------------------------------------------------------------------------
bool VsRagnarokPlugin::OnInspectorGUI()
	{
	ImGui::Text( "Ragnarok 0.1" );
	return false;
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsRagnarokPlugin::CreateHull( int VertexCount, const VsVector3* Vertices )
	{
	if ( RkHull* Hull = rkCreateHull( VertexCount, (const RkVector3*)Vertices ) )
		{
		mHulls.push_back( new VsRagnarokHull( Hull ) );
		return mHulls.back();
		}

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokPlugin::DestroyHull( IVsHull* Hull )
	{
	if ( !Hull )
		{
		return;
		}

	std::erase( mHulls, Hull );
	delete static_cast< VsRagnarokHull* >( Hull );
	}


//--------------------------------------------------------------------------------------------------
int VsRagnarokPlugin::GetHullCount() const
	{
	return static_cast< int >( mHulls.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsRagnarokPlugin::GetHull( int HullIndex )
	{
	return ( 0 <= HullIndex && HullIndex < GetHullCount() ) ? mHulls[ HullIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsHull* VsRagnarokPlugin::GetHull( int HullIndex ) const
	{
	return ( 0 <= HullIndex && HullIndex < GetHullCount() ) ? mHulls[ HullIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsRagnarokPlugin::CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices )
	{
	// DIRK_TODO: ...
	VS_ASSERT( 0 );

	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokPlugin::DestroyMesh( IVsMesh* Mesh )
	{
	if ( !Mesh )
		{
		return;
		}

	std::erase( mMeshes, Mesh );
	delete static_cast< VsRagnarokMesh* >( Mesh );
	}


//--------------------------------------------------------------------------------------------------
int VsRagnarokPlugin::GetMeshCount() const
	{
	return static_cast< int >( mMeshes.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsMesh* VsRagnarokPlugin::GetMesh( int MeshIndex )
	{
	return ( 0 <= MeshIndex && MeshIndex < GetMeshCount() ) ? mMeshes[ MeshIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsMesh* VsRagnarokPlugin::GetMesh( int MeshIndex ) const
	{
	return ( 0 <= MeshIndex && MeshIndex < GetMeshCount() ) ? mMeshes[ MeshIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsRagnarokPlugin::CreateWorld()
	{
	VsRagnarokWorld* World = new VsRagnarokWorld( this );
	mWorlds.push_back( World );

	return World;
	}


//--------------------------------------------------------------------------------------------------
void VsRagnarokPlugin::DestroyWorld( IVsWorld* World )
	{
	if ( !World )
		{
		return;
		}

	std::erase( mWorlds, World );
	delete static_cast< VsRagnarokWorld* >( World );
	}


//--------------------------------------------------------------------------------------------------
int VsRagnarokPlugin::GetWorldCount() const
	{
	return static_cast< int >( mWorlds.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsWorld* VsRagnarokPlugin::GetWorld( int WorldIndex )
	{
	return ( 0 <= WorldIndex && WorldIndex < GetWorldCount() ) ? mWorlds[ WorldIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsWorld* VsRagnarokPlugin::GetWorld( int WorldIndex ) const
	{
	return ( 0 <= WorldIndex && WorldIndex < GetWorldCount() ) ? mWorlds[ WorldIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
tf::Executor* VsRagnarokPlugin::GetExecutor() const
	{
	return mExecutor;
	}


// Export
VS_EXPORT_PLUGIN( VsRagnarokPlugin );
