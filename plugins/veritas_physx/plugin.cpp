//--------------------------------------------------------------------------------------------------
// plugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "plugin.h"

// Windows
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>


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
	VS_ASSERT( SceneDesc.isValid() );
	
	mNative = Physics->createScene( SceneDesc );
	VS_ASSERT( mNative );
	}


//--------------------------------------------------------------------------------------------------
VsPhysXWorld::~VsPhysXWorld()
	{
	PX_RELEASE( mNative );
	}


//--------------------------------------------------------------------------------------------------
IVsPlugin* VsPhysXWorld::GetPlugin() const
	{
	return mPlugin;
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
// 	for ( VsPhysXBody* Body : mBodies )
		{
// 		Listener->OnBodyAdded( Body );
// 		for ( int ShapeIndex = 0; ShapeIndex < Body->GetShapeCount(); ++ShapeIndex )
// 			{
// 			IVsShape* Shape = Body->GetShape( ShapeIndex );
// 			Listener->OnShapeAdded( Body, Shape );
// 			}
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
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::DestroyBody( IVsBody* Body )
	{

	}


//--------------------------------------------------------------------------------------------------
int VsPhysXWorld::GetBodyCount() const
	{
	return static_cast< int >( mBodies.size() );
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsPhysXWorld::GetBody( int BodyIndex )
	{
	return nullptr; // ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsBody* VsPhysXWorld::GetBody( int BodyIndex ) const
	{
	return  nullptr; // ( 0 <= BodyIndex && BodyIndex < GetBodyCount() ) ? mBodies[ BodyIndex ] : nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::Step( float Timestep )
	{
	// DIRK_TODO: Pass scratch buffer...
	mNative->simulate( Timestep );
	mNative->fetchResults( true );
	}


//--------------------------------------------------------------------------------------------------
// VsPhysXPlugin
//--------------------------------------------------------------------------------------------------
VsPhysXPlugin::VsPhysXPlugin()
	{
	fs::path ModulePath = fs::current_path() / "plugins/physx";
	SetDllDirectoryW( ModulePath.c_str() );

	mFoundation = PxCreateFoundation( PX_PHYSICS_VERSION, mAllocator, mErrorCallback );
	mPhysics = PxCreatePhysics( PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale() );
	bool Success = PxInitExtensions( *mPhysics, NULL );
	VS_ASSERT( Success );
	mDispatcher = PxDefaultCpuDispatcherCreate( std::min( 8, (int)std::thread::hardware_concurrency() / 2 ) );

	SetDllDirectory( NULL );
	}


//--------------------------------------------------------------------------------------------------
VsPhysXPlugin::~VsPhysXPlugin()
	{
	PX_RELEASE( mDispatcher );
	PxCloseExtensions();
	PX_RELEASE( mPhysics );
	PX_RELEASE( mFoundation );
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
	return "5.6.1";
	}


//--------------------------------------------------------------------------------------------------
IVsHull* VsPhysXPlugin::CreateHull( int VertexCount, const VsVector3* Vertices )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXPlugin::DestroyHull( IVsHull* Hull )
	{

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
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXPlugin::DestroyMesh( IVsMesh* Mesh )
	{

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
