//--------------------------------------------------------------------------------------------------
// plugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "plugin.h"



//--------------------------------------------------------------------------------------------------
// VsPhysXWorld
//--------------------------------------------------------------------------------------------------
VsPhysXWorld::VsPhysXWorld( VsPhysXPlugin* Plugin )
	: mPlugin( Plugin )
	{

	}


//--------------------------------------------------------------------------------------------------
VsPhysXWorld::~VsPhysXWorld()
	{

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

	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::RemoveListener( IVsWorldListener* Listener )
	{

	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsPhysXWorld::GetGravity() const
	{
	return { 0.0f, -10.0f, 0.0f };
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::SetGravity( const VsVector3& Gravity )
	{

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
	return 0;
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsPhysXWorld::GetBody( int BodyIndex )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsBody* VsPhysXWorld::GetBody( int BodyIndex ) const
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsPhysXWorld::Step( float Timestep )
	{

	}


//--------------------------------------------------------------------------------------------------
// VsPhysXPlugin
//--------------------------------------------------------------------------------------------------
VsPhysXPlugin::VsPhysXPlugin()
	{
	mFoundation = PxCreateFoundation( PX_PHYSICS_VERSION, mAllocator, mErrorCallback );
	mPhysics = PxCreatePhysics( PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale() );
	bool Success = PxInitExtensions( *mPhysics, NULL );
	VS_ASSERT( Success );
	}


//--------------------------------------------------------------------------------------------------
VsPhysXPlugin::~VsPhysXPlugin()
	{
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


// Export
VS_EXPORT_PLUGIN( VsPhysXPlugin );
