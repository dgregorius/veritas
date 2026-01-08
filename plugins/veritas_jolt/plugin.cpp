//--------------------------------------------------------------------------------------------------
// plugin.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "plugin.h"

// Jolt
#include <Jolt/Jolt.h>


//--------------------------------------------------------------------------------------------------
// VsJoltWorld
//--------------------------------------------------------------------------------------------------
VsJoltWorld::VsJoltWorld( VsJoltPlugin* Plugin )
	: mPlugin( Plugin )
	{

	}


//--------------------------------------------------------------------------------------------------
VsJoltWorld::~VsJoltWorld()
	{

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

	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::RemoveListener( IVsWorldListener* Listener )
	{

	}


//--------------------------------------------------------------------------------------------------
VsVector3 VsJoltWorld::GetGravity() const
	{
	return { 0.0f, -10.0f, 0.0f };
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::SetGravity( const VsVector3& Gravity )
	{

	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsJoltWorld::CreateBody( VsBodyType Type )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::DestroyBody( IVsBody* Body )
	{

	}


//--------------------------------------------------------------------------------------------------
int VsJoltWorld::GetBodyCount() const
	{
	return 0;
	}


//--------------------------------------------------------------------------------------------------
IVsBody* VsJoltWorld::GetBody( int BodyIndex )
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
const IVsBody* VsJoltWorld::GetBody( int BodyIndex ) const
	{
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltWorld::Step( float Timestep )
	{

	}


//--------------------------------------------------------------------------------------------------
// VsJoltPlugin
//--------------------------------------------------------------------------------------------------
VsJoltPlugin::VsJoltPlugin()
	{

	}


//--------------------------------------------------------------------------------------------------
VsJoltPlugin::~VsJoltPlugin()
	{
	
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
	return nullptr;
	}


//--------------------------------------------------------------------------------------------------
void VsJoltPlugin::DestroyHull( IVsHull* Hull )
	{

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


// Export
VS_EXPORT_PLUGIN( VsJoltPlugin );



