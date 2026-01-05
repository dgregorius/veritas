//--------------------------------------------------------------------------------------------------
/**
	@file		veritas.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <filesystem>
namespace fs = std::filesystem;


//--------------------------------------------------------------------------------------------------
// VsVector3
//--------------------------------------------------------------------------------------------------
struct VsVector3
	{
	float X, Y, Z;
	};


//--------------------------------------------------------------------------------------------------
// VsMatrix3
//--------------------------------------------------------------------------------------------------
struct VsMatrix3
	{
	float A11, A21, A31;
	float A12, A22, A32;
	float A13, A23, A33;
	};


//--------------------------------------------------------------------------------------------------
// VsQuaternion
//--------------------------------------------------------------------------------------------------
struct VsQuaternion
	{
	float X, Y, Z, W;
	};


//--------------------------------------------------------------------------------------------------
// VsFrame
//--------------------------------------------------------------------------------------------------
struct VsFrame
	{
	VsVector3 Origin;
	VsQuaternion Basis;
	};


//--------------------------------------------------------------------------------------------------
// VsShapeType
//--------------------------------------------------------------------------------------------------
enum VsShapeType
	{
	VS_SPHERE_SHAPE,
	VS_CAPSULE_SHAPE,
	VS_HULL_SHAPE,
	VS_MESH_SHAPE
	};


//--------------------------------------------------------------------------------------------------
// IVsShape
//--------------------------------------------------------------------------------------------------
struct IVsShape
	{
	protected:
		virtual ~IVsShape() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsSphereShape
//--------------------------------------------------------------------------------------------------
struct IVsSphereShape : IVsShape
	{
	virtual VsVector3 GetCenter() const = 0;
	virtual float GetRadius() const = 0;

	protected:
		virtual ~IVsSphereShape() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsCapsuleShape
//--------------------------------------------------------------------------------------------------
struct IVsCapsuleShape : IVsShape
	{
	virtual VsVector3 GetCenter1() const = 0;
	virtual VsVector3 GetCenter2() const = 0;
	virtual float GetRadius() const = 0;

	protected:
		virtual ~IVsCapsuleShape() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsHull
//--------------------------------------------------------------------------------------------------
struct IVsHull
	{
	virtual int GetVertexCount() const = 0;
	virtual const VsVector3* GetVertexPositions() const = 0;
	virtual const VsVector3* GetVertexNormals() const = 0;
	virtual int GetEdgeCount() const = 0;
	virtual const VsVector3* GetEdges() const = 0;

	protected:
		virtual ~IVsHull() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsHullShape
//--------------------------------------------------------------------------------------------------
struct IVsHullShape : IVsShape
	{
	virtual const IVsHull* GetHull() const = 0;

	protected:
		virtual ~IVsHullShape() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsMesh
//--------------------------------------------------------------------------------------------------
struct IVsMesh
	{
	virtual int GetVertexCount() const = 0;
	virtual const VsVector3* GetVertexPositions() const = 0;
	virtual const VsVector3* GetVertexNormals() const = 0;

	protected:
		virtual ~IVsMesh() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsMeshShape
//--------------------------------------------------------------------------------------------------
struct IVsMeshShape : IVsShape
	{
	virtual const IVsMesh* GetMesh() const = 0;

	protected:
		virtual ~IVsMeshShape() = default;
	};


//--------------------------------------------------------------------------------------------------
// VsBodyType
//--------------------------------------------------------------------------------------------------
enum VsBodyType
	{
	VS_STATIC_BODY,
	VS_KEYFRAMED_BODY,
	VS_DYNAMIC_BODY
	};


//--------------------------------------------------------------------------------------------------
// IVsBody
//--------------------------------------------------------------------------------------------------
struct IVsBody
	{
	// Type 
	virtual VsBodyType GetType() const = 0;

	// Transform
	virtual VsVector3 GetPosition() const = 0;
	virtual void SetPosition( const VsVector3& Position ) = 0;
	virtual VsQuaternion GetOrientation() const = 0;
	virtual void SetOrientation( const VsQuaternion& Orientation ) = 0;

	// Shapes
	virtual IVsSphereShape* CreateSphere( const VsVector3& Center, float Radius ) = 0;
	virtual IVsCapsuleShape* CreateCapulse( const VsVector3& Center1, const VsVector3& Center2, float Radius ) = 0;
	virtual IVsHullShape* CreateHull( const IVsHull* Hull ) = 0;
	virtual IVsMeshShape* CreateMesh( const IVsMesh* Mesh ) = 0;
	virtual void DestroyShape( IVsShape* Shape ) = 0;
	
	virtual int GetShapeCount() const = 0;
	virtual IVsShape* GetShape( int ShapeIndex ) = 0;
	virtual const IVsShape* GetShape( int ShapeIndex ) const = 0;

	protected:
		virtual ~IVsBody() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsWorldListener
//--------------------------------------------------------------------------------------------------
struct IVsWorldListener
	{
	virtual void OnBodyAdded( IVsBody* Body ) = 0;
	virtual void OnBodyRemoved( IVsBody* Body ) = 0;
	virtual void OnShapeAdded( IVsBody* Body, IVsShape* Shape ) = 0;
	virtual void OnShapeRemoved( IVsBody* Body, IVsShape* Shape ) = 0;

	protected:
		virtual ~IVsWorldListener() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsWorld
//--------------------------------------------------------------------------------------------------
struct IVsWorld
	{
	virtual void AddListener( IVsWorldListener* Listener ) = 0;
	virtual void RemoveListener( IVsWorldListener* Listener ) = 0;

	virtual VsVector3 GetGravity() const = 0;
	virtual void SetGravity( const VsVector3& Gravity ) = 0;

	virtual IVsBody* CreateBody( VsBodyType Type ) = 0;
	virtual void DestroyBody( IVsBody* Body ) = 0;
	virtual int GetBodyCount() const = 0;
	virtual IVsBody* GetBody( int BodyIndex ) = 0;
	virtual const IVsBody* GetBody( int BodyIndex ) const = 0;

	virtual void Step( float Timestep ) = 0;

	protected:
		virtual ~IVsWorld() = default;
	};





//--------------------------------------------------------------------------------------------------
// IVsPlugin
//--------------------------------------------------------------------------------------------------
struct IVsPlugin
	{
	// Module
	virtual const char* GetName() const = 0;
	virtual const char* GetVersion() const = 0;
	
	// Hull
	IVsHull* CreateBox( VsVector3 Extent );
	virtual IVsHull* CreateHull( int VertexCount, const VsVector3* Vertices ) = 0;
	virtual void DestroyHull( IVsHull* Hull ) = 0;
	virtual int GetHullCount() const = 0;
	virtual IVsHull* GetHull( int HullIndex ) = 0;
	virtual const IVsHull* GetHull( int HullIndex ) const = 0;

	// Mesh
	virtual IVsMesh* CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices ) = 0;
	virtual void DestroyMesh( IVsMesh* Mesh ) = 0;
	virtual int GetMeshCount() const = 0;
	virtual IVsMesh* GetMesh( int MeshIndex ) = 0;
	virtual const IVsMesh* GetMesh( int MeshIndex ) const = 0;
	
	// World
	virtual IVsWorld* CreateWorld() = 0;
	virtual void DestroyWorld( IVsWorld* World ) = 0;
	virtual int GetWorldCount() const = 0;
	virtual IVsWorld* GetWorld( int WorldIndex ) = 0;
	virtual const IVsWorld* GetWorld( int WorldIndex ) const = 0;

	protected:
		virtual ~IVsPlugin() = default;
	};

// Plugin export
extern "C"
	{
	typedef IVsPlugin* ( *VsCreatePluginFunc )( );
	typedef void ( *VsDestroyPluginFunc )( IVsPlugin* );
	}

#define VS_EXPORT_PLUGIN(PluginClass)							\
extern "C" __declspec(dllexport)								\
IVsPlugin* vsCreatePlugin()										\
	{															\
    return new PluginClass;										\
	}															\
extern "C" __declspec(dllexport)								\
void vsDestroyPlugin(IVsPlugin* Plugin)							\
	{															\
    delete static_cast< PluginClass* >( Plugin );				\
	}


//--------------------------------------------------------------------------------------------------
// VsModule
//--------------------------------------------------------------------------------------------------
struct VsModule;

VsModule* vsLoadModule( const fs::path& Path );
IVsPlugin* vsGetPlugin( VsModule* Module );
void vsFreeModule( VsModule* Module );
