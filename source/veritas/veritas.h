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

#include <algorithm>
#include <variant>
#include <vector>


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
// VsSphereDef
//--------------------------------------------------------------------------------------------------
struct VsSphereDef
	{
	VsVector3 Center;
	float Radius;
	};


//--------------------------------------------------------------------------------------------------
// VsCapsuleDef
//--------------------------------------------------------------------------------------------------
struct VsCapsuleDef
	{
	VsVector3 Center1;
	VsVector3 Center2;
	float Radius;
	};


//--------------------------------------------------------------------------------------------------
// VsHullDef
//--------------------------------------------------------------------------------------------------
struct VsHullDef
	{
	
	};


//--------------------------------------------------------------------------------------------------
// VsMeshDef
//--------------------------------------------------------------------------------------------------
struct VsMeshDef
	{
	
	};


//--------------------------------------------------------------------------------------------------
// VsShapeDef
//--------------------------------------------------------------------------------------------------
struct VsShapeDef
	{
	using VsShapeVariant = std::variant< VsSphereDef, VsCapsuleDef, VsHullDef, VsMeshDef >;
	VsShapeVariant Variant;
	};


//--------------------------------------------------------------------------------------------------
// VsBodyType
//--------------------------------------------------------------------------------------------------
enum VsBodyType
	{ 
	VS_STATIC_BODY,
	VS_KEYFRAME_BODY,
	VS_DYNAMIC_BODY
	};


//--------------------------------------------------------------------------------------------------
// VsBodyDef
//--------------------------------------------------------------------------------------------------
struct VsBodyDef
	{
	VsBodyType Type = VS_STATIC_BODY;
	};


//--------------------------------------------------------------------------------------------------
// VsSphericalJointDef
//--------------------------------------------------------------------------------------------------
struct VsSphericalJointDef
	{
	bool EnableSwingLimit = false;
	float MaxSwingAngle = 90.0f;

	bool EnableTwistLimt = false;
	float MinTwistAngle = -175.0f;
	float MaxTwistAngle = -175.0f;
	};


//--------------------------------------------------------------------------------------------------
// VsRevoluteJointDef
//--------------------------------------------------------------------------------------------------
struct VsRevoluteJointDef
	{
	bool EnableLimt = false;
	float MinAngle = -175.0f;
	float MaxAngle = -175.0f;
	};


//--------------------------------------------------------------------------------------------------
// VsPrismaticJointDef
//--------------------------------------------------------------------------------------------------
struct VsPrismaticJointDef
	{
	bool EnableLimt = false;
	float MinOffset = 0.0f;
	float MaxOffset = 1.0f;
	};


//--------------------------------------------------------------------------------------------------
// VsRigidJoint
//--------------------------------------------------------------------------------------------------
struct VsRigidJointDef
	{
	float LinearFrequency;
	float LinearDampingRatio;
	float AngularFrequency;
	float AngularDampingRatio;
	};


//--------------------------------------------------------------------------------------------------
// VsJointDef
//--------------------------------------------------------------------------------------------------
struct VsJointDef
	{
	int BodyIndex1;
	VsFrame LocalFrame1;
	int BodyIndex2;
	VsFrame LocalFrame2;

	using VsJointVariant = std::variant< VsSphericalJointDef, VsRevoluteJointDef, VsPrismaticJointDef, VsRigidJointDef >;
	VsJointVariant Variant;
	};


//--------------------------------------------------------------------------------------------------
// VsWorldDef
//--------------------------------------------------------------------------------------------------
struct VsWorldDef
	{
	std::vector< VsBodyDef > Bodies;
	std::vector< VsJointDef > Joints;
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
	protected:
		virtual ~IVsHull() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsHullShape
//--------------------------------------------------------------------------------------------------
struct IVsHullShape : IVsShape
	{
	protected:
		virtual ~IVsHullShape() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsMesh
//--------------------------------------------------------------------------------------------------
struct IVsMesh
	{
	protected:
		virtual ~IVsMesh() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsMeshShape
//--------------------------------------------------------------------------------------------------
struct IVsMeshShape : IVsShape
	{
	protected:
		virtual ~IVsMeshShape() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsBody
//--------------------------------------------------------------------------------------------------
struct IVsBody
	{
	virtual VsBodyType GetType() const = 0;

	virtual VsVector3 GetPosition() const = 0;
	virtual void SetPosition( const VsVector3& Position ) = 0;
	virtual VsQuaternion GetOrientation() const = 0;
	virtual void SetOrientation( const VsQuaternion& Orientation ) = 0;

	virtual float GetMass() const = 0;
	virtual float GetMassInv() const = 0;
	virtual VsMatrix3 GetLocalInertia() const = 0;
	virtual VsMatrix3 GetLocalInertiaInv() const = 0;
	virtual VsMatrix3 GetInertia() const = 0;
	virtual VsMatrix3 GetInertiaInv() const = 0;
	virtual VsVector3 GetLocalMassCenter() const = 0;
	virtual VsVector3 GetMassCenter() const = 0;

	virtual IVsSphereShape* CreateSphere( const VsSphereDef& SphereDef ) = 0;
	virtual IVsCapsuleShape* CreateCapulse( const VsCapsuleDef& CapsuleDef ) = 0;
	virtual IVsHullShape* CreateHull( const VsHullDef& HullDef ) = 0;
	virtual IVsMeshShape* CreateMesh( const VsMeshDef& MeshDef ) = 0;
	virtual void DestroyShape( IVsShape* Shape ) = 0;

	virtual int GetShapeCount() const = 0;
	virtual IVsShape* GetShape( int ShapeIndex ) = 0;
	virtual const IVsShape* GetShape( int ShapeIndex ) const = 0;

	protected:
		virtual ~IVsBody() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsWorld
//--------------------------------------------------------------------------------------------------
struct IVsWorld
	{
	virtual VsVector3 GetGravity() const = 0;
	virtual void SetGravity( const VsVector3& Gravity ) = 0;

	virtual IVsBody* CreateBody( const VsBodyDef& BodyDef ) = 0;
	virtual void DestroyBody( IVsBody* Body ) = 0;

	protected:
		virtual ~IVsWorld() = default;
	};


//--------------------------------------------------------------------------------------------------
// IVsPlugin
//--------------------------------------------------------------------------------------------------
struct IVsPlugin
	{
	virtual const char* GetName() const = 0;
	virtual const char* GetVersion() const = 0;
	
	virtual IVsHull* CreateHull( int VertexCount, const VsVector3* Vertices ) = 0;
	virtual IVsMesh* CreateMesh( int TriangleCount, const int* TriangleIndices, int VertexCount, const VsVector3* Vertices ) = 0;

	virtual IVsWorld* CreateWorld( const VsWorldDef& WorldDef ) = 0;
	virtual void DestroyWorld( IVsWorld* World ) = 0;

	protected:
		virtual ~IVsPlugin() = default;
	};

// Plugin entry point signatures
extern "C" 
	{
	typedef IVsPlugin* ( *VsCreatePlugin )();
	typedef void ( *VsDestroyPlugin )( IVsPlugin* );
	}

// Macro for plugin implementation
#define VS_EXPORT_PLUGIN(PluginClass)                       \
    extern "C" __declspec(dllexport)                        \
    IVsPlugin* vsCreatePlugin()								\
	{														\
        return new PluginClass();                           \
    }                                                       \
    extern "C" __declspec(dllexport)                        \
    void vsDestroyPlugin(IPlugin* Plugin)					\
	{														\
        delete p;                                           \
    }
	
