//--------------------------------------------------------------------------------------------------
/**
	@file		scene.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <variant>
#include <vector>


//--------------------------------------------------------------------------------------------------
// TlFrame
//--------------------------------------------------------------------------------------------------
struct TlFrame
	{
	glm::vec3 Origin;
	glm::quat Basis;
	};


//--------------------------------------------------------------------------------------------------
// TlGeometry
//--------------------------------------------------------------------------------------------------
struct TlGeometry
	{
	int TriangleCount = 0;
	std::vector< int > TriangleIndices;

	int VertexCount = 0;
	std::vector< float > VertexPositions;
	std::vector< float > VertexNormals;

	int EdgeCount = 0;
	std::vector< int > EdgeIndices;   
	std::vector< uint32_t > EdgeColors;  
	};


//--------------------------------------------------------------------------------------------------
// TlShapeType
//--------------------------------------------------------------------------------------------------
enum TlShapeType
	{
	kSphereShape,
	kCapsuleShape,
	kHullShape,
	kMeshShape
	};


//--------------------------------------------------------------------------------------------------
// TlSphereShape
//--------------------------------------------------------------------------------------------------
struct TlSphereShape
	{
	glm::vec3 Center;
	float Radius;
	};


//--------------------------------------------------------------------------------------------------
// TlCapsuleShape
//--------------------------------------------------------------------------------------------------
struct TlCapsuleShape
	{
	glm::vec3 Center1;
	glm::vec3 Center2;
	float Radius;
	};


//--------------------------------------------------------------------------------------------------
// TlHullShape
//--------------------------------------------------------------------------------------------------
struct TlHullShape 
	{
	const TlGeometry* Geometry;
	float Scale = 1.0f;
	};


//--------------------------------------------------------------------------------------------------
// TlMeshShape
//--------------------------------------------------------------------------------------------------
struct TlMeshShape 
	{
	const TlGeometry* Geometry;
	glm::vec3 Scale = { 1.0f, 1.0f, 1.0f };
	};


//--------------------------------------------------------------------------------------------------
// TlShape
//--------------------------------------------------------------------------------------------------
struct TlShape
	{
	using TlShapeVariant = std::variant< TlSphereShape, TlCapsuleShape, TlHullShape, TlMeshShape >;
	TlShapeVariant Variant;
	};


//--------------------------------------------------------------------------------------------------
// TlBodyType
//--------------------------------------------------------------------------------------------------
enum TlBodyType 
	{ 
	kStaticBody,
	kKeyframedBody,
	kDynamicBody
	};


//--------------------------------------------------------------------------------------------------
// TlBodyType
//--------------------------------------------------------------------------------------------------
struct TlBody
	{
	TlBodyType Type = kStaticBody;
	glm::vec3 Position = { 0.0f, 0.0f, 0.0f };
	glm::quat Orientation = { 1.0f, 0.0f, 0.0f, 0.0f };

	std::vector< TlShape > Shapes;

	// Shape helpers
	int GetShapeCount() const
		{
		return static_cast< int >( Shapes.size() );
		}

	TlSphereShape& AddSphereShape( const glm::vec3& Center, float Radius )
		{
		TlShape& Shape = Shapes.emplace_back( TlSphereShape{ Center, Radius } );
		return std::get< TlSphereShape >( Shape.Variant );
		}

	TlCapsuleShape& AddCapsuleShape( const glm::vec3& Center1, const glm::vec3& Center2, float Radius )
		{
		TlShape& Shape = Shapes.emplace_back( TlCapsuleShape{ Center1, Center2, Radius } );
		return std::get< TlCapsuleShape >( Shape.Variant );
		}

	TlHullShape& AddHullShape( const TlGeometry* Geometry, float Scale = 1.0f )
		{
		TlShape& Shape = Shapes.emplace_back( TlHullShape{ Geometry, Scale } );
		return std::get< TlHullShape >( Shape.Variant );
		}

	TlMeshShape& AddMeshShape( const TlGeometry* Geometry, const glm::vec3 Scale = { 1.0f, 1.0f, 1.0f } )
		{
		TlShape& Shape = Shapes.emplace_back( TlMeshShape{ Geometry, Scale } );
		return std::get< TlMeshShape >( Shape.Variant );
		}
	};


//--------------------------------------------------------------------------------------------------
// TlSphericalJoint
//--------------------------------------------------------------------------------------------------
struct TlSphericalJoint
	{
	bool EnableSwingLimit = false;
	float MaxSwingAngle = 90.0f;

	bool EnableTwistLimt = false;
	float MinTwistAngle = -175.0f;
	float MaxTwistAngle = -175.0f;
	};


//--------------------------------------------------------------------------------------------------
// TlRevoluteJoint
//--------------------------------------------------------------------------------------------------
struct TlRevoluteJoint
	{
	bool EnableLimt = false;
	float MinAngle = -175.0f;
	float MaxAngle = -175.0f;
	};


//--------------------------------------------------------------------------------------------------
// TlPrismaticJoint
//--------------------------------------------------------------------------------------------------
struct TlPrismaticJoint
	{
	bool EnableLimt = false;
	float MinOffset = 0.0f;
	float MaxOffset = 1.0f;
	};


//--------------------------------------------------------------------------------------------------
// TlRigidJoint
//--------------------------------------------------------------------------------------------------
struct TlRigidJoint
	{
	float LinearFrequency;
	float LinearDampingRatio;
	float AngularFrequency;
	float AngularDampingRatio;
	};


//--------------------------------------------------------------------------------------------------
// TlJoint
//--------------------------------------------------------------------------------------------------
struct TlJoint
	{
	int BodyIndex1;
	TlFrame LocalFrame1;
	int BodyIndex2;
	TlFrame LocalFrame2;

	using TlJointVariant = std::variant< TlSphericalJoint, TlRevoluteJoint, TlPrismaticJoint, TlRigidJoint >;
	TlJointVariant Variant;
	};


//--------------------------------------------------------------------------------------------------
// TlScene
//--------------------------------------------------------------------------------------------------
struct TlScene
	{
	std::vector< TlBody > Bodies;
	std::vector< TlJoint > Joints;

	// Body helpers
	int GetBodyCount() const
		{
		return static_cast<int>( Bodies.size() );
		}

	TlBody& AddBody()
		{
		return Bodies.emplace_back();
		}

	// Joint helpers
	int GetJointCount() const
		{
		return static_cast<int>( Joints.size() );
		}

	TlSphericalJoint& AddSphericalJoint( int BodyIndex1, const TlFrame& LocalFrame1, int BodyIndex2, const TlFrame& LocalFrame2 )
		{
		TlJoint& Joint = Joints.emplace_back( BodyIndex1, LocalFrame1, BodyIndex2, LocalFrame2, TlSphericalJoint{} );
		return std::get< TlSphericalJoint >( Joint.Variant );
		}

	TlRevoluteJoint& AddRevoluteJoint( int BodyIndex1, const TlFrame& LocalFrame1, int BodyIndex2, const TlFrame& LocalFrame2 )
		{
		TlJoint& Joint = Joints.emplace_back( BodyIndex1, LocalFrame1, BodyIndex2, LocalFrame2, TlRevoluteJoint{} );
		return std::get< TlRevoluteJoint >( Joint.Variant );
		}

	TlPrismaticJoint& AddPrismaticJoint( int BodyIndex1, const TlFrame& LocalFrame1, int BodyIndex2, const TlFrame& LocalFrame2 )
		{
		TlJoint& Joint = Joints.emplace_back( BodyIndex1, LocalFrame1, BodyIndex2, LocalFrame2, TlPrismaticJoint{} );
		return std::get< TlPrismaticJoint >( Joint.Variant );
		}

	TlRigidJoint& AddRigidJoint( int BodyIndex1, const TlFrame& LocalFrame1, int BodyIndex2, const TlFrame& LocalFrame2 )
		{
		TlJoint& Joint = Joints.emplace_back( BodyIndex1, LocalFrame1, BodyIndex2, LocalFrame2, TlRigidJoint{} );
		return std::get< TlRigidJoint >( Joint.Variant );
		}
	};
