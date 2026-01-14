//--------------------------------------------------------------------------------------------------
// mass.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "mass.h"


//--------------------------------------------------------------------------------------------------
// RkMassProperties
//--------------------------------------------------------------------------------------------------
RkMassProperties::RkMassProperties() 
	: Mass( 0.0f )
	, Center( RK_VEC3_ZERO )
	, Inertia( RK_MAT3_ZERO )
	{

	}

//--------------------------------------------------------------------------------------------------
RkMassProperties::RkMassProperties( float Mass, const RkVector3& Center, const RkMatrix3& Inertia )
	: Mass( Mass )
	, Center( Center )
	, Inertia( Inertia )
	{

	}


//--------------------------------------------------------------------------------------------------
bool RkMassProperties::AreValid() const
	{
	if ( Mass <= 0.0f )
		{
		return false;
		}

	float Det = rkDet( Inertia );
	if ( Det < 1000.0f * RK_F32_MIN )
		{
		return false;
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
RkMassProperties rkComputeMassProperties( int TriangleCount, const int* Triangles, int VertexCount, const RkVector3* Vertices, float Density )
	{
	RkMassProperties Out;
	if ( TriangleCount <= 0 || !Triangles )
		{
		return Out;
		}
	if ( VertexCount <= 0 || !Vertices )
		{
		return Out;
		}

	// M. Kallay - "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
	float Volume = 0;
	RkVector3 Center = RK_VEC3_ZERO;

	float XX = 0;  float XY = 0;
	float YY = 0;  float XZ = 0;
	float ZZ = 0;  float YZ = 0;

	for ( int TriangleIndex = 0; TriangleIndex < TriangleCount; ++TriangleIndex )
		{
		int VertexIndex1 = Triangles[ 3 * TriangleIndex + 0 ];
		int VertexIndex2 = Triangles[ 3 * TriangleIndex + 1 ];
		int VertexIndex3 = Triangles[ 3 * TriangleIndex + 2 ];

		RkVector3 V1 = Vertices[ VertexIndex1 ];
		RkVector3 V2 = Vertices[ VertexIndex2 ];
		RkVector3 V3 = Vertices[ VertexIndex3 ];

		// Signed volume of this tetrahedron
		float Det = rkDet( V1, V2, V3 );

		// Contribution to mass
		Volume += Det;

		// Contribution to centroid
		RkVector3 V4 = V1 + V2 + V3;
		Center += Det * V4;

		// Contribution to inertia monomials
		XX += Det * ( V1.X * V1.X + V2.X * V2.X + V3.X * V3.X + V4.X * V4.X );
		YY += Det * ( V1.Y * V1.Y + V2.Y * V2.Y + V3.Y * V3.Y + V4.Y * V4.Y );
		ZZ += Det * ( V1.Z * V1.Z + V2.Z * V2.Z + V3.Z * V3.Z + V4.Z * V4.Z );
		XY += Det * ( V1.X * V1.Y + V2.X * V2.Y + V3.X * V3.Y + V4.X * V4.Y );
		XZ += Det * ( V1.X * V1.Z + V2.X * V2.Z + V3.X * V3.Z + V4.X * V4.Z );
		YZ += Det * ( V1.Y * V1.Z + V2.Y * V2.Z + V3.Y * V3.Z + V4.Y * V4.Z );
		}
	RK_ASSERT( Volume > 0 );

	// Fetch result
	RkMatrix3 Inertia;
	Inertia.A11 = YY + ZZ;  Inertia.A12 =     -XY;  Inertia.A13 =     -XZ;
	Inertia.A21 =     -XY;  Inertia.A22 = XX + ZZ;  Inertia.A23 =     -YZ;
	Inertia.A31 =     -XZ;  Inertia.A32 =     -YZ;  Inertia.A33 = XX + YY;

	Out.Mass = Density * Volume / 6.0f;
	Out.Center = Center / ( 4.0f * Volume );
	Out.Inertia = ( Density / 120.0f ) * Inertia - rkSteiner( Out.Mass, Out.Center );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RkMatrix3 rkSphereInertia( float Mass, float Radius )
	{
	float Ixx = 0.4f * Mass * Radius * Radius;
	
	return RkMatrix3( Ixx, Ixx, Ixx );
	}


//--------------------------------------------------------------------------------------------------
RkMatrix3 rkCylinderInertia( float Mass, float Radius, float Height )
	{
	float Ixx = Mass * ( 3 * Radius * Radius + Height * Height ) / 12.0f;
	float Iyy = 0.5f * Mass * Radius * Radius;
	
	return RkMatrix3( Ixx, Iyy, Ixx );
	}


//--------------------------------------------------------------------------------------------------
RkMatrix3 rkSteiner( float Mass, const RkVector3& Origin )
	{
	// Usage: Io = Ic + Is and Ic = Io - Is
	float Ixx =  Mass * ( Origin.Y * Origin.Y + Origin.Z * Origin.Z );
	float Iyy =  Mass * ( Origin.X * Origin.X + Origin.Z * Origin.Z );
	float Izz =  Mass * ( Origin.X * Origin.X + Origin.Y * Origin.Y );
	float Ixy = -Mass * Origin.X * Origin.Y;
	float Ixz = -Mass * Origin.X * Origin.Z;
	float Iyz = -Mass * Origin.Y * Origin.Z;

	RkMatrix3 Out;
	Out.A11 = Ixx; Out.A12 = Ixy; Out.A13 = Ixz;
	Out.A21 = Ixy; Out.A22 = Iyy; Out.A23 = Iyz;
	Out.A31 = Ixz; Out.A32 = Iyz; Out.A33 = Izz;

	return Out;
	}