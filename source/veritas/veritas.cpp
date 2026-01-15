//--------------------------------------------------------------------------------------------------
// veritas.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "veritas.h"


//--------------------------------------------------------------------------------------------------
// VsColor
//--------------------------------------------------------------------------------------------------
bool operator==( const VsColor& Lhs, const VsColor& Rhs )
	{
	return Lhs.R == Rhs.R && Lhs.G == Rhs.G && Lhs.B == Rhs.B && Lhs.A == Rhs.A;
	}


//--------------------------------------------------------------------------------------------------
bool operator!=( const VsColor& Lhs, const VsColor& Rhs )
	{
	return Lhs.R != Rhs.R || Lhs.G != Rhs.G || Lhs.B != Rhs.B || Lhs.A != Rhs.A;
	}


//--------------------------------------------------------------------------------------------------
// VsVector3
//--------------------------------------------------------------------------------------------------
VsVector3 operator+( const VsVector3& V1, const VsVector3& V2 )
	{
	VsVector3 Out;
	Out.X = V1.X + V2.X;
	Out.Y = V1.Y + V2.Y;
	Out.Z = V1.Z + V2.Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
VsVector3 operator-( const VsVector3& V1, const VsVector3& V2 )
	{
	VsVector3 Out;
	Out.X = V1.X - V2.X;
	Out.Y = V1.Y - V2.Y;
	Out.Z = V1.Z - V2.Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
VsVector3 operator*( float F, const VsVector3& V )
	{
	VsVector3 Out;
	Out.X = F * V.X;
	Out.Y = F * V.Y;
	Out.Z = F * V.Z;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
VsVector3 operator*( const VsVector3& V, float F )
	{
	VsVector3 Out;
	Out.X = V.X * F;
	Out.Y = V.Y * F;
	Out.Z = V.Z * F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
VsVector3 operator/( const VsVector3& V, float F )
	{
	VsVector3 Out;
	Out.X = V.X / F;
	Out.Y = V.Y / F;
	Out.Z = V.Z / F;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
VsVector3 vsCross( const VsVector3& V1, const VsVector3& V2 )
	{
	VsVector3 Out;
	Out.X = V1.Y * V2.Z - V1.Z * V2.Y;
	Out.Y = V1.Z * V2.X - V1.X * V2.Z;
	Out.Z = V1.X * V2.Y - V1.Y * V2.X;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
VsVector3 vsNormalize( const VsVector3& V )
	{
	float LengthSq = vsDot( V, V );
	if ( LengthSq > 1000.0f * FLT_MIN )
		{
		return V / sqrtf( LengthSq );
		}

	return { 0.0f, 0.0f, 0.0f };
	}


//--------------------------------------------------------------------------------------------------
VsVector3 vsPerp( const VsVector3& V )
	{
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	return vsNormalize( fabsf( V.X ) >= 0.57735f ? VsVector3( V.Y, -V.X, 0.0f ) : VsVector3( 0.0f, V.Z, -V.Y ) );
	}


//--------------------------------------------------------------------------------------------------
float vsDot( const VsVector3& V1, const VsVector3& V2 )
	{
	return V1.X * V2.X + V1.Y * V2.Y + V1.Z * V2.Z;
	}


//--------------------------------------------------------------------------------------------------
float vsLength( const VsVector3& V )
	{
	return sqrtf( vsDot( V, V ) );
	}


//--------------------------------------------------------------------------------------------------
// IVsBody
//--------------------------------------------------------------------------------------------------
void IVsBody::DeleteShape( IVsShape* Shape )
	{
	delete Shape;
	}


//--------------------------------------------------------------------------------------------------
// IVsPlugin
//--------------------------------------------------------------------------------------------------
IVsHull* IVsPlugin::CreateBox( const VsVector3& Extent )
	{
	VsVector3 Vertices[] = 
		{
		VsVector3(  Extent.X,  Extent.Y,  Extent.Z ),
		VsVector3( -Extent.X,  Extent.Y,  Extent.Z ),
		VsVector3( -Extent.X, -Extent.Y,  Extent.Z ),
		VsVector3(  Extent.X, -Extent.Y,  Extent.Z ),
		VsVector3(  Extent.X,  Extent.Y, -Extent.Z ),
		VsVector3( -Extent.X,  Extent.Y, -Extent.Z ),
		VsVector3( -Extent.X, -Extent.Y, -Extent.Z ),
		VsVector3(  Extent.X, -Extent.Y, -Extent.Z )
		};

	return CreateHull( std::size( Vertices ), Vertices );
	}


//--------------------------------------------------------------------------------------------------
IVsHull* IVsPlugin::CreateBox( const VsVector3& Center, const VsVector3& Extent )
	{
	VsVector3 Vertices[] = 
		{
		Center + VsVector3(  Extent.X,  Extent.Y,  Extent.Z ),
		Center + VsVector3( -Extent.X,  Extent.Y,  Extent.Z ),
		Center + VsVector3( -Extent.X, -Extent.Y,  Extent.Z ),
		Center + VsVector3(  Extent.X, -Extent.Y,  Extent.Z ),
		Center + VsVector3(  Extent.X,  Extent.Y, -Extent.Z ),
		Center + VsVector3( -Extent.X,  Extent.Y, -Extent.Z ),
		Center + VsVector3( -Extent.X, -Extent.Y, -Extent.Z ),
		Center + VsVector3(  Extent.X, -Extent.Y, -Extent.Z )
		};

	return CreateHull( std::size( Vertices ), Vertices );
	}


//--------------------------------------------------------------------------------------------------
IVsHull* IVsPlugin::CreateCylinder( float Radius, float Height, int Slices )
	{
	VS_ASSERT( Height > 0.0f );
	VS_ASSERT( Radius > 0.0f );
	VS_ASSERT( 3 <= Slices && Slices <= 32 );

	int VertexCount = 2 * Slices;
	VsVector3* Vertices = (VsVector3*)alloca( VertexCount * sizeof( VsVector3 ) );
	VS_ASSERT( Vertices );

	float Alpha = 0.0f;
	float DeltaAlpha = VS_2PI / Slices;

	for ( int Index = 0; Index < Slices; ++Index )
		{
		float SinAlpha = sinf( Alpha );
		float CosAlpha = cosf( Alpha );

		Vertices[ 2 * Index + 0 ] = VsVector3( Radius * CosAlpha, 0.0f, Radius * SinAlpha );
		Vertices[ 2 * Index + 1 ] = VsVector3( Radius * CosAlpha, Height, Radius * SinAlpha );

		Alpha += DeltaAlpha;
		}

	return CreateHull( VertexCount, Vertices );
	}


//--------------------------------------------------------------------------------------------------
IVsHull* IVsPlugin::CreateConvex( float Radius, int VertexCount )
	{
	// Golden ratio
	const float Phi = ( 1.0f + sqrtf( 5.0f ) ) / 2.0f;

	// Random points on sphere (Fibonacci lattice)
	VsVector3* Vertices = (VsVector3*)alloca( VertexCount * sizeof( VsVector3 ) );
	for ( int VertexIndex = 0; VertexIndex < VertexCount; ++VertexIndex )
		{
		float Theta = VS_2PI * VertexIndex / Phi;							// Azimuthal angle
		float Z = 1.0f - ( 2.0f * VertexIndex + 1.0f ) / VertexCount;		// Z coordinate
		float Radius_XY = sqrtf( 1.0f - Z * Z );							// Radius in xy-plane

		Vertices[ VertexIndex ].X = Radius * Radius_XY * cosf( Theta );
		Vertices[ VertexIndex ].Y = Radius * Radius_XY * sinf( Theta );
		Vertices[ VertexIndex ].Z = Radius * Z;
		}

	return CreateHull( VertexCount, Vertices );
	}
