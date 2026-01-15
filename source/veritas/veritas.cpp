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
IVsHull* IVsPlugin::CreateBox( VsVector3 Extent )
	{
	int VertexCount = 8;
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

	return CreateHull( VertexCount, Vertices );
	}