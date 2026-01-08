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