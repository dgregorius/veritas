//--------------------------------------------------------------------------------------------------
// qhMath.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "qhMath.h"

//--------------------------------------------------------------------------------------------------
// qhBounds3
//--------------------------------------------------------------------------------------------------
qhBounds3 qhBuildBounds( qhArray< qhVector3 >& Vertices )
	{
	qhBounds3 Bounds = QH_BOUNDS3_EMPTY;
	for ( int i = 0; i < Vertices.Size(); ++i )
		{
		Bounds += Vertices[ i ];
		}

	return Bounds;
	}


//--------------------------------------------------------------------------------------------------
// Pre-processing
//--------------------------------------------------------------------------------------------------
qhVector3 qhComputeCentroid( const qhArray< qhVector3 >& Vertices )
	{
	qhVector3 Centroid = QH_VEC3_ZERO;
	for ( int i = 0; i < Vertices.Size(); ++i )
		{
		Centroid += Vertices[ i ];
		}
	Centroid /= qhReal( Vertices.Size() );

	return Centroid;
	}


//--------------------------------------------------------------------------------------------------
void qhShiftVertices( qhArray< qhVector3 >& Vertices, const qhVector3& Translation )
	{
	if ( Translation == QH_VEC3_ZERO )
		{
		return;
		}

	for ( int i = 0; i < Vertices.Size(); ++i )
		{
		Vertices[ i ] += Translation;
		}
	}


//--------------------------------------------------------------------------------------------------
void qhWeldVertices( qhArray< qhVector3 >& Vertices, const qhVector3& Tolerance )
	{
	// DIRK_TODO: This is O(n^2) - use grid!
	for ( int i = 0; i < Vertices.Size(); ++i )
		{
		for ( int k = Vertices.Size() - 1; k > i; --k )
			{
			qhVector3 Offset = Vertices[ i ] - Vertices[ k ];
			if ( qhAbs( Offset.X ) < Tolerance.X && qhAbs( Offset.Y ) < Tolerance.Y && qhAbs( Offset.Z ) < Tolerance.Z )
				{
				Vertices[ k ] = Vertices.Back();
				Vertices.PopBack();
				}
			}
		}
	}

