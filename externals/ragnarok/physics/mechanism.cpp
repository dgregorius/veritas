//--------------------------------------------------------------------------------------------------
// mechanism.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "mechanism.h"
#include "body.h"
#include "joint.h"

// Internal 
#include "solver.h"
#include "jointsolver.h"


//--------------------------------------------------------------------------------------------------
// LDLT Decomposition
//--------------------------------------------------------------------------------------------------
// static void rkFillNode( RkArray< float >& Node, RkJoint* Joint )
// 	{
// 
// 	}
// 
// 
// 
// //--------------------------------------------------------------------------------------------------
// static void rkFillEdge( RkArray< float >& Edge, RkJoint* Joint1, RkJoint* Joint2 )
// 	{
// 
// 	}


//--------------------------------------------------------------------------------------------------
// RkMechanism
//--------------------------------------------------------------------------------------------------
RkMechanism::RkMechanism( RkWorld* World, const RkArray< RkJoint* >& Joints )
	{
	RK_ASSERT( World );
	mWorld = World;

	mJoints = Joints;
	for ( RkJoint* Joint : mJoints )
		{
		RK_ASSERT( !Joint->GetMechanism() );
		Joint->SetMechanism( this );
		}
	}

//--------------------------------------------------------------------------------------------------
RkMechanism::~RkMechanism()
	{
	for ( RkJoint* Joint : mJoints )
		{
		RK_ASSERT( Joint->GetMechanism() == this );
		Joint->SetMechanism( nullptr );
		}
	}


//--------------------------------------------------------------------------------------------------
RkWorld* RkMechanism::GetWorld() const
	{
	return mWorld;
	}


//--------------------------------------------------------------------------------------------------
int RkMechanism::LoadConstraints( uint8* ConstraintBuffer, RkVector3* VelocityBuffer, const RkCompliance& Compliance, float Timestep )
	{
	return 0;
	}


//--------------------------------------------------------------------------------------------------
int RkMechanism::SolveConstraints( uint8* ConstraintBuffer, RkVector3* VelocityBuffer, const RkCompliance& Compliance )
	{
	return 0;
	}


//--------------------------------------------------------------------------------------------------
int RkMechanism::SaveConstraints( uint8* ConstraintBuffer )
	{
	return 0;
	}


//--------------------------------------------------------------------------------------------------
int RkMechanism::Factor( )
	{
// 	int JointCount = mJoints.Size();
// 
// 	// Populate
// 	int mMatrixSize = 0;
// 	RkArray< float > mMatrix;
// 	mMatrix.Reserve( mMatrixSize * mMatrixSize );
// 
// 	int mColumnCount = JointCount;
// 	RkArray< int > mColumnWidth;
// 	mColumnWidth.Reserve( mColumnCount );
// 	RkArray< int > mColumnHeight;
// 	mColumnHeight.Reserve( mColumnCount );
// 	RkArray< int > mColumnOffset;
// 	mColumnOffset.Reserve( mColumnCount );
// 
// 	int ColumnHeight = mMatrixSize;
// 	for ( int JointIndex1 = 0; JointIndex1 < JointCount; ++JointIndex1 )
// 		{
// 		RkJoint* Joint1 = mJoints[ JointIndex1 ];
// 
// 		const int Dimensions[] = { 3, 5, 5, 6 };
// 		int ColumnWidth = Dimensions[ Joint1->GetType() ];
// 		mColumnWidth.PushBack( ColumnWidth );
// 		mColumnHeight.PushBack( ColumnHeight );
// 		ColumnHeight -= ColumnWidth;
// 		int ColumnOffset = mMatrix.Size();
// 		mColumnOffset.PushBack( ColumnOffset );
// 		
// 		for ( int JointIndex2 = JointIndex1; JointIndex2 < JointCount; ++JointIndex2 )
// 			{
// 			RkJoint* Joint2 = mJoints[ JointIndex2 ];
// 
// 			if ( Joint1 == Joint2 )
// 				{
// 				rkFillNode( mMatrix, Joint1 );
// 				}
// 			else
// 				{
// 				rkFillEdge( mMatrix, Joint1, Joint2 );
// 				}
// 			}
// 		}
// 
// 	// Factor
// 	for ( int ColumnIndex = 0; ColumnIndex < mColumnCount; ++ColumnIndex )
// 		{
// 		int ColumnWidth = mColumnWidth[ ColumnIndex ];
// 		int ColumnHeight = mColumnHeight[ ColumnIndex ];
// 		int ColumnOffset = mColumnOffset[ ColumnIndex ];
// 		float* Column = mMatrix.Data() + ColumnOffset;
// 
// 		int NodeSize = ColumnWidth;
// 		float* Node = Column;
// 		//rkInvert( NodeSize, Node );
// 
// 		int EdgeWidth = ColumnWidth;
// 		int EdgeHeight = ColumnHeight - NodeSize;
// 		float* Edge = Column + NodeSize * NodeSize;
// 		}


	return 0;
	}


//--------------------------------------------------------------------------------------------------
int RkMechanism::Solve( RkVector3* VelocityBuffer )
	{
	return 0;
	}
