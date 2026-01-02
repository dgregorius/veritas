//--------------------------------------------------------------------------------------------------
// basics.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "veritas_testlab/test.h"


//--------------------------------------------------------------------------------------------------
// TlBasicScene1 - Falling Box
//--------------------------------------------------------------------------------------------------
class TlBasicScene1 : public TlTest
	{
	public:
		explicit TlBasicScene1()
			{
// 			int BodyIndex1 = mScene.GetBodyCount();
// 			TlBody& Body1 = mScene.AddBody();
// 			Body1.Position = { 0.0f, -1.0f, 0.0f };
// 			Body1.AddHullShape( mGround );
// 
// 			int BodyIndex2 = mScene.GetBodyCount();
// 			TlBody& Body2 = mScene.AddBody();
// 			Body2.Type = kDynamicBody;
// 			Body2.Position = { 0.0f, -1.0f, 0.0f };
// 			Body2.Orientation = { 1.0f, 0.0f, 0.0f, 0.0f };
// 			Body2.AddSphereShape( { 0.0f, 0.0f, 0.0f }, 1.0f );
// 		
// 			TlSphericalJoint& Joint = mScene.AddSphericalJoint( BodyIndex1, TlFrame{}, BodyIndex2, TlFrame{} );
// 			Joint.EnableSwingLimit = true;
// 			Joint.MaxSwingAngle = 45.0f;
// 			Joint.EnableTwistLimt = true;
// 			Joint.MinTwistAngle = -90.0f;
// 			Joint.MaxTwistAngle = -90.0f;
			}

		virtual ~TlBasicScene1() override
			{
			
			}

	private:
		TlGeometry* mGround = nullptr;
		TlGeometry* mBox = nullptr;

	};

// Registry
TL_DEFINE_TEST( "Basics", "Scene1 - Falling Box", TlBasicScene1 );
