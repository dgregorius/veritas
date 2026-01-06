//--------------------------------------------------------------------------------------------------
// basics.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "veritas_testlab/test.h"


//--------------------------------------------------------------------------------------------------
// VsBasicScene1 - Falling Box
//--------------------------------------------------------------------------------------------------
class VsBasicScene1 : public VsTest
	{
	using VsTest::VsTest;
	
	public:
		virtual void Create( VsCamera* Camera ) override
			{
			
// 			IVsHull* Hull = mPlugin->CreateBox( VsVector3( 1.0f, 1.0f, 1.0f ) );
// 			IVsBody* Body = mWorld->CreateBody( VS_STATIC_BODY );
// 			Body->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
// 			Body->SetOrientation( VsQuaternion( 0.0f, 0.0f, 0.0f, 1.0f ) );
// 			Body->CreateHull( Hull );
				
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene1 - Falling Box", VsBasicScene1 );
