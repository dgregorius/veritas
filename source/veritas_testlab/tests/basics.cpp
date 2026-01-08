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
			IVsHull* Ground = mPlugin->CreateBox( VsVector3( 15.0f, 1.0f, 15.0f ) );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			if ( GroundBody )
				{
				GroundBody->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
				IVsShape* GroundShape = GroundBody->CreateHull( Ground );
				GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );
				}

			IVsHull* Box = mPlugin->CreateBox( VsVector3( 1.0f, 1.0f, 1.0f ) );
			IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
			if ( BoxBody )
				{
				BoxBody->SetPosition( VsVector3( 0.0f, 15.0f, 0.0f ) );
				BoxBody->CreateHull( Box );
				}
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene1 - Falling Box", VsBasicScene1 );
