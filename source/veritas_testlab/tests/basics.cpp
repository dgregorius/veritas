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
			GroundBody->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );


			IVsHull* Box = mPlugin->CreateBox( VsVector3( 1.0f, 1.0f, 1.0f ) );
			IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
			BoxBody->SetPosition( VsVector3( 0.0f, 15.0f, 0.0f ) );
			BoxBody->CreateHull( Box );
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene1 - Falling Box", VsOrbit( 45.0f, -20.0f, 40.0f ), VsBasicScene1);


//--------------------------------------------------------------------------------------------------
// VsBasicScene2 - Small stack
//--------------------------------------------------------------------------------------------------
class VsBasicScene2 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create( VsCamera* Camera ) override
			{
			IVsHull* Ground = mPlugin->CreateBox( VsVector3( 15.0f, 1.0f, 15.0f ) );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			IVsHull* Box = mPlugin->CreateBox( VsVector3( 1.0f, 1.0f, 1.0f ) );
			for ( int Index = 0; Index < 5; ++Index )
				{
				IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
				BoxBody->SetPosition( VsVector3( 0.0f, 2.5f * Index + 1.5f, 0.0f ) );
				BoxBody->CreateHull( Box );
				}
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene2 - Small stack", VsOrbit( 45.0f, -20.0f, 30.0f, { 0.0f, 4.0f, 0.0f } ), VsBasicScene2 );


//--------------------------------------------------------------------------------------------------
// VsBasicScene3 - Small pyramid
//--------------------------------------------------------------------------------------------------
class VsBasicScene3 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create( VsCamera* Camera ) override
			{
			IVsHull* Ground = mPlugin->CreateBox( VsVector3( 25.0f, 1.0f, 25.0f ) );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			IVsHull* Box = mPlugin->CreateBox( VsVector3( 1.0f, 1.0f, 1.0f ) );
			for ( int Row = 0; Row < 12; ++Row )
				{
				for ( int Column = 0; Column < 12 - Row; ++Column )
					{
					IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
					BoxBody->SetPosition( VsVector3( -11.0f + 2.0f * Column + Row, 1.5f + 2.5f * Row, 0.0f ) );
					BoxBody->CreateHull( Box );
					}
				}
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene3 - Small pyramid", VsOrbit( 0.0f, -30.0f, 50.0f, { 0.0f, 6.0f, 0.0f } ), VsBasicScene3);
