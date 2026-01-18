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
		virtual void Create() override
			{
			IVsHull* Ground = mPlugin->CreateBox( { 15.0f, 1.0f, 15.0f } );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( { 0.0f, -1.0f, 0.0f } );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );


			IVsHull* Box = mPlugin->CreateBox( { 1.0f, 1.0f, 1.0f } );
			IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
			BoxBody->SetPosition( { 0.0f, 15.0f, 0.0f } );
			BoxBody->CreateHull( Box );
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene1 - Falling Box", VsOrbit( 45.0f, -20.0f, 40.0f ), VsBasicScene1 );


//--------------------------------------------------------------------------------------------------
// VsBasicScene2 - Small stack
//--------------------------------------------------------------------------------------------------
class VsBasicScene2 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create() override
			{
			IVsHull* Ground = mPlugin->CreateBox( { 15.0f, 1.0f, 15.0f } );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( { 0.0f, -1.0f, 0.0f } );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			IVsHull* Box = mPlugin->CreateBox( { 1.0f, 1.0f, 1.0f } );
			for ( int Index = 0; Index < 5; ++Index )
				{
				IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
				BoxBody->SetPosition( { 0.0f, 2.5f * Index + 1.5f, 0.0f }  );
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
		virtual void Create() override
			{
			IVsHull* Ground = mPlugin->CreateBox( { 25.0f, 1.0f, 25.0f } );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( { 0.0f, -1.0f, 0.0f } );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			IVsHull* Box = mPlugin->CreateBox( { 1.0f, 1.0f, 1.0f } );
			for ( int Row = 0; Row < 12; ++Row )
				{
				for ( int Column = 0; Column < 12 - Row; ++Column )
					{
					IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
					BoxBody->SetPosition( { -11.0f + 2.0f * Column + Row, 1.5f + 2.5f * Row, 0.0f } );
					BoxBody->CreateHull( Box );
					}
				}
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene3 - Small pyramid", VsOrbit( 0.0f, -30.0f, 50.0f, { 0.0f, 6.0f, 0.0f } ), VsBasicScene3 );


//--------------------------------------------------------------------------------------------------
// VsBasicScene4 - Inclined Plane
//--------------------------------------------------------------------------------------------------
class VsBasicScene4 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create() override
			{
			IVsHull* Ground = mPlugin->CreateBox( { 50.0f, 1.0f, 50.0f } );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( { 0.0f, -1.0f, 0.0f } );
			GroundBody->SetFriction( 1.0f );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			IVsHull* Plane = mPlugin->CreateBox( { 16.0f, 0.5f, 10.0f } );
			IVsBody* PlaneBody = mWorld->CreateBody( VS_STATIC_BODY );
			PlaneBody->SetPosition( { 0.0f, 7.5f, -5.0f } );
			PlaneBody->SetOrientation( { sinf( 20.0f * VS_DEG2RAD ), 0.0f, 0.0f, cosf( 20.0f * VS_DEG2RAD ) } );
			PlaneBody->SetFriction( 1.0f );
			IVsShape* PlaneShape = PlaneBody->CreateHull( Plane );
			PlaneShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );
			
			IVsHull* Box = mPlugin->CreateBox( VsVector3( 1.0f, 1.0f, 1.0f ) );
			for ( int Index = 0; Index < 5; ++Index )
				{
				IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
				BoxBody->SetPosition( { -10.0f + 5.0f * Index, 15.75f, -10.6f } );
				BoxBody->SetFriction( ( Index + 1 ) * 0.2f );
				BoxBody->CreateHull( Box );
				}
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene4 - Inclined Plane", VsOrbit( -55.0f, -30.0f, 60.0f, { 0.0f, 7.5f, 0.0f } ), VsBasicScene4 );


//--------------------------------------------------------------------------------------------------
// VsBasicScene5 - Sliding Boxes
//--------------------------------------------------------------------------------------------------
class VsBasicScene5 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create() override
			{
			IVsHull* Ground = mPlugin->CreateBox( { 75.0f, 1.0f, 75.0f } );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( { 0.0f, -1.0f, 0.0f } );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			IVsHull* Box = mPlugin->CreateBox( { 1.0f, 1.0f, 1.0f } );
			for ( int Index = 0; Index < 32; ++Index )
				{
				float Alpha = VS_PI / 16.0f * Index;
				IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
				BoxBody->SetPosition( { 15.0f * cosf( Alpha ), 1.0f, 15.0f * sinf( Alpha ) } );
				BoxBody->SetOrientation( { 0.0f, sinf( -Alpha / 2.0f ), 0.0f, cosf( -Alpha / 2.0f ) } );
				BoxBody->SetLinearVelocity( { 20.0f * cosf( Alpha ), 0.0f, 20.0f * sinf( Alpha ) } );
				BoxBody->CreateHull( Box );
				}
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene5 - Sliding Boxes", VsOrbit( 45.0f, -35.0f, 140.0f, { 0.0f, 1.0f, 0.0f } ), VsBasicScene5 );


//--------------------------------------------------------------------------------------------------
// VsBasicScene6 - Spinning Books
//--------------------------------------------------------------------------------------------------
class VsBasicScene6 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create() override
			{
			mWorld->SetGravity( { 0.0f, 0.0f, 0.0f } );

			IVsHull* Book = mPlugin->CreateBox( { 0.35f, 0.08f, 0.5f } );
			IVsBody* BookBody1 = mWorld->CreateBody( VS_DYNAMIC_BODY );
			BookBody1->SetPosition( { -2.0f, 2.0f, 0.0f } );
			BookBody1->SetAngularVelocity( { 5.0f, 0.01f, 0.01f } );
			BookBody1->CreateHull( Book );
		
			IVsBody* BookBody2 = mWorld->CreateBody( VS_DYNAMIC_BODY );
			BookBody2->SetPosition( { 0.0f, 2.0f, 0.0f } );
			BookBody2->SetAngularVelocity( { 0.01f, 5.0f, 0.01f } );
			BookBody2->CreateHull( Book );

			IVsBody* BookBody3 = mWorld->CreateBody( VS_DYNAMIC_BODY );
			BookBody3->SetPosition( { 2.0f, 2.0f, 0.0f } );
			BookBody3->SetAngularVelocity( { 0.01f, 0.01f, 5.0f } );
			BookBody3->CreateHull( Book );
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene6 - Spinning Books", VsOrbit( 0.0f, -30.0f, 10.0f, { 0.0f, 1.0f, 0.0f } ), VsBasicScene6 );


//--------------------------------------------------------------------------------------------------
// VsBasicScene7 - Dzhanibekov Effect
//--------------------------------------------------------------------------------------------------
class VsBasicScene7 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create() override
			{
			mWorld->SetGravity( { 0.0f, 0.0f, 0.0f } );

			IVsHull* Box = mPlugin->CreateBox( { 1.0f, 0.05f, 0.1f } );
			IVsHull* Cylinder = mPlugin->CreateCylinder( 0.15f, 0.6f, 32 );

			IVsBody* Body = mWorld->CreateBody( VS_DYNAMIC_BODY );
			Body->SetPosition( { 0.0f, 2.0f, 0.0f } );
			Body->SetOrientation( { sinf( 45.0f * VS_DEG2RAD ), 0.0f, 0.0f, cosf( 45.0f * VS_DEG2RAD ) } );
			Body->SetAngularVelocity( { 0.01f, 0.01f, 5.0f } );
			Body->CreateHull( Cylinder );
			Body->CreateHull( Box );
			}
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene7 - Dzhanibekov Effect", VsOrbit( 0.0f, -30.0f, 10.0f, { 0.0f, 1.0f, 0.0f } ), VsBasicScene7 );


