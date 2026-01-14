//--------------------------------------------------------------------------------------------------
// benchmarks.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "veritas_testlab/test.h"


//--------------------------------------------------------------------------------------------------
// VsBenchmarkScene1
//--------------------------------------------------------------------------------------------------
class VsBenchmarkScene1 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create( VsCamera* Camera )
			{
			IVsHull* Ground = mPlugin->CreateBox( VsVector3( 250.0f, 1.0f, 250.0f ) );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			int Height = VS_DEBUG_TEST ? 10 : 50;
			IVsHull* Box = mPlugin->CreateBox( VsVector3( 1.0f, 1.0f, 1.0f ) );
			for ( int Y = 0; Y < Height; ++Y )
				{
				for ( int X = 0; X < 10; ++X )
					{
					for ( int Z = 0; Z < 10; ++Z )
						{
						IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
						BoxBody->SetPosition( VsVector3( -20.0f + 4.0f * X, 4.0f * Y + 5.0f, -20.0f + 4.0f * Z ) );
						BoxBody->CreateHull( Box );
						}
					}
				}
			}
	};

// Registry
VS_DEFINE_TEST( "Benchmark", "Scene1 - 5000 Falling Boxes", VsOrbit( 45.0f, -25.0f, 300.0f, { 0.0f, 5.0f, 0.0f } ), VsBenchmarkScene1 );


//--------------------------------------------------------------------------------------------------
// VsBenchmarkScene2
//--------------------------------------------------------------------------------------------------
class VsBenchmarkScene2 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create( VsCamera* Camera )
			{
			IVsHull* Ground = mPlugin->CreateBox( VsVector3( 180.0f, 1.0f, 180.0f ) );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			IVsHull* Box = mPlugin->CreateBox( VsVector3( 1.0f, 1.0f, 1.0f ) );
			for ( float X = -154.0f; X <= 154.0f; X += 22.0f )
				{
				for ( float Z = -154.0f; Z <= 154.0f; Z += 22.0f )
					{
					AddPyramid( VsVector3( X, 0.0f, Z ), Box );
					}
				}
			}

	private:
		void AddPyramid( const VsVector3& Position, const IVsHull* Box )
			{
			for ( int Row = 0; Row < 10; ++Row )
				{
				for ( int Column = 0; Column < 10 - Row; ++Column )
					{
					IVsBody* BoxBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
					BoxBody->SetPosition( VsVector3( Position.X -10.0f + 2.0f * Column + Row, Position.Y + 1.0f + 2.0f * Row, Position.Z ) );
					BoxBody->CreateHull( Box );
					}
				}
			}

	};

// Registry
VS_DEFINE_TEST( "Benchmark", "Scene2 - 225 Pyramids", VsOrbit( 15.0f, -25.0f, 360.0f, { 0.0f, 6.0f, 0.0f } ), VsBenchmarkScene2 );