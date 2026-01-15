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


//--------------------------------------------------------------------------------------------------
// VsBenchmarkScene3
//--------------------------------------------------------------------------------------------------
class VsBenchmarkScene3 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create( VsCamera* Camera )
			{
			IVsHull* Ground = mPlugin->CreateBox( VsVector3( 100.0f, 1.0f, 100.0f ) );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			CreateDrum();
			CreateLoad();
			}

		virtual void Update( double Time, float ElapsedTime )
			{
			const float Omega = 10.0f;
			float TargetAngle = mAngle;
			VsFrame Keyframe = { mWasher->GetPosition(), { 0.0f, 0.0f, sinf( TargetAngle * VS_DEG2RAD ),cosf( TargetAngle * VS_DEG2RAD ) } };
			mAngle += Omega * ElapsedTime;

			mWasher->SetVelocityFromKeyframe( Keyframe, ElapsedTime );

			VsTest::Update( Time, ElapsedTime );
			}

	private:
		void CreateDrum()
			{
			// Geometry
			float InnerRadius = 16.0f;
			float OuterRadius = 18.0f;
			VsVector3 Extent( 0.0f, 0.0f, 10.0f );

			// Create washer body
			mWasher = mWorld->CreateBody( VS_KEYFRAMED_BODY );
			mWasher->SetPosition( VsVector3( 0.0f, OuterRadius + 1.0f, 0.0f ) );

			// Create washer shapes
			int Slices = 36;
			int VertexCount = Slices;
			VsVector3* InnerRing = (VsVector3*)alloca( VertexCount * sizeof( VsVector3 ) );
			VS_ASSERT( InnerRing );
			VsVector3* OuterRing = (VsVector3*)alloca( VertexCount * sizeof( VsVector3 ) );
			VS_ASSERT( OuterRing );

			float Alpha = 0.0f;
			float DeltaAlpha = VS_2PI / Slices;
			for ( int Slice = 0; Slice < Slices; ++Slice )
				{
				float SinAlpha = sinf( Alpha );
				float CosAlpha = cosf( Alpha );
				InnerRing[ Slice ] = VsVector3( InnerRadius * CosAlpha, InnerRadius * SinAlpha, 0.0f );
				OuterRing[ Slice ] = VsVector3( OuterRadius * CosAlpha, OuterRadius * SinAlpha, 0.0f );

				Alpha += DeltaAlpha;
				}

			VsVector3 InnerVertex1 = InnerRing[ Slices - 1 ];
			VsVector3 OuterVertex1 = OuterRing[ Slices - 1 ];
			for ( int Slice = 0; Slice < Slices; ++Slice )
				{
				VsVector3 InnerVertex2 = InnerRing[ Slice ];
				VsVector3 OuterVertex2 = OuterRing[ Slice ];

				// Inflate against tunneling
				const float Expansion = 0.25f;
				VsVector3 InnerDelta = Expansion * vsNormalize( InnerVertex2 - InnerVertex1 );
				VsVector3 OuterDelta = Expansion * vsNormalize( OuterVertex2 - OuterVertex1 );

				// Washer piece
				{
				VsVector3 Vertices[] =
					{
					// Front
					( OuterVertex1 - OuterDelta ) + Extent,
					( InnerVertex1 - InnerDelta ) + Extent,
					( OuterVertex2 + OuterDelta ) + Extent,
					( InnerVertex2 + InnerDelta ) + Extent,

					// Back
					( OuterVertex1 - OuterDelta ) - Extent,
					( InnerVertex1 - InnerDelta ) - Extent,
					( OuterVertex2 + OuterDelta ) - Extent,
					( InnerVertex2 + InnerDelta ) - Extent
					};

				IVsHull* Hull = mPlugin->CreateHull( std::size( Vertices ), Vertices );
				mWasher->CreateHull( Hull );
				}

				// Bump
				if ( Slice % 9 == 0 )
					{
					float Scale = ( OuterRadius - InnerRadius );
					VsVector3 Delta1 = Scale * vsNormalize( InnerVertex1 - OuterVertex1 );
					VsVector3 Delta2 = Scale * vsNormalize( InnerVertex2 - OuterVertex2 );

					VsVector3 Vertices[] =
						{
						// Front
						( OuterVertex1 - OuterDelta ) + Extent,
						( InnerVertex1 - InnerDelta ) + Delta1 + Extent,
						( OuterVertex2 + OuterDelta ) + Extent,
						( InnerVertex2 + InnerDelta ) + Delta2 + Extent,

						// Back
						( OuterVertex1 - OuterDelta ) - Extent,
						( InnerVertex1 - InnerDelta ) + Delta1 - Extent,
						( OuterVertex2 + OuterDelta ) - Extent,
						( InnerVertex2 + InnerDelta ) + Delta2 - Extent
						};

					IVsHull* Hull = mPlugin->CreateHull( std::size( Vertices ), Vertices );
					mWasher->CreateHull( Hull );
					}

				InnerVertex1 = InnerVertex2;
				OuterVertex1 = OuterVertex2;
				}
			}

		void CreateLoad()
			{
			float Extent = 0.2f;
			IVsHull* Box = mPlugin->CreateBox( VsVector3( Extent, Extent, Extent ) );

			int GridCount = VS_DEBUG_TEST ? 10 : 20;
			float X = -2.0f * Extent * GridCount;
			for ( int GridX = 0; GridX < GridCount; ++GridX )
				{
				float Y = -2.0f * Extent * GridCount + 20.0f;
				for ( int GridY = 0; GridY < GridCount; ++GridY )
					{
					float Z = -2.0f * Extent * GridCount;
					for ( int GridZ = 0; GridZ < GridCount; ++GridZ )
						{
						IVsBody* Load = mWorld->CreateBody( VS_DYNAMIC_BODY );
						Load->SetPosition( VsVector3( X, Y, Z ) );
						Load->CreateHull( Box );

						Z += 4.0f * Extent;
						}

					Y += 4.0f * Extent;
					}

				X += 4.0f * Extent;
				}
			}

	private:
		IVsBody* mWasher = nullptr;
		float mAngle = 0.0f;
			
	};

// Registry
VS_DEFINE_TEST( "Benchmark", "Scene3 - Washer", VsOrbit( 0.0f, -20.0f, 75.0f, { 0.0f, 10.0f, 0.0f } ), VsBenchmarkScene3 );