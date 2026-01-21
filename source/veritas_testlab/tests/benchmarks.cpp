//--------------------------------------------------------------------------------------------------
// benchmarks.cpp	
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "veritas_testlab/test.h"


//--------------------------------------------------------------------------------------------------
// VsBenchmarkScene1 - 5000 Falling Boxes
//--------------------------------------------------------------------------------------------------
class VsBenchmarkScene1 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create()
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
// VsBenchmarkScene2 - 225 Pyramids
//--------------------------------------------------------------------------------------------------
class VsBenchmarkScene2 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create()
			{
			mWorld->SetAutoSleeping( false );

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
// VsBenchmarkScene3 - Washer
//--------------------------------------------------------------------------------------------------
class VsBenchmarkScene3 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create()
			{
			IVsHull* Ground = mPlugin->CreateBox( VsVector3( 100.0f, 1.0f, 100.0f ) );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			CreateDrum();
			CreateLoad();
			}

		virtual void Update( VsCamera* Camera, float Timestep )
			{
			const float Omega = 10.0f;
			mAngle += Omega * Timestep;
			VsFrame Keyframe = { mWasher->GetPosition(), { 0.0f, 0.0f, sinf( mAngle * VS_DEG2RAD ),cosf( mAngle * VS_DEG2RAD ) } };
			mWasher->SetVelocityFromKeyframe( Keyframe, Timestep );

			VsTest::Update( Camera, Timestep );
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


//--------------------------------------------------------------------------------------------------
// VsBenchmarkScene4 - Junkyard
//--------------------------------------------------------------------------------------------------
class VsBenchmarkScene4 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create()
			{
			IVsHull* Ground = mPlugin->CreateBox( VsVector3( 120.0f, 1.0f, 120.0f ) );
			IVsHull* Wall1 =  mPlugin->CreateBox( VsVector3( -50.0f, 8.0f,  0.0f ), VsVector3( 1.0f, 8.0f, 50.0f ) );
			IVsHull* Wall2 =  mPlugin->CreateBox( VsVector3(  50.0f, 8.0f,  0.0f ), VsVector3( 1.0f, 8.0f, 50.0f ) );
			IVsHull* Wall3 =  mPlugin->CreateBox( VsVector3(  0.0f, 8.0f, -50.0f ), VsVector3( 50.0f, 8.0f, 1.0f ) );
			IVsHull* Wall4 =  mPlugin->CreateBox( VsVector3(  0.0f, 8.0f,  50.0f ), VsVector3( 50.0f, 8.0f, 1.0f ) );

			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );
			IVsShape* WallShape1 = GroundBody->CreateHull( Wall1 );
			WallShape1->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );
			IVsShape* WallShape2 = GroundBody->CreateHull( Wall2 );
			WallShape2->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );
			IVsShape* WallShape3 = GroundBody->CreateHull( Wall3 );
			WallShape3->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );
			IVsShape* WallShape4 = GroundBody->CreateHull( Wall4 );
			WallShape4->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			int Size = VS_DEBUG_TEST ? 6 : 24;
			IVsHull* Convex = mPlugin->CreateConvex( 1.5f, 10 );
			for ( int Y = 0; Y < Size; ++Y )
				{
				for ( int X = 0; X <= 20; ++X )
					{
					for ( int Z = 0; Z <= 20; ++Z )
						{
						IVsBody* JunkBody = mWorld->CreateBody( VS_DYNAMIC_BODY );
						JunkBody->SetPosition( VsVector3( -40.0f + 4.0f * X, 4.0f * Y + mHeight + 1.0f, -40.0f + 4.0f * Z ) );
						JunkBody->CreateHull( Convex );
						}
					}
				}

			mCylinder = mPlugin->CreateCylinder( 4.0f, mHeight );
			mStrider = mWorld->CreateBody( VS_KEYFRAMED_BODY );
			mStrider->SetPosition( VsVector3( mRadius * cosf( mAngle * VS_DEG2RAD ), 0.0f, mRadius * sinf( mAngle * VS_DEG2RAD ) ) );
			mStrider->CreateHull( mCylinder );
			}

		virtual void Update( VsCamera* Camera, float Timestep )
			{
			const float Omega = -6.0f;
			mAngle += Omega * Timestep;
			VsFrame Keyframe = { { mRadius * cosf( mAngle * VS_DEG2RAD ), 0.0f, mRadius * sinf( mAngle * VS_DEG2RAD ) }, { 0.0f, 0.0f, 0.0f, 1.0f } };
			mStrider->SetVelocityFromKeyframe( Keyframe, Timestep );

			VsTest::Update( Camera, Timestep );
			}

	private:
		float mAngle = 0.0f;
		float mRadius = 35.0f;
		float mHeight = 24.0f;
		IVsHull* mCylinder = nullptr;
		IVsBody* mStrider = nullptr;
	};

// Registry
VS_DEFINE_TEST( "Benchmark", "Scene4 - Junkyard", VsOrbit( 45.0f, -25.0f, 150.0f, { 0.0f, 5.0f, 0.0f } ), VsBenchmarkScene4 );


//--------------------------------------------------------------------------------------------------
// VsBenchmarkScene5 - Dominos
//--------------------------------------------------------------------------------------------------
class VsBenchmarkScene5 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create()
			{
			IVsHull* Ground = mPlugin->CreateBox( VsVector3( 50.0f, 1.0f, 50.0f ) );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			float Scale = 1.0f;
			std::vector< VsVector3 > Vertices;
			CreateRingVertices( Vertices, 40.0f, 0.0f );
			const float ShrinkFactor = 0.05f;

			VsVector3 Extent( 0.5f, 1.0f, 0.1f );
			IVsHull* Box = mPlugin->CreateBox( Extent );
			for ( int RingIndex = 0; RingIndex < kRingCount; ++RingIndex )
				{
				for ( int VertexIndex = 0; VertexIndex < kVertexCount; ++VertexIndex )
					{
					IVsBody* Domino = mWorld->CreateBody( VS_DYNAMIC_BODY );
					Domino->SetPosition( Vertices[ VertexIndex ] * Scale + VsVector3( 0.0f, Extent.Y, 0.0f ) );
					VsVector3 Normal = vsNormalize( Vertices[ VertexIndex ] );
					Domino->SetOrientation( vsShortestArc( { 1.0f, 0.0f, 0.0f }, Normal));
					if ( VertexIndex == 0 )
						{
						Domino->SetLinearVelocity( { 0.0f, 0.0f, 4.0f } );
						Domino->SetAngularVelocity( { 4.0f, 0.0f, 0.0f } );
						}
					Domino->CreateHull( Box );

					Scale -= ShrinkFactor / float( kVertexCount );
					}
				}
			}

	private:
		void CreateRingVertices( std::vector< VsVector3 >& Vertices, float Radius, float Y )
			{
			Vertices.resize( kVertexCount );
			for ( int VertexIndex = 0; VertexIndex < kVertexCount; ++VertexIndex )
				{
				float Alpha = float( VertexIndex ) / float( kVertexCount ) * VS_2PI;
				Vertices[ VertexIndex ] = { cosf( Alpha ) * Radius, Y, sinf( Alpha ) * Radius };
				}
			}

		enum { kRingCount = 16, kVertexCount = 200 };
	};

// Registry
VS_DEFINE_TEST( "Benchmark", "Scene5 - Dominos", VsOrbit( 85.0f, -12.0f, 55.0f, { 0.0f, 1.0f, 0.0f } ), VsBenchmarkScene5 );


//--------------------------------------------------------------------------------------------------
// VsBenchmarkScene6 - Convex Stacks
//--------------------------------------------------------------------------------------------------
class VsBenchmarkScene6 : public VsTest
	{
	using VsTest::VsTest;

	public:
		virtual void Create()
			{
			mWorld->SetAutoSleeping( false );

			IVsHull* Ground = mPlugin->CreateBox( { 50.0f, 1.0f, 50.0f } );
			IVsBody* GroundBody = mWorld->CreateBody( VS_STATIC_BODY );
			GroundBody->SetPosition( { 0.0f, -1.0f, 0.0f } );
			IVsShape* GroundShape = GroundBody->CreateHull( Ground );
			GroundShape->SetColor( { 0.3f, 0.3f, 0.3f, 1.0f } );

			IVsHull* Cone = mPlugin->CreateCone( 0.9f, 1.8f, 2.0f, 32 );
			for ( float X = -32.0f; X <= 32.0f; X += 4 )
				{
				for ( float Y = 0.0f; Y <= 16.0f; Y += 2.0f )
					{
					for ( float Z = -32.0f; Z <= 32.0f; Z += 4 )
						{
						IVsBody* Body = mWorld->CreateBody( VS_DYNAMIC_BODY );
						Body->SetPosition( { X, Y, Z } );
						Body->CreateHull( Cone );
						}
					}
				}
			}
	};

// Registry
VS_DEFINE_TEST( "Benchmark", "Scene6 - Convex Stack", VsOrbit( 45.0f, -20.0f, 100.0f, { 0.0f, 4.0f, 0.0f } ), VsBenchmarkScene6 );