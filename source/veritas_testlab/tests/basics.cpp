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
	public:
		VsBasicScene1( IVsPlugin* Plugin )
			: VsTest( Plugin )
			{
			if ( IVsWorld* World = Plugin->CreateWorld() )
				{
				World->SetGravity( VsVector3( 0.0f, -10.0f, 0.0f ) );

				IVsHull* Hull = Plugin->CreateBox( VsVector3( 1.0f, 1.0f, 1.0f ) );
				IVsBody* Body = World->CreateBody( VS_STATIC_BODY );
				Body->SetPosition( VsVector3( 0.0f, -1.0f, 0.0f ) );
				Body->SetOrientation( VsQuaternion( 0.0f, 0.0f, 0.0f, 1.0f ) );
				IVsShape* Shape = Body->CreateHull( Hull );

// 				Body->DestroyShape( Shape );
// 				World->DestroyBody( Body );
// 				Plugin->DestroyHull( Hull );
				Plugin->DestroyWorld( World );
				}
			}
		
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene1 - Falling Box", VsBasicScene1 );
