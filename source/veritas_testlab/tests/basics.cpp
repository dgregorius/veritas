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
		using VsTest::VsTest;
		
	};

// Registry
VS_DEFINE_TEST( "Basics", "Scene1 - Falling Box", VsBasicScene1 );
