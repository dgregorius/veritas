//--------------------------------------------------------------------------------------------------
/*
	@file		convexcontact.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/contact.h"


//--------------------------------------------------------------------------------------------------
// RkConvexContact
//--------------------------------------------------------------------------------------------------
class RkConvexContact : public RkContact
	{
	public:
		// Construction
		RkConvexContact( RkWorld* World, RkShape* Shape1, RkShape* Shape2 );

		// Collision
		virtual void Collide() override;
		
	private:
		RkContactCache mCache;
	};