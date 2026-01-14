//--------------------------------------------------------------------------------------------------
/*
	@file		meshcontact.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/physics/contact.h"


//--------------------------------------------------------------------------------------------------
// RkMeshContact
//--------------------------------------------------------------------------------------------------
class RkMeshContact : public RkContact
	{
	public:
		// Construction
		RkMeshContact( RkWorld* World, RkShape* Shape1, RkShape* Shape2 );

		// Collision
		virtual void Collide() override;
	};