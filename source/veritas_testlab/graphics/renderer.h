//--------------------------------------------------------------------------------------------------
/**
	@file		renderer.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include <veritas/veritas.h>


//--------------------------------------------------------------------------------------------------
// VsWorldRenderer
//--------------------------------------------------------------------------------------------------
class VsWorldRenderer : public IVsWorldListener
	{
	public:
		
	private:
		// World listener 
		virtual void OnBodyAdded( IVsBody* Body ) override;
		virtual void OnBodyRemoved( IVsBody* Body ) override;
		virtual void OnShapeAdded( IVsBody* Body, IVsShape* Shape ) override;
		virtual void OnShapeRemoved( IVsBody* Body, IVsShape* Shape ) override;


	};