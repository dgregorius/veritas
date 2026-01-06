//--------------------------------------------------------------------------------------------------
/*
	@file		shaderlibrary.h

	@author		Dirk Gregorius
	@version	0.1
	@date		12/30/2025

	Copyright (c) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

class VsShader;


//--------------------------------------------------------------------------------------------------
// VsShaderLibrary
//--------------------------------------------------------------------------------------------------
struct VsShaderLibrary
	{
	static inline VsShader* BackgroundShader = nullptr;
	static inline VsShader* MeshShader = nullptr;
	};

void vsLoadShaderLibrary();
void vsUnloadShaderLibrary();
