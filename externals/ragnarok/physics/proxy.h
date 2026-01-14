//--------------------------------------------------------------------------------------------------
/*
	@file		proxy.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/assert.h"
#include "ragnarok/common/types.h"

// Verstable
#define NAME RkProxySetImpl
#define KEY_TY uint64_t
#define HASH_FN vt_hash_integer
#define CMPR_FN vt_cmpr_integer
#include "verstable.h"


//--------------------------------------------------------------------------------------------------
// RkProxy
//--------------------------------------------------------------------------------------------------
struct RkProxy
	{
	uint32 Type : 2;
	uint32 Index : 30;

	operator uint32() const
		{
		return reinterpret_cast< const uint32& >( *this );
		}
	};

RK_GLOBAL_CONSTANT const RkProxy RK_NULL_PROXY = { 3, 0 };


//--------------------------------------------------------------------------------------------------
// RkProxyPair
//--------------------------------------------------------------------------------------------------
struct RkProxyPair
	{
	RkProxy Proxy1;
	RkProxy Proxy2;
	};


//--------------------------------------------------------------------------------------------------
// RkProxySet
//--------------------------------------------------------------------------------------------------
class RkProxySet
	{
	public:
		// Construction / Destruction
		RkProxySet();
		~RkProxySet();

		int Size() const;
		void Clear();

		void Insert( RkProxy Proxy );
		bool Remove( RkProxy Proxy );
		bool Contains( RkProxy Proxy ) const;

	private:
		RkProxySetImpl mImpl;
	};

