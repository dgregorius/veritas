//--------------------------------------------------------------------------------------------------
// proxy.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "proxy.h"


//--------------------------------------------------------------------------------------------------
// RkProxySet
//--------------------------------------------------------------------------------------------------
RkProxySet::RkProxySet()
	{
	RkProxySetImpl_init( &mImpl );
	}


//--------------------------------------------------------------------------------------------------
RkProxySet::~RkProxySet()
	{
	RkProxySetImpl_cleanup( &mImpl );
	}


//--------------------------------------------------------------------------------------------------
int RkProxySet::Size() const
	{
	return static_cast< int >( RkProxySetImpl_size( &mImpl ) );
	}


//--------------------------------------------------------------------------------------------------
void RkProxySet::Clear()
	{
	RkProxySetImpl_clear( &mImpl );
	}


//--------------------------------------------------------------------------------------------------
void RkProxySet::Insert( RkProxy Proxy )
	{
	RkProxySetImpl_insert( &mImpl, Proxy );
	}


//--------------------------------------------------------------------------------------------------
bool RkProxySet::Remove( RkProxy Proxy )
	{
	return RkProxySetImpl_erase( &mImpl, Proxy );
	}


//--------------------------------------------------------------------------------------------------
bool RkProxySet::Contains( RkProxy Proxy ) const
	{
	RkProxySetImpl_itr Iterator = RkProxySetImpl_get( &mImpl, Proxy );
	return !RkProxySetImpl_is_end( Iterator );
	}