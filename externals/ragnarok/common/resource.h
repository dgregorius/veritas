//--------------------------------------------------------------------------------------------------
/*
	@file		resource.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once


//--------------------------------------------------------------------------------------------------
// RkResourcePointer
//--------------------------------------------------------------------------------------------------
template < typename T >
class RkResourcePointer
	{
	public:
		RkResourcePointer() = default;
		explicit RkResourcePointer( const T* Pointer );
		explicit RkResourcePointer( const T& Reference );

		RkResourcePointer& operator=( const T* Pointer );
		RkResourcePointer& operator=( const T& Reference );

		T* operator->();
		const T* operator->() const;

		operator T*();
		operator const T*() const;

		bool IsAligned( int Alignment = alignof( T ) ) const;

	private:
		int mOffset = 0;
	};

// String typedef
typedef RkResourcePointer< char > RkResourceString;


#include "resource.inl"
