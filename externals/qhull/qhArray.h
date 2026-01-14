//--------------------------------------------------------------------------------------------------
/*
	@file		qhArray.h

	@author		Dirk Gregorius
	@version	0.1
	@date		05/12/2012

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "qhTypes.h"
#include "qhMemory.h"


//--------------------------------------------------------------------------------------------------
// qhArray 
//--------------------------------------------------------------------------------------------------
template < typename T >
class qhArray
	{
	public:
		qhArray();
		qhArray( std::initializer_list< T > List );
		qhArray( const T* First, const T* Last );
		qhArray( const qhArray& Other );
		qhArray( qhArray&& Other );
		~qhArray();

		qhArray& operator=( std::initializer_list< T > List );
		qhArray& operator=( const qhArray& Other );
		qhArray& operator=( qhArray&& Other );

		int Size() const;
		int Capacity() const;
		bool Empty() const;

		void Clear();
		void Reserve( int Count );
		void Resize( int Count );

		int PushBack();
		void PushBack( const T& Other );
		void PushBack( T&& Other );
		void PopBack();

		void Append( const qhArray< T >& Other );
		void Append( const T* First, const T* Last );

		int IndexOf( const T& Object ) const;

		T& operator[]( int Index );
		const T& operator[]( int Index ) const;

		T& Front();
		const T& Front() const;
		T& Back();
		const T& Back() const;
		T* Begin();
		const T* Begin() const;
		T* End();
		const T* End() const;

		void Swap( qhArray< T >& Other );

		bool operator==( const qhArray& Other ) const;
		bool operator!=( const qhArray& Other ) const;

	private:
		bool Inside( const T* Object ) const;

		T* mFirst;
		T* mLast;
		T* mEnd;

		// STL like iterators to enable range based loop support. (Do not use directly!)
		inline friend T* begin( qhArray& Array )				{ return Array.Begin(); }
		inline friend const T* begin( const qhArray& Array )	{ return Array.Begin(); }
		inline friend T* end( qhArray& Array )					{ return Array.End(); }
		inline friend const T* end( const qhArray& Array )		{ return Array.End(); }
	};



#include "qhArray.inl"