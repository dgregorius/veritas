//--------------------------------------------------------------------------------------------------
/*
	@file		array.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/assert.h"
#include "ragnarok/common/memory.h"

#include <algorithm>
#include <numeric>


//--------------------------------------------------------------------------------------------------
// RkArray 
//--------------------------------------------------------------------------------------------------
template < typename T >
class RkArray
	{
	public:
		RkArray() = default;
		RkArray( RkArray&& Other );
		RkArray( const RkArray& Other );
		RkArray( std::initializer_list< T > List );
		RkArray( const T* First, const T* Last );
		explicit RkArray( int Count );
	    RkArray( int Count, const T& Value );
		RkArray( T* First, T* Last, T* End );
		~RkArray();

		RkArray& operator=( RkArray&& Other );
		RkArray& operator=( const RkArray& Other );
		RkArray& operator=( std::initializer_list< T > List );

		int Size() const;
		int Capacity() const;
		bool Empty() const;

		void Clear();
		void Reserve( int Count );
		void Resize( int Count );
		void Resize( int Count, const T& Value );

		int PushBack();
		int PushBack( const T& Value );
		void PushBack( int Count, const T* List );
		void PushBack( const T* First, const T* Last );
		void PushBack( const RkArray< T >& Other );
		void Insert( int Where, const T& Value );

		void PopBack();
		void Remove( int Where );
		void Remove( const T& Value );
		
		bool Contains( const T& Value ) const;
		int IndexOf( const T& Value ) const;

		T& Front();
		const T& Front() const;
		T& Back();
		const T& Back() const;
		T* Begin();
		const T* Begin() const;
		T* End();
		const T* End() const;

		T* Data();
		const T* Data() const;

		T& operator[]( int Index );
		const T& operator[]( int Index ) const;

		bool operator==( const RkArray& Other ) const;
		bool operator!=( const RkArray& Other ) const;

	protected:
		enum { InitialCapacity = 16 };
		void Grow();

		T* mFirst = nullptr;
		T* mLast = nullptr;
		T* mEnd = nullptr;
		bool mAllocated = true;

		// STL like iterators to enable range based loop support. (Do not use directly!)
		inline friend T* begin( RkArray& Array ) { return Array.Begin(); }
		inline friend T* end( RkArray& Array ) { return Array.End(); }
		inline friend const T* begin( const RkArray& Array ) { return Array.Begin(); }
		inline friend const T* end( const RkArray& Array ) { return Array.End(); }

		// STL like accessors for range conversion support. (Do not use directly!)
		inline friend T* data( RkArray& Array ) { return Array.Data(); }
		inline friend const T* data( const RkArray& Array ) { return Array.Data(); }
		inline friend size_t size( const RkArray& Array ) { return Array.Size(); }
	};

// Utilities
template < typename T >
void rkDeleteAll( T** First, T** Last );
template < typename T >
void rkDeleteAll( RkArray< T* >& Array );


//--------------------------------------------------------------------------------------------------
// RkStackArray 
//--------------------------------------------------------------------------------------------------
template < typename T, int N >
class RkStackArray : public RkArray< T >
	{
	public:
		RkStackArray();
		explicit RkStackArray( int Count );
		RkStackArray( int Count, const T& Value  );

	private:
		alignas( alignof( T ) ) uint8 mStorage[ N * sizeof( T ) ];
	};


//--------------------------------------------------------------------------------------------------
// RkIndexedArray
//--------------------------------------------------------------------------------------------------
template < typename T, int T::*MemberIndex >
class RkIndexedArray 
	{
	public:
		int Size() const;
		int Capacity() const;
		bool Empty() const;

		int PushBack( T* Object );
		void PopBack();
		void Remove( T* Object );
		bool Contains( T* Value ) const;

		T* Front();
		const T* Front() const;
		T* Back();
		const T* Back() const;
		T** Begin();
		T* const* Begin() const;
		T** End();
		T* const* End() const;

		T* operator[]( int Index );
		const T* operator[]( int Index ) const;

	private:
		RkArray< T* > mArray;

		// STL like iterators to enable range based loop support. (Do not use directly!)
		inline friend T** begin( RkIndexedArray& Array ) { return Array.Begin(); }
		inline friend T** end( RkIndexedArray& Array ) { return Array.End(); }
		inline friend T* const* begin( const RkIndexedArray& Array ) { return Array.Begin(); }
		inline friend T* const* end( const RkIndexedArray& Array ) { return Array.End(); }
	};


template < typename T, int T::* MemberIndex >
void rkDeleteAll( RkIndexedArray< T, MemberIndex >& Array );


#include "array.inl"


		