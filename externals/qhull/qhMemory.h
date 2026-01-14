//--------------------------------------------------------------------------------------------------
/*
	@file		qhMemory.h

	@author		Dirk Gregorius
	@version	0.1
	@date		30/11/2011

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "qhTypes.h"

#include <new>
#include <type_traits>
#include <xutility>


//--------------------------------------------------------------------------------------------------
// qhMemory
//--------------------------------------------------------------------------------------------------
extern void* ( *qhAllocHook )( size_t, size_t );
extern void ( *qhFreeHook )( void* );

void* qhAlloc( size_t Bytes, size_t Alignment );
void qhFree( void* Address );


void* qhAddByteOffset( void* Address, size_t Offset );
const void* qhAddByteOffset( const void* Address, size_t Offset );

void* qhAlign( void* Address, size_t Alignment );
const void* qhAlign( const void* Address, size_t Alignment );
bool qhIsAligned( const void* Address, size_t Alignment );

//--------------------------------------------------------------------------------------------------
// qhPool
//--------------------------------------------------------------------------------------------------
template < typename T >
class qhPool
	{
	public:
		qhPool();
		qhPool( const qhPool& ) = delete;
		~qhPool();

		qhPool& operator=( const qhPool& ) = delete;

		void Clear();
		void Resize( int Size );
		T* Allocate();
		void Free( T* Address );

	private:
		int mSize;
		T* mPool;
		int mFree;
	};


//--------------------------------------------------------------------------------------------------
// Memory utilities
//--------------------------------------------------------------------------------------------------
template < typename T > void qhConstruct( T* Object );
template < typename T > void qhConstruct( int Count, T* Objects );
template < typename T > void qhConstruct( T* First, T* Last );
template < typename T > void qhDestroy( T* Object );
template < typename T > void qhDestroy( int Count, T* Objects );
template < typename T > void qhDestroy( T* First, T* Last );

template < typename T > void qhCopyConstruct( T* Object, const T& Value );
template < typename T > void qhMoveConstruct( T* Object, T&& Value );

template< typename T > void qhCopy( const T* First, const T* Last, T* Dest );
template< typename T > void qhMove( T* First, T* Last, T* Dest );
template< typename T > void qhFill( T* First, T* Last, const T& Value );
template< typename T > void qhUninitializedCopy( const T* First, const T* Last, T* Dest );
template< typename T > void qhUninitializedMove( T* First, T* Last, T* Dest );
template< typename T > void qhUninitializedFill( T* First, T* Last, const T& Value );



#include "qhMemory.inl"