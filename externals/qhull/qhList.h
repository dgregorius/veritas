//--------------------------------------------------------------------------------------------------
/*
	@file		qhList.h

	@author		Dirk Gregorius
	@version	0.1
	@date		30/11/2011

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once


//--------------------------------------------------------------------------------------------------
// qhListNode	
//--------------------------------------------------------------------------------------------------
template < typename T >
void qhInsert( T* Node, T* Where );

template < typename T >
void qhRemove( T* Node );

template < typename T >
bool qhInList( T* Node );


//--------------------------------------------------------------------------------------------------
// qhList	
//--------------------------------------------------------------------------------------------------
template < typename T >
class qhList
	{
	public:
		qhList();

		int Size() const;
		bool Empty() const;

		void Clear();
		void PushFront( T* Node );
		T* PopFront();
		void PushBack( T* Node );
		T* PopBack();

		void Insert( T* Node, T* Where );
		void Remove( T* Node );
		int IndexOf( const T* Node ) const;

		T* Front();
		const T* Front() const;
		T* Back();
		const T* Back() const;
		
		T* Begin();
		const T* Begin() const;
		T* End();
		const T* End() const;

	private:
		T mHead;

		// Non-copyable
		qhList( const qhList& );
		qhList& operator=( const qhList& );
	};



#include "qhList.inl"