// SPDX-FileCopyrightText: 2025 Dirk Gregorius
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box3d/base.h"

// todo_erin only used for qhull
template <typename T> class b3Pool
{
public:
	// Construction / Destruction
	b3Pool();
	b3Pool( b3Pool&& ) = delete;
	b3Pool( const b3Pool& ) = delete;
	~b3Pool();

	// Assignment
	b3Pool& operator=( b3Pool&& ) = delete;
	b3Pool& operator=( const b3Pool& ) = delete;

	// Memory
	void Reserve( int capacity );

	int Alloc();
	void Free( int index );
	T* Allocate();
	void Free( T* address );

	// Accessors
	//T& operator[]( int index );
	const T& operator[]( int index ) const;

	int m_capacity;
	T* m_objects;
	int m_next;
};

template <typename T>
b3Pool<T>::b3Pool()
	: m_capacity( 0 )
	, m_objects( nullptr )
	, m_next( -1 )
{
}

template <typename T> b3Pool<T>::~b3Pool()
{
	b3Free( m_objects, m_capacity * sizeof(T) );
}

template <typename T> void b3Pool<T>::Reserve( int capacity )
{
	if ( capacity > m_capacity )
	{
		T* objects = m_objects;
		m_objects = static_cast<T*>( b3Alloc( capacity * sizeof( T ) ) );
		memcpy( m_objects, objects, m_capacity * sizeof( T ) );

		b3Free( objects, m_capacity * sizeof( T ) );
		objects = nullptr;

		for ( int i = m_capacity; i < capacity - 1; ++i )
		{
			int* nNext = (int*)( m_objects + i );
			*nNext = i + 1;
		}

		int* nNext = (int*)( m_objects + capacity - 1 );
		*nNext = -1;

		m_next = m_capacity;
		m_capacity = capacity;
	}
}

template <typename T> int b3Pool<T>::Alloc()
{
	// Grow the pool if the free list is empty
	if ( m_next < 0 )
	{
		int newCapacity = m_capacity > 0 ? 2 * m_capacity : 2;
		Reserve( newCapacity );
	}

	// Peel a node from the free list
	int index = m_next;
	m_next = *(int*)( m_objects + index );

	return index;
}

template <typename T> void b3Pool<T>::Free( int index )
{
	// Re-insert object into free list
	B3_ASSERT( 0 <= index && index < m_capacity );

	*(int*)( m_objects + index ) = m_next;
	m_next = index;
}

template <typename T> T* b3Pool<T>::Allocate()
{
	int index = Alloc();
	return m_objects + index;
}

template <typename T> void b3Pool<T>::Free( T* address )
{
	int index = static_cast<int>( address - m_objects );
	Free( index );
}

template <typename T> const T& b3Pool<T>::operator[]( int index ) const
{
	B3_ASSERT( 0 <= index && index < m_capacity );
	return m_objects[index];
}
