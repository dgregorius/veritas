// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

//#include "constants.h"
#include "core.h"

#include <string.h>

// Array for standard layout, no constructors/destructors.
// Supports shallow copy.
// No RAII so you must call Create and Destroy.
// Zero initialization is valid.
template <typename T> struct b3Array
{
	void Create( int initialCapacity )
	{
		static_assert( __is_trivial( T ) && __is_standard_layout( T ), "template type must be plain data" );

		data = nullptr;
		capacity = 0;
		count = 0;

		if ( initialCapacity > 0 )
		{
			Reserve( initialCapacity );
		}
	}

	void Destroy()
	{
		b3Free( data, capacity * sizeof( T ) );
		data = nullptr;
		capacity = 0;
		count = 0;
	}

	B3_FORCE_INLINE T& operator[]( int index )
	{
		B3_ASSERT( 0 <= index && index < count );
		return data[index];
	}

	B3_FORCE_INLINE const T& operator[]( int index ) const
	{
		B3_ASSERT( 0 <= index && index < count );
		return data[index];
	}

	// Use this for bounds checking
	T* Get( int index )
	{
		B3_ASSERT( 0 <= index && index < count );
		return data + index;
	}

	void Reserve( int newCapacity );


	int GetCount() const
	{
		return count;
	}

	bool IsEmpty() const
	{
		return count == 0;
	}

	void Clear()
	{
		count = 0;
	}

	void Resize( int newCount )
	{
		Reserve( newCount );
		count = newCount;
	}

	void MemZero()
	{
		if (capacity > 0)
		{
			memset( data, 0, capacity * sizeof( T ) );
		}
	}

	int AddUninitialized()
	{
		if (count == capacity)
		{
			// grow by 50%
			// int newCapacity = capacity > 2 ? capacity + ( capacity >> 1 ) : 8;

			// double capacity
			int newCapacity = capacity > 2 ? 2 * capacity : 8;

			Reserve( newCapacity );
		}

		count += 1;
		return count - 1;
	}

	T* Add()
	{
		if ( count == capacity )
		{
			// grow by 50%
			// int newCapacity = capacity > 2 ? capacity + ( capacity >> 1 ) : 8;

			// double capacity
			int newCapacity = capacity > 2 ? 2 * capacity : 8;

			Reserve( newCapacity );
		}

		count += 1;
		return data + ( count - 1 );
	}

	void PushBack( const T& value )
	{
		if (count == capacity)
		{
			// grow by 50%
			// int newCapacity = capacity > 2 ? capacity + ( capacity >> 1 ) : 8;

			// double capacity
			int newCapacity = capacity > 2 ? 2 * capacity : 8;

			Reserve( newCapacity );
		}

		data[count] = value;
		count += 1;
	}

	void Append( const T* otherValues, int otherCount )
	{
		if ( count + otherCount > capacity )
		{
			int requiredCapacity = count + otherCount;
			int newCapacity = requiredCapacity > 2 ? requiredCapacity + ( requiredCapacity >> 1 ) : 8;
			Reserve( newCapacity );
		}

		memcpy( data + count, otherValues, otherCount * sizeof( T ) );
		count += otherCount;
	}

	T PopBack()
	{
		B3_ASSERT( count > 0 );
		count -= 1;
		return data[count];
	}

	T& Back()
	{
		B3_ASSERT( count > 0 );
		return data[count - 1];
	}

	const T& Back() const
	{
		B3_ASSERT( count > 0 );
		return data[count - 1];
	}

	// Removes an element, moving the last element into the empty slot.
	// Returns the previous index of the element that was moved.
	int RemoveSwap( int index )
	{
		B3_ASSERT( 0 <= index && index < count );
		int movedIndex = B3_NULL_INDEX;

		if ( index < count - 1 )
		{
			movedIndex = count - 1;
			data[index] = data[movedIndex];
		}

		count -= 1;
		return movedIndex;
	}

	// slow search
	bool Contains( const T& value ) const;

	int GetByteCount() const
	{
		return capacity * sizeof( T );
	}

#if 1
	// STL range loop support
	T* begin()
	{
		return data;
	}

	const T* begin() const
	{
		return data;
	}

	T* end()
	{
		return data + count;
	}

	const T* end() const
	{
		return data + count;
	}
#endif

	T* data;
	int capacity;
	int count;
};

template <typename T> void b3Array<T>::Reserve( int newCapacity )
{
	B3_ASSERT( newCapacity >= 0 );
	if ( newCapacity <= capacity )
	{
		return;
	}

	// Allocate new buffers
	static_assert( alignof( T ) <= 64, "alignment" );
	T* newData = static_cast<T*>( b3Alloc( newCapacity * sizeof( T ) ) );

	// Move old buffer
	if ( count > 0 )
	{
		memcpy( newData, data, count * sizeof( T ) );
	}

	b3Free( data, capacity * sizeof( T ) );

	data = newData;
	capacity = newCapacity;
}

template <typename T> bool b3Array<T>::Contains( const T& value ) const
{
	for ( int i = 0; i < count; ++i )
	{
		if ( data[i] == value )
		{
			return true;
		}
	}

	return false;
}
