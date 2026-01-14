// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/base.h"
#include "core.h"
#include <string.h>

// Array for standard layout, no constructors/destructors. This tries to use the stack but can grow if needed.
// todo consider using an arena instead of the stack
template <typename T, int N> class b3StackArray
{
  public:
	b3StackArray()
	{
		static_assert( __is_trivial( T ) && __is_standard_layout( T ), "template type must be plain data" );

		Create();
	}

	~b3StackArray()
	{
		Destroy();
	}

	void Create()
	{
		m_data = m_stackData;
		m_capacity = N;
		m_count = 0;
	}

	void Destroy()
	{
		if (m_data != m_stackData)
		{
			b3Free(m_data, m_capacity * sizeof(T));
		}
	}

	b3StackArray(const b3StackArray& other) = delete;
	void operator=(const b3StackArray& other) = delete;

	T& operator[](int index)
	{
		B3_ASSERT(0 <= index && index < m_count);
		return m_data[index];
	}

	const T& operator[](int index) const
	{
		B3_ASSERT(0 <= index && index < m_count);
		return m_data[index];
	}

	// Use this for bounds checking
	T* Get( int index )
	{
		B3_ASSERT( 0 <= index && index < m_count );
		return m_data + index;
	}

	void Clear()
	{
		m_count = 0;
	}

	void Reserve(int capacity);
	void Resize(int count)
	{
		Reserve(count);
		m_count = count;
	}

	void PushBack(const T& value);
	T* Add();
	T PopBack();

	void Append( const T* otherValues, int otherCount )
	{
		if ( m_count + otherCount > m_capacity )
		{
			int requiredCapacity = m_count + otherCount;
			int newCapacity = requiredCapacity > 2 ? requiredCapacity + ( requiredCapacity >> 1 ) : 8;
			Reserve( newCapacity );
		}

		memcpy( m_data + m_count, otherValues, otherCount * sizeof( T ) );
		m_count += otherCount;
	}

	/// Removes an element, moving the last element into the empty slot
	//void Remove(int index);

	T m_stackData[N];
	T* m_data;
	int m_capacity;
	int m_count;
};

template <typename T, int N> inline void b3StackArray<T, N>::Reserve(int capacity)
{
	B3_ASSERT(capacity >= 0);
	if (capacity <= m_capacity)
	{
		return;
	}

	static_assert( alignof( T ) <= 16, "alignment" );
	T* data = static_cast<T*>(b3Alloc(capacity * sizeof(T)));

	if (m_count > 0)
	{
		memcpy(data, m_data, m_count * sizeof(T));
	}

	if (m_data != m_stackData)
	{
		b3Free(m_data, m_capacity * sizeof(T));
	}

	m_data = data;
	m_capacity = capacity;
}

template <typename T, int N> inline void b3StackArray<T, N>::PushBack(const T& value)
{
	if (m_count == m_capacity)
	{
		// grow by 50%
		int newCapacity = m_capacity > 2 ? m_capacity + (m_capacity >> 1) : 8;
		Reserve(newCapacity);
	}

	m_data[m_count] = value;
	m_count += 1;
}

template <typename T, int N> inline T* b3StackArray<T, N>::Add()
{
	if (m_count == m_capacity)
	{
		// grow by 50%
		int newCapacity = m_capacity > 2 ? m_capacity + (m_capacity >> 1) : 8;
		Reserve(newCapacity);
	}

	m_count += 1;
	return m_data + (m_count - 1);
}

template <typename T, int N> inline T b3StackArray<T, N>::PopBack()
{
	B3_ASSERT(m_count > 0);
	m_count -= 1;
	return m_data[m_count];
}

//template <typename T, int N> inline void b3StackArray<T, N>::Remove(int index)
//{
//	B3_ASSERT(0 <= index && index < m_count);
//
//	if (index < m_count - 1)
//	{
//		m_data[index] = m_data[m_count - 1];
//	}
//
//	m_count -= 1;
//}
