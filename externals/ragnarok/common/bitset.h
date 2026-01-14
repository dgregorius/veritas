//--------------------------------------------------------------------------------------------------
/*
	@file		bitset.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/array.h"
#include "ragnarok/common/types.h"

#include <bit>   


//--------------------------------------------------------------------------------------------------
// RkBitset
//--------------------------------------------------------------------------------------------------
class RkBitset
	{
	public:
		RkBitset( int BitCount = 0 );

		int Size() const;
		int Count() const;
		bool Empty() const;

		void Clear();
		void Resize( int BitCount );
		
		RkBitset& Set( int BitIndex );
		RkBitset& Reset( int BitIndex );
		RkBitset& Flip( int BitIndex );
		bool Test( int BitIndex ) const;

		RkBitset& operator&=( const RkBitset& Other );
		RkBitset& operator|=( const RkBitset& Other );
		RkBitset& operator^=( const RkBitset& Other );

		template< typename T >
		void ForEachSetBit( T&& Callback ) const
			{
			for ( int BlockIndex = 0; BlockIndex < mBitVector.Size(); ++BlockIndex )
				{
				uint64 Word = mBitVector[ BlockIndex ];
				while ( Word )
					{
					int Bit = std::countr_zero( Word );
					Callback( BlockIndex * BlockSize + Bit );
					
					// Clear lowest bit
					Word &= Word - 1;
					}
				}
			}

		template< typename T >
		void ForEachSetBitR( T&& Callback ) const
			{
			for ( int BlockIndex = mBitVector.Size() - 1; BlockIndex >= 0; --BlockIndex )
				{
				uint64 Word = mBitVector[ BlockIndex ];
				while ( Word )
					{
					int Bit = ( BlockSize - 1 ) - std::countl_zero( Word );
					Callback( BlockIndex * 64 + Bit );

					// Clear highest bit
					Word &= ~( 1ULL << Bit );
					}
				}
			}
		
	private:
		enum { BlockSize = 64 };

		int mBitCount = 0;
		RkArray< uint64 > mBitVector;
	};