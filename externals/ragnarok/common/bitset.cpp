//--------------------------------------------------------------------------------------------------
// bitset.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "bitset.h"


//--------------------------------------------------------------------------------------------------
// Local constants
//--------------------------------------------------------------------------------------------------
#define RK_BLOCK_SIZE 64


//--------------------------------------------------------------------------------------------------
// Local utilities
//--------------------------------------------------------------------------------------------------
static inline bool rkTestHighBits( int BitCount, const RkArray< uint64 >& BitVector )
	{
	if ( BitVector.Empty() )
		{
		RK_ASSERT( BitCount == 0 );
		return true;
		}

	// Any blocks beyond what we need must be zero (defensive).
	int MinBlocks = ( BitCount + ( RK_BLOCK_SIZE - 1 ) ) / RK_BLOCK_SIZE;
	for ( int BlockIndex = MinBlocks; BlockIndex < BitVector.Size(); ++BlockIndex )
		{
		if ( BitVector[ BlockIndex ] != 0 )
			{
			return false;
			}
		}

	if ( MinBlocks == 0 )
		{
		RK_ASSERT( BitCount == 0 );
		return true;
		}
	
	int Remainder = BitCount & ( RK_BLOCK_SIZE - 1 );
	if ( Remainder == 0 )
		{
		// Exact block boundary => no high bits inside last block
		return true;
		}

	// Low remainder bits are valid
	uint64 UsedBits = ( 1ULL << Remainder ) - 1;
	return ( BitVector[ MinBlocks - 1 ] & ~UsedBits ) == 0;
	}


//--------------------------------------------------------------------------------------------------
// RkBitset
//--------------------------------------------------------------------------------------------------
RkBitset::RkBitset( int BitCount )
	{
	Resize( BitCount );
	}


//--------------------------------------------------------------------------------------------------
int RkBitset::Size() const
	{
	return mBitCount;
	}


//--------------------------------------------------------------------------------------------------
int RkBitset::Count() const
	{
	int PopCount = 0;
	for ( uint64 Block : mBitVector )
		{
		PopCount += std::popcount( Block );
		}

	return PopCount;
	}


//--------------------------------------------------------------------------------------------------
bool RkBitset::Empty() const
	{
	return mBitCount == 0;
	}


//--------------------------------------------------------------------------------------------------
void RkBitset::Clear()
	{
	mBitCount = 0;
	mBitVector.Clear();

	// Maintain invariant that unused bits are zero
	RK_ASSERT( rkTestHighBits( mBitCount, mBitVector ) );
	}


//--------------------------------------------------------------------------------------------------
void RkBitset::Resize( int BitCount )
	{
	RK_ASSERT( BitCount >= 0 );
	if ( BitCount == mBitCount )
		{
		return;
		}

	mBitCount = BitCount;
	int BlockCount = ( BitCount + ( RK_BLOCK_SIZE - 1 ) ) / RK_BLOCK_SIZE;
	mBitVector.Resize( BlockCount, 0 );

	if ( !mBitVector.Empty() )
		{
		// Clear unused high bits in the last block -> 0..(BlockSize-1)
		int Remainder = BitCount & ( RK_BLOCK_SIZE - 1 );
		if ( Remainder )
			{
			mBitVector.Back() &= ( ( 1ULL << Remainder ) - 1 );
			}
		}

	// Maintain invariant that unused bits are zero
	RK_ASSERT( rkTestHighBits( mBitCount, mBitVector ) );
	}


//--------------------------------------------------------------------------------------------------
RkBitset& RkBitset::Set( int BitIndex )
	{
	RK_ASSERT( BitIndex >= 0 );
	if ( BitIndex >= mBitCount )
		{
		Resize( BitIndex + RK_BLOCK_SIZE );
		}
	
	int BlockIndex = BitIndex / RK_BLOCK_SIZE;
	mBitVector[ BlockIndex ] |= ( 1ULL << ( BitIndex & ( RK_BLOCK_SIZE - 1 ) ) );

	// Maintain invariant that unused bits are zero
	RK_ASSERT( rkTestHighBits( mBitCount, mBitVector ) );
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RkBitset& RkBitset::Reset( int BitIndex )
	{
	RK_ASSERT( 0 <= BitIndex && BitIndex < mBitCount );

	uint32_t BlockIndex = BitIndex / RK_BLOCK_SIZE;
	mBitVector[ BlockIndex ] &= ~( 1ULL << ( BitIndex & ( RK_BLOCK_SIZE - 1 ) ) );

	// Maintain invariant that unused bits are zero
	RK_ASSERT( rkTestHighBits( mBitCount, mBitVector ) );
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RkBitset& RkBitset::Flip( int BitIndex )
	{
	RK_ASSERT( 0 <= BitIndex && BitIndex < mBitCount );

	uint32_t BlockIndex = BitIndex / RK_BLOCK_SIZE;
	mBitVector[ BlockIndex ] ^= ( 1ULL << ( BitIndex & ( RK_BLOCK_SIZE - 1 ) ) );

	// Maintain invariant that unused bits are zero
	RK_ASSERT( rkTestHighBits( mBitCount, mBitVector ) );
	return *this;
	}


//--------------------------------------------------------------------------------------------------
bool RkBitset::Test( int BitIndex ) const
	{
	if ( !( 0 <= BitIndex && BitIndex < mBitCount ) )
		{
		// Return false, for every bit outside range
		return false;
		}

	int BlockIndex = BitIndex / RK_BLOCK_SIZE;
	return mBitVector[ BlockIndex ] & ( 1ULL << ( BitIndex & ( RK_BLOCK_SIZE - 1 ) ) ); ;
	}


//--------------------------------------------------------------------------------------------------
RkBitset& RkBitset::operator&=( const RkBitset& Other )
	{
	RK_ASSERT( mBitCount == Other.mBitCount );
	RK_ASSERT( mBitVector.Size() == Other.mBitVector.Size() );

	int BlockCount = mBitVector.Size();
	for ( int BlockIndex = 0; BlockIndex < BlockCount; ++BlockIndex )
		{
		mBitVector[ BlockIndex ] &= Other.mBitVector[ BlockIndex ];
		}

	// Maintain invariant that unused bits are zero
	RK_ASSERT( rkTestHighBits( mBitCount, mBitVector ) );
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RkBitset& RkBitset::operator|=( const RkBitset& Other )
	{
	RK_ASSERT( mBitCount == Other.mBitCount );
	RK_ASSERT( mBitVector.Size() == Other.mBitVector.Size() );

	int BlockCount = mBitVector.Size();
	for ( int BlockIndex = 0; BlockIndex < BlockCount; ++BlockIndex )
		{
		mBitVector[ BlockIndex ] |= Other.mBitVector[ BlockIndex ];
		}

	// Maintain invariant that unused bits are zero
	RK_ASSERT( rkTestHighBits( mBitCount, mBitVector ) );
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RkBitset& RkBitset::operator^=( const RkBitset& Other )
	{
	int BlockCount = mBitVector.Size();
	RK_ASSERT( Other.mBitVector.Size() == BlockCount );
	for ( int BlockIndex = 0; BlockIndex < BlockCount; ++BlockIndex )
		{
		mBitVector[ BlockIndex ] ^= Other.mBitVector[ BlockIndex ];
		}

	// Maintain invariant that unused bits are zero
	RK_ASSERT( rkTestHighBits( mBitCount, mBitVector ) );
	return *this;
	}