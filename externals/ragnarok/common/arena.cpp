//--------------------------------------------------------------------------------------------------
// arena.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "arena.h"

// Virtual Memory
#define VC_EXTRALEAN
#include <windows.h>


//--------------------------------------------------------------------------------------------------
// RkArena
//--------------------------------------------------------------------------------------------------
RkArena::RkArena( size_t Size )
	{
	if ( Size > 0 )
		{
		// Reserve the entire address space (no physical memory used yet)
		uint8_t* Base = static_cast< uint8_t* >( VirtualAlloc( NULL, Size, MEM_RESERVE, PAGE_READWRITE ) );
		RK_ASSERT( Base );

		mReservedSize = Size;
		mReservedBase = Base;
		mCommittedSize = 0;
		mRemainingSize = 0;
		}
	}


//--------------------------------------------------------------------------------------------------
RkArena::~RkArena()
	{
	if ( mReservedBase )
		{
		// Release the entire reserved region (decommits implicitly)
		BOOL Success = VirtualFree( mReservedBase, 0, MEM_RELEASE );
		RK_ASSERT( Success );
		}
	}


//--------------------------------------------------------------------------------------------------
void* RkArena::Allocate( size_t Size, size_t Alignment )
	{
	RK_ASSERT( Size != 0 );
	RK_ASSERT( Alignment != 0 );
	RK_ASSERT( ( Alignment & ( Alignment - 1 ) ) == 0 );

	// Cursor is derived from reserved base and difference 
	// between committed and remaining (committed) size.
	uint8* Cursor = mReservedBase + mCommittedSize - mRemainingSize;

	uintptr_t NextAddress = reinterpret_cast< uintptr_t >( Cursor );
	uintptr_t NextAligned = rkAlign( NextAddress, Alignment );
	RK_ASSERT( NextAligned >= NextAddress );
	size_t Padding = NextAligned - NextAddress;
	size_t NeededSize = Padding + Size;
	RK_ASSERT( NeededSize >= Size );

	if ( NeededSize > mRemainingSize )
		{
		// Round the deficit up to the next multiple of InitialCommit 
		// (64KB) This ensures we always commit in 64KB chunks, which 
		// is the Windows 10/11 (64bit) allocation granularity.
		const size_t InitialCommit = 64 * 1024;
		size_t Deficit = NeededSize - mRemainingSize;
		size_t CommitSize = rkAlign( Deficit, InitialCommit );

		// Boundary check
		size_t RemainingReservedSize = mReservedSize - mCommittedSize;
		if ( CommitSize > RemainingReservedSize )
			{
			// Try multiple of page size
			const size_t PageSize = 4096;
			CommitSize = rkAlign( Deficit, PageSize );

			// If even page-aligned deficit exceeds reservation, we are truly done!
			if ( CommitSize > RemainingReservedSize )
				{
				return nullptr;
				}
			}
		
		uint8* CommitAddress = mReservedBase + mCommittedSize;
		if ( !VirtualAlloc( CommitAddress, CommitSize, MEM_COMMIT, PAGE_READWRITE ) )
			{
			return nullptr;
			}

#if defined( DEBUG ) || defined( _DEBUG )
		// POISONING: Fill newly committed memory with a recognizable pattern.
		memset( CommitAddress, 0xCC, CommitSize );
#endif
		
		mCommittedSize += CommitSize;
		mRemainingSize += CommitSize;
		}

	RK_ASSERT( mRemainingSize >= NeededSize );
	mRemainingSize -= NeededSize;

	// Final check: The resulting pointer must be (inclusively) within the COMMITTED range.
	RK_ASSERT( mReservedBase <= (uint8_t*)NextAligned + Size && (uint8_t*)NextAligned + Size <= mReservedBase + mCommittedSize );
	return reinterpret_cast< void* >( NextAligned );
	}


//--------------------------------------------------------------------------------------------------
void RkArena::Release()
	{
	// Resetting 'remaining' size to the total 'committed' also
	// moves the derived cursor back to the start of the arena.
	mRemainingSize = mCommittedSize;

#if defined( DEBUG ) || defined( _DEBUG )
	// RE-POISON the memory to eventually catch "use-after-release" bugs
	if ( mCommittedSize > 0 )
		{
		memset( mReservedBase, 0xCC, mCommittedSize );
		}
#endif
	}