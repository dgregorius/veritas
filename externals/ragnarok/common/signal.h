//--------------------------------------------------------------------------------------------------
/*
	@file		signal.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/array.h"
#include "ragnarok/common/delegate.h"


//--------------------------------------------------------------------------------------------------
// RkSignal (Base)
//--------------------------------------------------------------------------------------------------
template < typename T > 
class RkSignal
	{
	};


//--------------------------------------------------------------------------------------------------
// RkSignal (Specialization)
//--------------------------------------------------------------------------------------------------
template < typename RkReturnValue, typename... RkArguments >
class RkSignal < RkReturnValue ( RkArguments... ) >
	{
	public:
		template< RkReturnValue( *Function )( RkArguments... ) >
		void Connect()
			{
			RkSlot Slot;
			Slot.Bind< Function >();
			mSlots.PushBack( Slot );
			}

		template < typename T, RkReturnValue( T::*Method )( RkArguments... ) >
		void Connect( T* Instance )
			{
			RkSlot Slot;
			Slot.Bind< T, Method >( Instance );
			mSlots.PushBack( Slot );
			}

		template < typename T, RkReturnValue( T::*Method )( RkArguments... ) const >
		void Connect( const T* Instance )
			{
			RkSlot Slot;
			Slot.Bind< T, Method >( Instance );
			mSlots.PushBack( Slot );
			}

		template< RkReturnValue( *Function )( RkArguments... ) >
		void Disconnect()
			{
			RkSlot Slot;
			Slot.Bind< Function >();

			for ( int Index = mSlots.Size() - 1; Index >= 0; --Index )
				{
				if ( mSlots[ Index ] == Slot )
					{
					mSlots[ Index ] = mSlots.Back();
					mSlots.PopBack();

					return;
					}
				}
			}

		template < typename T, RkReturnValue( T::*Method )( RkArguments... ) >
		void Disconnect( T* Instance )
			{
			RkSlot Slot;
			Slot.Bind< T, Method >( Instance );

			for ( int Index = mSlots.Size() - 1; Index >= 0; --Index )
				{
				if ( mSlots[ Index ] == Slot )
					{
					mSlots[ Index ] = mSlots.Back();
					mSlots.PopBack();

					return;
					}
				}
			}

		template < typename T, RkReturnValue( T::*Method )( RkArguments... ) const >
		void Disconnect( const T* Instance )
			{
			RkSlot Slot;
			Slot.Bind< T, Method >( Instance );

			for ( int Index = mSlots.Size() - 1; Index >= 0; --Index )
				{
				if ( mSlots[ Index ] == Slot )
					{
					mSlots[ Index ] = mSlots.Back();
					mSlots.PopBack();

					return;
					}
				}
			}

		void Emit( RkArguments... Arguments ) const
			{
			for ( const RkSlot& Slot : mSlots )
				{
				Slot.Invoke( std::forward< RkArguments >( Arguments )... );
				}
			}

	private:
		typedef RkDelegate< RkReturnValue( RkArguments... ) > RkSlot;
		RkArray< RkSlot > mSlots;
	};