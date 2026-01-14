//--------------------------------------------------------------------------------------------------
/*
	@file		delegate.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "assert.h"


//--------------------------------------------------------------------------------------------------
// RkDelegate (Base)
//--------------------------------------------------------------------------------------------------
template < typename T >
class RkDelegate
	{
	};


//--------------------------------------------------------------------------------------------------
// RkDelegate (Specialization)
//--------------------------------------------------------------------------------------------------
template < typename RkReturnValue, typename... RkArguments >
class RkDelegate < RkReturnValue( RkArguments... ) >
	{
	public:
		RkDelegate()
			: mInstance( nullptr )
			, mProxy( nullptr )
			{

			}

		template< RkReturnValue( *Function )( RkArguments... ) >
		void Bind()
			{
			mInstance = nullptr;
			mProxy = &FunctionProxy< Function >;
			}

		template < typename T, RkReturnValue( T::*Method )( RkArguments... ) >
		void Bind( T* Instance )
			{
			RK_ASSERT( Instance );
			mInstance = Instance;
			mProxy = &MethodProxy< T, Method >;
			}

		template < typename T, RkReturnValue( T::*Method )( RkArguments... ) const >
		void Bind( const T* Instance )
			{
			RK_ASSERT( Instance );
			mInstance = const_cast< T* >( Instance );
			mProxy = &ConstMethodProxy< T, Method >;
			}

		RkReturnValue Invoke( RkArguments... Arguments ) const
			{
			RK_ASSERT( mProxy );
			return mProxy( mInstance, std::forward< RkArguments >( Arguments )... );
			}

		friend bool operator<( const RkDelegate& Lhs, const RkDelegate& Rhs )
			{
			if ( Lhs.mInstance < Rhs.mInstance )
				{
				return true;
				}

			if ( Lhs.mInstance == Rhs.mInstance )
				{
				return Lhs.mProxy < Rhs.mProxy;
				}

			return false;
			}

		friend bool operator==( const RkDelegate& Lhs, const RkDelegate& Rhs )
			{
			return Lhs.mInstance == Rhs.mInstance && Lhs.mProxy == Rhs.mProxy;
			}

		friend bool operator!=( const RkDelegate& Lhs, const RkDelegate& Rhs )
			{
			return Lhs.mInstance != Rhs.mInstance || Lhs.mProxy != Rhs.mProxy;
			}

	private:
		typedef RkReturnValue( *RkProxyFunction )( void*, RkArguments... );

		template < RkReturnValue( *Function )( RkArguments... ) >
		static inline RkReturnValue FunctionProxy( void*, RkArguments... Arguments )
			{
			return Function( std::forward< RkArguments >( Arguments )... );
			}

		template < typename T, RkReturnValue( T::*Method )( RkArguments... ) >
		static inline RkReturnValue MethodProxy( void* Instance, RkArguments... Arguments )
			{
			return ( static_cast< T* >( Instance )->*Method )( std::forward< RkArguments >( Arguments )... );
			}

		template < typename T, RkReturnValue( T::*Method )( RkArguments... ) const >
		static inline RkReturnValue ConstMethodProxy( void* Instance, RkArguments... Arguments )
			{
			return ( static_cast< const T* >( Instance )->*Method )( std::forward< RkArguments >( Arguments )... );
			}

		void* mInstance;
		RkProxyFunction mProxy;
	};