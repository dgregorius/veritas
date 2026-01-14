//--------------------------------------------------------------------------------------------------
// string.cpp
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "string.h"
#include "memory.h"


//--------------------------------------------------------------------------------------------------
// RkString
//--------------------------------------------------------------------------------------------------
RkString::RkString( RkString&& Other )
	{
	// Move
	mBuffer = Other.mBuffer;
	Other.mBuffer = nullptr;
	}


//--------------------------------------------------------------------------------------------------
RkString::RkString( const RkString& Other )
	: mBuffer( rkStrDup( Other ) )
	{

	}


//--------------------------------------------------------------------------------------------------
RkString::RkString( const char* String )
	: mBuffer( rkStrDup( String ) )
	{

	}


//--------------------------------------------------------------------------------------------------
RkString::~RkString()
	{
	rkFree( mBuffer );
	}


//--------------------------------------------------------------------------------------------------
RkString& RkString::operator=( RkString&& Other )
	{
	if ( this != &Other )
		{
		// Destroy
		rkFree( mBuffer );

		// Move
		mBuffer = Other.mBuffer;
		Other.mBuffer = nullptr;
		}

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RkString& RkString::operator=( const RkString& Other )
	{
	if ( this != &Other )
		{
		rkFree( mBuffer );
		mBuffer = rkStrDup( Other );
		}

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RkString& RkString::operator=( const char* String )
	{	
	if ( mBuffer != String )
		{
		rkFree( mBuffer );
		mBuffer = rkStrDup( String );
		}
		
	return *this;
	}


//--------------------------------------------------------------------------------------------------
RkString& RkString::operator+=( const RkString& Other )
	{
	if ( !Other.Empty() )
		{
		char* Buffer = static_cast< char* >( rkAlloc( Length() + Other.Length() + 1 ) );
		Buffer[ 0 ] = '\0';

		if ( !Empty() )
			{
			rkStrCat( Buffer, mBuffer );
			}
		rkStrCat( Buffer, Other );

		rkFree( mBuffer );
		mBuffer = Buffer;
		}

	return *this;
	}

//--------------------------------------------------------------------------------------------------
RkString& RkString::operator+=( const char* String )
	{
	if ( String && *String != '\0' )
		{
		char* Buffer = static_cast< char* >( rkAlloc( Length() + rkStrLen( String ) + 1 ) );
		Buffer[ 0 ] = '\0';

		if ( !Empty() )
			{
			rkStrCat( Buffer, mBuffer );
			}
		rkStrCat( Buffer, String );

		rkFree( mBuffer );
		mBuffer = Buffer;
		}

	return *this;
	}


//--------------------------------------------------------------------------------------------------
RkString::operator const char*() const
	{
	return mBuffer ? mBuffer : "";
	}


//--------------------------------------------------------------------------------------------------
bool RkString::Empty() const
	{
	return !( mBuffer && *mBuffer != '\0' );
	}


//--------------------------------------------------------------------------------------------------
int RkString::Length() const
	{
	return mBuffer ? rkStrLen( mBuffer ) : 0;
	}


//--------------------------------------------------------------------------------------------------
void RkString::Clear()
	{
	rkFree( mBuffer );
	mBuffer = nullptr;
	}


//--------------------------------------------------------------------------------------------------
char* RkString::Data()
	{
	return mBuffer;
	}


//--------------------------------------------------------------------------------------------------
const char* RkString::Data() const
	{
	return mBuffer;
	}


//--------------------------------------------------------------------------------------------------
RkString operator+( const RkString& Lhs, const RkString& Rhs )
	{
	RkString Out = Lhs;
	Out += Rhs;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RkString operator+( const RkString& Lhs, const char* Rhs )
	{
	RkString Out = Lhs;
	Out += Rhs;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
bool operator<( const RkString& Lhs, const RkString& Rhs )
	{
	// No null check necessary since conversion operator returns *empty* string
	return rkStrCmp( Lhs, Rhs ) < 0;
	}


//--------------------------------------------------------------------------------------------------
bool operator==( const RkString& Lhs, const RkString& Rhs )
	{
	// No null check necessary since conversion operator returns *empty* string
	return rkStrCmp( Lhs, Rhs ) == 0;
	}


//--------------------------------------------------------------------------------------------------
bool operator!=( const RkString& Lhs, const RkString& Rhs )
	{
	// No null check necessary since conversion operator returns *empty* string
	return rkStrCmp( Lhs, Rhs ) != 0;
	}


//--------------------------------------------------------------------------------------------------
bool operator==( const RkString& Lhs, const char* Rhs )
	{
	if ( !Rhs )
		{
		return Lhs.Empty();
		}

	return rkStrCmp( Lhs, Rhs ) == 0;
	}


//--------------------------------------------------------------------------------------------------
bool operator!=( const RkString& Lhs, const char* Rhs )
	{
	if ( !Rhs )
		{
		return !Lhs.Empty();
		}

	return rkStrCmp( Lhs, Rhs ) != 0;
	}


//--------------------------------------------------------------------------------------------------
bool operator==( const char* Lhs, const RkString& Rhs )
	{
	if ( !Lhs )
		{
		return Rhs.Empty();
		}

	return rkStrCmp( Lhs, Rhs ) == 0;
	}


//--------------------------------------------------------------------------------------------------
bool operator!=( const char* Lhs, const RkString& Rhs )
	{
	if ( !Lhs )
		{
		return !Rhs.Empty();
		}

	return rkStrCmp( Lhs, Rhs ) != 0;
	}