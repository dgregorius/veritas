//--------------------------------------------------------------------------------------------------
/*
	@file		string.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/assert.h"
#include "ragnarok/common/types.h"


//--------------------------------------------------------------------------------------------------
// RkString
//--------------------------------------------------------------------------------------------------
class RkString
	{
	public:
		RkString() = default;
		RkString( RkString&& Other );
		RkString( const RkString& Other );
		RkString( const char* String );
		~RkString();

		RkString& operator=( RkString&& Other );
		RkString& operator=( const RkString& Other );
		RkString& operator=( const char* String );

		RkString& operator+=( const RkString& Other );
		RkString& operator+=( const char* String );
		
		operator const char*() const;

		bool Empty() const;
		int Length() const;
		void Clear();

		char* Data();
		const char* Data() const;

	private:
		char* mBuffer = nullptr;
	};

// Binary operators
RkString operator+( const RkString& Lhs, const RkString& Rhs );
RkString operator+( const RkString& Lhs, const char* Rhs );

bool operator<( const RkString& Lhs, const RkString& Rhs );
bool operator==( const RkString& Lhs, const RkString& Rhs );
bool operator!=( const RkString& Lhs, const RkString& Rhs );
bool operator==( const RkString& Lhs, const char* Rhs );
bool operator!=( const RkString& Lhs, const char* Rhs );
bool operator==( const char* Lhs, const RkString& Rhs );
bool operator!=( const char* Lhs, const RkString& Rhs );