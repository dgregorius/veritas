// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stddef.h>

#ifdef __cplusplus
#define B3_TYPE_OF( A ) auto
#else
#define B3_TYPE_OF typeof(A)
#endif

#define B3_SWAP( x, y )                                                                                                          \
	do                                                                                                                           \
	{                                                                                                                            \
		B3_TYPE_OF( x ) B3_SWAP_TEMP = x;                                                                                        \
		x = y;                                                                                                                   \
		y = B3_SWAP_TEMP;                                                                                                        \
	}                                                                                                                            \
	while ( 0 )
