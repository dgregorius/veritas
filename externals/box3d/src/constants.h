// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

extern float b3_lengthUnitsPerMeter;

// Used to detect bad values. Positions greater than about 16km will have precision
// problems, so 100km as a limit should be fine in all cases.
#define B3_HUGE ( 100000.0f * b3_lengthUnitsPerMeter )

// Maximum parallel workers. Used to size some static arrays.
#define B3_MAX_WORKERS 64

// Maximum number of colors in the constraint graph. Constraints that cannot
// find a color are added to the overflow set which are solved single-threaded.
// The compound barrel benchmark has minor overflow with 24 colors
#define B3_GRAPH_COLOR_COUNT 24

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant. In meters.
// @warning modifying this can have a significant impact on stability
#define B3_LINEAR_SLOP ( 0.005f * b3_lengthUnitsPerMeter )

// The distance between shapes where they are considered overlapped. This is needed
// because GJK may return small positive values for overlapped shapes in degenerate
// configurations.
#define B3_OVERLAP_SLOP ( 0.1f * B3_LINEAR_SLOP )

// Maximum number of simultaneous worlds that can be allocated
#ifndef B3_MAX_WORLDS
#define B3_MAX_WORLDS 128
#endif

// The maximum rotation of a body per time step. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
// @warning increasing this to 0.5f * b3_pi or greater will break continuous collision.
#define B3_MAX_ROTATION ( 0.25f * B3_PI )

// @warning modifying this can have a significant impact on performance and stability
#define B3_SPECULATIVE_DISTANCE ( 4.0f * B3_LINEAR_SLOP )

// This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment. This is in meters.
// @warning modifying this can have a significant impact on performance
#define B3_AABB_MARGIN ( 0.05f * b3_lengthUnitsPerMeter )

// The time that a body must be still before it will go to sleep. In seconds.
#define B3_TIME_TO_SLEEP 0.5f

// An invalid feature pair id that should be skipped for matching contacts.
#define B3_INVALID_FEATURE_PAIR ( ~0u )

// The maximum number of manifolds in a mesh contact.
#define B3_MAX_MANIFOLDS 3

// Body and shape name lengths
#define B3_NAME_LENGTH 64

// These generous limits allow for easy hashing. See b3ShapePairKey.
#define B3_SHAPE_POWER 22
#define B3_CHILD_POWER ( 64 - 2 * B3_SHAPE_POWER )
#define B3_MAX_SHAPES ( 1 << B3_SHAPE_POWER )
#define B3_MAX_CHILD_SHAPES ( 1 << B3_CHILD_POWER )
