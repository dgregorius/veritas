//--------------------------------------------------------------------------------------------------
/*
	@file		constants.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once


//--------------------------------------------------------------------------------------------------
// General 
//--------------------------------------------------------------------------------------------------
//! A small distance used as collision detection and dynamics tolerance. It is chosen 
//! to be numerically significant, but visually insignificant.
#define RK_LINEAR_SLOP					0.005f

//! A small angle used a collision detection and dynamics tolerance. It is chosen 
//! to be numerically significant, but visually insignificant.
#define RK_ANGULAR_SLOP					(2.0f / 180.0f * RK_PI)


//--------------------------------------------------------------------------------------------------
// Broadphase 
//--------------------------------------------------------------------------------------------------
//! This is used to fatten AABBs in the dynamic tree. This allows proxies
//! to move by a small amount without triggering a tree adjustment. 
#define RK_BOUNDS_EXTENSION				0.05f	


//--------------------------------------------------------------------------------------------------
// Narrowphase 
//--------------------------------------------------------------------------------------------------
//! An invalid feature pair id that should be skipped for matching contacts.
#define RK_INVALID_FEATURE_PAIR			RK_U32_MAX

//! The maximum number of contact points between two touching shapes. 
#define RK_MAX_MANIFOLD_POINTS			4

//! The maximum number of manifolds in a mesh contact.
#define RK_MAX_MANIFOLDS				3

//! The radius of a shape skin. This should not be modified. Making this 
//! smaller means polytopes will have an insufficient buffer for continuous 
//! collision. Making it larger may create artifacts for vertex collision.
#define RK_CONVEX_RADIUS				(2.0f * RK_LINEAR_SLOP)


//--------------------------------------------------------------------------------------------------
// Dynamics 
//--------------------------------------------------------------------------------------------------
//! The minimum relative velocity threshold below we drop restitution
#define RK_RESTITUTION_THRESHOLD		1.0f

//! Sleeping threshold
#define RK_SLEEP_TIME					0.5f
#define RK_SLEEP_THRESHOLD				0.05f

//! Maximum number of graph colors in constraint solver
#define RK_MAX_GRAPH_COLORS				32
#define RK_DYNAMIC_GRAPH_COLORS			24
#define RK_STATIC_GRAPH_COLORS			8