//--------------------------------------------------------------------------------------------------
/*
	@file		tracy.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

// Thread naming for Tracy
#include <taskflow/taskflow.hpp>


//--------------------------------------------------------------------------------------------------
// Tracy
//--------------------------------------------------------------------------------------------------
#ifndef RK_PROFILE_LEVEL
#  define RK_PROFILE_LEVEL 2
#endif

#ifdef TRACY_ENABLE
#  include <tracy/tracy.hpp>
#  define RK_TRACY_FRAME() FrameMark
#  define RK_TRACY_ZONE_LVL1( Name, Color ) ZoneScopedNC( Name, Color )
#  if RK_PROFILE_LEVEL >= 2
#    define RK_TRACY_ZONE_LVL2( Name, Color ) ZoneScopedNC( Name, Color )
#  else
#    define RK_TRACY_ZONE_LVL2( Name, Color )
#  endif
#  if RK_PROFILE_LEVEL >= 3
#    define RK_TRACY_ZONE_LVL3( Name, Color ) ZoneScopedNC( Name, Color )
#  else
#    define RK_TRACY_ZONE_LVL3( Name, Color )
#  endif
#else
#  define RK_TRACY_FRAME()
#  define RK_TRACY_ZONE_LVL1( Name, Color ) 
#  define RK_TRACY_ZONE_LVL2( Name, Color ) 
#  define RK_TRACY_ZONE_LVL3( Name, Color ) 
#endif


//--------------------------------------------------------------------------------------------------
// Tracy Profiler Color Scheme for Physics
//--------------------------------------------------------------------------------------------------
// Color hierarchy: Stronger colors at parent level, progressively lighter for children
// Tasks have cyan tint + border for easy identification of parallel work
constexpr unsigned int Tracy_Step = 0xDD1C5E;
	constexpr unsigned int Tracy_Broadphase = 0x1E5FAA;
		constexpr unsigned int Tracy_UpdatePairs = 0x3B72B0;
			constexpr unsigned int Tracy_UpdatePairsTask = 0x5D8FC8;
		constexpr unsigned int Tracy_AddContacts = 0x4580BA;
	constexpr unsigned int Tracy_Narrowphase = 0xD45500;
		constexpr unsigned int Tracy_CollideContacts = 0xDD6520;
			constexpr unsigned int Tracy_CollideContactsTask = 0xEA8A55;
		constexpr unsigned int Tracy_UpdateContacts = 0xE07528;
	constexpr unsigned int Tracy_Solve = 0x0D8540;
		constexpr unsigned int Tracy_IntegrateForces = 0x1A8F4E;
			constexpr unsigned int Tracy_IntegrateForcesTask = 0x3AAA6C;
		constexpr unsigned int Tracy_LoadConstraints = 0x188B4A;
			constexpr unsigned int Tracy_LoadConstraintsTask = 0x38A668;
		constexpr unsigned int Tracy_SolveConstraints = 0x1C9352;
			constexpr unsigned int Tracy_SolveConstraintsTask = 0x3CAE70;
		constexpr unsigned int Tracy_SaveConstraints = 0x178746;
			constexpr unsigned int Tracy_SaveConstraintsTask = 0x37A264;
		constexpr unsigned int Tracy_IntegrateVelocities = 0x158344;
			constexpr unsigned int Tracy_IntegrateVelocitiesTask = 0x359E62;
		constexpr unsigned int Tracy_RefitProxies = 0x117B52;
			constexpr unsigned int Tracy_UpdateBounds = 0x239166;
				constexpr unsigned int Tracy_UpdateBoundsTask = 0x3FAC84;
			constexpr unsigned int Tracy_PropagateBounds = 0x208C61;
		constexpr unsigned int Tracy_Sleeping = 0x0A6B35;
			constexpr unsigned int Tracy_AdvanceIslands = 0x1A8148;
				constexpr unsigned int Tracy_AdvanceIslandsTask = 0x359C66;
			constexpr unsigned int Tracy_SleepIslands = 0x187D45;

constexpr unsigned int Tracy_BackgroundTask = 0xC62828;


//--------------------------------------------------------------------------------------------------
// Tracy Profiler Thread naming (Taskflow)
//--------------------------------------------------------------------------------------------------
class TracyWorkerInterface : public tf::WorkerInterface
	{
	public:
		TracyWorkerInterface();

		virtual void scheduler_prologue( tf::Worker& Worker ) override;
		virtual void scheduler_epilogue( tf::Worker& Worker, std::exception_ptr ) override;
	};