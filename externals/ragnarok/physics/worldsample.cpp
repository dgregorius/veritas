//--------------------------------------------------------------------------------------------------
// worldsample.cpp	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "worldsample.h"


//--------------------------------------------------------------------------------------------------
// RkWorldSample
//--------------------------------------------------------------------------------------------------
RkWorldSample rkAdd( const RkWorldSample& Lhs, const RkWorldSample& Rhs )
	{
	RkWorldSample Out;
	Out.Step = Lhs.Step + Rhs.Step;
		Out.Broadphase = Lhs.Broadphase + Rhs.Broadphase;
			Out.UpdatePairs = Lhs.UpdatePairs + Rhs.UpdatePairs;
			Out.AddContacts = Lhs.AddContacts + Rhs.AddContacts;
		Out.Narrowphase = Lhs.Narrowphase + Rhs.Narrowphase;
			Out.CollideContacts = Lhs.CollideContacts + Rhs.CollideContacts;
			Out.UpdateContacts = Lhs.UpdateContacts + Rhs.UpdateContacts;
		Out.Solve = Lhs.Solve + Rhs.Solve;
			Out.Allocate = Lhs.Allocate + Rhs.Allocate;
			Out.IntegrateForces = Lhs.IntegrateForces + Rhs.IntegrateForces;
			Out.LoadConstraints = Lhs.LoadConstraints + Rhs.LoadConstraints;
			Out.SolveConstraints = Lhs.SolveConstraints + Rhs.SolveConstraints;
			Out.SaveConstraints = Lhs.SaveConstraints + Rhs.SaveConstraints;
			Out.IntegrateVelocities = Lhs.IntegrateVelocities + Rhs.IntegrateVelocities;
			Out.RefitProxies = Lhs.RefitProxies + Rhs.RefitProxies;
				Out.UpdateBounds = Lhs.UpdateBounds + Rhs.UpdateBounds;
				Out.PropagateBounds = Lhs.PropagateBounds + Rhs.PropagateBounds;
			Out.Sleeping = Lhs.Sleeping + Rhs.Sleeping;
				Out.AdvanceIslands = Lhs.AdvanceIslands + Rhs.AdvanceIslands;
				Out.SleepIslands = Lhs.SleepIslands + Rhs.SleepIslands;

	Out.RebuildTask = Lhs.RebuildTask + Rhs.RebuildTask;
	Out.SplitTask = Lhs.SplitTask + Rhs.SplitTask;
		
	return Out;
	}


//--------------------------------------------------------------------------------------------------
RkWorldSample rkMin( const RkWorldSample& Lhs, const RkWorldSample& Rhs )
	{
	RkWorldSample Out;
	Out.Step = rkMin( Lhs.Step, Rhs.Step );
		Out.Broadphase = rkMin( Lhs.Broadphase, Rhs.Broadphase );
			Out.UpdatePairs = rkMin( Lhs.UpdatePairs, Rhs.UpdatePairs );
			Out.AddContacts = rkMin( Lhs.AddContacts, Rhs.AddContacts );
		Out.Narrowphase = rkMin( Lhs.Narrowphase, Rhs.Narrowphase );
			Out.CollideContacts = rkMin( Lhs.CollideContacts, Rhs.CollideContacts );
			Out.UpdateContacts = rkMin( Lhs.UpdateContacts, Rhs.UpdateContacts );
	Out.Solve = rkMin( Lhs.Solve, Rhs.Solve );
		Out.Allocate = rkMin( Lhs.Allocate, Rhs.Allocate );
		Out.IntegrateForces = rkMin( Lhs.IntegrateForces, Rhs.IntegrateForces );
		Out.LoadConstraints = rkMin( Lhs.LoadConstraints, Rhs.LoadConstraints );
		Out.SolveConstraints = rkMin( Lhs.SolveConstraints, Rhs.SolveConstraints );
		Out.SaveConstraints = rkMin( Lhs.SaveConstraints, Rhs.SaveConstraints );
		Out.IntegrateVelocities = rkMin( Lhs.IntegrateVelocities, Rhs.IntegrateVelocities );
		Out.RefitProxies = rkMin( Lhs.RefitProxies, Rhs.RefitProxies );
			Out.UpdateBounds = rkMin( Lhs.UpdateBounds, Rhs.UpdateBounds );
			Out.PropagateBounds = rkMin( Lhs.PropagateBounds, Rhs.PropagateBounds );
		Out.Sleeping = rkMin( Lhs.Sleeping, Rhs.Sleeping );
			Out.AdvanceIslands = rkMin( Lhs.AdvanceIslands, Rhs.AdvanceIslands );
			Out.SleepIslands = rkMin( Lhs.SleepIslands, Rhs.SleepIslands );
		
	Out.RebuildTask = rkMin( Lhs.RebuildTask, Rhs.RebuildTask );
	Out.SplitTask = rkMin( Lhs.SplitTask, Rhs.SplitTask );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RkWorldSample rkMax( const RkWorldSample& Lhs, const RkWorldSample& Rhs )
	{
	RkWorldSample Out;
	Out.Step = rkMax( Lhs.Step, Rhs.Step );
		Out.Broadphase = rkMax( Lhs.Broadphase, Rhs.Broadphase );
			Out.UpdatePairs = rkMax( Lhs.UpdatePairs, Rhs.UpdatePairs );
			Out.AddContacts = rkMax( Lhs.AddContacts, Rhs.AddContacts );
		Out.Narrowphase = rkMax( Lhs.Narrowphase, Rhs.Narrowphase );
			Out.CollideContacts = rkMax( Lhs.CollideContacts, Rhs.CollideContacts );
			Out.UpdateContacts = rkMax( Lhs.UpdateContacts, Rhs.UpdateContacts );
		Out.Solve = rkMax( Lhs.Solve, Rhs.Solve );
			Out.Allocate = rkMax( Lhs.Allocate, Rhs.Allocate );
			Out.IntegrateForces = rkMax( Lhs.IntegrateForces, Rhs.IntegrateForces );
			Out.LoadConstraints = rkMax( Lhs.LoadConstraints, Rhs.LoadConstraints );
			Out.SolveConstraints = rkMax( Lhs.SolveConstraints, Rhs.SolveConstraints );
			Out.SaveConstraints = rkMax( Lhs.SaveConstraints, Rhs.SaveConstraints );
			Out.IntegrateVelocities = rkMax( Lhs.IntegrateVelocities, Rhs.IntegrateVelocities );
			Out.RefitProxies = rkMax( Lhs.RefitProxies, Rhs.RefitProxies );
				Out.UpdateBounds = rkMax( Lhs.UpdateBounds, Rhs.UpdateBounds );
				Out.PropagateBounds = rkMax( Lhs.PropagateBounds, Rhs.PropagateBounds );
			Out.Sleeping = rkMax( Lhs.Sleeping, Rhs.Sleeping );
				Out.AdvanceIslands = rkMax( Lhs.AdvanceIslands, Rhs.AdvanceIslands );
				Out.SleepIslands = rkMax( Lhs.SleepIslands, Rhs.SleepIslands );

	Out.RebuildTask = rkMax( Lhs.RebuildTask, Rhs.RebuildTask );
	Out.SplitTask = rkMax( Lhs.SplitTask, Rhs.SplitTask );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RkWorldSample operator+( const RkWorldSample& Lhs, const RkWorldSample& Rhs )
	{
	return rkAdd( Lhs, Rhs );
	}


//--------------------------------------------------------------------------------------------------
RkWorldSample operator*( float Lhs, const RkWorldSample& Rhs )
	{
	RkWorldSample Out;
	Out.Step = Lhs * Rhs.Step;
		Out.Broadphase= Lhs * Rhs.Broadphase;
			Out.UpdatePairs = Lhs * Rhs.UpdatePairs;
			Out.AddContacts = Lhs * Rhs.AddContacts;
		Out.Narrowphase = Lhs * Rhs.Narrowphase;
			Out.CollideContacts = Lhs * Rhs.CollideContacts;
			Out.UpdateContacts = Lhs * Rhs.UpdateContacts;
		Out.Solve = Lhs * Rhs.Solve;
			Out.Allocate = Lhs * Rhs.Allocate;
			Out.IntegrateForces = Lhs * Rhs.IntegrateForces;
			Out.LoadConstraints = Lhs * Rhs.LoadConstraints;
			Out.SolveConstraints = Lhs * Rhs.SolveConstraints;
			Out.SaveConstraints = Lhs * Rhs.SaveConstraints;
			Out.IntegrateVelocities = Lhs * Rhs.IntegrateVelocities;
			Out.RefitProxies = Lhs * Rhs.RefitProxies;
				Out.UpdateBounds = Lhs * Rhs.UpdateBounds;
				Out.PropagateBounds = Lhs * Rhs.PropagateBounds;
			Out.Sleeping = Lhs * Rhs.Sleeping;
				Out.AdvanceIslands = Lhs * Rhs.AdvanceIslands;
				Out.SleepIslands = Lhs * Rhs.SleepIslands;

	Out.RebuildTask = Lhs * Rhs.RebuildTask;
	Out.SplitTask = Lhs * Rhs.SplitTask;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RkWorldSample operator*( const RkWorldSample& Lhs, float Rhs )
	{
	RkWorldSample Out;
	Out.Step = Lhs.Step * Rhs;
		Out.Broadphase= Lhs.Broadphase * Rhs;
			Out.UpdatePairs = Lhs.UpdatePairs * Rhs;
			Out.AddContacts = Lhs.AddContacts * Rhs;
		Out.Narrowphase = Lhs.Narrowphase * Rhs;
			Out.CollideContacts = Lhs.CollideContacts * Rhs;
			Out.UpdateContacts = Lhs.UpdateContacts * Rhs;
		Out.Solve = Lhs.Solve * Rhs;
			Out.Allocate = Lhs.Allocate * Rhs;
			Out.IntegrateForces = Lhs.IntegrateForces * Rhs;
			Out.LoadConstraints = Lhs.LoadConstraints * Rhs;
			Out.SolveConstraints = Lhs.SolveConstraints * Rhs;
			Out.SaveConstraints = Lhs.SaveConstraints * Rhs;
			Out.IntegrateVelocities = Lhs.IntegrateVelocities * Rhs;
			Out.RefitProxies = Lhs.RefitProxies * Rhs;
				Out.UpdateBounds = Lhs.UpdateBounds * Rhs;
				Out.PropagateBounds = Lhs.PropagateBounds * Rhs;
			Out.Sleeping = Lhs.Sleeping * Rhs;
				Out.AdvanceIslands = Lhs.AdvanceIslands * Rhs;
				Out.SleepIslands = Lhs.SleepIslands * Rhs;

	Out.RebuildTask = Lhs.RebuildTask * Rhs;
	Out.SplitTask = Lhs.SplitTask * Rhs;

	return Out;
	}


//--------------------------------------------------------------------------------------------------
RkWorldSample operator/( const RkWorldSample& Lhs, float Rhs )
	{
	RK_ASSERT( Rhs > 0.0f );

	RkWorldSample Out;
	Out.Step = Lhs.Step / Rhs;
		Out.Broadphase= Lhs.Broadphase / Rhs;
			Out.UpdatePairs = Lhs.UpdatePairs / Rhs;
			Out.AddContacts = Lhs.AddContacts / Rhs;
		Out.Narrowphase = Lhs.Narrowphase / Rhs;
			Out.CollideContacts = Lhs.CollideContacts / Rhs;
			Out.UpdateContacts = Lhs.UpdateContacts / Rhs;
		Out.Solve = Lhs.Solve / Rhs;
			Out.Allocate = Lhs.Allocate / Rhs;
			Out.IntegrateForces = Lhs.IntegrateForces / Rhs;
			Out.LoadConstraints = Lhs.LoadConstraints / Rhs;
			Out.SolveConstraints = Lhs.SolveConstraints / Rhs;
			Out.SaveConstraints = Lhs.SaveConstraints / Rhs;
			Out.IntegrateVelocities = Lhs.IntegrateVelocities / Rhs;
			Out.RefitProxies = Lhs.RefitProxies / Rhs;
				Out.UpdateBounds = Lhs.UpdateBounds / Rhs;
				Out.PropagateBounds = Lhs.PropagateBounds / Rhs;
			Out.Sleeping = Lhs.Sleeping / Rhs;
				Out.AdvanceIslands = Lhs.AdvanceIslands / Rhs;
				Out.SleepIslands = Lhs.SleepIslands / Rhs;

	Out.RebuildTask = Lhs.RebuildTask / Rhs;
	Out.SplitTask = Lhs.SplitTask / Rhs;

	return Out;
	}