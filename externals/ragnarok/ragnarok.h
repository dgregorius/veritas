//--------------------------------------------------------------------------------------------------
/**
	@file		ragnarok.h

	@author		Dirk Gregorius
	@version	0.1
	@date		02/15/2025

    This file constitutes the Ragnarok API

	Copyright(C) D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

// Common
#include "ragnarok/common/arena.h"
#include "ragnarok/common/array.h"
#include "ragnarok/common/assert.h"
#include "ragnarok/common/bitset.h"
#include "ragnarok/common/delegate.h"
#include "ragnarok/common/log.h"
#include "ragnarok/common/math.h"
#include "ragnarok/common/memory.h"
#include "ragnarok/common/pool.h"
#include "ragnarok/common/resource.h"
#include "ragnarok/common/signal.h"
#include "ragnarok/common/string.h"
#include "ragnarok/common/timer.h"
#include "ragnarok/common/tracy.h"
#include "ragnarok/common/types.h"

// Physics
#include "ragnarok/physics/body.h"
#include "ragnarok/physics/broadphase.h"
#include "ragnarok/physics/bvh.h"
#include "ragnarok/physics/capsule.h"
#include "ragnarok/physics/capsuleshape.h"
#include "ragnarok/physics/clipping.h"
#include "ragnarok/physics/constants.h"
#include "ragnarok/physics/constraintgraph.h"
#include "ragnarok/physics/contact.h"
#include "ragnarok/physics/contactsolver.h"
#include "ragnarok/physics/convexcontact.h"
#include "ragnarok/physics/dynamictree.h"
#include "ragnarok/physics/event.h"
#include "ragnarok/physics/filter.h"
#include "ragnarok/physics/gjk.h"
#include "ragnarok/physics/gjksimplex.h"
#include "ragnarok/physics/hull.h"
#include "ragnarok/physics/hullshape.h"
#include "ragnarok/physics/island.h"
#include "ragnarok/physics/joint.h"
#include "ragnarok/physics/jointsolver.h"
#include "ragnarok/physics/manifold.h"
#include "ragnarok/physics/mass.h"
#include "ragnarok/physics/meshcontact.h"
#include "ragnarok/physics/meshshape.h"
#include "ragnarok/physics/prismaticjoint.h"
#include "ragnarok/physics/proxy.h"
#include "ragnarok/physics/revolutejoint.h"
#include "ragnarok/physics/rigidjoint.h"
#include "ragnarok/physics/sat.h"
#include "ragnarok/physics/shape.h"
#include "ragnarok/physics/solver.h"
#include "ragnarok/physics/sphere.h"
#include "ragnarok/physics/sphereshape.h"
#include "ragnarok/physics/sphericaljoint.h"
#include "ragnarok/physics/toi.h"
#include "ragnarok/physics/world.h"
#include "ragnarok/physics/worldsample.h"