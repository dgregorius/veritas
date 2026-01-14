//--------------------------------------------------------------------------------------------------
/*
	@file		event.h

	@author		Dirk Gregorius
	@version	0.1
	@date		07/01/2012

	Copyright(C) by D. Gregorius. All rights reserved.
*/
//--------------------------------------------------------------------------------------------------
#pragma once

#include "ragnarok/common/signal.h"

class RkBody;
class RkShape;
class RkJoint;
class RkWorld;


//--------------------------------------------------------------------------------------------------
// RkEvent
//--------------------------------------------------------------------------------------------------
struct RkEvent
	{ 
	static inline RkSignal< void ( RkBody*, RkShape* ) > ShapeAdded;
	static inline RkSignal< void ( RkBody*, RkShape* ) > ShapeRemoved;
	static inline RkSignal< void( RkWorld*, RkBody* ) > BodyAdded;
	static inline RkSignal< void( RkWorld*, RkBody* ) > BodyRemoved;
	static inline RkSignal< void( RkWorld*, RkJoint* ) > JointAdded;
	static inline RkSignal< void( RkWorld*, RkJoint* ) > JointRemoved;

	static inline RkSignal< void( RkWorld*, RkContact* ) > TouchBegin;
	static inline RkSignal< void( RkWorld*, RkContact* ) > TouchEnd;
	};