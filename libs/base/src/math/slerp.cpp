/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/math/slerp.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;


void mrpt::math::slerp(
	const CPose3D  & p0,
	const CPose3D  & p1,
	const double     t,
	CPose3D        & p)
{
	CQuaternionDouble q0,q1,q;
	p0.getAsQuaternion(q0);
	p1.getAsQuaternion(q1);
	// The quaternion part (this will raise exception on t not in [0,1])
	mrpt::math::slerp(q0,q1,t, q);
	// XYZ:
	p = CPose3D(
		q,
		(1-t)*p0.x()+t*p1.x(),
		(1-t)*p0.y()+t*p1.y(),
		(1-t)*p0.z()+t*p1.z() );
}

void mrpt::math::slerp(
	const CPose3DQuat & q0,
	const CPose3DQuat & q1,
	const double        t,
	CPose3DQuat       & q)
{
	// The quaternion part (this will raise exception on t not in [0,1])
	mrpt::math::slerp(q0.quat(), q1.quat(),t, q.quat());
	// XYZ:
	q.x( (1-t)*q0.x()+t*q1.x() );
	q.y( (1-t)*q0.y()+t*q1.y() );
	q.z( (1-t)*q0.z()+t*q1.z() );
}
