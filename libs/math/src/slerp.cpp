/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/slerp.h>

using namespace mrpt;
using namespace mrpt::math;

void mrpt::math::slerp(
	const TPose3D& p0, const TPose3D& p1, const double t, TPose3D& p)
{
	CQuaternionDouble q0, q1, q;
	p0.getAsQuaternion(q0);
	p1.getAsQuaternion(q1);
	// The quaternion part (this will raise exception on t not in [0,1])
	mrpt::math::slerp(q0, q1, t, q);
	q.rpy(p.roll, p.pitch, p.yaw);
	// XYZ:
	p.x = (1 - t) * p0.x + t * p1.x;
	p.y = (1 - t) * p0.y + t * p1.y;
	p.z = (1 - t) * p0.z + t * p1.z;
}

void mrpt::math::slerp_ypr(
	const mrpt::math::TPose3D& q0, const mrpt::math::TPose3D& q1,
	const double t, mrpt::math::TPose3D& p)
{
	mrpt::math::CQuaternionDouble quat0(UNINITIALIZED_QUATERNION),
		quat1(UNINITIALIZED_QUATERNION), q(UNINITIALIZED_QUATERNION);
	q0.getAsQuaternion(quat0);
	q1.getAsQuaternion(quat1);
	mrpt::math::slerp(quat0, quat1, t, q);

	p.x = p.y = p.z = 0;
	q.rpy(p.roll, p.pitch, p.yaw);
}
