/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/math/slerp.h>

using namespace mrpt;
using namespace mrpt::math;

namespace
{
// Via the rotation matrix, which keeps the rotation exact near gimbal lock:
void quatToYPR(const CQuaternionDouble& q, TPose3D& p)
{
  CMatrixDouble33 R;
  q.rotationMatrixNoResize(R);
  TPose3D::SO3_to_yaw_pitch_roll(R, p.yaw, p.pitch, p.roll);
}
}  // namespace

void mrpt::math::slerp(const TPose3D& p0, const TPose3D& p1, const double t, TPose3D& p)
{
  CQuaternionDouble q0, q1, q;
  p0.getAsQuaternion(q0);
  p1.getAsQuaternion(q1);
  // The quaternion part (this will raise exception on t not in [0,1])
  mrpt::math::slerp(q0, q1, t, q);
  quatToYPR(q, p);
  // XYZ:
  p.x = (1 - t) * p0.x + t * p1.x;
  p.y = (1 - t) * p0.y + t * p1.y;
  p.z = (1 - t) * p0.z + t * p1.z;
}

void mrpt::math::slerp_ypr(
    const mrpt::math::TPose3D& q0,
    const mrpt::math::TPose3D& q1,
    const double t,
    mrpt::math::TPose3D& p)
{
  mrpt::math::CQuaternionDouble quat0(UNINITIALIZED_QUATERNION), quat1(UNINITIALIZED_QUATERNION),
      q(UNINITIALIZED_QUATERNION);
  q0.getAsQuaternion(quat0);
  q1.getAsQuaternion(quat1);
  mrpt::math::slerp(quat0, quat1, t, q);

  p.x = p.y = p.z = 0;
  quatToYPR(q, p);
}
