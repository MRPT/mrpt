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

#include <mrpt/kinematics/CVehicleSimulVirtualBase.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/random.h>

#include <algorithm>
#include <cmath>

using namespace mrpt::kinematics;

CVehicleSimulVirtualBase::CVehicleSimulVirtualBase() = default;

CVehicleSimulVirtualBase::~CVehicleSimulVirtualBase() = default;
void CVehicleSimulVirtualBase::setCurrentGTPose(const mrpt::math::TPose2D& pose)
{
  m_GT_pose = pose;
}

void CVehicleSimulVirtualBase::simulateOneTimeStep(const double dt)
{
  using mrpt::math::TPose2D;
  const double final_t = m_time + dt;
  while (m_time < final_t)
  {
    const double h = std::min(m_firmware_control_period, final_t - m_time);

    // Simulate movement during h:
    TPose2D nextOdometry = m_odometry;
    nextOdometry.x += m_odometric_vel.vx * h;
    nextOdometry.y += m_odometric_vel.vy * h;
    nextOdometry.phi += m_odometric_vel.omega * h;
    mrpt::math::wrapToPiInPlace(nextOdometry.phi);

    TPose2D gtDelta = m_GT_vel * h;
    // Add some errors, defined in the vehicle frame (std. per second):
    if (m_use_odo_error)
    {
      auto& rng = mrpt::random::getRandomGenerator();
      const double k = 1.0 / std::sqrt(h);
      TPose2D dl = m_GT_vel.rotated(-m_GT_pose.phi) * h;
      dl.x *= 1.0 + m_Ax_err_bias + k * m_Ax_err_std * rng.drawGaussian1D_normalized();
      dl.y *= 1.0 + m_Ay_err_bias + k * m_Ay_err_std * rng.drawGaussian1D_normalized();
      dl.phi *= 1.0 + m_Aphi_err_bias + k * m_Aphi_err_std * rng.drawGaussian1D_normalized();
      const double c = std::cos(m_GT_pose.phi);
      const double s = std::sin(m_GT_pose.phi);
      gtDelta = TPose2D(c * dl.x - s * dl.y, s * dl.x + c * dl.y, dl.phi);
    }

    TPose2D nextGT = {
        m_GT_pose.x + gtDelta.x, m_GT_pose.y + gtDelta.y, m_GT_pose.phi + gtDelta.phi};
    mrpt::math::wrapToPiInPlace(nextGT.phi);

    this->internal_simulControlStep(h);

    // Now rotate our current Odo velocity into GT coordinates
    m_GT_vel = getCurrentOdometricVelLocal();
    m_GT_vel.rotate(m_GT_pose.phi);

    m_odometry = nextOdometry;
    m_GT_pose = nextGT;

    m_time = (h < m_firmware_control_period) ? final_t : m_time + h;
  }
}

void CVehicleSimulVirtualBase::resetStatus()
{
  m_GT_pose = mrpt::math::TPose2D(.0, .0, .0);
  m_GT_vel = mrpt::math::TTwist2D(.0, .0, .0);
  m_odometry = mrpt::math::TPose2D(.0, .0, .0);
  m_odometric_vel = mrpt::math::TTwist2D(.0, .0, .0);
  internal_clear();
}

void CVehicleSimulVirtualBase::resetTime() { m_time = .0; }
mrpt::math::TTwist2D CVehicleSimulVirtualBase::getCurrentGTVelLocal() const
{
  mrpt::math::TTwist2D tl = this->m_GT_vel;
  tl.rotate(-m_GT_pose.phi);
  return tl;
}

mrpt::math::TTwist2D CVehicleSimulVirtualBase::getCurrentOdometricVelLocal() const
{
  mrpt::math::TTwist2D tl = this->m_odometric_vel;
  tl.rotate(-m_odometry.phi);
  return tl;
}
