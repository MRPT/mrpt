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
#pragma once

#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/system/datetime.h>

namespace mrpt::kinematics
{
/** Virtual base for planar vehicle kinematic simulators (see
 * CVehicleSimul_DiffDriven, CVehicleSimul_Holo), including odometry errors.
 *  Main API methods are:
 *  - sendVelCmd(): sends a velocity command to the vehicle, whose dynamics
 *    depend on the derived class.
 *  - simulateOneTimeStep(): runs the simulator for a given time period.
 *
 * \ingroup mrpt_kinematics_grp
 */
class CVehicleSimulVirtualBase
{
 public:
  CVehicleSimulVirtualBase();
  virtual ~CVehicleSimulVirtualBase();

  /** @name Kinematic simulation and control interface
   * @{ */
  /** Runs the simulator during "dt" seconds. It will be split into periods of
   * "m_firmware_control_period", the last one shortened to end exactly at the
   * requested time. Non-positive values of dt have no effect. */
  void simulateOneTimeStep(const double dt);

  /** Returns the instantaneous, ground truth pose in world coordinates */
  const mrpt::math::TPose2D& getCurrentGTPose() const { return m_GT_pose; }
  /** Brute-force move robot to target coordinates ("teleport") */
  void setCurrentGTPose(const mrpt::math::TPose2D& pose);

  /** Returns the current pose according to (noisy) odometry \sa
   * setOdometryErrors */
  const mrpt::math::TPose2D& getCurrentOdometricPose() const { return m_odometry; }
  /** Brute-force overwrite robot odometry  */
  template <typename T>
  void setCurrentOdometricPose(const T& pose)
  {
    m_odometry = mrpt::math::TPose2D(pose);
  }

  /** Returns the instantaneous, ground truth velocity vector (vx,vy,omega) in
   * world coordinates. With odometry errors enabled, this is the noise-free
   * commanded velocity, not the rate of change of the ground truth pose. */
  const mrpt::math::TTwist2D& getCurrentGTVel() const { return m_GT_vel; }
  /** Returns the instantaneous, ground truth velocity vector (vx,vy,omega) in
   * the robot local frame */
  mrpt::math::TTwist2D getCurrentGTVelLocal() const;

  /** Returns the instantaneous, odometric velocity vector (vx,vy,omega) in
   * the odometry frame of reference */
  const mrpt::math::TTwist2D& getCurrentOdometricVel() const { return m_odometric_vel; }
  /** Returns the instantaneous, odometric velocity vector (vx,vy,omega) in
   * the robot local frame */
  mrpt::math::TTwist2D getCurrentOdometricVelLocal() const;

  /** Get the current simulation time */
  double getTime() const { return m_time; }
  /** Sends a velocity command to the robot. The number of components and
   * their meaning depends
   * on the vehicle-kinematics derived class */
  virtual void sendVelCmd(const CVehicleVelCmd& cmd_vel) = 0;
  /** Gets an empty velocity command object that can be queried to find out
   * the number of velcmd components,... */
  virtual CVehicleVelCmd::Ptr getVelCmdType() const = 0;

  /** Enable/Disable odometry errors. Errors are relative factors applied to
   * the displacement in the vehicle frame (x: forward, y: left, phi: heading
   * change). Bias values are relative errors; std values are the 1-sigma
   * relative errors accumulated over 1 second of motion (Gaussian white noise,
   * independent of the internal control period). All of them are
   * dimensionless: the DEG2RAD() in the default Aphi values is not a unit
   * conversion. */
  void setOdometryErrors(
      bool enabled,
      double Ax_err_bias = 1e-3,
      double Ax_err_std = 10e-3,
      double Ay_err_bias = 1e-3,
      double Ay_err_std = 10e-3,
      double Aphi_err_bias = mrpt::DEG2RAD(1e-3),
      double Aphi_err_std = mrpt::DEG2RAD(10e-3))
  {
    m_use_odo_error = enabled;
    m_Ax_err_bias = Ax_err_bias;
    m_Ax_err_std = Ax_err_std;
    m_Ay_err_bias = Ay_err_bias;
    m_Ay_err_std = Ay_err_std;
    m_Aphi_err_bias = Aphi_err_bias;
    m_Aphi_err_std = Aphi_err_std;
  }

  /** Reset all simulator variables to 0 (except the simulation time).
   * \sa resetTime */
  void resetStatus();
  /** Reset time counter \sa resetStatus */
  void resetTime();

  /** @} */

 protected:
  /** @name State vector
   *  @{ */
  /** simulation running time */
  double m_time;
  /** ground truth pose in world coordinates. */
  mrpt::math::TPose2D m_GT_pose;
  /** Velocity in (x,y,omega) */
  mrpt::math::TTwist2D m_GT_vel;
  /** Velocity in (x,y,omega) */
  mrpt::math::TTwist2D m_odometric_vel;
  mrpt::math::TPose2D m_odometry;
  /** @} */
  /** The period at which the low-level controller updates velocities
   * (Default: 0.5 ms) */
  double m_firmware_control_period{500e-6};

  /** Whether to corrupt odometry with noise */
  bool m_use_odo_error{false};
  double m_Ax_err_bias, m_Ax_err_std;
  double m_Ay_err_bias, m_Ay_err_std;
  double m_Aphi_err_bias, m_Aphi_err_std;

  virtual void internal_simulControlStep(const double dt) = 0;
  /** Resets all pending cmds */
  virtual void internal_clear() = 0;

 private:
};

}  // namespace mrpt::kinematics
