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
#include <mrpt/nav/reactive/TCandidateMovementPTG.h>
#include <mrpt/system/CTimeLogger.h>

namespace mrpt::nav
{
/** Handles velocity command generation with speed limits and filtering.
 *
 * Extracted from CAbstractPTGBasedReactive to reduce its complexity.
 * \ingroup nav_reactive
 */
class VelocityFilter
{
 public:
  /** Generates a velocity command from the selected candidate movement,
   * applying speed limits and filtering.
   * \return The [0,1] velocity scale of the raw PTG cmd_vel */
  double generateVelCmd(
      const TCandidateMovementPTG& in_movement,
      mrpt::kinematics::CVehicleVelCmd::Ptr& new_vel_cmd,
      double speedfilter_tau,
      const mrpt::kinematics::CVehicleVelCmd::TVelCmdParams& robot_absolute_speed_limits,
      double meanExecutionPeriod,
      mrpt::system::CTimeLogger& timelogger);

  /** Access to the last velocity command (for hysteresis comparisons) */
  [[nodiscard]] mrpt::kinematics::CVehicleVelCmd::Ptr getLastVelCmd() const
  {
    return m_last_vel_cmd;
  }

  /** Reset the last velocity command state */
  void resetLastVelCmd() { m_last_vel_cmd.reset(); }

 private:
  mrpt::kinematics::CVehicleVelCmd::Ptr m_last_vel_cmd;
};

}  // namespace mrpt::nav
