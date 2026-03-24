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

#include <mrpt/nav/reactive/VelocityFilter.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>

#include <iostream>

using namespace mrpt::nav;

double VelocityFilter::generateVelCmd(
    const TCandidateMovementPTG& in_movement,
    mrpt::kinematics::CVehicleVelCmd::Ptr& new_vel_cmd,
    double speedfilter_tau,
    const mrpt::kinematics::CVehicleVelCmd::TVelCmdParams& robot_absolute_speed_limits,
    double meanExecutionPeriod,
    mrpt::system::CTimeLogger& timelogger)
{
  mrpt::system::CTimeLoggerEntry tle(timelogger, "generate_vel_cmd");
  double cmdvel_speed_scale = 1.0;
  try
  {
    if (in_movement.speed == 0)
    {
      new_vel_cmd = in_movement.PTG->getSupportedKinematicVelocityCommand();
      new_vel_cmd->setToStop();
    }
    else
    {
      const bool is_slowdown = in_movement.props.count("is_slowdown") != 0
                                   ? in_movement.props.at("is_slowdown") != 0
                                   : false;

      new_vel_cmd = in_movement.PTG->directionToMotionCommand(
          in_movement.PTG->alpha2index(in_movement.direction));

      if (!is_slowdown)
      {
        new_vel_cmd->cmdVel_scale(in_movement.speed);
        cmdvel_speed_scale *= in_movement.speed;

        if (!m_last_vel_cmd)
          m_last_vel_cmd = in_movement.PTG->getSupportedKinematicVelocityCommand();

        const double beta = meanExecutionPeriod / (meanExecutionPeriod + speedfilter_tau);
        cmdvel_speed_scale *=
            new_vel_cmd->cmdVel_limits(*m_last_vel_cmd, beta, robot_absolute_speed_limits);
      }
    }

    m_last_vel_cmd = new_vel_cmd;
  }
  catch (const std::exception& e)
  {
    std::cerr << "[VelocityFilter::generateVelCmd] Exception: " << e.what() << std::endl;
  }
  return cmdvel_speed_scale;
}
