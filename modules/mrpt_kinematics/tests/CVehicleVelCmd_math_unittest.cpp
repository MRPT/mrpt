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

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>

#include <cmath>

using namespace mrpt::kinematics;

// Holo cmdVel_limits() must honor robotMax_W_radps
TEST(CVehicleVelCmd_Holo, cmdVel_limits_honors_max_rot_speed)
{
  CVehicleVelCmd_Holo prev;
  CVehicleVelCmd_Holo c(0.5, 0.3, 0.8, -5.0);

  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 1.0;
  p.robotMax_W_radps = 1.0;

  c.cmdVel_limits(prev, 1.0, p);
  EXPECT_LE(std::abs(c.rot_speed), p.robotMax_W_radps + 1e-12);
  EXPECT_LE(std::abs(c.vel), p.robotMax_V_mps + 1e-12);
  EXPECT_NEAR(c.dir_local, 0.3, 1e-12);
}

TEST(CVehicleVelCmd_DiffDriven, cmdVel_limits_blend_keeps_curvature)
{
  CVehicleVelCmd_DiffDriven prev;
  prev.lin_vel = 0.2;
  prev.ang_vel = .0;

  CVehicleVelCmd_DiffDriven c;
  c.lin_vel = 1.0;
  c.ang_vel = 0.5;

  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 2.0;
  p.robotMax_W_radps = 2.0;

  const double scale = c.cmdVel_limits(prev, 0.5, p);
  EXPECT_NEAR(scale, 1.0, 1e-12);
  // v is blended, w follows the new curvature w/v=0.5:
  EXPECT_NEAR(c.lin_vel, 0.6, 1e-12);
  EXPECT_NEAR(c.ang_vel, 0.3, 1e-12);
}

TEST(CVehicleVelCmd_DiffDriven, cmdVel_limits_both_limits_active)
{
  CVehicleVelCmd_DiffDriven prev;
  CVehicleVelCmd_DiffDriven c;
  c.lin_vel = -3.0;
  c.ang_vel = 6.0;

  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 1.0;
  p.robotMax_W_radps = 1.0;

  const double scale = c.cmdVel_limits(prev, 1.0, p);
  EXPECT_LE(std::abs(c.lin_vel), 1.0 + 1e-12);
  EXPECT_LE(std::abs(c.ang_vel), 1.0 + 1e-12);
  // Curvature and sign of both components are preserved:
  EXPECT_NEAR(c.ang_vel / c.lin_vel, -2.0, 1e-12);
  EXPECT_LT(c.lin_vel, .0);
  // The returned factor is the overall shrink of the command:
  EXPECT_NEAR(scale, c.ang_vel / 6.0, 1e-12);
  EXPECT_NEAR(c.ang_vel, 1.0, 1e-12);
}

TEST(TVelCmdParams, max_angular_speed_is_read_in_degrees)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("S", "robotMax_V_mps", 1.25);
  cfg.write("S", "robotMax_W_degps", 90.0);

  CVehicleVelCmd::TVelCmdParams p;
  p.loadConfigFile(cfg, "S");
  EXPECT_NEAR(p.robotMax_V_mps, 1.25, 1e-12);
  EXPECT_NEAR(p.robotMax_W_radps, M_PI * 0.5, 1e-12);
  EXPECT_LT(p.robotMinCurvRadius, .0);  // optional, keeps its default

  mrpt::config::CConfigFileMemory missing;
  missing.write("S", "robotMax_V_mps", 1.0);
  CVehicleVelCmd::TVelCmdParams q;
  EXPECT_ANY_THROW(q.loadConfigFile(missing, "S"));
}
