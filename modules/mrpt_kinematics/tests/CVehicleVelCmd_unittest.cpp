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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::kinematics;

// ---------------------------------------------------------------------------
//  CVehicleVelCmd_DiffDriven
// ---------------------------------------------------------------------------
TEST(CVehicleVelCmd_DiffDriven, component_accessors)
{
  CVehicleVelCmd_DiffDriven c;
  EXPECT_EQ(c.getVelCmdLength(), 2U);
  EXPECT_EQ(c.getVelCmdDescription(0), "lin_vel");
  EXPECT_EQ(c.getVelCmdDescription(1), "ang_vel");
  EXPECT_ANY_THROW(c.getVelCmdDescription(2));

  c.setVelCmdElement(0, 1.5);
  c.setVelCmdElement(1, -0.25);
  EXPECT_NEAR(c.getVelCmdElement(0), 1.5, 1e-12);
  EXPECT_NEAR(c.getVelCmdElement(1), -0.25, 1e-12);
  EXPECT_NEAR(c.lin_vel, 1.5, 1e-12);
  EXPECT_NEAR(c.ang_vel, -0.25, 1e-12);

  EXPECT_ANY_THROW(c.getVelCmdElement(2));
  EXPECT_ANY_THROW(c.setVelCmdElement(2, 0.0));
}

TEST(CVehicleVelCmd_DiffDriven, stop_detection)
{
  CVehicleVelCmd_DiffDriven c;
  EXPECT_TRUE(c.isStopCmd());  // default-constructed is (0,0)

  c.lin_vel = 0.5;
  EXPECT_FALSE(c.isStopCmd());
  c.lin_vel = .0;
  c.ang_vel = 0.5;
  EXPECT_FALSE(c.isStopCmd());

  c.setToStop();
  EXPECT_TRUE(c.isStopCmd());
  EXPECT_EQ(c.lin_vel, .0);
  EXPECT_EQ(c.ang_vel, .0);
}

TEST(CVehicleVelCmd_DiffDriven, asString_mentions_components)
{
  CVehicleVelCmd_DiffDriven c;
  c.lin_vel = 0.7;
  c.ang_vel = 0.3;
  const std::string s = c.asString();
  EXPECT_NE(s.find("lin_vel"), std::string::npos);
  EXPECT_NE(s.find("ang_vel"), std::string::npos);
}

TEST(CVehicleVelCmd_DiffDriven, cmdVel_scale_scales_both_components)
{
  CVehicleVelCmd_DiffDriven c;
  c.lin_vel = 1.0;
  c.ang_vel = -2.0;
  c.cmdVel_scale(0.5);
  EXPECT_NEAR(c.lin_vel, 0.5, 1e-12);
  EXPECT_NEAR(c.ang_vel, -1.0, 1e-12);
}

TEST(CVehicleVelCmd_DiffDriven, cmdVel_limits_no_clipping_needed)
{
  CVehicleVelCmd_DiffDriven prev;  // (0,0)
  CVehicleVelCmd_DiffDriven c;
  c.lin_vel = 0.5;
  c.ang_vel = 0.1;

  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 1.0;
  p.robotMax_W_radps = 1.0;

  // beta=1 => no blending with the previous command:
  const double scale = c.cmdVel_limits(prev, 1.0, p);
  EXPECT_NEAR(scale, 1.0, 1e-12);
  EXPECT_NEAR(c.lin_vel, 0.5, 1e-12);
  EXPECT_NEAR(c.ang_vel, 0.1, 1e-12);
}

TEST(CVehicleVelCmd_DiffDriven, cmdVel_limits_clips_linear_speed)
{
  CVehicleVelCmd_DiffDriven prev;
  CVehicleVelCmd_DiffDriven c;
  c.lin_vel = 4.0;
  c.ang_vel = 0.4;

  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 1.0;
  p.robotMax_W_radps = 10.0;

  const double scale = c.cmdVel_limits(prev, 1.0, p);
  EXPECT_LE(std::abs(c.lin_vel), p.robotMax_V_mps + 1e-9);
  EXPECT_GT(scale, 0.0);
  EXPECT_LT(scale, 1.0);
  // The path curvature (w/v ratio) must be preserved while clipping:
  EXPECT_NEAR(c.ang_vel / c.lin_vel, 0.4 / 4.0, 1e-9);
}

TEST(CVehicleVelCmd_DiffDriven, cmdVel_limits_clips_angular_speed)
{
  CVehicleVelCmd_DiffDriven prev;
  CVehicleVelCmd_DiffDriven c;
  c.lin_vel = 0.5;
  c.ang_vel = 5.0;

  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 10.0;
  p.robotMax_W_radps = 1.0;

  const double scale = c.cmdVel_limits(prev, 1.0, p);
  EXPECT_LE(std::abs(c.ang_vel), p.robotMax_W_radps + 1e-9);
  EXPECT_LT(scale, 1.0);
}

TEST(CVehicleVelCmd_DiffDriven, cmdVel_limits_blends_pure_rotation)
{
  CVehicleVelCmd_DiffDriven prev;
  prev.lin_vel = .0;
  prev.ang_vel = 1.0;

  CVehicleVelCmd_DiffDriven c;
  c.lin_vel = .0;  // below the 0.01 threshold => "nearly pure rotation"
  c.ang_vel = .0;

  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 1.0;
  p.robotMax_W_radps = 2.0;

  c.cmdVel_limits(prev, 0.25, p);
  // 0.25*0 + 0.75*1.0
  EXPECT_NEAR(c.ang_vel, 0.75, 1e-9);
}

TEST(CVehicleVelCmd_DiffDriven, cmdVel_limits_requires_valid_limits)
{
  CVehicleVelCmd_DiffDriven prev, c;
  c.lin_vel = 0.5;
  CVehicleVelCmd::TVelCmdParams p;  // both limits left at -1 (unset)
  EXPECT_ANY_THROW(c.cmdVel_limits(prev, 1.0, p));
}

TEST(CVehicleVelCmd_DiffDriven, cmdVel_limits_rejects_foreign_prev_cmd)
{
  CVehicleVelCmd_Holo prev;
  CVehicleVelCmd_DiffDriven c;
  c.lin_vel = 0.5;

  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 1.0;
  p.robotMax_W_radps = 1.0;
  EXPECT_ANY_THROW(c.cmdVel_limits(prev, 1.0, p));
}

TEST(CVehicleVelCmd_DiffDriven, serialization_roundtrip)
{
  CVehicleVelCmd_DiffDriven c;
  c.lin_vel = 0.33;
  c.ang_vel = -0.77;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << c;
  buf.Seek(0);

  CVehicleVelCmd_DiffDriven c2;
  arch >> c2;
  EXPECT_NEAR(c2.lin_vel, c.lin_vel, 1e-12);
  EXPECT_NEAR(c2.ang_vel, c.ang_vel, 1e-12);
}

// ---------------------------------------------------------------------------
//  CVehicleVelCmd_Holo
// ---------------------------------------------------------------------------
TEST(CVehicleVelCmd_Holo, component_accessors)
{
  CVehicleVelCmd_Holo c(1.0, 0.5, 0.8, 0.2);
  EXPECT_EQ(c.getVelCmdLength(), 4U);
  EXPECT_EQ(c.getVelCmdDescription(0), "vel");
  EXPECT_EQ(c.getVelCmdDescription(1), "dir_local");
  EXPECT_EQ(c.getVelCmdDescription(2), "ramp_time");
  EXPECT_EQ(c.getVelCmdDescription(3), "rot_speed");
  EXPECT_ANY_THROW(c.getVelCmdDescription(4));

  EXPECT_NEAR(c.getVelCmdElement(0), 1.0, 1e-12);
  EXPECT_NEAR(c.getVelCmdElement(1), 0.5, 1e-12);
  EXPECT_NEAR(c.getVelCmdElement(2), 0.8, 1e-12);
  EXPECT_NEAR(c.getVelCmdElement(3), 0.2, 1e-12);
  EXPECT_ANY_THROW(c.getVelCmdElement(4));

  for (int i = 0; i < 4; i++)
  {
    c.setVelCmdElement(i, 0.1 * (i + 1));
  }
  EXPECT_NEAR(c.vel, 0.1, 1e-12);
  EXPECT_NEAR(c.dir_local, 0.2, 1e-12);
  EXPECT_NEAR(c.ramp_time, 0.3, 1e-12);
  EXPECT_NEAR(c.rot_speed, 0.4, 1e-12);
  EXPECT_ANY_THROW(c.setVelCmdElement(4, 0.0));
}

TEST(CVehicleVelCmd_Holo, stop_detection)
{
  CVehicleVelCmd_Holo c(1.0, 0.5, 0.8, 0.2);
  EXPECT_FALSE(c.isStopCmd());
  c.setToStop();
  EXPECT_TRUE(c.isStopCmd());
  EXPECT_EQ(c.vel, .0);
  EXPECT_EQ(c.dir_local, .0);
  EXPECT_EQ(c.ramp_time, .0);
  EXPECT_EQ(c.rot_speed, .0);
}

TEST(CVehicleVelCmd_Holo, cmdVel_scale_only_affects_linear_speed)
{
  CVehicleVelCmd_Holo c(1.0, 0.5, 0.8, 0.2);
  c.cmdVel_scale(0.5);
  EXPECT_NEAR(c.vel, 0.5, 1e-12);
  // A holonomic path is invariant to the rotation speed:
  EXPECT_NEAR(c.rot_speed, 0.2, 1e-12);
  EXPECT_NEAR(c.ramp_time, 0.8, 1e-12);
}

TEST(CVehicleVelCmd_Holo, cmdVel_limits_scales_down_over_max_speed)
{
  CVehicleVelCmd_Holo prev;
  CVehicleVelCmd_Holo c(2.0, 0.0, 0.8, 1.0);

  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 1.0;

  const double f = c.cmdVel_limits(prev, 1.0, p);
  EXPECT_NEAR(f, 0.5, 1e-12);
  EXPECT_NEAR(c.vel, 1.0, 1e-12);
  EXPECT_NEAR(c.rot_speed, 0.5, 1e-12);
  EXPECT_NEAR(c.ramp_time, 0.8, 1e-12);  // left unchanged
}

TEST(CVehicleVelCmd_Holo, cmdVel_limits_below_max_is_a_noop)
{
  CVehicleVelCmd_Holo prev;
  CVehicleVelCmd_Holo c(0.4, 0.0, 0.8, 1.0);

  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 1.0;

  EXPECT_NEAR(c.cmdVel_limits(prev, 1.0, p), 1.0, 1e-12);
  EXPECT_NEAR(c.vel, 0.4, 1e-12);
  EXPECT_NEAR(c.rot_speed, 1.0, 1e-12);
}

TEST(CVehicleVelCmd_Holo, cmdVel_limits_requires_valid_max_speed)
{
  CVehicleVelCmd_Holo prev;
  CVehicleVelCmd_Holo c(0.4, 0.0, 0.8, 1.0);
  CVehicleVelCmd::TVelCmdParams p;  // robotMax_V_mps left at -1
  EXPECT_ANY_THROW(c.cmdVel_limits(prev, 1.0, p));
}

TEST(CVehicleVelCmd_Holo, serialization_roundtrip)
{
  CVehicleVelCmd_Holo c(1.25, -0.5, 0.9, 0.15);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << c;
  buf.Seek(0);

  CVehicleVelCmd_Holo c2;
  arch >> c2;
  EXPECT_NEAR(c2.vel, c.vel, 1e-12);
  EXPECT_NEAR(c2.dir_local, c.dir_local, 1e-12);
  EXPECT_NEAR(c2.ramp_time, c.ramp_time, 1e-12);
  EXPECT_NEAR(c2.rot_speed, c.rot_speed, 1e-12);
}

// ---------------------------------------------------------------------------
//  Base class: copy semantics and TVelCmdParams
// ---------------------------------------------------------------------------
TEST(CVehicleVelCmd, copy_and_assign)
{
  CVehicleVelCmd_DiffDriven a;
  a.lin_vel = 1.0;
  a.ang_vel = 2.0;

  CVehicleVelCmd_DiffDriven b(a);
  EXPECT_NEAR(b.lin_vel, 1.0, 1e-12);

  CVehicleVelCmd_DiffDriven c;
  c = a;
  EXPECT_NEAR(c.ang_vel, 2.0, 1e-12);

  // Assignment through the abstract base class reference:
  CVehicleVelCmd& base_c = c;
  CVehicleVelCmd& base_a = a;
  base_c = base_a;
  EXPECT_NEAR(c.lin_vel, 1.0, 1e-12);
}

TEST(TVelCmdParams, config_file_roundtrip)
{
  CVehicleVelCmd::TVelCmdParams p;
  p.robotMax_V_mps = 1.5;
  p.robotMax_W_radps = 0.75;
  p.robotMinCurvRadius = 0.3;

  mrpt::config::CConfigFileMemory cfg;
  p.saveToConfigFile(cfg, "S");

  CVehicleVelCmd::TVelCmdParams q;
  q.loadConfigFile(cfg, "S");

  EXPECT_NEAR(q.robotMax_V_mps, 1.5, 1e-9);
  // deg <-> rad conversion through the text config file loses some precision:
  EXPECT_NEAR(q.robotMax_W_radps, 0.75, 1e-6);
  EXPECT_NEAR(q.robotMinCurvRadius, 0.3, 1e-9);
}

TEST(TVelCmdParams, defaults_are_unset)
{
  CVehicleVelCmd::TVelCmdParams p;
  EXPECT_LT(p.robotMax_V_mps, .0);
  EXPECT_LT(p.robotMax_W_radps, .0);
  EXPECT_LT(p.robotMinCurvRadius, .0);
}
