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
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/viz/CSetOfObjects.h>

using namespace mrpt::nav;

TEST(TWaypoint, getAsText_reports_unset_fields)
{
  TWaypoint wp;  // nothing set
  const std::string s = wp.getAsText();
  EXPECT_NE(s.find("Coordinates not set"), std::string::npos);
  EXPECT_NE(s.find("allowed_distance not set"), std::string::npos);
  EXPECT_NE(s.find("heading: any"), std::string::npos);
}

TEST(TWaypoint, getAsText_reports_set_fields)
{
  TWaypoint wp(1.0, 2.0, 0.5, false, mrpt::DEG2RAD(90.0), 0.25);
  const std::string s = wp.getAsText();
  EXPECT_NE(s.find("target="), std::string::npos);
  EXPECT_NE(s.find("phi="), std::string::npos);
  EXPECT_NE(s.find("allowed_dist="), std::string::npos);
  EXPECT_NE(s.find("allow_skip: NO"), std::string::npos);
  EXPECT_NE(s.find("speed_ratio"), std::string::npos);
}

TEST(TWaypoint, invalid_heading_sentinel_is_treated_as_unset)
{
  // Backwards-compatibility: the old "no heading" sentinel value:
  TWaypoint wp(1.0, 2.0, 0.5, true, static_cast<double>(TWaypoint::INVALID_NUM));
  EXPECT_FALSE(wp.target_heading.has_value());
}

TEST(TWaypoint, is_invalid_if_only_some_fields_are_set)
{
  TWaypoint wp;
  wp.target = mrpt::math::TPoint2D(1, 2);
  EXPECT_FALSE(wp.isValid());  // allowed_distance still unset

  wp.allowed_distance = 0.5;
  EXPECT_TRUE(wp.isValid());

  wp.target.y = TWaypoint::INVALID_NUM;
  EXPECT_FALSE(wp.isValid());
}

TEST(TWaypointSequence, config_file_roundtrip)
{
  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 2.0, 0.5, true, mrpt::DEG2RAD(45.0), 0.75);
  seq.waypoints.emplace_back(3.0, 4.0, 0.6, false);
  seq.waypoints.back().target_frame_id = "odom";

  mrpt::config::CConfigFileMemory cfg;
  seq.save(cfg, "WPS");

  TWaypointSequence seq2;
  seq2.load(cfg, "WPS");

  ASSERT_EQ(seq2.waypoints.size(), 2U);

  EXPECT_NEAR(seq2.waypoints[0].target.x, 1.0, 1e-9);
  EXPECT_NEAR(seq2.waypoints[0].target.y, 2.0, 1e-9);
  EXPECT_NEAR(seq2.waypoints[0].allowed_distance, 0.5, 1e-9);
  EXPECT_TRUE(seq2.waypoints[0].allow_skip);
  ASSERT_TRUE(seq2.waypoints[0].target_heading.has_value());
  // the config file stores a rounded decimal representation:
  EXPECT_NEAR(*seq2.waypoints[0].target_heading, mrpt::DEG2RAD(45.0), 1e-6);
  EXPECT_NEAR(seq2.waypoints[0].speed_ratio, 0.75, 1e-9);

  EXPECT_FALSE(seq2.waypoints[1].allow_skip);
  EXPECT_FALSE(seq2.waypoints[1].target_heading.has_value());
  EXPECT_EQ(seq2.waypoints[1].target_frame_id, "odom");
  EXPECT_NEAR(seq2.waypoints[1].speed_ratio, 1.0, 1e-9);
}

TEST(TWaypointSequence, load_of_an_empty_section_throws)
{
  mrpt::config::CConfigFileMemory cfg;
  TWaypointSequence seq;
  EXPECT_ANY_THROW(seq.load(cfg, "DoesNotExist"));
}

TEST(TWaypointSequence, load_clears_previous_contents)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("WPS", "waypoint_count", 0);

  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 2.0, 0.5);
  seq.load(cfg, "WPS");
  EXPECT_TRUE(seq.waypoints.empty());
}

TEST(TWaypointSequence, opengl_visualization_has_one_object_per_waypoint)
{
  TWaypointSequence seq;
  seq.waypoints.emplace_back(1.0, 2.0, 0.5);                              // no heading
  seq.waypoints.emplace_back(3.0, 4.0, 0.5, false, mrpt::DEG2RAD(90.0));  // + heading arrow

  auto gl = mrpt::viz::CSetOfObjects::Create();
  seq.getAsOpenglVisualization(*gl);
  // 2 disks + 1 heading arrow:
  EXPECT_EQ(gl->size(), 3U);

  // Rendering again must clear the previous contents, not accumulate:
  seq.getAsOpenglVisualization(*gl);
  EXPECT_EQ(gl->size(), 3U);

  // Labels can be turned off:
  TWaypointsRenderingParams p;
  p.show_labels = false;
  seq.getAsOpenglVisualization(*gl, p);
  EXPECT_EQ(gl->size(), 3U);
}

TEST(TWaypointStatus, getAsText_includes_reached_flag)
{
  TWaypointStatus ws;
  ws = TWaypoint(1.0, 2.0, 0.5);
  EXPECT_NE(ws.getAsText().find("reached=NO"), std::string::npos);
  ws.reached = true;
  EXPECT_NE(ws.getAsText().find("reached=YES"), std::string::npos);
}

TEST(TWaypointStatusSequence, getAsText_summarizes_progress)
{
  TWaypointStatusSequence seq;
  seq.waypoints.resize(2);
  seq.waypoints[0] = TWaypoint(1.0, 2.0, 0.5);
  seq.waypoints[1] = TWaypoint(3.0, 4.0, 0.5);
  seq.waypoints[0].reached = true;
  seq.waypoint_index_current_goal = 1;

  const std::string s = seq.getAsText();
  EXPECT_NE(s.find("Status for 2 waypoints"), std::string::npos);
  EXPECT_NE(s.find("final_goal_reached:NO"), std::string::npos);
  EXPECT_NE(s.find("waypoint_index_current_goal=1"), std::string::npos);

  seq.final_goal_reached = true;
  EXPECT_NE(seq.getAsText().find("final_goal_reached:YES"), std::string::npos);
}

TEST(TWaypointStatusSequence, opengl_visualization_colors_by_status)
{
  TWaypointStatusSequence seq;
  seq.waypoints.resize(3);
  seq.waypoints[0] = TWaypoint(1.0, 0.0, 0.5);
  seq.waypoints[1] = TWaypoint(2.0, 0.0, 0.5, false);
  seq.waypoints[2] = TWaypoint(3.0, 0.0, 0.5, true, mrpt::DEG2RAD(90.0));
  seq.waypoints[0].reached = true;
  seq.waypoint_index_current_goal = 1;

  auto gl = mrpt::viz::CSetOfObjects::Create();
  seq.getAsOpenglVisualization(*gl);
  // 3 disks + 1 heading arrow:
  EXPECT_EQ(gl->size(), 4U);

  TWaypointsRenderingParams p;
  p.show_labels = false;
  seq.getAsOpenglVisualization(*gl, p);
  EXPECT_EQ(gl->size(), 4U);
}

TEST(TWaypointsRenderingParams, defaults_are_sane)
{
  TWaypointsRenderingParams p;
  EXPECT_GT(p.outer_radius, p.inner_radius);
  EXPECT_GT(p.heading_arrow_len, .0);
  EXPECT_TRUE(p.show_labels);
}
