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

/** Unit tests for the odometry-only node registration decider
 *  CFixedIntervalsNRD, together with the CNodeRegistrationDecider /
 *  CRegistrationDeciderOrOptimizer interfaces it builds upon. Both supported
 *  rawlog formats (observation-only and action-observation) are simulated
 *  in-memory, so no dataset file nor GUI is involved.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/NRD/CFixedIntervalsNRD.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationOdometry.h>

#include <sstream>
#include <string>

using graph_t = mrpt::graphs::CNetworkOfPoses2DInf;
using decider_t = mrpt::graphslam::deciders::CFixedIntervalsNRD<graph_t>;

namespace
{
/** Exposes the protected registration-condition helpers */
class TestNRD : public decider_t
{
 public:
  using decider_t::checkRegistrationCondition;
};

mrpt::obs::CObservation::Ptr makeOdometryObs(double x, double y, double phi)
{
  auto o = mrpt::obs::CObservationOdometry::Create();
  o->odometry = mrpt::poses::CPose2D(x, y, phi);
  o->timestamp = mrpt::Clock::now();
  return o;
}

mrpt::obs::CActionCollection::Ptr makeOdometryAction(const mrpt::poses::CPose2D& increment)
{
  mrpt::obs::CActionRobotMovement2D act;
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions opts;
  opts.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
  opts.gaussianModel.minStdXY = 0.02;
  opts.gaussianModel.minStdPHI = mrpt::DEG2RAD(0.5);
  act.computeFromOdometry(increment, opts);

  auto acts = mrpt::obs::CActionCollection::Create();
  acts->insert(act);
  return acts;
}
}  // namespace

TEST(CFixedIntervalsNRD, params_default_to_the_documented_values)
{
  // The defaults must hold on a freshly constructed decider, i.e. without
  // any call to loadParams():
  const decider_t d;
  EXPECT_DOUBLE_EQ(d.params.registration_max_distance, 0.5);
  EXPECT_DOUBLE_EQ(d.params.registration_max_angle, mrpt::DEG2RAD(60.0));
}

TEST(CFixedIntervalsNRD, params_load_from_config_and_convert_the_angle_to_radians)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("NodeRegistrationDeciderParameters", "registration_max_distance", 1.25);
  cfg.write("NodeRegistrationDeciderParameters", "registration_max_angle", 30.0);

  decider_t d;
  d.params.loadFromConfigFile(cfg, "NodeRegistrationDeciderParameters");

  EXPECT_DOUBLE_EQ(d.params.registration_max_distance, 1.25);
  EXPECT_DOUBLE_EQ(d.params.registration_max_angle, mrpt::DEG2RAD(30.0));

  // The textual representation must report both, the angle back in degrees:
  const std::string s = d.params.getAsString();
  EXPECT_NE(s.find("Max distance for registration = 1.25 m"), std::string::npos) << s;
  EXPECT_NE(s.find("Max angle for registration    = 30.00 deg"), std::string::npos) << s;

  std::stringstream ss;
  d.params.dumpToTextStream(ss);
  EXPECT_EQ(ss.str(), s);
}

TEST(CFixedIntervalsNRD, registration_condition_2D)
{
  TestNRD d;
  d.params.registration_max_distance = 0.5;
  d.params.registration_max_angle = mrpt::DEG2RAD(60.0);

  const mrpt::poses::CPose2D origin(0, 0, 0);

  EXPECT_FALSE(d.checkRegistrationCondition(origin, mrpt::poses::CPose2D(0.4, 0, 0)));
  EXPECT_TRUE(d.checkRegistrationCondition(origin, mrpt::poses::CPose2D(0.6, 0, 0)));

  // Angle alone is enough, and it must be compared through wrapToPi():
  EXPECT_FALSE(
      d.checkRegistrationCondition(origin, mrpt::poses::CPose2D(0, 0, mrpt::DEG2RAD(50.0))));
  EXPECT_TRUE(
      d.checkRegistrationCondition(origin, mrpt::poses::CPose2D(0, 0, mrpt::DEG2RAD(70.0))));
  // +190 deg wraps to -170 deg, i.e. still beyond the 60 deg threshold:
  EXPECT_TRUE(
      d.checkRegistrationCondition(origin, mrpt::poses::CPose2D(0, 0, mrpt::DEG2RAD(190.0))));
  // ...while +350 deg wraps to -10 deg, which is below it:
  EXPECT_FALSE(
      d.checkRegistrationCondition(origin, mrpt::poses::CPose2D(0, 0, mrpt::DEG2RAD(350.0))));
}

TEST(CFixedIntervalsNRD, registration_condition_3D)
{
  TestNRD d;
  d.params.registration_max_distance = 0.5;
  d.params.registration_max_angle = mrpt::DEG2RAD(60.0);

  const mrpt::poses::CPose3D origin;

  EXPECT_FALSE(d.checkRegistrationCondition(origin, mrpt::poses::CPose3D(0.4, 0, 0, 0, 0, 0)));
  EXPECT_TRUE(d.checkRegistrationCondition(origin, mrpt::poses::CPose3D(0, 0, 0.6, 0, 0, 0)));

  // Each of yaw/pitch/roll on its own must trigger it:
  const double a = mrpt::DEG2RAD(70.0);
  EXPECT_TRUE(d.checkRegistrationCondition(origin, mrpt::poses::CPose3D(0, 0, 0, a, 0, 0)));
  EXPECT_TRUE(d.checkRegistrationCondition(origin, mrpt::poses::CPose3D(0, 0, 0, 0, a, 0)));
  EXPECT_TRUE(d.checkRegistrationCondition(origin, mrpt::poses::CPose3D(0, 0, 0, 0, 0, a)));
  EXPECT_FALSE(d.checkRegistrationCondition(
      origin, mrpt::poses::CPose3D(0, 0, 0, mrpt::DEG2RAD(10.0), 0, 0)));
}

TEST(CFixedIntervalsNRD, observation_only_format_registers_nodes_by_odometry)
{
  graph_t graph;
  decider_t d;
  d.setMinLoggingLevel(mrpt::system::LVL_ERROR);
  d.setGraphPtr(&graph);
  d.params.registration_max_distance = 1.0;
  d.params.registration_max_angle = mrpt::DEG2RAD(60.0);

  // Below the threshold: nothing is registered yet.
  EXPECT_FALSE(d.updateState(nullptr, nullptr, makeOdometryObs(0.5, 0, 0)));
  EXPECT_EQ(graph.nodeCount(), 0U);

  // Beyond it: both the root node and the new one appear.
  EXPECT_TRUE(d.updateState(nullptr, nullptr, makeOdometryObs(1.5, 0, 0)));
  EXPECT_EQ(graph.nodeCount(), 2U);
  EXPECT_EQ(graph.edgeCount(), 1U);
  // The root node is the origin of the graph frame:
  EXPECT_NEAR(graph.nodes.at(0).x(), 0.0, 1e-6);
  EXPECT_NEAR(graph.nodes.at(1).x(), 1.5, 1e-6);

  // The next node is measured from the last registered one, so another
  // 0.5 m is again not enough:
  EXPECT_FALSE(d.updateState(nullptr, nullptr, makeOdometryObs(2.0, 0, 0)));
  EXPECT_EQ(graph.nodeCount(), 2U);

  EXPECT_TRUE(d.updateState(nullptr, nullptr, makeOdometryObs(3.0, 0, 0)));
  EXPECT_EQ(graph.nodeCount(), 3U);
  EXPECT_EQ(graph.edgeCount(), 2U);

  EXPECT_NEAR(graph.nodes.at(2).x(), 3.0, 1e-6);
}

TEST(CFixedIntervalsNRD, observation_only_format_ignores_other_observation_types)
{
  graph_t graph;
  decider_t d;
  d.setMinLoggingLevel(mrpt::system::LVL_ERROR);
  d.setGraphPtr(&graph);

  auto obs = mrpt::obs::CObservationOdometry::Create();
  obs->odometry = mrpt::poses::CPose2D(10, 10, 0);
  obs->timestamp = mrpt::Clock::now();

  // A non-odometry observation must leave the estimate untouched:
  auto other = mrpt::obs::CObservationComment::Create();
  other->timestamp = mrpt::Clock::now();
  EXPECT_FALSE(d.updateState(nullptr, nullptr, other));
  EXPECT_EQ(graph.nodeCount(), 0U);
}

TEST(CFixedIntervalsNRD, action_observation_format_accumulates_increments)
{
  graph_t graph;
  decider_t d;
  d.setMinLoggingLevel(mrpt::system::LVL_ERROR);
  d.setGraphPtr(&graph);
  d.params.registration_max_distance = 1.0;
  d.params.registration_max_angle = mrpt::DEG2RAD(60.0);

  auto sf = mrpt::obs::CSensoryFrame::Create();

  // Two 0.6 m increments: the first is below the threshold, the second one
  // accumulates past it.
  EXPECT_FALSE(d.updateState(makeOdometryAction(mrpt::poses::CPose2D(0.6, 0, 0)), sf, nullptr));
  EXPECT_EQ(graph.nodeCount(), 0U);

  EXPECT_TRUE(d.updateState(makeOdometryAction(mrpt::poses::CPose2D(0.6, 0, 0)), sf, nullptr));
  EXPECT_EQ(graph.nodeCount(), 2U);
  EXPECT_NEAR(graph.nodes.at(0).x(), 0.0, 1e-6);
  EXPECT_NEAR(graph.nodes.at(1).x(), 1.2, 1e-6);

  // After a registration the accumulated PDF is reset:
  EXPECT_FALSE(d.updateState(makeOdometryAction(mrpt::poses::CPose2D(0.3, 0, 0)), sf, nullptr));
  EXPECT_EQ(graph.nodeCount(), 2U);
}

TEST(CFixedIntervalsNRD, action_without_a_movement_estimation_is_a_no_op)
{
  graph_t graph;
  decider_t d;
  d.setMinLoggingLevel(mrpt::system::LVL_ERROR);
  d.setGraphPtr(&graph);

  auto empty_action = mrpt::obs::CActionCollection::Create();
  EXPECT_FALSE(d.updateState(empty_action, mrpt::obs::CSensoryFrame::Create(), nullptr));
  EXPECT_EQ(graph.nodeCount(), 0U);
}

TEST(CFixedIntervalsNRD, descriptive_report_mentions_the_strategy_and_the_params)
{
  decider_t d;
  d.setMinLoggingLevel(mrpt::system::LVL_ERROR);

  std::string report = "stale contents";
  d.getDescriptiveReport(&report);

  EXPECT_NE(report.find("Fixed Odometry-based Intervals"), std::string::npos);
  EXPECT_NE(report.find("Node Registration Decider Strategy"), std::string::npos);
  EXPECT_NE(report.find("Max distance for registration"), std::string::npos);
  EXPECT_EQ(report.find("stale contents"), std::string::npos);
}

TEST(CFixedIntervalsNRD, class_name_and_multi_robot_flag)
{
  decider_t d;
  EXPECT_EQ(d.getClassName(), "CFixedIntervalsNRD");
  EXPECT_FALSE(d.isMultiRobotSlamClass());
}
