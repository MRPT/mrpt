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

/** Unit tests for the CIncrementalNodeRegistrationDecider interface. The class
 *  has no concrete implementation in the library, so a minimal one is defined
 *  here; instantiating it is also what keeps the template compiling.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/interfaces/CIncrementalNodeRegistrationDecider.h>

#include <sstream>
#include <string>

using graph_t = mrpt::graphs::CNetworkOfPoses2DInf;

namespace
{
/** Minimal concrete decider: each updateState() call adds a fixed increment to
 *  the pose accumulated since the last registered node.
 */
class StepNRD : public mrpt::graphslam::deciders::CIncrementalNodeRegistrationDecider<graph_t>
{
 public:
  using base_t = mrpt::graphslam::deciders::CIncrementalNodeRegistrationDecider<graph_t>;
  using base_t::params;

  explicit StepNRD(const mrpt::poses::CPose2D& step) : m_step(step)
  {
    this->initializeLoggers("StepNRD");
    this->setMinLoggingLevel(mrpt::system::LVL_ERROR);
  }

  bool updateState(
      [[maybe_unused]] mrpt::obs::CActionCollection::Ptr action,
      [[maybe_unused]] mrpt::obs::CSensoryFrame::Ptr observations,
      [[maybe_unused]] mrpt::obs::CObservation::Ptr observation) override
  {
    this->m_since_prev_node_PDF.mean = this->m_since_prev_node_PDF.mean + m_step;
    return this->checkRegistrationCondition();
  }

 private:
  mrpt::poses::CPose2D m_step;
};
}  // namespace

TEST(CIncrementalNodeRegistrationDecider, params_default_to_the_documented_values)
{
  const StepNRD d(mrpt::poses::CPose2D(0, 0, 0));
  EXPECT_DOUBLE_EQ(d.params.registration_max_distance, 0.5);
  EXPECT_DOUBLE_EQ(d.params.registration_max_angle, mrpt::DEG2RAD(15.0));
}

TEST(CIncrementalNodeRegistrationDecider, params_load_from_config_file)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("NodeRegistrationDeciderParameters", "registration_max_distance", 2.0);
  cfg.write("NodeRegistrationDeciderParameters", "registration_max_angle", 45.0);

  StepNRD d(mrpt::poses::CPose2D(0, 0, 0));
  d.params.loadFromConfigFile(cfg, "NodeRegistrationDeciderParameters");

  EXPECT_DOUBLE_EQ(d.params.registration_max_distance, 2.0);
  EXPECT_DOUBLE_EQ(d.params.registration_max_angle, mrpt::DEG2RAD(45.0));

  std::stringstream ss;
  d.params.dumpToTextStream(ss);
  EXPECT_EQ(ss.str(), d.params.getAsString());
  EXPECT_FALSE(ss.str().empty());
}

TEST(CIncrementalNodeRegistrationDecider, registers_a_node_once_the_threshold_is_crossed)
{
  graph_t graph;
  StepNRD d(mrpt::poses::CPose2D(0.4, 0, 0));
  d.setGraphPtr(&graph);
  d.params.registration_max_distance = 1.0;
  d.params.registration_max_angle = mrpt::DEG2RAD(60.0);

  EXPECT_FALSE(d.updateState(nullptr, nullptr, nullptr));  // 0.4 m
  EXPECT_FALSE(d.updateState(nullptr, nullptr, nullptr));  // 0.8 m
  EXPECT_EQ(graph.nodeCount(), 0U);

  EXPECT_TRUE(d.updateState(nullptr, nullptr, nullptr));  // 1.2 m
  EXPECT_EQ(graph.nodeCount(), 2U);
  EXPECT_EQ(graph.edgeCount(), 1U);
  EXPECT_NEAR(graph.nodes.at(0).x(), 0.0, 1e-6);
  EXPECT_NEAR(graph.nodes.at(1).x(), 1.2, 1e-6);
}

TEST(CIncrementalNodeRegistrationDecider, registration_condition_on_2D_and_3D_poses)
{
  StepNRD d(mrpt::poses::CPose2D(0, 0, 0));
  d.params.registration_max_distance = 0.5;
  d.params.registration_max_angle = mrpt::DEG2RAD(20.0);

  const mrpt::poses::CPose2D o2;
  EXPECT_FALSE(d.checkRegistrationConditionPose(o2, mrpt::poses::CPose2D(0.4, 0, 0)));
  EXPECT_TRUE(d.checkRegistrationConditionPose(o2, mrpt::poses::CPose2D(0.6, 0, 0)));
  EXPECT_TRUE(
      d.checkRegistrationConditionPose(o2, mrpt::poses::CPose2D(0, 0, mrpt::DEG2RAD(30.0))));

  const mrpt::poses::CPose3D o3;
  EXPECT_FALSE(d.checkRegistrationConditionPose(o3, mrpt::poses::CPose3D(0.4, 0, 0, 0, 0, 0)));
  EXPECT_TRUE(d.checkRegistrationConditionPose(o3, mrpt::poses::CPose3D(0, 0, 0.6, 0, 0, 0)));
  EXPECT_TRUE(d.checkRegistrationConditionPose(
      o3, mrpt::poses::CPose3D(0, 0, 0, 0, mrpt::DEG2RAD(30.0), 0)));
}

TEST(CIncrementalNodeRegistrationDecider, descriptive_report_includes_the_params)
{
  StepNRD d(mrpt::poses::CPose2D(0, 0, 0));

  std::string report;
  d.getDescriptiveReport(&report);
  EXPECT_NE(report.find("Node Registration Decider Strategy"), std::string::npos) << report;
  EXPECT_NE(report.find("Max distance for registration"), std::string::npos) << report;
}
