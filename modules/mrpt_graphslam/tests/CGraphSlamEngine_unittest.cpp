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

/** Math checks for CGraphSlamEngine, run headless (no window manager) on an
 *  in-memory action-observation sequence: odometry bookkeeping, what happens
 *  to the optimized node poses, and the SLAM (deformation energy) metric.
 */

#include <gtest/gtest.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/CGraphSlamEngine.h>
#include <mrpt/graphslam/NRD/CFixedIntervalsNRD.h>
#include <mrpt/graphslam/interfaces/CEdgeRegistrationDecider.h>
#include <mrpt/graphslam/interfaces/CGraphSlamOptimizer.h>
#include <mrpt/graphslam/levmarq.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

using graph_t = mrpt::graphs::CNetworkOfPoses2DInf;
using engine_t = mrpt::graphslam::CGraphSlamEngine<graph_t>;
using mrpt::graphs::TNodeID;
using mrpt::poses::CPose2D;
using mrpt::poses::CPose3D;

namespace
{
/** Adds a single, deliberately inconsistent, loop closure once the graph
 *  reaches a given number of nodes */
class LoopERD : public mrpt::graphslam::deciders::CEdgeRegistrationDecider<graph_t>
{
 public:
  LoopERD(size_t at_node_count, const CPose2D& measurement) :
      m_at(at_node_count), m_meas(measurement)
  {
  }

  bool updateState(
      [[maybe_unused]] mrpt::obs::CActionCollection::Ptr action,
      [[maybe_unused]] mrpt::obs::CSensoryFrame::Ptr observations,
      [[maybe_unused]] mrpt::obs::CObservation::Ptr observation) override
  {
    if (!m_done && this->m_graph->nodeCount() == m_at)
    {
      graph_t::constraint_t c;
      c.mean = m_meas;
      c.cov_inv.setDiagonal(1e4);
      this->m_graph->insertEdge(static_cast<TNodeID>(m_at - 1), 0, c);
      m_done = true;
    }
    return true;
  }

 private:
  size_t m_at;
  CPose2D m_meas;
  bool m_done = false;
};

/** Optimizes the whole graph on every new node and keeps a copy of the result */
class RecordingGSO : public mrpt::graphslam::optimizers::CGraphSlamOptimizer<graph_t>
{
 public:
  bool updateState(
      [[maybe_unused]] mrpt::obs::CActionCollection::Ptr action,
      [[maybe_unused]] mrpt::obs::CSensoryFrame::Ptr observations,
      [[maybe_unused]] mrpt::obs::CObservation::Ptr observation) override
  {
    if (this->m_graph->nodeCount() > m_last_count)
    {
      m_last_count = this->m_graph->nodeCount();
      optimizeGraph();
    }
    return true;
  }
  void optimizeGraph() override
  {
    mrpt::graphslam::TResultInfoSpaLevMarq info;
    mrpt::graphslam::optimize_graph_spa_levmarq(*this->m_graph, info);
    optimized_nodes = this->m_graph->nodes;
    optimized_chi2 = this->m_graph->chi2();
  }
  bool justFullyOptimizedGraph() const override { return true; }

  graph_t::global_poses_t optimized_nodes;
  double optimized_chi2 = 0;

 private:
  size_t m_last_count = 0;
};

/** Exposes protected members of the engine */
class TestEngine : public engine_t
{
 public:
  using engine_t::accumulateAngleDiffs;
  using engine_t::computeSlamMetric;
  using engine_t::m_curr_deformation_energy;
  using engine_t::m_curr_odometry_only_pose;
  using engine_t::m_graph;
  using engine_t::m_GT_poses;

  TestEngine(
      const std::string& cfg,
      mrpt::graphslam::deciders::CNodeRegistrationDecider<graph_t>* nrd,
      mrpt::graphslam::deciders::CEdgeRegistrationDecider<graph_t>* erd,
      mrpt::graphslam::optimizers::CGraphSlamOptimizer<graph_t>* gso) :
      engine_t(cfg, "", "", nullptr, nrd, erd, gso)
  {
  }
};

std::string writeConfigFile()
{
  const std::string fname = mrpt::system::getTempFileName();
  std::ofstream f(fname);
  f << "[GeneralConfiguration]\n"
       "class_verbosity = 3\n"
       "[VisualizationParameters]\n"
       "visualize_map = false\n"
       "visualize_odometry_poses = false\n"
       "visualize_estimated_trajectory = false\n"
       "visualize_ground_truth = false\n"
       "visualize_SLAM_metric = false\n"
       "enable_curr_pos_viewport = false\n"
       "[NodeRegistrationDeciderParameters]\n"
       "class_verbosity = 3\n"
       "registration_max_distance = 0.5\n"
       "registration_max_angle = 60\n"
       "[EdgeRegistrationDeciderParameters]\n"
       "class_verbosity = 3\n"
       "[OptimizerParameters]\n"
       "class_verbosity = 3\n";
  return fname;
}

mrpt::obs::CActionCollection::Ptr makeOdometryAction(const CPose2D& increment, int step)
{
  mrpt::obs::CActionRobotMovement2D act;
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions opts;
  opts.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
  opts.gaussianModel.minStdXY = 0.02;
  opts.gaussianModel.minStdPHI = mrpt::DEG2RAD(0.5);
  act.computeFromOdometry(increment, opts);
  act.timestamp = mrpt::Clock::fromDouble(1000.0 + step);

  auto acts = mrpt::obs::CActionCollection::Create();
  acts->insert(act);
  return acts;
}

void runSquare(TestEngine& engine, int steps)
{
  const CPose2D increment(1.0, 0.0, M_PI / 2);
  for (int i = 0; i < steps; i++)
  {
    auto action = makeOdometryAction(increment, i);
    auto sf = mrpt::obs::CSensoryFrame::Create();
    mrpt::obs::CObservation::Ptr obs;
    size_t entry = static_cast<size_t>(i);
    engine._execGraphSlamStep(action, sf, obs, entry);
  }
}

double rotationAngle(const CPose3D& a, const CPose3D& b)
{
  const Eigen::Matrix3d R =
      a.getRotationMatrix().asEigen().transpose() * b.getRotationMatrix().asEigen();
  return Eigen::AngleAxisd(R).angle();
}
}  // namespace

TEST(CGraphSlamEngine, odometry_only_pose_counts_each_increment_once)
{
  const std::string cfg = writeConfigFile();
  mrpt::graphslam::deciders::CFixedIntervalsNRD<graph_t> nrd;
  LoopERD erd(1000, CPose2D());
  RecordingGSO gso;
  TestEngine engine(cfg, &nrd, &erd, &gso);

  runSquare(engine, 1);
  EXPECT_NEAR(engine.m_curr_odometry_only_pose.x(), 1.0, 1e-9) << engine.m_curr_odometry_only_pose;
  EXPECT_NEAR(engine.m_curr_odometry_only_pose.y(), 0.0, 1e-9) << engine.m_curr_odometry_only_pose;
  EXPECT_NEAR(engine.m_curr_odometry_only_pose.phi(), M_PI / 2, 1e-9)
      << engine.m_curr_odometry_only_pose;
  mrpt::system::deleteFile(cfg);
}

TEST(CGraphSlamEngine, node_poses_keep_the_optimizer_result)
{
  const std::string cfg = writeConfigFile();
  mrpt::graphslam::deciders::CFixedIntervalsNRD<graph_t> nrd;
  // A 4 x 1 m square loop; the closing edge 4 -> 0 disagrees with odometry.
  LoopERD erd(5, CPose2D(0.3, 0.2, 0.1));
  RecordingGSO gso;
  TestEngine engine(cfg, &nrd, &erd, &gso);

  runSquare(engine, 4);
  const auto& g = engine.getGraph();
  ASSERT_EQ(g.nodeCount(), 5U);
  ASSERT_EQ(g.edgeCount(), 5U);
  ASSERT_EQ(gso.optimized_nodes.size(), 5U);

  EXPECT_NEAR(g.chi2(), gso.optimized_chi2, 1e-6 * (1.0 + gso.optimized_chi2))
      << "engine chi2 = " << g.chi2() << " optimized chi2 = " << gso.optimized_chi2;
  for (const auto& [id, p] : gso.optimized_nodes)
  {
    EXPECT_NEAR(g.nodes.at(id).x(), p.x(), 1e-9) << "node " << id;
    EXPECT_NEAR(g.nodes.at(id).y(), p.y(), 1e-9) << "node " << id;
  }
  mrpt::system::deleteFile(cfg);
}

TEST(CGraphSlamEngine, slam_metric_is_the_mean_relative_error)
{
  const std::string cfg = writeConfigFile();
  mrpt::graphslam::deciders::CFixedIntervalsNRD<graph_t> nrd;
  LoopERD erd(1000, CPose2D());
  RecordingGSO gso;
  TestEngine engine(cfg, &nrd, &erd, &gso);

  const std::vector<CPose2D> gt = {
      CPose2D(0, 0, 0), CPose2D(1, 0, 0.1), CPose2D(2, 0.2, 0.3), CPose2D(3, 0.5, 0.2),
      CPose2D(4, 0.6, 0.0)};
  const std::vector<CPose2D> est = {
      CPose2D(0, 0, 0), CPose2D(1.1, 0.05, 0.12), CPose2D(2.0, 0.3, 0.25), CPose2D(3.2, 0.5, 0.3),
      CPose2D(4.1, 0.4, -0.05)};
  engine.m_GT_poses = gt;
  for (TNodeID i = 0; i < est.size(); i++)
  {
    engine.m_graph.nodes[i] = est[i];
  }
  for (TNodeID i = 0; i < est.size(); i++)
  {
    engine.computeSlamMetric(i, i);
  }

  // Kummerle et al. (2009): mean over the relations of the squared
  // translational and rotational errors of the relative poses.
  double sum = 0;
  for (size_t i = 1; i < est.size(); i++)
  {
    const CPose2D d_est = est[i] - est[i - 1];
    const CPose2D d_gt = gt[i] - gt[i - 1];
    const CPose2D err = d_est - d_gt;
    sum += err.x() * err.x() + err.y() * err.y() + err.phi() * err.phi();
  }
  const double expected = sum / static_cast<double>(est.size() - 1);
  EXPECT_NEAR(engine.m_curr_deformation_energy, expected, 1e-9 * expected);

  std::vector<double> evolution;
  engine.getDeformationEnergyVector(&evolution);
  // One value per evaluation that had at least one relation:
  EXPECT_EQ(evolution.size(), est.size() - 1);
  mrpt::system::deleteFile(cfg);
}

TEST(CGraphSlamEngine, angle_difference_2D_is_wrapped)
{
  const double d = TestEngine::accumulateAngleDiffs(CPose2D(0, 0, 3.0), CPose2D(0, 0, -3.0));
  EXPECT_NEAR(d, 6.0 - 2 * M_PI, 1e-12);
}

TEST(CGraphSlamEngine, angle_difference_3D_is_the_relative_rotation_angle)
{
  const CPose3D p1(0, 0, 0, 0.1, 0.0, 0.0);
  const CPose3D p2(0, 0, 0, 0.0, 0.0, 0.1);
  const double expected = rotationAngle(p1, p2);
  ASSERT_GT(expected, 0.1);
  EXPECT_NEAR(std::abs(TestEngine::accumulateAngleDiffs(p1, p2)), expected, 1e-6);
}
