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

/** Math checks for CLoopCloserERD: pairwise consistency of loop closure
 *  hypotheses, spectral selection of the consistent set, partition splitting,
 *  ICP vs odometry gating and minimum uncertainty paths. No laser scans nor
 *  GUI are involved: graphs and hypotheses are built in memory.
 */

#include <gtest/gtest.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/ERD/CLoopCloserERD.h>
#include <mrpt/math/CMatrixDynamic.h>

#include <Eigen/Dense>
#include <cmath>
#include <set>
#include <vector>

using graph_t = mrpt::graphs::CNetworkOfPoses2DInf;
using erd_t = mrpt::graphslam::deciders::CLoopCloserERD<graph_t>;
using constraint_t = graph_t::constraint_t;
using hypot_t = erd_t::hypot_t;
using mrpt::graphs::TNodeID;
using mrpt::poses::CPose2D;

namespace
{
/** Exposes the protected helpers and sets the parameters normally read from
 *  the .ini file */
class TestLC : public erd_t
{
 public:
  using erd_t::execDijkstraProjection;
  using erd_t::getMinUncertaintyPath;
  using erd_t::m_laser_params;
  using erd_t::m_lc_params;
  using erd_t::mahalanobisDistanceOdometryToICPEdge;
  using erd_t::queryOptimalPath;
  using erd_t::splitPartitionToGroups;

  TestLC()
  {
    this->setMinLoggingLevel(mrpt::system::LVL_ERROR);
    m_lc_params.LC_min_nodeid_diff = 30;
    m_lc_params.LC_eigenvalues_ratio_thresh = 2.0;
    m_lc_params.LC_min_remote_nodes = 3;
    m_lc_params.LC_check_curr_partition_only = true;
    m_lc_params.full_partition_per_nodes = 50;
  }
};

constraint_t makeEdge(const CPose2D& mean, double information)
{
  constraint_t c;
  c.mean = mean;
  c.cov_inv.setDiagonal(information);
  return c;
}

/** Poses of a robot that drives away and comes back */
const std::vector<CPose2D>& loopPoses()
{
  static const std::vector<CPose2D> poses = {CPose2D(0.0, 0.0, 0.0), CPose2D(1.0, 0.0, 0.1),
                                             CPose2D(2.0, 0.2, 1.0), CPose2D(2.5, 1.0, 2.0),
                                             CPose2D(2.0, 1.8, 3.0), CPose2D(1.0, 1.5, -2.9),
                                             CPose2D(0.0, 1.2, -3.0)};
  return poses;
}

/** Chain graph with exact odometry edges i -> i+1 */
void buildChainGraph(graph_t& g, double information)
{
  const auto& p = loopPoses();
  for (size_t i = 0; i < p.size(); i++)
  {
    g.nodes[i] = p[i];
  }
  for (size_t i = 0; i + 1 < p.size(); i++)
  {
    g.insertEdge(i, i + 1, makeEdge(p[i + 1] - p[i], information));
  }
}

/** Hypotheses b -> a (as generateHypotsPool() creates them), with the exact
 *  relative pose, optionally perturbed by "err" on the right */
std::vector<hypot_t> makeHypots(
    const std::vector<uint32_t>& groupA,
    const std::vector<uint32_t>& groupB,
    TNodeID perturbed_from,
    TNodeID perturbed_to,
    const CPose2D& err,
    double information)
{
  const auto& p = loopPoses();
  std::vector<hypot_t> hs;
  int id = 0;
  for (const auto b : groupB)
  {
    for (const auto a : groupA)
    {
      hypot_t h;
      h.from = b;
      h.to = a;
      h.id = id++;
      CPose2D rel = p[a] - p[b];
      if (b == perturbed_from && a == perturbed_to)
      {
        rel = rel + err;
      }
      h.setEdge(makeEdge(rel, information));
      hs.push_back(h);
    }
  }
  return hs;
}

erd_t::hypotsp_t pointersTo(std::vector<hypot_t>& hs)
{
  erd_t::hypotsp_t ptrs;
  for (auto& h : hs)
  {
    ptrs.push_back(&h);
  }
  return ptrs;
}

int idOf(const std::vector<hypot_t>& hs, TNodeID from, TNodeID to)
{
  for (const auto& h : hs)
  {
    if (h.from == from && h.to == to)
    {
      return h.id;
    }
  }
  return -1;
}

std::set<int> selectedIDs(const erd_t::hypotsp_t& valid)
{
  std::set<int> ids;
  for (const auto* h : valid)
  {
    ids.insert(h->id);
  }
  return ids;
}

erd_t::hypotsp_t evalMatrix(
    TestLC& lc, const mrpt::math::CMatrixDouble& m, std::vector<hypot_t>& hs)
{
  erd_t::hypotsp_t valid;
  lc.evalPWConsistenciesMatrix(m, pointersTo(hs), &valid);
  return valid;
}

std::vector<hypot_t> plainHypots(size_t n)
{
  std::vector<hypot_t> hs(n);
  for (size_t i = 0; i < n; i++)
  {
    hs[i].id = static_cast<int>(i);
    hs[i].from = static_cast<TNodeID>(100 + i);
    hs[i].to = static_cast<TNodeID>(i);
  }
  return hs;
}
}  // namespace

// ---------------------------------------------------------------------------
// Pairwise consistency
// ---------------------------------------------------------------------------

TEST(CLoopCloserERD, consistency_of_an_exact_loop_is_one)
{
  // The loop a1 -> a2 => b1 -> b2 => a1 closes exactly if the hypotheses are
  // the true relative poses. Non-trivial headings exercise the composition
  // order and the inversion of the b1 -> a2 hypothesis.
  graph_t g;
  buildChainGraph(g, 1e4);
  TestLC lc;
  lc.setGraphPtr(&g);

  const std::vector<uint32_t> groupA = {0, 1};
  const std::vector<uint32_t> groupB = {5, 6};
  auto hs = makeHypots(groupA, groupB, 0, 0, CPose2D(), 1e4);
  const auto ptrs = pointersTo(hs);

  mrpt::math::CMatrixDouble m(hs.size(), hs.size());
  lc.generatePWConsistenciesMatrix(groupA, groupB, ptrs, &m);

  const int h60 = idOf(hs, 6, 0);
  const int h51 = idOf(hs, 5, 1);
  EXPECT_NEAR(m(h60, h51), 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(m(h60, h51), m(h51, h60));
}

TEST(CLoopCloserERD, consistency_of_an_inconsistent_loop_is_low)
{
  graph_t g;
  buildChainGraph(g, 1e4);
  TestLC lc;
  lc.setGraphPtr(&g);

  const std::vector<uint32_t> groupA = {0, 1};
  const std::vector<uint32_t> groupB = {5, 6};
  const CPose2D err(0.3, -0.2, 0.15);
  auto hs = makeHypots(groupA, groupB, 6, 0, err, 1e4);
  const auto ptrs = pointersTo(hs);

  mrpt::math::CMatrixDouble m(hs.size(), hs.size());
  lc.generatePWConsistenciesMatrix(groupA, groupB, ptrs, &m);

  const int h60 = idOf(hs, 6, 0);
  const int h51 = idOf(hs, 5, 1);
  EXPECT_LT(m(h60, h51), 1e-3) << "consistency = " << m(h60, h51);
}

TEST(CLoopCloserERD, consistency_is_evaluated_for_parallel_hypotheses)
{
  graph_t g;
  buildChainGraph(g, 1e4);
  TestLC lc;
  lc.setGraphPtr(&g);

  const std::vector<uint32_t> groupA = {0, 1};
  const std::vector<uint32_t> groupB = {5, 6};
  auto hs = makeHypots(groupA, groupB, 0, 0, CPose2D(), 1e4);
  const auto ptrs = pointersTo(hs);

  mrpt::math::CMatrixDouble m(hs.size(), hs.size());
  lc.generatePWConsistenciesMatrix(groupA, groupB, ptrs, &m);

  const int h50 = idOf(hs, 5, 0);
  const int h61 = idOf(hs, 6, 1);
  EXPECT_GT(m(h50, h61), 0.5) << "consistency = " << m(h50, h61);
}

// ---------------------------------------------------------------------------
// Spectral selection of the consistent hypotheses
// ---------------------------------------------------------------------------

TEST(CLoopCloserERD, evalPWConsistenciesMatrix_selects_the_dominant_cluster)
{
  TestLC lc;
  auto hs = plainHypots(5);
  mrpt::math::CMatrixDouble m(5, 5);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (i != j)
      {
        m(i, j) = 0.9;
      }
    }
  }
  m(3, 4) = 0.3;
  m(4, 3) = 0.3;

  const auto valid = evalMatrix(lc, m, hs);
  EXPECT_EQ(selectedIDs(valid), std::set<int>({0, 1, 2}));
}

TEST(CLoopCloserERD, evalPWConsistenciesMatrix_accepts_a_set_with_no_second_cluster)
{
  TestLC lc;
  auto hs = plainHypots(4);
  mrpt::math::CMatrixDouble m(4, 4);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (i != j)
      {
        m(i, j) = 0.9;
      }
    }
  }
  // eigenvalues: {-0.9, -0.9, 0, 1.8}
  const auto valid = evalMatrix(lc, m, hs);
  EXPECT_EQ(selectedIDs(valid), std::set<int>({0, 1, 2}));

  // All the hypotheses consistent: eigenvalues {-0.9, -0.9, 1.8}
  auto hs3 = plainHypots(3);
  mrpt::math::CMatrixDouble m3(3, 3);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (i != j)
      {
        m3(i, j) = 0.9;
      }
    }
  }
  const auto valid3 = evalMatrix(lc, m3, hs3);
  EXPECT_EQ(selectedIDs(valid3), std::set<int>({0, 1, 2}));
}

TEST(CLoopCloserERD, evalPWConsistenciesMatrix_discretization_drops_weak_members)
{
  TestLC lc;
  auto hs = plainHypots(6);
  mrpt::math::CMatrixDouble m(6, 6);
  for (int i = 1; i < 4; i++)
  {
    for (int j = 1; j < 4; j++)
    {
      if (i != j)
      {
        m(i, j) = 0.9;
      }
    }
    m(0, i) = 0.1;
    m(i, 0) = 0.1;
  }
  m(4, 5) = 0.3;
  m(5, 4) = 0.3;

  // Independent reference: dominant eigenvector, then the best threshold.
  Eigen::MatrixXd A = m.asEigen();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
  const Eigen::VectorXd u = es.eigenvectors().col(5).cwiseAbs();
  std::vector<int> order = {0, 1, 2, 3, 4, 5};
  std::sort(order.begin(), order.end(), [&u](int a, int b) { return u[a] > u[b]; });
  double best = 0;
  size_t best_k = 0;
  double sum = 0;
  for (size_t k = 0; k < order.size(); k++)
  {
    sum += u[order[k]];
    const double cosine = sum / std::sqrt(static_cast<double>(k + 1));
    if (cosine > best)
    {
      best = cosine;
      best_k = k + 1;
    }
  }
  std::set<int> expected(order.begin(), order.begin() + static_cast<long>(best_k));
  ASSERT_EQ(expected, std::set<int>({1, 2, 3}));

  const auto valid = evalMatrix(lc, m, hs);
  EXPECT_EQ(selectedIDs(valid), expected);
}

TEST(CLoopCloserERD, evalPWConsistenciesMatrix_rejects_two_equal_clusters)
{
  // Two equally strong clusters: eigenvalue ratio 1 < 2, nothing selected.
  TestLC lc;
  auto hs = plainHypots(4);
  mrpt::math::CMatrixDouble m(4, 4);
  m(0, 1) = 0.9;
  m(1, 0) = 0.9;
  m(2, 3) = 0.9;
  m(3, 2) = 0.9;
  m(0, 0) = 1.0;
  m(1, 1) = 1.0;
  m(2, 2) = 1.0;
  m(3, 3) = 1.0;
  const auto valid = evalMatrix(lc, m, hs);
  EXPECT_TRUE(valid.empty());
}

// ---------------------------------------------------------------------------
// Partition split
// ---------------------------------------------------------------------------

TEST(CLoopCloserERD, splitPartitionToGroups_splits_at_the_node_id_gap)
{
  TestLC lc;
  std::vector<uint32_t> partition = {40, 41, 42, 43, 100, 101, 102, 103};
  std::vector<uint32_t> groupA;
  std::vector<uint32_t> groupB;
  lc.splitPartitionToGroups(partition, &groupA, &groupB, -1);
  EXPECT_EQ(groupA, std::vector<uint32_t>({40, 41, 42, 43}));
  EXPECT_EQ(groupB, std::vector<uint32_t>({100, 101, 102, 103}));
}

TEST(CLoopCloserERD, splitPartitionToGroups_limits_the_group_sizes)
{
  TestLC lc;
  std::vector<uint32_t> partition = {0, 1, 2, 3, 4, 5, 6, 50, 51, 52, 53, 54, 55, 56};
  std::vector<uint32_t> groupA;
  std::vector<uint32_t> groupB;
  lc.splitPartitionToGroups(partition, &groupA, &groupB, 3);
  // The first nodes of A and the last ones of B:
  EXPECT_EQ(groupA, std::vector<uint32_t>({0, 1, 2}));
  EXPECT_EQ(groupB, std::vector<uint32_t>({54, 55, 56}));
}

// ---------------------------------------------------------------------------
// ICP edge vs. current graph gating
// ---------------------------------------------------------------------------

TEST(CLoopCloserERD, mahalanobis_gate_uses_the_edge_parameter_frame)
{
  graph_t g;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(1.0, 0.0, M_PI / 2);
  TestLC lc;
  lc.setGraphPtr(&g);

  // Edge certain along x (in the from frame), uncertain along y:
  constraint_t edge;
  edge.mean = CPose2D(1.0, 0.5, M_PI / 2);
  edge.cov_inv.setZero();
  edge.cov_inv(0, 0) = 1e4;
  edge.cov_inv(1, 1) = 1.0;
  edge.cov_inv(2, 2) = 1e4;

  lc.mahalanobisDistanceOdometryToICPEdge(0, 1, edge);

  // Parameter-space difference (0, 0.5, 0) under that covariance:
  const double expected = 0.5 * 0.5 * 1.0;
  EXPECT_NEAR(lc.m_laser_params.mahal_distance_ICP_odom_win.getMedian(), expected, 1e-6);
}

TEST(CLoopCloserERD, mahalanobis_gate_is_zero_for_an_edge_equal_to_the_graph)
{
  graph_t g;
  g.nodes[0] = CPose2D(0.5, -1.0, 0.3);
  g.nodes[1] = CPose2D(1.5, 0.2, 2.9);
  TestLC lc;
  lc.setGraphPtr(&g);

  const constraint_t edge = makeEdge(g.nodes[1] - g.nodes[0], 100.0);
  EXPECT_TRUE(lc.mahalanobisDistanceOdometryToICPEdge(0, 1, edge));
  EXPECT_NEAR(lc.m_laser_params.mahal_distance_ICP_odom_win.getMedian(), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Minimum uncertainty paths
// ---------------------------------------------------------------------------

TEST(CLoopCloserERD, getMinUncertaintyPath_inverts_backward_edges)
{
  graph_t g;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(1.0, 2.0, 0.7);
  // Only a backward edge 1 -> 0 exists:
  const CPose2D e10 = g.nodes[0] - g.nodes[1];
  g.insertEdge(1, 0, makeEdge(e10, 100.0));
  TestLC lc;
  lc.setGraphPtr(&g);

  erd_t::path_t path;
  lc.getMinUncertaintyPath(0, 1, &path);
  ASSERT_EQ(path.nodes_traversed, std::vector<TNodeID>({0, 1}));
  const CPose2D expected = CPose2D() - e10;
  EXPECT_NEAR(path.curr_pose_pdf.mean.x(), expected.x(), 1e-9);
  EXPECT_NEAR(path.curr_pose_pdf.mean.y(), expected.y(), 1e-9);
  EXPECT_NEAR(path.curr_pose_pdf.mean.phi(), expected.phi(), 1e-9);
}

TEST(CLoopCloserERD, getMinUncertaintyPath_picks_the_most_certain_edge)
{
  graph_t g;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(1.0, 0.0, 0.0);
  g.insertEdge(0, 1, makeEdge(CPose2D(1.0, 0.0, 0.0), 10.0));
  g.insertEdge(1, 0, makeEdge(CPose2D(-1.1, 0.0, 0.0), 1000.0));
  TestLC lc;
  lc.setGraphPtr(&g);

  erd_t::path_t path;
  lc.getMinUncertaintyPath(0, 1, &path);
  // The backward (more certain) edge is the one used:
  EXPECT_NEAR(path.curr_pose_pdf.mean.x(), 1.1, 1e-9);
}

TEST(CLoopCloserERD, dijkstra_projection_follows_the_most_certain_route)
{
  // Chain 0-1-2-3 of precise edges, plus a weak direct edge 0 -> 3:
  graph_t g;
  for (TNodeID i = 0; i < 4; i++)
  {
    g.nodes[i] = CPose2D(static_cast<double>(i), 0, 0);
  }
  for (TNodeID i = 0; i < 3; i++)
  {
    g.insertEdge(i, i + 1, makeEdge(CPose2D(1.0, 0, 0), 1e4));
  }
  g.insertEdge(0, 3, makeEdge(CPose2D(3.0, 0, 0), 1.0));
  TestLC lc;
  lc.setGraphPtr(&g);

  lc.execDijkstraProjection(0, 3);
  const auto* p = lc.queryOptimalPath(3);
  ASSERT_TRUE(p != nullptr);
  EXPECT_EQ(p->nodes_traversed, std::vector<TNodeID>({0, 1, 2, 3}));
  EXPECT_NEAR(p->curr_pose_pdf.mean.x(), 3.0, 1e-9);
}

TEST(CLoopCloserERD, dijkstra_projection_without_end_node_terminates)
{
  graph_t g;
  for (TNodeID i = 0; i < 4; i++)
  {
    g.nodes[i] = CPose2D(static_cast<double>(i), 0, 0);
  }
  for (TNodeID i = 0; i < 3; i++)
  {
    g.insertEdge(i, i + 1, makeEdge(CPose2D(1.0, 0, 0), 1e4));
  }
  TestLC lc;
  lc.setGraphPtr(&g);
  lc.execDijkstraProjection(0);
  EXPECT_TRUE(lc.queryOptimalPath(3) != nullptr);
}
