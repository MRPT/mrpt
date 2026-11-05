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

/** Checks for CLevMarqGSO: which nodes are handed to the optimizer in full
 *  and partial (distance-limited) optimization runs.
 */

#include <gtest/gtest.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/GSO/CLevMarqGSO.h>

#include <set>

using graph_t = mrpt::graphs::CNetworkOfPoses2DInf;
using gso_t = mrpt::graphslam::optimizers::CLevMarqGSO<graph_t>;
using mrpt::graphs::TNodeID;
using mrpt::poses::CPose2D;

namespace
{
class TestGSO : public gso_t
{
 public:
  using gso_t::_optimizeGraph;
  using gso_t::getNearbyNodesOf;

  explicit TestGSO(double optimization_distance)
  {
    this->setMinLoggingLevel(mrpt::system::LVL_ERROR);
    opt_params.optimization_distance = optimization_distance;
    opt_params.optimization_on_second_thread = false;
    opt_params.LC_min_nodeid_diff = 30;
    opt_params.cfg["max_iterations"] = 100;
  }
};

graph_t::constraint_t makeEdge(const CPose2D& mean, double information)
{
  graph_t::constraint_t c;
  c.mean = mean;
  c.cov_inv.setDiagonal(information);
  return c;
}

/** Chain of unit steps along x with an inconsistent closing edge n-1 -> 0 */
graph_t makeInconsistentLoop(TNodeID n)
{
  graph_t g;
  for (TNodeID i = 0; i < n; i++)
  {
    g.nodes[i] = CPose2D(static_cast<double>(i), 0, 0);
  }
  for (TNodeID i = 0; i + 1 < n; i++)
  {
    g.insertEdge(i, i + 1, makeEdge(CPose2D(1.0, 0, 0), 1e4));
  }
  g.insertEdge(n - 1, 0, makeEdge(CPose2D(-(static_cast<double>(n) - 1.5), 0.0, 0.0), 1e4));
  return g;
}
}  // namespace

TEST(CLevMarqGSO, nearby_nodes_are_selected_by_distance)
{
  graph_t g = makeInconsistentLoop(10);
  TestGSO gso(2.5);
  gso.setGraphPtr(&g);
  std::set<TNodeID> nodes;
  gso.getNearbyNodesOf(&nodes, 9, 2.5);
  EXPECT_EQ(nodes, std::set<TNodeID>({7, 8}));
}

TEST(CLevMarqGSO, full_optimization_keeps_the_root_fixed)
{
  graph_t g = makeInconsistentLoop(4);
  const double chi2_before = g.chi2();
  TestGSO gso(5.0);
  gso.setGraphPtr(&g);
  gso._optimizeGraph(true);

  EXPECT_LT(g.chi2(), 0.5 * chi2_before);
  EXPECT_NEAR(g.nodes.at(0).x(), 0.0, 1e-12);
  EXPECT_NEAR(g.nodes.at(0).y(), 0.0, 1e-12);
  EXPECT_NEAR(g.nodes.at(0).phi(), 0.0, 1e-12);
}

TEST(CLevMarqGSO, partial_optimization_keeps_the_root_fixed)
{
  graph_t g = makeInconsistentLoop(4);
  TestGSO gso(5.0);
  gso.setGraphPtr(&g);
  gso._optimizeGraph(false);

  EXPECT_NEAR(g.nodes.at(0).x(), 0.0, 1e-9) << g.nodes.at(0);
  EXPECT_NEAR(g.nodes.at(0).y(), 0.0, 1e-9) << g.nodes.at(0);
  EXPECT_NEAR(g.nodes.at(0).phi(), 0.0, 1e-9) << g.nodes.at(0);
}

TEST(CLevMarqGSO, partial_optimization_leaves_far_nodes_untouched)
{
  graph_t g = makeInconsistentLoop(10);
  const graph_t g0 = g;
  TestGSO gso(2.5);
  gso.setGraphPtr(&g);
  gso._optimizeGraph(false);

  for (TNodeID i = 0; i < 7; i++)
  {
    EXPECT_EQ(g.nodes.at(i), g0.nodes.at(i)) << "node " << i;
  }
}
