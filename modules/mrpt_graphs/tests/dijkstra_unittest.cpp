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
#include <mrpt/graphs/CDirectedGraph.h>
#include <mrpt/graphs/dijkstra.h>

using namespace mrpt::graphs;

namespace
{
/** A minimal class-type edge value (TYPE_EDGES must be a class/struct, not a
 * plain scalar, since edge_t derives from it). */
struct TWeight
{
  double w{0};
  TWeight() = default;
  TWeight(double v) : w(v) {}
  operator double() const { return w; }
};
}  // namespace

using graph_t = CDirectedGraph<TWeight>;
using dijkstra_t = CDijkstra<graph_t>;

static graph_t buildSampleGraph()
{
  graph_t g;
  g.insertEdge(0, 1, 1.0);
  g.insertEdge(1, 2, 1.0);
  g.insertEdge(0, 3, 10.0);
  g.insertEdge(3, 4, 1.0);
  g.insertEdge(2, 4, 1.0);
  return g;
}

TEST(CDijkstra, DefaultUnitWeights)
{
  const graph_t g = buildSampleGraph();
  dijkstra_t dij(g, 0);

  // With unit hop-count weights, the 0-3-4 path (2 hops) beats 0-1-2-4 (3
  // hops):
  EXPECT_DOUBLE_EQ(*dij.getNodeDistanceToRoot(0), 0.0);
  EXPECT_DOUBLE_EQ(*dij.getNodeDistanceToRoot(1), 1.0);
  EXPECT_DOUBLE_EQ(*dij.getNodeDistanceToRoot(2), 2.0);
  EXPECT_DOUBLE_EQ(*dij.getNodeDistanceToRoot(3), 1.0);
  EXPECT_DOUBLE_EQ(*dij.getNodeDistanceToRoot(4), 2.0);

  EXPECT_EQ(dij.getRootNodeID(), 0U);
  EXPECT_EQ(dij.getListOfAllNodes().size(), 5U);
}

TEST(CDijkstra, CustomEdgeWeightFunctor)
{
  const graph_t g = buildSampleGraph();
  dijkstra_t::functor_edge_weight_t weightFn = [](const graph_t&, TNodeID, TNodeID,
                                                  const graph_t::edge_t& edge) { return edge; };

  dijkstra_t dij(g, 0, weightFn);

  // Now the direct edge weights matter: 0-1-2-4 costs 1+1+1=3, cheaper than
  // the direct 0-3 edge (weight 10). And node 3 is in turn reached more
  // cheaply through 0-1-2-4-3 (cost 4) than via the direct 0-3 edge (10),
  // since Dijkstra explores the undirected adjacency regardless of edge
  // direction.
  EXPECT_DOUBLE_EQ(*dij.getNodeDistanceToRoot(4), 3.0);
  EXPECT_DOUBLE_EQ(*dij.getNodeDistanceToRoot(3), 4.0);
}

TEST(CDijkstra, ProgressFunctorInvokedPerVisitedNode)
{
  const graph_t g = buildSampleGraph();

  size_t progressCalls = 0;
  size_t lastVisitedCount = 0;
  dijkstra_t::functor_on_progress_t progressFn = [&](const graph_t&, size_t visitedCount)
  {
    progressCalls++;
    lastVisitedCount = visitedCount;
  };

  dijkstra_t dij(g, 0, dijkstra_t::functor_edge_weight_t(), progressFn);
  EXPECT_EQ(progressCalls, 5U);
  EXPECT_EQ(lastVisitedCount, 5U);
}

TEST(CDijkstra, GetNodeDistanceToRootUnknownNode)
{
  const graph_t g = buildSampleGraph();
  dijkstra_t dij(g, 0);
  EXPECT_FALSE(dij.getNodeDistanceToRoot(99).has_value());
}

TEST(CDijkstra, GetShortestPathToRootIsEmpty)
{
  const graph_t g = buildSampleGraph();
  dijkstra_t dij(g, 0);
  EXPECT_TRUE(dij.getShortestPathTo(0).empty());
}

TEST(CDijkstra, GetShortestPathToRegularNode)
{
  const graph_t g = buildSampleGraph();
  dijkstra_t dij(g, 0);

  const auto path = dij.getShortestPathTo(4);
  ASSERT_EQ(path.size(), 2U);
  EXPECT_EQ(path.front(), std::make_pair(TNodeID(0), TNodeID(3)));
  EXPECT_EQ(path.back(), std::make_pair(TNodeID(3), TNodeID(4)));
}

TEST(CDijkstra, GetTreeGraph)
{
  const graph_t g = buildSampleGraph();
  dijkstra_t dij(g, 0);

  const auto tree = dij.getTreeGraph();
  EXPECT_EQ(tree.root, 0U);

  std::set<TNodeID> visitedIds;
  tree.visitDepthFirst(
      tree.root, [&](TNodeID, const dijkstra_t::tree_graph_t::TEdgeInfo& edge, size_t)
      { visitedIds.insert(edge.id); });
  EXPECT_EQ(visitedIds, (std::set<TNodeID>{1, 2, 3, 4}));

  dijkstra_t::tree_graph_t treeOut;
  dij.getTreeGraph(treeOut);
  EXPECT_EQ(treeOut.root, 0U);
}

TEST(CDijkstra, CachedAdjacencyMatrix)
{
  const graph_t g = buildSampleGraph();
  dijkstra_t dij(g, 0);
  const auto& adj = dij.getCachedAdjacencyMatrix();
  EXPECT_EQ(adj.size(), 5U);
}

TEST(CDijkstra, ThrowsOnUnknownSourceNode)
{
  const graph_t g = buildSampleGraph();
  EXPECT_THROW(dijkstra_t(g, 999), std::exception);
}

TEST(CDijkstra, MaximumDistanceLimitsSearch)
{
  const graph_t g = buildSampleGraph();
  dijkstra_t dij(g, 0, dijkstra_t::functor_edge_weight_t(), dijkstra_t::functor_on_progress_t(), 1);

  EXPECT_DOUBLE_EQ(*dij.getNodeDistanceToRoot(0), 0.0);
  EXPECT_DOUBLE_EQ(*dij.getNodeDistanceToRoot(1), 1.0);
  EXPECT_DOUBLE_EQ(*dij.getNodeDistanceToRoot(3), 1.0);
  // Node 2 and 4 are farther than the given topological limit:
  EXPECT_FALSE(dij.getNodeDistanceToRoot(2).has_value());
  EXPECT_FALSE(dij.getNodeDistanceToRoot(4).has_value());
}

TEST(CDijkstra, NotConnectedGraphThrows)
{
  graph_t g = buildSampleGraph();
  // Isolated node 5, only referenced through a self-loop (also exercises the
  // "ignore self-loops" branch inside CDijkstra):
  g.insertEdge(5, 5, 1.0);

  bool threw = false;
  try
  {
    dijkstra_t dij(g, 0);
  }
  catch (mrpt::graphs::detail::NotConnectedGraph& ex)
  {
    threw = true;
    std::set<TNodeID> unconnected;
    ex.getUnconnectedNodeIDs(&unconnected);
    EXPECT_EQ(unconnected, (std::set<TNodeID>{5}));
    EXPECT_NE(std::string(ex.what()), "");
  }
  EXPECT_TRUE(threw);
}
