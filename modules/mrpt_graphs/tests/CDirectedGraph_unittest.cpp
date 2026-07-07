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

#include <cstdio>
#include <fstream>
#include <sstream>

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

TEST(CDirectedGraph, EmptyGraph)
{
  graph_t g;
  EXPECT_EQ(g.edgeCount(), 0U);
  EXPECT_EQ(g.getAllNodes().size(), 0U);
  EXPECT_EQ(g.countDifferentNodesInEdges(), 0U);
}

TEST(CDirectedGraph, InsertAndQueryEdges)
{
  graph_t g;
  g.insertEdge(0, 1, 1.0);
  g.insertEdgeAtEnd(1, 2, 2.0);
  g.insertEdge(2, 3, 3.0);

  EXPECT_EQ(g.edgeCount(), 3U);
  EXPECT_TRUE(g.edgeExists(0, 1));
  EXPECT_TRUE(g.edgeExists(1, 2));
  EXPECT_FALSE(g.edgeExists(1, 0));
  EXPECT_FALSE(g.edgeExists(5, 6));

  EXPECT_DOUBLE_EQ(g.getEdge(0, 1), 1.0);
  EXPECT_DOUBLE_EQ(g.getEdge(2, 3), 3.0);

  EXPECT_THROW(g.getEdge(9, 10), std::exception);
}

TEST(CDirectedGraph, MultipleEdgesBetweenNodes)
{
  graph_t g;
  g.insertEdge(0, 1, 10.0);
  g.insertEdge(0, 1, 20.0);

  auto range = g.getEdges(0, 1);
  int count = 0;
  for (auto it = range.first; it != range.second; ++it) count++;
  EXPECT_EQ(count, 2);

  const graph_t& cg = g;
  auto crange = cg.getEdges(0, 1);
  int ccount = 0;
  for (auto it = crange.first; it != crange.second; ++it) ccount++;
  EXPECT_EQ(ccount, 2);
}

TEST(CDirectedGraph, EraseEdge)
{
  graph_t g;
  g.insertEdge(0, 1, 1.0);
  g.insertEdge(1, 2, 2.0);
  ASSERT_TRUE(g.edgeExists(0, 1));
  g.eraseEdge(0, 1);
  EXPECT_FALSE(g.edgeExists(0, 1));
  EXPECT_EQ(g.edgeCount(), 1U);

  // No-op if not existing:
  g.eraseEdge(50, 60);
  EXPECT_EQ(g.edgeCount(), 1U);

  g.clearEdges();
  EXPECT_EQ(g.edgeCount(), 0U);
}

TEST(CDirectedGraph, GetAllNodes)
{
  graph_t g;
  g.insertEdge(0, 1, 1.0);
  g.insertEdge(1, 2, 1.0);
  g.insertEdge(2, 0, 1.0);

  std::set<TNodeID> nodes;
  g.getAllNodes(nodes);
  EXPECT_EQ(nodes.size(), 3U);
  EXPECT_EQ(g.getAllNodes(), nodes);
  EXPECT_EQ(g.countDifferentNodesInEdges(), 3U);
}

TEST(CDirectedGraph, GetNeighborsOf)
{
  graph_t g;
  g.insertEdge(0, 1, 1.0);
  g.insertEdge(2, 0, 1.0);
  g.insertEdge(3, 4, 1.0);

  std::set<TNodeID> neighbors;
  g.getNeighborsOf(0, neighbors);
  EXPECT_EQ(neighbors.size(), 2U);
  EXPECT_TRUE(neighbors.count(1) != 0);
  EXPECT_TRUE(neighbors.count(2) != 0);

  EXPECT_EQ(g.getNeighborsOf(0), neighbors);
  EXPECT_TRUE(g.getNeighborsOf(99).empty());
}

TEST(CDirectedGraph, GetAdjacencyMatrix)
{
  graph_t g;
  g.insertEdge(0, 1, 1.0);
  g.insertEdge(1, 2, 1.0);

  std::map<TNodeID, std::set<TNodeID>> adj;
  g.getAdjacencyMatrix(adj);
  EXPECT_EQ(adj.size(), 3U);
  EXPECT_TRUE(adj[0].count(1) != 0);
  EXPECT_TRUE(adj[1].count(0) != 0);
  EXPECT_TRUE(adj[1].count(2) != 0);

  // Restricted to a subset of nodes:
  std::set<TNodeID> onlyThese{0, 1};
  std::map<TNodeID, std::set<TNodeID>> adjRestricted;
  g.getAdjacencyMatrix(adjRestricted, onlyThese);
  EXPECT_EQ(adjRestricted.size(), 2U);
  EXPECT_TRUE(adjRestricted[0].count(1) != 0);
}

TEST(CDirectedGraph, SaveAsDotStream)
{
  graph_t g;
  g.insertEdge(0, 1, 1.0);
  g.insertEdge(1, 2, 2.0);

  std::ostringstream ss;
  TGraphvizExportParams p;
  p.mark_edges_as_not_constraint = true;
  p.node_names[0] = "start";
  EXPECT_TRUE(g.saveAsDot(ss, p));

  const std::string s = ss.str();
  EXPECT_NE(s.find("digraph G"), std::string::npos);
  EXPECT_NE(s.find("start"), std::string::npos);
  EXPECT_NE(s.find("constraint=false"), std::string::npos);
}

TEST(CDirectedGraph, SaveAsDotFile)
{
  graph_t g;
  g.insertEdge(0, 1, 1.0);

  const std::string fil = "test_CDirectedGraph_dot_out.dot";
  EXPECT_TRUE(g.saveAsDot(fil));

  std::ifstream f(fil);
  ASSERT_TRUE(f.is_open());
  std::stringstream buffer;
  buffer << f.rdbuf();
  EXPECT_NE(buffer.str().find("digraph G"), std::string::npos);
  f.close();
  std::remove(fil.c_str());
}

TEST(CDirectedGraph, SaveAsDotFileInvalidPath)
{
  graph_t g;
  g.insertEdge(0, 1, 1.0);
  EXPECT_FALSE(g.saveAsDot("/nonexistent_dir_xyz/out.dot"));
}

TEST(CDirectedGraph, CopyConstructorFromMultimap)
{
  graph_t::edges_map_t m;
  m.insert(std::make_pair(std::make_pair(TNodeID(0), TNodeID(1)), graph_t::edge_t(1.0)));
  graph_t g(m);
  EXPECT_EQ(g.edgeCount(), 1U);
  EXPECT_TRUE(g.edgeExists(0, 1));
}

TEST(CDirectedGraph, Iterators)
{
  graph_t g;
  g.insertEdge(0, 1, 1.0);
  g.insertEdge(1, 2, 2.0);

  size_t count = 0;
  for (auto it = g.begin(); it != g.end(); ++it) count++;
  EXPECT_EQ(count, 2U);

  const graph_t& cg = g;
  size_t ccount = 0;
  for (auto it = cg.begin(); it != cg.end(); ++it) ccount++;
  EXPECT_EQ(ccount, 2U);
}
