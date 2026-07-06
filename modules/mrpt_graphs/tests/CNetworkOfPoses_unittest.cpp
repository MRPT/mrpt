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
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphs/registerAllClasses.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

#include <cstdio>
#include <sstream>

using namespace mrpt::graphs;
using namespace mrpt::poses;

TEST(CNetworkOfPoses, DefaultConstructionAndClear)
{
  CNetworkOfPoses2D g;
  EXPECT_EQ(g.nodeCount(), 0U);
  EXPECT_EQ(g.root, 0U);
  EXPECT_FALSE(g.edges_store_inverse_poses);

  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(1, 0, 0);
  g.insertEdge(0, 1, CPose2D(1, 0, 0));
  g.root = 1;
  g.edges_store_inverse_poses = true;

  g.clear();
  EXPECT_EQ(g.nodeCount(), 0U);
  EXPECT_EQ(g.edgeCount(), 0U);
  EXPECT_EQ(g.root, 0U);
  EXPECT_FALSE(g.edges_store_inverse_poses);
}

TEST(CNetworkOfPoses, GlobalSquareErrorZeroForConsistentGraph)
{
  CNetworkOfPoses2D g;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(1, 0, 0);
  g.nodes[2] = CPose2D(1, 1, 0);
  g.insertEdge(0, 1, CPose2D(1, 0, 0));
  g.insertEdge(1, 2, CPose2D(0, 1, 0));

  EXPECT_NEAR(g.chi2(), 0.0, 1e-12);
  EXPECT_NEAR(g.getGlobalSquareError(true), 0.0, 1e-12);
  EXPECT_NEAR(g.getEdgeSquareError(0, 1), 0.0, 1e-12);
}

TEST(CNetworkOfPoses, GlobalSquareErrorNonZeroForInconsistentGraph)
{
  CNetworkOfPoses2D g;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(5, 5, 0);  // Inconsistent with the edge below
  g.insertEdge(0, 1, CPose2D(1, 0, 0));

  EXPECT_GT(g.chi2(), 0.0);

  const auto itEdge = g.edges.find(std::make_pair(TNodeID(0), TNodeID(1)));
  ASSERT_NE(itEdge, g.edges.end());
  EXPECT_GT(g.getEdgeSquareError(itEdge), 0.0);
}

TEST(CNetworkOfPoses, GetEdgeSquareErrorThrowsOnMissingEdge)
{
  CNetworkOfPoses2D g;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(1, 0, 0);
  EXPECT_THROW(g.getEdgeSquareError(0, 1), std::exception);
}

TEST(CNetworkOfPoses, DijkstraNodesEstimate)
{
  CNetworkOfPoses2D g;
  g.root = 0;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.insertEdge(0, 1, CPose2D(1, 0, 0));
  g.insertEdge(1, 2, CPose2D(0, 1, 0));

  std::map<TNodeID, size_t> topoDist;
  g.dijkstra_nodes_estimate(std::ref(topoDist));

  ASSERT_EQ(g.nodeCount(), 3U);
  EXPECT_NEAR(g.nodes[0].x(), 0.0, 1e-9);
  EXPECT_NEAR(g.nodes[1].x(), 1.0, 1e-9);
  EXPECT_NEAR(g.nodes[2].x(), 1.0, 1e-9);
  EXPECT_NEAR(g.nodes[2].y(), 1.0, 1e-9);

  EXPECT_EQ(topoDist.at(0), 0U);
  EXPECT_EQ(topoDist.at(1), 1U);
  EXPECT_EQ(topoDist.at(2), 2U);
}

TEST(CNetworkOfPoses, DijkstraNodesEstimateWithInversePoses)
{
  CNetworkOfPoses2D g;
  g.root = 0;
  g.edges_store_inverse_poses = true;
  g.nodes[0] = CPose2D(0, 0, 0);
  // Edge stores the inverse: pose of "0" as seen from "1":
  g.insertEdge(0, 1, CPose2D(-1, 0, 0));

  g.dijkstra_nodes_estimate();
  EXPECT_NEAR(g.nodes[1].x(), 1.0, 1e-9);
}

TEST(CNetworkOfPoses, CollapseDuplicatedEdges)
{
  CNetworkOfPoses2D g;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(1, 0, 0);
  g.insertEdge(0, 1, CPose2D(1, 0, 0));
  g.insertEdge(0, 1, CPose2D(1.1, 0, 0));
  g.insertEdge(1, 0, CPose2D(-0.9, 0, 0));

  EXPECT_EQ(g.edgeCount(), 3U);
  const size_t nRemoved = g.collapseDuplicatedEdges();
  EXPECT_EQ(nRemoved, 2U);
  EXPECT_EQ(g.edgeCount(), 1U);
}

TEST(CNetworkOfPoses, GetAs3DObjectThrows)
{
  CNetworkOfPoses2D g;
  mrpt::containers::yaml params;
  EXPECT_THROW(g.getAs3DObject(mrpt::viz::CSetOfObjects::Create(), params), std::exception);
}

TEST(CNetworkOfPoses, WriteAndReadAsTextRoundTrip)
{
  CNetworkOfPoses2D g;
  g.root = 1;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(2, 3, 0.5);
  g.insertEdge(0, 1, CPose2D(2, 3, 0.5));

  std::stringstream ss;
  g.writeAsText(ss);

  CNetworkOfPoses2D g2;
  g2.readAsText(ss);

  EXPECT_EQ(g2.nodeCount(), 2U);
  EXPECT_EQ(g2.root, 1U);
  EXPECT_EQ(g2.edgeCount(), 1U);
  EXPECT_NEAR(g2.nodes[1].x(), 2.0, 1e-6);
  EXPECT_NEAR(g2.nodes[1].y(), 3.0, 1e-6);
}

TEST(CNetworkOfPoses, SaveAndLoadTextFileRoundTrip)
{
  CNetworkOfPoses2D g;
  g.root = 0;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(1, 0, 0);
  g.nodes[2] = CPose2D(1, 1, 0);
  g.insertEdge(0, 1, CPose2D(1, 0, 0));
  g.insertEdge(1, 2, CPose2D(0, 1, 0));

  const std::string fil = "test_CNetworkOfPoses_out.graph";
  g.saveToTextFile(fil);

  CNetworkOfPoses2D g2;
  g2.loadFromTextFile(fil);

  EXPECT_EQ(g2.nodeCount(), 3U);
  EXPECT_EQ(g2.edgeCount(), 2U);
  EXPECT_EQ(g2.root, 0U);

  std::remove(fil.c_str());
}

TEST(CNetworkOfPoses, SaveToTextFileInvalidPathThrows)
{
  CNetworkOfPoses2D g;
  g.nodes[0] = CPose2D(0, 0, 0);
  EXPECT_THROW(g.saveToTextFile("/nonexistent_dir_xyz/out.graph"), std::exception);
}

TEST(CNetworkOfPoses, LoadFromTextFileMissingFileThrows)
{
  CNetworkOfPoses2D g;
  EXPECT_THROW(g.loadFromTextFile("/nonexistent_dir_xyz/missing.graph"), std::exception);
}

TEST(CNetworkOfPoses, LoadFromTextStreamWithEquivAndFix)
{
  const std::string txt =
      "VERTEX_SE2 0 0 0 0\n"
      "VERTEX_SE2 1 1 0 0\n"
      "VERTEX_SE2 2 2 0 0\n"
      "EQUIV 2 1\n"
      "EDGE_SE2 0 1 1 0 0 1 0 0 1 0 1\n"
      "FIX 0\n";
  std::istringstream ss(txt);

  CNetworkOfPoses2D g;
  g.readAsText(ss);

  // Node 2 got merged into node 1 via EQUIV, so only 2 nodes remain:
  EXPECT_EQ(g.nodeCount(), 2U);
  EXPECT_EQ(g.root, 0U);
}

TEST(CNetworkOfPoses, LoadFromTextStreamRejectsVertex3In2DGraph)
{
  const std::string txt = "VERTEX3 0 0 0 0 0 0 0\n";
  std::istringstream ss(txt);
  CNetworkOfPoses2D g;
  EXPECT_THROW(g.readAsText(ss), std::exception);
}

TEST(CNetworkOfPoses, LoadFromTextStreamDuplicatedVertexThrows)
{
  const std::string txt =
      "VERTEX_SE2 0 0 0 0\n"
      "VERTEX_SE2 0 1 1 0\n";
  std::istringstream ss(txt);
  CNetworkOfPoses2D g;
  EXPECT_THROW(g.readAsText(ss), std::exception);
}

TEST(CNetworkOfPoses, ExtractSubGraph)
{
  CNetworkOfPoses2D g;
  for (int i = 0; i <= 4; i++) g.nodes[i] = CPose2D(i, 0, 0);
  for (int i = 0; i < 4; i++) g.insertEdge(i, i + 1, CPose2D(1, 0, 0));
  g.root = 0;

  CNetworkOfPoses2D sub;
  std::set<TNodeID> nodeIDs{0, 1, 2};
  g.extractSubGraph(nodeIDs, &sub);

  EXPECT_EQ(sub.nodeCount(), 3U);
  EXPECT_EQ(sub.edgeCount(), 2U);
}

TEST(CNetworkOfPoses, ExtractSubGraphNonConsecutiveAutoExpand)
{
  CNetworkOfPoses2D g;
  for (int i = 0; i <= 4; i++) g.nodes[i] = CPose2D(i, 0, 0);
  for (int i = 0; i < 4; i++) g.insertEdge(i, i + 1, CPose2D(1, 0, 0));
  g.root = 0;

  CNetworkOfPoses2D sub;
  // Non-consecutive node IDs, with the default auto_expand_set=true: the gap
  // must be auto-filled with the in-between nodes:
  std::set<TNodeID> nodeIDs{0, 2, 4};
  g.extractSubGraph(nodeIDs, &sub);

  EXPECT_EQ(sub.nodeCount(), 5U);
  EXPECT_EQ(sub.edgeCount(), 4U);
}

TEST(CNetworkOfPoses, ExtractSubGraphRootWithNoNeighborsIsStitched)
{
  CNetworkOfPoses2D g;
  for (int i = 0; i <= 4; i++) g.nodes[i] = CPose2D(i, 0, 0);
  for (int i = 0; i < 4; i++) g.insertEdge(i, i + 1, CPose2D(1, 0, 0));
  g.root = 0;

  CNetworkOfPoses2D sub;
  // {0,2}: neither is adjacent to the other in the original graph, and the
  // root (0) ends up with zero neighbors in the extracted subgraph, forcing
  // the "root has no neighbors" virtual-edge stitch:
  std::set<TNodeID> nodeIDs{0, 2};
  g.extractSubGraph(nodeIDs, &sub, /*root_node_in=*/INVALID_NODEID, /*auto_expand_set=*/false);

  EXPECT_EQ(sub.nodeCount(), 2U);
  EXPECT_EQ(sub.edgeCount(), 1U);
  EXPECT_EQ(sub.root, 0U);
  EXPECT_TRUE(sub.edgeExists(0, 2));
}

TEST(CNetworkOfPoses, ExtractSubGraphVirtualEdgeSingleIsland)
{
  CNetworkOfPoses2D g;
  for (int i = 0; i <= 4; i++) g.nodes[i] = CPose2D(i, 0, 0);
  for (int i = 0; i < 4; i++) g.insertEdge(i, i + 1, CPose2D(1, 0, 0));
  g.root = 0;

  CNetworkOfPoses2D sub;
  // {0,1,3,4}: the root already has a real neighbor (1), so the "root has no
  // neighbors" stitch is skipped, but {3,4} remains a single disconnected
  // island (each already internally connected via a real edge) that must be
  // stitched via the NotConnectedGraph-catch / connectGraphPartitions path:
  std::set<TNodeID> nodeIDs{0, 1, 3, 4};
  g.extractSubGraph(nodeIDs, &sub, /*root_node_in=*/INVALID_NODEID, /*auto_expand_set=*/false);

  EXPECT_EQ(sub.nodeCount(), 4U);
  EXPECT_EQ(sub.edgeCount(), 3U);
  EXPECT_EQ(sub.root, 0U);
  EXPECT_TRUE(sub.edgeExists(0, 1));
  EXPECT_TRUE(sub.edgeExists(3, 4));
}

TEST(CNetworkOfPoses, ExtractSubGraphVirtualEdgeMultipleIslands)
{
  CNetworkOfPoses2D g;
  for (int i = 0; i <= 7; i++) g.nodes[i] = CPose2D(i, 0, 0);
  for (int i = 0; i < 7; i++) g.insertEdge(i, i + 1, CPose2D(1, 0, 0));
  g.root = 0;

  CNetworkOfPoses2D sub;
  // {0,1,3,4,6,7}: three internally-connected pairs {0,1},{3,4},{6,7} with
  // non-contiguous gaps, forcing the "multiple islands" branch to stitch
  // more than once across successive loop iterations:
  std::set<TNodeID> nodeIDs{0, 1, 3, 4, 6, 7};
  g.extractSubGraph(nodeIDs, &sub, /*root_node_in=*/INVALID_NODEID, /*auto_expand_set=*/false);

  EXPECT_EQ(sub.nodeCount(), 6U);
  EXPECT_EQ(sub.edgeCount(), 5U);
  EXPECT_EQ(sub.root, 0U);
  // The resulting graph must be fully connected (every node has >=1
  // neighbor):
  for (const auto& n : sub.getAllNodes()) EXPECT_GE(sub.getNeighborsOf(n).size(), 1U);
}

TEST(CNetworkOfPoses, GlobalPoseOperators)
{
  CNetworkOfPoses2D g;
  g.nodes[0] = CPose2D(1, 2, 0.5);
  g.nodes[1] = CPose2D(1, 2, 0.5);
  g.nodes[2] = CPose2D(3, 4, 0.1);

  EXPECT_TRUE(g.nodes[0] == g.nodes[1]);
  EXPECT_FALSE(g.nodes[0] == g.nodes[2]);
  EXPECT_TRUE(g.nodes[0] != g.nodes[2]);

  std::ostringstream ss;
  ss << g.nodes[0];
  EXPECT_FALSE(ss.str().empty());
}

TEST(CNetworkOfPoses, RegisterAllClasses)
{
  // Just exercise the (idempotent) class registration entry point:
  mrpt::graphs::registerAllClasses_mrpt_graphs();
  SUCCEED();
}

TEST(CNetworkOfPoses, CovarianceEdgeTypesTextRoundTrip)
{
  CNetworkOfPoses2DCov g2;
  g2.nodes[0] = CPose2D(0, 0, 0);
  g2.nodes[1] = CPose2D(1, 0, 0);
  CPosePDFGaussian edge2;
  edge2.mean = CPose2D(1, 0, 0);
  edge2.cov.setIdentity();
  g2.insertEdge(0, 1, edge2);

  std::stringstream ss2;
  g2.writeAsText(ss2);
  EXPECT_NE(ss2.str().find("EDGE_SE2"), std::string::npos);

  CNetworkOfPoses3DCov g3;
  g3.nodes[0] = CPose3D(0, 0, 0, 0, 0, 0);
  g3.nodes[1] = CPose3D(1, 0, 0, 0, 0, 0);
  CPose3DPDFGaussian edge3;
  edge3.mean = CPose3D(1, 0, 0, 0, 0, 0);
  edge3.cov.setIdentity();
  g3.insertEdge(0, 1, edge3);

  std::stringstream ss3;
  g3.writeAsText(ss3);
  EXPECT_NE(ss3.str().find("EDGE3"), std::string::npos);
}

TEST(CNetworkOfPoses, BinarySerializationRoundTrip)
{
  CNetworkOfPoses2D g;
  g.root = 1;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(2, 3, 0.5);
  g.insertEdge(0, 1, CPose2D(2, 3, 0.5));

  mrpt::io::CMemoryStream mem;
  auto arch = mrpt::serialization::archiveFrom(mem);
  arch << g;

  mem.Seek(0);
  CNetworkOfPoses2D g2;
  arch >> g2;

  EXPECT_EQ(g2.nodeCount(), 2U);
  EXPECT_EQ(g2.edgeCount(), 1U);
  EXPECT_EQ(g2.root, 1U);
}

TEST(CNetworkOfPoses, ThreeDGraphVertex3RoundTrip)
{
  CNetworkOfPoses3D g;
  g.root = 0;
  g.nodes[0] = CPose3D(0, 0, 0, 0, 0, 0);
  g.nodes[1] = CPose3D(1, 0, 0, 0, 0, 0);
  g.insertEdge(0, 1, CPose3D(1, 0, 0, 0, 0, 0));

  std::stringstream ss;
  g.writeAsText(ss);

  CNetworkOfPoses3D g2;
  g2.readAsText(ss);
  EXPECT_EQ(g2.nodeCount(), 2U);
  EXPECT_NEAR(g2.nodes[1].x(), 1.0, 1e-6);
}

TEST(CNetworkOfPoses, LoadEdge3IntoTwoDGraphThrows)
{
  const std::string txt =
      "VERTEX_SE2 0 0 0 0\n"
      "VERTEX_SE2 1 1 0 0\n"
      "EDGE3 0 1 1 0 0 0 0 0\n";
  std::istringstream ss(txt);
  CNetworkOfPoses2D g;
  EXPECT_THROW(g.readAsText(ss), std::exception);
}

TEST(CNetworkOfPoses, InformationMatrixGraphChi2)
{
  CNetworkOfPoses2DInf g;
  g.nodes[0] = CPose2D(0, 0, 0);
  g.nodes[1] = CPose2D(1, 0, 0);

  CPosePDFGaussianInf edge;
  edge.mean = CPose2D(1, 0, 0);
  edge.cov_inv.setIdentity();
  g.insertEdge(0, 1, edge);

  EXPECT_NEAR(g.chi2(), 0.0, 1e-9);
  EXPECT_NEAR(g.getGlobalSquareError(/*ignoreCovariances=*/false), 0.0, 1e-9);
}
