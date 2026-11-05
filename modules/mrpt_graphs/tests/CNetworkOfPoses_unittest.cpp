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
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random.h>
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

// This file intentionally does not include Eigen, to check that the public
// graph headers can be used without it.
TEST(CNetworkOfPoses, Chi2WithCovariancesAllPdfTypes)
{
  // The "from" node is at the origin, so err = (edge mean) - (to node), with
  // a translation error of (0.5, 0.2, -0.1) in all cases.
  {
    CNetworkOfPoses2DCov g;
    g.nodes[0] = CPose2D(0, 0, 0);
    g.nodes[1] = CPose2D(0.5, 0.2, 0);
    CPosePDFGaussian edge;
    edge.mean = CPose2D(1.0, 0.4, 0);
    edge.cov.setDiagonal(std::vector<double>{0.25, 0.5, 1.0});
    g.insertEdge(0, 1, edge);
    EXPECT_NEAR(g.chi2(), 1.08, 1e-9);
  }
  {
    CNetworkOfPoses2DInf g;
    g.nodes[0] = CPose2D(0, 0, 0);
    g.nodes[1] = CPose2D(0.5, 0.2, 0);
    CPosePDFGaussianInf edge;
    edge.mean = CPose2D(1.0, 0.4, 0);
    edge.cov_inv.setIdentity();
    edge.cov_inv(0, 0) = 4.0;
    edge.cov_inv(0, 1) = 1.0;
    edge.cov_inv(1, 0) = 1.0;
    edge.cov_inv(1, 1) = 2.0;
    g.insertEdge(0, 1, edge);
    EXPECT_NEAR(g.chi2(), 1.28, 1e-9);
  }
  {
    CNetworkOfPoses3DCov g;
    g.nodes[0] = CPose3D();
    g.nodes[1] = CPose3D(0.5, 0.2, -0.1, 0, 0, 0);
    CPose3DPDFGaussian edge;
    edge.mean = CPose3D(1.0, 0.4, -0.2, 0, 0, 0);
    edge.cov.setDiagonal(std::vector<double>{0.25, 0.5, 1.0, 1.0, 1.0, 1.0});
    g.insertEdge(0, 1, edge);
    EXPECT_NEAR(g.chi2(), 1.09, 1e-9);
  }
  {
    CNetworkOfPoses3DInf g;
    g.nodes[0] = CPose3D();
    g.nodes[1] = CPose3D(0.5, 0.2, -0.1, 0, 0, 0);
    CPose3DPDFGaussianInf edge;
    edge.mean = CPose3D(1.0, 0.4, -0.2, 0, 0, 0);
    edge.cov_inv.setIdentity();
    edge.cov_inv(0, 0) = 4.0;
    edge.cov_inv(0, 1) = 1.0;
    edge.cov_inv(1, 0) = 1.0;
    edge.cov_inv(1, 1) = 2.0;
    g.insertEdge(0, 1, edge);
    EXPECT_NEAR(g.chi2(), 1.29, 1e-9);
  }
}

namespace
{
// Finite-difference Jacobian of the pose parameters of Z (+) exp(e) wrt e.
template <class SE>
typename SE::matrix_TxT numericJacobPoseParamsWrtLocal(const typename SE::type& Z)
{
  typename SE::matrix_TxT J;
  const double h = 1e-6;
  for (size_t k = 0; k < SE::DOFs; k++)
  {
    typename SE::tangent_vector ep;
    ep.setZero();
    ep[k] = h;
    typename SE::tangent_vector em;
    em.setZero();
    em[k] = -h;
    const auto pp = (Z + SE::exp(ep)).asVectorVal();
    const auto pm = (Z + SE::exp(em)).asVectorVal();
    for (size_t r = 0; r < SE::DOFs; r++)
    {
      double d = pp[r] - pm[r];
      if (r >= SE::DOFs - (SE::DOFs == 3 ? 1 : 3))
      {
        d = mrpt::math::wrapToPi(d);
      }
      J(r, k) = d / (2 * h);
    }
  }
  return J;
}
}  // namespace

TEST(CNetworkOfPoses, EdgeInformationJacobiansMatchFiniteDifferences)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);
  for (int t = 0; t < 200; t++)
  {
    const CPose2D Z2(rng.drawUniform(-5, 5), rng.drawUniform(-5, 5), rng.drawUniform(-3, 3));
    const auto J2 = mrpt::graphs::detail::jacobPoseParamsWrtLocal(Z2);
    const auto J2fd = numericJacobPoseParamsWrtLocal<Lie::SE<2>>(Z2);
    const auto J2i = mrpt::graphs::detail::jacobLocalWrtPoseParams(Z2);
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        EXPECT_NEAR(J2(r, c), J2fd(r, c), 1e-6);
        double s = 0;
        for (int k = 0; k < 3; k++)
        {
          s += J2(r, k) * J2i(k, c);
        }
        EXPECT_NEAR(s, r == c ? 1.0 : 0.0, 1e-12);
      }
    }

    const CPose3D Z3(
        rng.drawUniform(-5, 5), rng.drawUniform(-5, 5), rng.drawUniform(-5, 5),
        rng.drawUniform(-3, 3), rng.drawUniform(-1.4, 1.4), rng.drawUniform(-3, 3));
    const auto J3 = mrpt::graphs::detail::jacobPoseParamsWrtLocal(Z3);
    const auto J3fd = numericJacobPoseParamsWrtLocal<Lie::SE<3>>(Z3);
    const auto J3i = mrpt::graphs::detail::jacobLocalWrtPoseParams(Z3);
    for (int r = 0; r < 6; r++)
    {
      for (int c = 0; c < 6; c++)
      {
        EXPECT_NEAR(J3(r, c), J3fd(r, c), 1e-5);
        double s = 0;
        for (int k = 0; k < 6; k++)
        {
          s += J3(r, k) * J3i(k, c);
        }
        EXPECT_NEAR(s, r == c ? 1.0 : 0.0, 1e-9);
      }
    }
  }
}

// TORO EDGE2 lines list the information as xx xy yy pp xp yp
TEST(CNetworkOfPoses, LoadToroEdge2InformationOrder)
{
  std::stringstream ss;
  ss << "VERTEX2 0 0 0 0\nVERTEX2 1 1 0 0\n";
  ss << "EDGE2 0 1 1 0 0 10 1 20 30 2 3\n";
  CNetworkOfPoses2DInf g;
  g.readAsText(ss);
  ASSERT_EQ(g.edgeCount(), 1U);
  const auto& I = g.edges.begin()->second.cov_inv;
  EXPECT_DOUBLE_EQ(I(0, 0), 10);
  EXPECT_DOUBLE_EQ(I(0, 1), 1);
  EXPECT_DOUBLE_EQ(I(1, 1), 20);
  EXPECT_DOUBLE_EQ(I(2, 2), 30);
  EXPECT_DOUBLE_EQ(I(0, 2), 2);
  EXPECT_DOUBLE_EQ(I(1, 2), 3);
  EXPECT_DOUBLE_EQ(I(2, 0), 2);
}

// g2o EDGE_SE2 information refers to the residual in the frame of the edge
// end, MRPT edge PDFs to the frame of the edge origin
TEST(CNetworkOfPoses, LoadSaveG2oEdgeSE2InformationFrame)
{
  std::stringstream ss;
  ss << "VERTEX_SE2 0 0 0 0\nVERTEX_SE2 1 1 0 1.5707963267948966\n";
  ss << "EDGE_SE2 0 1 1 0 1.5707963267948966 100 0 0 1 0 5\n";
  CNetworkOfPoses2DInf g;
  g.readAsText(ss);
  ASSERT_EQ(g.edgeCount(), 1U);
  const auto& I = g.edges.begin()->second.cov_inv;
  // Local x (precise) is the global y for a 90 deg edge:
  EXPECT_NEAR(I(0, 0), 1, 1e-9);
  EXPECT_NEAR(I(1, 1), 100, 1e-9);
  EXPECT_NEAR(I(0, 1), 0, 1e-9);
  EXPECT_NEAR(I(2, 2), 5, 1e-9);

  // Save and load again: the information must be unchanged:
  std::stringstream ss2;
  g.writeAsText(ss2);
  CNetworkOfPoses2DInf g2;
  g2.readAsText(ss2);
  ASSERT_EQ(g2.edgeCount(), 1U);
  const auto& I2 = g2.edges.begin()->second.cov_inv;
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      EXPECT_NEAR(I2(r, c), I(r, c), 1e-9);
    }
  }
  EXPECT_NE(ss2.str().find("EDGE_SE2 0 1 1 0 1.5707963267948966 100 "), std::string::npos)
      << ss2.str();
}

// The squared Mahalanobis distance of a small local perturbation must be the
// same using g2o's EDGE_SE3:QUAT information on [t,qx,qy,qz] and the loaded
// MRPT information on the (x,y,z,yaw,pitch,roll) parameters
TEST(CNetworkOfPoses, LoadG2oEdgeSE3QuatInformation)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(4321);
  const CPose3D Z(0.5, -0.3, 0.2, 0.7, -0.4, 1.1);
  // Random positive definite g2o information:
  const auto L = rng.drawDefinitePositiveMatrix<
      mrpt::math::CMatrixDouble66, mrpt::math::CVectorFixed<double, 6>>(6, 3.0, 1.0);
  const CPose3DQuat q(Z);
  std::stringstream ss;
  ss.precision(17);
  ss << "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\nVERTEX_SE3:QUAT 1 0 0 0 0 0 0 1\n";
  ss << "EDGE_SE3:QUAT 0 1 " << q.x() << " " << q.y() << " " << q.z() << " " << q.quat().x() << " "
     << q.quat().y() << " " << q.quat().z() << " " << q.quat().r();
  for (int r = 0; r < 6; r++)
  {
    for (int c = r; c < 6; c++)
    {
      ss << " " << L(r, c);
    }
  }
  ss << "\n";
  CNetworkOfPoses3DInf g;
  g.readAsText(ss);
  ASSERT_EQ(g.edgeCount(), 1U);
  const auto& edge = g.edges.begin()->second;
  EXPECT_NEAR((edge.mean.asVectorVal() - Z.asVectorVal()).norm(), 0, 1e-9);

  for (int t = 0; t < 20; t++)
  {
    Lie::SE<3>::tangent_vector e;
    for (int i = 0; i < 6; i++)
    {
      e[i] = rng.drawUniform(-1e-4, 1e-4);
    }
    const CPose3D X = Z + Lie::SE<3>::exp(e);
    // g2o residual of X wrt Z:
    const CPose3DQuat dq(X - Z);
    const double sgn = dq.quat().r() < 0 ? -1.0 : 1.0;
    const double eg[6] = {
        dq.x(), dq.y(), dq.z(), sgn * dq.quat().x(), sgn * dq.quat().y(), sgn * dq.quat().z()};
    double d2g2o = 0;
    for (int r = 0; r < 6; r++)
    {
      for (int c = 0; c < 6; c++)
      {
        d2g2o += eg[r] * L(r, c) * eg[c];
      }
    }
    // MRPT: difference of pose parameters
    auto dp = X.asVectorVal() - Z.asVectorVal();
    for (int i = 3; i < 6; i++)
    {
      dp[i] = mrpt::math::wrapToPi(dp[i]);
    }
    double d2mrpt = 0;
    for (int r = 0; r < 6; r++)
    {
      for (int c = 0; c < 6; c++)
      {
        d2mrpt += dp[r] * edge.cov_inv(r, c) * dp[c];
      }
    }
    EXPECT_NEAR(d2mrpt, d2g2o, 1e-3 * d2g2o);
  }
}

// EDGE3 rotational cross terms must survive a save/load round trip
TEST(CNetworkOfPoses, Edge3FullInformationTextRoundTrip)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(77);
  CNetworkOfPoses3DInf g;
  g.nodes[0] = CPose3D();
  g.nodes[1] = CPose3D(1, 0, 0, 0.1, 0.2, 0.3);
  const auto I = rng.drawDefinitePositiveMatrix<
      mrpt::math::CMatrixDouble66, mrpt::math::CVectorFixed<double, 6>>(6, 3.0, 1.0);
  g.insertEdge(0, 1, CPose3DPDFGaussianInf(CPose3D(1, 0, 0, 0.1, 0.2, 0.3), I));
  std::stringstream ss;
  g.writeAsText(ss);
  CNetworkOfPoses3DInf g2;
  g2.readAsText(ss);
  ASSERT_EQ(g2.edgeCount(), 1U);
  const auto& I2 = g2.edges.begin()->second.cov_inv;
  for (int r = 0; r < 6; r++)
  {
    for (int c = 0; c < 6; c++)
    {
      EXPECT_NEAR(I2(r, c), I(r, c), 1e-9 * (1 + std::abs(I(r, c)))) << "r=" << r << " c=" << c;
    }
  }
}

// Text output must not lose precision
TEST(CNetworkOfPoses, TextRoundTripIsLossless)
{
  CNetworkOfPoses2D g;
  g.nodes[0] = CPose2D(123456.789012345, -98765.4321098, 0.123456789012345);
  g.nodes[1] = CPose2D(123457.789012345, -98765.4321098, 0.123456789012345);
  g.insertEdge(0, 1, CPose2D(1.000000000123, 1e-9, 1e-10));
  std::stringstream ss;
  g.writeAsText(ss);
  CNetworkOfPoses2D g2;
  g2.readAsText(ss);
  EXPECT_DOUBLE_EQ(g2.nodes[0].x(), g.nodes[0].x());
  EXPECT_DOUBLE_EQ(g2.nodes[0].y(), g.nodes[0].y());
  EXPECT_DOUBLE_EQ(g2.nodes[0].phi(), g.nodes[0].phi());
  EXPECT_DOUBLE_EQ(g2.edges.begin()->second.x(), 1.000000000123);
}

// Graphs with edges_store_inverse_poses: errors and text output must honor it
TEST(CNetworkOfPoses, InverseEdgesSquareErrorAndTextOutput)
{
  const CPose2D p1(1, 2, 0.5);
  const CPose2D p2(3, 1, -1.0);
  CNetworkOfPoses2D g;
  g.edges_store_inverse_poses = true;
  g.nodes[0] = CPose2D();
  g.nodes[1] = p1;
  g.nodes[2] = p2;
  // Edge i->j holds i (-) j:
  g.insertEdge(0, 1, CPose2D() - p1);
  g.insertEdge(1, 2, p1 - p2);
  EXPECT_NEAR(g.getGlobalSquareError(), 0, 1e-12);
  EXPECT_NEAR(g.getEdgeSquareError(1, 2), 0, 1e-12);

  std::stringstream ss;
  g.writeAsText(ss);
  CNetworkOfPoses2D g2;
  g2.readAsText(ss);
  EXPECT_FALSE(g2.edges_store_inverse_poses);
  EXPECT_NEAR(g2.getGlobalSquareError(), 0, 1e-12);
  g2.dijkstra_nodes_estimate();
  EXPECT_NEAR((g2.nodes[2] - p2).norm(), 0, 1e-12);
  EXPECT_NEAR(mrpt::math::wrapToPi(g2.nodes[2].phi() - p2.phi()), 0, 1e-12);
}

// Chi2 with covariances is invariant to the pose of the edge origin node
TEST(CNetworkOfPoses, Chi2InvariantToGlobalFrame)
{
  mrpt::math::CMatrixDouble33 I;
  I(0, 0) = 100;
  I(1, 1) = 1;
  I(2, 2) = 10;
  I(0, 1) = I(1, 0) = 3;
  const CPose2D Z(1, 0.5, 0.8);
  const CPose2D err(0.01, -0.02, 0.005);
  double chi2Ref = -1;
  for (double phi0 : {0.0, 1.0, -2.5})
  {
    CNetworkOfPoses2DInf g;
    const CPose2D P0(3, -1, phi0);
    g.nodes[0] = P0;
    g.nodes[1] = P0 + Z + err;
    g.insertEdge(0, 1, CPosePDFGaussianInf(Z, I));
    const double c = g.chi2();
    if (chi2Ref < 0)
    {
      chi2Ref = c;
    }
    EXPECT_NEAR(c, chi2Ref, 1e-9 * chi2Ref);
  }
  EXPECT_GT(chi2Ref, 0);
}
