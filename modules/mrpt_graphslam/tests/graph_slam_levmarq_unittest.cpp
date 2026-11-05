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
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

#include <array>

#include "graph_slam_levmarq_test_common.h"

using namespace mrpt;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace std;

// Define in/out files for testing:
using in_out_filenames = std::set<std::tuple<std::string, std::string>>;
const std::map<std::string, in_out_filenames> inout_graph_files{
    {   "GraphTester2D",
     {{"graphslam_SE2_in.graph", "graphslam_SE2_out_good.graph"},
     {"graphslam_SE2_in2.graph", "graphslam_SE2_out_good2.graph"},
     {"graphslam_SE2_in3.graph", "graphslam_SE2_out_good3.graph"}}    },
    {"GraphTester2DInf",
     {{"graphslam_SE2_in.graph", "graphslam_SE2_out_good.graph"},
     {"graphslam_SE2pdf_in.graph", "graphslam_SE2pdf_out_good.graph"}}},
    {   "GraphTester3D",
     {{"graphslam_SE3_in_torus3D-first100.graph", "graphslam_SE3_out_good_torus3D-first100.graph"},
     {"graphslam_SE2_in2.graph", "graphslam_SE2_out_good2.graph"}}    }
};

template <class my_graph_t>
class GraphTester : public GraphSlamLevMarqTest<my_graph_t>, public ::testing::Test
{
 protected:
  void SetUp() override {}
  void TearDown() override {}
  void test_ring_path([[maybe_unused]] const char* className)
  {
    // This is the initial input graph (make a copy for later use):
    my_graph_t graph;
    GraphSlamLevMarqTest<my_graph_t>::create_ring_path(graph);

    const my_graph_t graph_initial = graph;

    // ----------------------------
    //  Run graph slam:
    // ----------------------------
    mrpt::containers::yaml params;
    // params["verbose"] = true;
    params["max_iterations"] = 100;

    graphslam::TResultInfoSpaLevMarq levmarq_info;

    graphslam::optimize_graph_spa_levmarq(graph, levmarq_info, nullptr, params);

    const double err_init = graph_initial.chi2();
    const double err_end = graph.chi2();
    std::cout << "err_init: " << err_init << "\n";
    std::cout << "err_end: " << err_end << "\n";
    //		graph_initial.saveToTextFile(
    //			string("in_") + string(className) + string(".graph"));

    // Do some basic checks on the results:
    EXPECT_GE(levmarq_info.num_iters, 2U);
    EXPECT_LE(levmarq_info.final_total_sq_error, 5e-2);
    EXPECT_LT(err_end, err_init);

  }  // end test_ring_path

  void compare_two_graphs(
      const my_graph_t& g1,
      const my_graph_t& g2,
      const double eps_node_pos = 1e-3,
      const double eps_edges = 1e-3)
  {
    EXPECT_EQ(g1.edges.size(), g2.edges.size());
    EXPECT_EQ(g1.nodes.size(), g2.nodes.size());
    EXPECT_EQ(g1.root, g2.root);

    if (g1.edges.size() != g2.edges.size() || g1.nodes.size() != g2.nodes.size())
    {
      return;
    }
    // Check that the edge values are OK:
    {
      typename my_graph_t::const_iterator it1, it2;
      for (it1 = g1.edges.begin(), it2 = g2.edges.begin(); it1 != g1.edges.end(); ++it1, ++it2)
      {
        EXPECT_EQ(it1->first, it2->first);
        EXPECT_NEAR(
            0,
            (it1->second.getPoseMean().asVectorVal() - it2->second.getPoseMean().asVectorVal())
                .sum_abs(),
            eps_edges);
      }
    }

    // Check nodes:
    {
      auto itn1 = g1.nodes.cbegin(), itn2 = g2.nodes.cbegin();
      for (; itn1 != g1.nodes.cend(); ++itn1, ++itn2)
      {
        EXPECT_EQ(itn1->first, itn2->first);
        EXPECT_NEAR(
            0, (itn1->second.asVectorVal() - itn2->second.asVectorVal()).sum_abs(), eps_node_pos)
            << "Poses of keyframe #" << itn1->first << " do not match:"
            << "\n"
            << "- Expected: " << itn2->second << "\n"
            << "- Got     : " << itn1->second << "\n";
      }
    }
  }

  void test_graph_text_serialization()
  {
    my_graph_t graph;
    GraphSlamLevMarqTest<my_graph_t>::create_ring_path(graph);
    // text write:
    std::stringstream ss;
    graph.writeAsText(ss);
    // read:
    my_graph_t read_graph;
    ss.seekg(0);  // rewind
    read_graph.readAsText(ss);

    compare_two_graphs(graph, read_graph);
  }

  void test_graph_bin_serialization()
  {
    my_graph_t graph;
    GraphSlamLevMarqTest<my_graph_t>::create_ring_path(graph);
    // binary write:
    mrpt::io::CMemoryStream mem;
    auto arch = mrpt::serialization::archiveFrom(mem);
    arch << graph;
    // read:
    my_graph_t read_graph;
    mem.Seek(0);
    arch >> read_graph;

    compare_two_graphs(graph, read_graph);
  }

  void test_optimize_compare_known_solution(const char* type)
  {
    auto files_it = inout_graph_files.find(type);
    if (files_it == inout_graph_files.end()) return;  // No tests for this type

    const string prefix = mrpt::mrpt_data_dir() + string("/tests/");
    for (const auto& tst : files_it->second)
    {
      std::cout << "Testing graph type `" << type << "`, in_file=`" << std::get<0>(tst) << "`"
                << "\n";

      const string in_f = prefix + std::get<0>(tst);
      ASSERT_FILE_EXISTS_(in_f);
      const string good_f = prefix + std::get<1>(tst);
      ASSERT_FILE_EXISTS_(good_f);

      my_graph_t graph, graph_good;
      graph.loadFromTextFile(in_f);

      graph_good.loadFromTextFile(good_f);

      ASSERT_(graph.nodeCount() > 1);
      ASSERT_EQ(graph.nodeCount(), graph_good.nodeCount());
      ASSERT_EQ(graph.edgeCount(), graph_good.edgeCount());

      // Optimize:
      const my_graph_t graph_initial = graph;
      mrpt::containers::yaml params;
      params["max_iterations"] = 100;

      graphslam::TResultInfoSpaLevMarq levmarq_info;

      graphslam::optimize_graph_spa_levmarq(graph, levmarq_info, nullptr, params);

      /* DEBUG */
      const double err_init = graph_initial.chi2();
      const double err_end = graph.chi2();
      const double err_good = graph_good.chi2();
      std::cout << "err_init: " << err_init << "\n";
      std::cout << "err_end: " << err_end << "\n";
      std::cout << "err_good: " << err_good << "\n";

      // Do some basic checks on the results:
      EXPECT_GE(levmarq_info.num_iters, 2U);
      EXPECT_LE(levmarq_info.final_total_sq_error, 0.2);
      EXPECT_LT(err_end, err_init);

      // Compare to good solution:
      compare_two_graphs(graph, graph_good);
    }
  }
};

using GraphTester2D = GraphTester<CNetworkOfPoses2D>;
using GraphTester3D = GraphTester<CNetworkOfPoses3D>;
using GraphTester2DInf = GraphTester<CNetworkOfPoses2DInf>;
using GraphTester3DInf = GraphTester<CNetworkOfPoses3DInf>;

#define GRAPHS_TESTS(_TYPE)                 \
  TEST_F(_TYPE, OptimizeSampleRingPath)     \
  {                                         \
    for (int seed = 1; seed <= 3; seed++)   \
    {                                       \
      getRandomGenerator().randomize(seed); \
      test_ring_path(#_TYPE);               \
    }                                       \
  }                                         \
  TEST_F(_TYPE, BinarySerialization)        \
  {                                         \
    getRandomGenerator().randomize(123);    \
    test_graph_bin_serialization();         \
  }                                         \
  TEST_F(_TYPE, WriteReadTextFile)          \
  {                                         \
    getRandomGenerator().randomize(123);    \
    test_graph_text_serialization();        \
  }                                         \
  TEST_F(_TYPE, OptimizeCompareKnownSolution) { test_optimize_compare_known_solution(#_TYPE); }

GRAPHS_TESTS(GraphTester2D)
GRAPHS_TESTS(GraphTester3D)
GRAPHS_TESTS(GraphTester2DInf)
GRAPHS_TESTS(GraphTester3DInf)

namespace
{
mrpt::math::CMatrixDouble33 diag3(double a, double b, double c)
{
  mrpt::math::CMatrixDouble33 m;
  m(0, 0) = a;
  m(1, 1) = b;
  m(2, 2) = c;
  return m;
}
mrpt::math::CMatrixDouble66 diag6(const std::array<double, 6>& d)
{
  mrpt::math::CMatrixDouble66 m;
  for (int i = 0; i < 6; i++)
  {
    m(i, i) = d[i];
  }
  return m;
}
}  // namespace

// Edge information is given for the (x,y,phi) edge parameters, with the
// translation in the frame of the edge origin: two contradictory edges with
// a 90 deg rotation and complementary precise axes.
TEST(GraphSlamLevMarq, InformationIn2DEdgeOriginFrame)
{
  CNetworkOfPoses2DInf g;
  g.root = 0;
  g.nodes[0] = CPose2D();
  g.nodes[1] = CPose2D(0.3, 0.2, 1.4);
  g.insertEdge(0, 1, CPosePDFGaussianInf(CPose2D(1, 0, M_PI / 2), diag3(1e4, 1, 1)));
  g.insertEdge(0, 1, CPosePDFGaussianInf(CPose2D(0, 1, M_PI / 2), diag3(1, 1e4, 1)));

  graphslam::TResultInfoSpaLevMarq info;
  graphslam::optimize_graph_spa_levmarq(g, info);

  // Exact optimum: weighted mean of each coordinate
  const double expected = 1e4 / (1e4 + 1);
  EXPECT_NEAR(g.nodes[1].x(), expected, 1e-6);
  EXPECT_NEAR(g.nodes[1].y(), expected, 1e-6);
  EXPECT_NEAR(g.nodes[1].phi(), M_PI / 2, 1e-6);
  // The reported error is the weighted one, and agrees with chi2():
  EXPECT_NEAR(info.final_total_sq_error, g.chi2(), 1e-6);
  EXPECT_NEAR(info.final_total_sq_error, 2 * 1e4 / (1e4 + 1), 1e-6);
}

// Steps must be accepted or rejected according to the weighted error
TEST(GraphSlamLevMarq, WeightedErrorDrivesStepAcceptance)
{
  CNetworkOfPoses2DInf g;
  g.root = 0;
  g.nodes[0] = CPose2D();
  // Start at the minimum of the unweighted error:
  g.nodes[1] = CPose2D(0.5, 0.5, 0);
  g.insertEdge(0, 1, CPosePDFGaussianInf(CPose2D(1, 0, 0), diag3(100, 1, 1)));
  g.insertEdge(0, 1, CPosePDFGaussianInf(CPose2D(0, 1, 0), diag3(1, 100, 1)));

  graphslam::TResultInfoSpaLevMarq info;
  graphslam::optimize_graph_spa_levmarq(g, info);
  EXPECT_NEAR(g.nodes[1].x(), 100.0 / 101, 1e-6);
  EXPECT_NEAR(g.nodes[1].y(), 100.0 / 101, 1e-6);
  EXPECT_NEAR(g.nodes[1].phi(), 0, 1e-6);
}

// The yaw/pitch/roll ordering of the edge information must be honored
TEST(GraphSlamLevMarq, InformationIn3DEdgeParameters)
{
  CNetworkOfPoses3DInf g;
  g.root = 0;
  g.nodes[0] = CPose3D();
  g.nodes[1] = CPose3D(0.01, 0.02, 0.03, 0.05, 0.02, 0.05);
  // A yaw-precise edge and a roll-precise edge:
  g.insertEdge(
      0, 1, CPose3DPDFGaussianInf(CPose3D(0, 0, 0, 0.2, 0, 0), diag6({1, 1, 1, 1e4, 1, 1})));
  g.insertEdge(
      0, 1, CPose3DPDFGaussianInf(CPose3D(0, 0, 0, 0, 0, 0.2), diag6({1, 1, 1, 1, 1, 1e4})));

  graphslam::TResultInfoSpaLevMarq info;
  graphslam::optimize_graph_spa_levmarq(g, info);
  // Each precise angle follows its edge, up to second order effects of the
  // linearized information:
  EXPECT_NEAR(g.nodes[1].yaw(), 0.2, 5e-3);
  EXPECT_NEAR(g.nodes[1].roll(), 0.2, 5e-3);
  EXPECT_NEAR(g.nodes[1].pitch(), 0, 3e-2);
  EXPECT_NEAR(g.nodes[1].translation().norm(), 0, 1e-6);
}

// 3D translation information is given in the frame of the edge origin
TEST(GraphSlamLevMarq, InformationIn3DEdgeOriginFrame)
{
  CNetworkOfPoses3DInf g;
  g.root = 0;
  g.nodes[0] = CPose3D();
  g.nodes[1] = CPose3D(0.3, 0.2, 0.1, 1.4, 0.1, -0.1);
  const auto IA = diag6({1e4, 1, 1, 1, 1, 1});
  const auto IB = diag6({1, 1e4, 1, 1, 1, 1});
  g.insertEdge(0, 1, CPose3DPDFGaussianInf(CPose3D(1, 0, 0, M_PI / 2, 0, 0), IA));
  g.insertEdge(0, 1, CPose3DPDFGaussianInf(CPose3D(0, 1, 0, M_PI / 2, 0, 0), IB));

  graphslam::TResultInfoSpaLevMarq info;
  graphslam::optimize_graph_spa_levmarq(g, info);
  const double expected = 1e4 / (1e4 + 1);
  EXPECT_NEAR(g.nodes[1].x(), expected, 1e-6);
  EXPECT_NEAR(g.nodes[1].y(), expected, 1e-6);
  EXPECT_NEAR(g.nodes[1].z(), 0, 1e-6);
  EXPECT_NEAR(g.nodes[1].yaw(), M_PI / 2, 1e-6);
  EXPECT_NEAR(g.nodes[1].pitch(), 0, 1e-6);
  EXPECT_NEAR(g.nodes[1].roll(), 0, 1e-6);
}

// After a rejected step, LM must retry with a larger lambda
TEST(GraphSlamLevMarq, RecoversAfterRejectedSteps)
{
  CNetworkOfPoses2D g;
  g.root = 0;
  const unsigned int N = 20;
  std::vector<CPose2D> gt;
  for (unsigned int i = 0; i < N; i++)
  {
    const double a = 2 * M_PI * i / N;
    gt.emplace_back(5 * cos(a), 5 * sin(a), a + M_PI / 2);
  }
  for (unsigned int i = 0; i < N; i++)
  {
    g.insertEdge(i, (i + 1) % N, gt[(i + 1) % N] - gt[i]);
  }
  // Deterministic perturbation (std::normal_distribution differs across
  // standard libraries):
  for (unsigned int i = 0; i < N; i++)
  {
    const double k = static_cast<double>(i);
    g.nodes[i] =
        (i == 0)
            ? gt[0]
            : gt[i] + CPose2D(0.5 * std::sin(3 * k), 0.5 * std::cos(5 * k), 0.6 * std::sin(7 * k));
  }
  graphslam::TResultInfoSpaLevMarq info;
  mrpt::containers::yaml params;
  params["max_iterations"] = 300;
  // A tiny lambda forces a few rejected Gauss-Newton steps:
  params["initial_lambda"] = 1e-9;
  graphslam::optimize_graph_spa_levmarq(g, info, nullptr, params);
  EXPECT_LT(info.final_total_sq_error, 1e-8);
  for (unsigned int i = 0; i < N; i++)
  {
    const CPose2D d = g.nodes[i] - gt[i];
    EXPECT_NEAR(d.norm(), 0, 1e-4) << "i=" << i;
    EXPECT_NEAR(mrpt::math::wrapToPi(d.phi()), 0, 1e-4) << "i=" << i;
  }
}

// Graphs whose edges hold i (-) j must be optimized as such
TEST(GraphSlamLevMarq, EdgesStoreInversePoses)
{
  const CPose2D p1(1, 2, 0.5);
  const CPose2D p2(3, 1, -1.0);
  CNetworkOfPoses2DInf g;
  g.root = 0;
  g.edges_store_inverse_poses = true;
  CMatrixDouble33 I;
  I(0, 0) = 10;
  I(1, 1) = 2;
  I(2, 2) = 5;
  g.insertEdge(0, 1, CPosePDFGaussianInf(CPose2D() - p1, I));
  g.insertEdge(1, 2, CPosePDFGaussianInf(p1 - p2, I));
  g.insertEdge(0, 2, CPosePDFGaussianInf(CPose2D() - p2, I));
  g.nodes[0] = CPose2D();
  g.nodes[1] = p1 + CPose2D(0.1, -0.2, 0.1);
  g.nodes[2] = p2 + CPose2D(-0.1, 0.1, -0.2);

  graphslam::TResultInfoSpaLevMarq info;
  graphslam::optimize_graph_spa_levmarq(g, info);
  EXPECT_NEAR((g.nodes[1] - p1).norm(), 0, 1e-6);
  EXPECT_NEAR(mrpt::math::wrapToPi(g.nodes[1].phi() - p1.phi()), 0, 1e-6);
  EXPECT_NEAR((g.nodes[2] - p2).norm(), 0, 1e-6);
  EXPECT_NEAR(mrpt::math::wrapToPi(g.nodes[2].phi() - p2.phi()), 0, 1e-6);
  EXPECT_NEAR(info.final_total_sq_error, 0, 1e-10);
}

// Optimizing a g2o 3D graph must follow g2o's error definition: compare
// against the stationarity of the g2o cost at the solution
TEST(GraphSlamLevMarq, G2oEdgeSE3QuatSemantics)
{
  const CPose3D ZA(0.5, 0, 0, 0.2, 0.1, -0.1);
  const CPose3D ZB(0.4, 0.1, 0, -0.1, 0.0, 0.25);
  const std::array<double, 6> dA = {10, 10, 10, 1e4, 10, 10};
  const std::array<double, 6> dB = {10, 10, 10, 10, 10, 1e4};
  auto writeEdge = [](std::ostream& o, const CPose3D& Z, const std::array<double, 6>& d)
  {
    const CPose3DQuat q(Z);
    o << "EDGE_SE3:QUAT 0 1 " << q.x() << " " << q.y() << " " << q.z() << " " << q.quat().x() << " "
      << q.quat().y() << " " << q.quat().z() << " " << q.quat().r();
    for (int r = 0; r < 6; r++)
    {
      for (int c = r; c < 6; c++)
      {
        o << " " << (r == c ? d[r] : 0.0);
      }
    }
    o << "\n";
  };
  std::stringstream ss;
  ss.precision(17);
  ss << "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\nFIX 0\nVERTEX_SE3:QUAT 1 0.3 0 0 0 0 0 1\n";
  writeEdge(ss, ZA, dA);
  writeEdge(ss, ZB, dB);
  CNetworkOfPoses3DInf g;
  g.readAsText(ss);

  graphslam::TResultInfoSpaLevMarq info;
  graphslam::optimize_graph_spa_levmarq(g, info);
  const CPose3D X = g.nodes[1];

  // g2o cost, as a function of a local perturbation of X:
  auto g2oCost = [&](const mrpt::poses::Lie::SE<3>::tangent_vector& e)
  {
    const CPose3D Xe = X + mrpt::poses::Lie::SE<3>::exp(e);
    double c = 0;
    for (const auto& [Z, d] : {std::make_pair(ZA, dA), std::make_pair(ZB, dB)})
    {
      const CPose3DQuat dq(Xe - Z);
      const double s = dq.quat().r() < 0 ? -1.0 : 1.0;
      const double r[6] = {dq.x(),           dq.y(), dq.z(), s * dq.quat().x(), s * dq.quat().y(),
                           s * dq.quat().z()};
      for (int i = 0; i < 6; i++)
      {
        c += d[i] * r[i] * r[i];
      }
    }
    return c;
  };
  // The g2o cost must be (nearly) stationary at the solution, in comparison
  // to its gradient at the initial guess:
  const double h = 1e-6;
  double gradNorm2 = 0;
  for (int k = 0; k < 6; k++)
  {
    mrpt::poses::Lie::SE<3>::tangent_vector ep;
    ep.setZero();
    ep[k] = h;
    mrpt::poses::Lie::SE<3>::tangent_vector em;
    em.setZero();
    em[k] = -h;
    gradNorm2 += mrpt::square((g2oCost(ep) - g2oCost(em)) / (2 * h));
  }
  // Gradient at the optimum of the first-order equivalent cost is only due
  // to the difference between the quaternion and rotation vector residuals:
  EXPECT_LT(std::sqrt(gradNorm2), 5.0);
  EXPECT_NEAR(X.x(), 0.45, 1e-3);
  EXPECT_NEAR(X.y(), 0.05, 1e-3);
  EXPECT_NEAR(X.yaw(), -0.0974866, 2e-3);
  EXPECT_NEAR(X.pitch(), 0.0337647, 2e-3);
  EXPECT_NEAR(X.roll(), -0.119665, 2e-3);
}
