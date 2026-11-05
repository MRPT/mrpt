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
#include <mrpt/graphs/ScalarFactorGraph.h>

#include <Eigen/Dense>
#include <deque>
#include <random>

using namespace mrpt;
using namespace mrpt::graphs;
using namespace std;

#if EIGEN_VERSION_AT_LEAST(3, 1, 0)  // Requires Eigen>=3.1

struct MySimpleUnaryEdge : public ScalarFactorGraph::UnaryFactorVirtualBase
{
  MySimpleUnaryEdge(vector<double>& parent, size_t nodeid, double observation, double information) :
      m_parent(parent), m_observation(observation), m_information(information)
  {
    this->node_id = nodeid;
  }

  double evaluateResidual() const override { return m_parent[node_id] - m_observation; }
  double getInformation() const override { return m_information; }
  void evalJacobian(double& dr_dx) const override { dr_dx = 1.0; }

 protected:
  vector<double>& m_parent;
  double m_observation, m_information;
};

struct MySimpleBinaryEdge : public ScalarFactorGraph::BinaryFactorVirtualBase
{
  MySimpleBinaryEdge(vector<double>& parent, size_t nodeid_i, size_t nodeid_j, double information) :
      m_parent(parent), m_information(information)
  {
    this->node_id_i = nodeid_i;
    this->node_id_j = nodeid_j;
  }

  double evaluateResidual() const override { return m_parent[node_id_i] - m_parent[node_id_j]; }
  double getInformation() const override { return m_information; }
  void evalJacobian(double& dr_dx_i, double& dr_dx_j) const override
  {
    dr_dx_i = +1.0;
    dr_dx_j = -1.0;
  }

 protected:
  vector<double>& m_parent;
  double m_information;
};

TEST(ScalarFactorGraph, MiniMRF_UnaryEdges)
{
  const size_t N = 4;
  vector<double> my_map(N, .0);

  ScalarFactorGraph gmrf;
  gmrf.enableProfiler(false);

  gmrf.initialize(N);

  std::deque<MySimpleUnaryEdge> edges1;

  edges1.emplace_back(my_map, 0, 1.0, 4.0);
  edges1.emplace_back(my_map, 1, 5.0, 4.0);
  edges1.emplace_back(my_map, 2, 3.0, 4.0);
  edges1.emplace_back(my_map, 3, 2.0, 16.0);

  for (const auto& e : edges1) gmrf.addConstraint(e);

  // Test 1:
  // --------------
  {
    my_map.assign(N, .0);

    mrpt::math::CVectorDouble x_incr, x_var;
    gmrf.updateEstimation(x_incr, &x_var);

    EXPECT_NEAR(x_incr[0], 1.0, 1e-9);
    EXPECT_NEAR(x_incr[1], 5.0, 1e-9);
    EXPECT_NEAR(x_incr[2], 3.0, 1e-9);
    EXPECT_NEAR(x_incr[3], 2.0, 1e-9);

    EXPECT_NEAR(x_var[0], 1.0 / 4.0, 1e-9);
    EXPECT_NEAR(x_var[1], 1.0 / 4.0, 1e-9);
    EXPECT_NEAR(x_var[2], 1.0 / 4.0, 1e-9);
    EXPECT_NEAR(x_var[3], 1.0 / 16.0, 1e-9);
  }

  // Test 2:
  // --------------
  {
    my_map.assign(N, .0);

    // Add new edge:
    edges1.emplace_back(my_map, 0, 4.0, 2.0);
    gmrf.addConstraint(*edges1.rbegin());

    mrpt::math::CVectorDouble x_incr, x_var;
    gmrf.updateEstimation(x_incr, &x_var);

    EXPECT_NEAR(x_incr[0], 2.0, 1e-9);
    EXPECT_NEAR(x_incr[1], 5.0, 1e-9);
    EXPECT_NEAR(x_incr[2], 3.0, 1e-9);
    EXPECT_NEAR(x_incr[3], 2.0, 1e-9);

    EXPECT_NEAR(x_var[0], 1.0 / (4.0 + 2.0), 1e-9);
    EXPECT_NEAR(x_var[1], 1.0 / 4.0, 1e-9);
    EXPECT_NEAR(x_var[2], 1.0 / 4.0, 1e-9);
    EXPECT_NEAR(x_var[3], 1.0 / 16.0, 1e-9);
  }
}

TEST(ScalarFactorGraph, MiniMRF_BinaryEdges)
{
  const size_t N = 4;
  vector<double> my_map(N, .0);

  ScalarFactorGraph gmrf;
  gmrf.enableProfiler(false);

  gmrf.initialize(N);

  // Edge to assign a value to node 0:
  MySimpleUnaryEdge edge_val0(my_map, 0, 1.0 /*value*/, 1.0 /*information*/);
  gmrf.addConstraint(edge_val0);

  MySimpleBinaryEdge edge_01(my_map, 0, 1, .1);
  gmrf.addConstraint(edge_01);
  MySimpleBinaryEdge edge_12(my_map, 1, 2, .1);
  gmrf.addConstraint(edge_12);
  MySimpleBinaryEdge edge_23(my_map, 2, 3, .1);
  gmrf.addConstraint(edge_23);
  MySimpleBinaryEdge edge_30(my_map, 3, 0, .1);
  gmrf.addConstraint(edge_30);

  // Test 1:
  // --------------
  {
    my_map.assign(N, .0);

    mrpt::math::CVectorDouble x_incr, x_var;
    gmrf.updateEstimation(x_incr, &x_var);

    EXPECT_NEAR(x_incr[0], 1.0, 1e-6);
    EXPECT_NEAR(x_incr[1], 1.0, 1e-6);
    EXPECT_NEAR(x_incr[2], 1.0, 1e-6);
    EXPECT_NEAR(x_incr[3], 1.0, 1e-6);

    EXPECT_NEAR(x_var[0], 1.0, 1e-6);

    EXPECT_GT(x_var[1], x_var[0]);
    EXPECT_GT(x_var[3], x_var[0]);

    EXPECT_GT(x_var[2], x_var[1]);
    EXPECT_GT(x_var[2], x_var[3]);
  }
}

// Solution and variances of a random sparse problem must match those of the
// dense normal equations
TEST(ScalarFactorGraph, RandomSparseProblemMatchesDenseSolution)
{
  const size_t N = 40;
  vector<double> my_map(N, .0);
  std::mt19937 rng(1234);
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  ScalarFactorGraph gmrf;
  gmrf.initialize(N);

  std::deque<MySimpleUnaryEdge> unaryEdges;
  std::deque<MySimpleBinaryEdge> binaryEdges;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(N, N);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(N);

  // Some observed nodes:
  for (size_t i = 0; i < N; i += 7)
  {
    const double obs = 10 * unif(rng) - 5;
    const double inf = 0.5 + 2 * unif(rng);
    unaryEdges.emplace_back(my_map, i, obs, inf);
    gmrf.addConstraint(unaryEdges.back());
    H(i, i) += inf;
    g[i] += inf * obs;
  }
  // A chain plus random long-range links:
  auto addBinary = [&](size_t i, size_t j)
  {
    const double inf = 0.1 + unif(rng);
    binaryEdges.emplace_back(my_map, i, j, inf);
    gmrf.addConstraint(binaryEdges.back());
    H(i, i) += inf;
    H(j, j) += inf;
    H(i, j) -= inf;
    H(j, i) -= inf;
  };
  for (size_t i = 0; i + 1 < N; i++)
  {
    addBinary(i, i + 1);
  }
  for (size_t k = 0; k < 25; k++)
  {
    const size_t i = static_cast<size_t>(unif(rng) * N) % N;
    const size_t j = static_cast<size_t>(unif(rng) * N) % N;
    if (i != j)
    {
      addBinary(i, j);
    }
  }

  mrpt::math::CVectorDouble x_incr;
  mrpt::math::CVectorDouble x_var;
  gmrf.updateEstimation(x_incr, &x_var);

  // Dense reference, starting from x=0:
  const Eigen::MatrixXd Sigma = H.inverse();
  const Eigen::VectorXd x = H.ldlt().solve(g);
  ASSERT_EQ(static_cast<size_t>(x_incr.size()), N);
  ASSERT_EQ(static_cast<size_t>(x_var.size()), N);
  for (size_t i = 0; i < N; i++)
  {
    EXPECT_NEAR(x_incr[i], x[i], 1e-8 * (1 + std::abs(x[i]))) << "i=" << i;
    EXPECT_NEAR(x_var[i], Sigma(i, i), 1e-8 * Sigma(i, i)) << "i=" << i;
  }
}

#endif  // Eigen>=3.1
