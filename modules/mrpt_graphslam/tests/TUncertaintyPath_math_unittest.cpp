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

/** Math checks for TUncertaintyPath: the mean and covariance of a path must
 *  equal the first-order composition of its edges, starting from an exactly
 *  known source node (the path is a relative pose of the destination with
 *  respect to the source).
 */

#include <gtest/gtest.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/misc/TUncertaintyPath.h>
#include <mrpt/math/CMatrixFixed.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

using mrpt::graphs::TNodeID;
using path_t = mrpt::graphslam::TUncertaintyPath<mrpt::graphs::CNetworkOfPoses2DInf>;
using constraint_t = path_t::constraint_t;

namespace
{
constraint_t makeEdge(double x, double y, double phi, const Eigen::Matrix3d& cov)
{
  constraint_t c;
  c.mean = mrpt::poses::CPose2D(x, y, phi);
  c.cov_inv = mrpt::math::CMatrixDouble33(cov.inverse());
  return c;
}

/** Independent first-order propagation of x (+) u for SE(2) */
struct Gauss2D
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
};

Gauss2D composeFirstOrder(const Gauss2D& x, const Eigen::Vector3d& u, const Eigen::Matrix3d& cov_u)
{
  const double c = std::cos(x.mean[2]);
  const double s = std::sin(x.mean[2]);
  Eigen::Matrix3d J1 = Eigen::Matrix3d::Identity();
  J1(0, 2) = -s * u[0] - c * u[1];
  J1(1, 2) = c * u[0] - s * u[1];
  Eigen::Matrix3d J2 = Eigen::Matrix3d::Identity();
  J2(0, 0) = c;
  J2(0, 1) = -s;
  J2(1, 0) = s;
  J2(1, 1) = c;

  Gauss2D r;
  r.mean[0] = x.mean[0] + c * u[0] - s * u[1];
  r.mean[1] = x.mean[1] + s * u[0] + c * u[1];
  r.mean[2] = x.mean[2] + u[2];
  r.cov = J1 * x.cov * J1.transpose() + J2 * cov_u * J2.transpose();
  return r;
}

Eigen::Matrix3d pathCovariance(const path_t& p)
{
  return p.curr_pose_pdf.cov_inv.asEigen().inverse();
}

struct EdgeDef
{
  double x;
  double y;
  double phi;
};

const std::vector<EdgeDef>& testEdges()
{
  static const std::vector<EdgeDef> edges = {
      {1.0,  0.0,  0.3},
      {1.2,  0.1, -0.2},
      {0.8, -0.3,  0.5},
      {1.0,  0.2,  0.1},
      {0.9,  0.0, -0.4},
      {1.1, -0.1,  0.2},
      {1.0,  0.3,  0.0},
      {0.7,  0.0,  0.6},
      {1.3, -0.2,  0.1},
      {1.0,  0.1, -0.3}
  };
  return edges;
}

Eigen::Matrix3d edgeCov()
{
  // A precise scan-matching edge: 1 cm, 0.2 deg
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  cov(0, 0) = 1e-4;
  cov(1, 1) = 1e-4;
  cov(2, 2) = std::pow(0.2 * M_PI / 180.0, 2);
  return cov;
}
}  // namespace

TEST(TUncertaintyPath, mean_is_the_right_composition_of_the_edges)
{
  path_t p(TNodeID(0));
  Gauss2D ref;
  TNodeID n = 1;
  for (const auto& e : testEdges())
  {
    p.addToPath(n++, makeEdge(e.x, e.y, e.phi, edgeCov()));
    ref = composeFirstOrder(ref, Eigen::Vector3d(e.x, e.y, e.phi), edgeCov());
  }
  EXPECT_NEAR(p.curr_pose_pdf.mean.x(), ref.mean[0], 1e-9);
  EXPECT_NEAR(p.curr_pose_pdf.mean.y(), ref.mean[1], 1e-9);
  EXPECT_NEAR(p.curr_pose_pdf.mean.phi(), mrpt::math::wrapToPi(ref.mean[2]), 1e-9);
}

TEST(TUncertaintyPath, single_edge_path_reproduces_the_edge_uncertainty)
{
  const auto edge = makeEdge(5.0, 0.0, 0.0, edgeCov());
  const path_t p(TNodeID(0), TNodeID(1), edge);
  const Eigen::Matrix3d cov = pathCovariance(p);
  const Eigen::Matrix3d expected = edgeCov();
  EXPECT_LT((cov - expected).norm(), 1e-3 * expected.norm()) << "cov:\n"
                                                             << cov << "\nexpected:\n"
                                                             << expected;
}

TEST(TUncertaintyPath, path_covariance_matches_first_order_composition)
{
  path_t p(TNodeID(0));
  Gauss2D ref;
  TNodeID n = 1;
  for (const auto& e : testEdges())
  {
    p.addToPath(n++, makeEdge(e.x, e.y, e.phi, edgeCov()));
    ref = composeFirstOrder(ref, Eigen::Vector3d(e.x, e.y, e.phi), edgeCov());
  }
  const Eigen::Matrix3d cov = pathCovariance(p);
  EXPECT_LT((cov - ref.cov).norm(), 1e-2 * ref.cov.norm()) << "cov:\n"
                                                           << cov << "\nexpected:\n"
                                                           << ref.cov;
}

TEST(TUncertaintyPath, concatenation_equals_sequential_composition)
{
  const auto& edges = testEdges();
  path_t seq(TNodeID(0));
  path_t first(TNodeID(0));
  path_t second(TNodeID(5));
  for (size_t i = 0; i < edges.size(); i++)
  {
    const auto e = makeEdge(edges[i].x, edges[i].y, edges[i].phi, edgeCov());
    seq.addToPath(TNodeID(i + 1), e);
    if (i < 5)
    {
      first.addToPath(TNodeID(i + 1), e);
    }
    else
    {
      second.addToPath(TNodeID(i + 1), e);
    }
  }
  path_t cat = first;
  cat += second;

  ASSERT_EQ(cat.nodes_traversed, seq.nodes_traversed);
  EXPECT_NEAR(cat.curr_pose_pdf.mean.x(), seq.curr_pose_pdf.mean.x(), 1e-9);
  EXPECT_NEAR(cat.curr_pose_pdf.mean.y(), seq.curr_pose_pdf.mean.y(), 1e-9);
  EXPECT_NEAR(cat.curr_pose_pdf.mean.phi(), seq.curr_pose_pdf.mean.phi(), 1e-9);

  const Eigen::Matrix3d cov_cat = pathCovariance(cat);
  const Eigen::Matrix3d cov_seq = pathCovariance(seq);
  EXPECT_LT((cov_cat - cov_seq).norm(), 1e-6 * cov_seq.norm()) << "concatenated:\n"
                                                               << cov_cat << "\nsequential:\n"
                                                               << cov_seq;
}

TEST(TUncertaintyPath, determinant_is_that_of_the_information_matrix)
{
  path_t p(TNodeID(0));
  p.addToPath(TNodeID(1), makeEdge(1.0, 0.5, 0.2, edgeCov()));
  p.addToPath(TNodeID(2), makeEdge(0.5, -0.5, 0.7, edgeCov()));
  ASSERT_TRUE(p.curr_pose_pdf.isInfType());
  const double expected = p.curr_pose_pdf.cov_inv.asEigen().determinant();
  EXPECT_NEAR(p.getDeterminant(), expected, 1e-9 * std::abs(expected));
}

TEST(TUncertaintyPath, longer_path_of_equal_edges_has_higher_uncertainty)
{
  path_t short_path(TNodeID(0));
  path_t long_path(TNodeID(0));
  for (TNodeID i = 1; i <= 3; i++)
  {
    short_path.addToPath(i, makeEdge(1.0, 0.0, 0.1, edgeCov()));
  }
  for (TNodeID i = 1; i <= 6; i++)
  {
    long_path.addToPath(i, makeEdge(1.0, 0.0, 0.1, edgeCov()));
  }
  EXPECT_TRUE(short_path.hasLowerUncertaintyThan(long_path));
  EXPECT_FALSE(long_path.hasLowerUncertaintyThan(short_path));
}
