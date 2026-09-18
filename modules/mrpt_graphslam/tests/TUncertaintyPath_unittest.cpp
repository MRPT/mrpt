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

/** Unit tests for TUncertaintyPath, the node+constraint chain used by
 *  CLoopCloserERD to compare candidate loop closures by their uncertainty.
 */

#include <gtest/gtest.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/misc/TUncertaintyPath.h>

#include <sstream>
#include <string>

using mrpt::graphs::TNodeID;
using path_t = mrpt::graphslam::TUncertaintyPath<mrpt::graphs::CNetworkOfPoses2DInf>;
using constraint_t = path_t::constraint_t;

namespace
{
/** A relative pose with the given (isotropic) information matrix */
constraint_t makeEdge(double x, double y, double phi, double information)
{
  constraint_t c;
  c.mean = mrpt::poses::CPose2D(x, y, phi);
  c.cov_inv = mrpt::math::CMatrixDouble33::Identity() * information;
  return c;
}
}  // namespace

TEST(TUncertaintyPath, default_constructed_is_empty)
{
  const path_t p;
  EXPECT_TRUE(p.isEmpty());
  EXPECT_TRUE(p.nodes_traversed.empty());
  EXPECT_EQ(p.curr_pose_pdf.mean, mrpt::poses::CPose2D(0, 0, 0));
  // clear() seeds a very high information (i.e. a very certain origin):
  EXPECT_DOUBLE_EQ(p.curr_pose_pdf.cov_inv(0, 0), 10000.0);
  EXPECT_DOUBLE_EQ(p.curr_pose_pdf.cov_inv(0, 1), 0.0);
}

TEST(TUncertaintyPath, single_node_constructor)
{
  const path_t p(TNodeID(7));
  EXPECT_FALSE(p.isEmpty());
  ASSERT_EQ(p.nodes_traversed.size(), 1U);
  EXPECT_EQ(p.getSource(), TNodeID(7));
  EXPECT_EQ(p.getDestination(), TNodeID(7));
}

TEST(TUncertaintyPath, two_node_constructor_applies_the_edge)
{
  const auto edge = makeEdge(1.0, 2.0, 0.0, 5.0);
  const path_t p(TNodeID(3), TNodeID(4), edge);

  ASSERT_EQ(p.nodes_traversed.size(), 2U);
  EXPECT_EQ(p.getSource(), TNodeID(3));
  EXPECT_EQ(p.getDestination(), TNodeID(4));
  EXPECT_NEAR(p.curr_pose_pdf.mean.x(), 1.0, 1e-9);
  EXPECT_NEAR(p.curr_pose_pdf.mean.y(), 2.0, 1e-9);
}

TEST(TUncertaintyPath, addToPath_chains_relative_poses)
{
  path_t p(TNodeID(0));
  p.addToPath(TNodeID(1), makeEdge(1.0, 0.0, 0.0, 100.0));
  p.addToPath(TNodeID(2), makeEdge(1.0, 0.0, 0.0, 100.0));

  ASSERT_EQ(p.nodes_traversed.size(), 3U);
  EXPECT_EQ(p.getDestination(), TNodeID(2));
  EXPECT_NEAR(p.curr_pose_pdf.mean.x(), 2.0, 1e-9);
  EXPECT_NEAR(p.curr_pose_pdf.mean.y(), 0.0, 1e-9);
}

TEST(TUncertaintyPath, concatenating_two_paths)
{
  path_t a(TNodeID(0));
  a.addToPath(TNodeID(1), makeEdge(1.0, 0.0, 0.0, 100.0));

  path_t b(TNodeID(1));
  b.addToPath(TNodeID(2), makeEdge(0.0, 1.0, 0.0, 100.0));

  a += b;

  ASSERT_EQ(a.nodes_traversed.size(), 3U);
  EXPECT_EQ(a.getSource(), TNodeID(0));
  EXPECT_EQ(a.getDestination(), TNodeID(2));
  // The shared node must not be duplicated:
  EXPECT_EQ(a.nodes_traversed[1], TNodeID(1));
}

TEST(TUncertaintyPath, equality_compares_nodes_and_pdf)
{
  path_t a(TNodeID(0));
  a.addToPath(TNodeID(1), makeEdge(1.0, 0.0, 0.0, 100.0));

  path_t b(TNodeID(0));
  b.addToPath(TNodeID(1), makeEdge(1.0, 0.0, 0.0, 100.0));
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);

  // Same nodes, different constraint:
  path_t c(TNodeID(0));
  c.addToPath(TNodeID(1), makeEdge(2.0, 0.0, 0.0, 100.0));
  EXPECT_TRUE(a != c);

  // Same constraint, different nodes:
  path_t d(TNodeID(0));
  d.addToPath(TNodeID(9), makeEdge(1.0, 0.0, 0.0, 100.0));
  EXPECT_TRUE(a != d);
}

TEST(TUncertaintyPath, assertIsBetweenNodeIDs_accepts_the_actual_ends)
{
  path_t p(TNodeID(4));
  p.addToPath(TNodeID(5), makeEdge(1.0, 0.0, 0.0, 100.0));
  EXPECT_NO_THROW(p.assertIsBetweenNodeIDs(TNodeID(4), TNodeID(5)));
}

TEST(TUncertaintyPath, getDeterminant_is_cached_and_invalidated_by_addToPath)
{
  path_t p(TNodeID(0));
  const double d0 = p.getDeterminant();
  EXPECT_TRUE(p.determinant_is_updated);
  // A second query must return the cached value untouched:
  EXPECT_DOUBLE_EQ(p.getDeterminant(), d0);

  p.addToPath(TNodeID(1), makeEdge(1.0, 0.0, 0.0, 10.0));
  EXPECT_FALSE(p.determinant_is_updated);
  const double d1 = p.getDeterminant();
  EXPECT_TRUE(p.determinant_is_updated);
  // Adding an edge can only lose information:
  EXPECT_LT(d1, d0);
}

TEST(TUncertaintyPath, hasLowerUncertaintyThan_in_information_form)
{
  // In information form the *larger* determinant is the more certain one.
  path_t certain(TNodeID(0));
  certain.addToPath(TNodeID(1), makeEdge(1.0, 0.0, 0.0, 1000.0));

  path_t uncertain(TNodeID(0));
  uncertain.addToPath(TNodeID(1), makeEdge(1.0, 0.0, 0.0, 1.0));

  ASSERT_TRUE(certain.curr_pose_pdf.isInfType());
  EXPECT_GT(certain.getDeterminant(), uncertain.getDeterminant());
  EXPECT_TRUE(certain.hasLowerUncertaintyThan(uncertain));
  EXPECT_FALSE(uncertain.hasLowerUncertaintyThan(certain));
}

TEST(TUncertaintyPath, string_representation_lists_the_traversed_nodes)
{
  path_t p(TNodeID(11));
  p.addToPath(TNodeID(12), makeEdge(1.0, 0.0, 0.0, 100.0));

  const std::string s = p.getAsString();
  EXPECT_NE(s.find("Path properties"), std::string::npos);
  EXPECT_NE(s.find("11"), std::string::npos);
  EXPECT_NE(s.find("12"), std::string::npos);
  EXPECT_NE(s.find("Determinant"), std::string::npos);

  std::stringstream ss;
  p.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("Path properties"), std::string::npos);

  // operator<< uses the same representation:
  std::stringstream ss2;
  ss2 << p;
  EXPECT_NE(ss2.str().find("Path properties"), std::string::npos);
}

TEST(TUncertaintyPath, clear_restores_the_default_state)
{
  path_t p(TNodeID(0));
  p.addToPath(TNodeID(1), makeEdge(1.0, 2.0, 0.5, 3.0));
  ASSERT_FALSE(p.isEmpty());

  p.clear();
  EXPECT_TRUE(p.isEmpty());
  EXPECT_TRUE(p.nodes_traversed.empty());
  EXPECT_FALSE(p.determinant_is_updated);
}
