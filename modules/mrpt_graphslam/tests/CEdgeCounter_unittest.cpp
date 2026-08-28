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

/** Unit tests for the edge bookkeeping of CEdgeCounter. The visualization
 *  half of the class needs a live CDisplayWindow3D and is not exercised here;
 *  every method below is reachable with no window manager attached.
 */

#include <gtest/gtest.h>
#include <mrpt/graphslam/misc/CEdgeCounter.h>

#include <string>

using mrpt::graphslam::detail::CEdgeCounter;

TEST(CEdgeCounter, default_constructed_is_empty)
{
  CEdgeCounter c;
  EXPECT_EQ(c.getTotalNumOfEdges(), 0);
  EXPECT_EQ(c.getLoopClosureEdges(), 0);
  EXPECT_EQ(c.cbegin(), c.cend());
}

TEST(CEdgeCounter, registering_and_counting_edge_types)
{
  CEdgeCounter c;
  c.addEdgeType("odometry");
  c.addEdgeType("ICP2D");

  EXPECT_EQ(c.getNumForEdgeType("odometry"), 0);
  EXPECT_EQ(c.getNumForEdgeType("ICP2D"), 0);
  EXPECT_EQ(c.getTotalNumOfEdges(), 0);

  // Registering the same type twice is an error:
  EXPECT_ANY_THROW(c.addEdgeType("odometry"));
  // ... and so is querying an unknown one:
  EXPECT_ANY_THROW(c.getNumForEdgeType("nope"));
}

TEST(CEdgeCounter, adding_edges_of_a_known_type)
{
  CEdgeCounter c;
  c.addEdgeType("odometry");

  c.addEdge("odometry");
  c.addEdge("odometry");
  EXPECT_EQ(c.getNumForEdgeType("odometry"), 2);
  EXPECT_EQ(c.getTotalNumOfEdges(), 2);

  int viaOutParam = -1;
  c.getNumForEdgeType("odometry", &viaOutParam);
  EXPECT_EQ(viaOutParam, 2);

  int totalViaOutParam = -1;
  c.getTotalNumOfEdges(&totalViaOutParam);
  EXPECT_EQ(totalViaOutParam, 2);
}

TEST(CEdgeCounter, adding_an_edge_of_an_unknown_type_requires_is_new)
{
  CEdgeCounter c;
  EXPECT_ANY_THROW(c.addEdge("ICP2D"));

  c.addEdge("ICP2D", false /*is_loop_closure*/, true /*is_new*/);
  EXPECT_EQ(c.getNumForEdgeType("ICP2D"), 1);

  // Once it exists, is_new must not be passed again:
  EXPECT_ANY_THROW(c.addEdge("ICP2D", false, true));
}

TEST(CEdgeCounter, loop_closures_are_counted_separately)
{
  CEdgeCounter c;
  c.addEdgeType("ICP2D");

  c.addEdge("ICP2D");
  c.addEdge("ICP2D", true /*is_loop_closure*/);
  c.addEdge("ICP2D", true);

  EXPECT_EQ(c.getNumForEdgeType("ICP2D"), 3);
  EXPECT_EQ(c.getTotalNumOfEdges(), 3);
  EXPECT_EQ(c.getLoopClosureEdges(), 2);
}

TEST(CEdgeCounter, loop_closure_flag_on_a_brand_new_edge_type)
{
  CEdgeCounter c;

  // Documenting current behavior, which is asymmetric: for an edge type that
  // already exists, passing is_new together with is_loop_closure throws, but
  // for a brand-new type the same combination is accepted and the loop
  // closure is simply not counted.
  c.addEdge("brand_new", true /*is_loop_closure*/, true /*is_new*/);
  EXPECT_EQ(c.getNumForEdgeType("brand_new"), 1);
  EXPECT_EQ(c.getLoopClosureEdges(), 0);

  EXPECT_ANY_THROW(c.addEdge("brand_new", true /*is_loop_closure*/, true /*is_new*/));
}

TEST(CEdgeCounter, loop_closures_can_be_set_manually)
{
  CEdgeCounter c;
  c.setLoopClosureEdgesManually(7);
  EXPECT_EQ(c.getLoopClosureEdges(), 7);
}

TEST(CEdgeCounter, edge_counts_can_be_set_manually)
{
  CEdgeCounter c;
  c.addEdgeType("odometry");
  c.setEdgesManually("odometry", 42);
  EXPECT_EQ(c.getNumForEdgeType("odometry"), 42);
  EXPECT_EQ(c.getTotalNumOfEdges(), 42);

  EXPECT_ANY_THROW(c.setEdgesManually("unknown", 1));
}

TEST(CEdgeCounter, removed_edges_yield_the_unique_edge_count)
{
  CEdgeCounter c;
  c.addEdgeType("odometry");
  c.setEdgesManually("odometry", 10);

  c.setRemovedEdges(3);
  // Not directly readable, but it is reported in the summary:
  EXPECT_NE(c.getAsString().find("7"), std::string::npos);
}

TEST(CEdgeCounter, iteration_visits_every_registered_type)
{
  CEdgeCounter c;
  c.addEdgeType("a");
  c.addEdgeType("b");
  c.setEdgesManually("a", 2);
  c.setEdgesManually("b", 5);

  int sum = 0;
  int n = 0;
  for (auto it = c.cbegin(); it != c.cend(); ++it)
  {
    sum += it->second;
    ++n;
  }
  EXPECT_EQ(n, 2);
  EXPECT_EQ(sum, 7);

  // The mutable iterators allow editing in place:
  for (auto it = c.begin(); it != c.end(); ++it)
  {
    it->second = 1;
  }
  EXPECT_EQ(c.getTotalNumOfEdges(), 2);
}

TEST(CEdgeCounter, summary_string_lists_the_registered_edges)
{
  CEdgeCounter c;
  c.addEdgeType("odometry");
  c.addEdgeType("ICP2D");
  c.addEdge("odometry");
  c.addEdge("ICP2D", true /*is_loop_closure*/);

  const std::string s = c.getAsString();
  EXPECT_NE(s.find("Summary of Edges"), std::string::npos);
  EXPECT_NE(s.find("odometry"), std::string::npos);
  EXPECT_NE(s.find("Total registered edges"), std::string::npos);
  EXPECT_NE(s.find("Loop closure edges"), std::string::npos);

  // The out-parameter overload agrees with the returning one:
  std::string viaOutParam;
  c.getAsString(&viaOutParam);
  EXPECT_EQ(viaOutParam, s);
}

TEST(CEdgeCounter, clearAllEdges_resets_every_counter)
{
  CEdgeCounter c;
  c.addEdgeType("odometry");
  c.setEdgesManually("odometry", 10);
  c.addEdge("odometry", true /*loop closure*/);
  c.setRemovedEdges(4);

  ASSERT_EQ(c.getTotalNumOfEdges(), 11);
  ASSERT_EQ(c.getLoopClosureEdges(), 1);

  c.clearAllEdges();

  EXPECT_EQ(c.getTotalNumOfEdges(), 0);
  EXPECT_EQ(c.getLoopClosureEdges(), 0);
  EXPECT_EQ(c.cbegin(), c.cend());
  EXPECT_ANY_THROW(c.getNumForEdgeType("odometry"));

  // The "unique edges" tally must not survive the reset either:
  EXPECT_NE(c.getAsString().find("same nodes): 0"), std::string::npos);
}

TEST(CEdgeCounter, dumpToConsole_does_not_throw)
{
  CEdgeCounter c;
  c.addEdgeType("odometry");
  c.addEdge("odometry");
  EXPECT_NO_THROW(c.dumpToConsole());
}
