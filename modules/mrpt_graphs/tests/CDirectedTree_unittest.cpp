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
#include <mrpt/graphs/CDirectedTree.h>

using namespace mrpt::graphs;

using tree_t = CDirectedTree<int>;

static tree_t buildSampleTree()
{
  tree_t tree;
  tree.root = 0;
  tree.edges_to_children[0].emplace_back(1, false, 10);
  tree.edges_to_children[0].emplace_back(2, false, 20);
  tree.edges_to_children[1].emplace_back(3, true, 30);
  return tree;
}

TEST(CDirectedTree, DefaultAndClear)
{
  tree_t tree;
  tree.root = 5;
  tree.edges_to_children[5].emplace_back(6);
  tree.clear();
  EXPECT_EQ(tree.root, INVALID_NODEID);
  EXPECT_TRUE(tree.edges_to_children.empty());
}

TEST(CDirectedTree, TEdgeInfoDefaults)
{
  tree_t::TEdgeInfo e(7);
  EXPECT_EQ(e.id, 7U);
  EXPECT_FALSE(e.reverse);
  EXPECT_EQ(e.data, 0);

  tree_t::TEdgeInfo e2(8, true, 42);
  EXPECT_EQ(e2.id, 8U);
  EXPECT_TRUE(e2.reverse);
  EXPECT_EQ(e2.data, 42);
}

TEST(CDirectedTree, VisitDepthFirstLambda)
{
  tree_t tree = buildSampleTree();

  std::vector<std::pair<TNodeID, TNodeID>> visited;  // (parent, child)
  std::vector<size_t> depths;

  tree.visitDepthFirst(
      tree.root,
      [&](TNodeID parent, const tree_t::TEdgeInfo& edge, size_t depth)
      {
        visited.emplace_back(parent, edge.id);
        depths.push_back(depth);
      });

  ASSERT_EQ(visited.size(), 3U);
  // Depth-first: 0->1, 1->3, 0->2
  EXPECT_EQ(visited[0], std::make_pair(TNodeID(0), TNodeID(1)));
  EXPECT_EQ(visited[1], std::make_pair(TNodeID(1), TNodeID(3)));
  EXPECT_EQ(visited[2], std::make_pair(TNodeID(0), TNodeID(2)));

  EXPECT_EQ(depths[0], 1U);
  EXPECT_EQ(depths[1], 2U);
  EXPECT_EQ(depths[2], 1U);
}

TEST(CDirectedTree, VisitBreadthFirstLambda)
{
  tree_t tree = buildSampleTree();

  std::vector<TNodeID> visitedChildren;
  tree.visitBreadthFirst(
      tree.root,
      [&](TNodeID, const tree_t::TEdgeInfo& edge, size_t) { visitedChildren.push_back(edge.id); });

  ASSERT_EQ(visitedChildren.size(), 3U);
  EXPECT_EQ(visitedChildren[0], 1U);
  EXPECT_EQ(visitedChildren[1], 2U);
  EXPECT_EQ(visitedChildren[2], 3U);
}

TEST(CDirectedTree, VisitNoChildren)
{
  tree_t tree = buildSampleTree();
  size_t count = 0;
  tree.visitDepthFirst(
      99 /* not present */, [&](TNodeID, const tree_t::TEdgeInfo&, size_t) { count++; });
  EXPECT_EQ(count, 0U);

  tree.visitBreadthFirst(99, [&](TNodeID, const tree_t::TEdgeInfo&, size_t) { count++; });
  EXPECT_EQ(count, 0U);
}

namespace
{
struct CollectingVisitor : public tree_t::Visitor
{
  std::vector<TNodeID> ids;
  void OnVisitNode(
      const TNodeID parent, const tree_t::TEdgeInfo& edge_to_child, size_t depth_level) override
  {
    (void)parent;
    (void)depth_level;
    ids.push_back(edge_to_child.id);
  }
};
}  // namespace

TEST(CDirectedTree, DeprecatedVisitorClassDepthFirst)
{
  tree_t tree = buildSampleTree();
  CollectingVisitor visitor;
  tree.visitDepthFirst(tree.root, visitor);
  ASSERT_EQ(visitor.ids.size(), 3U);
  EXPECT_EQ(visitor.ids[0], 1U);
  EXPECT_EQ(visitor.ids[1], 3U);
  EXPECT_EQ(visitor.ids[2], 2U);
}

TEST(CDirectedTree, DeprecatedVisitorClassBreadthFirst)
{
  tree_t tree = buildSampleTree();
  CollectingVisitor visitor;
  tree.visitBreadthFirst(tree.root, visitor);
  ASSERT_EQ(visitor.ids.size(), 3U);
  EXPECT_EQ(visitor.ids[0], 1U);
  EXPECT_EQ(visitor.ids[1], 2U);
  EXPECT_EQ(visitor.ids[2], 3U);
}

TEST(CDirectedTree, GetAsTextDescription)
{
  tree_t tree = buildSampleTree();
  const std::string s = tree.getAsTextDescription();
  EXPECT_NE(s.find("0"), std::string::npos);
  EXPECT_NE(s.find("->1"), std::string::npos);
  EXPECT_NE(s.find("<-3"), std::string::npos);
  EXPECT_NE(s.find("->2"), std::string::npos);
}
