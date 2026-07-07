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
#include <mrpt/graphs/CAStarAlgorithm.h>

using namespace mrpt::graphs;

namespace
{
/** A trivial 1D path-finding problem: reach "target" from position 0 by steps
 * of size 1 or 2, each step costing its size. Since the cost of reaching any
 * position equals that position, the remaining-distance heuristic is exact
 * (and thus admissible). */
struct PathSolution
{
  int pos{0};
  double cost{0};

  bool operator==(const PathSolution& o) const { return pos == o.pos; }
};

class LinePathFinder : public CAStarAlgorithm<PathSolution>
{
 public:
  explicit LinePathFinder(int target, bool allowValid = true) :
      target_(target), allowValid_(allowValid)
  {
  }

  bool isSolutionEnded(const PathSolution& sol) override { return sol.pos == target_; }
  bool isSolutionValid(const PathSolution& sol) override
  {
    return allowValid_ && sol.pos <= target_;
  }
  void generateChildren(const PathSolution& sol, std::vector<PathSolution>& sols) override
  {
    sols.clear();
    PathSolution c1;
    c1.pos = sol.pos + 1;
    c1.cost = sol.cost + 1;
    sols.push_back(c1);

    PathSolution c2;
    c2.pos = sol.pos + 2;
    c2.cost = sol.cost + 2;
    sols.push_back(c2);
  }
  double getHeuristic(const PathSolution& sol) override { return std::max(0, target_ - sol.pos); }
  double getCost(const PathSolution& sol) override { return sol.cost; }

 private:
  int target_;
  bool allowValid_;
};
}  // namespace

TEST(CAStarAlgorithm, FindsOptimalSolution)
{
  LinePathFinder finder(4);
  PathSolution initial;
  PathSolution final_sol;

  const int ret = finder.getOptimalSolution(initial, final_sol);
  EXPECT_EQ(ret, 1);
  EXPECT_EQ(final_sol.pos, 4);
  EXPECT_DOUBLE_EQ(final_sol.cost, 4.0);
}

TEST(CAStarAlgorithm, NoValidSolutionFound)
{
  // Never accept any children as valid solutions:
  LinePathFinder finder(4, /*allowValid=*/false);
  PathSolution initial;
  PathSolution final_sol;

  const int ret = finder.getOptimalSolution(initial, final_sol);
  EXPECT_EQ(ret, 0);
}

TEST(CAStarAlgorithm, UpperLevelCutoffPreventsSearch)
{
  LinePathFinder finder(4);
  PathSolution initial;
  PathSolution final_sol;

  // The upper bound is stricter than what is achievable, so no solution
  // should be accepted:
  const int ret = finder.getOptimalSolution(initial, final_sol, /*upperLevel=*/0.0);
  EXPECT_EQ(ret, 0);
}

TEST(CAStarAlgorithm, AlreadyPresentDuplicateIsSkipped)
{
  // With target=3, both paths "1+2" and "2+1" reach pos=3 with the same
  // cost=3, exercising the "already present" de-duplication branch.
  LinePathFinder finder(3);
  PathSolution initial;
  PathSolution final_sol;

  const int ret = finder.getOptimalSolution(initial, final_sol);
  EXPECT_EQ(ret, 1);
  EXPECT_EQ(final_sol.pos, 3);
  EXPECT_DOUBLE_EQ(final_sol.cost, 3.0);
}

TEST(CAStarAlgorithm, ImmediateTimeout)
{
  LinePathFinder finder(10);
  PathSolution initial;
  PathSolution final_sol;

  // A zero max computation time makes the very first iteration time out
  // before any solution is found:
  const int ret = finder.getOptimalSolution(
      initial, final_sol, /*upperLevel=*/HUGE_VAL, /*maxComputationTime=*/0.0);
  EXPECT_EQ(ret, 0);
}

TEST(CAStarAlgorithm, TrivialAlreadyEndedInitialSolution)
{
  LinePathFinder finder(0);
  PathSolution initial;
  PathSolution final_sol;

  const int ret = finder.getOptimalSolution(initial, final_sol);
  EXPECT_EQ(ret, 1);
  EXPECT_EQ(final_sol.pos, 0);
  EXPECT_DOUBLE_EQ(final_sol.cost, 0.0);
}
