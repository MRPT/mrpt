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
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/serialization/CArchive.h>
#include <test_mrpt_common.h>

TEST(PlannerSimple2D, findPath)
{
  using namespace std::string_literals;

  const auto fil = mrpt::mrpt_data_dir() + "/datasets/2006-MalagaCampus.gridmap.gz"s;

  // Load the gridmap:
  mrpt::maps::COccupancyGridMap2D gridmap;

  {
    mrpt::io::CCompressedInputStream f(fil);
    auto arch = mrpt::serialization::archiveFrom(f);
    arch >> gridmap;
  }

  // Find path:
  mrpt::nav::PlannerSimple2D pathPlanning;
  pathPlanning.robotRadius = 0.30f;

  {
    const mrpt::poses::CPose2D origin(20, -110, 0), target(90, 40, 0);
    const auto optPath = pathPlanning.computePath(gridmap, origin, target);

    ASSERT_TRUE(optPath.has_value());
    const auto& thePath = *optPath;
    EXPECT_EQ(thePath.size(), 416U);
    EXPECT_NEAR(thePath.at(0).x, origin.x(), 1.0);
    EXPECT_NEAR(thePath.at(0).y, origin.y(), 1.0);
    EXPECT_NEAR(thePath.back().x, target.x(), 1.0);
    EXPECT_NEAR(thePath.back().y, target.y(), 1.0);
  }
  {
    const mrpt::poses::CPose2D origin(90, 40, 0), target(20, -110, 0);
    const auto optPath =
        pathPlanning.computePath(gridmap, origin, target, 300.0f /* Max. distance */);

    ASSERT_TRUE(optPath.has_value());
    const auto& thePath = *optPath;
    EXPECT_EQ(thePath.size(), 416U);
    EXPECT_NEAR(thePath.at(0).x, origin.x(), 1.0);
    EXPECT_NEAR(thePath.at(0).y, origin.y(), 1.0);
    EXPECT_NEAR(thePath.back().x, target.x(), 1.0);
    EXPECT_NEAR(thePath.back().y, target.y(), 1.0);
  }

  {
    const mrpt::poses::CPose2D origin(20, -110, 0), target(90, 40, 0);
    EXPECT_FALSE(
        pathPlanning.computePath(gridmap, origin, target, 10.0f /* Max. distance */).has_value());
  }

  {
    const mrpt::poses::CPose2D origin(20, -110, 0), target(900, 40, 0);
    EXPECT_FALSE(
        pathPlanning.computePath(gridmap, origin, target, 100.0f /* Max. distance */).has_value());
  }
}
