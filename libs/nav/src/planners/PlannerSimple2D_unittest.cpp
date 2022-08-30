/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/serialization/CArchive.h>
#include <test_mrpt_common.h>

TEST(PlannerSimple2D, findPath)
{
	using namespace std::string_literals;

	const auto fil = mrpt::UNITTEST_BASEDIR() +
		"/share/mrpt/datasets/2006-MalagaCampus.gridmap.gz"s;

	// Load the gridmap:
	mrpt::maps::COccupancyGridMap2D gridmap;

	{
		mrpt::io::CFileGZInputStream f(fil);
		auto arch = mrpt::serialization::archiveFrom(f);
		arch >> gridmap;
	}

	// Find path:
	mrpt::nav::PlannerSimple2D pathPlanning;
	pathPlanning.robotRadius = 0.30f;

	{
		std::deque<mrpt::math::TPoint2D> thePath;
		bool notFound;
		const mrpt::poses::CPose2D origin(20, -110, 0), target(90, 40, 0);
		pathPlanning.computePath(gridmap, origin, target, thePath, notFound);

		EXPECT_FALSE(notFound);
		EXPECT_EQ(thePath.size(), 416U);
		EXPECT_NEAR(thePath.at(0).x, origin.x(), 1.0);
		EXPECT_NEAR(thePath.at(0).y, origin.y(), 1.0);
		EXPECT_NEAR(thePath.back().x, target.x(), 1.0);
		EXPECT_NEAR(thePath.back().y, target.y(), 1.0);
	}
	{
		std::deque<mrpt::math::TPoint2D> thePath;
		bool notFound;
		const mrpt::poses::CPose2D origin(90, 40, 0), target(20, -110, 0);
		pathPlanning.computePath(
			gridmap, origin, target, thePath, notFound,
			300.0f /* Max. distance */);

		EXPECT_FALSE(notFound);
		EXPECT_EQ(thePath.size(), 416U);
		EXPECT_NEAR(thePath.at(0).x, origin.x(), 1.0);
		EXPECT_NEAR(thePath.at(0).y, origin.y(), 1.0);
		EXPECT_NEAR(thePath.back().x, target.x(), 1.0);
		EXPECT_NEAR(thePath.back().y, target.y(), 1.0);
	}

	{
		std::deque<mrpt::math::TPoint2D> thePath;
		bool notFound;
		const mrpt::poses::CPose2D origin(20, -110, 0), target(90, 40, 0);
		pathPlanning.computePath(
			gridmap, origin, target, thePath, notFound,
			10.0f /* Max. distance */);

		EXPECT_TRUE(notFound);
	}

	{
		std::deque<mrpt::math::TPoint2D> thePath;
		bool notFound;
		const mrpt::poses::CPose2D origin(20, -110, 0), target(900, 40, 0);
		pathPlanning.computePath(
			gridmap, origin, target, thePath, notFound,
			100.0f /* Max. distance */);

		EXPECT_TRUE(notFound);
		EXPECT_EQ(thePath.size(), 0U);
	}
}
