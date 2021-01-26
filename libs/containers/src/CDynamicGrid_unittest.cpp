/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/core/common.h>

template class mrpt::CTraitsTest<mrpt::containers::CDynamicGrid<double>>;

using mrpt::containers::CDynamicGrid;

TEST(CDynamicGrid, GetSetAndResize)
{
	CDynamicGrid<double> grid{-10.0, 10.0, -10.0, 10.0, 0.1};

	*grid.cellByPos(3.0, 4.0) = 8.0;
	*grid.cellByPos(-2.0, -7.0) = 9.0;

	EXPECT_NEAR(*grid.cellByPos(3.0, 4.0), 8.0, 1e-10);
	EXPECT_NEAR(*grid.cellByPos(-2.0, -7.0), 9.0, 1e-10);

	grid.resize(-20.0, 20.0, -20.0, 20.0, 0.0, 0.0);
	EXPECT_NEAR(*grid.cellByPos(3.0, 4.0), 8.0, 1e-10);
	EXPECT_NEAR(*grid.cellByPos(-2.0, -7.0), 9.0, 1e-10);
}

TEST(CDynamicGrid, rangeForLoop)
{
	CDynamicGrid<int> grid{-10.0, 10.0, -10.0, 10.0, 0.1};

	grid.fill(0);
	*grid.cellByPos(3.0, 4.0) = 8;

	std::map<int, int> counter;

	for (const int& d : grid)
		counter[d]++;

	EXPECT_EQ(counter.at(0), 20 * 20 * 10 * 10 - 1);
	EXPECT_EQ(counter.at(8), 1);
}
