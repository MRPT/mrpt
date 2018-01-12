/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/core/common.h>
#include <CTraitsTest.h>

#include <gtest/gtest.h>

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
