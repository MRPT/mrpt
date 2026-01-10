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
  {
    counter[d]++;
  }

  EXPECT_EQ(counter.at(0), 20 * 20 * 10 * 10 - 1);
  EXPECT_EQ(counter.at(8), 1);
}
