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

#include <cstdio>
#include <fstream>

template class mrpt::CTraitsTest<mrpt::containers::CDynamicGrid<double>>;

using mrpt::containers::CDynamicGrid;

namespace
{
// Minimal mock matrix, just enough to exercise CDynamicGrid::getAsMatrix()
// without depending on mrpt_math:
struct MockMatrix
{
  void setSize(std::size_t rows, std::size_t cols)
  {
    m_rows = rows;
    m_cols = cols;
    m_data.assign(rows * cols, 0.0);
  }
  double& operator()(std::size_t r, std::size_t c) { return m_data.at(r * m_cols + c); }
  double operator()(std::size_t r, std::size_t c) const { return m_data.at(r * m_cols + c); }

  std::size_t m_rows{0};
  std::size_t m_cols{0};
  std::vector<double> m_data;
};
}  // namespace

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

TEST(CDynamicGrid, SetSizeWithFillValue)
{
  const double fillValue = 5.0;
  CDynamicGrid<double> grid;
  grid.setSize(-1.0, 1.0, -1.0, 1.0, 1.0, &fillValue);

  for (const double& cell : grid)
  {
    EXPECT_NEAR(cell, fillValue, 1e-10);
  }
}

TEST(CDynamicGrid, Clear)
{
  CDynamicGrid<double> grid{-1.0, 1.0, -1.0, 1.0, 1.0};
  grid.fill(3.0);
  grid.clear();
  for (const double& cell : grid)
  {
    EXPECT_NEAR(cell, 0.0, 1e-10);
  }
}

TEST(CDynamicGrid, CellByIndex)
{
  CDynamicGrid<double> grid{-1.0, 1.0, -1.0, 1.0, 1.0};
  const auto sx = grid.getSizeX();
  const auto sy = grid.getSizeY();

  *grid.cellByIndex(0, 0) = 42.0;
  EXPECT_NEAR(*grid.cellByIndex(0, 0), 42.0, 1e-10);

  const CDynamicGrid<double>& cgrid = grid;
  EXPECT_NEAR(*cgrid.cellByIndex(0, 0), 42.0, 1e-10);

  // Out of range accesses return nullptr:
  EXPECT_EQ(grid.cellByIndex(static_cast<unsigned int>(sx), 0), nullptr);
  EXPECT_EQ(grid.cellByIndex(0, static_cast<unsigned int>(sy)), nullptr);
  EXPECT_EQ(cgrid.cellByIndex(static_cast<unsigned int>(sx), 0), nullptr);
  EXPECT_EQ(cgrid.cellByIndex(0, static_cast<unsigned int>(sy)), nullptr);
}

TEST(CDynamicGrid, CellByPosOutOfRangeAndConst)
{
  CDynamicGrid<double> grid{-1.0, 1.0, -1.0, 1.0, 1.0};
  EXPECT_EQ(grid.cellByPos(100.0, 100.0), nullptr);
  EXPECT_EQ(grid.cellByPos(0.0, 100.0), nullptr);

  const CDynamicGrid<double>& cgrid = grid;
  EXPECT_EQ(cgrid.cellByPos(100.0, 100.0), nullptr);
  EXPECT_EQ(cgrid.cellByPos(0.0, 100.0), nullptr);
  EXPECT_NE(cgrid.cellByPos(0.0, 0.0), nullptr);
}

TEST(CDynamicGrid, GettersAndIndexConversions)
{
  CDynamicGrid<double> grid{-10.0, 10.0, -20.0, 20.0, 0.5};

  EXPECT_NEAR(grid.getXMin(), -10.0, 1e-10);
  EXPECT_NEAR(grid.getXMax(), 10.0, 1e-10);
  EXPECT_NEAR(grid.getYMin(), -20.0, 1e-10);
  EXPECT_NEAR(grid.getYMax(), 20.0, 1e-10);
  EXPECT_NEAR(grid.getResolution(), 0.5, 1e-10);
  EXPECT_EQ(grid.getSizeX(), 40u);
  EXPECT_EQ(grid.getSizeY(), 80u);

  EXPECT_EQ(grid.x2idx(-10.0), 0);
  EXPECT_EQ(grid.y2idx(-20.0), 0);
  EXPECT_EQ(grid.xy2idx(-10.0, -20.0), 0);

  int cx = -1;
  int cy = -1;
  grid.idx2cxcy(grid.xy2idx(-9.75, -19.75), cx, cy);
  EXPECT_EQ(cx, 0);
  EXPECT_EQ(cy, 0);

  EXPECT_NEAR(grid.idx2x(0), -10.0 + 0.5 * 0.5, 1e-10);
  EXPECT_NEAR(grid.idx2y(0), -20.0 + 0.5 * 0.5, 1e-10);
}

TEST(CDynamicGrid, GetAsMatrix)
{
  CDynamicGrid<double> grid{-1.0, 1.0, -1.0, 1.0, 1.0};
  grid.fill(0.0);
  *grid.cellByIndex(0, 0) = 7.0;

  MockMatrix m;
  grid.getAsMatrix(m);
  EXPECT_EQ(m.m_rows, grid.getSizeY());
  EXPECT_EQ(m.m_cols, grid.getSizeX());
  EXPECT_NEAR(m(0, 0), 7.0, 1e-10);

  // Empty grid: getAsMatrix() just resizes the output and returns:
  CDynamicGrid<double> emptyGrid{0.0, 0.0, 0.0, 0.0, 1.0};
  MockMatrix m2;
  emptyGrid.getAsMatrix(m2);
  EXPECT_EQ(m2.m_rows, 0u);
  EXPECT_EQ(m2.m_cols, 0u);
}

TEST(CDynamicGrid, Cell2FloatDefault)
{
  CDynamicGrid<double> grid{-1.0, 1.0, -1.0, 1.0, 1.0};
  EXPECT_NEAR(grid.cell2float(1.234), 0.0f, 1e-6);
}

TEST(CDynamicGrid, SaveToTextFile)
{
  CDynamicGrid<double> grid{-1.0, 1.0, -1.0, 1.0, 1.0};
  grid.fill(1.5);

  const std::string fileName = "test_CDynamicGrid_save.txt";
  EXPECT_TRUE(grid.saveToTextFile(fileName));

  std::ifstream f(fileName);
  ASSERT_TRUE(f.is_open());
  std::string firstLine;
  std::getline(f, firstLine);
  EXPECT_FALSE(firstLine.empty());
  f.close();
  std::remove(fileName.c_str());

  // Writing to a non-existent directory must fail gracefully:
  EXPECT_FALSE(grid.saveToTextFile("/nonexistent_dir_xyz/out.txt"));
}

TEST(CDynamicGrid, ResizeNoOpWhenAlreadyContained)
{
  CDynamicGrid<double> grid{-10.0, 10.0, -10.0, 10.0, 1.0};
  *grid.cellByIndex(0, 0) = 5.0;

  // New bounds fully contained within the current grid: resize() is a no-op.
  grid.resize(-5.0, 5.0, -5.0, 5.0, 0.0, 0.0);
  EXPECT_EQ(grid.getSizeX(), 20u);
  EXPECT_EQ(grid.getSizeY(), 20u);
}

TEST(CDynamicGrid, ResizeExpandsEachSideIndependently)
{
  // Expand only in +x:
  {
    CDynamicGrid<double> grid{-10.0, 10.0, -10.0, 10.0, 1.0};
    grid.resize(-10.0, 20.0, -10.0, 10.0, -1.0, 0.0);
    EXPECT_NEAR(grid.getXMax(), 20.0, 1e-6);
  }
  // Expand only in -x:
  {
    CDynamicGrid<double> grid{-10.0, 10.0, -10.0, 10.0, 1.0};
    grid.resize(-20.0, 10.0, -10.0, 10.0, -1.0, 0.0);
    EXPECT_NEAR(grid.getXMin(), -20.0, 1e-6);
  }
  // Expand only in +y:
  {
    CDynamicGrid<double> grid{-10.0, 10.0, -10.0, 10.0, 1.0};
    grid.resize(-10.0, 10.0, -10.0, 20.0, -1.0, 0.0);
    EXPECT_NEAR(grid.getYMax(), 20.0, 1e-6);
  }
  // Expand only in -y:
  {
    CDynamicGrid<double> grid{-10.0, 10.0, -10.0, 10.0, 1.0};
    grid.resize(-10.0, 10.0, -20.0, 10.0, -1.0, 0.0);
    EXPECT_NEAR(grid.getYMin(), -20.0, 1e-6);
  }
}

TEST(CDynamicGrid, ResizeWithMarginAndNonAlignedBounds)
{
  CDynamicGrid<double> grid{-10.0, 10.0, -10.0, 10.0, 1.0};
  *grid.cellByPos(3.0, 4.0) = 8.0;

  // Request bounds not aligned to the resolution, with a margin: exercises
  // the floor/ceil margin branches and the fabs-based re-alignment branches.
  grid.resize(-15.3, 15.3, -15.3, 15.3, -1.0, 2.0);

  EXPECT_LE(grid.getXMin(), -15.3);
  EXPECT_GE(grid.getXMax(), 15.3);
  EXPECT_LE(grid.getYMin(), -15.3);
  EXPECT_GE(grid.getYMax(), 15.3);

  // Previous contents must be preserved after the resize:
  EXPECT_NEAR(*grid.cellByPos(3.0, 4.0), 8.0, 1e-6);
}

TEST(CDynamicGrid, ResizeWithNonResolutionAlignedBoundsAndNoMargin)
{
  // With no margin, bounds that are not exact multiples of the resolution
  // must be re-aligned by rounding to the nearest cell boundary.
  CDynamicGrid<double> grid{-10.0, 10.0, -10.0, 10.0, 1.0};
  grid.resize(-15.3, 15.3, -15.3, 15.3, -1.0, 0.0);

  EXPECT_NEAR(grid.getXMin(), -15.0, 1e-6);
  EXPECT_NEAR(grid.getXMax(), 15.0, 1e-6);
  EXPECT_NEAR(grid.getYMin(), -15.0, 1e-6);
  EXPECT_NEAR(grid.getYMax(), 15.0, 1e-6);
}
