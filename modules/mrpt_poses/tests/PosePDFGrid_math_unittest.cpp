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

// Math checks for grid PDFs: moments, index <-> coordinate mapping, sampling.

#include <gtest/gtest.h>
#include <mrpt/poses/CPose3DPDFGrid.h>
#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

// Two equal-mass cells at x=-0.5 and x=+0.5: mean 0, var_x 0.25.
TEST(CPosePDFGridMath, CovarianceAndMean_TwoCells)
{
  CPosePDFGrid g(-1, 1, -1, 1, 0.5, mrpt::DEG2RAD(90.0));
  g.fill(0.0);
  *g.getByPos(-0.5, 0.0, 0.0) = 0.5;
  *g.getByPos(0.5, 0.0, 0.0) = 0.5;

  CPose2D m;
  g.getMean(m);
  EXPECT_NEAR(m.x(), 0.0, 1e-9);

  const auto [C, mean] = g.getCovarianceAndMean();
  EXPECT_NEAR(mean.x(), 0.0, 1e-9);
  EXPECT_NEAR(C(0, 0), 0.25, 1e-9) << C.inMatlabFormat();
  EXPECT_NEAR(C(1, 1), 0.0, 1e-9);
  EXPECT_NEAR(C(2, 2), 0.0, 1e-9);
}

TEST(CPose3DPDFGridMath, CovarianceAndMean_TwoCells)
{
  CPose3DPDFGrid g(
      TPose3D(-1, -1, -1, -M_PI, -0.5 * M_PI, -0.5 * M_PI),
      TPose3D(1, 1, 1, M_PI, 0.5 * M_PI, 0.5 * M_PI), 0.5, mrpt::DEG2RAD(30.0));
  g.fill(0.0);
  *g.getByPos(-0.5, 0, 0, 0, 0, 0) = 0.5;
  *g.getByPos(0.5, 0, 0, 0, 0, 0) = 0.5;

  const auto [C, mean] = g.getCovarianceAndMean();
  EXPECT_NEAR(mean.x(), 0.0, 1e-9);
  EXPECT_NEAR(C(0, 0), 0.25, 1e-9);
  EXPECT_NEAR(C(3, 3), 0.0, 1e-9);
}

// Every coordinate inside [min,max] must map to a valid cell whose center is
// within half a cell of it, also when the bounds are not multiples of the
// resolution.
TEST(CPosePDFGridMath, IndexCoordinateMapping)
{
  for (const double xMin : {-1.0, -1.2, -0.9})
  {
    const double xMax = -xMin;
    const double res = 0.5;
    CPosePDFGrid g(xMin, xMax, xMin, xMax, res, mrpt::DEG2RAD(45.0));
    for (size_t i = 0; i < g.getSizeX(); i++)
    {
      EXPECT_EQ(g.x2idx(g.idx2x(i)), i);
    }
    for (double x = xMin; x <= xMax + 1e-12; x += 0.05)
    {
      bool ok = true;
      size_t idx = 0;
      try
      {
        idx = g.x2idx(x);
      }
      catch (const std::exception&)
      {
        ok = false;
      }
      EXPECT_TRUE(ok) << "xMin=" << xMin << " x=" << x << " sizeX=" << g.getSizeX();
      if (ok)
      {
        EXPECT_LE(std::abs(g.idx2x(idx) - x), 0.5 * res + 1e-9);
      }
    }
  }
}

// Sampling frequencies follow the cell masses.
TEST(CPosePDFGridMath, DrawSingleSample_Frequencies)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  CPosePDFGrid g(-1, 1, -1, 1, 0.5, mrpt::DEG2RAD(90.0));
  g.fill(0.0);
  *g.getByPos(-0.5, 0.0, 0.0) = 1.0;
  *g.getByPos(0.5, 0.5, mrpt::DEG2RAD(90.0)) = 3.0;
  g.normalize();

  const size_t N = 20000;
  size_t n = 0;
  for (size_t i = 0; i < N; i++)
  {
    CPose2D p;
    g.drawSingleSample(p);
    if (p.x() > 0)
    {
      n++;
      EXPECT_NEAR(p.phi(), M_PI / 2, 1e-9);
    }
  }
  EXPECT_NEAR(static_cast<double>(n) / N, 0.75, 0.015);
}

// Shifting a grid PDF by an exact multiple of the cell size moves the mass.
TEST(CPosePDFGridMath, ChangeCoordinatesReference_Translation)
{
  CPosePDFGrid g(-2, 2, -2, 2, 0.5, mrpt::DEG2RAD(90.0));
  g.fill(0.0);
  *g.getByPos(0.0, 0.5, 0.0) = 1.0;
  g.changeCoordinatesReference(CPose3D(1.0, -0.5, 0, 0, 0, 0));
  EXPECT_NEAR(*g.getByPos(1.0, 0.0, 0.0), 1.0, 1e-12);
}

// getAsMatrix() must accept any valid heading, e.g. pi with 3 heading cells.
TEST(CPosePDFGridMath, GetAsMatrix_ValidHeading)
{
  CPosePDFGrid g(-1, 1, -1, 1, 0.5, mrpt::DEG2RAD(180.0));
  ASSERT_EQ(g.getSizePhi(), 3u);
  CMatrixDouble m;
  bool ok = true;
  try
  {
    g.getAsMatrix(M_PI, m);
  }
  catch (const std::exception&)
  {
    ok = false;
  }
  EXPECT_TRUE(ok);
}

// Out-of-range cell indices, including negative ones, must be rejected.
TEST(CPose3DPDFGridMath, GetByIndex_OutOfRange)
{
  CPose3DPDFGrid g(
      TPose3D(-1, -1, -1, -M_PI, -0.5 * M_PI, -0.5 * M_PI),
      TPose3D(1, 1, 1, M_PI, 0.5 * M_PI, 0.5 * M_PI), 0.5, mrpt::DEG2RAD(30.0));
  EXPECT_NO_THROW(g.getByIndex(0, 0, 0, 0, 0, 0));
  EXPECT_THROW(g.getByIndex(-1, 0, 0, 0, 0, 0), std::exception);
  EXPECT_THROW(g.getByIndex(0, 0, 0, 0, 0, -1), std::exception);
  EXPECT_THROW(g.getByIndex(static_cast<int>(g.getSizeX()), 0, 0, 0, 0, 0), std::exception);
}
