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
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CRandomFieldGridMap3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

TEST(CRandomFieldGridMap3D, insertCheckMapBounds)
{
  using mrpt::math::TPoint3D;

  mrpt::maps::CRandomFieldGridMap3D::TVoxelInterpolationMethod im =
      mrpt::maps::CRandomFieldGridMap3D::gimNearest;

  mrpt::maps::CRandomFieldGridMap3D grid3d;
  // grid3d.setMinLoggingLevel(mrpt::system::LVL_DEBUG);

  grid3d.setSize(
      -4.0, 4.0, 0.0, 4.0, 0.0, 4.0, 1.0 /*voxel size*/);  // x:[-10,10] * y:[0,5] * z:[0,4]

  const double val = 10.0, var = 1.0;

  // Inside:
  EXPECT_TRUE(grid3d.insertIndividualReading(1.0 * val, var, TPoint3D(2.0, 3.0, 1.0), im, false));
  EXPECT_TRUE(grid3d.insertIndividualReading(2.0 * val, var, TPoint3D(-3.0, 0.4, 1.0), im, false));
  EXPECT_TRUE(grid3d.insertIndividualReading(3.0 * val, var, TPoint3D(3.0, 3.8, 3.0), im, false));
  // Outside:
  EXPECT_FALSE(grid3d.insertIndividualReading(val, var, TPoint3D(-11.0, 2.0, 2.0), im, false));
  EXPECT_FALSE(grid3d.insertIndividualReading(val, var, TPoint3D(11.0, 2.0, 3.0), im, false));
  EXPECT_FALSE(grid3d.insertIndividualReading(val, var, TPoint3D(2.0, -1.0, 11.0), im, false));
  EXPECT_FALSE(grid3d.insertIndividualReading(val, var, TPoint3D(2.0, 6.0, 3.0), im, false));

  grid3d.updateMapEstimation();
  grid3d.saveAsCSV(mrpt::system::getTempFileName());
}

TEST(CRandomFieldGridMap3D, insertPointsAndRead)
{
  using mrpt::math::TPoint3D;

  mrpt::maps::CRandomFieldGridMap3D::TVoxelInterpolationMethod im =
      mrpt::maps::CRandomFieldGridMap3D::gimNearest;
  mrpt::maps::CRandomFieldGridMap3D grid3d;
  // grid3d.setMinLoggingLevel(mrpt::system::LVL_DEBUG);

  grid3d.setSize(
      -4.0, 4.0, 0.0, 4.0, 0.0, 4.0, 1.0 /*voxel size*/);  // x:[-10,10] * y:[0,5] * z:[0,4]

  const double val = 55.0, var = 1.0;

  EXPECT_TRUE(grid3d.insertIndividualReading(val, var, TPoint3D(2.0, 3.0, 1.0), im, false));

  grid3d.insertionOptions.GMRF_skip_variance = true;
  grid3d.updateMapEstimation();

  {
    const double map_value = grid3d.cellByPos(2.0, 3.0, 1.0)->mean_value;
    EXPECT_NEAR(map_value, val, 1e-6);
  }

  // Test after map enlarge:
  grid3d.resize(-5.0, 5.0, -1.0, 5.0, -1.0, 5.0, mrpt::maps::TRandomFieldVoxel(), .0);
  {
    const double map_value = grid3d.cellByPos(2.0, 3.0, 1.0)->mean_value;
    EXPECT_NEAR(map_value, val, 1e-6);
  }
}

TEST(CRandomFieldGridMap3D, InsertionOptionsLoadFromConfigFileAndDump)
{
  mrpt::config::CConfigFileMemory cfg;
  const std::string section = "InsertionOpts";
  cfg.write(section, "GMRF_lambdaPrior", 0.05f);
  cfg.write(section, "GMRF_skip_variance", true);

  mrpt::maps::CRandomFieldGridMap3D::TInsertionOptions opts;
  opts.loadFromConfigFile(cfg, section);

  EXPECT_NEAR(opts.GMRF_lambdaPrior, 0.05, 1e-6);
  EXPECT_TRUE(opts.GMRF_skip_variance);

  std::ostringstream ss;
  opts.dumpToTextStream(ss);
  const std::string text = ss.str();
  EXPECT_NE(text.find("GMRF_lambdaPrior"), std::string::npos);
  EXPECT_NE(text.find("GMRF_skip_variance"), std::string::npos);
}

TEST(CRandomFieldGridMap3D, SaveAsCSVBothFiles)
{
  using mrpt::math::TPoint3D;

  mrpt::maps::CRandomFieldGridMap3D grid3d;
  grid3d.setSize(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0, 1.0);

  EXPECT_TRUE(grid3d.insertIndividualReading(
      10.0, 1.0, TPoint3D(0.0, 0.0, 0.0), mrpt::maps::CRandomFieldGridMap3D::gimNearest, true));

  const std::string filMean = mrpt::system::getTempFileName();
  const std::string filStddev = mrpt::system::getTempFileName();

  EXPECT_TRUE(grid3d.saveAsCSV(filMean, filStddev));
  EXPECT_TRUE(mrpt::system::fileExists(filMean));
  EXPECT_TRUE(mrpt::system::fileExists(filStddev));
}

TEST(CRandomFieldGridMap3D, SaveAsCSVInvalidPathReturnsFalse)
{
  mrpt::maps::CRandomFieldGridMap3D grid3d;
  grid3d.setSize(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0);

  EXPECT_FALSE(grid3d.saveAsCSV("/this/dir/does/not/exist/out.csv"));
}

TEST(CRandomFieldGridMap3D, InsertIndividualReadingWithUpdateMapTrue)
{
  using mrpt::math::TPoint3D;

  mrpt::maps::CRandomFieldGridMap3D grid3d;
  grid3d.setSize(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0, 1.0);
  grid3d.insertionOptions.GMRF_skip_variance = true;

  const double val = 42.0;
  EXPECT_TRUE(grid3d.insertIndividualReading(
      val, 1.0, TPoint3D(0.0, 0.0, 0.0), mrpt::maps::CRandomFieldGridMap3D::gimNearest,
      true /*update_map*/));

  // No separate updateMapEstimation() call is needed: passing
  // update_map=true already refreshed the estimation in-place.
  const double map_value = grid3d.cellByPos(0.0, 0.0, 0.0)->mean_value;
  EXPECT_NEAR(map_value, val, 1e-6);
}

TEST(CRandomFieldGridMap3D, BilinearInterpolationMethodParameterIsCurrentlyUnused)
{
  // The `method` parameter of insertIndividualReading() is presently
  // ignored by the implementation (see the commented-out parameter name in
  // CRandomFieldGridMap3D.cpp), so gimBilinear behaves identically to
  // gimNearest today. This test documents that current behavior without
  // claiming it is a bug to fix.
  using mrpt::math::TPoint3D;

  mrpt::maps::CRandomFieldGridMap3D gridNearest;
  gridNearest.setSize(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0, 1.0);
  gridNearest.insertionOptions.GMRF_skip_variance = true;
  gridNearest.insertIndividualReading(
      33.0, 1.0, TPoint3D(0.0, 0.0, 0.0), mrpt::maps::CRandomFieldGridMap3D::gimNearest, true);

  mrpt::maps::CRandomFieldGridMap3D gridBilinear;
  gridBilinear.setSize(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0, 1.0);
  gridBilinear.insertionOptions.GMRF_skip_variance = true;
  gridBilinear.insertIndividualReading(
      33.0, 1.0, TPoint3D(0.0, 0.0, 0.0), mrpt::maps::CRandomFieldGridMap3D::gimBilinear, true);

  EXPECT_NEAR(
      gridNearest.cellByPos(0.0, 0.0, 0.0)->mean_value,
      gridBilinear.cellByPos(0.0, 0.0, 0.0)->mean_value, 1e-9);
}

TEST(CRandomFieldGridMap3D, OptionsByNameContainsInsertionOptions)
{
  mrpt::maps::CRandomFieldGridMap3D grid3d;
  auto opts = grid3d.optionsByName();
  EXPECT_NE(opts.find("insertionOptions"), opts.end());
}

TEST(CRandomFieldGridMap3D, SerializeRoundTrip)
{
  using mrpt::math::TPoint3D;

  mrpt::maps::CRandomFieldGridMap3D src;
  src.setSize(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0, 1.0);
  src.insertionOptions.GMRF_skip_variance = true;
  src.insertIndividualReading(
      77.0, 1.0, TPoint3D(0.0, 0.0, 0.0), mrpt::maps::CRandomFieldGridMap3D::gimNearest, true);

  mrpt::io::CMemoryStream buf;
  {
    auto arch = mrpt::serialization::archiveFrom(buf);
    arch << src;
  }
  buf.Seek(0);

  mrpt::maps::CRandomFieldGridMap3D dst;
  {
    auto arch = mrpt::serialization::archiveFrom(buf);
    arch >> dst;
  }

  EXPECT_NEAR(
      dst.cellByPos(0.0, 0.0, 0.0)->mean_value, src.cellByPos(0.0, 0.0, 0.0)->mean_value, 1e-6);
  EXPECT_NEAR(dst.insertionOptions.GMRF_lambdaPrior, src.insertionOptions.GMRF_lambdaPrior, 1e-9);
  EXPECT_EQ(dst.insertionOptions.GMRF_skip_variance, src.insertionOptions.GMRF_skip_variance);
}

// Regression: after a resize, the observations kept their old voxel indices,
// so they were applied to other voxels.
TEST(CRandomFieldGridMap3D, ResizeKeepsObservationsAtTheirVoxels)
{
  using M3 = mrpt::maps::CRandomFieldGridMap3D;
  M3 m(0, 2, 0, 2, 0, 2, 1.0);
  m.insertionOptions.GMRF_lambdaPrior = 1.0;
  m.clear();
  ASSERT_TRUE(m.insertIndividualReading(5.0, 1e-4, {1.5, 1.5, 1.5}, M3::gimNearest, false));
  ASSERT_TRUE(m.insertIndividualReading(-5.0, 1e-4, {0.5, 0.5, 0.5}, M3::gimNearest, true));
  const double v1 = m.cellByPos(1.5, 1.5, 1.5)->mean_value;
  const double v2 = m.cellByPos(0.5, 0.5, 0.5)->mean_value;
  EXPECT_NEAR(v1, 5.0, 1e-2);
  EXPECT_NEAR(v2, -5.0, 1e-2);

  m.resize(-2, 2, -2, 2, -2, 2, mrpt::maps::TRandomFieldVoxel(0, 0), 0.0);
  ASSERT_EQ(m.getVoxelCount(), 64u);
  m.updateMapEstimation();

  EXPECT_NEAR(m.cellByPos(1.5, 1.5, 1.5)->mean_value, v1, 1e-2);
  EXPECT_NEAR(m.cellByPos(0.5, 0.5, 0.5)->mean_value, v2, 1e-2);
}

// GMRF estimate (mean and std) against a dense solution of the same model.
TEST(CRandomFieldGridMap3D, GMRF_MatchesDenseSolution)
{
  using M3 = mrpt::maps::CRandomFieldGridMap3D;
  M3 m(0, 2, 0, 2, 0, 2, 1.0);
  const double lambdaPrior = 2.0;
  m.insertionOptions.GMRF_lambdaPrior = lambdaPrior;
  m.clear();
  ASSERT_EQ(m.getVoxelCount(), 8u);

  const std::vector<mrpt::math::TPoint3D> pts = {
      {0.5, 0.5, 0.5},
      {1.5, 0.5, 1.5}
  };
  const std::vector<double> zs = {1.0, -2.0};
  const std::vector<double> vars = {0.5, 0.1};

  const int N = 8;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(N, N);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(N);
  for (size_t q = 0; q < pts.size(); q++)
  {
    ASSERT_TRUE(m.insertIndividualReading(zs[q], vars[q], pts[q], M3::gimNearest, false));
    const auto c = static_cast<int>(
        m.cellAbsIndexFromCXCYCZ(m.x2idx(pts[q].x), m.y2idx(pts[q].y), m.z2idx(pts[q].z)));
    H(c, c) += 1.0 / vars[q];
    b(c) += zs[q] / vars[q];
  }
  m.updateMapEstimation();

  // 6-connectivity prior on the 2x2x2 grid:
  for (int j = 0; j < N; j++)
  {
    for (const int incr : {1, 2, 4})
    {
      const int bit = incr;
      if ((j & bit) != 0)
      {
        continue;
      }
      const int i = j + incr;
      H(i, i) += lambdaPrior;
      H(j, j) += lambdaPrior;
      H(i, j) -= lambdaPrior;
      H(j, i) -= lambdaPrior;
    }
  }
  const Eigen::MatrixXd Hinv = H.inverse();
  const Eigen::VectorXd mu = Hinv * b;
  for (int j = 0; j < N; j++)
  {
    const auto& v = m.data()[static_cast<size_t>(j)];
    EXPECT_NEAR(v.mean_value, mu[j], 1e-9) << "j=" << j;
    EXPECT_NEAR(v.stddev_value, std::sqrt(Hinv(j, j)), 1e-9) << "j=" << j;
  }
}

// Regression: the header of the std. dev. file went to the mean file; also,
// the coordinates are those of the voxel centers.
TEST(CRandomFieldGridMap3D, SaveAsCSVHeadersAndVoxelCenters)
{
  using M3 = mrpt::maps::CRandomFieldGridMap3D;
  M3 m(0, 2, 0, 2, 0, 2, 1.0);
  m.clear();
  ASSERT_TRUE(m.insertIndividualReading(1.0, 1.0, {0.5, 0.5, 0.5}, M3::gimNearest, true));

  const std::string filMean = mrpt::system::getTempFileName();
  const std::string filStd = mrpt::system::getTempFileName();
  ASSERT_TRUE(m.saveAsCSV(filMean, filStd));

  for (const auto& fil : {filMean, filStd})
  {
    std::ifstream f(fil);
    ASSERT_TRUE(f.is_open());
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line))
    {
      lines.push_back(line);
    }
    ASSERT_EQ(lines.size(), 1u + 8u) << fil;
    EXPECT_EQ(lines[0], "x coord, y coord, z coord, scalar");
    EXPECT_EQ(lines[1].rfind("0.500000, 0.500000, 0.500000, ", 0), 0u) << lines[1];
    EXPECT_EQ(lines[8].rfind("1.500000, 1.500000, 1.500000, ", 0), 0u) << lines[8];
  }
}
