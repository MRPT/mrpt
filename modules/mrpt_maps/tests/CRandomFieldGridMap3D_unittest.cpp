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

#include <sstream>

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
