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
#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/maps/CHeightGridMap2D_MRF.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <cmath>
#include <sstream>

template <class MAP>
void do_test_insertCheckMapBounds()
{
  mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams pt_params;
  pt_params.update_map_after_insertion = false;

  MAP dem;                                // mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-4.0, 4.0, 0.0, 4.0, 1.0);  // x:[-10,10] * y:[0,5]
  // Inside:
  EXPECT_TRUE(dem.insertIndividualPoint(2.0, 3.0, 56.0, pt_params));
  EXPECT_TRUE(dem.insertIndividualPoint(-3.0, 0.4, 56.0, pt_params));
  EXPECT_TRUE(dem.insertIndividualPoint(3.0, 3.8, 56.0, pt_params));
  // Outside:
  EXPECT_FALSE(dem.insertIndividualPoint(-11.0, 2.0, 56.0, pt_params));
  EXPECT_FALSE(dem.insertIndividualPoint(11.0, 2.0, 56.0, pt_params));
  EXPECT_FALSE(dem.insertIndividualPoint(2.0, -1.0, 56.0, pt_params));
  EXPECT_FALSE(dem.insertIndividualPoint(2.0, 6.0, 56.0, pt_params));
}
TEST(CHeightGridMap2Ds, insertCheckMapBounds)
{
  do_test_insertCheckMapBounds<mrpt::maps::CHeightGridMap2D>();
  do_test_insertCheckMapBounds<mrpt::maps::CHeightGridMap2D_MRF>();
}

template <class MAP>
void do_test_insertPointsAndRead()
{
  MAP dem;
  dem.setSize(0.0, 5.0, 0.0, 5.0, 0.5);  // x:[-10,10] * y:[0,5]
  // Inside:
  const double x = 2.1, y = 3.1, z_write = 56.0;
  dem.insertIndividualPoint(x, y, z_write);
  double z_read;
  bool res = dem.dem_get_z(x, y, z_read);
  EXPECT_TRUE(res);
  EXPECT_NEAR(z_read, z_write, 1e-6);
}
TEST(CHeightGridMap2Ds, insertPointsAndRead)
{
  do_test_insertPointsAndRead<mrpt::maps::CHeightGridMap2D>();
  do_test_insertPointsAndRead<mrpt::maps::CHeightGridMap2D_MRF>();
}

// ------------------- CHeightGridMap2D_Base (via CHeightGridMap2D) -------------------

TEST(CHeightGridMap2Ds, GetMinMaxHeightEmptyMap)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-2.0, 2.0, -2.0, 2.0, 1.0);

  float z_min, z_max;
  EXPECT_FALSE(dem.getMinMaxHeight(z_min, z_max));
  EXPECT_FALSE(dem.getMinMaxHeightOpt().has_value());
}

TEST(CHeightGridMap2Ds, GetMinMaxHeightWithData)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-2.0, 2.0, -2.0, 2.0, 1.0);
  dem.insertIndividualPoint(-1.5, -1.5, 2.0);
  dem.insertIndividualPoint(1.5, 1.5, -3.0);
  dem.insertIndividualPoint(0.5, 0.5, 0.0);

  float z_min, z_max;
  ASSERT_TRUE(dem.getMinMaxHeight(z_min, z_max));
  EXPECT_NEAR(z_min, -3.0f, 1e-4);
  EXPECT_NEAR(z_max, 2.0f, 1e-4);

  const auto opt = dem.getMinMaxHeightOpt();
  ASSERT_TRUE(opt.has_value());
  EXPECT_NEAR(opt->first, -3.0f, 1e-4);
  EXPECT_NEAR(opt->second, 2.0f, 1e-4);
}

TEST(CHeightGridMap2Ds, IntersectLine3DEmptyMapReturnsFalse)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-4.0, 4.0, 0.0, 4.0, 1.0);

  const mrpt::math::TLine3D ray(
      mrpt::math::TPoint3D(2.0, 3.0, 10.0), mrpt::math::TPoint3D(2.0, 3.0, -10.0));
  mrpt::math::TObject3D obj;
  EXPECT_FALSE(dem.intersectLine3D(ray, obj));
}

TEST(CHeightGridMap2Ds, IntersectLine3DHitsCell)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-4.0, 4.0, 0.0, 4.0, 1.0);
  dem.insertIndividualPoint(2.0, 3.0, 2.5);

  // Cast a ray straight down through the cell that holds data:
  const mrpt::math::TLine3D ray(
      mrpt::math::TPoint3D(2.0, 3.0, 10.0), mrpt::math::TPoint3D(2.0, 3.0, -10.0));
  mrpt::math::TObject3D obj;
  ASSERT_TRUE(dem.intersectLine3D(ray, obj));

  mrpt::math::TPoint3D pt;
  ASSERT_TRUE(obj.getPoint(pt));
  EXPECT_NEAR(pt.x, 2.0, 0.5);
  EXPECT_NEAR(pt.y, 3.0, 0.5);
  EXPECT_NEAR(pt.z, 2.5, 1e-3);
}

TEST(CHeightGridMap2Ds, IntersectLine3DMissesUnobservedCell)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-4.0, 4.0, 0.0, 4.0, 1.0);
  // Only one cell has data, far from the ray cast below:
  dem.insertIndividualPoint(2.0, 3.0, 2.5);

  const mrpt::math::TLine3D ray(
      mrpt::math::TPoint3D(-3.0, 0.5, 10.0), mrpt::math::TPoint3D(-3.0, 0.5, -10.0));
  mrpt::math::TObject3D obj;
  EXPECT_FALSE(dem.intersectLine3D(ray, obj));
}

TEST(CHeightGridMap2Ds, DemInternalInsertObservationNoPointsReturnsFalse)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-2.0, 2.0, -2.0, 2.0, 1.0);

  // Odometry observations carry no spatial (x,y,z) information, so this
  // yields zero points to insert:
  mrpt::obs::CObservationOdometry obs;
  EXPECT_FALSE(dem.dem_internal_insertObservation(obs));
  EXPECT_EQ(dem.countObservedCells(), 0u);
}

TEST(CHeightGridMap2Ds, DemInternalInsertObservationInsertsPoints)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-20.0, 20.0, -20.0, 20.0, 1.0);

  mrpt::obs::CObservation2DRangeScan scan;
  mrpt::obs::stock_observations::example2DRangeScan(scan);

  EXPECT_TRUE(dem.dem_internal_insertObservation(scan));
  EXPECT_GT(dem.countObservedCells(), 0u);
}

// ------------------- CHeightGridMap2D specifics -------------------

TEST(CHeightGridMap2D, InsertIndividualPointFilterByHeight)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-2.0, 2.0, -2.0, 2.0, 1.0);
  dem.insertionOptions.filterByHeight = true;
  dem.insertionOptions.z_min = -1.0f;
  dem.insertionOptions.z_max = 1.0f;

  // In map bounds, but outside the height filter range: the point is
  // discarded, although the method still reports success (in-bounds insert
  // attempt):
  EXPECT_TRUE(dem.insertIndividualPoint(0.5, 0.5, 5.0));
  double z;
  EXPECT_FALSE(dem.dem_get_z(0.5, 0.5, z));

  // Inside the height filter range: it gets stored.
  EXPECT_TRUE(dem.insertIndividualPoint(0.5, 0.5, 0.3));
  ASSERT_TRUE(dem.dem_get_z(0.5, 0.5, z));
  EXPECT_NEAR(z, 0.3, 1e-6);
}

TEST(CHeightGridMap2D, InsertIndividualPointRunningMean)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-2.0, 2.0, -2.0, 2.0, 1.0);

  // Three readings falling into the same cell exercise the running mean
  // update branch (cell->w already nonzero):
  dem.insertIndividualPoint(0.5, 0.5, 1.0);
  dem.insertIndividualPoint(0.5, 0.5, 3.0);
  dem.insertIndividualPoint(0.5, 0.5, 5.0);

  double z;
  ASSERT_TRUE(dem.dem_get_z(0.5, 0.5, z));
  EXPECT_NEAR(z, 3.0, 1e-3);
}

TEST(CHeightGridMap2D, ComputeObservationLikelihoodThrows)
{
  mrpt::maps::CHeightGridMap2D dem;
  mrpt::obs::CObservationOdometry obs;
  EXPECT_THROW(dem.computeObservationLikelihood(obs, mrpt::poses::CPose3D()), std::exception);
}

TEST(CHeightGridMap2D, SerializeRoundTrip)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-1.0, 1.0, -1.0, 1.0, 0.5);
  dem.insertIndividualPoint(0.5, 0.5, 3.0);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << dem;
  buf.Seek(0);

  mrpt::maps::CHeightGridMap2D dem2;
  arch >> dem2;

  EXPECT_NEAR(dem2.getXMin(), dem.getXMin(), 1e-6);
  EXPECT_EQ(dem2.countObservedCells(), dem.countObservedCells());
  double z;
  ASSERT_TRUE(dem2.dem_get_z(0.5, 0.5, z));
  EXPECT_NEAR(z, 3.0, 1e-3);
}

TEST(CHeightGridMap2D, SaveMetricMapRepresentationToFile)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-1.0, 1.0, -1.0, 1.0, 0.5);
  dem.insertIndividualPoint(0.0, 0.0, 2.0);

  const std::string prefix = mrpt::system::getTempFileName();
  dem.saveMetricMapRepresentationToFile(prefix);

  const std::string meanFile = prefix + "_mean.txt";
  EXPECT_TRUE(mrpt::system::fileExists(meanFile));
  mrpt::system::deleteFile(meanFile);
}

TEST(CHeightGridMap2D, GetVisualizationIntoMeshMode)
{
  const bool prevMode = mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH();
  mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH(true);

  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-2.0, 2.0, -2.0, 2.0, 0.5);
  dem.insertIndividualPoint(0.0, 0.0, 1.5);

  mrpt::viz::CSetOfObjects o;
  dem.getVisualizationInto(o);
  EXPECT_EQ(o.size(), 1u);

  mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH(prevMode);
}

TEST(CHeightGridMap2D, GetVisualizationIntoPointCloudMode)
{
  const bool prevMode = mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH();
  mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH(false);

  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-2.0, 2.0, -2.0, 2.0, 0.5);
  dem.insertIndividualPoint(0.0, 0.0, 1.5);
  dem.insertIndividualPoint(1.0, 1.0, -1.0);

  mrpt::viz::CSetOfObjects o;
  dem.getVisualizationInto(o);
  EXPECT_EQ(o.size(), 1u);

  mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH(prevMode);
}

TEST(CHeightGridMap2D, GetVisualizationIntoDisabledReturnsEmpty)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(-1.0, 1.0, -1.0, 1.0, 0.5);
  dem.insertIndividualPoint(0.0, 0.0, 1.0);
  dem.genericMapParams.enableSaveAs3DObject = false;

  mrpt::viz::CSetOfObjects o;
  dem.getVisualizationInto(o);
  EXPECT_EQ(o.size(), 0u);
}

TEST(CHeightGridMap2D, CountObservedCells)
{
  mrpt::maps::CHeightGridMap2D dem;
  dem.setSize(0.0, 4.0, 0.0, 4.0, 1.0);
  EXPECT_EQ(dem.countObservedCells(), 0u);

  dem.insertIndividualPoint(0.5, 0.5, 1.0);
  dem.insertIndividualPoint(2.5, 2.5, 2.0);
  dem.insertIndividualPoint(2.6, 2.6, 2.5);  // Same cell as the previous one.

  EXPECT_EQ(dem.countObservedCells(), 2u);
}

TEST(CHeightGridMap2D, OptionsByName)
{
  mrpt::maps::CHeightGridMap2D dem;
  auto opts = dem.optionsByName();
  ASSERT_EQ(opts.count("insertionOptions"), 1u);
  EXPECT_EQ(opts.at("insertionOptions"), &dem.insertionOptions);
}

TEST(CHeightGridMap2D, TInsertionOptionsLoadFromConfigFileGrayscale)
{
  mrpt::maps::CHeightGridMap2D::TInsertionOptions opts;
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "filterByHeight", true);
  cfg.write("sec", "z_min", -2.0f);
  cfg.write("sec", "z_max", 2.0f);
  cfg.write("sec", "colorMap", "grayscale");

  opts.loadFromConfigFile(cfg, "sec");
  EXPECT_TRUE(opts.filterByHeight);
  EXPECT_FLOAT_EQ(opts.z_min, -2.0f);
  EXPECT_FLOAT_EQ(opts.z_max, 2.0f);
  EXPECT_EQ(opts.colorMap, mrpt::img::cmGRAYSCALE);

  std::stringstream ss;
  opts.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("filterByHeight"), std::string::npos);
}

TEST(CHeightGridMap2D, MapDefinitionLoadDumpAndCreate)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("test_creationOpts", "min_x", -3.0);
  cfg.write("test_creationOpts", "max_x", 3.0);
  cfg.write("test_creationOpts", "min_y", -2.0);
  cfg.write("test_creationOpts", "max_y", 2.0);
  cfg.write("test_creationOpts", "resolution", 0.5);
  cfg.write("test_creationOpts", "mapType", "mrSimpleAverage");
  cfg.write("test_insertOpts", "filterByHeight", true);
  cfg.write("test_insertOpts", "z_min", -1.0f);
  cfg.write("test_insertOpts", "z_max", 1.0f);

  mrpt::maps::CHeightGridMap2D::TMapDefinition def;
  def.loadFromConfigFile(cfg, "test");

  EXPECT_DOUBLE_EQ(def.min_x, -3.0);
  EXPECT_DOUBLE_EQ(def.max_x, 3.0);
  EXPECT_DOUBLE_EQ(def.resolution, 0.5);
  EXPECT_TRUE(def.insertionOpts.filterByHeight);

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("MAP TYPE"), std::string::npos);

  auto mapObj = mrpt::maps::CHeightGridMap2D::internal_CreateFromMapDefinition(def);
  ASSERT_TRUE(mapObj);
  auto dem = std::dynamic_pointer_cast<mrpt::maps::CHeightGridMap2D>(mapObj);
  ASSERT_TRUE(dem);
  EXPECT_NEAR(dem->getXMin(), -3.0, 1e-9);
  EXPECT_NEAR(dem->getXMax(), 3.0, 1e-9);
  EXPECT_TRUE(dem->insertionOptions.filterByHeight);
}

// ------------------- CHeightGridMap2D_MRF specifics -------------------

TEST(CHeightGridMap2D_MRF, ComputeObservationLikelihoodThrows)
{
  mrpt::maps::CHeightGridMap2D_MRF dem;
  mrpt::obs::CObservationOdometry obs;
  EXPECT_THROW(dem.computeObservationLikelihood(obs, mrpt::poses::CPose3D()), std::exception);
}

TEST(CHeightGridMap2D_MRF, DemUpdateMapRunsSmoothing)
{
  mrpt::maps::CHeightGridMap2D_MRF dem;
  dem.setSize(-2.0, 2.0, -2.0, 2.0, 0.5);

  mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams pt_params;
  pt_params.update_map_after_insertion = false;
  dem.insertIndividualPoint(-1.0, -1.0, 2.0, pt_params);
  dem.insertIndividualPoint(1.0, 1.0, 4.0, pt_params);

  // Run the actual MRF smoothing pass (unlike CHeightGridMap2D's no-op):
  EXPECT_NO_THROW(dem.dem_update_map());

  double z;
  ASSERT_TRUE(dem.dem_get_z(-1.0, -1.0, z));
  EXPECT_TRUE(std::isfinite(z));
  ASSERT_TRUE(dem.dem_get_z(1.0, 1.0, z));
  EXPECT_TRUE(std::isfinite(z));
}

TEST(CHeightGridMap2D_MRF, SerializeRoundTrip)
{
  mrpt::maps::CHeightGridMap2D_MRF dem;
  dem.setSize(-2.0, 2.0, -2.0, 2.0, 0.5);
  dem.insertIndividualPoint(0.5, 0.5, 3.0);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << dem;
  buf.Seek(0);

  mrpt::maps::CHeightGridMap2D_MRF dem2;
  arch >> dem2;

  EXPECT_NEAR(dem2.getXMin(), dem.getXMin(), 1e-6);
  double z;
  EXPECT_TRUE(dem2.dem_get_z(0.5, 0.5, z));
}

TEST(CHeightGridMap2D_MRF, GetVisualizationIntoAndGetAs3DObject)
{
  mrpt::maps::CHeightGridMap2D_MRF dem;
  dem.setSize(-2.0, 2.0, -2.0, 2.0, 0.5);
  dem.insertIndividualPoint(0.5, 0.5, 3.0);

  mrpt::viz::CSetOfObjects o;
  dem.getVisualizationInto(o);
  EXPECT_GT(o.size(), 0u);

  mrpt::viz::CSetOfObjects meanObj;
  mrpt::viz::CSetOfObjects varObj;
  dem.getAs3DObject(meanObj, varObj);
  EXPECT_GT(meanObj.size(), 0u);

  dem.genericMapParams.enableSaveAs3DObject = false;
  mrpt::viz::CSetOfObjects o2;
  dem.getVisualizationInto(o2);
  EXPECT_EQ(o2.size(), 0u);
}

TEST(CHeightGridMap2D_MRF, OptionsByName)
{
  mrpt::maps::CHeightGridMap2D_MRF dem;
  auto opts = dem.optionsByName();
  ASSERT_EQ(opts.count("insertionOptions"), 1u);
  EXPECT_EQ(opts.at("insertionOptions"), &dem.insertionOptions);
}

TEST(CHeightGridMap2D_MRF, InsertionOptionsLoadDump)
{
  mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions opts;
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "sigma", 0.2f);
  opts.loadFromConfigFile(cfg, "sec");
  EXPECT_NEAR(opts.sigma, 0.2, 1e-6);

  std::stringstream ss;
  opts.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CHeightGridMap2D_MRF, MapDefinitionLoadDumpAndCreate)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("test_creationOpts", "run_map_estimation_at_ctor", false);
  cfg.write("test_creationOpts", "min_x", -3.0);
  cfg.write("test_creationOpts", "max_x", 3.0);
  cfg.write("test_creationOpts", "min_y", -2.0);
  cfg.write("test_creationOpts", "max_y", 2.0);
  cfg.write("test_creationOpts", "resolution", 0.5);

  mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition def;
  def.loadFromConfigFile(cfg, "test");

  EXPECT_DOUBLE_EQ(def.min_x, -3.0);
  EXPECT_DOUBLE_EQ(def.max_x, 3.0);
  EXPECT_FALSE(def.run_map_estimation_at_ctor);

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("MAP TYPE"), std::string::npos);

  auto mapObj = mrpt::maps::CHeightGridMap2D_MRF::internal_CreateFromMapDefinition(def);
  ASSERT_TRUE(mapObj);
  auto dem = std::dynamic_pointer_cast<mrpt::maps::CHeightGridMap2D_MRF>(mapObj);
  ASSERT_TRUE(dem);
  EXPECT_NEAR(dem->getXMin(), -3.0, 1e-9);
  EXPECT_NEAR(dem->getXMax(), 3.0, 1e-9);
}
