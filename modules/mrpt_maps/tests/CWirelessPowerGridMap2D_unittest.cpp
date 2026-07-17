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
#include <mrpt/maps/CWirelessPowerGridMap2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <sstream>

using mrpt::maps::CWirelessPowerGridMap2D;

// =========================================================================
//  Basic construction
// =========================================================================

TEST(CWirelessPowerGridMap2D, ConstructionAndGridSize)
{
  CWirelessPowerGridMap2D m(mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2, 2, -2, 2, 1.0);
  EXPECT_GT(m.getSizeX(), 0u);
  EXPECT_GT(m.getSizeY(), 0u);
}

TEST(CWirelessPowerGridMap2D, AsString)
{
  CWirelessPowerGridMap2D m;
  EXPECT_FALSE(m.asString().empty());
}

// =========================================================================
//  internal_insertObservation()
// =========================================================================

TEST(CWirelessPowerGridMap2D, InsertObservationValid)
{
  CWirelessPowerGridMap2D m(mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2, 2, -2, 2, 1.0);

  mrpt::obs::CObservationWirelessPower obs;
  obs.power = 55.0;
  obs.sensorPoseOnRobot = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);

  EXPECT_TRUE(m.insertObservation(obs));
}

TEST(CWirelessPowerGridMap2D, InsertObservationWrongObservationTypeReturnsFalse)
{
  CWirelessPowerGridMap2D m(mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2, 2, -2, 2, 1.0);

  mrpt::obs::CObservationOdometry obs;
  EXPECT_FALSE(m.insertObservation(obs));
}

// =========================================================================
//  internal_computeObservationLikelihood(): not implemented
// =========================================================================

TEST(CWirelessPowerGridMap2D, ComputeObservationLikelihoodThrows)
{
  CWirelessPowerGridMap2D m;
  mrpt::obs::CObservationWirelessPower obs;
  mrpt::poses::CPose3D pose;

  EXPECT_THROW(m.computeObservationLikelihood(obs, pose), std::exception);
}

// =========================================================================
//  Visualization
// =========================================================================

TEST(CWirelessPowerGridMap2D, VisualizationEnabled)
{
  CWirelessPowerGridMap2D m(mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2, 2, -2, 2, 1.0);

  mrpt::obs::CObservationWirelessPower obs;
  obs.power = 40.0;
  m.insertObservation(obs);

  m.genericMapParams.enableSaveAs3DObject = true;
  mrpt::viz::CSetOfObjects viz;
  EXPECT_NO_THROW(m.getVisualizationInto(viz));
}

TEST(CWirelessPowerGridMap2D, VisualizationDisabledIsANoOp)
{
  CWirelessPowerGridMap2D m(mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2, 2, -2, 2, 1.0);
  m.genericMapParams.enableSaveAs3DObject = false;

  mrpt::viz::CSetOfObjects viz;
  m.getVisualizationInto(viz);
  EXPECT_TRUE(viz.empty());
}

// =========================================================================
//  optionsByName()
// =========================================================================

TEST(CWirelessPowerGridMap2D, OptionsByName)
{
  CWirelessPowerGridMap2D m;
  const auto opts = m.optionsByName();
  const auto it = opts.find("insertionOptions");
  ASSERT_NE(it, opts.end());
  EXPECT_EQ(it->second, &m.insertionOptions);
}

// =========================================================================
//  Serialization round-trip
// =========================================================================

TEST(CWirelessPowerGridMap2D, SerializeRoundTrip)
{
  CWirelessPowerGridMap2D src(mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2, 2, -2, 2, 1.0);

  mrpt::obs::CObservationWirelessPower obs;
  obs.power = 30.0;
  src.insertObservation(obs);

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  CWirelessPowerGridMap2D dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    mrpt::serialization::CSerializable::Ptr obj;
    ar >> obj;
    ASSERT_NE(obj, nullptr);
    auto* dstPtr = dynamic_cast<CWirelessPowerGridMap2D*>(obj.get());
    ASSERT_NE(dstPtr, nullptr);
    dst = *dstPtr;
  }

  EXPECT_EQ(dst.getSizeX(), src.getSizeX());
  EXPECT_EQ(dst.getSizeY(), src.getSizeY());
}

// =========================================================================
//  TInsertionOptions::loadFromConfigFile() / dumpToTextStream()
// =========================================================================

TEST(CWirelessPowerGridMap2D, InsertionOptionsLoadFromConfigFileAndDump)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("ins", "R_min", 0.0f);
  cfg.write("ins", "R_max", 100.0f);
  cfg.write("ins", "sigma", 0.25f);

  CWirelessPowerGridMap2D::TInsertionOptions opts;
  opts.loadFromConfigFile(cfg, "ins");

  EXPECT_FLOAT_EQ(opts.R_min, 0.0f);
  EXPECT_FLOAT_EQ(opts.R_max, 100.0f);
  EXPECT_FLOAT_EQ(opts.sigma, 0.25f);

  std::stringstream ss;
  opts.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());
}

// =========================================================================
//  TMapDefinition: loadFromConfigFile_map_specific(), dumpToTextStream(),
//  internal_CreateFromMapDefinition()
// =========================================================================

TEST(CWirelessPowerGridMap2D, MapDefinitionLoadAndCreate)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[map_wifiGrid_00_creationOpts]
min_x=-3
max_x=3
min_y=-3
max_y=3
resolution=0.5
mapType=mrKalmanFilter

[map_wifiGrid_00_insertOpts]
R_max=80
)"""");

  CWirelessPowerGridMap2D::TMapDefinition def;
  def.loadFromConfigFile(cfg, "map_wifiGrid_00");

  EXPECT_DOUBLE_EQ(def.min_x, -3.0);
  EXPECT_DOUBLE_EQ(def.max_x, 3.0);
  EXPECT_FLOAT_EQ(def.insertionOpts.R_max, 80.0f);

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());

  const auto createdMap = CWirelessPowerGridMap2D::CreateFromMapDefinition(def);
  ASSERT_NE(createdMap, nullptr);
  EXPECT_GT(createdMap->getSizeX(), 0u);
}
