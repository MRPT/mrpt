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
#include <mrpt/img/CImage.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CReflectivityGridMap2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationReflectivity.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <filesystem>
#include <sstream>

using mrpt::maps::CReflectivityGridMap2D;

// =========================================================================
//  Basic construction
// =========================================================================

TEST(CReflectivityGridMap2D, ConstructionAndGridSize)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.1);
  // Grid should have non-zero extent after construction.
  EXPECT_GT(m.getSizeX(), 0u);
  EXPECT_GT(m.getSizeY(), 0u);
  // Note: isEmpty() is a stub that always returns false for this map type.
}

TEST(CReflectivityGridMap2D, AsString)
{
  CReflectivityGridMap2D m;
  EXPECT_FALSE(m.asString().empty());
}

// =========================================================================
//  cell2float basic sanity
// =========================================================================

TEST(CReflectivityGridMap2D, Cell2Float)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.1);
  // The default log-odds cell value (0) maps to 0.5 probability.
  const int8_t neutral = 0;
  const float p = m.cell2float(neutral);
  EXPECT_NEAR(p, 0.5f, 0.05f);
}

// =========================================================================
//  Insert a CObservationReflectivity and verify the map is no longer empty
// =========================================================================

TEST(CReflectivityGridMap2D, InsertObservation)
{
  CReflectivityGridMap2D m(-5, 5, -5, 5, 0.1);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.8f;  // bright surface
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);

  mrpt::poses::CPose3D robotPose;
  m.insertObservation(obs, robotPose);

  EXPECT_FALSE(m.isEmpty());
}

// =========================================================================
//  clear resets cells to neutral probability
// =========================================================================

TEST(CReflectivityGridMap2D, ClearResetsToNeutral)
{
  CReflectivityGridMap2D m(-5, 5, -5, 5, 0.1);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.9f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  mrpt::poses::CPose3D robotPose;
  m.insertObservation(obs, robotPose);

  // After insertion at least the sensor cell should differ from neutral (0).
  // After clear, internal_clear fills with neutral log-odds => cell2float ~0.5.
  m.clear();

  // Spot-check the centre cell: after clear it should be near neutral (0.5).
  const auto* cell = m.cellByPos(0.0, 0.0);
  ASSERT_NE(cell, nullptr);
  EXPECT_NEAR(m.cell2float(*cell), 0.5f, 0.05f);
}

// =========================================================================
//  getAsImage smoke test
// =========================================================================

TEST(CReflectivityGridMap2D, GetAsImage)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.6f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  mrpt::poses::CPose3D robotPose;
  m.insertObservation(obs, robotPose);

  mrpt::img::CImage img;
  m.getAsImage(img);
  EXPECT_GT(img.getWidth(), 0u);
  EXPECT_GT(img.getHeight(), 0u);

  // Also with verticalFlip and forceRGB
  mrpt::img::CImage img2;
  m.getAsImage(img2, /*verticalFlip=*/true, /*forceRGB=*/true);
  EXPECT_EQ(img2.getWidth(), img.getWidth());
}

// =========================================================================
//  Serialization round-trip
// =========================================================================

TEST(CReflectivityGridMap2D, SerializeRoundTrip)
{
  CReflectivityGridMap2D src(-3, 3, -3, 3, 0.2);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.9f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  mrpt::poses::CPose3D robotPose;
  src.insertObservation(obs, robotPose);

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  CReflectivityGridMap2D dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    mrpt::serialization::CSerializable::Ptr obj;
    ar >> obj;
    ASSERT_NE(obj, nullptr);
    auto* dstPtr = dynamic_cast<CReflectivityGridMap2D*>(obj.get());
    ASSERT_NE(dstPtr, nullptr);
    dst = *dstPtr;
  }

  // The map was non-empty before serialization; verify grid size survived.
  EXPECT_GT(dst.getSizeX(), 0u);
}

// =========================================================================
//  saveMetricMapRepresentationToFile smoke test
// =========================================================================

TEST(CReflectivityGridMap2D, SaveRepresentation)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.2);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.7f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  mrpt::poses::CPose3D robotPose;
  m.insertObservation(obs, robotPose);

  const auto prefix = (std::filesystem::temp_directory_path() / "reflectivity_test").string();
  EXPECT_NO_THROW(m.saveMetricMapRepresentationToFile(prefix));
}

// =========================================================================
//  isEmpty(): stub, always false
// =========================================================================

TEST(CReflectivityGridMap2D, IsEmptyAlwaysFalse)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);
  EXPECT_FALSE(m.isEmpty());

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.5f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  m.insertObservation(obs);
  EXPECT_FALSE(m.isEmpty());
}

// =========================================================================
//  internal_insertObservation(): channel filtering, wrong observation type
// =========================================================================

TEST(CReflectivityGridMap2D, InsertObservationChannelMismatchReturnsFalse)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);
  m.insertionOptions.channel = 3;

  mrpt::obs::CObservationReflectivity obs;
  obs.channel = 9;
  obs.reflectivityLevel = 0.5f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);

  mrpt::poses::CPose3D robotPose;
  EXPECT_FALSE(m.insertObservation(obs, robotPose));
}

TEST(CReflectivityGridMap2D, InsertObservationWrongObservationTypeReturnsFalse)
{
  CReflectivityGridMap2D m;
  mrpt::obs::CObservationOdometry obs;
  mrpt::poses::CPose3D robotPose;
  EXPECT_FALSE(m.insertObservation(obs, robotPose));
}

// NOTE (suspected bug, see CReflectivityGridMap2D.cpp around the resize()
// call in internal_insertObservation): the upper grid bounds are recomputed
// with std::min() instead of std::max(), so a reading beyond the positive
// x/y bounds fails to grow the grid and ends up throwing (via ASSERTMSG_)
// instead of resizing. The symmetric negative-direction case happens to
// work because CDynamicGrid::resize() internally clamps a shrinking upper
// bound back to its previous value. These two tests document the current
// (asymmetric) behavior.
TEST(CReflectivityGridMap2D, InsertObservationBeyondNegativeBoundsResizesOk)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.5f;
  obs.sensorPose = mrpt::poses::CPose3D(-10, -10, 0, 0, 0, 0);

  mrpt::poses::CPose3D robotPose;
  EXPECT_TRUE(m.insertObservation(obs, robotPose));
}

TEST(CReflectivityGridMap2D, InsertObservationBeyondPositiveBoundsThrows)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.5f;
  obs.sensorPose = mrpt::poses::CPose3D(10, 10, 0, 0, 0, 0);

  mrpt::poses::CPose3D robotPose;
  EXPECT_THROW(m.insertObservation(obs, robotPose), std::exception);
}

// =========================================================================
//  internal_computeObservationLikelihood()
// =========================================================================

TEST(CReflectivityGridMap2D, ComputeObservationLikelihoodValid)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);

  mrpt::obs::CObservationReflectivity obs;
  // NOTE (suspected bug): internal_computeObservationLikelihood() asserts
  // reflectivityLevel in [0, 0.1], even though the field is documented (see
  // CObservationReflectivity.h) to range over [0,1]. Stay within the
  // (buggy) asserted range here; a separate test documents the assertion.
  obs.reflectivityLevel = 0.05f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  obs.sensorStdNoise = 0.2f;

  mrpt::poses::CPose3D takenFrom;
  const double lik = m.computeObservationLikelihood(obs, takenFrom);
  EXPECT_LE(lik, 0.0);
}

TEST(CReflectivityGridMap2D, ComputeObservationLikelihoodWrongObservationTypeReturnsZero)
{
  CReflectivityGridMap2D m;
  mrpt::obs::CObservationOdometry obs;
  mrpt::poses::CPose3D takenFrom;
  EXPECT_DOUBLE_EQ(m.computeObservationLikelihood(obs, takenFrom), 0.0);
}

TEST(CReflectivityGridMap2D, ComputeObservationLikelihoodChannelMismatchReturnsZero)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);
  m.insertionOptions.channel = 3;

  mrpt::obs::CObservationReflectivity obs;
  obs.channel = 7;
  obs.reflectivityLevel = 0.05f;

  mrpt::poses::CPose3D takenFrom;
  EXPECT_DOUBLE_EQ(m.computeObservationLikelihood(obs, takenFrom), 0.0);
}

TEST(CReflectivityGridMap2D, ComputeObservationLikelihoodOutOfMapReturnsZero)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.05f;
  obs.sensorPose = mrpt::poses::CPose3D(100, 100, 0, 0, 0, 0);

  mrpt::poses::CPose3D takenFrom;
  EXPECT_DOUBLE_EQ(m.computeObservationLikelihood(obs, takenFrom), 0.0);
}

TEST(CReflectivityGridMap2D, ComputeObservationLikelihoodAboveBuggyBoundThrows)
{
  // Documents the suspected bug in the ASSERT_LE_ bound noted above: a
  // semantically valid reflectivityLevel (in [0,1]) outside [0,0.1] makes
  // the assertion fail.
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.5f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);

  mrpt::poses::CPose3D takenFrom;
  EXPECT_THROW(m.computeObservationLikelihood(obs, takenFrom), std::exception);
}

// =========================================================================
//  compute3DMatchingRatio(): always 0
// =========================================================================

TEST(CReflectivityGridMap2D, Compute3DMatchingRatioAlwaysZero)
{
  CReflectivityGridMap2D m;
  CReflectivityGridMap2D other;
  mrpt::poses::CPose3D pose;
  mrpt::maps::TMatchingRatioParams params;
  EXPECT_FLOAT_EQ(m.compute3DMatchingRatio(&other, pose, params), 0.0f);
}

// =========================================================================
//  getVisualizationInto()
// =========================================================================

TEST(CReflectivityGridMap2D, GetVisualizationEnabled)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.7f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  m.insertObservation(obs);

  m.genericMapParams.enableSaveAs3DObject = true;
  mrpt::viz::CSetOfObjects viz;
  EXPECT_NO_THROW(m.getVisualizationInto(viz));
  EXPECT_FALSE(viz.empty());
}

TEST(CReflectivityGridMap2D, GetVisualizationDisabledIsANoOp)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);
  m.genericMapParams.enableSaveAs3DObject = false;

  mrpt::viz::CSetOfObjects viz;
  m.getVisualizationInto(viz);
  EXPECT_TRUE(viz.empty());
}

// =========================================================================
//  optionsByName()
// =========================================================================

TEST(CReflectivityGridMap2D, OptionsByName)
{
  CReflectivityGridMap2D m;
  const auto opts = m.optionsByName();
  const auto it = opts.find("insertionOptions");
  ASSERT_NE(it, opts.end());
  EXPECT_EQ(it->second, &m.insertionOptions);
}

// =========================================================================
//  TInsertionOptions::loadFromConfigFile() / dumpToTextStream()
// =========================================================================

TEST(CReflectivityGridMap2D, InsertionOptionsLoadFromConfigFileAndDump)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("ins", "channel", 5);

  CReflectivityGridMap2D::TInsertionOptions opts;
  opts.loadFromConfigFile(cfg, "ins");
  EXPECT_EQ(opts.channel, 5);

  std::stringstream ss;
  opts.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());
}

// =========================================================================
//  TMapDefinition: loadFromConfigFile_map_specific(), dumpToTextStream(),
//  internal_CreateFromMapDefinition()
// =========================================================================

TEST(CReflectivityGridMap2D, MapDefinitionLoadAndCreate)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[map_reflectivityMap_00_creationOpts]
min_x=-4
max_x=4
min_y=-4
max_y=4
resolution=0.25

[map_reflectivityMap_00_insertOpts]
channel=2
)"""");

  CReflectivityGridMap2D::TMapDefinition def;
  def.loadFromConfigFile(cfg, "map_reflectivityMap_00");

  EXPECT_DOUBLE_EQ(def.min_x, -4.0);
  EXPECT_DOUBLE_EQ(def.max_x, 4.0);
  EXPECT_EQ(def.insertionOpts.channel, 2);

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());

  const auto createdMap = CReflectivityGridMap2D::CreateFromMapDefinition(def);
  ASSERT_NE(createdMap, nullptr);
  EXPECT_GT(createdMap->getSizeX(), 0u);
}
