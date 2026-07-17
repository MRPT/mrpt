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
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <sstream>

const double xMin = -4.0, xMax = 4.0, yMin = -4.0, yMax = 4.0;
const double val = 0.2, sigma = 1.0;
const bool ti = true;   // time invariant
const bool um = false;  // update map. Manual call to updateMapEstimation()

static void test_CGasConcentrationGridMap2D_insertAndRead(
    mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation mapType,
    const double resolution,
    const std::function<double(const mrpt::maps::TRandomFieldCell&)>& cellToValue)
{
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mapType, static_cast<float>(xMin), static_cast<float>(xMax), static_cast<float>(yMin),
      static_cast<float>(yMax), static_cast<float>(resolution));

  // Inside:
  grid.insertIndividualReading(1.0 * val, {2.0, 3.0}, um, ti, sigma);
  grid.insertIndividualReading(2.0 * val, {1.0, 0.5}, um, ti, sigma);
  grid.insertIndividualReading(3.0 * val, {-1.0, -2.0}, um, ti, sigma);

  grid.insertionOptions.GMRF_skip_variance = true;
  grid.updateMapEstimation();

  for (int i = 0; i < 2; i++)
  {
    {
      const double map_value = cellToValue(*grid.cellByPos(2.0, 3.0));
      EXPECT_NEAR(map_value, 1.0 * val, 1e-2) << "mapType: " << mapType;
    }
    {
      const double map_value = cellToValue(*grid.cellByPos(1.0, 0.5));
      EXPECT_NEAR(map_value, 2.0 * val, 1e-2) << "mapType: " << mapType;
    }
    {
      const double map_value = cellToValue(*grid.cellByPos(-1.0, -2.0));
      EXPECT_NEAR(map_value, 3.0 * val, 1e-2) << "mapType: " << mapType;

      // Test after map enlarge:
      grid.resize(-5.0, 5.0, -5.0, 5.0, {}, .0);
    }
  }
}

TEST(CGasConcentrationGridMap2D, insertAndRead)
{
  test_CGasConcentrationGridMap2D_insertAndRead(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, 1.0,
      [](const mrpt::maps::TRandomFieldCell& c) { return c.kf_mean(); });
  test_CGasConcentrationGridMap2D_insertAndRead(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanApproximate, 1.0,
      [](const mrpt::maps::TRandomFieldCell& c) { return c.kf_mean(); });
  test_CGasConcentrationGridMap2D_insertAndRead(
      mrpt::maps::CRandomFieldGridMap2D::mrGMRF_SD, 0.5,
      [](const mrpt::maps::TRandomFieldCell& c) { return c.gmrf_mean(); });

#if 0
	test_CGasConcentrationGridMap2D_insertAndRead(
		mrpt::maps::CRandomFieldGridMap2D::mrKernelDM, 0.1,
		[](const mrpt::maps::TRandomFieldCell& c) { return c.dm_mean(); });
	test_CGasConcentrationGridMap2D_insertAndRead(
		mrpt::maps::CRandomFieldGridMap2D::mrKernelDMV, 0.1,
		[](const mrpt::maps::TRandomFieldCell& c) { return c.dm_mean(); });
#endif
}

// =========================================================================
//  internal_insertObservation(): gas-sensor-specific logic
// =========================================================================

namespace
{
mrpt::obs::CObservationGasSensors::TObservationENose BuildENose(
    const std::vector<float>& readings, const std::vector<int>& sensorTypes)
{
  mrpt::obs::CObservationGasSensors::TObservationENose enose;
  enose.eNosePoseOnTheRobot = mrpt::math::TPose3D(0, 0, 0, 0, 0, 0);
  enose.readingsVoltage = readings;
  enose.sensorTypes = sensorTypes;
  enose.isActive = true;
  return enose;
}
}  // namespace

TEST(CGasConcentrationGridMap2D, InsertObservationLabelMismatchReturnsFalse)
{
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 1.0f);
  grid.insertionOptions.gasSensorLabel = "MCEnose";

  mrpt::obs::CObservationGasSensors obs;
  obs.sensorLabel = "SomeOtherSensor";
  obs.m_readings.push_back(BuildENose({1.0f}, {0x2600}));

  EXPECT_FALSE(grid.insertObservation(obs));
}

TEST(CGasConcentrationGridMap2D, InsertObservationWrongObservationTypeReturnsFalse)
{
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 1.0f);

  mrpt::obs::CObservationOdometry obs;
  EXPECT_FALSE(grid.insertObservation(obs));
}

TEST(CGasConcentrationGridMap2D, InsertObservationMCEnoseMeanReading)
{
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 1.0f);
  grid.insertionOptions.gasSensorLabel = "MCEnose";
  grid.insertionOptions.gasSensorType = 0x0000;  // mean of all readings
  grid.insertionOptions.enose_id = 0;

  mrpt::obs::CObservationGasSensors obs;
  obs.sensorLabel = "MCEnose";
  obs.m_readings.push_back(BuildENose({1.0f, 2.0f, 3.0f}, {0x2600, 0x2602, 0x2620}));

  EXPECT_TRUE(grid.insertObservation(obs));
}

TEST(CGasConcentrationGridMap2D, InsertObservationFullMCEnoseSpecificSensorFound)
{
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 1.0f);
  grid.insertionOptions.gasSensorLabel = "Full_MCEnose";
  grid.insertionOptions.gasSensorType = 0x2602;  // present at index 1
  grid.insertionOptions.enose_id = 0;

  mrpt::obs::CObservationGasSensors obs;
  obs.sensorLabel = "Full_MCEnose";
  obs.m_readings.push_back(BuildENose({1.0f, 2.0f, 3.0f}, {0x2600, 0x2602, 0x2620}));

  EXPECT_TRUE(grid.insertObservation(obs));
}

TEST(CGasConcentrationGridMap2D, InsertObservationMCEnoseSensorTypeNotFoundFallsBackToMean)
{
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 1.0f);
  grid.insertionOptions.gasSensorLabel = "MCEnose";
  grid.insertionOptions.gasSensorType = 0x9999;  // not present in sensorTypes
  grid.insertionOptions.enose_id = 0;

  mrpt::obs::CObservationGasSensors obs;
  obs.sensorLabel = "MCEnose";
  obs.m_readings.push_back(BuildENose({1.0f, 2.0f, 3.0f}, {0x2600, 0x2602, 0x2620}));

  EXPECT_TRUE(grid.insertObservation(obs));
}

TEST(CGasConcentrationGridMap2D, InsertObservationOtherLabelUsesFirstReading)
{
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 1.0f);
  grid.insertionOptions.gasSensorLabel = "GDM";

  mrpt::obs::CObservationGasSensors obs;
  obs.sensorLabel = "GDM";
  obs.m_readings.push_back(BuildENose({2.5f}, {0x4161}));

  EXPECT_TRUE(grid.insertObservation(obs));
}

// =========================================================================
//  internal_computeObservationLikelihood(): not implemented
// =========================================================================

TEST(CGasConcentrationGridMap2D, ComputeObservationLikelihoodThrows)
{
  mrpt::maps::CGasConcentrationGridMap2D grid;
  mrpt::obs::CObservationGasSensors obs;
  obs.m_readings.push_back(BuildENose({1.0f}, {0x2600}));
  mrpt::poses::CPose3D pose;

  EXPECT_THROW(grid.computeObservationLikelihood(obs, pose), std::exception);
}

// =========================================================================
//  Visualization
// =========================================================================

TEST(CGasConcentrationGridMap2D, VisualizationDisabledIsANoOp)
{
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 1.0f);
  grid.genericMapParams.enableSaveAs3DObject = false;

  mrpt::viz::CSetOfObjects viz;
  grid.getVisualizationInto(viz);
  EXPECT_TRUE(viz.empty());

  mrpt::viz::CSetOfObjects meanObj, varObj;
  grid.getAs3DObject(meanObj, varObj);
  EXPECT_TRUE(meanObj.empty());
}

// =========================================================================
//  Wind information: build_Gaussian_Wind_Grid(), getWindAs3DObject(),
//  increaseUncertainty(), simulateAdvection()
// =========================================================================

TEST(CGasConcentrationGridMap2D, SimulateAdvectionWithoutWindReturnsFalse)
{
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanApproximate, -3.0f, 3.0f, -3.0f, 3.0f, 1.0f);
  ASSERT_FALSE(grid.insertionOptions.useWindInformation);
  EXPECT_FALSE(grid.simulateAdvection(0.01));
}

TEST(CGasConcentrationGridMap2D, IncreaseUncertainty)
{
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanApproximate, -2.0f, 2.0f, -2.0f, 2.0f, 1.0f);
  grid.insertIndividualReading(0.3, {0.0, 0.0});
  EXPECT_NO_THROW(grid.increaseUncertainty(0.05));
}

TEST(CGasConcentrationGridMap2D, WindGridVisualizationAndAdvection)
{
  // Small grid (6x6 cells) to keep the wind LUT and the advection transition
  // matrix cheap to compute.
  mrpt::maps::CGasConcentrationGridMap2D grid(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanApproximate, -3.0f, 3.0f, -3.0f, 3.0f, 1.0f);

  // NOTE: TInsertionOptions::advectionFreq has no default member initializer
  // (see reported bug), so it must be set explicitly before triggering the
  // wind LUT build to avoid using an indeterminate value.
  grid.insertionOptions.advectionFreq = 1.0f;
  grid.insertionOptions.useWindInformation = true;
  grid.clear();  // triggers build_Gaussian_Wind_Grid()

  grid.insertIndividualReading(0.5, {0.0, 0.0});

  mrpt::viz::CSetOfObjects meanObj, varObj;
  EXPECT_NO_THROW(grid.getAs3DObject(meanObj, varObj));

  mrpt::viz::CSetOfObjects viz;
  EXPECT_NO_THROW(grid.getVisualizationInto(viz));

  const auto windObj = grid.getWindAs3DObject();
  ASSERT_NE(windObj, nullptr);

  EXPECT_TRUE(grid.simulateAdvection(0.01));
}

// =========================================================================
//  optionsByName()
// =========================================================================

TEST(CGasConcentrationGridMap2D, OptionsByName)
{
  mrpt::maps::CGasConcentrationGridMap2D grid;
  const auto opts = grid.optionsByName();
  const auto it = opts.find("insertionOptions");
  ASSERT_NE(it, opts.end());
  EXPECT_EQ(it->second, &grid.insertionOptions);
}

// =========================================================================
//  Serialization round-trip
// =========================================================================

TEST(CGasConcentrationGridMap2D, SerializeRoundTrip)
{
  mrpt::maps::CGasConcentrationGridMap2D src(
      mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 1.0f);
  src.insertIndividualReading(0.4, {0.0, 0.0});

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  mrpt::maps::CGasConcentrationGridMap2D dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    mrpt::serialization::CSerializable::Ptr obj;
    ar >> obj;
    ASSERT_NE(obj, nullptr);
    auto* dstPtr = dynamic_cast<mrpt::maps::CGasConcentrationGridMap2D*>(obj.get());
    ASSERT_NE(dstPtr, nullptr);
    dst = *dstPtr;
  }

  EXPECT_EQ(dst.getSizeX(), src.getSizeX());
  EXPECT_EQ(dst.getSizeY(), src.getSizeY());
}

// =========================================================================
//  TInsertionOptions::loadFromConfigFile()
// =========================================================================

TEST(CGasConcentrationGridMap2D, InsertionOptionsLoadFromConfigFileHexSensorType)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("ins", "gasSensorType", std::string("2602"));

  mrpt::maps::CGasConcentrationGridMap2D::TInsertionOptions opts;
  opts.loadFromConfigFile(cfg, "ins");

  EXPECT_EQ(opts.gasSensorType, 0x2602);
}

TEST(CGasConcentrationGridMap2D, InsertionOptionsLoadFromConfigFileFallsBackToKFSensorType)
{
  mrpt::config::CConfigFileMemory cfg;
  // No "gasSensorType" key: the hex parse of the default "-1" is negative,
  // so it must fall back to the legacy "KF_sensorType" key.
  cfg.write("ins", "KF_sensorType", 4161);

  mrpt::maps::CGasConcentrationGridMap2D::TInsertionOptions opts;
  opts.loadFromConfigFile(cfg, "ins");

  EXPECT_EQ(opts.gasSensorType, 4161);
}

// =========================================================================
//  TMapDefinition: loadFromConfigFile_map_specific(), dumpToTextStream(),
//  internal_CreateFromMapDefinition()
// =========================================================================

TEST(CGasConcentrationGridMap2D, MapDefinitionLoadAndCreate)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[map_gasGrid_00_creationOpts]
min_x=-3
max_x=3
min_y=-3
max_y=3
resolution=0.5
mapType=mrKalmanFilter

[map_gasGrid_00_insertOpts]
gasSensorLabel=Full_MCEnose
)"""");

  mrpt::maps::CGasConcentrationGridMap2D::TMapDefinition def;
  def.loadFromConfigFile(cfg, "map_gasGrid_00");

  EXPECT_FLOAT_EQ(def.min_x, -3.0f);
  EXPECT_FLOAT_EQ(def.max_x, 3.0f);
  EXPECT_EQ(def.insertionOpts.gasSensorLabel, "Full_MCEnose");

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());

  const auto createdMap = mrpt::maps::CGasConcentrationGridMap2D::CreateFromMapDefinition(def);
  ASSERT_NE(createdMap, nullptr);
  EXPECT_GT(createdMap->getSizeX(), 0u);
}
