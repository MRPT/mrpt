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
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <cmath>
#include <filesystem>
#include <sstream>
#include <vector>

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

// =========================================================================
//  Wind-advection look-up table
// =========================================================================

TEST(CGasConcentrationGridMap2D, GaussianWindLUTBuildSaveAndReload)
{
  // build_Gaussian_Wind_Grid() caches the generated table in a file in the
  // current working directory, so run this from a scratch directory.
  const auto prevDir = std::filesystem::current_path();
  const auto tmpDir = std::filesystem::path(mrpt::system::getTempFileName() + "_windlut");
  std::filesystem::create_directories(tmpDir);
  std::filesystem::current_path(tmpDir);

  try
  {
    mrpt::maps::CGasConcentrationGridMap2D grid(
        mrpt::maps::CRandomFieldGridMap2D::mrKalmanApproximate, -0.5f, 0.5f, -0.5f, 0.5f, 0.1f);
    grid.insertionOptions.useWindInformation = true;
    grid.insertionOptions.advectionFreq = 1.0f;

    // The first clear() generates the table from scratch and saves it:
    grid.clear();
    EXPECT_FALSE(std::filesystem::is_empty(tmpDir));

    // A second map with the same parameters must find and load the cached one:
    mrpt::maps::CGasConcentrationGridMap2D grid2(
        mrpt::maps::CRandomFieldGridMap2D::mrKalmanApproximate, -0.5f, 0.5f, -0.5f, 0.5f, 0.1f);
    grid2.insertionOptions.useWindInformation = true;
    grid2.insertionOptions.advectionFreq = 1.0f;
    grid2.clear();

    // The advection simulation now has a usable look-up table:
    grid2.insertIndividualReading(0.5, {0.0, 0.0}, true, true, 1.0);
    EXPECT_TRUE(grid2.simulateAdvection(0.01));

    // ...but only the approximate-Kalman representation supports it:
    mrpt::maps::CGasConcentrationGridMap2D dmGrid(
        mrpt::maps::CRandomFieldGridMap2D::mrKernelDM, -0.5f, 0.5f, -0.5f, 0.5f, 0.1f);
    dmGrid.insertionOptions.useWindInformation = true;
    dmGrid.insertionOptions.advectionFreq = 1.0f;
    dmGrid.clear();
    EXPECT_FALSE(dmGrid.simulateAdvection(0.01));
  }
  catch (...)
  {
    std::filesystem::current_path(prevDir);
    std::filesystem::remove_all(tmpDir);
    throw;
  }

  std::filesystem::current_path(prevDir);
  std::filesystem::remove_all(tmpDir);
}

namespace
{
mrpt::obs::CObservationGasSensors makeGasObs(
    double reading, const mrpt::math::TPose3D& sensorPoseOnRobot = mrpt::math::TPose3D())
{
  mrpt::obs::CObservationGasSensors obs;
  obs.sensorLabel = "GDM";
  mrpt::obs::CObservationGasSensors::TObservationENose eNose;
  eNose.eNosePoseOnTheRobot = sensorPoseOnRobot;
  eNose.readingsVoltage = {static_cast<float>(reading)};
  eNose.sensorTypes = {0};
  obs.m_readings.push_back(eNose);
  return obs;
}
}  // namespace

// Regression: the running variance of all the readings (the DM+V prior
// variance) used the updated mean only, underestimating it.
TEST(CGasConcentrationGridMap2D, RunningVarianceOfNormalizedReadings)
{
  using mrpt::maps::CGasConcentrationGridMap2D;
  using mrpt::maps::CRandomFieldGridMap2D;

  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrKernelDMV, -5, 5, -5, 5, 0.2f);
  grid.insertionOptions.gasSensorLabel = "GDM";
  grid.insertionOptions.R_min = 0;
  grid.insertionOptions.R_max = 1;

  const std::vector<double> readings = {0.0, 1.0, 0.5, 0.2};
  for (const double r : readings)
  {
    EXPECT_TRUE(grid.insertObservation(makeGasObs(r), mrpt::poses::CPose3D()));
  }

  double mean = 0;
  for (const double r : readings)
  {
    mean += r;
  }
  mean /= static_cast<double>(readings.size());
  double var = 0;
  for (const double r : readings)
  {
    var += mrpt::square(r - mean);
  }
  var /= static_cast<double>(readings.size());

  // A never observed cell takes the average of all readings:
  double predVal = 0;
  double valVar = 0;
  grid.predictMeasurement(4.0, 4.0, predVal, valVar, false, CRandomFieldGridMap2D::gimNearest);
  EXPECT_NEAR(predVal, mean, 1e-9);
  EXPECT_NEAR(valVar, var, 1e-9);

  // And so does a query out of the map:
  grid.predictMeasurement(1e4, 1e4, predVal, valVar, false, CRandomFieldGridMap2D::gimNearest);
  EXPECT_NEAR(predVal, mean, 1e-9);
  EXPECT_NEAR(valVar, var, 1e-9);
}

// Regression: the sensor position ignored the robot attitude (only its 2D
// projection was used).
TEST(CGasConcentrationGridMap2D, SensorPositionUsesThe3DRobotPose)
{
  using mrpt::maps::CGasConcentrationGridMap2D;
  using mrpt::maps::CRandomFieldGridMap2D;

  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrKernelDM, -2, 2, -2, 2, 0.1f);
  grid.insertionOptions.gasSensorLabel = "GDM";

  // The sensor is 1 m above the robot origin, and the robot is rolled 90 deg,
  // so the sensor lies at (0,-1):
  const mrpt::poses::CPose3D robotPose(0, 0, 0, 0, 0, mrpt::DEG2RAD(90.0));
  EXPECT_TRUE(
      grid.insertObservation(makeGasObs(0.5, mrpt::math::TPose3D(0, 0, 1, 0, 0, 0)), robotPose));

  const auto* cellAtSensor = grid.cellByPos(0.05, -0.95);
  const auto* cellAtRobot = grid.cellByPos(0.05, 0.05);
  ASSERT_TRUE(cellAtSensor != nullptr);
  ASSERT_TRUE(cellAtRobot != nullptr);
  EXPECT_GT(cellAtSensor->dm_mean_w(), 0.5);
  EXPECT_NEAR(cellAtRobot->dm_mean_w(), 0.0, 1e-6);
}

// Regression: advectionFreq had no default value.
TEST(CGasConcentrationGridMap2D, AdvectionFreqHasADefaultValue)
{
  const mrpt::maps::CGasConcentrationGridMap2D::TInsertionOptions opts;
  EXPECT_FLOAT_EQ(opts.advectionFreq, 1.0f);
}

// increaseUncertainty() adds its argument to the variance of each cell, and
// is a no-op (instead of an out-of-bounds write) for other representations.
TEST(CGasConcentrationGridMap2D, IncreaseUncertaintyAddsToTheVariance)
{
  using mrpt::maps::CGasConcentrationGridMap2D;
  using mrpt::maps::CRandomFieldGridMap2D;

  {
    CGasConcentrationGridMap2D grid(
        CRandomFieldGridMap2D::mrKalmanApproximate, -2.0f, 2.0f, -2.0f, 2.0f, 1.0f);
    grid.insertIndividualReading(0.3, {0.0, 0.0});
    mrpt::math::CVectorDouble means;
    mrpt::math::CVectorDouble stds0;
    grid.getMeanAndSTD(means, stds0);
    grid.increaseUncertainty(0.05);
    mrpt::math::CVectorDouble stds1;
    grid.getMeanAndSTD(means, stds1);
    ASSERT_EQ(stds0.size(), stds1.size());
    for (int i = 0; i < stds0.size(); i++)
    {
      EXPECT_NEAR(mrpt::square(stds1[i]), mrpt::square(stds0[i]) + 0.05, 1e-9);
    }
  }
  {
    CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrKernelDM, -1, 1, -1, 1, 0.1f);
    grid.insertIndividualReading(0.3, {0.0, 0.0});
    mrpt::math::CMatrixDouble before;
    grid.getAsMatrix(before);
    grid.increaseUncertainty(0.05);
    mrpt::math::CMatrixDouble after;
    grid.getAsMatrix(after);
    EXPECT_EQ(before, after);
  }
}
