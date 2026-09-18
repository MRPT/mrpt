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

/** Unit tests for the EKF range-bearing SLAM filters (SE(2) and SE(3)),
 *  driven by observations simulated from a synthetic landmark map.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/random.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
#include <mrpt/system/filesystem.h>

#include <sstream>

using mrpt::slam::CRangeBearingKFSLAM;
using mrpt::slam::CRangeBearingKFSLAM2D;

namespace
{
/** Four landmarks around the origin, all at z=0 so both the 2D and the 6D
 *  filters can see them. */
mrpt::maps::CLandmarksMap makeLandmarks()
{
  const std::vector<mrpt::math::TPoint3D> pts = {
      {4.0,  1.0, 0.0},
      {5.0, -2.0, 0.0},
      {8.0,  2.0, 0.0},
      {9.0, -1.0, 0.0}
  };

  mrpt::maps::CLandmarksMap m;
  int64_t id = 0;
  for (const auto& p : pts)
  {
    mrpt::maps::CLandmark lm;
    lm.ID = id++;
    lm.pose_mean = p;
    m.landmarks.push_back(lm);
  }
  return m;
}

mrpt::obs::CSensoryFrame::Ptr simulateObs(
    const mrpt::maps::CLandmarksMap& lms,
    const mrpt::poses::CPose3D& robotPose,
    bool withIDs,
    double sigmaPitch)
{
  auto obs = mrpt::obs::CObservationBearingRange::Create();
  obs->minSensorDistance = 0;
  obs->maxSensorDistance = 100.0f;
  obs->fieldOfView_yaw = 2 * M_PIf;
  obs->fieldOfView_pitch = M_PIf;
  obs->sensor_std_range = 0.01f;
  obs->sensor_std_yaw = mrpt::DEG2RAD(0.2f);
  obs->sensor_std_pitch = static_cast<float>(sigmaPitch);

  lms.simulateRangeBearingReadings(
      robotPose, mrpt::poses::CPose3D(), *obs, withIDs, 0.01 /*sigmaRange*/,
      mrpt::DEG2RAD(0.2) /*sigmaYaw*/, sigmaPitch);

  auto sf = mrpt::obs::CSensoryFrame::Create();
  sf->insert(obs);
  return sf;
}

mrpt::obs::CActionCollection::Ptr makeOdometry(const mrpt::poses::CPose2D& incr)
{
  mrpt::obs::CActionRobotMovement2D act;
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions o;
  o.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
  o.gaussianModel.minStdXY = 0.02;
  o.gaussianModel.minStdPHI = mrpt::DEG2RAD(0.5);
  act.computeFromOdometry(incr, o);
  act.timestamp = mrpt::Clock::now();

  auto acts = mrpt::obs::CActionCollection::Create();
  acts->insert(act);
  return acts;
}

/** Runs the given filter along a short straight path, returning the true
 *  final robot pose. */
template <class FILTER>
mrpt::poses::CPose2D runSession(
    FILTER& f,
    const mrpt::maps::CLandmarksMap& lms,
    size_t nSteps = 4,
    bool withIDs = true,
    bool withOdometry = true,
    double sigmaPitch = mrpt::DEG2RAD(0.2))
{
  mrpt::poses::CPose2D gtPose(0, 0, 0);

  {
    auto acts = makeOdometry(mrpt::poses::CPose2D(0, 0, 0));
    auto sf = simulateObs(lms, mrpt::poses::CPose3D(gtPose), withIDs, sigmaPitch);
    if (!withOdometry) acts = mrpt::obs::CActionCollection::Create();
    f.processActionObservation(acts, sf);
  }

  for (size_t i = 0; i < nSteps; i++)
  {
    const mrpt::poses::CPose2D incr(0.25, 0, 0);
    gtPose = gtPose + incr;

    auto acts = makeOdometry(incr);
    if (!withOdometry) acts = mrpt::obs::CActionCollection::Create();
    auto sf = simulateObs(lms, mrpt::poses::CPose3D(gtPose), withIDs, sigmaPitch);
    f.processActionObservation(acts, sf);
  }
  return gtPose;
}
}  // namespace

// ------------------------- SE(2) filter -------------------------

TEST(CRangeBearingKFSLAM2D, mapsFourLandmarksWithKnownIDs)
{
  mrpt::random::getRandomGenerator().randomize(1234);

  const auto lms = makeLandmarks();

  CRangeBearingKFSLAM2D f;
  f.options.create_simplemap = true;
  const auto gtPose = runSession(f, lms, 4, true, true, 0 /*sigmaPitch*/);

  mrpt::poses::CPosePDFGaussian robotPose;
  std::vector<mrpt::math::TPoint2D> lmPos;
  std::map<unsigned int, mrpt::maps::CLandmark::TLandmarkID> lmIDs;
  mrpt::math::CVectorDouble fullState;
  mrpt::math::CMatrixDouble fullCov;
  f.getCurrentState(robotPose, lmPos, lmIDs, fullState, fullCov);

  EXPECT_EQ(lmPos.size(), 4U);
  EXPECT_EQ(lmIDs.size(), 4U);
  EXPECT_EQ(static_cast<size_t>(fullState.size()), 3U + 2U * 4U);
  EXPECT_EQ(fullCov.rows(), fullState.size());

  EXPECT_NEAR(robotPose.mean.x(), gtPose.x(), 0.25);
  EXPECT_NEAR(robotPose.mean.y(), gtPose.y(), 0.25);

  // Each mapped landmark must be near its ground truth position:
  for (const auto& [idxInMap, id] : lmIDs)
  {
    const auto* gtLm = lms.landmarks.getByBeaconID(id);
    ASSERT_TRUE(gtLm != nullptr);
    EXPECT_NEAR(lmPos[idxInMap].x, gtLm->pose_mean.x, 0.35);
    EXPECT_NEAR(lmPos[idxInMap].y, gtLm->pose_mean.y, 0.35);
  }

  mrpt::poses::CPosePDFGaussian onlyPose;
  f.getCurrentRobotPose(onlyPose);
  EXPECT_NEAR(onlyPose.mean.x(), robotPose.mean.x(), 1e-9);

  // With known landmark IDs no data association is run, but the ID-based
  // matching is still reported back:
  const auto& da = f.getLastDataAssociation();
  EXPECT_TRUE(da.predictions_IDs.empty());
  EXPECT_EQ(da.results.associations.size(), 4U);

  auto obj = mrpt::viz::CSetOfObjects::Create();
  f.getAs3DObject(obj);
  EXPECT_GT(obj->size(), 4U);

  f.reset();
  f.getCurrentState(robotPose, lmPos, lmIDs, fullState, fullCov);
  EXPECT_EQ(lmPos.size(), 0U);
  EXPECT_NEAR(robotPose.mean.x(), 0.0, 1e-9);
}

TEST(CRangeBearingKFSLAM2D, unknownIDsAreDataAssociated)
{
  mrpt::random::getRandomGenerator().randomize(4321);

  const auto lms = makeLandmarks();

  CRangeBearingKFSLAM2D f;
  f.options.data_assoc_method = mrpt::slam::assocJCBB;
  f.options.data_assoc_metric = mrpt::slam::metricMaha;
  f.options.data_assoc_IC_metric = mrpt::slam::metricMaha;

  runSession(f, lms, 4, false /* no landmark IDs */, true, 0 /*sigmaPitch*/);

  mrpt::poses::CPosePDFGaussian robotPose;
  std::vector<mrpt::math::TPoint2D> lmPos;
  std::map<unsigned int, mrpt::maps::CLandmark::TLandmarkID> lmIDs;
  mrpt::math::CVectorDouble fullState;
  mrpt::math::CMatrixDouble fullCov;
  f.getCurrentState(robotPose, lmPos, lmIDs, fullState, fullCov);

  // Data association must not duplicate the 4 real landmarks:
  EXPECT_EQ(lmPos.size(), 4U);

  const auto& daInfo = f.getLastDataAssociation();
  EXPECT_EQ(daInfo.predictions_IDs.size(), 4U);
  EXPECT_EQ(daInfo.results.associations.size(), 4U);
}

TEST(CRangeBearingKFSLAM2D, worksWithoutOdometry)
{
  mrpt::random::getRandomGenerator().randomize(99);

  const auto lms = makeLandmarks();

  CRangeBearingKFSLAM2D f;
  // The 2D filter has no "force_ignore_odometry" flag: feed empty actions.
  runSession(f, lms, 3, true /*withIDs*/, false /*withOdometry*/, 0 /*sigmaPitch*/);

  mrpt::poses::CPosePDFGaussian robotPose;
  std::vector<mrpt::math::TPoint2D> lmPos;
  std::map<unsigned int, mrpt::maps::CLandmark::TLandmarkID> lmIDs;
  mrpt::math::CVectorDouble fullState;
  mrpt::math::CMatrixDouble fullCov;
  f.getCurrentState(robotPose, lmPos, lmIDs, fullState, fullCov);
  EXPECT_EQ(lmPos.size(), 4U);
}

TEST(CRangeBearingKFSLAM2D, optionsLoadAndDump)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("RangeBearingKFSLAM", "stds_Q_no_odo", "[0.05 0.05 0.02]");
  cfg.write("RangeBearingKFSLAM", "std_sensor_range", 0.05);
  cfg.write("RangeBearingKFSLAM", "std_sensor_yaw_deg", 1.5);
  cfg.write("RangeBearingKFSLAM", "quantiles_3D_representation", 2.0);
  cfg.write("RangeBearingKFSLAM", "create_simplemap", true);
  cfg.write("RangeBearingKFSLAM", "data_assoc_method", "assocJCBB");
  cfg.write("RangeBearingKFSLAM", "data_assoc_metric", "metricMaha");
  cfg.write("RangeBearingKFSLAM", "data_assoc_IC_metric", "metricMaha");

  CRangeBearingKFSLAM2D f;
  f.loadOptions(cfg);

  EXPECT_NEAR(f.options.std_sensor_range, 0.05f, 1e-6);
  EXPECT_NEAR(f.options.std_sensor_yaw, mrpt::DEG2RAD(1.5f), 1e-6);
  EXPECT_NEAR(f.options.quantiles_3D_representation, 2.0f, 1e-6);
  EXPECT_TRUE(f.options.create_simplemap);
  EXPECT_EQ(f.options.data_assoc_method, mrpt::slam::assocJCBB);

  std::stringstream ss;
  f.options.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("std_sensor_range"), std::string::npos);
  // The heading component is reported in degrees, as it is read:
  EXPECT_NE(
      ss.str().find("stds_Q_no_odo                           = [0.050000 m, "
                    "0.050000 m, 0.020000 deg]"),
      std::string::npos);
}

TEST(CRangeBearingKFSLAM2D, saveMapAsMATLABFile)
{
  mrpt::random::getRandomGenerator().randomize(7);

  const auto lms = makeLandmarks();
  CRangeBearingKFSLAM2D f;
  f.options.create_simplemap = true;
  runSession(f, lms, 3, true, true, 0 /*sigmaPitch*/);

  const std::string file = mrpt::system::getTempFileName() + std::string(".m");
  f.saveMapAndPath2DRepresentationAsMATLABFile(file);
  EXPECT_TRUE(mrpt::system::fileExists(file));
  EXPECT_GT(mrpt::system::getFileSize(file), 0U);
  mrpt::system::deleteFile(file);
}

// ------------------------- SE(3) filter -------------------------

TEST(CRangeBearingKFSLAM, mapsFourLandmarksWithKnownIDs)
{
  mrpt::random::getRandomGenerator().randomize(1234);

  const auto lms = makeLandmarks();

  CRangeBearingKFSLAM f;
  f.options.create_simplemap = true;
  const auto gtPose = runSession(f, lms);

  mrpt::poses::CPose3DQuatPDFGaussian robotPose;
  std::vector<mrpt::math::TPoint3D> lmPos;
  std::map<unsigned int, mrpt::maps::CLandmark::TLandmarkID> lmIDs;
  mrpt::math::CVectorDouble fullState;
  mrpt::math::CMatrixDouble fullCov;
  f.getCurrentState(robotPose, lmPos, lmIDs, fullState, fullCov);

  EXPECT_EQ(lmPos.size(), 4U);
  EXPECT_EQ(static_cast<size_t>(fullState.size()), 7U + 3U * 4U);

  EXPECT_NEAR(robotPose.mean.x(), gtPose.x(), 0.3);
  EXPECT_NEAR(robotPose.mean.y(), gtPose.y(), 0.3);

  for (const auto& [idxInMap, id] : lmIDs)
  {
    const auto* gtLm = lms.landmarks.getByBeaconID(id);
    ASSERT_TRUE(gtLm != nullptr);
    EXPECT_NEAR(lmPos[idxInMap].x, gtLm->pose_mean.x, 0.5);
    EXPECT_NEAR(lmPos[idxInMap].y, gtLm->pose_mean.y, 0.5);
  }

  // The 6D-angles flavor of the same getters:
  mrpt::poses::CPose3DPDFGaussian rpEuler;
  f.getCurrentState(rpEuler, lmPos, lmIDs, fullState, fullCov);
  EXPECT_NEAR(rpEuler.mean.x(), robotPose.mean.x(), 1e-9);
  f.getCurrentRobotPose(rpEuler);
  EXPECT_NEAR(rpEuler.mean.x(), robotPose.mean.x(), 1e-9);
  EXPECT_NEAR(f.getCurrentRobotPoseMean().x(), robotPose.mean.x(), 1e-9);

  auto obj = mrpt::viz::CSetOfObjects::Create();
  f.getAs3DObject(obj);
  EXPECT_GT(obj->size(), 4U);

  f.reset();
  f.getCurrentState(robotPose, lmPos, lmIDs, fullState, fullCov);
  EXPECT_EQ(lmPos.size(), 0U);
}

TEST(CRangeBearingKFSLAM, worksWithoutOdometry)
{
  mrpt::random::getRandomGenerator().randomize(55);

  const auto lms = makeLandmarks();

  CRangeBearingKFSLAM f;
  f.options.force_ignore_odometry = true;
  runSession(f, lms, 3);

  mrpt::poses::CPose3DQuatPDFGaussian robotPose;
  std::vector<mrpt::math::TPoint3D> lmPos;
  std::map<unsigned int, mrpt::maps::CLandmark::TLandmarkID> lmIDs;
  mrpt::math::CVectorDouble fullState;
  mrpt::math::CMatrixDouble fullCov;
  f.getCurrentState(robotPose, lmPos, lmIDs, fullState, fullCov);
  EXPECT_EQ(lmPos.size(), 4U);
}

TEST(CRangeBearingKFSLAM, partitioningExperimentSpectral)
{
  mrpt::random::getRandomGenerator().randomize(21);

  const auto lms = makeLandmarks();

  CRangeBearingKFSLAM f;
  f.options.create_simplemap = true;
  f.options.doPartitioningExperiment = true;
  f.options.partitioningMethod = 0;  // spectral graph-cut
  runSession(f, lms, 4);

  std::vector<std::vector<uint32_t>> parts;
  f.getLastPartition(parts);
  EXPECT_GT(parts.size(), 0U);

  std::vector<std::vector<uint32_t>> membership;
  f.getLastPartitionLandmarks(membership);
  EXPECT_EQ(membership.size(), 4U);

  EXPECT_TRUE(std::isfinite(f.computeOffDiagonalBlocksApproximationError(membership)));

  f.getLastPartitionLandmarksAsIfFixedSubmaps(2, membership);
  EXPECT_EQ(membership.size(), 4U);

  EXPECT_NO_THROW(f.reconsiderPartitionsNow());

  // The 3D representation labels each landmark with its partitions:
  auto obj = mrpt::viz::CSetOfObjects::Create();
  f.getAs3DObject(obj);
  EXPECT_GT(obj->size(), 4U);
}

TEST(CRangeBearingKFSLAM, partitioningExperimentFixedSize)
{
  mrpt::random::getRandomGenerator().randomize(22);

  const auto lms = makeLandmarks();

  CRangeBearingKFSLAM f;
  f.options.create_simplemap = true;
  f.options.doPartitioningExperiment = true;
  f.options.partitioningMethod = 2;  // one cut every 2 observations
  runSession(f, lms, 4);

  std::vector<std::vector<uint32_t>> parts;
  f.getLastPartition(parts);
  EXPECT_GT(parts.size(), 0U);
}

TEST(CRangeBearingKFSLAM, optionsLoadAndDump)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("RangeBearingKFSLAM", "stds_Q_no_odo", "[0.05 0.05 0.05 0.01 0.01 0.01 0.01]");
  cfg.write("RangeBearingKFSLAM", "std_sensor_range", 0.07);
  cfg.write("RangeBearingKFSLAM", "std_sensor_yaw_deg", 1.0);
  cfg.write("RangeBearingKFSLAM", "std_sensor_pitch_deg", 1.0);
  cfg.write("RangeBearingKFSLAM", "std_odo_z_additional", 0.02);
  cfg.write("RangeBearingKFSLAM", "doPartitioningExperiment", true);
  cfg.write("RangeBearingKFSLAM", "partitioningMethod", 3);
  cfg.write("RangeBearingKFSLAM", "data_assoc_method", "assocNN");
  cfg.write("RangeBearingKFSLAM", "data_assoc_metric", "metricML");
  cfg.write("RangeBearingKFSLAM", "data_assoc_IC_metric", "metricML");

  CRangeBearingKFSLAM f;
  f.loadOptions(cfg);

  EXPECT_NEAR(f.options.std_sensor_range, 0.07f, 1e-6);
  EXPECT_NEAR(f.options.std_sensor_pitch, mrpt::DEG2RAD(1.0f), 1e-6);
  EXPECT_NEAR(f.options.std_odo_z_additional, 0.02f, 1e-6);
  EXPECT_TRUE(f.options.doPartitioningExperiment);
  EXPECT_EQ(f.options.partitioningMethod, 3);
  EXPECT_EQ(f.options.data_assoc_metric, mrpt::slam::metricML);

  std::stringstream ss;
  f.options.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("std_sensor_range"), std::string::npos);
}

TEST(CRangeBearingKFSLAM, saveMapAsMATLABFile)
{
  mrpt::random::getRandomGenerator().randomize(23);

  const auto lms = makeLandmarks();
  CRangeBearingKFSLAM f;
  f.options.create_simplemap = true;
  runSession(f, lms, 3);

  const std::string file = mrpt::system::getTempFileName() + std::string(".m");
  f.saveMapAndPath2DRepresentationAsMATLABFile(file);
  EXPECT_TRUE(mrpt::system::fileExists(file));
  EXPECT_GT(mrpt::system::getFileSize(file), 0U);
  mrpt::system::deleteFile(file);
}
