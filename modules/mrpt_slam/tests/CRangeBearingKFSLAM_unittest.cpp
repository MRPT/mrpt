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
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/random.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
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

// ------------------------- Model Jacobians -------------------------

namespace
{
/** Exposes the protected KF model callbacks */
class TestKF3D : public CRangeBearingKFSLAM
{
 public:
  using CRangeBearingKFSLAM::OnGetAction;
  using CRangeBearingKFSLAM::OnInverseObservationModel;
  using CRangeBearingKFSLAM::OnObservationJacobians;
  using CRangeBearingKFSLAM::OnObservationModel;
  using CRangeBearingKFSLAM::OnTransitionJacobian;
  using CRangeBearingKFSLAM::OnTransitionModel;
  KFVector& state() { return m_xkk; }
  void setSF(const mrpt::obs::CSensoryFrame::Ptr& sf) { m_SF = sf; }
  void setAction(const mrpt::obs::CActionCollection::Ptr& a) { m_action = a; }
};

class TestKF2D : public CRangeBearingKFSLAM2D
{
 public:
  using CRangeBearingKFSLAM2D::OnInverseObservationModel;
  using CRangeBearingKFSLAM2D::OnObservationJacobians;
  using CRangeBearingKFSLAM2D::OnObservationModel;
  KFVector& state() { return m_xkk; }
  void setSF(const mrpt::obs::CSensoryFrame::Ptr& sf) { m_SF = sf; }
};

mrpt::obs::CSensoryFrame::Ptr sfWithSensorPose(const mrpt::poses::CPose3D& sensorPose)
{
  auto obs = mrpt::obs::CObservationBearingRange::Create();
  obs->sensorLocationOnRobot = sensorPose;
  auto sf = mrpt::obs::CSensoryFrame::Create();
  sf->insert(obs);
  return sf;
}

/** Central-difference derivative of the predicted observation of landmark #0
 * wrt the state vector element "idx" */
template <class KF>
Eigen::VectorXd numericObsDerivative(KF& kf, size_t idx)
{
  const double h = 1e-6;
  auto& x = kf.state();
  const double x0 = x[idx];
  typename KF::vector_KFArray_OBS pred;
  x[idx] = x0 + h;
  kf.OnObservationModel({0}, pred);
  const auto fp = pred[0];
  x[idx] = x0 - h;
  kf.OnObservationModel({0}, pred);
  const auto fm = pred[0];
  x[idx] = x0;
  const auto n = static_cast<Eigen::Index>(fp.size());
  Eigen::VectorXd d = Eigen::VectorXd::Zero(n);
  for (Eigen::Index k = 0; k < n; k++)
  {
    // Range is not an angle, bearings are:
    d[k] = (k == 0 ? fp[k] - fm[k] : mrpt::math::wrapToPi(fp[k] - fm[k])) / (2 * h);
  }
  return d;
}
}  // namespace

TEST(CRangeBearingKFSLAM, observationJacobiansMatchNumericalDerivatives)
{
  TestKF3D kf;
  kf.setSF(sfWithSensorPose(mrpt::poses::CPose3D(0.3, -0.1, 0.4, 0.4, 0.2, -0.3)));
  const mrpt::poses::CPose3DQuat q(mrpt::poses::CPose3D(1.0, 2.0, 0.5, 0.7, -0.3, 0.2));
  auto& x = kf.state();
  x.resize(10);
  for (int i = 0; i < 7; i++)
  {
    x[i] = q[i];
  }
  x[7] = 5.0;
  x[8] = 4.0;
  x[9] = 1.5;

  TestKF3D::KFMatrix_OxV Hx;
  TestKF3D::KFMatrix_OxF Hy;
  kf.OnObservationJacobians(0, Hx, Hy);
  for (size_t j = 0; j < 10; j++)
  {
    const Eigen::VectorXd d = numericObsDerivative(kf, j);
    for (int k = 0; k < 3; k++)
    {
      const double analytic = j < 7 ? Hx(k, j) : Hy(k, j - 7);
      EXPECT_NEAR(analytic, d[k], 1e-6) << "row=" << k << " col=" << j;
    }
  }
}

TEST(CRangeBearingKFSLAM, inverseObservationModel)
{
  TestKF3D kf;
  kf.setSF(sfWithSensorPose(mrpt::poses::CPose3D(0.3, -0.1, 0.4, 0.4, 0.2, -0.3)));
  const mrpt::poses::CPose3DQuat q(mrpt::poses::CPose3D(1.0, 2.0, 0.5, 0.7, -0.3, 0.2));
  auto& x = kf.state();
  x.resize(7);
  for (int i = 0; i < 7; i++)
  {
    x[i] = q[i];
  }

  TestKF3D::KFArray_OBS z;
  z[0] = 4.0;
  z[1] = 0.6;
  z[2] = -0.4;
  TestKF3D::KFArray_FEAT yn;
  TestKF3D::KFMatrix_FxV dyn_dxv;
  TestKF3D::KFMatrix_FxO dyn_dhn;
  kf.OnInverseObservationModel(z, yn, dyn_dxv, dyn_dhn);

  const double h = 1e-6;
  for (size_t j = 0; j < 7; j++)
  {
    const double x0 = x[j];
    TestKF3D::KFArray_FEAT yp;
    TestKF3D::KFArray_FEAT ym;
    TestKF3D::KFMatrix_FxV dummy1;
    TestKF3D::KFMatrix_FxO dummy2;
    x[j] = x0 + h;
    kf.OnInverseObservationModel(z, yp, dummy1, dummy2);
    x[j] = x0 - h;
    kf.OnInverseObservationModel(z, ym, dummy1, dummy2);
    x[j] = x0;
    for (int k = 0; k < 3; k++)
    {
      EXPECT_NEAR(dyn_dxv(k, j), (yp[k] - ym[k]) / (2 * h), 1e-6) << k << "," << j;
    }
  }
  for (int j = 0; j < 3; j++)
  {
    TestKF3D::KFArray_OBS zp = z;
    TestKF3D::KFArray_OBS zm = z;
    zp[j] += h;
    zm[j] -= h;
    TestKF3D::KFArray_FEAT yp;
    TestKF3D::KFArray_FEAT ym;
    TestKF3D::KFMatrix_FxV dummy1;
    TestKF3D::KFMatrix_FxO dummy2;
    kf.OnInverseObservationModel(zp, yp, dummy1, dummy2);
    kf.OnInverseObservationModel(zm, ym, dummy1, dummy2);
    for (int k = 0; k < 3; k++)
    {
      EXPECT_NEAR(dyn_dhn(k, j), (yp[k] - ym[k]) / (2 * h), 1e-6) << k << "," << j;
    }
  }

  // The forward model of the new landmark gives back the observation:
  x.resize(10);
  x[7] = yn[0];
  x[8] = yn[1];
  x[9] = yn[2];
  TestKF3D::vector_KFArray_OBS pred;
  kf.OnObservationModel({0}, pred);
  for (int k = 0; k < 3; k++)
  {
    EXPECT_NEAR(pred[0][k], z[k], 1e-9);
  }
}

// The transition model must be a function of its input vehicle state, as
// required to estimate its Jacobian numerically.
TEST(CRangeBearingKFSLAM, transitionJacobianMatchesNumericalDerivative)
{
  TestKF3D kf;
  kf.setSF(sfWithSensorPose(mrpt::poses::CPose3D()));
  kf.setAction(makeOdometry(mrpt::poses::CPose2D(0.5, 0.1, 0.2)));
  const mrpt::poses::CPose3DQuat q(mrpt::poses::CPose3D(1.0, 2.0, 0.5, 0.7, -0.3, 0.2));
  auto& x = kf.state();
  x.resize(10);
  for (int i = 0; i < 7; i++)
  {
    x[i] = q[i];
  }

  TestKF3D::KFArray_ACT u;
  kf.OnGetAction(u);
  TestKF3D::KFMatrix_VxV F;
  kf.OnTransitionJacobian(F);

  const double h = 1e-6;
  for (int j = 0; j < 7; j++)
  {
    TestKF3D::KFArray_VEH xp;
    TestKF3D::KFArray_VEH xm;
    for (int i = 0; i < 7; i++)
    {
      xp[i] = xm[i] = x[i];
    }
    xp[j] += h;
    xm[j] -= h;
    bool skip = false;
    kf.OnTransitionModel(u, xp, skip);
    kf.OnTransitionModel(u, xm, skip);
    for (int k = 0; k < 7; k++)
    {
      EXPECT_NEAR(F(k, j), (xp[k] - xm[k]) / (2 * h), 1e-6) << k << "," << j;
    }
  }
}

TEST(CRangeBearingKFSLAM2D, observationModelJacobians)
{
  TestKF2D kf;
  kf.setSF(sfWithSensorPose(mrpt::poses::CPose3D(0.3, -0.1, 0.0, 0.4, 0.0, 0.0)));
  auto& x = kf.state();
  x.resize(5);
  x[0] = 1.0;
  x[1] = 2.0;
  x[2] = 2.9;
  x[3] = -3.0;
  x[4] = 4.5;

  TestKF2D::KFMatrix_OxV Hx;
  TestKF2D::KFMatrix_OxF Hy;
  kf.OnObservationJacobians(0, Hx, Hy);
  for (size_t j = 0; j < 5; j++)
  {
    const Eigen::VectorXd d = numericObsDerivative(kf, j);
    for (int k = 0; k < 2; k++)
    {
      const double analytic = j < 3 ? Hx(k, j) : Hy(k, j - 3);
      EXPECT_NEAR(analytic, d[k], 1e-6) << "row=" << k << " col=" << j;
    }
  }

  // Inverse model, consistent with the forward one:
  TestKF2D::KFArray_OBS z;
  z[0] = 3.0;
  z[1] = -2.5;
  TestKF2D::KFArray_FEAT yn;
  TestKF2D::KFMatrix_FxV dyn_dxv;
  TestKF2D::KFMatrix_FxO dyn_dhn;
  kf.OnInverseObservationModel(z, yn, dyn_dxv, dyn_dhn);
  x[3] = yn[0];
  x[4] = yn[1];
  TestKF2D::vector_KFArray_OBS pred;
  kf.OnObservationModel({0}, pred);
  EXPECT_NEAR(pred[0][0], z[0], 1e-9);
  EXPECT_NEAR(pred[0][1], z[1], 1e-9);
}

// ------------------- Data association of bearings near +-pi -------------------

namespace
{
template <class FILTER>
size_t mapLandmarksBehindTheRobot(FILTER& f)
{
  // Two landmarks almost exactly behind the robot, whose observed bearings
  // jump between +pi and -pi due to the noise:
  const std::vector<mrpt::math::TPoint3D> pts = {
      {-6.0,  0.01, 0.0},
      {-5.0, -0.02, 0.0},
      { 4.0,   1.0, 0.0},
      { 5.0,  -2.0, 0.0}
  };
  mrpt::maps::CLandmarksMap lms;
  int64_t id = 0;
  for (const auto& p : pts)
  {
    mrpt::maps::CLandmark lm;
    lm.ID = id++;
    lm.pose_mean = p;
    lms.landmarks.push_back(lm);
  }

  f.options.std_sensor_yaw = mrpt::DEG2RAD(0.5f);
  f.options.std_sensor_range = 0.01f;

  mrpt::poses::CPose2D gtPose(0, 0, 0);
  for (int i = 0; i < 12; i++)
  {
    const mrpt::poses::CPose2D incr(i == 0 ? 0.0 : 0.1, 0, 0);
    gtPose = gtPose + incr;
    auto obs = mrpt::obs::CObservationBearingRange::Create();
    obs->minSensorDistance = 0;
    obs->maxSensorDistance = 100.0f;
    obs->fieldOfView_yaw = 2 * M_PIf;
    obs->fieldOfView_pitch = M_PIf;
    lms.simulateRangeBearingReadings(
        mrpt::poses::CPose3D(gtPose), mrpt::poses::CPose3D(), *obs, false /*no IDs*/, 0.01,
        mrpt::DEG2RAD(0.5), 0.0);
    auto sf = mrpt::obs::CSensoryFrame::Create();
    sf->insert(obs);
    auto acts = makeOdometry(incr);
    f.processActionObservation(acts, sf);
  }
  return f.getNumberOfLandmarksInTheMap();
}
}  // namespace

TEST(CRangeBearingKFSLAM2D, landmarksBehindTheRobotAreNotDuplicated)
{
  mrpt::random::getRandomGenerator().randomize(1);
  CRangeBearingKFSLAM2D f;
  EXPECT_EQ(mapLandmarksBehindTheRobot(f), 4U);
}

TEST(CRangeBearingKFSLAM, landmarksBehindTheRobotAreNotDuplicated)
{
  mrpt::random::getRandomGenerator().randomize(1);
  CRangeBearingKFSLAM f;
  EXPECT_EQ(mapLandmarksBehindTheRobot(f), 4U);
}

TEST(CRangeBearingKFSLAM, offDiagonalBlocksApproximationErrorValue)
{
  mrpt::random::getRandomGenerator().randomize(21);

  const auto lms = makeLandmarks();
  CRangeBearingKFSLAM f;
  f.options.create_simplemap = true;
  f.options.doPartitioningExperiment = true;
  f.options.partitioningMethod = 0;
  runSession(f, lms, 4);

  // Each landmark in its own partition, so all off-diagonal blocks count:
  const std::vector<std::vector<uint32_t>> membership = {{0}, {1}, {2}, {3}};

  // Independent computation, with the absolute values of the information
  // matrix and full 3x3 landmark blocks:
  mrpt::poses::CPose3DQuatPDFGaussian robotPose;
  std::vector<mrpt::math::TPoint3D> lmPos;
  std::map<unsigned int, mrpt::maps::CLandmark::TLandmarkID> lmIDs;
  mrpt::math::CVectorDouble fullState;
  mrpt::math::CMatrixDouble fullCov;
  f.getCurrentState(robotPose, lmPos, lmIDs, fullState, fullCov);
  Eigen::MatrixXd P = fullCov.asEigen();
  for (int i = 0; i < 7; i++)
  {
    P(i, i) = std::max(P(i, i), 1e-6);
  }
  const Eigen::MatrixXd H = P.inverse().cwiseAbs();
  double off = 0;
  for (int i = 0; i < 4; i++)
  {
    for (int j = i + 1; j < 4; j++)
    {
      off += 2 * H.block<3, 3>(7 + 3 * j, 7 + 3 * i).sum();
    }
  }
  const double expected = off / H.block(7, 7, 12, 12).sum();

  EXPECT_NEAR(f.computeOffDiagonalBlocksApproximationError(membership), expected, 1e-6);
}
