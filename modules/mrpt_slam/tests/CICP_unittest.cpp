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
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/random.h>
#include <mrpt/slam/CICP.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace std;

class ICPTests : public ::testing::Test
{
 protected:
  void SetUp() override {}
  void TearDown() override {}
  void align2scans(const TICPAlgorithm icp_method)
  {
    CSimplePointsMap m1, m2;
    CICP::TReturnInfo info;
    CICP ICP;

    // Load scans:
    CObservation2DRangeScan scan1;
    stock_observations::example2DRangeScan(scan1, 0);

    CObservation2DRangeScan scan2;
    stock_observations::example2DRangeScan(scan2, 1);

    // Build the points maps from the scans:
    m1.insertObservation(scan1);
    m2.insertObservation(scan2);

    // -----------------------------------------------------
    ICP.options.ICP_algorithm = icp_method;

    ICP.options.maxIterations = 100;
    ICP.options.thresholdAng = DEG2RAD(10.0f);
    ICP.options.thresholdDist = 0.75f;
    ICP.options.ALFA = 0.5f;
    ICP.options.smallestThresholdDist = 0.05f;
    ICP.options.doRANSAC = false;
    // ICP.options.dumpToConsole();
    // -----------------------------------------------------
    CPose2D initialPose(0.8f, 0.0f, static_cast<float>(DEG2RAD(0.0f)));

    CPosePDF::Ptr pdf = ICP.Align(&m1, &m2, initialPose, info);

    const CPose2D good_pose(0.820, 0.084, 8.73_deg);

    EXPECT_NEAR(good_pose.distanceTo(pdf->getMeanVal()), 0, 0.02);
  }

  static void generateSyntheticCloud(CSimplePointsMap& out_map, const CPose3D& origin)
  {
    // Generate a grid of points on a plane to serve as a synthetic 3D scan.
    for (int ix = -10; ix <= 10; ix++)
    {
      for (int iy = -10; iy <= 10; iy++)
      {
        mrpt::math::TPoint3D pt(ix * 0.2, iy * 0.2, 0.0);
        mrpt::math::TPoint3D g;
        origin.composePoint(pt.x, pt.y, pt.z, g.x, g.y, g.z);
        out_map.insertPointFast(
            static_cast<float>(g.x), static_cast<float>(g.y), static_cast<float>(g.z));
      }
    }
    out_map.mark_as_modified();
  }
};

TEST_F(ICPTests, AlignScans_icpClassic) { align2scans(icpClassic); }
TEST_F(ICPTests, AlignScans_icpLevenbergMarquardt) { align2scans(icpLevenbergMarquardt); }

TEST_F(ICPTests, SyntheticICP3D)
{
  CPose3D SCAN2_POSE_ERROR(0.05, -0.03, 0.02, 0.01, 0.02, 0.02);

  // Generate two synthetic 3D point clouds from the same reference frame.
  CSimplePointsMap M1, M2;
  generateSyntheticCloud(M1, CPose3D());
  generateSyntheticCloud(M2, CPose3D());

  // Create the wrongly-localized M2:
  CSimplePointsMap M2_noisy;
  M2_noisy = M2;
  M2_noisy.changeCoordinatesReference(SCAN2_POSE_ERROR);

  // Do the ICP-3D
  CICP icp;
  CICP::TReturnInfo icp_info;

  icp.options.thresholdDist = 0.40f;
  icp.options.thresholdAng = 0;

  CPose3DPDF::Ptr pdf = icp.Align3D(
      &M2_noisy,  // Map to align
      &M1,        // Reference map
      CPose3D(),  // Initial gross estimate
      icp_info);

  CPose3D mean = pdf->getMeanVal();

  // Checks:
  EXPECT_NEAR(0, (mean.asVectorVal() - SCAN2_POSE_ERROR.asVectorVal()).array().abs().mean(), 0.02)
      << "ICP output: mean= " << mean << "\n"
      << "Real displacement: " << SCAN2_POSE_ERROR << "\n";
}

TEST_F(ICPTests, CovarianceMethods)
{
  CSimplePointsMap m1, m2;
  CObservation2DRangeScan scan1, scan2;
  stock_observations::example2DRangeScan(scan1, 0);
  stock_observations::example2DRangeScan(scan2, 1);
  m1.insertObservation(scan1);
  m2.insertObservation(scan2);

  const CPose2D initialPose(0.8, 0.0, 0.0);

  for (const auto method : {icpCovLinealMSE, icpCovFiniteDifferences})
  {
    CICP icp;
    CICP::TReturnInfo info;
    icp.options.ICP_algorithm = icpClassic;
    icp.options.maxIterations = 40;
    icp.options.thresholdDist = 0.75f;
    icp.options.thresholdAng = DEG2RAD(10.0f);
    icp.options.ALFA = 0.5f;
    icp.options.smallestThresholdDist = 0.05f;
    icp.options.skip_cov_calculation = false;
    icp.options.ICP_covariance_method = method;

    CPosePDF::Ptr pdf = icp.Align(&m1, &m2, initialPose, info);
    ASSERT_TRUE(pdf);

    const auto gauss = std::dynamic_pointer_cast<CPosePDFGaussian>(pdf);
    ASSERT_TRUE(gauss);
    // A valid, positive-definite covariance:
    EXPECT_GT(gauss->cov(0, 0), 0.0);
    EXPECT_GT(gauss->cov(1, 1), 0.0);
    EXPECT_GT(gauss->cov(2, 2), 0.0);
  }
}

TEST_F(ICPTests, InvalidCovarianceMethodThrows)
{
  CSimplePointsMap m1, m2;
  CObservation2DRangeScan scan1, scan2;
  stock_observations::example2DRangeScan(scan1, 0);
  stock_observations::example2DRangeScan(scan2, 1);
  m1.insertObservation(scan1);
  m2.insertObservation(scan2);

  CICP icp;
  CICP::TReturnInfo info;
  icp.options.ICP_algorithm = icpClassic;
  icp.options.maxIterations = 10;
  icp.options.skip_cov_calculation = false;
  icp.options.ICP_covariance_method = static_cast<TICPCovarianceMethod>(99);

  EXPECT_THROW(icp.Align(&m1, &m2, CPose2D(0.8, 0, 0), info), std::exception);
}

TEST_F(ICPTests, SkipQualityAndCovarianceCalculations)
{
  CSimplePointsMap m1, m2;
  CObservation2DRangeScan scan1, scan2;
  stock_observations::example2DRangeScan(scan1, 0);
  stock_observations::example2DRangeScan(scan2, 1);
  m1.insertObservation(scan1);
  m2.insertObservation(scan2);

  CICP icp;
  CICP::TReturnInfo info;
  icp.options.ICP_algorithm = icpClassic;
  icp.options.maxIterations = 20;
  icp.options.skip_cov_calculation = true;
  icp.options.skip_quality_calculation = true;

  CPosePDF::Ptr pdf = icp.Align(&m1, &m2, CPose2D(0.8, 0, 0), info);
  ASSERT_TRUE(pdf);
  // With the quality estimation skipped, it equals the correspondences ratio:
  EXPECT_NEAR(info.quality, info.goodness, 1e-9);
}

TEST_F(ICPTests, AlignPDFWithGaussianInitialGuess)
{
  CSimplePointsMap m1, m2;
  CObservation2DRangeScan scan1, scan2;
  stock_observations::example2DRangeScan(scan1, 0);
  stock_observations::example2DRangeScan(scan2, 1);
  m1.insertObservation(scan1);
  m2.insertObservation(scan2);

  CICP icp;
  CICP::TReturnInfo info;
  icp.options.ICP_algorithm = icpClassic;
  icp.options.maxIterations = 40;
  icp.options.thresholdDist = 0.75f;
  icp.options.thresholdAng = DEG2RAD(10.0f);
  icp.options.ALFA = 0.5f;
  icp.options.smallestThresholdDist = 0.05f;

  CPosePDFGaussian init;
  init.mean = CPose2D(0.8, 0.0, 0.0);
  init.cov.setDiagonal(0.01);

  CPosePDF::Ptr pdf = icp.AlignPDF(&m1, &m2, init, info);
  ASSERT_TRUE(pdf);

  const CPose2D good_pose(0.820, 0.084, 8.73_deg);
  EXPECT_NEAR(good_pose.distanceTo(pdf->getMeanVal()), 0, 0.05);
}

TEST_F(ICPTests, Align3DPDFWithGaussianInitialGuess)
{
  const CPose3D poseError(0.05, -0.03, 0.02, 0.01, 0.02, 0.02);

  CSimplePointsMap M1, M2;
  generateSyntheticCloud(M1, CPose3D());
  generateSyntheticCloud(M2, CPose3D());

  CSimplePointsMap M2_noisy = M2;
  M2_noisy.changeCoordinatesReference(poseError);

  CICP icp;
  CICP::TReturnInfo info;
  icp.options.thresholdDist = 0.40f;
  icp.options.thresholdAng = 0;

  mrpt::poses::CPose3DPDFGaussian init;
  init.cov.setDiagonal(0.01);

  CPose3DPDF::Ptr pdf = icp.Align3DPDF(&M2_noisy, &M1, init, info);
  ASSERT_TRUE(pdf);

  const CPose3D mean = pdf->getMeanVal();
  EXPECT_NEAR(0, (mean.asVectorVal() - poseError.asVectorVal()).array().abs().mean(), 0.03);
}

namespace
{
/** Walls of a non-symmetric 2D room */
std::vector<TPoint2D> roomWallPoints()
{
  std::vector<TPoint2D> pts;
  for (double t = -4; t <= 6; t += 0.25)
  {
    pts.emplace_back(t, -3.0);
    pts.emplace_back(t, 4.0);
  }
  for (double t = -3; t <= 4; t += 0.25)
  {
    pts.emplace_back(-4.0, t);
    pts.emplace_back(6.0, t * 0.5 + 1.0);
  }
  return pts;
}

/** Both maps sample the same wall points with independent noise */
void noisyScanPair(const CPose2D& gt, double sigma, CSimplePointsMap& m1, CSimplePointsMap& m2)
{
  auto& rng = mrpt::random::getRandomGenerator();
  m1.clear();
  m2.clear();
  for (const auto& p : roomWallPoints())
  {
    m1.insertPoint(
        static_cast<float>(p.x + sigma * rng.drawGaussian1D_normalized()),
        static_cast<float>(p.y + sigma * rng.drawGaussian1D_normalized()), 0);
    const TPoint2D l = gt.inverseComposePoint(p);
    m2.insertPoint(
        static_cast<float>(l.x + sigma * rng.drawGaussian1D_normalized()),
        static_cast<float>(l.y + sigma * rng.drawGaussian1D_normalized()), 0);
  }
}
}  // namespace

// The covariance must be in the units of the pose, and agree with the actual
// spread of the ICP estimates (Monte Carlo):
TEST_F(ICPTests, CovarianceIsCalibrated)
{
  mrpt::random::getRandomGenerator().randomize(123);
  const CPose2D gt(0.2, -0.1, 0.05);

  for (const auto algo : {icpClassic, icpLevenbergMarquardt})
  {
    for (const double sigma : {0.005, 0.02})
    {
      const int N = 60;
      std::vector<Eigen::Vector3d> ests;
      Eigen::Matrix3d meanCov = Eigen::Matrix3d::Zero();
      for (int it = 0; it < N; it++)
      {
        CSimplePointsMap m1;
        CSimplePointsMap m2;
        noisyScanPair(gt, sigma, m1, m2);
        CICP icp;
        icp.options.ICP_algorithm = algo;
        icp.options.ICP_covariance_method = icpCovFiniteDifferences;
        icp.options.thresholdDist = 0.2;
        icp.options.thresholdAng = 0;
        icp.options.smallestThresholdDist = 0.05;
        icp.options.maxIterations = 100;
        // Both maps are noisy:
        icp.options.covariance_varPoints = 2 * sigma * sigma;
        CICP::TReturnInfo info;
        const auto pdf = icp.AlignPDF(&m1, &m2, CPosePDFGaussian(CPose2D(0.18, -0.08, 0.04)), info);
        CPosePDFGaussian g;
        g.copyFrom(*pdf);
        ests.emplace_back(g.mean.x(), g.mean.y(), g.mean.phi());
        meanCov += g.cov.asEigen() / N;
      }
      Eigen::Vector3d mu = Eigen::Vector3d::Zero();
      for (const auto& e : ests)
      {
        mu += e / N;
      }
      Eigen::Matrix3d mcCov = Eigen::Matrix3d::Zero();
      for (const auto& e : ests)
      {
        mcCov += (e - mu) * (e - mu).transpose() / (N - 1);
      }
      for (int i = 0; i < 3; i++)
      {
        const double ratio = std::sqrt(meanCov(i, i) / mcCov(i, i));
        EXPECT_GT(ratio, 0.6) << "algo=" << algo << " sigma=" << sigma << " i=" << i;
        EXPECT_LT(ratio, 1.6) << "algo=" << algo << " sigma=" << sigma << " i=" << i;
      }
    }
  }
}

// The RANSAC step must use the correspondences of the ICP solution, not those
// used to evaluate the quality at perturbed poses.
TEST_F(ICPTests, RansacIndependentOfQualityCalculation)
{
  mrpt::random::getRandomGenerator().randomize(7);
  CSimplePointsMap m1;
  CSimplePointsMap m2;
  noisyScanPair(CPose2D(0.2, -0.1, 0.05), 0.01, m1, m2);

  std::vector<CPose2D> results;
  for (const bool skipQuality : {true, false})
  {
    mrpt::random::getRandomGenerator().randomize(1);
    CICP icp;
    icp.options.thresholdDist = 0.2;
    icp.options.thresholdAng = 0;
    icp.options.smallestThresholdDist = 0.05;
    icp.options.doRANSAC = true;
    icp.options.skip_quality_calculation = skipQuality;
    CICP::TReturnInfo info;
    const auto pdf = icp.AlignPDF(&m1, &m2, CPosePDFGaussian(CPose2D(0.18, -0.08, 0.04)), info);
    ASSERT_TRUE(std::dynamic_pointer_cast<CPosePDFSOG>(pdf));
    results.push_back(pdf->getMeanVal());
  }
  EXPECT_NEAR(results[0].x(), results[1].x(), 1e-9);
  EXPECT_NEAR(results[0].y(), results[1].y(), 1e-9);
  EXPECT_NEAR(results[0].phi(), results[1].phi(), 1e-9);
}

// 3D ICP: rigid fit (no scale) and a non-null covariance.
TEST_F(ICPTests, ICP3DRigidAndCovariance)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(5);
  // Points on three orthogonal planes, far from the origin:
  std::vector<TPoint3D> pts;
  for (double a = 0; a <= 4.0; a += 0.2)
  {
    for (double b = 0; b <= 3.0; b += 0.2)
    {
      pts.emplace_back(20.0 + a, 10.0 + b, 0.0);
      pts.emplace_back(20.0, 10.0 + a, b);
      pts.emplace_back(20.0 + a, 10.0, b);
    }
  }
  const CPose3D gt(0.03, -0.02, 0.01, 0.002, 0.001, -0.001);
  const double sigma = 0.01;
  CSimplePointsMap m1;
  CSimplePointsMap m2;
  for (const auto& p : pts)
  {
    m1.insertPoint(
        static_cast<float>(p.x + sigma * rng.drawGaussian1D_normalized()),
        static_cast<float>(p.y + sigma * rng.drawGaussian1D_normalized()),
        static_cast<float>(p.z + sigma * rng.drawGaussian1D_normalized()));
    const auto l = gt.inverseComposePoint(p);
    m2.insertPoint(
        static_cast<float>(l.x + sigma * rng.drawGaussian1D_normalized()),
        static_cast<float>(l.y + sigma * rng.drawGaussian1D_normalized()),
        static_cast<float>(l.z + sigma * rng.drawGaussian1D_normalized()));
  }
  CICP icp;
  icp.options.thresholdDist = 0.15;
  icp.options.thresholdAng = 0;
  icp.options.smallestThresholdDist = 0.03;
  icp.options.maxIterations = 200;
  icp.options.covariance_varPoints = 2 * sigma * sigma;
  CICP::TReturnInfo info;
  const auto pdf = icp.Align3DPDF(&m1, &m2, CPose3DPDFGaussian(CPose3D()), info);
  CPose3DPDFGaussian g;
  g.copyFrom(*pdf);

  // Error within ~4 sigmas of the reported covariance, which is not null:
  const auto err = g.mean.asVectorVal() - gt.asVectorVal();
  for (int i = 0; i < 6; i++)
  {
    ASSERT_GT(g.cov(i, i), 0.0);
    EXPECT_LT(std::abs(err[i]), 4 * std::sqrt(g.cov(i, i)) + 1e-4) << "i=" << i;
  }
  EXPECT_LT(std::sqrt(g.cov(0, 0)), 0.05);
}
