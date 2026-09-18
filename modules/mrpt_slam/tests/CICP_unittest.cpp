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
