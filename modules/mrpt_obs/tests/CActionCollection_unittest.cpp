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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>

#include <cmath>
#include <cstring>
#include <sstream>

using namespace mrpt::obs;
using namespace mrpt::poses;

// ------------------- CActionRobotMovement2D -------------------

TEST(CActionRobotMovement2D, ComputeFromOdometryGaussianModel)
{
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  opts.modelSelection = CActionRobotMovement2D::mmGaussian;
  act.computeFromOdometry(CPose2D(0.5, 0.1, 0.05), opts);

  ASSERT_TRUE(act.poseChange);
  EXPECT_EQ(act.estimationMethod, CActionRobotMovement2D::emOdometry);

  CPose2D mean;
  mrpt::math::CMatrixDouble33 cov;
  act.poseChange->getCovarianceAndMean(cov, mean);
  EXPECT_NEAR(mean.x(), 0.5, 1e-6);
}

// The covariance of the Gaussian model must be J*Sigma*J^T, with J the
// Jacobian of f(u) = H(u_phi) * u (see the motion models tutorial). Checked
// against a numerical Jacobian of f().
TEST(CActionRobotMovement2D, GaussianModelCovarianceMatchesJacobian)
{
  CActionRobotMovement2D::TMotionModelOptions opts;
  opts.modelSelection = CActionRobotMovement2D::mmGaussian;
  const auto& g = opts.gaussianModel;

  const double u[3] = {0.4, 0.15, 0.6};
  CActionRobotMovement2D act;
  act.computeFromOdometry(CPose2D(u[0], u[1], u[2]), opts);

  CPose2D mean;
  mrpt::math::CMatrixDouble33 cov;
  act.poseChange->getCovarianceAndMean(cov, mean);

  const auto f = [](const double v[3], double out[3])
  {
    const double c = std::cos(v[2] / 2);
    const double s = std::sin(v[2] / 2);
    out[0] = c * v[0] - s * v[1];
    out[1] = s * v[0] + c * v[1];
    out[2] = v[2];
  };

  double J[3][3];
  const double h = 1e-6;
  for (int col = 0; col < 3; col++)
  {
    double up[3] = {u[0], u[1], u[2]};
    double um[3] = {u[0], u[1], u[2]};
    up[col] += h;
    um[col] -= h;
    double fp[3];
    double fm[3];
    f(up, fp);
    f(um, fm);
    for (int row = 0; row < 3; row++)
    {
      J[row][col] = (fp[row] - fm[row]) / (2 * h);
    }
  }

  const double dist = std::sqrt(u[0] * u[0] + u[1] * u[1]);
  const double stdXY = g.minStdXY + g.a1 * dist + g.a2 * std::abs(u[2]);
  const double stdPhi = g.minStdPHI + g.a3 * dist + g.a4 * std::abs(u[2]);
  const double sigma[3] = {stdXY * stdXY, stdXY * stdXY, stdPhi * stdPhi};

  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      double expected = 0;
      for (int k = 0; k < 3; k++)
      {
        expected += J[r][k] * sigma[k] * J[c][k];
      }
      EXPECT_NEAR(cov(r, c), expected, 1e-9) << "r=" << r << " c=" << c;
    }
  }
}

TEST(CActionRobotMovement2D, ComputeFromOdometryThrunModel)
{
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  opts.modelSelection = CActionRobotMovement2D::mmThrun;
  opts.thrunModel.nParticlesCount = 50;
  act.computeFromOdometry(CPose2D(0.3, -0.1, 0.02), opts);
  ASSERT_TRUE(act.poseChange);

  CPose2D sample;
  act.drawSingleSample(sample);  // should not throw

  act.prepareFastDrawSingleSamples();
  CPose2D fastSample;
  act.fastDrawSingleSample(fastSample);
}

TEST(CActionRobotMovement2D, DrawSingleSampleGaussianModel)
{
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  opts.modelSelection = CActionRobotMovement2D::mmGaussian;
  act.computeFromOdometry(CPose2D(0.2, 0.0, 0.0), opts);

  CPose2D sample;
  act.drawSingleSample(sample);

  act.prepareFastDrawSingleSamples();
  CPose2D fastSample;
  act.fastDrawSingleSample(fastSample);
}

TEST(CActionRobotMovement2D, ComputeFromEncoders)
{
  CActionRobotMovement2D act;
  act.hasEncodersInfo = true;
  act.encoderLeftTicks = 100;
  act.encoderRightTicks = 110;
  act.computeFromEncoders(0.001, 0.001, 0.3);
  ASSERT_TRUE(act.poseChange);
}

TEST(CActionRobotMovement2D, VelocityAccessors)
{
  CActionRobotMovement2D act;
  act.hasVelocities = true;
  act.velocityLocal.vx = 1.5;
  act.velocityLocal.omega = 0.2;
  EXPECT_DOUBLE_EQ(act.velocityLin(), 1.5);
  EXPECT_DOUBLE_EQ(act.velocityAng(), 0.2);
}

TEST(CActionRobotMovement2D, GetDescriptionAsText)
{
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  act.computeFromOdometry(CPose2D(0.1, 0.1, 0.1), opts);
  std::stringstream ss;
  act.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CActionRobotMovement2D, SerializationRoundtrip)
{
  CActionRobotMovement2D act1;
  CActionRobotMovement2D::TMotionModelOptions opts;
  act1.computeFromOdometry(CPose2D(0.4, 0.2, 0.03), opts);
  act1.hasEncodersInfo = true;
  act1.encoderLeftTicks = 10;
  act1.encoderRightTicks = 12;
  act1.hasVelocities = true;
  act1.velocityLocal.vx = 0.5;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << act1;
  buf.Seek(0);

  CActionRobotMovement2D act2;
  arch >> act2;

  EXPECT_EQ(act2.hasEncodersInfo, act1.hasEncodersInfo);
  EXPECT_EQ(act2.encoderLeftTicks, act1.encoderLeftTicks);
  EXPECT_EQ(act2.hasVelocities, act1.hasVelocities);
  EXPECT_DOUBLE_EQ(act2.velocityLocal.vx, act1.velocityLocal.vx);
}

TEST(CActionRobotMovement2D, SerializationRoundtripNonOdometryEstimationMethod)
{
  // When estimationMethod != emOdometry, the PDF itself (not the raw
  // odometry increment) is what gets serialized:
  CActionRobotMovement2D act1;
  act1.estimationMethod = CActionRobotMovement2D::emScan2DMatching;
  auto pdf = std::make_shared<mrpt::poses::CPosePDFGaussian>();
  pdf->mean = CPose2D(0.3, 0.1, 0.02);
  act1.poseChange = pdf;
  act1.hasVelocities = false;
  act1.hasEncodersInfo = false;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << act1;
  buf.Seek(0);

  CActionRobotMovement2D act2;
  arch >> act2;
  EXPECT_EQ(act2.estimationMethod, CActionRobotMovement2D::emScan2DMatching);
  EXPECT_FALSE(act2.hasVelocities);
  EXPECT_FALSE(act2.hasEncodersInfo);
  ASSERT_TRUE(act2.poseChange);
  CPose2D mean;
  mrpt::math::CMatrixDouble33 cov;
  act2.poseChange->getCovarianceAndMean(cov, mean);
  EXPECT_NEAR(mean.x(), 0.3, 1e-6);
}

namespace
{
void writeObjectHeader(mrpt::serialization::CArchive& arch, const char* className, uint8_t version)
{
  const auto len = static_cast<int8_t>(strlen(className) | 0x80);
  arch << len;
  arch.WriteBuffer(className, strlen(className));
  arch << version;
}
void writeObjectFooter(mrpt::serialization::CArchive& arch) { arch << static_cast<uint8_t>(0x88); }
}  // namespace

TEST(CActionRobotMovement2D, DeserializeLegacyVersion0)
{
  // Version 0: only the PDF and the estimation method are stored; other
  // fields default to zero/false.
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeObjectHeader(arch, "CActionRobotMovement2D", 0);

  mrpt::poses::CPosePDFGaussian::Ptr pdf = std::make_shared<mrpt::poses::CPosePDFGaussian>();
  pdf->mean = CPose2D(0.2, 0.1, 0.0);
  arch << std::static_pointer_cast<mrpt::poses::CPosePDF>(pdf);
  arch << static_cast<int32_t>(CActionRobotMovement2D::emOdometry);

  writeObjectFooter(arch);
  buf.Seek(0);

  CActionRobotMovement2D act;
  arch >> act;

  EXPECT_EQ(act.estimationMethod, CActionRobotMovement2D::emOdometry);
  EXPECT_FALSE(act.hasVelocities);
  EXPECT_FALSE(act.hasEncodersInfo);
  EXPECT_NEAR(act.rawOdometryIncrementReading.x(), 0.2, 1e-6);
}

// ------------------- CActionRobotMovement3D -------------------

TEST(CActionRobotMovement3D, ComputeFromOdometry)
{
  CActionRobotMovement3D act;
  CActionRobotMovement3D::TMotionModelOptions opts;
  act.computeFromOdometry(CPose3D(0.5, 0.1, 0.0, 0.05, 0.0, 0.0), opts);
  // The 6DOF motion model's mean is a Monte Carlo estimate, so allow a small
  // tolerance:
  EXPECT_NEAR(act.poseChange.mean.x(), 0.5, 1e-3);
}

// Without model noise, the 6DOF model must reproduce the odometry increment
TEST(CActionRobotMovement3D, mm6DOFNoiseFreeMeanIsTheIncrement)
{
  mrpt::random::getRandomGenerator().randomize(1);
  for (const auto& incr :
       {CPose3D(1.0, 0.2, 0.3, 0.4, 0.1, 0.05), CPose3D(1.0, 0.2, 0.0, 0.4, 0.0, 0.0),
        CPose3D(1.0, 0.0, 0.5, 0.0, 0.0, 0.0), CPose3D(-0.5, -0.3, -0.2, -0.6, 0.2, -0.1)})
  {
    CActionRobotMovement3D act;
    CActionRobotMovement3D::TMotionModelOptions o;
    o.mm6DOFModel.additional_std_XYZ = 1e-6f;
    o.mm6DOFModel.additional_std_angle = 1e-6f;
    act.computeFromOdometry(incr, o);
    const auto& m = act.poseChange.mean;
    EXPECT_NEAR(m.x(), incr.x(), 1e-5) << incr;
    EXPECT_NEAR(m.y(), incr.y(), 1e-5) << incr;
    EXPECT_NEAR(m.z(), incr.z(), 1e-5) << incr;
    EXPECT_NEAR(m.yaw(), incr.yaw(), 1e-5) << incr;
    EXPECT_NEAR(m.pitch(), incr.pitch(), 1e-5) << incr;
    EXPECT_NEAR(m.roll(), incr.roll(), 1e-5) << incr;
  }
}

// The uncertainty of mirrored motions (turning left vs right) must be equal
TEST(CActionRobotMovement3D, mm6DOFNoiseIsSymmetric)
{
  std::vector<double> stds;
  for (const double sign : {+1.0, -1.0})
  {
    mrpt::random::getRandomGenerator().randomize(1);
    CActionRobotMovement3D act;
    CActionRobotMovement3D::TMotionModelOptions o;
    o.mm6DOFModel.nParticlesCount = 2000;
    o.mm6DOFModel.a1 = 0.2f;
    o.mm6DOFModel.a2 = 0.01f;
    o.mm6DOFModel.a9 = 0.2f;
    o.mm6DOFModel.a10 = 0.01f;
    act.computeFromOdometry(CPose3D(1.0, sign * 0.3, 0.0, sign * 0.6, 0, 0), o);
    stds.push_back(std::sqrt(act.poseChange.cov(3, 3)));
  }
  // Same random draws, so the yaw std must be the same up to round-off:
  EXPECT_NEAR(stds[0], stds[1], 1e-6);
}

TEST(CActionRobotMovement3D, mmGaussianUsesTheIncrement)
{
  CActionRobotMovement3D act;
  CActionRobotMovement3D::TMotionModelOptions o;
  o.modelSelection = CActionRobotMovement3D::mmGaussian;
  o.mm6DOFModel.additional_std_XYZ = 0.1f;
  o.mm6DOFModel.additional_std_angle = 0.01f;
  const CPose3D incr(1.0, 0.2, 0.3, 0.4, 0.1, 0.05);
  act.computeFromOdometry(incr, o);
  EXPECT_NEAR((act.poseChange.mean.asVectorVal() - incr.asVectorVal()).norm(), 0.0, 1e-9);
  for (int i = 0; i < 6; i++)
  {
    EXPECT_NEAR(act.poseChange.cov(i, i), i < 3 ? 0.01 : 1e-4, 1e-8) << i;
  }
}

TEST(CActionRobotMovement3D, GetDescriptionAsText)
{
  CActionRobotMovement3D act;
  CActionRobotMovement3D::TMotionModelOptions opts;
  act.computeFromOdometry(CPose3D(0.1, 0.0, 0.0, 0.0, 0.0, 0.0), opts);
  std::stringstream ss;
  act.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CActionRobotMovement3D, SerializationRoundtrip)
{
  CActionRobotMovement3D act1;
  CActionRobotMovement3D::TMotionModelOptions opts;
  act1.computeFromOdometry(CPose3D(0.2, 0.1, 0.0, 0.02, 0.01, 0.0), opts);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << act1;
  buf.Seek(0);

  CActionRobotMovement3D act2;
  arch >> act2;
  EXPECT_NEAR(act2.poseChange.mean.x(), act1.poseChange.mean.x(), 1e-9);
}

// ------------------- CAction -------------------

TEST(CAction, GetDescriptionAsTextValue)
{
  CActionRobotMovement2D act;
  act.timestamp = mrpt::Clock::now();
  const std::string txt = act.getDescriptionAsTextValue();
  EXPECT_FALSE(txt.empty());
}

// ------------------- CActionCollection -------------------

TEST(CActionCollection, InsertGetSizeClear)
{
  CActionCollection ac;
  EXPECT_EQ(ac.size(), 0u);

  CActionRobotMovement2D act2d;
  CActionRobotMovement2D::TMotionModelOptions opts2d;
  act2d.computeFromOdometry(CPose2D(0.1, 0, 0), opts2d);
  ac.insert(act2d);
  EXPECT_EQ(ac.size(), 1u);

  CActionRobotMovement3D act3d;
  CActionRobotMovement3D::TMotionModelOptions opts3d;
  act3d.computeFromOdometry(CPose3D(0.1, 0, 0, 0, 0, 0), opts3d);
  ac.insert(act3d);
  EXPECT_EQ(ac.size(), 2u);

  ac.clear();
  EXPECT_EQ(ac.size(), 0u);
}

TEST(CActionCollection, ConstructFromSingleAction)
{
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  act.computeFromOdometry(CPose2D(0.2, 0, 0), opts);
  CActionCollection ac(act);
  EXPECT_EQ(ac.size(), 1u);
}

TEST(CActionCollection, InsertPtrAndGet)
{
  CActionCollection ac;
  auto act = CActionRobotMovement2D::Create();
  CActionRobotMovement2D::TMotionModelOptions opts;
  act->computeFromOdometry(CPose2D(0.3, 0, 0), opts);
  ac.insertPtr(act);
  ASSERT_EQ(ac.size(), 1u);

  auto got = ac.get(0);
  ASSERT_TRUE(got);
  EXPECT_THROW(ac.get(100), std::exception);

  const CActionCollection& cac = ac;
  auto gotConst = cac.get(0);
  ASSERT_TRUE(gotConst);
}

TEST(CActionCollection, GetActionByClass)
{
  CActionCollection ac;
  CActionRobotMovement2D act2d;
  CActionRobotMovement2D::TMotionModelOptions opts2d;
  act2d.computeFromOdometry(CPose2D(0.1, 0, 0), opts2d);
  ac.insert(act2d);

  auto found = ac.getActionByClass<CActionRobotMovement2D>();
  EXPECT_TRUE(found);

  auto notFound = ac.getActionByClass<CActionRobotMovement3D>();
  EXPECT_FALSE(notFound);

  // Non-const overload:
  auto found2 = ac.getActionByClass<CActionRobotMovement2D>();
  EXPECT_TRUE(found2);
}

TEST(CActionCollection, GetBestMovementEstimation)
{
  CActionCollection ac;
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  act.computeFromOdometry(CPose2D(0.4, 0, 0), opts);
  ac.insert(act);

  auto best = ac.getBestMovementEstimation();
  ASSERT_TRUE(best);

  auto byType = ac.getMovementEstimationByType(CActionRobotMovement2D::emOdometry);
  ASSERT_TRUE(byType);
  auto byWrongType = ac.getMovementEstimationByType(CActionRobotMovement2D::emScan2DMatching);
  EXPECT_FALSE(byWrongType);
}

TEST(CActionCollection, GetBestMovementEstimationEmptyCollection)
{
  CActionCollection ac;
  EXPECT_FALSE(ac.getBestMovementEstimation());
}

TEST(CActionCollection, GetFirstMovementEstimationMean)
{
  CActionCollection ac;
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  act.computeFromOdometry(CPose2D(0.5, 0.1, 0.02), opts);
  ac.insert(act);

  CPose3D meanOldApi;
  EXPECT_TRUE(ac.getFirstMovementEstimationMean(meanOldApi));

  const auto meanOpt = ac.getFirstMovementEstimationMean();
  ASSERT_TRUE(meanOpt.has_value());
  EXPECT_NEAR(meanOpt->x(), 0.5, 1e-6);

  CActionCollection emptyAc;
  CPose3D dummy;
  EXPECT_FALSE(emptyAc.getFirstMovementEstimationMean(dummy));
  EXPECT_FALSE(emptyAc.getFirstMovementEstimationMean().has_value());
}

TEST(CActionCollection, GetFirstMovementEstimation)
{
  CActionCollection ac;
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  act.computeFromOdometry(CPose2D(0.3, 0.0, 0.0), opts);
  ac.insert(act);

  mrpt::poses::CPose3DPDFGaussian outOldApi;
  EXPECT_TRUE(ac.getFirstMovementEstimation(outOldApi));

  const auto out = ac.getFirstMovementEstimation();
  ASSERT_TRUE(out.has_value());

  CActionCollection emptyAc;
  EXPECT_FALSE(emptyAc.getFirstMovementEstimation().has_value());
}

TEST(CActionCollection, EraseByIndexAndIterator)
{
  CActionCollection ac;
  for (int i = 0; i < 3; i++)
  {
    CActionRobotMovement2D act;
    CActionRobotMovement2D::TMotionModelOptions opts;
    act.computeFromOdometry(CPose2D(0.1 * i, 0, 0), opts);
    ac.insert(act);
  }
  ASSERT_EQ(ac.size(), 3u);

  ac.eraseByIndex(1);
  EXPECT_EQ(ac.size(), 2u);
  EXPECT_THROW(ac.eraseByIndex(100), std::exception);

  size_t count = 0;
  for (auto it = ac.begin(); it != ac.end(); ++it) count++;
  EXPECT_EQ(count, 2u);

  auto it = ac.begin();
  ac.erase(it);
  EXPECT_EQ(ac.size(), 1u);

  const CActionCollection& cac = ac;
  size_t constCount = 0;
  for (auto cit = cac.begin(); cit != cac.end(); ++cit) constCount++;
  EXPECT_EQ(constCount, 1u);
}

TEST(CActionCollection, SerializationRoundtrip)
{
  CActionCollection ac1;
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  act.computeFromOdometry(CPose2D(0.25, 0.05, 0.01), opts);
  ac1.insert(act);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << ac1;
  buf.Seek(0);

  CActionCollection ac2;
  arch >> ac2;
  EXPECT_EQ(ac2.size(), ac1.size());
}
