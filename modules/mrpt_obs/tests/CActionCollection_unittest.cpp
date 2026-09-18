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
#include <mrpt/serialization/CArchive.h>

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
  // The 6DOF motion model's mean is a numerical approximation, not an exact
  // copy of the input increment, so allow a small tolerance:
  EXPECT_NEAR(act.poseChange.mean.x(), 0.5, 1e-3);
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
