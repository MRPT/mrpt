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

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPoseRandomSampler.h>

template class mrpt::CTraitsTest<mrpt::poses::CPoseRandomSampler>;

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;

TEST(CPoseRandomSampler, SetPosePDF2D_GaussianAndSample)
{
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  CPosePDFGaussian pdf(CPose2D(1, 2, 0.1), cov);

  CPoseRandomSampler sampler;
  EXPECT_FALSE(sampler.isPrepared());
  sampler.setPosePDF(pdf);
  EXPECT_TRUE(sampler.isPrepared());

  CPose2D sample;
  sampler.drawSample(sample);

  CPose3D sample3d;
  sampler.drawSample(sample3d);  // 2D pdf, ask for 3D sample
  EXPECT_NEAR(sample3d.z(), 0.0, 1e-9);

  CPose2D meanOut;
  sampler.getSamplingMean2D(meanOut);
  EXPECT_NEAR(meanOut.x(), 1.0, 1e-6);

  CPose3D mean3dOut;
  sampler.getSamplingMean3D(mean3dOut);
  EXPECT_NEAR(mean3dOut.x(), 1.0, 1e-6);

  CMatrixDouble33 c2 = sampler.getOriginalPDFCov2D();
  EXPECT_NEAR(c2(0, 0), 0.01, 1e-6);
  CMatrixDouble33 c2b;
  CMatrixDouble c2c;
  CMatrixDouble66 c3b;
  CMatrixDouble c3c;
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  // Exercise the deprecated output-argument overloads for coverage:
  sampler.getOriginalPDFCov2D(c2b);
  sampler.getOriginalPDFCov2D(c2c);
  sampler.getOriginalPDFCov3D(c3b);
  sampler.getOriginalPDFCov3D(c3c);
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

  CMatrixDouble66 c3 = sampler.getOriginalPDFCov3D();
  (void)c3;
}

TEST(CPoseRandomSampler, SetPosePDF2D_Particles)
{
  CPosePDFParticles particles(50);
  particles.resetDeterministic(mrpt::math::TPose2D(1, 1, 0), 50);

  CPoseRandomSampler sampler;
  sampler.setPosePDF(particles);
  CPose2D sample;
  sampler.drawSample(sample);
  EXPECT_NEAR(sample.x(), 1.0, 1e-6);
}

TEST(CPoseRandomSampler, SetPosePDF3D_GaussianAndParticles)
{
  CMatrixDouble66 cov;
  cov.setIdentity();
  cov *= 0.01;
  CPose3DPDFGaussian pdf(CPose3D(1, 2, 3, 0.1, 0.2, 0.3), cov);

  CPoseRandomSampler sampler;
  sampler.setPosePDF(pdf);
  CPose3D sample;
  sampler.drawSample(sample);

  CPose2D sample2d;
  sampler.drawSample(sample2d);  // 3D pdf, ask for 2D sample

  CPose3DPDFParticles particles;
  particles.resetDeterministic(mrpt::math::TPose3D(1, 1, 1, 0, 0, 0), 20);
  CPoseRandomSampler sampler2;
  sampler2.setPosePDF(particles);
  CPose3D sample3;
  sampler2.drawSample(sample3);
  EXPECT_NEAR(sample3.x(), 1.0, 1e-6);
}

TEST(CPoseRandomSampler, CopyMoveAndErrors)
{
  CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.01;
  CPosePDFGaussian pdf(CPose2D(1, 2, 0.1), cov);

  CPoseRandomSampler sampler;
  sampler.setPosePDF(pdf);

  CPoseRandomSampler copy(sampler);
  EXPECT_TRUE(copy.isPrepared());

  CPoseRandomSampler assigned;
  assigned = sampler;
  EXPECT_TRUE(assigned.isPrepared());

  CPoseRandomSampler moved(std::move(copy));
  EXPECT_TRUE(moved.isPrepared());

  CPoseRandomSampler movedAssigned;
  movedAssigned = std::move(assigned);
  EXPECT_TRUE(movedAssigned.isPrepared());

  CPoseRandomSampler empty;
  EXPECT_THROW((void)empty.getOriginalPDFCov2D(), std::exception);
  CPose2D p;
  EXPECT_THROW(empty.drawSample(p), std::exception);
}
