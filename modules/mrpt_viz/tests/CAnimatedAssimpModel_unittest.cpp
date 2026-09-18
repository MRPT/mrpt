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
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CAnimatedAssimpModel.h>
#include <mrpt/viz/config.h>  // MRPT_HAS_ASSIMP
#include <test_mrpt_common.h>

#if MRPT_HAS_ASSIMP

using namespace mrpt::viz;

namespace
{
/** A glTF 2.0 model with two joints and one 3-keyframe rotation animation,
 *  small enough to keep in the test data directory. */
std::string skinnedModelPath()
{
  //! JS_PRELOAD_FILE <tests/skinned_model.gltf>
  return mrpt::UNITTEST_BASEDIR() + std::string("/tests/skinned_model.gltf");
}

CAnimatedAssimpModel::Ptr loadTestModel()
{
  const auto fil = skinnedModelPath();
  auto m = CAnimatedAssimpModel::Create();
  m->loadScene(fil, CAssimpModel::LoadFlags::RealTimeQuality);
  return m;
}
}  // namespace

TEST(CAnimatedAssimpModel, EmptyModelDefaults)
{
  auto m = CAnimatedAssimpModel::Create();
  EXPECT_EQ(m->getBoneCount(), 0U);
  EXPECT_EQ(m->getAnimationCount(), 0U);
  EXPECT_EQ(m->getBoneIndex("anything"), -1);
  EXPECT_DOUBLE_EQ(m->getAnimationDuration(0), 0.0);
  EXPECT_TRUE(m->getAnimationName(0).empty());
  EXPECT_DOUBLE_EQ(m->getAnimationProgress(), 0.0);

  // All of these must be safe no-ops on a model with no skeleton:
  EXPECT_NO_THROW(m->setAnimationTime(1.0));
  EXPECT_NO_THROW(m->setActiveAnimation(0));
  EXPECT_NO_THROW(m->setActiveAnimation("none"));
  EXPECT_NO_THROW(m->setBoneLocalTransform(0, mrpt::math::CMatrixDouble44::Identity()));
  EXPECT_NO_THROW(m->clearBoneOverrides());
}

TEST(CAnimatedAssimpModel, SkeletonAndAnimationExtraction)
{
  ASSERT_FILE_EXISTS_(skinnedModelPath());
  auto m = loadTestModel();

  EXPECT_EQ(m->getBoneCount(), 2U);
  EXPECT_GE(m->getBoneIndex("joint0"), 0);
  EXPECT_GE(m->getBoneIndex("joint1"), 0);
  EXPECT_NE(m->getBoneIndex("joint0"), m->getBoneIndex("joint1"));
  EXPECT_EQ(m->getBoneIndex("not_a_bone"), -1);

  ASSERT_EQ(m->getAnimationCount(), 1U);
  EXPECT_EQ(m->getAnimationName(0), "wave");
  // 1000 ticks at 1000 ticks/s == 1 second:
  EXPECT_NEAR(m->getAnimationDuration(0), 1.0, 1e-6);

  // Out-of-range queries return neutral values, not exceptions:
  EXPECT_DOUBLE_EQ(m->getAnimationDuration(99), 0.0);
  EXPECT_TRUE(m->getAnimationName(99).empty());

  // The geometry itself was loaded through the CAssimpModel base:
  EXPECT_EQ(m->getTotalTriangleCount(), 2U);
}

TEST(CAnimatedAssimpModel, SelectActiveAnimation)
{
  auto m = loadTestModel();

  m->setActiveAnimation(0U);
  EXPECT_DOUBLE_EQ(m->getAnimationProgress(), 0.0);

  m->setAnimationTime(0.5);
  EXPECT_NEAR(m->getAnimationProgress(), 0.5, 1e-6);

  // Selecting by name resets the playback time:
  m->setActiveAnimation("wave");
  EXPECT_DOUBLE_EQ(m->getAnimationProgress(), 0.0);

  // Unknown name and out-of-range index leave the selection untouched:
  m->setAnimationTime(0.25);
  m->setActiveAnimation("does_not_exist");
  m->setActiveAnimation(99U);
  EXPECT_NEAR(m->getAnimationProgress(), 0.25, 1e-6);
}

TEST(CAnimatedAssimpModel, ProgressLoopingVsClamped)
{
  auto m = loadTestModel();
  m->setActiveAnimation(0U);

  // Looping is the default: 1.25 s of a 1 s animation wraps to 0.25:
  m->setAnimationTime(1.25);
  EXPECT_NEAR(m->getAnimationProgress(), 0.25, 1e-6);

  m->setLooping(false);
  m->setAnimationTime(1.25);
  EXPECT_NEAR(m->getAnimationProgress(), 1.0, 1e-6);
}

TEST(CAnimatedAssimpModel, SkinningDeformsGeometry)
{
  auto m = loadTestModel();
  m->setActiveAnimation(0U);

  m->setAnimationTime(0.0);
  const auto bbRest = m->getBoundingBoxLocal();

  // Half-way through, "joint1" is rotated 45 deg about +Z, which must drag
  // the two vertices bound to it away from their rest position.
  // (t=1.0 is not usable here: with looping on it wraps back to t=0.)
  m->setAnimationTime(0.5);
  const auto bbPosed = m->getBoundingBoxLocal();

  EXPECT_GT((bbPosed.max - bbRest.max).norm() + (bbPosed.min - bbRest.min).norm(), 1e-3);

  // Going back to the start must restore the original pose exactly (the
  // bind pose is re-applied after every skinning pass):
  m->setAnimationTime(0.0);
  const auto bbBack = m->getBoundingBoxLocal();
  EXPECT_NEAR(bbBack.min.x, bbRest.min.x, 1e-4);
  EXPECT_NEAR(bbBack.max.y, bbRest.max.y, 1e-4);
}

TEST(CAnimatedAssimpModel, BoneOverrides)
{
  auto m = loadTestModel();
  m->setActiveAnimation(0U);
  m->setAnimationTime(0.0);
  const auto bbRest = m->getBoundingBoxLocal();

  const int j1 = m->getBoneIndex("joint1");
  ASSERT_GE(j1, 0);

  // Translate the tip bone far along +X:
  auto T = mrpt::math::CMatrixDouble44::Identity();
  T(0, 3) = 5.0;
  m->setBoneLocalTransform(static_cast<size_t>(j1), T);
  m->setAnimationTime(0.0);
  const auto bbOverridden = m->getBoundingBoxLocal();
  EXPECT_GT(bbOverridden.max.x, bbRest.max.x + 1.0);

  // Out-of-range bone index is a no-op:
  EXPECT_NO_THROW(m->setBoneLocalTransform(9999, T));

  m->clearBoneOverrides();
  m->setAnimationTime(0.0);
  const auto bbRestored = m->getBoundingBoxLocal();
  EXPECT_NEAR(bbRestored.max.x, bbRest.max.x, 1e-4);
}

TEST(CAnimatedAssimpModel, InterpolationBetweenKeyframes)
{
  auto m = loadTestModel();
  m->setActiveAnimation(0U);

  // The animation has keys at t=0, 0.5 and 1.0 s; sampling in between must
  // give a pose strictly between the two neighboring keyframes.
  m->setAnimationTime(0.5);
  const auto bbMid = m->getBoundingBoxLocal();
  m->setAnimationTime(1.0);
  const auto bbEnd = m->getBoundingBoxLocal();
  m->setAnimationTime(0.25);
  const auto bbQuarter = m->getBoundingBoxLocal();

  EXPECT_GT((bbMid.max - bbQuarter.max).norm(), 1e-4);
  EXPECT_GT((bbEnd.max - bbMid.max).norm(), 1e-4);

  // Sampling past the end (with looping off) must not throw nor produce NaNs:
  m->setLooping(false);
  m->setAnimationTime(10.0);
  const auto bb = m->getBoundingBoxLocal();
  EXPECT_TRUE(std::isfinite(bb.max.x));
  EXPECT_TRUE(std::isfinite(bb.min.z));
}

TEST(CAnimatedAssimpModel, SerializationRoundTrip)
{
  auto m = loadTestModel();
  m->setActiveAnimation(0U);
  m->setAnimationTime(0.5);
  m->setLooping(false);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << *m;
  buf.Seek(0);

  auto m2 = CAnimatedAssimpModel::Create();
  arch >> *m2;

  EXPECT_EQ(m2->getModelPath(), m->getModelPath());
  EXPECT_EQ(m2->getTotalTriangleCount(), m->getTotalTriangleCount());
  // The assimp scene is not part of the stream, so the skeleton cannot be
  // re-extracted; the plain animation state must survive nonetheless:
  EXPECT_NEAR(m2->getAnimationProgress(), 0.0, 1e-9);
}

#endif  // MRPT_HAS_ASSIMP
