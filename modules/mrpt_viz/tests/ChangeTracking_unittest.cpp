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

/** Change tracking of visual objects, as used by renderers: which changes
 * require regenerating buffers (data version) and which only move or hide an
 * object (transform version), and the scene-wide change counters.
 */

#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/TLightParameters.h>

#include <thread>

using namespace mrpt::viz;

namespace
{
/** A box that counts how many times its buffers are regenerated */
class CountingBox : public CBox
{
 public:
  void updateBuffers() const override
  {
    calls++;
    CBox::updateBuffers();
  }
  mutable int calls = 0;
};
}  // namespace

TEST(ChangeTracking, PoseScaleVisibilityAndShadowsAreTransformChanges)
{
  auto box = CBox::Create();
  const auto data0 = box->dataVersion();
  auto transform = box->transformVersion();

  box->setPose(mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0));
  EXPECT_GT(box->transformVersion(), transform);
  transform = box->transformVersion();

  box->setLocation(4, 5, 6);
  EXPECT_GT(box->transformVersion(), transform);
  transform = box->transformVersion();

  box->setScale(2.0f);
  EXPECT_GT(box->transformVersion(), transform);
  transform = box->transformVersion();

  box->setVisibility(false);
  EXPECT_GT(box->transformVersion(), transform);
  transform = box->transformVersion();

  box->castShadows(false);
  EXPECT_GT(box->transformVersion(), transform);

  EXPECT_EQ(box->dataVersion(), data0) << "No buffer needs regenerating";
}

TEST(ChangeTracking, GeometryAndAppearanceAreDataChanges)
{
  auto box = CBox::Create();
  auto data = box->dataVersion();

  box->setBoxCorners({0, 0, 0}, {1, 2, 3});
  EXPECT_GT(box->dataVersion(), data);
  data = box->dataVersion();

  box->setColor_u8(10, 20, 30);
  EXPECT_GT(box->dataVersion(), data);
  data = box->dataVersion();

  box->materialShininess(0.8f);
  EXPECT_GT(box->dataVersion(), data) << "Shininess changes must reach the renderers";
}

TEST(ChangeTracking, UpdateBuffersIfNeededRegeneratesOncePerChange)
{
  auto box = std::make_shared<CountingBox>();
  box->updateBuffersIfNeeded();
  box->updateBuffersIfNeeded();
  EXPECT_EQ(box->calls, 1);

  box->setPose(mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0));
  box->updateBuffersIfNeeded();
  EXPECT_EQ(box->calls, 1) << "Moving an object does not change its buffers";

  box->setColor_u8(1, 2, 3);
  box->updateBuffersIfNeeded();
  box->updateBuffersIfNeeded();
  EXPECT_EQ(box->calls, 2);

  // A copy has its own buffers to regenerate:
  CountingBox copy(*box);
  copy.calls = 0;
  copy.updateBuffersIfNeeded();
  EXPECT_EQ(copy.calls, 1);
}

TEST(ChangeTracking, SceneCountersFollowChangesAndStructure)
{
  auto scene = Scene::Create();
  auto group = CSetOfObjects::Create();
  auto box = CBox::Create();

  auto changes = sceneChangeCount();
  auto structure = sceneStructureChangeCount();

  scene->insert(group);
  EXPECT_GT(sceneStructureChangeCount(), structure);
  structure = sceneStructureChangeCount();

  group->insert(box);
  EXPECT_GT(sceneStructureChangeCount(), structure);
  structure = sceneStructureChangeCount();
  EXPECT_GT(sceneChangeCount(), changes);
  changes = sceneChangeCount();

  box->setLocation(1, 0, 0);
  EXPECT_GT(sceneChangeCount(), changes);
  EXPECT_EQ(sceneStructureChangeCount(), structure) << "Moving is not a structural change";
  changes = sceneChangeCount();

  box->setColor_u8(1, 2, 3);
  EXPECT_GT(sceneChangeCount(), changes);

  group->removeObject(box);
  EXPECT_GT(sceneStructureChangeCount(), structure);
  structure = sceneStructureChangeCount();

  group->clear();
  EXPECT_GT(sceneStructureChangeCount(), structure);
  structure = sceneStructureChangeCount();

  scene->getViewport()->clear();
  EXPECT_GT(sceneStructureChangeCount(), structure);
  structure = sceneStructureChangeCount();

  scene->createViewport("other");
  EXPECT_GT(sceneStructureChangeCount(), structure);
  structure = sceneStructureChangeCount();

  scene->clear();
  EXPECT_GT(sceneStructureChangeCount(), structure);
}

TEST(ChangeTracking, ActiveCameraFollowsInsertedAndRemovedCameras)
{
  auto scene = Scene::Create();
  auto vp = scene->getViewport();
  EXPECT_EQ(&vp->resolveActiveCamera(), &vp->getCamera());

  auto group = CSetOfObjects::Create();
  scene->insert(group);
  EXPECT_EQ(&vp->resolveActiveCamera(), &vp->getCamera());

  auto cam = CCamera::Create();
  group->insert(cam);
  EXPECT_EQ(&vp->resolveActiveCamera(), cam.get());

  group->removeObject(cam);
  EXPECT_EQ(&vp->resolveActiveCamera(), &vp->getCamera());
}

TEST(ChangeTracking, AxisLabelsAvailableInEveryThread)
{
  auto axis = CAxis::Create(-1, -1, -1, 1, 1, 1, 0.5f);
  axis->enableTickMarks(true);

  // A renderer in one thread regenerates the buffers...
  std::thread([&]() { axis->updateBuffersIfNeeded(); }).join();
  // ... and one in another thread must still get the labels:
  size_t numLabels = 0;
  std::thread([&]() { numLabels = axis->getInternalChildren().size(); }).join();
  EXPECT_GT(numLabels, 0U);

  // Labels follow changes:
  const auto before = axis->getInternalChildren().size();
  axis->setFrequency(0.25f);
  EXPECT_GT(axis->getInternalChildren().size(), before);
}

TEST(ChangeTracking, LightParametersShadowDistanceSerialization)
{
  TLightParameters lp;
  lp.shadow_max_distance = 12.5f;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << lp;
  buf.Seek(0);

  TLightParameters lp2;
  arch >> lp2;
  EXPECT_FLOAT_EQ(lp2.shadow_max_distance, 12.5f);

  // Streams from before this field existed (v9) read it as automatic (0):
  mrpt::io::CMemoryStream old;
  auto oldArch = mrpt::serialization::archiveFrom(old);
  oldArch.WriteAs<uint8_t>(9);
  oldArch << lp.ambient << lp.shadow_bias << lp.shadow_bias_cam2frag << lp.shadow_bias_normal
          << lp.eyeDistance2lightShadowExtension << lp.minimum_shadow_map_extension_ratio
          << lp.gamma_correction;
  oldArch.WriteAs<uint8_t>(0);  // no lights
  oldArch << lp.ambientSkyColor << lp.ambientGroundColor;
  oldArch << lp.fog_enabled << lp.fog_color << lp.fog_near << lp.fog_far << lp.fog_mode
          << lp.fog_density;
  oldArch << lp.shadow_cascades << lp.shadow_cascade_lambda;
  oldArch << lp.ssao_enabled << lp.ssao_radius << lp.ssao_bias << lp.ssao_power
          << lp.ssao_kernel_size;
  old.Seek(0);

  TLightParameters lp3;
  lp3.shadow_max_distance = 99.0f;  // must be overwritten
  oldArch >> lp3;
  EXPECT_FLOAT_EQ(lp3.shadow_max_distance, 0.0f);
}
