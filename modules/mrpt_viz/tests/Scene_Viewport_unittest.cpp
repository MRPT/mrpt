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
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/Scene.h>

using namespace mrpt::viz;

TEST(Scene, DefaultHasMainViewport)
{
  Scene scene;
  EXPECT_EQ(scene.viewportsCount(), 1u);
  auto main = scene.getViewport("main");
  ASSERT_TRUE(main);
  EXPECT_EQ(main->getName(), "main");
}

TEST(Scene, CreateViewportIsIdempotent)
{
  Scene scene;
  auto vp1 = scene.createViewport("second");
  ASSERT_TRUE(vp1);
  EXPECT_EQ(scene.viewportsCount(), 2u);

  auto vp2 = scene.createViewport("second");
  EXPECT_EQ(vp1, vp2);
  EXPECT_EQ(scene.viewportsCount(), 2u);
}

TEST(Scene, GetViewportUnknownReturnsNull)
{
  Scene scene;
  EXPECT_FALSE(scene.getViewport("does_not_exist"));
}

TEST(Scene, InsertAndGetByName)
{
  Scene scene;
  auto box = CBox::Create();
  box->setName("mybox");
  scene.insert(box);

  auto found = scene.getByName("mybox");
  EXPECT_EQ(found, box);
  EXPECT_FALSE(scene.getByName("no_such_object"));
}

TEST(Scene, InsertIntoNamedViewport)
{
  Scene scene;
  scene.createViewport("second");
  auto sphere = CSphere::Create();
  sphere->setName("s1");
  scene.insert(sphere, "second");

  EXPECT_FALSE(scene.getViewport("main")->getByName("s1"));
  EXPECT_TRUE(scene.getViewport("second")->getByName("s1"));
}

TEST(Scene, InsertIntoUnknownViewportThrows)
{
  Scene scene;
  auto box = CBox::Create();
  EXPECT_THROW(scene.insert(box, "no_such_viewport"), std::exception);
}

TEST(Scene, InsertCollection)
{
  Scene scene;
  std::vector<CVisualObject::Ptr> objs{CBox::Create(), CSphere::Create()};
  scene.insertCollection(objs);
  EXPECT_EQ(scene.getViewport("main")->size(), 2u);
}

TEST(Scene, GetByClass)
{
  Scene scene;
  auto sphere = CSphere::Create();
  scene.insert(sphere);
  auto found = scene.getByClass<CSphere>();
  EXPECT_EQ(found, sphere);

  EXPECT_FALSE(scene.getByClass<CBox>());
}

TEST(Scene, RemoveObject)
{
  Scene scene;
  auto box = CBox::Create();
  box->setName("removable");
  scene.insert(box);
  ASSERT_TRUE(scene.getByName("removable"));

  scene.removeObject(box);
  EXPECT_FALSE(scene.getByName("removable"));
}

TEST(Scene, Clear)
{
  Scene scene;
  scene.createViewport("second");
  scene.insert(CBox::Create());
  ASSERT_GT(scene.viewportsCount(), 1u);

  scene.clear(true);
  EXPECT_EQ(scene.viewportsCount(), 1u);
  EXPECT_TRUE(scene.getViewport("main")->empty());
}

TEST(Scene, FollowCameraFlag)
{
  Scene scene;
  EXPECT_FALSE(scene.followCamera());
  scene.enableFollowCamera(true);
  EXPECT_TRUE(scene.followCamera());
}

TEST(Scene, DumpListAndYAML)
{
  Scene scene;
  scene.insert(CBox::Create());
  std::vector<std::string> lst;
  scene.dumpListOfObjects(lst);
  EXPECT_FALSE(lst.empty());

  const auto yaml = scene.asYAML();
  EXPECT_FALSE(yaml.empty());
}

TEST(Scene, TraceRayAndBoundingBox)
{
  Scene scene;
  auto sphere = CSphere::Create();
  sphere->setRadius(1.0f);
  scene.insert(sphere);

  double dist = 0;
  const bool hit = scene.traceRay(mrpt::poses::CPose3D(-10, 0, 0, 0, 0, 0), dist);
  (void)hit;

  const auto bb = scene.getBoundingBox();
  EXPECT_LE(bb.min.x, bb.max.x);
}

TEST(Scene, TraceRayOnUnimplementedObjectThrows)
{
  // CBox::traceRay() is a currently-unimplemented stub (throws).
  Scene scene;
  scene.insert(CBox::Create(mrpt::math::TPoint3D(-1, -1, -1), mrpt::math::TPoint3D(1, 1, 1)));
  double dist = 0;
  EXPECT_THROW(scene.traceRay(mrpt::poses::CPose3D(-10, 0, 0, 0, 0, 0), dist), std::exception);
}

TEST(Scene, VisitAllObjectsRecursesIntoSetOfObjects)
{
  Scene scene;
  auto group = CSetOfObjects::Create();
  group->insert(CBox::Create());
  group->insert(CSphere::Create());
  scene.insert(group);
  scene.insert(CBox::Create());

  size_t count = 0;
  scene.visitAllObjects([&count](const CVisualObject::Ptr&) { count++; });
  // 1 top-level group + 2 children inside + 1 top-level box = 4
  EXPECT_EQ(count, 4u);
}

TEST(Scene, CopyConstructorAndAssignment)
{
  Scene scene;
  scene.insert(CBox::Create());

  Scene copy(scene);
  EXPECT_EQ(copy.viewportsCount(), scene.viewportsCount());
  EXPECT_EQ(copy.getViewport("main")->size(), scene.getViewport("main")->size());

  Scene assigned;
  assigned = scene;
  EXPECT_EQ(assigned.getViewport("main")->size(), scene.getViewport("main")->size());
}

TEST(Scene, SaveAndLoadFile)
{
  Scene scene;
  auto box = CBox::Create();
  box->setName("roundtrip_box");
  scene.insert(box);

  const std::string file = mrpt::system::getTempFileName() + "_scene.3Dscene";
  ASSERT_TRUE(scene.saveToFile(file));

  Scene loaded;
  ASSERT_TRUE(loaded.loadFromFile(file));
  EXPECT_TRUE(loaded.getByName("roundtrip_box"));

  mrpt::system::deleteFile(file);
}

TEST(Scene, LoadNonExistentFileFails)
{
  EXPECT_FALSE(Scene().loadFromFile("/no/such/file.3Dscene"));
}

TEST(Scene, StreamInsertOperator)
{
  auto scene = Scene::Create();
  auto box = CBox::Create();
  scene << box;
  EXPECT_TRUE(scene->getViewport("main")->getByClass<CBox>());
}

// ------------------------ Viewport ------------------------

TEST(Viewport, PositionAndClipDistances)
{
  Scene scene;
  auto vp = scene.getViewport("main");

  vp->setViewportPosition(0.1, 0.2, 0.5, 0.6);
  double x = 0;
  double y = 0;
  double w = 0;
  double h = 0;
  vp->getViewportPosition(x, y, w, h);
  EXPECT_DOUBLE_EQ(x, 0.1);
  EXPECT_DOUBLE_EQ(y, 0.2);
  EXPECT_DOUBLE_EQ(w, 0.5);
  EXPECT_DOUBLE_EQ(h, 0.6);

  vp->setViewportClipDistances(0.5f, 100.f);
  float cmin = 0;
  float cmax = 0;
  vp->getViewportClipDistances(cmin, cmax);
  EXPECT_FLOAT_EQ(cmin, 0.5f);
  EXPECT_FLOAT_EQ(cmax, 100.f);

  vp->setLightShadowClipDistances(1.0f, 50.f);
  vp->getLightShadowClipDistances(cmin, cmax);
  EXPECT_FLOAT_EQ(cmin, 1.0f);
  EXPECT_FLOAT_EQ(cmax, 50.f);
}

TEST(Viewport, BorderAndBackground)
{
  Scene scene;
  auto vp = scene.getViewport("main");

  vp->setBorderSize(3);
  EXPECT_EQ(vp->getBorderSize(), 3u);

  vp->setBorderColor(mrpt::img::TColor(10, 20, 30));
  EXPECT_EQ(vp->getBorderColor(), mrpt::img::TColor(10, 20, 30));

  EXPECT_FALSE(vp->isTransparent());
  vp->setTransparent(true);
  EXPECT_TRUE(vp->isTransparent());

  vp->setCustomBackgroundColor({0.1f, 0.2f, 0.3f, 1.0f});
  const auto bg = vp->getCustomBackgroundColor();
  EXPECT_FLOAT_EQ(bg.R, 0.1f);
}

TEST(Viewport, VisibilityAndPolygonNicest)
{
  Scene scene;
  auto vp = scene.getViewport("main");

  EXPECT_TRUE(vp->getViewportVisibility());
  vp->setViewportVisibility(false);
  EXPECT_FALSE(vp->getViewportVisibility());

  EXPECT_TRUE(vp->isPolygonNicestEnabled());
  vp->enablePolygonNicest(false);
  EXPECT_FALSE(vp->isPolygonNicestEnabled());
}

TEST(Viewport, ShadowsAndSSAO)
{
  Scene scene;
  auto vp = scene.getViewport("main");

  EXPECT_FALSE(vp->isShadowCastingEnabled());
  vp->enableShadowCasting(true, 512, 512);
  EXPECT_TRUE(vp->isShadowCastingEnabled());
  vp->enableShadowCasting(false);
  EXPECT_FALSE(vp->isShadowCastingEnabled());

  EXPECT_FALSE(vp->isSSAOEnabled());
  vp->enableSSAO(true);
  EXPECT_TRUE(vp->isSSAOEnabled());
}

TEST(Viewport, CloneView)
{
  Scene scene;
  scene.createViewport("cloned");
  auto vp = scene.getViewport("cloned");

  EXPECT_FALSE(vp->isCloned());
  vp->setCloneView("main");
  EXPECT_TRUE(vp->isCloned());
  EXPECT_EQ(vp->getClonedViewportName(), "main");

  vp->setCloneCamera(true);
  EXPECT_TRUE(vp->isClonedCamera());

  vp->setClonedCameraFrom("main");
  EXPECT_TRUE(vp->isClonedCamera());
  EXPECT_EQ(vp->isClonedCameraFrom(), "main");

  vp->resetCloneView();
  EXPECT_FALSE(vp->isCloned());
}

TEST(Viewport, ImageViewMode)
{
  Scene scene;
  auto vp = scene.getViewport("main");
  EXPECT_FALSE(vp->isImageViewMode());

  mrpt::img::CImage img(4, 4, mrpt::img::CH_RGB);
  vp->setImageView(img);
  EXPECT_TRUE(vp->isImageViewMode());
  EXPECT_TRUE(vp->getImageViewPlane());
}

TEST(Viewport, InsertClearAndIteration)
{
  Scene scene;
  auto vp = scene.getViewport("main");
  vp->insert(CBox::Create());
  vp->insert(CSphere::Create());
  EXPECT_EQ(vp->size(), 2u);
  EXPECT_FALSE(vp->empty());

  size_t n = 0;
  for (auto it = vp->begin(); it != vp->end(); ++it)
  {
    n++;
  }
  EXPECT_EQ(n, 2u);

  vp->clear();
  EXPECT_TRUE(vp->empty());
}

TEST(Viewport, CameraAccessAndBoundingBox)
{
  Scene scene;
  auto vp = scene.getViewport("main");
  vp->insert(CBox::Create(mrpt::math::TPoint3D(-2, -2, -2), mrpt::math::TPoint3D(2, 2, 2)));

  CCamera& cam = vp->getCamera();
  cam.setAzimuthDegrees(10.f);
  EXPECT_FLOAT_EQ(vp->getCamera().getAzimuthDegrees(), 10.f);

  const auto bb = vp->getBoundingBox();
  EXPECT_LE(bb.min.x, bb.max.x);
}

TEST(Viewport, LightParameters)
{
  Scene scene;
  auto vp = scene.getViewport("main");
  auto& light = vp->lightParameters();
  light.ssao_enabled = true;
  EXPECT_TRUE(vp->lightParameters().ssao_enabled);
}
