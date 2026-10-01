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
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/Viewport.h>

using namespace mrpt::viz;

namespace
{
Scene roundTrip(const Scene& s)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << s;
  buf.Seek(0);
  Scene out;
  arch >> out;
  return out;
}
}  // namespace

TEST(Viewport, TextMessagesSurviveSerialization)
{
  Scene scene;
  auto vp = scene.getViewport("main");
  TFontParams fp;
  fp.color = mrpt::img::TColorf(0.1f, 0.2f, 0.3f);
  fp.draw_shadow = true;
  vp->addTextMessage(0.1, 0.2, "hello", 7, fp);
  vp->addTextMessage(10, 20, "world", 9);

  Scene loaded = roundTrip(scene);
  auto vp2 = loaded.getViewport("main");
  ASSERT_TRUE(vp2);
  // updateTextMessage() only succeeds for messages that exist:
  EXPECT_TRUE(vp2->updateTextMessage(7, "changed"));
  EXPECT_TRUE(vp2->updateTextMessage(9, "changed too"));
  EXPECT_FALSE(vp2->updateTextMessage(8, "does not exist"));
}

TEST(Viewport, ImageViewSurvivesSerializationAndRvalueSetter)
{
  Scene scene;
  auto vp = scene.getViewport("main");

  mrpt::img::CImage img(16, 8, mrpt::img::CH_RGB);
  img.filledRectangle({0, 0}, {15, 7}, mrpt::img::TColor(10, 20, 30));
  vp->setImageView(std::move(img), false);
  ASSERT_TRUE(vp->isImageViewMode());

  Scene loaded = roundTrip(scene);
  auto vp2 = loaded.getViewport("main");
  ASSERT_TRUE(vp2);
  EXPECT_TRUE(vp2->isImageViewMode());
  ASSERT_TRUE(vp2->getImageViewPlane());

  vp2->setNormalMode();
  EXPECT_FALSE(vp2->isImageViewMode());
}

TEST(Viewport, NestedSetOfObjectsLookupAndRemoval)
{
  Scene scene;
  auto vp = scene.getViewport("main");

  auto group = CSetOfObjects::Create();
  group->setName("group");
  auto inner = CBox::Create();
  inner->setName("inner_box");
  group->insert(inner);
  auto top = CSphere::Create(1.0f);
  top->setName("top_sphere");
  vp->insert(group);
  vp->insert(top);

  // getByName() looks into groups:
  ASSERT_TRUE(vp->getByName("inner_box"));
  EXPECT_EQ(vp->getByName("inner_box"), inner);
  EXPECT_FALSE(vp->getByName("nope"));
  const Viewport& cvp = *vp;
  EXPECT_TRUE(cvp.getByName("inner_box"));

  // The dumps show nested objects too:
  std::vector<std::string> lst;
  scene.dumpListOfObjects(lst);
  bool foundInner = false;
  for (const auto& l : lst)
  {
    foundInner = foundInner || l.find("CBox") != std::string::npos;
  }
  EXPECT_TRUE(foundInner);
  const auto yaml = scene.asYAML();
  EXPECT_FALSE(yaml.empty());

  // removeObject() searches into groups:
  vp->removeObject(inner);
  EXPECT_FALSE(vp->getByName("inner_box"));
  EXPECT_TRUE(vp->getByName("top_sphere"));
}

TEST(Viewport, ClonedCameraTakesTheCameraOfTheOtherViewport)
{
  Scene scene;
  auto main = scene.getViewport("main");
  auto other = scene.createViewport("other");
  main->getCamera().setAzimuthDegrees(33.0f);
  other->getCamera().setAzimuthDegrees(77.0f);

  // Normal case: each one has its own camera
  EXPECT_NEAR(other->resolveActiveCamera().getAzimuthDegrees(), 77.0f, 1e-4f);

  // Cloning the camera needs the viewport to be in clone mode first:
  EXPECT_ANY_THROW(other->setCloneCamera(true));

  other->setCloneView("main");
  other->setCloneCamera(true);
  EXPECT_TRUE(other->isCloned());
  EXPECT_EQ(other->getClonedViewportName(), "main");
  EXPECT_TRUE(other->isClonedCamera());
  EXPECT_EQ(other->isClonedCameraFrom(), "main");
  EXPECT_NEAR(other->resolveActiveCamera().getAzimuthDegrees(), 33.0f, 1e-4f);

  // Disabling it goes back to the viewport's own camera:
  other->setCloneCamera(false);
  EXPECT_FALSE(other->isClonedCamera());
  EXPECT_TRUE(other->isClonedCameraFrom().empty());
  EXPECT_NEAR(other->resolveActiveCamera().getAzimuthDegrees(), 77.0f, 1e-4f);

  // The camera can also be borrowed by a viewport that is not a clone:
  other->resetCloneView();
  other->setClonedCameraFrom("main");
  EXPECT_NEAR(other->resolveActiveCamera().getAzimuthDegrees(), 33.0f, 1e-4f);

  // Pointing to a viewport that does not exist is an error:
  other->setClonedCameraFrom("missing");
  EXPECT_ANY_THROW(static_cast<void>(other->resolveActiveCamera()));
}

TEST(Viewport, RayForPixelCoordWithOrthogonalCamera)
{
  Scene scene;
  auto vp = scene.getViewport("main");
  auto& cam = vp->getCamera();
  cam.setPointingAt(0.f, 0.f, 0.f);
  cam.setZoomDistance(10.0f);
  cam.setOrthogonal();

  const mrpt::img::TPixelCoord vpSize(200, 100);
  const auto center = vp->get3DRayForPixelCoord({100, 50}, vpSize);
  const auto left = vp->get3DRayForPixelCoord({0, 50}, vpSize);
  const auto top = vp->get3DRayForPixelCoord({100, 0}, vpSize);

  // Rays of an orthogonal camera are parallel...
  const mrpt::math::TVector3D dc(center.director);
  const mrpt::math::TVector3D dl(left.director);
  const mrpt::math::TVector3D dt(top.director);
  EXPECT_NEAR((dc - dl).norm(), 0.0, 1e-9);
  EXPECT_NEAR((dc - dt).norm(), 0.0, 1e-9);
  // ...and the central one goes through the point being looked at:
  EXPECT_LT(center.distance({0, 0, 0}), 1e-6);
  EXPECT_GT(left.distance({0, 0, 0}), 1.0);
  EXPECT_GT(top.distance({0, 0, 0}), 1.0);
}

TEST(Viewport, RayForPixelCoordOrthogonalMatchesRenderedExtents)
{
  // The visible area of an orthogonal camera must be the one used by the
  // renderer: zoom/2 along the shortest image side, scaled by the aspect
  // ratio along the longest one.
  Scene scene;
  auto vp = scene.getViewport("main");
  auto& cam = vp->getCamera();
  cam.setPointingAt(0.f, 0.f, 0.f);
  cam.setZoomDistance(10.0f);
  cam.setOrthogonal();

  // Landscape: 5 m wide, 2.5 m tall.
  {
    const mrpt::img::TPixelCoord vpSize(200, 100);
    const auto left = vp->get3DRayForPixelCoord({0, 50}, vpSize);
    const auto top = vp->get3DRayForPixelCoord({100, 0}, vpSize);
    EXPECT_NEAR(left.distance({0, 0, 0}), 5.0, 1e-4);
    EXPECT_NEAR(top.distance({0, 0, 0}), 2.5, 1e-4);
  }
  // Portrait: 2.5 m wide, 5 m tall.
  {
    const mrpt::img::TPixelCoord vpSize(100, 200);
    const auto left = vp->get3DRayForPixelCoord({0, 100}, vpSize);
    const auto top = vp->get3DRayForPixelCoord({50, 0}, vpSize);
    EXPECT_NEAR(left.distance({0, 0, 0}), 2.5, 1e-4);
    EXPECT_NEAR(top.distance({0, 0, 0}), 5.0, 1e-4);
  }
}

TEST(Viewport, StreamInsertOperators)
{
  Scene scene;
  auto vp = scene.getViewport("main");
  vp << CBox::Create();
  EXPECT_EQ(vp->size(), 1u);

  std::vector<CVisualObject::Ptr> several = {CBox::Create(), CSphere::Create(1.0f)};
  vp << several;
  EXPECT_EQ(vp->size(), 3u);
}
