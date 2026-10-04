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

/** Offscreen rendering tests of edits to the structure of an already compiled
 *  scene graph (objects removed, inserted again, or shared by several groups),
 *  of shadow casting flags, and of which changes regenerate GPU buffers.
 */

#include <gtest/gtest.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CompiledScene.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>

#include <memory>

#include "render_pixel_utils.h"

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
#endif

#if defined(RUN_OFFSCREEN_RENDER_TESTS)

using namespace mrpt::opengl::testing;

namespace
{
constexpr int W = 200;
constexpr int H = 150;

const RGB RED{255, 0, 0};

std::unique_ptr<mrpt::opengl::CFBORender> makeRenderer()
{
  try
  {
    return std::make_unique<mrpt::opengl::CFBORender>(W, H);
  }
  catch (const std::exception&)
  {
    return nullptr;
  }
}

mrpt::img::CImage render(mrpt::opengl::CFBORender& r, const mrpt::viz::Scene& scene)
{
  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);
  r.render_RGB(scene, frame);
  return frame;
}

/** Looking down at the XY plane, over a white background. */
mrpt::viz::Scene::Ptr topViewScene(float distance = 14.0f)
{
  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  vp->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  vp->getCamera().setPointingAt(0, 0, 0);
  vp->getCamera().setZoomDistance(distance);
  vp->getCamera().setAzimuthDegrees(-90);
  vp->getCamera().setElevationDegrees(90);
  return scene;
}

mrpt::viz::CBox::Ptr redBox()
{
  auto box =
      mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-1, -1, 0), mrpt::math::TPoint3D(1, 1, 1));
  box->setColor_u8(0xff, 0x00, 0x00);
  box->enableLight(false);
  box->enableBoxBorder(false);
  return box;
}

mrpt::viz::CSetOfObjects::Ptr groupAt(double x, double y)
{
  auto g = mrpt::viz::CSetOfObjects::Create();
  g->setLocation(x, y, 0);
  return g;
}

int countRed(const mrpt::img::CImage& im) { return countColor(im, 0, 0, W, H, RED, 60); }

/** Mean column and row of the red pixels */
std::pair<double, double> redCentroid(const mrpt::img::CImage& im)
{
  double sx = 0;
  double sy = 0;
  int n = 0;
  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      if (isNear(pixelRGB(im, x, y), RED, 60))
      {
        sx += x;
        sy += y;
        n++;
      }
    }
  }
  return n ? std::make_pair(sx / n, sy / n) : std::make_pair(-1.0, -1.0);
}

/** A box that counts how many times its buffers are regenerated */
class CountingBox : public mrpt::viz::CBox
{
 public:
  CountingBox() : CBox(mrpt::math::TPoint3D(-1, -1, 0), mrpt::math::TPoint3D(1, 1, 1)) {}
  void updateBuffers() const override
  {
    calls++;
    CBox::updateBuffers();
  }
  mutable int calls = 0;
};
}  // namespace

TEST(OpenGLSceneGraphEdits, ObjectRemovedAndInsertedAgainIsDrawn)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  auto scene = topViewScene();
  auto group = groupAt(0, 0);
  auto box = redBox();
  group->insert(box);
  scene->insert(group);

  ASSERT_GT(countRed(render(*r, *scene)), 100);

  group->removeObject(box);
  EXPECT_EQ(countRed(render(*r, *scene)), 0);

  group->insert(box);
  EXPECT_GT(countRed(render(*r, *scene)), 100) << "Object inserted again must be drawn";
}

TEST(OpenGLSceneGraphEdits, RemovingOneOccurrenceOfASharedObjectKeepsTheOther)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  auto box = redBox();

  // Expected result: the box only in the second group:
  auto expectedScene = topViewScene();
  {
    auto b = groupAt(3, 0);
    b->insert(box);
    expectedScene->insert(b);
  }
  const auto expected = redCentroid(render(*r, *expectedScene));
  ASSERT_GE(expected.first, 0);

  // The same box shared by two groups, then removed from the first one:
  auto scene = topViewScene();
  auto a = groupAt(-3, 0);
  auto b = groupAt(3, 0);
  a->insert(box);
  b->insert(box);
  scene->insert(a);
  scene->insert(b);
  render(*r, *scene);

  a->removeObject(box);
  const auto got = redCentroid(render(*r, *scene));
  EXPECT_NEAR(got.first, expected.first, 1.0);
  EXPECT_NEAR(got.second, expected.second, 1.0);
}

TEST(OpenGLSceneGraphEdits, NewOccurrenceOfAnAlreadyDrawnObjectIsDrawn)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  auto scene = topViewScene();
  auto box = redBox();
  auto a = groupAt(-3, 0);
  auto b = groupAt(3, 0);
  a->insert(box);
  scene->insert(a);
  scene->insert(b);
  const int single = countRed(render(*r, *scene));
  ASSERT_GT(single, 100);

  b->insert(box);  // the same object, at a second position
  EXPECT_GT(countRed(render(*r, *scene)), 1.8 * single);
}

TEST(OpenGLSceneGraphEdits, ObjectsAndGroupsThatDoNotCastShadows)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }

  // A floating box over a white floor, lit from one side so its shadow is
  // visible next to it:
  enum class Mode
  {
    NoShadows,
    Casts,
    BoxDoesNotCast,
    GroupDoesNotCast
  };
  auto renderWith = [&](Mode mode)
  {
    auto scene = topViewScene(12);
    auto vp = scene->getViewport();
    vp->enableShadowCasting(mode != Mode::NoShadows, 1024, 1024);
    vp->lightParameters().lights = {mrpt::viz::TLight::Directional({0.5774f, 0, -0.8165f})};
    auto floor =
        mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-6, -6, -0.1), mrpt::math::TPoint3D(6, 6, 0));
    floor->setColor_u8(0xff, 0xff, 0xff);
    floor->enableBoxBorder(false);
    auto box = mrpt::viz::CBox::Create(
        mrpt::math::TPoint3D(-0.5, -0.5, 1.5), mrpt::math::TPoint3D(0.5, 0.5, 2.5));
    box->enableBoxBorder(false);
    auto group = mrpt::viz::CSetOfObjects::Create();
    group->insert(box);
    if (mode == Mode::BoxDoesNotCast)
    {
      box->castShadows(false);
    }
    if (mode == Mode::GroupDoesNotCast)
    {
      group->castShadows(false);
    }
    scene->insert(floor);
    scene->insert(group);
    return render(*r, *scene);
  };

  auto differentPixels = [](const mrpt::img::CImage& a, const mrpt::img::CImage& b)
  {
    int n = 0;
    for (int y = 0; y < H; y++)
    {
      for (int x = 0; x < W; x++)
      {
        n += isNear(pixelRGB(a, x, y), pixelRGB(b, x, y), 10) ? 0 : 1;
      }
    }
    return n;
  };

  const auto noShadows = renderWith(Mode::NoShadows);
  ASSERT_GT(differentPixels(renderWith(Mode::Casts), noShadows), 100)
      << "The box should cast a visible shadow";
  EXPECT_EQ(differentPixels(renderWith(Mode::BoxDoesNotCast), noShadows), 0);
  EXPECT_EQ(differentPixels(renderWith(Mode::GroupDoesNotCast), noShadows), 0);
}

TEST(OpenGLSceneGraphEdits, MovingAnObjectDoesNotRegenerateItsBuffers)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  auto scene = topViewScene();
  auto box = std::make_shared<CountingBox>();
  box->setColor_u8(0xff, 0x00, 0x00);
  box->enableLight(false);
  box->enableBoxBorder(false);
  scene->insert(box);

  const auto c0 = redCentroid(render(*r, *scene));
  const int calls = box->calls;
  ASSERT_GE(calls, 1);

  box->setLocation(3, 0, 0);
  const auto c1 = redCentroid(render(*r, *scene));
  EXPECT_GT(std::abs(c1.first - c0.first) + std::abs(c1.second - c0.second), 10.0)
      << "The box should have moved";
  box->setVisibility(false);
  EXPECT_EQ(countRed(render(*r, *scene)), 0);
  box->setVisibility(true);
  render(*r, *scene);
  EXPECT_EQ(box->calls, calls) << "Moving or hiding must not regenerate the buffers";

  box->setColor_u8(0x00, 0xff, 0x00);
  render(*r, *scene);
  EXPECT_EQ(box->calls, calls + 1);
}

TEST(OpenGLSceneGraphEdits, ViewportsRemovedFromTheSceneAreDropped)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  // The renderer is only needed for its OpenGL context:
  mrpt::viz::Scene scene;
  scene.insert(redBox());
  auto small = scene.createViewport("small");
  small->setViewportPosition(0, 0, 0.3, 0.3);
  small->insert(redBox());

  mrpt::opengl::CompiledScene compiled;
  compiled.compile(scene);
  compiled.render(W, H);
  EXPECT_EQ(compiled.getViewportCount(), 2U);

  scene.clear();  // destroys both viewports, then creates a new "main" one
  compiled.updateIfNeeded();
  EXPECT_EQ(compiled.getViewportCount(), 1U);
  EXPECT_EQ(compiled.getProxyCount(), 0U);
  compiled.render(W, H);  // must not access the destroyed viewports
}

TEST(OpenGLSceneGraphEdits, UnchangedScenesAreNotTraversed)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  auto scene = topViewScene();
  auto box = redBox();
  scene->insert(box);

  mrpt::opengl::CompiledScene compiled;
  compiled.compile(*scene);

  mrpt::opengl::CompilationStats stats;
  EXPECT_FALSE(compiled.updateIfNeeded(&stats));
  EXPECT_EQ(stats.numObjectsTotal, 0U);

  box->setLocation(1, 0, 0);
  EXPECT_TRUE(compiled.hasPendingUpdates());
  EXPECT_TRUE(compiled.updateIfNeeded(&stats));
  EXPECT_GE(stats.numObjectsUpdated, 1U);
  EXPECT_FALSE(compiled.hasPendingUpdates());
}

#endif  // RUN_OFFSCREEN_RENDER_TESTS
