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

/** Offscreen rendering tests of objects inserted at several places of the
 *  scene graph (e.g. a 3D model shared by many groups): they share their GPU
 *  buffers and are drawn with instanced draw calls, which must look exactly
 *  like drawing each copy on its own.
 */

#include <gtest/gtest.h>
#include <mrpt/core/config.h>  // MRPT_IS_BIG_ENDIAN
#include <mrpt/core/get_env.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/Scene.h>

#include <functional>
#include <iterator>
#include <memory>

#include "render_pixel_utils.h"

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
#endif

#if defined(RUN_OFFSCREEN_RENDER_TESTS)

using namespace mrpt::opengl::testing;

namespace
{
constexpr int W = 320;
constexpr int H = 240;

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

mrpt::img::CImage render(mrpt::opengl::CFBORender& r, mrpt::viz::Scene& scene)
{
  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);
  r.render_RGB(scene, frame);
  return frame;
}

const mrpt::opengl::ViewportRenderStats& renderStats(const mrpt::opengl::CFBORender& r)
{
  return r.compiledScene()->getViewport("main")->lastRenderStats();
}

int differentPixels(const mrpt::img::CImage& a, const mrpt::img::CImage& b)
{
  int n = 0;
  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      n += isNear(pixelRGB(a, x, y), pixelRGB(b, x, y), 30) ? 0 : 1;
    }
  }
  return n;
}

constexpr int NUM_COPIES = 12;

/** A ground box and NUM_COPIES groups holding the object returned by
 * `object(i)`, each with its own pose and scale (one of them non-uniform, which
 * cannot be drawn instanced), lit from above with shadows. */
mrpt::viz::Scene::Ptr gridScene(const std::function<mrpt::viz::CVisualObject::Ptr(int)>& object)
{
  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  vp->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  vp->getCamera().setPointingAt(3.0, 2.0, 0.0);
  vp->getCamera().setZoomDistance(11);
  vp->getCamera().setAzimuthDegrees(-60);
  vp->getCamera().setElevationDegrees(35);
#if !MRPT_IS_BIG_ENDIAN
  vp->enableShadowCasting(true, 1024, 1024);
#endif

  auto ground =
      mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-2, -2, -0.1), mrpt::math::TPoint3D(8, 6, 0));
  ground->setColor_u8(0x90, 0x90, 0x90);
  scene->insert(ground);

  for (int i = 0; i < NUM_COPIES; i++)
  {
    auto group = mrpt::viz::CSetOfObjects::Create();
    group->insert(object(i));
    group->setPose(mrpt::poses::CPose3D(2.0 * (i % 4), 2.0 * (i / 4), 0.5, 0.4 * i, 0, 0));
    if (i == 5)
    {
      group->setScale(0.5f, 1.0f, 1.5f);
    }
    else
    {
      group->setScale(0.6f + 0.05f * static_cast<float>(i));
    }
    scene->insert(group);
  }
  return scene;
}

mrpt::viz::CVisualObject::Ptr redSphere()
{
  auto s = mrpt::viz::CSphere::Create(0.5f, 24);
  s->setColor_u8(0xff, 0x20, 0x20);
  return s;
}
}  // namespace

TEST(OpenGLInstancing, SharedObjectLooksLikeSeparateCopies)
{
  auto rShared = makeRenderer();
  auto rCopies = makeRenderer();
  if (!rShared || !rCopies)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  const auto shared = redSphere();
  auto sceneShared = gridScene([&](int) { return shared; });
  auto sceneCopies = gridScene([](int) { return redSphere(); });

  const auto imShared = render(*rShared, *sceneShared);
  const auto& statsShared = renderStats(*rShared);
  const auto imCopies = render(*rCopies, *sceneCopies);
  const auto& statsCopies = renderStats(*rCopies);

  // The shared object was compiled once (plus the ground), but it has one
  // proxy per position:
  EXPECT_EQ(rShared->compiledScene()->lastStats().numObjectsCompiled, 2U);
  EXPECT_EQ(rCopies->compiledScene()->lastStats().numObjectsCompiled, 1U + NUM_COPIES);
  EXPECT_EQ(rShared->compiledScene()->getProxyCount(), rCopies->compiledScene()->getProxyCount());

  // Copies of a shared object are drawn together; separate objects are not:
  if (!mrpt::get_env<bool>("MRPT_OPENGL_NO_INSTANCING", false))
  {
    EXPECT_GT(statsShared.numInstancedDrawCalls, 0U);
    EXPECT_LT(statsShared.numDrawCalls, statsCopies.numDrawCalls);
  }
  EXPECT_EQ(statsCopies.numInstancedDrawCalls, 0U);
  EXPECT_EQ(statsShared.numProxiesRendered, statsCopies.numProxiesRendered);

  // ...with the same result:
  EXPECT_GT(countColor(imShared, 0, 0, W, H, RGB{0xff, 0x20, 0x20}, 120), 500)
      << "the spheres were not drawn";
  EXPECT_LT(differentPixels(imShared, imCopies), W * H / 1000);
}

TEST(OpenGLInstancing, SharedObjectChangesShowUpInAllCopies)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  auto shared = mrpt::viz::CBox::Create(
      mrpt::math::TPoint3D(-0.5, -0.5, -0.5), mrpt::math::TPoint3D(0.5, 0.5, 0.5));
  shared->setColor_u8(0xff, 0x00, 0x00);
  auto scene = gridScene([&](int) { return shared; });

  const RGB RED{0xff, 0, 0};
  const RGB BLUE{0, 0, 0xff};

  const auto first = render(*renderer, *scene);
  const int redPixels = countColor(first, 0, 0, W, H, RED, 120);
  EXPECT_GT(redPixels, 500);
  EXPECT_EQ(countColor(first, 0, 0, W, H, BLUE, 120), 0);

  shared->setColor_u8(0x00, 0x00, 0xff);
  const auto second = render(*renderer, *scene);
  EXPECT_EQ(countColor(second, 0, 0, W, H, RED, 120), 0) << "some copy kept the old color";
  EXPECT_NEAR(countColor(second, 0, 0, W, H, BLUE, 120), redPixels, redPixels / 10);

  // Hiding one of the groups (the first object is the ground) hides only
  // that copy:
  (*std::next(scene->getViewport()->begin()))->setVisibility(false);
  const auto third = render(*renderer, *scene);
  const int bluePixels = countColor(third, 0, 0, W, H, BLUE, 120);
  EXPECT_GT(bluePixels, 0);
  EXPECT_LT(bluePixels, countColor(second, 0, 0, W, H, BLUE, 120));
}

#endif  // RUN_OFFSCREEN_RENDER_TESTS
