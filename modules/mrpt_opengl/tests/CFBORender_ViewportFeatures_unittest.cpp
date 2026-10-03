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

/** Offscreen rendering tests of the per-viewport features (borders, image view,
 *  cloned viewports, text overlays, shadows and SSAO), asserting invariants on
 *  the rendered pixels.
 */

#include <gtest/gtest.h>
#include <mrpt/core/config.h>  // MRPT_IS_BIG_ENDIAN
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/Scene.h>

#include <array>
#include <memory>

#include "render_pixel_utils.h"

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
#endif

#if defined(RUN_OFFSCREEN_RENDER_TESTS)

using namespace mrpt::opengl::testing;

namespace
{
constexpr int W = 240;
constexpr int H = 160;

/** Returns null if this machine cannot create an offscreen rendering context. */
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

mrpt::viz::Scene::Ptr sceneWithRedBox(const mrpt::img::TColorf& bg = {1.0f, 1.0f, 1.0f})
{
  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor(bg);
  auto box =
      mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-1, -1, 0), mrpt::math::TPoint3D(1, 1, 2));
  box->setColor_u8(0xff, 0x00, 0x00, 0xff);
  scene->insert(box);
  scene->getViewport()->getCamera().setPointingAt(0, 0, 1);
  scene->getViewport()->getCamera().setZoomDistance(8.0f);
  scene->getViewport()->getCamera().setAzimuthDegrees(35);
  scene->getViewport()->getCamera().setElevationDegrees(25);
  return scene;
}
}  // namespace

TEST(OpenGLViewport, BorderIsDrawnAroundTheViewport)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  vp->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  vp->setBorderSize(6);
  vp->setBorderColor(mrpt::img::TColor(0x00, 0x00, 0xff));

  const auto frame = render(*renderer, *scene);
  const RGB blue{0, 0, 255};
  const RGB white{255, 255, 255};

  // Blue along the four edges, white in the middle:
  EXPECT_GT(countColor(frame, 0, 0, W, 3, blue), W / 2);
  EXPECT_GT(countColor(frame, 0, H - 3, W, H, blue), W / 2);
  EXPECT_GT(countColor(frame, 0, 0, 3, H, blue), H / 2);
  EXPECT_GT(countColor(frame, W - 3, 0, W, H, blue), H / 2);
  EXPECT_TRUE(isNear(pixelRGB(frame, W / 2, H / 2), white));
}

TEST(OpenGLViewport, ImageViewFillsItsViewport)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});

  // The left half of the window shows an image:
  auto left = scene->createViewport("left");
  left->setViewportPosition(0.0, 0.0, 0.5, 1.0);
  mrpt::img::CImage img(32, 32, mrpt::img::CH_RGB);
  img.filledRectangle({0, 0}, {31, 31}, mrpt::img::TColor(0x00, 0xc0, 0x00));
  left->setImageView(img, false /*not transparent*/);
  ASSERT_TRUE(left->isImageViewMode());

  const auto frame = render(*renderer, *scene);
  const RGB green{0, 0xc0, 0};

  EXPECT_TRUE(isNear(pixelRGB(frame, W / 4, H / 2), green, 60)) << "no image in the left half";
  // The right half is not covered by it:
  EXPECT_FALSE(isNear(pixelRGB(frame, 3 * W / 4, H / 2), green, 60));

  // Going back to normal mode removes it:
  left->setNormalMode();
  const auto frame2 = render(*renderer, *scene);
  EXPECT_FALSE(isNear(pixelRGB(frame2, W / 4, H / 2), green, 60));
}

TEST(OpenGLViewport, ClonedViewportShowsTheObjectsOfTheOtherOne)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  auto scene = sceneWithRedBox();
  const RGB red{255, 0, 0};

  // The main viewport takes the left half, and a clone the right half:
  scene->getViewport()->setViewportPosition(0.0, 0.0, 0.5, 1.0);
  auto clone = scene->createViewport("clone");
  clone->setViewportPosition(0.5, 0.0, 0.5, 1.0);
  clone->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  clone->setCloneView("main");
  clone->setCloneCamera(true);

  const auto frame = render(*renderer, *scene);
  const int left = countColor(frame, 0, 0, W / 2, H, red);
  const int right = countColor(frame, W / 2, 0, W, H, red);
  EXPECT_GT(left, 200);
  EXPECT_GT(right, 200) << "the cloned viewport shows nothing";
  // Same objects, same camera, same viewport size: the same number of pixels
  // give or take the anti-aliasing at the border:
  EXPECT_NEAR(left, right, left / 5);
}

TEST(OpenGLViewport, TransparentViewportShowsTheOneBehind)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  auto scene = sceneWithRedBox();
  const RGB red{255, 0, 0};

  // A viewport on top of the whole window, with no objects: opaque, it hides
  // the box; transparent, it does not.
  auto over = scene->createViewport("over");
  over->setViewportPosition(0.0, 0.0, 1.0, 1.0);
  over->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  over->setTransparent(false);
  const int hidden = countColor(render(*renderer, *scene), 0, 0, W, H, red);

  over->setTransparent(true);
  const int visible = countColor(render(*renderer, *scene), 0, 0, W, H, red);

  EXPECT_LT(hidden, 20);
  EXPECT_GT(visible, 200);
}

TEST(OpenGLViewport, TextMessagesAreOverlaid)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  vp->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});

  const RGB black{0, 0, 0};
  const int before = countColor(render(*renderer, *scene), 0, 0, W, H, black, 60);
  EXPECT_EQ(before, 0);

  mrpt::viz::TFontParams fp;
  fp.color = mrpt::img::TColorf(0.0f, 0.0f, 0.0f);
  fp.vfont_scale = 16;
  vp->addTextMessage(10, 30, "MRPT TEXT", 1, fp);
  const auto frame = render(*renderer, *scene);
  const int with = countColor(frame, 0, 0, W, H, black, 60);
  EXPECT_GT(with, 30) << "the text is not visible";

  // Updating the text changes what is drawn, clearing removes it:
  ASSERT_TRUE(vp->updateTextMessage(1, "OTHER"));
  const int updated = countColor(render(*renderer, *scene), 0, 0, W, H, black, 60);
  EXPECT_GT(updated, 30);
  vp->clearTextMessages();
  EXPECT_EQ(countColor(render(*renderer, *scene), 0, 0, W, H, black, 60), 0);
}

TEST(OpenGLViewport, ShadowsAndSSAORender)
{
#if MRPT_IS_BIG_ENDIAN
  // The software GL renderer available on big-endian hosts hangs in these
  // render passes.
  GTEST_SKIP() << "Shadows/SSAO rendering not tested on big-endian hosts";
#endif
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  const RGB red{255, 0, 0};

  auto scene = sceneWithRedBox({0.9f, 0.9f, 0.9f});
  auto floor = mrpt::viz::CGridPlaneXY::Create(-5, 5, -5, 5, 0, 1);
  scene->insert(floor);
  auto sphere = mrpt::viz::CSphere::Create(0.5f);
  sphere->setLocation(2.5, 1.0, 0.5);
  sphere->setColor_u8(0x00, 0x00, 0xff, 0xff);
  scene->insert(sphere);
  auto vp = scene->getViewport();

  // Reference without any of the two effects:
  const auto plain = render(*renderer, *scene);
  const int plainRed = countColor(plain, 0, 0, W, H, red, 80);
  ASSERT_GT(plainRed, 200);

  // Shadows: the box must still be visible, and the frame must be different:
  vp->enableShadowCasting(true, 512, 512);
  ASSERT_TRUE(vp->isShadowCastingEnabled());
  const auto shadowed = render(*renderer, *scene);
  EXPECT_GT(countColor(shadowed, 0, 0, W, H, red, 100), 100);

  // SSAO:
  vp->enableShadowCasting(false);
  vp->enableSSAO(true);
  ASSERT_TRUE(vp->isSSAOEnabled());
  // (This is the first SSAO frame: everything has to be ready right away)
  const auto ao = render(*renderer, *scene);
  const int aoRed = countColor(ao, 0, 0, W, H, red, 100);
  EXPECT_NEAR(aoRed, plainRed, plainRed / 4) << "SSAO output is not the scene";
  // ...and the frame must look like the plain one, only a bit darker in the
  // creases: most pixels stay close.
  int similar = 0;
  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      similar += isNear(pixelRGB(ao, x, y), pixelRGB(plain, x, y), 90) ? 1 : 0;
    }
  }
  EXPECT_GT(similar, (W * H * 8) / 10);

  // Both together, and rendering repeatedly reuses the shadow/SSAO buffers:
  vp->enableShadowCasting(true, 512, 512);
  const auto both1 = render(*renderer, *scene);
  const auto both2 = render(*renderer, *scene);
  EXPECT_GT(countColor(both1, 0, 0, W, H, red, 100), 100);
  EXPECT_EQ(countColor(both1, 0, 0, W, H, red, 100), countColor(both2, 0, 0, W, H, red, 100));

  // Switching them off restores the plain look:
  vp->enableShadowCasting(false);
  vp->enableSSAO(false);
  const auto plain2 = render(*renderer, *scene);
  EXPECT_NEAR(countColor(plain2, 0, 0, W, H, red, 80), plainRed, plainRed / 20 + 5);
}

TEST(OpenGLViewport, CulledFacesStillCastShadows)
{
#if MRPT_IS_BIG_ENDIAN
  GTEST_SKIP() << "Shadows rendering not tested on big-endian hosts";
#endif
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  // Number of pixels in the shadow of something:
  const auto darkPixels = [](const mrpt::img::CImage& im)
  {
    int n = 0;
    for (int y = 0; y < H; y++)
    {
      for (int x = 0; x < W; x++)
      {
        const RGB p = pixelRGB(im, x, y);
        if (p.r + p.g + p.b < 3 * 150)
        {
          n++;
        }
      }
    }
    return n;
  };

  for (const auto cull : {mrpt::viz::TCullFace::FRONT, mrpt::viz::TCullFace::BACK})
  {
    auto scene = mrpt::viz::Scene::Create();
    auto vp = scene->getViewport();
    auto& lp = vp->lightParameters();
    lp.lights.clear();
    mrpt::viz::TLight sun;
    sun.type = mrpt::viz::TLightType::Directional;
    sun.direction = {-0.32f, 0.56f, -0.77f};
    sun.diffuse = 1.0f;
    sun.specular = 0.0f;
    lp.lights.push_back(sun);
    lp.ambient = 0.1f;
    vp->enableShadowCasting(true, 512, 512);

    auto floor =
        mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-4, -4, -0.1), mrpt::math::TPoint3D(4, 4, 0));
    floor->setColor_u8(0xff, 0xff, 0xff, 0xff);
    scene->insert(floor);

    // A white "ceiling" made of triangles facing up, with one of its sides
    // culled from the camera:
    auto ceiling = mrpt::viz::CSetOfTriangles::Create();
    const auto addTri = [&ceiling](const std::array<mrpt::math::TPoint2Df, 3>& xy)
    {
      mrpt::viz::TTriangle t;
      for (int i = 0; i < 3; i++)
      {
        t.vertices[i].xyzrgba.pt = {xy[i].x, xy[i].y, 1.0f};
      }
      t.setColor(mrpt::img::TColor(0xff, 0xff, 0xff));
      t.computeNormals();
      ceiling->insertTriangle(t);
    };
    addTri({
        {{-1, -1}, {1, -1}, {1, 1}}
    });
    addTri({
        {{-1, -1}, {1, 1}, {-1, 1}}
    });
    ceiling->cullFaces(cull);
    scene->insert(ceiling);

    auto& cam = vp->getCamera();
    cam.setPointingAt(0, 0, 0);
    cam.setZoomDistance(7.0f);
    cam.setAzimuthDegrees(-100);
    cam.setElevationDegrees(55);

    // Its shadow darkens the floor, whatever side is culled:
    const auto withCeiling = render(*renderer, *scene);
    ceiling->setVisibility(false);
    const auto withoutCeiling = render(*renderer, *scene);
    EXPECT_GT(darkPixels(withCeiling), darkPixels(withoutCeiling) + 500)
        << "cull: " << mrpt::typemeta::TEnumType<mrpt::viz::TCullFace>::value2name(cull);
  }
}

TEST(OpenGLViewport, CastersFarTowardsTheLightStillCastShadows)
{
#if MRPT_IS_BIG_ENDIAN
  GTEST_SKIP() << "Shadows rendering not tested on big-endian hosts";
#endif
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  auto& lp = vp->lightParameters();
  lp.lights.clear();
  mrpt::viz::TLight sun;
  sun.type = mrpt::viz::TLightType::Directional;
  sun.direction = {-0.6f, 0.5f, -0.62f};  // ~38 deg above the horizon
  sun.diffuse = 1.0f;
  sun.specular = 0.0f;
  lp.lights.push_back(sun);
  lp.ambient = 0.1f;
  vp->enableShadowCasting(true, 512, 512);

  auto floor = mrpt::viz::CBox::Create(
      mrpt::math::TPoint3D(-40, -40, -0.1), mrpt::math::TPoint3D(40, 40, 0));
  floor->setColor_u8(0xff, 0xff, 0xff, 0xff);
  scene->insert(floor);

  // A large ceiling high above, much farther from the floor seen by the
  // camera than the size of the visible area:
  auto ceiling =
      mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-40, -40, 8), mrpt::math::TPoint3D(40, 40, 8.1));
  ceiling->setColor_u8(0xff, 0xff, 0xff, 0xff);
  scene->insert(ceiling);

  // Camera below the ceiling, looking down at the floor from close:
  auto& cam = vp->getCamera();
  cam.setPointingAt(0, 0, 0);
  cam.setZoomDistance(3.0f);
  cam.setAzimuthDegrees(-90);
  cam.setElevationDegrees(89);

  const auto meanGray = [](const mrpt::img::CImage& im)
  {
    double sum = 0;
    for (int y = H / 2 - 10; y < H / 2 + 10; y++)
    {
      for (int x = W / 2 - 10; x < W / 2 + 10; x++)
      {
        const RGB p = pixelRGB(im, x, y);
        sum += (p.r + p.g + p.b) / 3.0;
      }
    }
    return sum / 400;
  };

  const double underCeiling = meanGray(render(*renderer, *scene));
  ceiling->setVisibility(false);
  const double sunlit = meanGray(render(*renderer, *scene));
  EXPECT_LT(underCeiling, 0.5 * sunlit);
}

TEST(OpenGLViewport, CameraOnlyCloneKeepsTheViewportsOwnObjects)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  const RGB red{255, 0, 0};
  const RGB blue{0, 0, 255};

  // Left half: the red box. Right half: another viewport with its own blue
  // sphere, which only borrows the camera of the first one.
  auto scene = sceneWithRedBox();
  scene->getViewport()->setViewportPosition(0.0, 0.0, 0.5, 1.0);
  auto other = scene->createViewport("other");
  other->setViewportPosition(0.5, 0.0, 0.5, 1.0);
  other->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  auto sphere = mrpt::viz::CSphere::Create(1.0f);
  sphere->setLocation(0, 0, 1);
  sphere->setColor_u8(0x00, 0x00, 0xff, 0xff);
  other->insert(sphere);
  other->setClonedCameraFrom("main");

  const auto frame = render(*renderer, *scene);
  EXPECT_GT(countColor(frame, 0, 0, W / 2, H, red), 200);
  EXPECT_GT(countColor(frame, W / 2, 0, W, H, blue), 100) << "the own object is missing";
  EXPECT_LT(countColor(frame, W / 2, 0, W, H, red), 20) << "objects of the other viewport leaked";
}

TEST(OpenGLViewport, LeavingCloneModeShowsTheOwnObjectsAgain)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  const RGB red{255, 0, 0};

  auto scene = sceneWithRedBox();
  scene->getViewport()->setViewportPosition(0.0, 0.0, 0.5, 1.0);
  auto clone = scene->createViewport("clone");
  clone->setViewportPosition(0.5, 0.0, 0.5, 1.0);
  clone->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  clone->setCloneView("main");
  clone->setCloneCamera(true);

  EXPECT_GT(countColor(render(*renderer, *scene), W / 2, 0, W, H, red), 200);

  // Back to a normal viewport: it has no objects, so the box is gone.
  clone->resetCloneView();
  const auto frame = render(*renderer, *scene);
  EXPECT_LT(countColor(frame, W / 2, 0, W, H, red), 20);
  EXPECT_GT(countColor(frame, 0, 0, W / 2, H, red), 200);
}

#endif  // RUN_OFFSCREEN_RENDER_TESTS
