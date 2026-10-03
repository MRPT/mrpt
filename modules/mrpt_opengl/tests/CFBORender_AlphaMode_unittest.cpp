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

/** Tests of texture alpha modes (TAlphaMode): automatic detection, and
 *  rendering of alpha cutout textures, which must not hide what is behind
 *  their transparent parts in any drawing order, nor cast shadows there.
 */

#include <gtest/gtest.h>
#include <mrpt/core/config.h>  // MRPT_IS_BIG_ENDIAN
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/Scene.h>

#include <functional>
#include <memory>

#include "render_pixel_utils.h"

namespace
{
mrpt::img::CImage solidImage(uint8_t r, uint8_t g, uint8_t b)
{
  mrpt::img::CImage im(8, 8, mrpt::img::CH_RGB);
  for (int y = 0; y < 8; y++)
  {
    for (int x = 0; x < 8; x++)
    {
      im.at<uint8_t>(x, y, 0) = r;
      im.at<uint8_t>(x, y, 1) = g;
      im.at<uint8_t>(x, y, 2) = b;
    }
  }
  return im;
}

/** An 8x8 alpha image, with the given value for each column. */
mrpt::img::CImage alphaImage(const std::function<uint8_t(int)>& alphaOfColumn)
{
  mrpt::img::CImage im(8, 8, mrpt::img::CH_GRAY);
  for (int y = 0; y < 8; y++)
  {
    for (int x = 0; x < 8; x++)
    {
      im.at<uint8_t>(x, y) = alphaOfColumn(x);
    }
  }
  return im;
}

/** Transparent left half, opaque right half */
mrpt::img::CImage halfTransparent()
{
  return alphaImage([](int x) { return static_cast<uint8_t>(x < 4 ? 0 : 255); });
}
}  // namespace

TEST(TAlphaMode, AutoDetectionFromTheTexture)
{
  auto obj = mrpt::viz::CTexturedPlane::Create(-1, 1, -1, 1);

  // No alpha: no cutout
  obj->assignImage(solidImage(0, 255, 0));
  EXPECT_EQ(obj->effectiveAlphaCutoff(), 0.0f);

  // Binary alpha: cutout
  obj->assignImage(solidImage(0, 255, 0), halfTransparent());
  EXPECT_GT(obj->effectiveAlphaCutoff(), 0.0f);

  // Smooth alpha gradient: blending
  obj->assignImage(
      solidImage(0, 255, 0), alphaImage([](int x) { return static_cast<uint8_t>(x * 30 + 20); }));
  EXPECT_EQ(obj->effectiveAlphaCutoff(), 0.0f);

  // Explicit modes override the detected one:
  obj->setAlphaMode(mrpt::viz::TAlphaMode::Mask);
  obj->setAlphaCutoff(0.3f);
  EXPECT_FLOAT_EQ(obj->effectiveAlphaCutoff(), 0.3f);
  obj->setAlphaMode(mrpt::viz::TAlphaMode::Opaque);
  EXPECT_LT(obj->effectiveAlphaCutoff(), 0.0f);
}

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL

using namespace mrpt::opengl::testing;

namespace
{
constexpr int W = 160;
constexpr int H = 160;

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

/** A green plane above a red one, seen from above (+X to the right). The green
 *  plane has its left half transparent. */
mrpt::img::CImage renderCutoutOverRed(mrpt::opengl::CFBORender& r, mrpt::viz::TAlphaMode mode)
{
  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  vp->setCustomBackgroundColor({0.0f, 0.0f, 1.0f});

  auto red = mrpt::viz::CTexturedPlane::Create(-2, 2, -2, 2);
  red->assignImage(solidImage(0xff, 0x00, 0x00));
  red->enableLighting(false);
  scene->insert(red);

  // Closer to the camera, so it is drawn before the red plane:
  auto green = mrpt::viz::CTexturedPlane::Create(-1, 1, -1, 1);
  green->setLocation(0, 0, 1);
  green->assignImage(solidImage(0x00, 0xff, 0x00), halfTransparent());
  green->setAlphaMode(mode);
  green->enableLighting(false);
  scene->insert(green);

  auto& cam = vp->getCamera();
  cam.setPointingAt(0, 0, 0);
  cam.setZoomDistance(5.0f);
  cam.setAzimuthDegrees(-90);
  cam.setElevationDegrees(90);

  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);
  r.render_RGB(*scene, frame);
  return frame;
}
}  // namespace

TEST(TAlphaMode, CutoutShowsWhatIsBehindInAnyDrawingOrder)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No offscreen rendering context available";
  }
  const RGB red{255, 0, 0};
  const RGB green{0, 255, 0};

  for (const auto mode : {mrpt::viz::TAlphaMode::Auto, mrpt::viz::TAlphaMode::Mask})
  {
    const auto frame = renderCutoutOverRed(*r, mode);
    // Left half of the green plane (transparent): the red plane behind it
    EXPECT_TRUE(isNear(pixelRGB(frame, W / 2 - 15, H / 2), red, 60));
    // Right half (opaque):
    EXPECT_TRUE(isNear(pixelRGB(frame, W / 2 + 15, H / 2), green, 60));
  }
}

TEST(TAlphaMode, CutoutPartsCastNoShadows)
{
#if MRPT_IS_BIG_ENDIAN
  GTEST_SKIP() << "Shadows rendering not tested on big-endian hosts";
#endif
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No offscreen rendering context available";
  }

  const auto meanGray = [](const mrpt::img::CImage& im, int cx)
  {
    double sum = 0;
    int n = 0;
    for (int y = H / 2 - 6; y < H / 2 + 6; y++)
    {
      for (int x = cx - 6; x < cx + 6; x++)
      {
        const RGB p = pixelRGB(im, x, y);
        sum += (p.r + p.g + p.b) / 3.0;
        n++;
      }
    }
    return sum / n;
  };

  for (const auto mode : {mrpt::viz::TAlphaMode::Mask, mrpt::viz::TAlphaMode::Opaque})
  {
    auto scene = mrpt::viz::Scene::Create();
    auto vp = scene->getViewport();
    auto& lp = vp->lightParameters();
    lp.lights.clear();
    mrpt::viz::TLight sun;
    sun.type = mrpt::viz::TLightType::Directional;
    sun.direction = {1.0f, 0.0f, -1.0f};  // the shadow moves +X as much as height
    sun.diffuse = 1.0f;
    sun.specular = 0.0f;
    lp.lights.push_back(sun);
    lp.ambient = 0.1f;
    vp->enableShadowCasting(true, 512, 512);

    auto floor =
        mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-4, -4, -0.1), mrpt::math::TPoint3D(6, 4, 0));
    floor->setColor_u8(0xff, 0xff, 0xff, 0xff);
    scene->insert(floor);

    // At height 2: its shadow is the X range [1, 3], [1, 2] from its
    // transparent half.
    auto quad = mrpt::viz::CTexturedPlane::Create(-1, 1, -1, 1);
    quad->setLocation(0, 0, 2);
    quad->assignImage(solidImage(0x00, 0xff, 0x00), halfTransparent());
    quad->setAlphaMode(mode);
    scene->insert(quad);

    auto& cam = vp->getCamera();
    cam.setPointingAt(1, 0, 0);
    cam.setZoomDistance(8.0f);
    cam.setAzimuthDegrees(-90);
    cam.setElevationDegrees(90);

    mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);
    r->render_RGB(*scene, frame);

    // Pixels of the floor at X=1.5 (below the transparent half) and X=2.5:
    const double halfExtent = 8.0 * std::tan(mrpt::DEG2RAD(cam.getProjectiveFOVdeg() / 2));
    const auto px = [&](double x)
    { return static_cast<int>(W / 2 + (x - 1.0) / halfExtent * (W / 2)); };
    const double underTransparent = meanGray(frame, px(1.5));
    const double underOpaque = meanGray(frame, px(2.5));

    if (mode == mrpt::viz::TAlphaMode::Mask)
    {
      EXPECT_GT(underTransparent, underOpaque + 60);
    }
    else
    {
      EXPECT_NEAR(underTransparent, underOpaque, 20);
    }
  }
}

#endif  // MRPT_HAS_OPENGL && MRPT_HAS_EGL
