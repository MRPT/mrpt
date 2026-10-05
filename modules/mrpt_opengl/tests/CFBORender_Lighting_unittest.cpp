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

/** Offscreen rendering tests of point/spot lights (range, shadows) and
 *  emissive maps, asserting invariants on the rendered pixels.
 */

#include <gtest/gtest.h>
#include <mrpt/core/config.h>  // MRPT_IS_BIG_ENDIAN
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CSetOfTexturedTriangles.h>
#include <mrpt/viz/Scene.h>

#include <algorithm>
#include <array>
#include <cmath>
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

size_t pointShadowFacesRendered(const mrpt::opengl::CFBORender& r)
{
  const auto* cs = r.compiledScene();
  if (cs == nullptr)
  {
    return 0;
  }
  return cs->getViewport("main")->lastRenderStats().numPointShadowFacesRendered;
}

/** Number of pixels darker than a gray level */
int darkPixels(const mrpt::img::CImage& im, int threshold)
{
  int n = 0;
  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      const RGB p = pixelRGB(im, x, y);
      if (p.r + p.g + p.b < 3 * threshold)
      {
        n++;
      }
    }
  }
  return n;
}

double meanGray(const mrpt::img::CImage& im, int x0, int y0, int x1, int y1)
{
  double sum = 0;
  for (int y = y0; y < y1; y++)
  {
    for (int x = x0; x < x1; x++)
    {
      const RGB p = pixelRGB(im, x, y);
      sum += (p.r + p.g + p.b) / 3.0;
    }
  }
  return sum / ((x1 - x0) * (y1 - y0));
}

/** A white floor, a floating slab above its center, and a point light above
 * the slab. No directional light. Shadow casting enabled in the viewport. */
struct PointLightScene
{
  mrpt::viz::Scene::Ptr scene = mrpt::viz::Scene::Create();
  mrpt::viz::CBox::Ptr slab;

  explicit PointLightScene(bool castShadows)
  {
    auto vp = scene->getViewport();
    vp->setCustomBackgroundColor({0, 0, 0});
    auto& lp = vp->lightParameters();
    lp.lights.clear();
    lp.ambient = 0.1f;
    auto light = mrpt::viz::TLight::PointLight(
        {0, 0, 3}, {1, 1, 1}, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 20.0f /*range*/);
    light.cast_shadows = castShadows;
    lp.lights.push_back(light);
    vp->enableShadowCasting(true, 512, 512);

    auto floor =
        mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-4, -4, -0.1), mrpt::math::TPoint3D(4, 4, 0));
    floor->setColor_u8(0xff, 0xff, 0xff, 0xff);
    scene->insert(floor);

    slab = mrpt::viz::CBox::Create(
        mrpt::math::TPoint3D(-0.5, -0.5, 1.0), mrpt::math::TPoint3D(0.5, 0.5, 1.2));
    slab->setColor_u8(0xff, 0xff, 0xff, 0xff);
    scene->insert(slab);

    // Looking at the floor from a side, so the shadow is visible beyond the
    // slab (a point light 3 m above casts it 1.5 times wider than the slab):
    auto& cam = vp->getCamera();
    cam.setPointingAt(0, 0, 0);
    cam.setZoomDistance(7.0f);
    cam.setAzimuthDegrees(-90);
    cam.setElevationDegrees(60);
  }
};
}  // namespace

TEST(OpenGLLighting, PointLightRangeLimitsItsReach)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  vp->setCustomBackgroundColor({0, 0, 0});
  auto& lp = vp->lightParameters();
  lp.lights.clear();
  lp.ambient = 0.0f;
  lp.lights.push_back(mrpt::viz::TLight::PointLight(
      {0, 0, 0.5f}, {1, 1, 1}, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f /*range*/));

  auto floor =
      mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-8, -8, -0.1), mrpt::math::TPoint3D(8, 8, 0));
  floor->setColor_u8(0xff, 0xff, 0xff, 0xff);
  scene->insert(floor);

  auto& cam = vp->getCamera();
  cam.setPointingAt(0, 0, 0);
  cam.setZoomDistance(12.0f);
  cam.setAzimuthDegrees(-90);
  cam.setElevationDegrees(89);

  // Without a range, the whole floor is lit (no attenuation):
  const auto unlimited = render(*renderer, *scene);
  EXPECT_EQ(darkPixels(unlimited, 20), 0);

  // With a range, the floor center is still lit, but not beyond the range:
  lp.lights[0].range = 2.0f;
  const auto limited = render(*renderer, *scene);
  EXPECT_GT(meanGray(limited, W / 2 - 5, H / 2 - 5, W / 2 + 5, H / 2 + 5), 100);
  EXPECT_LT(meanGray(limited, 0, 0, 10, 10), 5);
}

TEST(OpenGLLighting, EmissiveMapMakesOnlyPartsGlow)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  vp->setCustomBackgroundColor({0, 0, 0});
  auto& lp = vp->lightParameters();
  lp.lights.clear();  // no lights: only emission is visible
  lp.ambient = 0.0f;

  // A square facing the camera, textured in white, whose emissive map is
  // white on its left half and black on its right half:
  auto quad = mrpt::viz::CSetOfTexturedTriangles::Create();
  const auto makeImage = [](int whiteColumns)
  {
    mrpt::img::CImage im(16, 16, mrpt::img::CH_RGB);
    for (int y = 0; y < 16; y++)
    {
      for (int x = 0; x < 16; x++)
      {
        for (int ch = 0; ch < 3; ch++)
        {
          im.at<uint8_t>(x, y, ch) = x < whiteColumns ? 0xff : 0x00;
        }
      }
    }
    return im;
  };
  const auto tex = makeImage(16);
  const auto emissive = makeImage(8);
  quad->assignImage(tex);
  quad->assignEmissiveMap(emissive);
  quad->materialEmissive(mrpt::img::TColorf(1, 1, 1));

  const auto vertex = [](float x, float y, float u, float v)
  {
    mrpt::viz::TTriangle::Vertex vx;
    vx.xyzrgba.pt = {x, y, 0};
    vx.xyzrgba.r = vx.xyzrgba.g = vx.xyzrgba.b = vx.xyzrgba.a = 0xff;
    vx.uv = {u, v};
    return vx;
  };
  for (const auto& tri : {
           std::array{vertex(-1, -1, 0, 1), vertex(1, -1, 1, 1),  vertex(1, 1, 1, 0)},
           std::array{vertex(-1, -1, 0, 1), vertex(1,  1, 1, 0), vertex(-1, 1, 0, 0)}
  })
  {
    mrpt::viz::TTriangle t;
    for (int i = 0; i < 3; i++)
    {
      t.vertices[i] = tri[i];
    }
    t.computeNormals();
    quad->insertTriangle(t);
  }
  scene->insert(quad);

  auto& cam = vp->getCamera();
  cam.setPointingAt(0, 0, 0);
  cam.setZoomDistance(3.0f);
  cam.setAzimuthDegrees(-90);
  cam.setElevationDegrees(90);

  const auto im = render(*renderer, *scene);
  const double left = meanGray(im, W / 2 - 30, H / 2 - 5, W / 2 - 20, H / 2 + 5);
  const double right = meanGray(im, W / 2 + 20, H / 2 - 5, W / 2 + 30, H / 2 + 5);
  // Which half is which depends on the camera orientation: only the contrast
  // matters.
  EXPECT_GT(std::max(left, right), 200);
  EXPECT_LT(std::min(left, right), 20);
}

TEST(OpenGLLighting, PointLightCastsShadows)
{
#if MRPT_IS_BIG_ENDIAN
  GTEST_SKIP() << "Shadows rendering not tested on big-endian hosts";
#endif
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  PointLightScene noShadows(false);
  const auto imNoShadows = render(*renderer, *noShadows.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 0U);

  renderer->invalidateCompiledScene();
  PointLightScene withShadows(true);
  const auto imShadows = render(*renderer, *withShadows.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 6U);

  // The floor around the slab gets darker:
  EXPECT_GT(darkPixels(imShadows, 100), darkPixels(imNoShadows, 100) + 500);

  // ...but the floor far from the slab remains lit:
  EXPECT_NEAR(meanGray(imShadows, 5, 5, 25, 25), meanGray(imNoShadows, 5, 5, 25, 25), 3.0);
}

TEST(OpenGLLighting, PointLightShadowMapsAreReusedWhileNothingChanges)
{
#if MRPT_IS_BIG_ENDIAN
  GTEST_SKIP() << "Shadows rendering not tested on big-endian hosts";
#endif
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  PointLightScene s(true);
  auto farBox = mrpt::viz::CBox::Create(
      mrpt::math::TPoint3D(-0.5, -0.5, 0), mrpt::math::TPoint3D(0.5, 0.5, 1));
  farBox->setLocation(100, 0, 0);  // beyond the light range
  s.scene->insert(farBox);

  render(*renderer, *s.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 6U);

  // Same scene: reused
  const auto im1 = render(*renderer, *s.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 0U);

  // Moving an object out of reach of the light: reused
  farBox->setLocation(101, 0, 0);
  render(*renderer, *s.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 0U);

  // Moving the shadow caster: rendered again
  s.slab->setLocation(1.0, 0, 0);
  const auto im2 = render(*renderer, *s.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 6U);
  EXPECT_GT(std::abs(darkPixels(im1, 100) - darkPixels(im2, 100)), 0);

  // Moving the light: rendered again
  s.scene->getViewport()->lightParameters().lights[0].position = {0.5f, 0, 3};
  render(*renderer, *s.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 6U);

  // Disabling shadows in the viewport disables point light shadows too:
  s.scene->getViewport()->enableShadowCasting(false);
  render(*renderer, *s.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 0U);
}

TEST(OpenGLLighting, SpotLightShadowsOnlyRenderTheFacesInItsCone)
{
#if MRPT_IS_BIG_ENDIAN
  GTEST_SKIP() << "Shadows rendering not tested on big-endian hosts";
#endif
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  PointLightScene s(true);
  auto& lp = s.scene->getViewport()->lightParameters();
  auto spot = mrpt::viz::TLight::SpotLight(
      {0, 0, 3}, {0, 0, -1}, 25.0f, 30.0f, {1, 1, 1}, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 20.0f);
  spot.cast_shadows = true;
  lp.lights[0] = spot;

  const auto im = render(*renderer, *s.scene);
  // Only the face looking down:
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 1U);

  // The floor right below the slab is in its shadow (darker than the floor
  // lit by the spot):
  EXPECT_GT(darkPixels(im, 100), 500);
}

#endif  // RUN_OFFSCREEN_RENDER_TESTS
