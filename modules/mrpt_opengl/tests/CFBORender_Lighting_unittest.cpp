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

/** Offscreen rendering tests of point/spot lights (range, shadows, lights in
 *  the scene graph) and emissive maps, asserting invariants on the rendered pixels.
 */

#include <gtest/gtest.h>
#include <mrpt/core/config.h>  // MRPT_IS_BIG_ENDIAN
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CLight.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSetOfTexturedTriangles.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/Scene.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

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

/** Shadow casting modes to test: the software GL renderer available on
 * big-endian hosts hangs in shadow render passes. */
std::vector<bool> shadowModesToTest()
{
#if MRPT_IS_BIG_ENDIAN
  return {false};
#else
  return {false, true};
#endif
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
        for (int8_t ch = 0; ch < 3; ch++)
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

  // Setting the same pose again: reused
  s.slab->setLocation(0, 0, 0);
  render(*renderer, *s.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 0U);

  // Moving an object out of reach of the light: reused
  farBox->setLocation(101, 0, 0);
  render(*renderer, *s.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 0U);

  // Moving the shadow caster: only the faces it was or is now in (down, +X)
  // are rendered again
  s.slab->setLocation(1.0, 0, 0);
  const auto im2 = render(*renderer, *s.scene);
  EXPECT_EQ(pointShadowFacesRendered(*renderer), 2U);
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

TEST(OpenGLLighting, PointLightShadowFacesReusedMatchAFullRender)
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
  auto mover = mrpt::viz::CBox::Create(
      mrpt::math::TPoint3D(-0.2, -0.2, 0), mrpt::math::TPoint3D(0.2, 0.2, 0.6));
  s.scene->insert(mover);

  // Move an object around the light, across its cube faces. Each frame reuses
  // the faces of the previous one where nothing changed, and must look the
  // same as rendering all the faces again:
  constexpr int STEPS = 12;
  size_t reusedFaces = 0;
  for (int i = 0; i < STEPS; i++)
  {
    const double ang = 2 * M_PI * i / STEPS;
    mover->setLocation(2.0 * std::cos(ang), 2.0 * std::sin(ang), (i % 3) * 0.8);

    const auto reused = render(*renderer, *s.scene);
    if (i > 0)
    {
      reusedFaces += 6 - pointShadowFacesRendered(*renderer);
    }
    renderer->invalidateCompiledScene();
    const auto full = render(*renderer, *s.scene);
    EXPECT_EQ(pointShadowFacesRendered(*renderer), 6U);

    int different = 0;
    for (int y = 0; y < H; y++)
    {
      for (int x = 0; x < W; x++)
      {
        const RGB a = pixelRGB(reused, x, y);
        const RGB b = pixelRGB(full, x, y);
        if (a.r != b.r || a.g != b.g || a.b != b.b)
        {
          different++;
        }
      }
    }
    EXPECT_EQ(different, 0) << "step " << i;
  }
  EXPECT_GT(reusedFaces, 0U);
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

TEST(OpenGLLighting, ClonedViewportsGetPointLightShadows)
{
#if MRPT_IS_BIG_ENDIAN
  GTEST_SKIP() << "Shadows rendering not tested on big-endian hosts";
#endif
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  // The main viewport takes the left half, and a clone (same objects, camera
  // and lights) the right half:
  PointLightScene s(true);
  auto mainVp = s.scene->getViewport();
  mainVp->setViewportPosition(0.0, 0.0, 0.5, 1.0);
  auto clone = s.scene->createViewport("clone");
  clone->setViewportPosition(0.5, 0.0, 0.5, 1.0);
  clone->setCustomBackgroundColor({0, 0, 0});
  clone->setCloneView("main");
  clone->setCloneCamera(true);
  clone->lightParameters() = mainVp->lightParameters();
  clone->enableShadowCasting(true, 512, 512);

  const auto im = render(*renderer, *s.scene);
  EXPECT_EQ(
      renderer->compiledScene()
          ->getViewport("clone")
          ->lastRenderStats()
          .numPointShadowFacesRendered,
      6U);

  // The clone floor gets darker with its point light shadows (the background
  // and the rest of the scene are the same in both renders):
  const auto darkInRightHalf = [](const mrpt::img::CImage& frame)
  {
    int n = 0;
    for (int y = 0; y < H; y++)
    {
      for (int x = W / 2; x < W; x++)
      {
        const RGB p = pixelRGB(frame, x, y);
        if (p.r + p.g + p.b < 3 * 100)
        {
          n++;
        }
      }
    }
    return n;
  };
  clone->lightParameters().lights[0].cast_shadows = false;
  const auto imNoShadows = render(*renderer, *s.scene);
  EXPECT_GT(darkInRightHalf(im), darkInRightHalf(imNoShadows) + 200)
      << "the clone has no point light shadows";
}

namespace
{
/** A white floor seen from above, with no light other than those added by
 * each test. World X runs along image columns: x=+-3 m are at columns
 * W/2 +- 75 px. */
struct TopViewScene
{
  mrpt::viz::Scene::Ptr scene = mrpt::viz::Scene::Create();
  mrpt::viz::Viewport::Ptr vp = scene->getViewport();

  TopViewScene()
  {
    vp->setCustomBackgroundColor({0, 0, 0});
    auto& lp = vp->lightParameters();
    lp.lights.clear();
    lp.ambient = 0.0f;

    auto floor =
        mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-8, -8, -0.1), mrpt::math::TPoint3D(8, 8, 0));
    floor->setColor_u8(0xff, 0xff, 0xff, 0xff);
    scene->insert(floor);

    auto& cam = vp->getCamera();
    cam.setPointingAt(0, 0, 0);
    cam.setZoomDistance(12.0f);
    cam.setAzimuthDegrees(-90);
    cam.setElevationDegrees(89);
  }
};

mrpt::viz::TLight smallPointLight(const mrpt::math::TPoint3Df& pos)
{
  return mrpt::viz::TLight::PointLight(
      pos, {1, 1, 1}, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.5f /*range*/);
}

double grayAtColumn(const mrpt::img::CImage& im, int x)
{
  return meanGray(im, x - 5, H / 2 - 5, x + 5, H / 2 + 5);
}
}  // namespace

TEST(OpenGLLighting, CLightFollowsItsParentsAndIsSwitchedWithVisibility)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  TopViewScene s;
  auto vehicle = mrpt::viz::CSetOfObjects::Create();
  vehicle->setLocation(3, 0, 0);
  auto lamp = mrpt::viz::CLight::Create(smallPointLight({0, 0, 0.5f}));
  vehicle->insert(lamp);
  s.scene->insert(vehicle);

  constexpr int left = W / 2 - 75;
  constexpr int right = W / 2 + 75;

  const auto atRight = render(*renderer, *s.scene);
  EXPECT_GT(grayAtColumn(atRight, right), 100);
  EXPECT_LT(grayAtColumn(atRight, left), 5);

  // The light moves with its parent:
  vehicle->setLocation(-3, 0, 0);
  const auto atLeft = render(*renderer, *s.scene);
  EXPECT_LT(grayAtColumn(atLeft, right), 5);
  EXPECT_GT(grayAtColumn(atLeft, left), 100);

  // Hidden lights, or lights in hidden containers, are off:
  lamp->setVisibility(false);
  EXPECT_LT(grayAtColumn(render(*renderer, *s.scene), left), 5);

  lamp->setVisibility(true);
  vehicle->setVisibility(false);
  EXPECT_LT(grayAtColumn(render(*renderer, *s.scene), left), 5);

  vehicle->setVisibility(true);
  EXPECT_GT(grayAtColumn(render(*renderer, *s.scene), left), 100);

  // Changing the light parameters is also seen:
  auto l = lamp->light();
  l.diffuse = 0;
  lamp->light(l);
  EXPECT_LT(grayAtColumn(render(*renderer, *s.scene), left), 5);
}

TEST(OpenGLLighting, CLightSpotDirectionIsRotatedWithItsParent)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  // A "headlight" at the origin of a vehicle, pointing forward (+X) and down:
  TopViewScene s;
  auto vehicle = mrpt::viz::CSetOfObjects::Create();
  auto headlight = mrpt::viz::CLight::Create(mrpt::viz::TLight::SpotLight(
      {0, 0, 1.0f}, mrpt::math::TVector3Df(1, 0, -0.35f).unitarize(), 10.0f, 15.0f, {1, 1, 1}, 1.0f,
      0.0f, 1.0f, 0.0f, 0.0f, 6.0f /*range*/));
  vehicle->insert(headlight);
  s.scene->insert(vehicle);

  constexpr int left = W / 2 - 75;
  constexpr int right = W / 2 + 75;

  const auto forward = render(*renderer, *s.scene);
  EXPECT_GT(grayAtColumn(forward, right), 50);
  EXPECT_LT(grayAtColumn(forward, left), 5);

  // Turned around, it lights the other side:
  vehicle->setPose(mrpt::math::TPose3D(0, 0, 0, M_PI, 0, 0));
  const auto backward = render(*renderer, *s.scene);
  EXPECT_LT(grayAtColumn(backward, right), 5);
  EXPECT_GT(grayAtColumn(backward, left), 50);
}

TEST(OpenGLLighting, TooManyLightsKeepsThoseClosestToTheCamera)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  TopViewScene s;
  auto& lp = s.vp->lightParameters();
  // A directional light (with no effect), then MAX_LIGHTS point lights far
  // from the point the camera looks at, all in the viewport:
  lp.lights.push_back(mrpt::viz::TLight::Directional({0, 0, -1}, {1, 1, 1}, 0.0f, 0.0f));
  for (int i = 0; i < mrpt::viz::MAX_LIGHTS; i++)
  {
    lp.lights.push_back(smallPointLight({-3.0f, -2.0f + 0.5f * static_cast<float>(i), 0.5f}));
  }
  // ...and a light in the scene graph, right where the camera looks at:
  s.scene->insert(mrpt::viz::CLight::Create(smallPointLight({0, 0, 0.5f})));

  const auto im = render(*renderer, *s.scene);
  EXPECT_GT(grayAtColumn(im, W / 2), 100) << "the light closest to the camera was dropped";

  const auto& lights = renderer->compiledScene()->getViewport("main")->lightParameters().lights;
  ASSERT_EQ(lights.size(), static_cast<size_t>(mrpt::viz::MAX_LIGHTS));
  EXPECT_EQ(lights.front().type, mrpt::viz::TLightType::Directional);
  EXPECT_FLOAT_EQ(lights.back().position.x, 0.0f);
}

TEST(OpenGLLighting, SpecularFadesAtGrazingAngles)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  // A black, shiny floor: only the specular highlight is visible.
  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  vp->setCustomBackgroundColor({0, 0, 0});
  auto floor = mrpt::viz::CBox::Create(
      mrpt::math::TPoint3D(-50, -50, -0.1), mrpt::math::TPoint3D(50, 50, 0));
  floor->setColor_u8(0, 0, 0, 0xff);
  floor->materialShininess(1.0f);
  scene->insert(floor);

  // The camera looks at the floor center from the mirror direction of the
  // light, so the center gets the full specular highlight:
  const auto highlightAtElevation = [&](float elevDeg, bool shadows)
  {
    const float e = mrpt::DEG2RAD(elevDeg);
    auto& lp = vp->lightParameters();
    lp.lights.clear();
    lp.ambient = 0.0f;
    lp.lights.push_back(mrpt::viz::TLight::Directional(
        {-std::cos(e), 0, -std::sin(e)}, {1, 1, 1}, 0.0f /*diffuse*/, 1.0f /*specular*/));
    vp->enableShadowCasting(shadows, 512, 512);

    auto& cam = vp->getCamera();
    cam.setPointingAt(0, 0, 0);
    cam.setZoomDistance(10.0f);
    cam.setAzimuthDegrees(180);
    cam.setElevationDegrees(elevDeg);
    return meanGray(render(*renderer, *scene), W / 2 - 2, H / 2 - 2, W / 2 + 2, H / 2 + 2);
  };

  for (const bool shadows : shadowModesToTest())
  {
    const double steep = highlightAtElevation(60, shadows);
    const double grazing = highlightAtElevation(3, shadows);
    EXPECT_GT(steep, 150) << "shadows=" << shadows;
    EXPECT_LT(grazing, 0.5 * steep) << "shadows=" << shadows;
  }
}

TEST(OpenGLLighting, BackSidesOfThinSurfacesAreLitFromTheirSide)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }

  // A thin horizontal surface at z=2 (normal +Z), e.g. a roof, either plain
  // or textured (white), so all the lit shaders are covered:
  const auto makeSurface = [](bool textured) -> mrpt::viz::CVisualObject::Ptr
  {
    if (textured)
    {
      auto p = mrpt::viz::CTexturedPlane::Create(-4, 4, -4, 4);
      mrpt::img::CImage white(4, 4, mrpt::img::CH_RGB);
      white.filledRectangle({0, 0}, {3, 3}, mrpt::img::TColor::white());
      p->assignImage(white);
      p->enableLighting(true);
      p->setLocation(0, 0, 2);
      return p;
    }
    auto t = mrpt::viz::CSetOfTriangles::Create();
    using P = mrpt::math::TPoint3D;
    t->insertTriangle(
        mrpt::viz::TTriangle(mrpt::math::TPolygon3D({P(-4, -4, 2), P(4, -4, 2), P(4, 4, 2)})));
    t->insertTriangle(
        mrpt::viz::TTriangle(mrpt::math::TPolygon3D({P(-4, -4, 2), P(4, 4, 2), P(-4, 4, 2)})));
    return t;
  };

  for (const bool textured : {false, true})
  {
    for (const bool shadows : shadowModesToTest())
    {
      auto scene = mrpt::viz::Scene::Create();
      auto vp = scene->getViewport();
      vp->setCustomBackgroundColor({0, 0, 0});
      vp->enableShadowCasting(shadows, 512, 512);
      scene->insert(makeSurface(textured));

      // The sun, above:
      auto& lp = vp->lightParameters();
      lp.lights.clear();
      lp.ambient = 0.0f;
      lp.lights.push_back(
          mrpt::viz::TLight::Directional({0, 0, -1}, {1, 1, 1}, 1.0f /*diffuse*/, 0.0f));

      auto& cam = vp->getCamera();
      cam.setPointingAt(0, 0, 2);
      cam.setZoomDistance(4.0f);
      cam.setAzimuthDegrees(-90);
      const auto centerGray = [&](float camElevDeg)
      {
        cam.setElevationDegrees(camElevDeg);
        return meanGray(render(*renderer, *scene), W / 2 - 5, H / 2 - 5, W / 2 + 5, H / 2 + 5);
      };
      const auto ctx = [&]()
      { return ::testing::Message() << "textured=" << textured << " shadows=" << shadows; };

      // The top side is lit by the sun, but not the bottom one:
      EXPECT_GT(centerGray(80), 100) << ctx();
      EXPECT_LT(centerGray(-80), 10) << ctx();

      // A lamp below lights the bottom side, without shadowing it:
      auto lamp = mrpt::viz::TLight::PointLight(
          {0, 0, 1}, {1, 1, 1}, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 5.0f /*range*/);
      lamp.cast_shadows = shadows;
      lp.lights.push_back(lamp);
      EXPECT_GT(centerGray(-80), 60) << ctx();
    }
  }
}

#endif  // RUN_OFFSCREEN_RENDER_TESTS
