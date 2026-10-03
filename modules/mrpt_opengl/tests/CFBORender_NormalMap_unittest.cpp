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

/** Offscreen rendering tests of normal maps on lit textured triangles: a
 *  synthetic "dome" normal map must be lit on the side facing the light.
 */

#include <gtest/gtest.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/Viewport.h>

#include <algorithm>
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
constexpr int W = 160;
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

/** Tangent-space normal map of a dome, in the usual OpenGL convention: red is
 *  the image right direction, green the image up direction. */
mrpt::img::CImage domeNormalMap()
{
  constexpr int N = 64;
  mrpt::img::CImage im(N, N, mrpt::img::CH_RGB);
  for (int row = 0; row < N; row++)
  {
    for (int col = 0; col < N; col++)
    {
      const double x = (col + 0.5) / N * 2 - 1;
      const double y = 1 - (row + 0.5) / N * 2;
      double nx = 0;
      double ny = 0;
      if (x * x + y * y < 0.64)
      {
        nx = x / 0.8;
        ny = y / 0.8;
      }
      const double nz = std::sqrt(std::max(0.0, 1 - nx * nx - ny * ny));
      im.at<uint8_t>(col, row, 0) = static_cast<uint8_t>((nx * 0.5 + 0.5) * 255);
      im.at<uint8_t>(col, row, 1) = static_cast<uint8_t>((ny * 0.5 + 0.5) * 255);
      im.at<uint8_t>(col, row, 2) = static_cast<uint8_t>((nz * 0.5 + 0.5) * 255);
    }
  }
  return im;
}

/** Renders a white plane (with or without the dome normal map) seen from
 *  above (+X to the right, +Y up), lit by a directional light. */
mrpt::img::CImage renderPlane(
    mrpt::opengl::CFBORender& r,
    bool withNormalMap,
    const mrpt::math::TVector3Df& lightDir,
    bool shadows)
{
  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  auto& lp = vp->lightParameters();
  lp.lights.clear();
  mrpt::viz::TLight sun;
  sun.type = mrpt::viz::TLightType::Directional;
  sun.direction = lightDir;
  sun.diffuse = 1.0f;
  sun.specular = 0.0f;
  lp.lights.push_back(sun);
  lp.ambient = 0.05f;
  vp->enableShadowCasting(shadows);

  auto plane = mrpt::viz::CTexturedPlane::Create(-1, 1, -1, 1);
  plane->enableLighting(true);
  plane->assignImage(solidImage(0xff, 0xff, 0xff));
  if (withNormalMap)
  {
    plane->assignNormalMap(domeNormalMap());
  }
  scene->insert(plane);

  auto& cam = vp->getCamera();
  cam.setPointingAt(0, 0, 0);
  cam.setZoomDistance(3.0f);
  cam.setAzimuthDegrees(-90);
  cam.setElevationDegrees(90);

  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);
  r.render_RGB(*scene, frame);
  return frame;
}

/** Mean gray level of a rectangular region (pixel coordinates). */
double meanGray(const mrpt::img::CImage& im, int x0, int y0, int x1, int y1)
{
  double sum = 0;
  int n = 0;
  for (int y = y0; y < y1; y++)
  {
    for (int x = x0; x < x1; x++)
    {
      const RGB p = pixelRGB(im, x, y);
      sum += (p.r + p.g + p.b) / 3.0;
      n++;
    }
  }
  return sum / n;
}

void checkDomeShading(bool shadows)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No offscreen rendering context available";
  }

  // Inner square of the dome, split into halves:
  constexpr int c = W / 2;
  constexpr int d = W / 6;

  // Light coming from +X (screen right):
  const auto fromX = renderPlane(*r, true, {-1.0f, 0.0f, -0.5f}, shadows);
  EXPECT_GT(meanGray(fromX, c, c - d, c + d, c + d), meanGray(fromX, c - d, c - d, c, c + d) + 30);

  // Light coming from +Y (screen up, lower pixel rows):
  const auto fromY = renderPlane(*r, true, {0.0f, -1.0f, -0.5f}, shadows);
  EXPECT_GT(meanGray(fromY, c - d, c - d, c + d, c), meanGray(fromY, c - d, c, c + d, c + d) + 30);

  // Without the normal map, the flat plane is evenly lit:
  const auto flat = renderPlane(*r, false, {0.0f, -1.0f, -0.5f}, shadows);
  EXPECT_NEAR(meanGray(flat, c - d, c - d, c + d, c), meanGray(flat, c - d, c, c + d, c + d), 5.0);
}

}  // namespace

TEST(CFBORender, NormalMapDomeLitFromLightSide) { checkDomeShading(false); }

TEST(CFBORender, NormalMapDomeLitFromLightSideWithShadows) { checkDomeShading(true); }

#endif  // RUN_OFFSCREEN_RENDER_TESTS
