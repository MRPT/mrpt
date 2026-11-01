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

/** Offscreen rendering tests checking that the camera used by the renderer
 *  and the one used by 3D picking (Viewport::get3DRayForPixelCoord()) agree,
 *  for orbit (with roll) and 6DOF cameras, projective and orthogonal.
 */

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/geometry.h>  // crossProduct3D()
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/Scene.h>

#include <array>
#include <cmath>
#include <memory>
#include <optional>

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

struct Marker
{
  mrpt::math::TPoint3D center;
  RGB color;
};

const std::array<Marker, 3> kMarkers = {
    {
     {{2.0, 0.0, 0.0}, {255, 0, 0}},
     {{0.0, 2.0, 0.5}, {0, 255, 0}},
     {{-1.0, -1.5, 1.0}, {0, 0, 255}},
     }
};

mrpt::viz::Scene::Ptr markersScene()
{
  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  for (const auto& m : kMarkers)
  {
    auto s = mrpt::viz::CSphere::Create(0.15f);
    s->setLocation(m.center);
    s->setColor_u8(
        static_cast<uint8_t>(m.color.r), static_cast<uint8_t>(m.color.g),
        static_cast<uint8_t>(m.color.b));
    scene->insert(s);
  }
  return scene;
}

/** A pixel belongs to a marker if its channel is clearly dominant (robust to
 * the shading of the default lighting). */
bool isMarkerPixel(const RGB& p, const RGB& markerColor)
{
  const int c[3] = {p.r, p.g, p.b};
  const int m[3] = {markerColor.r, markerColor.g, markerColor.b};
  for (int i = 0; i < 3; i++)
  {
    if (m[i] != 0)
    {
      return c[i] > 60 && c[i] > 2 * c[(i + 1) % 3] + 20 && c[i] > 2 * c[(i + 2) % 3] + 20;
    }
  }
  return false;
}

/** Centroid (pixel coordinates) of the marker blob, if visible. */
std::optional<std::array<double, 2>> blobCentroid(const mrpt::img::CImage& im, const RGB& color)
{
  double sx = 0;
  double sy = 0;
  int n = 0;
  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      if (isMarkerPixel(pixelRGB(im, x, y), color))
      {
        sx += x;
        sy += y;
        n++;
      }
    }
  }
  if (n < 4)
  {
    return std::nullopt;
  }
  return std::array<double, 2>{sx / n, sy / n};
}

/** Renders the scene with its current camera and checks that the picking ray
 * through each rendered marker passes through the marker. */
void checkRenderMatchesPicking(
    mrpt::opengl::CFBORender& r, mrpt::viz::Scene& scene, const char* label)
{
  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);
  r.render_RGB(scene, frame);

  int visible = 0;
  for (const auto& m : kMarkers)
  {
    const auto c = blobCentroid(frame, m.color);
    if (!c)
    {
      continue;
    }
    visible++;
    const auto ray = scene.getViewport()->get3DRayForPixelCoord(
        mrpt::img::TPixelCoord(
            static_cast<int>(std::lround((*c)[0])), static_cast<int>(std::lround((*c)[1]))),
        mrpt::img::TPixelCoord(W, H));
    // The marker radius is 0.15: a ray through its rendered center must pass
    // well inside it.
    EXPECT_LT(ray.distance(m.center), 0.1)
        << label << ": marker at " << m.center.asString() << " rendered at pixel (" << (*c)[0]
        << "," << (*c)[1] << ")";
  }
  EXPECT_GE(visible, 2) << label << ": too few markers visible to test";
}

mrpt::viz::CCamera& mainCamera(mrpt::viz::Scene& scene) { return scene.getViewport()->getCamera(); }

}  // namespace

TEST(CFBORender_CameraPicking, OrbitCameraWithRoll)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering context available";
  }
  auto scene = markersScene();
  auto& cam = mainCamera(*scene);
  cam.setPointingAt(0.3f, 0.2f, 0.4f);
  cam.setZoomDistance(9.0f);
  cam.setAzimuthDegrees(30.0f);
  cam.setElevationDegrees(35.0f);

  for (const float roll : {0.0f, 35.0f, -70.0f})
  {
    cam.setRollDegrees(roll);
    cam.setProjectiveModel(true);
    checkRenderMatchesPicking(*renderer, *scene, "orbit projective");
    cam.setOrthogonal(true);
    checkRenderMatchesPicking(*renderer, *scene, "orbit orthogonal");
  }
}

TEST(CFBORender_CameraPicking, RollRotatesTheImage)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering context available";
  }
  auto scene = markersScene();
  auto& cam = mainCamera(*scene);
  cam.setPointingAt(0, 0, 0);
  cam.setZoomDistance(12.0f);
  cam.setAzimuthDegrees(-90.0f);
  cam.setElevationDegrees(30.0f);

  const auto& red = kMarkers[0];
  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);

  cam.setRollDegrees(0);
  renderer->render_RGB(*scene, frame);
  const auto c0 = blobCentroid(frame, red.color);
  ASSERT_TRUE(c0.has_value());

  cam.setRollDegrees(90);
  renderer->render_RGB(*scene, frame);
  const auto c1 = blobCentroid(frame, red.color);
  ASSERT_TRUE(c1.has_value());

  // Offsets from the image center (pointing point), with y pointing down.
  // A positive roll makes the scene appear rotated counterclockwise:
  // (x,y) -> (y,-x).
  const double x0 = (*c0)[0] - W / 2;
  const double y0 = (*c0)[1] - H / 2;
  const double x1 = (*c1)[0] - W / 2;
  const double y1 = (*c1)[1] - H / 2;
  ASSERT_GT(std::hypot(x0, y0), 20.0);
  EXPECT_NEAR(x1, y0, 2.0);
  EXPECT_NEAR(y1, -x0, 2.0);
}

TEST(CFBORender_CameraPicking, SixDOFCamera)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering context available";
  }
  auto scene = markersScene();
  auto& cam = mainCamera(*scene);
  cam.setZoomDistance(9.0f);  // orthogonal scale

  // Eye at (6,4,3), looking at the origin, with some roll: build the
  // rotation with +Z forward, +X right, +Y down.
  const mrpt::math::TPoint3D eye(6, 4, 3);
  const mrpt::math::TPoint3D target(0, 0, 0.3);
  mrpt::math::TVector3D fwd = target - eye;
  fwd *= 1.0 / fwd.norm();
  mrpt::math::TVector3D right = mrpt::math::crossProduct3D(fwd, mrpt::math::TVector3D(0, 0, 1));
  right *= 1.0 / right.norm();
  const mrpt::math::TVector3D down = mrpt::math::crossProduct3D(fwd, right);
  mrpt::math::CMatrixDouble33 R;
  for (int i = 0; i < 3; i++)
  {
    R(i, 0) = right[i];
    R(i, 1) = down[i];
    R(i, 2) = fwd[i];
  }
  const mrpt::poses::CPose3D lookAt(R, mrpt::math::TVector3D(eye.x, eye.y, eye.z));
  // Roll: a rotation around the camera +Z (viewing) axis:
  const auto rollPose = lookAt + mrpt::poses::CPose3D::FromYawPitchRoll(0.3, 0, 0);

  scene->getViewport()->setCurrentCameraFromPose(rollPose);
  cam.setProjectiveModel(true);
  checkRenderMatchesPicking(*renderer, *scene, "6DOF projective");
  cam.setOrthogonal(true);
  checkRenderMatchesPicking(*renderer, *scene, "6DOF orthogonal");
}

#endif  // RUN_OFFSCREEN_RENDER_TESTS
