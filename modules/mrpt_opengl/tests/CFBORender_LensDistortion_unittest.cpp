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

// CFBORender lens distortion and pixel noise of RGB images.

#include <gtest/gtest.h>
#include <mrpt/img/camera_geometry.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/Scene.h>

#include <cmath>
#include <memory>
#include <optional>

#include "render_pixel_utils.h"

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
#endif

#if defined(RUN_OFFSCREEN_RENDER_TESTS)

using namespace mrpt::opengl::testing;
using mrpt::math::TPoint3D;

namespace
{
constexpr int W = 320;
constexpr int H = 240;

const RGB GRAY{128, 128, 128};

// Center of the front face of the red mark, in camera coordinates. Near a
// corner of the image, where distortion moves it clearly:
const TPoint3D MARK_CENTER{1.2, 0.84, 2.0};

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

mrpt::img::TCamera pinholeCamera()
{
  mrpt::img::TCamera c;
  c.ncols = W;
  c.nrows = H;
  c.fx(200);
  c.fy(200);
  c.cx(W / 2.0);
  c.cy(H / 2.0);
  return c;
}

mrpt::img::TCamera distortedCamera(double k1 = -0.3)
{
  auto c = pinholeCamera();
  c.distortion = mrpt::img::DistortionModel::plumb_bob;
  c.k1(k1);
  c.k2(0.05);
  c.p1(0.002);
  c.p2(-0.001);
  return c;
}

/** A small red square in front of a camera at the origin, looking along +Z,
 * on a gray background. */
mrpt::viz::Scene::Ptr markScene()
{
  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({0.5f, 0.5f, 0.5f});

  const double s = 0.04;
  const auto& c = MARK_CENTER;
  auto box = mrpt::viz::CBox::Create(
      TPoint3D(c.x - s, c.y - s, c.z), TPoint3D(c.x + s, c.y + s, c.z + 0.005));
  box->setColor_u8(255, 0, 0);
  box->enableLight(false);
  box->enableBoxBorder(false);
  scene->insert(box);

  auto& cam = scene->getViewport()->getCamera();
  cam.set6DOFMode(true);
  cam.setPose(mrpt::poses::CPose3D());
  cam.setProjectiveFromPinhole(pinholeCamera());
  return scene;
}

mrpt::img::CImage render(mrpt::opengl::CFBORender& r, const mrpt::viz::Scene& scene)
{
  mrpt::img::CImage frame;
  r.render_RGB(scene, frame);
  return frame;
}

std::optional<mrpt::img::TPixelCoordf> redCentroid(const mrpt::img::CImage& im)
{
  double su = 0;
  double sv = 0;
  size_t n = 0;
  for (int v = 0; v < static_cast<int>(im.getHeight()); v++)
  {
    for (int u = 0; u < static_cast<int>(im.getWidth()); u++)
    {
      const auto p = pixelRGB(im, u, v);
      if (p.r > p.g + 80 && p.r > p.b + 80)
      {
        su += u;
        sv += v;
        n++;
      }
    }
  }
  if (n == 0)
  {
    return {};
  }
  return mrpt::img::TPixelCoordf(
      static_cast<float>(su / static_cast<double>(n)),
      static_cast<float>(sv / static_cast<double>(n)));
}

mrpt::img::TPixelCoordf project(const mrpt::img::TCamera& cam)
{
  mrpt::img::TPixelCoordf px;
  mrpt::img::camera_geometry::projectPoint_with_distortion(MARK_CENTER, cam, px);
  return px;
}
}  // namespace

TEST(CFBORender_LensDistortion, MarkAppearsWhereTheDistortedModelProjectsIt)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No offscreen OpenGL rendering available";
  }
  const auto scene = markScene();

  const auto cPinhole = redCentroid(render(*r, *scene));
  ASSERT_TRUE(cPinhole.has_value());

  r->setLensDistortion(distortedCamera());
  const auto im = render(*r, *scene);
  EXPECT_EQ(im.getWidth(), static_cast<size_t>(W));
  EXPECT_EQ(im.getHeight(), static_cast<size_t>(H));
  const auto cDistorted = redCentroid(im);
  ASSERT_TRUE(cDistorted.has_value());

  const auto expectedPinhole = project(pinholeCamera());
  const auto expected = project(distortedCamera());

  // Both renders must be consistent with their projection models, with the
  // same offset between pixel indices and image coordinates:
  const auto offsetX = cPinhole->x - expectedPinhole.x;
  const auto offsetY = cPinhole->y - expectedPinhole.y;
  EXPECT_LE(std::abs(offsetX), 0.5);
  EXPECT_LE(std::abs(offsetY), 0.5);
  EXPECT_NEAR(cDistorted->x - expected.x, offsetX, 0.2);
  EXPECT_NEAR(cDistorted->y - expected.y, offsetY, 0.2);

  // ...and the test is only meaningful if the distortion moves it clearly:
  EXPECT_GT(std::hypot(expected.x - expectedPinhole.x, expected.y - expectedPinhole.y), 5.0);
}

TEST(CFBORender_LensDistortion, NoBlackBordersWithBarrelDistortion)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No offscreen OpenGL rendering available";
  }
  r->setLensDistortion(distortedCamera());
  const auto im = render(*r, *markScene());

  EXPECT_EQ(countColor(im, 0, 0, W, 1, GRAY, 10), W);
  EXPECT_EQ(countColor(im, 0, H - 1, W, H, GRAY, 10), W);
  EXPECT_EQ(countColor(im, 0, 0, 1, H, GRAY, 10), H);
  EXPECT_EQ(countColor(im, W - 1, 0, W, H, GRAY, 10), H);
}

TEST(CFBORender_LensDistortion, ClearingItRestoresThePinholeImage)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No offscreen OpenGL rendering available";
  }
  const auto scene = markScene();
  const auto before = render(*r, *scene);

  r->setLensDistortion(distortedCamera());
  ASSERT_TRUE(r->getLensDistortion().has_value());
  (void)render(*r, *scene);

  // A camera without distortion clears it:
  r->setLensDistortion(pinholeCamera());
  EXPECT_FALSE(r->getLensDistortion().has_value());
  const auto after = render(*r, *scene);

  int different = 0;
  for (int v = 0; v < H; v++)
  {
    for (int u = 0; u < W; u++)
    {
      if (!isNear(pixelRGB(before, u, v), pixelRGB(after, u, v), 0))
      {
        different++;
      }
    }
  }
  EXPECT_EQ(different, 0);
}

TEST(CFBORender_LensDistortion, InvalidUses)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No offscreen OpenGL rendering available";
  }

  // Too strong:
  EXPECT_THROW(r->setLensDistortion(distortedCamera(-2.0)), std::exception);

  // Wrong image size:
  auto c = distortedCamera();
  c.ncols = W / 2;
  EXPECT_THROW(r->setLensDistortion(c), std::exception);

  EXPECT_FALSE(r->getLensDistortion().has_value());

  // Depth images are not supported:
  r->setLensDistortion(distortedCamera());
  mrpt::img::CImage rgb;
  mrpt::math::CMatrixFloat depth;
  EXPECT_THROW(r->render_RGBD(*markScene(), rgb, depth), std::exception);
  EXPECT_THROW(r->render_depth(*markScene(), depth), std::exception);
}

TEST(CFBORender_RGBNoise, NoiseHasTheRequestedStatistics)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No offscreen OpenGL rendering available";
  }
  const auto scene = markScene();
  const auto clean = render(*r, *scene);

  const float noiseStd = 10.0f;
  r->setRGBNoise(noiseStd);
  EXPECT_EQ(r->getRGBNoise(), noiseStd);
  const auto noisy1 = render(*r, *scene);
  const auto noisy2 = render(*r, *scene);

  // Statistics over the whole (gray) background, all channels:
  double sum = 0;
  double sumSq = 0;
  size_t n = 0;
  int changedBetweenFrames = 0;
  for (int v = 0; v < H; v++)
  {
    for (int u = 0; u < W; u++)
    {
      const auto c = pixelRGB(clean, u, v);
      if (!isNear(c, GRAY, 2))
      {
        continue;
      }
      const auto p1 = pixelRGB(noisy1, u, v);
      const auto p2 = pixelRGB(noisy2, u, v);
      for (const int d : {p1.r - c.r, p1.g - c.g, p1.b - c.b})
      {
        sum += d;
        sumSq += d * d;
        n++;
      }
      if (!isNear(p1, p2, 0))
      {
        changedBetweenFrames++;
      }
    }
  }
  ASSERT_GT(n, 100000U);
  const double mean = sum / static_cast<double>(n);
  const double stdDev = std::sqrt(sumSq / static_cast<double>(n) - mean * mean);
  EXPECT_NEAR(mean, 0.0, 0.2);
  EXPECT_NEAR(stdDev, noiseStd, 0.3);

  // Noise changes in each frame:
  EXPECT_GT(changedBetweenFrames, W * H / 2);

  // ...but the sequence is reproducible:
  auto r2 = makeRenderer();
  ASSERT_TRUE(r2);
  r2->setRGBNoise(noiseStd);
  const auto again = render(*r2, *scene);
  int different = 0;
  for (int v = 0; v < H; v++)
  {
    for (int u = 0; u < W; u++)
    {
      if (!isNear(pixelRGB(noisy1, u, v), pixelRGB(again, u, v), 0))
      {
        different++;
      }
    }
  }
  EXPECT_EQ(different, 0);

  // Disabled again:
  r->setRGBNoise(0);
  const auto cleanAgain = render(*r, *scene);
  const auto bg = pixelRGB(clean, 0, 0);
  EXPECT_EQ(countColor(cleanAgain, 0, 0, W / 4, H / 4, bg, 0), W * H / 16);
}

TEST(CFBORender_RGBNoise, WorksTogetherWithLensDistortion)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No offscreen OpenGL rendering available";
  }
  r->setLensDistortion(distortedCamera());
  r->setRGBNoise(3.0f);
  const auto im = render(*r, *markScene());

  const auto c = redCentroid(im);
  ASSERT_TRUE(c.has_value());
  const auto expected = project(distortedCamera());
  EXPECT_NEAR(c->x, expected.x, 1.0);
  EXPECT_NEAR(c->y, expected.y, 1.0);

  // Noisy, but still close to the background color:
  EXPECT_LT(countColor(im, 0, 0, W / 4, H / 4, GRAY, 2), W * H / 16 / 2);
  EXPECT_GT(countColor(im, 0, 0, W / 4, H / 4, GRAY, 15), W * H / 16 * 99 / 100);
}

#endif  // RUN_OFFSCREEN_RENDER_TESTS
