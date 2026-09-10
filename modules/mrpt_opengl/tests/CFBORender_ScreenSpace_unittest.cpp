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

/** Tests for the two "2D overlay" rendering modes, asserting geometric
 *  invariants on the rendered pixels instead of comparing against a reference
 *  image: a golden image captured while one of these was broken keeps passing.
 */

#include <gtest/gtest.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/Scene.h>

#include <iostream>
#include <optional>

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
#endif

namespace
{
struct PixelBox
{
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  [[nodiscard]] int width() const { return x1 - x0 + 1; }
  [[nodiscard]] int height() const { return y1 - y0 + 1; }
};

/** Bounding box of all pixels darker than `threshold` in any channel. */
std::optional<PixelBox> darkPixelsBoundingBox(const mrpt::img::CImage& im, uint8_t threshold = 128)
{
  std::optional<PixelBox> bb;

  const int imH = static_cast<int>(im.getHeight());
  const int imW = static_cast<int>(im.getWidth());

  for (int y = 0; y < imH; y++)
  {
    for (int x = 0; x < imW; x++)
    {
      const uint8_t r = im.at<uint8_t>(x, y, 2);
      const uint8_t g = im.at<uint8_t>(x, y, 1);
      const uint8_t b = im.at<uint8_t>(x, y, 0);
      if (r > threshold && g > threshold && b > threshold)
      {
        continue;  // background
      }

      if (!bb.has_value())
      {
        bb = PixelBox{x, y, x, y};
      }
      else
      {
        bb->x0 = std::min(bb->x0, x);
        bb->y0 = std::min(bb->y0, y);
        bb->x1 = std::max(bb->x1, x);
        bb->y1 = std::max(bb->y1, y);
      }
    }
  }
  return bb;
}

/** Horizontal extent of the non-background pixels in one image row. */
std::optional<std::pair<int, int>> rowExtent(const mrpt::img::CImage& im, int row)
{
  std::optional<std::pair<int, int>> ext;
  const int imW = static_cast<int>(im.getWidth());
  for (int x = 0; x < imW; x++)
  {
    const uint8_t r = im.at<uint8_t>(x, row, 2);
    const uint8_t g = im.at<uint8_t>(x, row, 1);
    const uint8_t b = im.at<uint8_t>(x, row, 0);
    if (r > 128 && g > 128 && b > 128)
    {
      continue;
    }
    if (!ext.has_value())
    {
      ext = {x, x};
    }
    else
    {
      ext->second = x;
    }
  }
  return ext;
}

/** A CCamera object inserted into the scene must override the viewport's own
 * camera. With setNoProjection() that means drawing directly in clip
 * coordinates, which is what 2D overlays (e.g. RawLogViewer's timeline) rely
 * on: the result must be a flat rectangle, not a perspective view of one.
 */
void test_noProjectionCameraFromScene()
{
  const int W = 400;
  const int H = 200;

  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});

  {
    auto glCam = mrpt::viz::CCamera::Create();
    glCam->setNoProjection();
    scene->insert(glCam);
  }

  // Spans x in [-0.9, 0.9], y in [-0.8, 0.8] of the clip-coords cube:
  auto box = mrpt::viz::CBox::Create(
      mrpt::math::TPoint3D(-0.9, -0.8, -0.01), mrpt::math::TPoint3D(0.9, 0.8, 0.01), false);
  box->setColor_u8(0xff, 0x00, 0x00, 0xff);
  scene->insert(box);

  mrpt::opengl::CFBORender renderer(W, H);
  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);
  renderer.render_RGB(*scene, frame);

  // Expected mapping of clip x=[-0.9, 0.9] onto pixel columns:
  const int expectedX0 = static_cast<int>((1.0 - 0.9) * 0.5 * W);
  const int expectedX1 = static_cast<int>((1.0 + 0.9) * 0.5 * W) - 1;

  const auto bb = darkPixelsBoundingBox(frame);
  ASSERT_TRUE(bb.has_value()) << "Nothing was rendered";

  EXPECT_NEAR(bb->x0, expectedX0, 2);
  EXPECT_NEAR(bb->x1, expectedX1, 2);
  EXPECT_NEAR(bb->y0, static_cast<int>((1.0 - 0.8) * 0.5 * H), 2);
  EXPECT_NEAR(bb->y1, static_cast<int>((1.0 + 0.8) * 0.5 * H) - 1, 2);

  // A perspective view of the same box would taper: require the very same
  // horizontal extent near the top, the middle and the bottom of the box.
  const auto top = rowExtent(frame, static_cast<int>(0.3 * H));
  const auto mid = rowExtent(frame, static_cast<int>(0.5 * H));
  const auto bot = rowExtent(frame, static_cast<int>(0.7 * H));

  ASSERT_TRUE(top.has_value() && mid.has_value() && bot.has_value());
  EXPECT_EQ(top->first, mid->first);
  EXPECT_EQ(top->second, mid->second);
  EXPECT_EQ(bot->first, mid->first);
  EXPECT_EQ(bot->second, mid->second);
}

/** Renders one CText of the given font height, with the camera at the given
 * distance, and returns the bounding box of the glyphs in pixels. */
std::optional<PixelBox> renderOneLabel(int fontHeight, float camDistance)
{
  const int W = 320;
  const int H = 240;

  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});

  auto txt = mrpt::viz::CText::Create("MRPT");
  txt->setFont("mono", fontHeight);
  txt->setColor_u8(0x00, 0x00, 0x00, 0xff);
  txt->setLocation(0, 0, 0);
  scene->insert(txt);

  mrpt::viz::CCamera camera;
  camera.setProjectiveFOVdeg(60.0f);
  camera.setPointingAt(0, 0, 0);
  camera.setAzimuthDegrees(0);
  camera.setElevationDegrees(0);
  camera.setZoomDistance(camDistance);

  mrpt::opengl::CFBORender renderer(W, H);
  renderer.setCamera(camera);

  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);
  renderer.render_RGB(*scene, frame);

  return darkPixelsBoundingBox(frame);
}

/** CText is a 2D label: its size on screen is set by setFont() in pixels and
 * must not depend on how far the camera is.
 */
void test_text2DFixedPixelSize()
{
  const int fontHeight = 20;

  const auto bbNear = renderOneLabel(fontHeight, 5.0f);
  const auto bbFar = renderOneLabel(fontHeight, 25.0f);

  ASSERT_TRUE(bbNear.has_value()) << "No text was rendered";
  ASSERT_TRUE(bbFar.has_value()) << "No text was rendered";

  // Same size regardless of camera distance:
  EXPECT_NEAR(bbNear->width(), bbFar->width(), 1);
  EXPECT_NEAR(bbNear->height(), bbFar->height(), 1);

  // ...and no larger than the requested font height (a single text line):
  EXPECT_LE(bbNear->height(), fontHeight);
  EXPECT_GE(bbNear->height(), fontHeight / 4);

  // Doubling the font height doubles the rendered size:
  const auto twice = renderOneLabel(2 * fontHeight, 5.0f);
  ASSERT_TRUE(twice.has_value());
  EXPECT_NEAR(twice->height(), 2 * bbNear->height(), 2);
  EXPECT_NEAR(twice->width(), 2 * bbNear->width(), 3);
}

}  // namespace

#if defined(RUN_OFFSCREEN_RENDER_TESTS)
TEST(OpenGL, noProjectionCameraFromScene)
#else
TEST(OpenGL, DISABLED_noProjectionCameraFromScene)
#endif
{
  try
  {
    test_noProjectionCameraFromScene();
  }
  catch (const std::exception& e)
  {
    std::cerr << "***** WARNING ****: Ignoring exception in test, likely due to limited "
                 "rendering capabilities on this device (?):\n"
              << e.what() << "\n";
  }
}

#if defined(RUN_OFFSCREEN_RENDER_TESTS)
TEST(OpenGL, text2DFixedPixelSize)
#else
TEST(OpenGL, DISABLED_text2DFixedPixelSize)
#endif
{
  try
  {
    test_text2DFixedPixelSize();
  }
  catch (const std::exception& e)
  {
    std::cerr << "***** WARNING ****: Ignoring exception in test, likely due to limited "
                 "rendering capabilities on this device (?):\n"
              << e.what() << "\n";
  }
}
