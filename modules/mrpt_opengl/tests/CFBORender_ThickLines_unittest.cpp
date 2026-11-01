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

/** Tests for lines wider than one pixel (WIREFRAME_THICK shader), asserting
 *  invariants on the rendered pixels: their width in pixels, colors, and
 *  behavior with perspective cameras.
 */

#include <gtest/gtest.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSimpleLine.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/stock_objects.h>

#include <functional>
#include <iostream>
#include <mutex>
#include <shared_mutex>

#include "render_pixel_utils.h"

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
#endif

namespace
{
using mrpt::opengl::testing::countColor;
using mrpt::opengl::testing::isNear;
using mrpt::opengl::testing::pixelRGB;
using mrpt::opengl::testing::RGB;

constexpr int W = 320;
constexpr int H = 200;

const RGB WHITE{255, 255, 255};
const RGB BLACK{0, 0, 0};

/** A scene with a white background, and a camera drawing straight in clip
 * coordinates (x,y in [-1,1] map to the whole image). */
mrpt::viz::Scene::Ptr makeClipSpaceScene()
{
  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  auto glCam = mrpt::viz::CCamera::Create();
  glCam->setNoProjection();
  scene->insert(glCam);
  return scene;
}

mrpt::img::CImage render(const mrpt::viz::Scene& scene)
{
  mrpt::opengl::CFBORender renderer(W, H);
  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);
  renderer.render_RGB(scene, frame);
  return frame;
}

bool isBackground(const mrpt::img::CImage& im, int x, int y)
{
  return isNear(pixelRGB(im, x, y), WHITE, 10);
}

/** Number of non-background pixels in image column `x`. */
int columnCoverage(const mrpt::img::CImage& im, int x)
{
  int n = 0;
  for (int y = 0; y < H; y++)
  {
    n += isBackground(im, x, y) ? 0 : 1;
  }
  return n;
}

/** Number of non-background pixels in image row `y`. */
int rowCoverage(const mrpt::img::CImage& im, int y)
{
  int n = 0;
  for (int x = 0; x < W; x++)
  {
    n += isBackground(im, x, y) ? 0 : 1;
  }
  return n;
}

mrpt::viz::CSimpleLine::Ptr blackLine(
    float x0, float y0, float z0, float x1, float y1, float z1, float width, bool aa = false)
{
  auto line = mrpt::viz::CSimpleLine::Create(x0, y0, z0, x1, y1, z1, width, aa);
  line->setColor_u8(0, 0, 0, 0xff);
  return line;
}

/** Aliased lines are as many pixels wide as their width (rounded, like
 * glLineWidth()), both horizontal and vertical ones. */
void test_widthInPixels()
{
  for (const float width : {1.0f, 1.3f, 2.0f, 3.0f, 5.0f, 8.0f, 12.0f})
  {
    const int expected = static_cast<int>(std::round(width));

    auto scene = makeClipSpaceScene();
    scene->insert(blackLine(-0.8f, 0.1f, 0, 0.8f, 0.1f, 0, width));    // horizontal
    scene->insert(blackLine(-0.5f, -0.9f, 0, -0.5f, 0.9f, 0, width));  // vertical
    const auto im = render(*scene);

    // Column through the horizontal line only, row through the vertical one:
    const int col = W * 3 / 4;
    const int row = H * 3 / 4;
    EXPECT_NEAR(columnCoverage(im, col), expected, 1) << "width=" << width;
    EXPECT_NEAR(rowCoverage(im, row), expected, 1) << "width=" << width;
  }
}

/** Diagonal lines keep their width perpendicular to the line direction. */
void test_diagonalWidth()
{
  const float width = 6.0f;
  // A 45 degree line in pixels (clip x and y scaled by the aspect ratio):
  const float k = static_cast<float>(H) / static_cast<float>(W);
  auto scene = makeClipSpaceScene();
  scene->insert(blackLine(-0.8f * k, -0.8f, 0, 0.8f * k, 0.8f, 0, width));
  const auto im = render(*scene);

  // A column crosses the line over width*sqrt(2) pixels:
  EXPECT_NEAR(columnCoverage(im, W / 2 + 10), width * std::sqrt(2.0f), 1.5);
}

/** Lines are as wide in pixels near and far from a perspective camera (unlike
 * geometry with a width in meters). */
void test_constantWidthWithPerspective()
{
  const float width = 6.0f;

  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  // Two horizontal lines (along +Y world, i.e. image horizontal), at 2 and 20
  // m in front of the camera, which looks along +X:
  scene->insert(blackLine(2, -0.5f, 0.3f, 2, 0.5f, 0.3f, width));
  scene->insert(blackLine(20, -5.0f, -3.0f, 20, 5.0f, -3.0f, width));

  mrpt::opengl::CFBORender renderer(W, H);
  mrpt::viz::CCamera camera;
  camera.setProjectiveFOVdeg(90.0f);
  camera.set6DOFMode(true);
  camera.setPose(mrpt::poses::CPose3D::FromYawPitchRoll(
      mrpt::DEG2RAD(-90.0), mrpt::DEG2RAD(0.0), mrpt::DEG2RAD(-90.0)));
  renderer.setCamera(camera);
  scene->getViewport()->setViewportClipDistances(0.1f, 100.0f);

  mrpt::img::CImage im(W, H, mrpt::img::CH_RGB);
  renderer.render_RGB(*scene, im);

  // Both lines cross the central column: two runs of `width` pixels.
  std::vector<int> runs;
  int run = 0;
  for (int y = 0; y < H; y++)
  {
    if (!isBackground(im, W / 2, y))
    {
      run++;
    }
    else if (run > 0)
    {
      runs.push_back(run);
      run = 0;
    }
  }
  ASSERT_EQ(runs.size(), 2u);
  EXPECT_NEAR(runs[0], width, 1);
  EXPECT_NEAR(runs[1], width, 1);
}

/** A perspective camera at the origin looking along +X (image right is -Y,
 * image up is +Z). */
mrpt::viz::CCamera cameraAlongX()
{
  mrpt::viz::CCamera camera;
  camera.setProjectiveFOVdeg(90.0f);
  camera.set6DOFMode(true);
  camera.setPose(mrpt::poses::CPose3D::FromYawPitchRoll(
      mrpt::DEG2RAD(-90.0), mrpt::DEG2RAD(0.0), mrpt::DEG2RAD(-90.0)));
  return camera;
}

/** Runs of non-background pixels along image column `x`, top to bottom. */
std::vector<int> columnRuns(const mrpt::img::CImage& im, int x, const RGB& color)
{
  std::vector<int> runs;
  int run = 0;
  for (int y = 0; y < H; y++)
  {
    if (isNear(pixelRGB(im, x, y), color, 60))
    {
      run++;
    }
    else if (run > 0)
    {
      runs.push_back(run);
      run = 0;
    }
  }
  if (run > 0)
  {
    runs.push_back(run);
  }
  return runs;
}

/** A line lying on a solid floor, seen from 35 degrees above, is drawn whole
 * (the quad faces the camera, so without a depth offset the floor hides
 * half of it), but it is still hidden by an object in front of it. */
void test_lineOnSurface()
{
  const float width = 8.0f;
  const float floorZ = -4.0f * std::tan(mrpt::DEG2RAD(35.0f));

  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  scene->getViewport()->setViewportClipDistances(0.1f, 100.0f);

  auto floor = mrpt::viz::CBox::Create(
      mrpt::math::TPoint3D(0, -20, floorZ - 0.1), mrpt::math::TPoint3D(40, 20, floorZ));
  floor->setColor_u8(0x80, 0x80, 0x80);
  scene->insert(floor);

  // Line on the floor, across the view at 4 m:
  auto line = blackLine(4, -10, floorZ, 4, 10, floorZ, width);
  line->setColor_u8(0xff, 0x00, 0x00);
  scene->insert(line);

  // An occluder in front of the center of the line:
  auto occluder = mrpt::viz::CBox::Create(
      mrpt::math::TPoint3D(2.0, -0.5, floorZ), mrpt::math::TPoint3D(2.4, 0.5, -0.5));
  occluder->setColor_u8(0x00, 0x00, 0xff);
  scene->insert(occluder);

  mrpt::opengl::CFBORender renderer(W, H);
  renderer.setCamera(cameraAlongX());
  mrpt::img::CImage im(W, H, mrpt::img::CH_RGB);
  renderer.render_RGB(*scene, im);

  const RGB RED{255, 0, 0};
  for (const int col : {W / 8, W * 7 / 8})
  {
    const auto runs = columnRuns(im, col, RED);
    ASSERT_EQ(runs.size(), 1u) << "col=" << col;
    EXPECT_NEAR(runs[0], width, 1) << "col=" << col;
  }
  EXPECT_TRUE(columnRuns(im, W / 2, RED).empty());
}

/** A segment with one end behind the camera is clipped at the near plane,
 * instead of projecting through w=0 into a huge or flipped quad. */
void test_crossingNearPlane()
{
  const float width = 4.0f;

  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  // From 5 m behind the camera to 10 m ahead of it, slightly below its axis:
  scene->insert(blackLine(-5, 0, -0.5f, 10, 0, -0.5f, width));

  mrpt::opengl::CFBORender renderer(W, H);
  mrpt::viz::CCamera camera;
  camera.setProjectiveFOVdeg(90.0f);
  camera.set6DOFMode(true);
  camera.setPose(mrpt::poses::CPose3D::FromYawPitchRoll(
      mrpt::DEG2RAD(-90.0), mrpt::DEG2RAD(0.0), mrpt::DEG2RAD(-90.0)));
  renderer.setCamera(camera);
  scene->getViewport()->setViewportClipDistances(0.1f, 100.0f);

  mrpt::img::CImage im(W, H, mrpt::img::CH_RGB);
  renderer.render_RGB(*scene, im);

  // A vertical line in the image, from the bottom edge up to the vanishing
  // point at the image center, `width` pixels wide all along:
  const int dark = countColor(im, 0, 0, W, H, BLACK, 60);
  EXPECT_GT(dark, (H / 2 - 10) * (width - 1));
  EXPECT_LT(dark, (H / 2 + 10) * (width + 1));
  EXPECT_NEAR(rowCoverage(im, H - 5), width, 1);
  EXPECT_NEAR(rowCoverage(im, H * 3 / 4), width, 1);
  EXPECT_EQ(rowCoverage(im, H / 4), 0);  // nothing above the horizon
}

/** Lines with one color per vertex: red at the start of each segment, blue at
 * its end. */
class GradientLines : public mrpt::viz::CSetOfLines
{
 public:
  void updateBuffers() const override
  {
    mrpt::viz::CSetOfLines::updateBuffers();
    using P = mrpt::viz::VisualObjectParams_Lines;
    std::unique_lock<std::shared_mutex> lck(P::m_linesMtx.data);
    auto& colors = P::m_color_buffer_data;
    colors.resize(P::m_vertex_buffer_data.size());
    for (size_t i = 0; i < colors.size(); i++)
    {
      colors[i] = (i % 2) == 0 ? mrpt::img::TColor(0xff, 0, 0) : mrpt::img::TColor(0, 0, 0xff);
    }
  }
};

/** Per-vertex colors are interpolated along the segment, and objects without
 * per-vertex colors use their own color. */
void test_colors()
{
  auto scene = makeClipSpaceScene();

  auto gradient = std::make_shared<GradientLines>();
  gradient->appendLine(-0.8, 0.5, 0, 0.8, 0.5, 0);
  gradient->setLineWidth(6.0f);
  scene->insert(gradient);

  auto green = blackLine(-0.8f, -0.5f, 0, 0.8f, -0.5f, 0, 6.0f);
  green->setColor_u8(0x00, 0xff, 0x00, 0xff);
  scene->insert(green);

  const auto im = render(*scene);

  const int yTop = H / 4;  // clip y=+0.5 (image rows grow downwards)
  const int yBottom = H * 3 / 4;

  // Gradient: mostly red near the start, mostly blue near the end, a mix in
  // the middle.
  const auto start = pixelRGB(im, static_cast<int>(W * 0.12), yTop);
  const auto mid = pixelRGB(im, W / 2, yTop);
  const auto end = pixelRGB(im, static_cast<int>(W * 0.88), yTop);
  EXPECT_GT(start.r, 200);
  EXPECT_LT(start.b, 80);
  EXPECT_GT(end.b, 200);
  EXPECT_LT(end.r, 80);
  EXPECT_GT(mid.r, 60);
  EXPECT_GT(mid.b, 60);
  EXPECT_LT(mid.g, 30);

  // Object color:
  EXPECT_TRUE(isNear(pixelRGB(im, W / 4, yBottom), {0, 255, 0}));
  EXPECT_TRUE(isNear(pixelRGB(im, W * 3 / 4, yBottom), {0, 255, 0}));

  // Two lines 6 pixels wide, and nothing else, in a column:
  EXPECT_NEAR(columnCoverage(im, W / 2), 12, 2);
}

/** Changing the width of a line already rendered switches between the thin
 * and thick paths. */
void test_widthChange()
{
  auto scene = makeClipSpaceScene();
  auto line = blackLine(-0.8f, 0, 0, 0.8f, 0, 0, 1.0f);
  scene->insert(line);

  mrpt::opengl::CFBORender renderer(W, H);
  mrpt::img::CImage im(W, H, mrpt::img::CH_RGB);
  for (const float width : {1.0f, 7.0f, 3.0f, 1.0f})
  {
    line->setLineWidth(width);
    renderer.render_RGB(*scene, im);
    EXPECT_NEAR(columnCoverage(im, W / 3), width, 1) << "width=" << width;
  }
}

/** Anti-aliased lines have partially covered pixels at their edges, and
 * about the same total coverage as the aliased ones. */
void test_antiAliasing()
{
  const float width = 4.0f;
  auto sceneAA = makeClipSpaceScene();
  // A slightly tilted line, so edges cross pixels at different offsets:
  sceneAA->insert(blackLine(-0.8f, -0.1f, 0, 0.8f, 0.1f, 0, width, true /*aa*/));
  const auto im = render(*sceneAA);

  int partial = 0;
  int full = 0;
  for (int y = 0; y < H; y++)
  {
    const auto c = pixelRGB(im, W / 2, y);
    if (isNear(c, BLACK, 30))
    {
      full++;
    }
    else if (!isNear(c, WHITE, 10))
    {
      partial++;
    }
  }
  EXPECT_GE(partial, 1);
  EXPECT_GE(full, 2);
  EXPECT_LE(full + partial, static_cast<int>(width) + 3);
}

/** CornerXYZSimple() is made of lines of the given width again. */
void test_cornerXYZSimple()
{
  auto scene = makeClipSpaceScene();
  auto corner = mrpt::viz::stock_objects::CornerXYZSimple(0.8f, 5.0f);
  scene->insert(corner);
  const auto im = render(*scene);

  // X axis (red) along the image row through the origin, Y (green) up the
  // central column; Z points to the camera, so it is a 5x5 dot at most.
  EXPECT_NEAR(columnCoverage(im, W * 3 / 4), 5, 1);
  EXPECT_NEAR(rowCoverage(im, H / 4), 5, 1);
  EXPECT_TRUE(isNear(pixelRGB(im, W * 3 / 4, H / 2), {255, 0, 0}));
  EXPECT_TRUE(isNear(pixelRGB(im, W / 2, H / 4), {0, 255, 0}));
}

void runIgnoringRenderingErrors(const std::function<void()>& f)
{
  try
  {
    f();
  }
  catch (const std::exception& e)
  {
    std::cerr << "***** WARNING ****: Ignoring exception in test, likely due to limited "
                 "rendering capabilities on this device (?):\n"
              << e.what() << "\n";
  }
}

}  // namespace

#if defined(RUN_OFFSCREEN_RENDER_TESTS)
#define THICK_LINES_TEST(NAME) TEST(OpenGL, ThickLines_##NAME)
#else
#define THICK_LINES_TEST(NAME) TEST(OpenGL, DISABLED_ThickLines_##NAME)
#endif

THICK_LINES_TEST(widthInPixels) { runIgnoringRenderingErrors(test_widthInPixels); }
THICK_LINES_TEST(diagonalWidth) { runIgnoringRenderingErrors(test_diagonalWidth); }
THICK_LINES_TEST(constantWidthWithPerspective)
{
  runIgnoringRenderingErrors(test_constantWidthWithPerspective);
}
THICK_LINES_TEST(lineOnSurface) { runIgnoringRenderingErrors(test_lineOnSurface); }
THICK_LINES_TEST(crossingNearPlane) { runIgnoringRenderingErrors(test_crossingNearPlane); }
THICK_LINES_TEST(colors) { runIgnoringRenderingErrors(test_colors); }
THICK_LINES_TEST(widthChange) { runIgnoringRenderingErrors(test_widthChange); }
THICK_LINES_TEST(antiAliasing) { runIgnoringRenderingErrors(test_antiAliasing); }
THICK_LINES_TEST(cornerXYZSimple) { runIgnoringRenderingErrors(test_cornerXYZSimple); }
