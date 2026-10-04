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

/** Offscreen rendering tests of frustum culling (perspective and orthographic
 *  views), drawing order of transparent objects, lighting of scaled objects,
 *  orthographic depth, flipped projections, and textures used
 *  from several OpenGL contexts.
 */

#include <gtest/gtest.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CompiledScene.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/Scene.h>

#include <memory>

#include "render_pixel_utils.h"

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
// Only needed for direct GL calls; GLEW headers are not visible to tests on Windows.
#include <mrpt/opengl/opengl_api.h>
#endif

#if defined(RUN_OFFSCREEN_RENDER_TESTS)

using namespace mrpt::opengl::testing;
using mrpt::math::TPoint3D;
using mrpt::math::TPoint3Df;

namespace
{
constexpr int W = 200;
constexpr int H = 150;

const RGB RED{255, 0, 0};
const RGB GREEN{0, 255, 0};

/** Gives access to the frame buffer, to render a CompiledScene directly */
class TestRenderer : public mrpt::opengl::CFBORender
{
 public:
  TestRenderer() : CFBORender(W, H) {}
  void bindFrameBuffer() { m_fb.bind(); }
};

std::unique_ptr<TestRenderer> makeRenderer()
{
  try
  {
    return std::make_unique<TestRenderer>();
  }
  catch (const std::exception&)
  {
    return nullptr;
  }
}

mrpt::img::CImage render(mrpt::opengl::CFBORender& r, const mrpt::viz::Scene& scene)
{
  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);
  r.render_RGB(scene, frame);
  return frame;
}

/** Reads the current frame buffer (rows bottom-up) */
mrpt::img::CImage readFrameBuffer()
{
  mrpt::img::CImage im(W, H, mrpt::img::CH_RGB);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, im.ptrLine<uint8_t>(0));
  return im;
}

mrpt::viz::Scene::Ptr whiteScene()
{
  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  return scene;
}

void setOrbitCamera(
    mrpt::viz::CCamera& cam,
    float distance,
    float azimuthDeg,
    float elevationDeg,
    bool ortho = false)
{
  cam.setPointingAt(0, 0, 0);
  cam.setZoomDistance(distance);
  cam.setAzimuthDegrees(azimuthDeg);
  cam.setElevationDegrees(elevationDeg);
  cam.setProjectiveModel(!ortho);
}

mrpt::viz::CBox::Ptr unlitBox(const TPoint3D& a, const TPoint3D& b, uint8_t R, uint8_t G, uint8_t B)
{
  auto box = mrpt::viz::CBox::Create(a, b);
  box->setColor_u8(R, G, B);
  box->enableLight(false);
  box->enableBoxBorder(false);
  return box;
}

/** A single-sided quad (two triangles) with the given corners, in order */
mrpt::viz::CSetOfTriangles::Ptr quad(
    const TPoint3Df& p0, const TPoint3Df& p1, const TPoint3Df& p2, const TPoint3Df& p3)
{
  auto q = mrpt::viz::CSetOfTriangles::Create();
  q->insertTriangle(mrpt::viz::TTriangle(p0, p1, p2));
  q->insertTriangle(mrpt::viz::TTriangle(p0, p2, p3));
  return q;
}

size_t culledProxies(const mrpt::opengl::CompiledScene& cs)
{
  return cs.getViewport("main")->lastRenderStats().numProxiesCulled;
}
}  // namespace

TEST(OpenGLCulling, LargeObjectsAroundTheCameraAreDrawn)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  // A huge floor seen from close: all its corners are outside the view.
  auto scene = whiteScene();
  setOrbitCamera(scene->getViewport()->getCamera(), 1.0f, 0, 30);
  scene->insert(unlitBox({-100, -100, -0.1}, {100, 100, 0}, 0x00, 0xff, 0x00));

  const auto im = render(*r, *scene);
  EXPECT_GT(countColor(im, 0, 0, W, H, GREEN, 60), W * H / 4);
}

TEST(OpenGLCulling, ObjectsOutOfViewAreNotDrawnInPerspectiveAndOrthoViews)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  for (const bool ortho : {false, true})
  {
    auto scene = whiteScene();
    setOrbitCamera(scene->getViewport()->getCamera(), 10.0f, 0, 0, ortho);
    // Looking from +X towards the origin: one box in view, one far aside.
    scene->insert(unlitBox({-1, -1, -1}, {1, 1, 1}, 0xff, 0x00, 0x00));
    scene->insert(unlitBox({-1, 99, -1}, {1, 101, 1}, 0x00, 0xff, 0x00));

    mrpt::opengl::CompiledScene compiled;
    r->bindFrameBuffer();
    compiled.compile(*scene);
    compiled.render(W, H);
    const auto im = readFrameBuffer();

    EXPECT_GT(countColor(im, 0, 0, W, H, RED, 60), 100) << "ortho=" << ortho;
    EXPECT_EQ(countColor(im, 0, 0, W, H, GREEN, 60), 0) << "ortho=" << ortho;
    EXPECT_GE(culledProxies(compiled), 1U) << "ortho=" << ortho;
  }
}

TEST(OpenGLOrdering, TransparentObjectsBlendFromBackToFront)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  // Two translucent quads facing the camera (looking from +X), the near one
  // inserted first: drawn in that order, the far one would fail the depth test.
  auto scene = whiteScene();
  setOrbitCamera(scene->getViewport()->getCamera(), 10.0f, 0, 0);
  auto nearQuad = quad({1, -2, -2}, {1, 2, -2}, {1, 2, 2}, {1, -2, 2});
  nearQuad->setColor_u8(mrpt::img::TColor(0xff, 0x00, 0x00, 0x80));
  nearQuad->enableLight(false);
  auto farQuad = quad({-1, -2, -2}, {-1, 2, -2}, {-1, 2, 2}, {-1, -2, 2});
  farQuad->setColor_u8(mrpt::img::TColor(0x00, 0xff, 0x00, 0x80));
  farQuad->enableLight(false);
  scene->insert(nearQuad);
  scene->insert(farQuad);

  const auto c = pixelRGB(render(*r, *scene), W / 2, H / 2);
  // Red over (green over white) gives less blue than red over white alone:
  EXPECT_LT(c.b, 165) << "rgb=" << c.r << "," << c.g << "," << c.b;
  EXPECT_LT(c.r, 245) << "rgb=" << c.r << "," << c.g << "," << c.b;
}

TEST(OpenGLLighting, NonUniformlyScaledObjectsAreLitLikeTheirScaledGeometry)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  // A quad tilted 45 deg, stretched along X by a scale of 4, must be lit as
  // the same quad with the stretched coordinates (a 14 deg slope).
  auto renderQuad = [&](bool useScale)
  {
    auto scene = whiteScene();
    auto vp = scene->getViewport();
    setOrbitCamera(vp->getCamera(), 10.0f, -90, 90);
    vp->lightParameters().ambient = 0;
    vp->lightParameters().lights = {
        mrpt::viz::TLight::Directional({1, 0, 0}, {1, 1, 1}, 1.0f, 0.0f)};
    const float k = useScale ? 1.0f : 4.0f;
    auto q = quad(
        {-0.5f * k, -1, -0.5f}, {0.5f * k, -1, 0.5f}, {0.5f * k, 1, 0.5f}, {-0.5f * k, 1, -0.5f});
    q->setColor_u8(mrpt::img::TColor(0xff, 0xff, 0xff));
    if (useScale)
    {
      q->setScale(4, 1, 1);
    }
    scene->insert(q);
    return pixelRGB(render(*r, *scene), W / 2, H / 2);
  };
  const auto scaled = renderQuad(true);
  const auto baked = renderQuad(false);
  EXPECT_TRUE(isNear(scaled, baked, 12))
      << "scaled=" << scaled.r << "," << scaled.g << "," << scaled.b << " baked=" << baked.r << ","
      << baked.g << "," << baked.b;
}

TEST(OpenGLOrthographic, DepthImagesAreDistancesFromTheEye)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  auto scene = whiteScene();
  scene->insert(unlitBox({-50, -50, -1}, {50, 50, 0}, 0x80, 0x80, 0x80));
  for (const bool ortho : {false, true})
  {
    mrpt::viz::CCamera cam;
    setOrbitCamera(cam, 5.0f, -90, 90, ortho);
    r->setCamera(cam);
    mrpt::math::CMatrixFloat depth;
    r->render_depth(*scene, depth);
    EXPECT_NEAR(depth(H / 2, W / 2), 5.0f, 0.05f) << "ortho=" << ortho;
  }
}

TEST(OpenGLProjection, FlippedProjectionIsTheSameInEveryFrame)
{
  auto r = makeRenderer();
  if (!r)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  // A red box in the upper half of the view:
  auto scene = whiteScene();
  setOrbitCamera(scene->getViewport()->getCamera(), 10.0f, -90, 90);
  scene->insert(unlitBox({-1, 1, 0}, {1, 3, 1}, 0xff, 0x00, 0x00));

  mrpt::opengl::CompiledScene compiled;
  compiled.setAutoUpdate(false);
  r->bindFrameBuffer();
  compiled.compile(*scene);
  compiled.getViewport("main")->flipVerticalProjection(true);

  auto redRows = [](const mrpt::img::CImage& im)
  {
    double sum = 0;
    int n = 0;
    for (int y = 0; y < H; y++)
    {
      for (int x = 0; x < W; x++)
      {
        if (isNear(pixelRGB(im, x, y), RED, 60))
        {
          sum += y;
          n++;
        }
      }
    }
    return n ? sum / n : -1.0;
  };

  compiled.render(W, H);
  const double row1 = redRows(readFrameBuffer());
  compiled.render(W, H);
  const double row2 = redRows(readFrameBuffer());
  ASSERT_GE(row1, 0);
  EXPECT_NEAR(row1, row2, 0.5);
}

TEST(OpenGLTextures, SameImageInRenderersWithTheirOwnContexts)
{
  std::unique_ptr<TestRenderer> r1;
  std::unique_ptr<TestRenderer> r2;
  r1 = makeRenderer();
  r2 = makeRenderer();  // a second, independent EGL context
  if (!r1 || !r2)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  mrpt::img::CImage tex(8, 8, mrpt::img::CH_RGB);
  tex.filledRectangle({0, 0}, {7, 7}, mrpt::img::TColor(0x00, 0xff, 0x00));

  auto scene = whiteScene();
  scene->getViewport()->getCamera().setNoProjection();
  auto plane = mrpt::viz::CTexturedPlane::Create(-0.8f, 0.8f, -0.8f, 0.8f);
  plane->enableLighting(false);
  plane->assignImage(tex);
  scene->insert(plane);

  const auto c1 = pixelRGB(render(*r1, *scene), W / 2, H / 2);
  const auto c2 = pixelRGB(render(*r2, *scene), W / 2, H / 2);
  EXPECT_TRUE(isNear(c1, GREEN, 60)) << "rgb=" << c1.r << "," << c1.g << "," << c1.b;
  EXPECT_TRUE(isNear(c2, GREEN, 60)) << "rgb=" << c2.r << "," << c2.g << "," << c2.b;
}

TEST(OpenGLTextures, RenderersInTheSameContextShareTextures)
{
  auto r1 = makeRenderer();
  if (!r1)
  {
    GTEST_SKIP() << "No off-screen rendering on this device";
  }
  mrpt::img::CImage tex(8, 8, mrpt::img::CH_RGB);
  tex.filledRectangle({0, 0}, {7, 7}, mrpt::img::TColor(0x00, 0xff, 0x00));

  auto scene = whiteScene();
  scene->getViewport()->getCamera().setNoProjection();
  auto plane = mrpt::viz::CTexturedPlane::Create(-0.8f, 0.8f, -0.8f, 0.8f);
  plane->enableLighting(false);
  plane->assignImage(tex);
  scene->insert(plane);

  // Leaves the context of r1 current:
  render(*r1, *scene);

  // A second renderer in the same context (e.g. sensors rendered in a GUI):
  mrpt::opengl::CFBORender::Parameters p;
  p.width = W;
  p.height = H;
  p.create_EGL_context = false;
  mrpt::opengl::CFBORender r2(p);

  const auto countTextures = []()
  {
    size_t n = 0;
    for (GLuint name = 1; name < 4096; name++)
    {
      n += glIsTexture(name) == GL_TRUE ? 1 : 0;
    }
    return n;
  };
  const size_t before = countTextures();
  const auto c2 = pixelRGB(render(r2, *scene), W / 2, H / 2);
  EXPECT_EQ(countTextures(), before);
  EXPECT_TRUE(isNear(c2, GREEN, 60)) << "rgb=" << c2.r << "," << c2.g << "," << c2.b;
}

#endif  // RUN_OFFSCREEN_RENDER_TESTS
