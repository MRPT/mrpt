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

/** Textures uploaded from the same image are shared between their users: one
 *  user going away must not delete the texture the others still draw with.
 */

#include <gtest/gtest.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/Texture.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/Scene.h>

#include <algorithm>
#include <memory>

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
// Only needed for direct GL queries; GLEW headers are not visible to tests on Windows.
#include <mrpt/opengl/opengl_api.h>
#endif

namespace
{
constexpr int W = 200;
constexpr int H = 100;

/** Red plane covering [x0,x1] x [-0.8,0.8] in clip coordinates. */
mrpt::viz::CTexturedPlane::Ptr makeRedPlane(const mrpt::img::CImage& texture, float x0, float x1)
{
  auto plane = mrpt::viz::CTexturedPlane::Create(x0, x1, -0.8f, 0.8f);
  plane->enableLighting(false);
  plane->assignImage(texture);  // shallow copy: all planes share the same pixel buffer
  return plane;
}

/** Neither the white background nor black (what sampling a deleted texture gives). */
bool isRed(const mrpt::img::CImage& im, int x, int y)
{
  const int c0 = im.at<uint8_t>(x, y, 0);
  const int c1 = im.at<uint8_t>(x, y, 1);
  const int c2 = im.at<uint8_t>(x, y, 2);
  return std::max({c0, c1, c2}) > 150 && std::min({c0, c1, c2}) < 100;
}

mrpt::img::CImage makeRedImage()
{
  mrpt::img::CImage img(8, 8, mrpt::img::CH_RGB);
  img.filledRectangle({0, 0}, {7, 7}, mrpt::img::TColor(0xff, 0x00, 0x00));
  return img;
}

/** Creates the renderer, or returns nullptr if this device cannot render
 * off-screen. Only this setup step may skip a test: any later exception must
 * fail it. */
std::unique_ptr<mrpt::opengl::CFBORender> createRendererOrNull(std::string& whyNot)
{
  try
  {
    return std::make_unique<mrpt::opengl::CFBORender>(W, H);
  }
  catch (const std::exception& e)
  {
    whyNot = e.what();
    return nullptr;
  }
}

/** Two planes share one image, so they share one GPU texture. Removing one of
 * them from the scene must leave the other one textured. */
void test_removeOneOfTwoPlanesSharingATexture(mrpt::opengl::CFBORender& renderer)
{
  const mrpt::img::CImage texture = makeRedImage();

  auto scene = mrpt::viz::Scene::Create();
  scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  {
    auto glCam = mrpt::viz::CCamera::Create();
    glCam->setNoProjection();
    scene->insert(glCam);
  }

  auto leftPlane = makeRedPlane(texture, -0.9f, -0.1f);
  auto rightPlane = makeRedPlane(texture, 0.1f, 0.9f);
  scene->insert(leftPlane);
  scene->insert(rightPlane);

  mrpt::img::CImage frame(W, H, mrpt::img::CH_RGB);

  const int leftX = W / 4;
  const int rightX = 3 * W / 4;
  const int midY = H / 2;

  renderer.render_RGB(*scene, frame);
  ASSERT_TRUE(isRed(frame, leftX, midY)) << "Left plane was not rendered textured";
  ASSERT_TRUE(isRed(frame, rightX, midY)) << "Right plane was not rendered textured";

  scene->removeObject(leftPlane);
  leftPlane.reset();

  renderer.render_RGB(*scene, frame);
  EXPECT_FALSE(isRed(frame, leftX, midY)) << "Removed plane is still rendered";
  EXPECT_TRUE(isRed(frame, rightX, midY)) << "Remaining plane lost its shared texture";
}

/** Assigning the same image twice to one Texture must leave a single
 * reference, so unloading it deletes the GPU texture. */
void test_reassignSameImageReleasesOnce()
{
#if defined(RUN_OFFSCREEN_RENDER_TESTS)  // direct GL calls
  const mrpt::img::CImage img = makeRedImage();

  mrpt::opengl::texture_name_t name = 0;
  {
    mrpt::opengl::Texture tex;
    tex.assignImage2D(img, mrpt::opengl::Texture::Options());
    tex.assignImage2D(img, mrpt::opengl::Texture::Options());
    name = tex.textureNameID();
    ASSERT_EQ(glIsTexture(name), GL_TRUE);
  }
  EXPECT_EQ(glIsTexture(name), GL_FALSE) << "Texture leaked after its only holder was destroyed";
#endif
}

}  // namespace

#if defined(RUN_OFFSCREEN_RENDER_TESTS)
TEST(OpenGL, removeOneOfTwoPlanesSharingATexture)
#else
TEST(OpenGL, DISABLED_removeOneOfTwoPlanesSharingATexture)
#endif
{
  std::string whyNot;
  auto renderer = createRendererOrNull(whyNot);
  if (!renderer)
  {
    GTEST_SKIP() << "No off-screen rendering on this device: " << whyNot;
  }
  test_removeOneOfTwoPlanesSharingATexture(*renderer);
}

#if defined(RUN_OFFSCREEN_RENDER_TESTS)
TEST(OpenGL, reassignSameImageReleasesOnce)
#else
TEST(OpenGL, DISABLED_reassignSameImageReleasesOnce)
#endif
{
  // The renderer is only needed for its current OpenGL context:
  std::string whyNot;
  auto renderer = createRendererOrNull(whyNot);
  if (!renderer)
  {
    GTEST_SKIP() << "No off-screen rendering on this device: " << whyNot;
  }
  test_reassignSameImageReleasesOnce();
}
