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

/** Offscreen rendering tests of what happens to an already compiled scene when
 *  it changes (objects moved, recolored, hidden, inserted or removed), plus
 *  textures, sky boxes, extra lights and fog. The renderer is kept across
 *  frames, so the incremental update paths are the ones exercised.
 */

#include <gtest/gtest.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSkyBox.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/Scene.h>

#include <memory>

#include "render_pixel_utils.h"

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
#endif

#if defined(RUN_OFFSCREEN_RENDER_TESTS)

using namespace mrpt::opengl::testing;

namespace
{
constexpr int W = 200;
constexpr int H = 150;

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

const RGB RED{255, 0, 0};
const RGB BLUE{0, 0, 255};
const RGB GREEN{0, 255, 0};

/** A scene looking at the origin from a distance, over a white background. */
mrpt::viz::Scene::Ptr emptyScene(float distance = 10.0f)
{
  auto scene = mrpt::viz::Scene::Create();
  auto vp = scene->getViewport();
  vp->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});
  vp->getCamera().setPointingAt(0, 0, 0);
  vp->getCamera().setZoomDistance(distance);
  vp->getCamera().setAzimuthDegrees(0);
  vp->getCamera().setElevationDegrees(0);
  return scene;
}

mrpt::viz::CBox::Ptr coloredBox(uint8_t r, uint8_t g, uint8_t b)
{
  auto box =
      mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-1, -1, -1), mrpt::math::TPoint3D(1, 1, 1));
  box->setColor_u8(r, g, b, 0xff);
  return box;
}

/** Mean column of the pixels of that color, or -1 if there are none. */
double meanColumn(const mrpt::img::CImage& im, const RGB& c, int tol = 110)
{
  double sum = 0;
  int n = 0;
  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      if (isNear(pixelRGB(im, x, y), c, tol))
      {
        sum += x;
        n++;
      }
    }
  }
  return n ? sum / n : -1;
}

/** Pixels of an object with lighting are darker than its color, hence the
 * generous default tolerance. */
int count(const mrpt::img::CImage& im, const RGB& c, int tol = 110)
{
  return countColor(im, 0, 0, W, H, c, tol);
}

/** Number of pixels that are noticeably different between two frames. */
int differentPixels(const mrpt::img::CImage& a, const mrpt::img::CImage& b)
{
  int n = 0;
  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      n += isNear(pixelRGB(a, x, y), pixelRGB(b, x, y), 30) ? 0 : 1;
    }
  }
  return n;
}
}  // namespace

TEST(OpenGLSceneUpdates, ObjectChangesShowUpInTheNextFrame)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  auto scene = emptyScene();
  auto box = coloredBox(255, 0, 0);
  scene->insert(box);

  EXPECT_GT(count(render(*renderer, *scene), RED), 500);

  // Color:
  box->setColor_u8(0, 0, 255, 255);
  {
    const auto f = render(*renderer, *scene);
    EXPECT_LT(count(f, RED), 20);
    EXPECT_GT(count(f, BLUE), 500);
  }

  // Pose: moving the object along the screen's horizontal axis:
  const double x0 = meanColumn(render(*renderer, *scene), BLUE);
  box->setLocation(0, 3, 0);
  const double x1 = meanColumn(render(*renderer, *scene), BLUE);
  EXPECT_GT(std::abs(x1 - x0), 15.0);

  // Visibility:
  box->setVisibility(false);
  EXPECT_LT(count(render(*renderer, *scene), BLUE), 20);
  box->setVisibility(true);
  EXPECT_GT(count(render(*renderer, *scene), BLUE), 500);

  // Removal:
  scene->getViewport()->removeObject(box);
  EXPECT_LT(count(render(*renderer, *scene), BLUE), 20);
}

TEST(OpenGLSceneUpdates, ObjectsAndGroupsInsertedAfterTheFirstFrame)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  auto scene = emptyScene(14.0f);
  EXPECT_EQ(count(render(*renderer, *scene), RED), 0);  // nothing yet

  // A new group, with a red box at its origin:
  auto group = mrpt::viz::CSetOfObjects::Create();
  group->insert(coloredBox(255, 0, 0));
  scene->insert(group);
  const auto f1 = render(*renderer, *scene);
  EXPECT_GT(count(f1, RED), 500);
  const double xRed = meanColumn(f1, RED);

  // A child added to a group that was already compiled:
  auto blue = coloredBox(0, 0, 255);
  blue->setLocation(0, -4, 0);
  group->insert(blue);
  const auto f2 = render(*renderer, *scene);
  EXPECT_GT(count(f2, BLUE), 500);

  // A nested group:
  auto nested = mrpt::viz::CSetOfObjects::Create();
  auto green = coloredBox(0, 255, 0);
  green->setLocation(0, 4, 0);
  nested->insert(green);
  group->insert(nested);
  EXPECT_GT(count(render(*renderer, *scene), GREEN), 500);

  // Moving the group moves everything in it:
  group->setLocation(0, 2, 0);
  const auto f3 = render(*renderer, *scene);
  EXPECT_GT(std::abs(meanColumn(f3, RED) - xRed), 8.0);

  // Hiding the group hides everything in it:
  group->setVisibility(false);
  const auto f4 = render(*renderer, *scene);
  EXPECT_LT(count(f4, RED) + count(f4, BLUE) + count(f4, GREEN), 40);
  group->setVisibility(true);

  // Objects that disappear from the scene are removed from the renderer:
  scene->getViewport()->removeObject(group);
  const auto f5 = render(*renderer, *scene);
  EXPECT_LT(count(f5, RED) + count(f5, BLUE) + count(f5, GREEN), 40);
}

TEST(OpenGLSceneUpdates, ShownNamesFollowTheObject)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  auto scene = emptyScene(20.0f);
  // (the labels are white by default)
  scene->getViewport()->setCustomBackgroundColor({0.0f, 0.0f, 0.0f});
  auto box = coloredBox(255, 0, 0);
  box->setName("OBJECT NAME");
  scene->insert(box);

  const auto plain = render(*renderer, *scene);
  box->enableShowName(true);
  render(*renderer, *scene);

  // Renaming the object changes its label, which needs the scene to notice
  // the change:
  box->setName("FIRST NAME");
  const auto first = render(*renderer, *scene);
  EXPECT_GT(differentPixels(plain, first), 10) << "no label was drawn";

  box->setName("A MUCH SECOND NAME");
  const auto second = render(*renderer, *scene);
  EXPECT_GT(differentPixels(first, second), 10) << "the label did not follow the new name";

  // Without the label it is the same picture as at the beginning:
  box->enableShowName(false);
  const auto plain2 = render(*renderer, *scene);
  EXPECT_LT(differentPixels(plain, plain2), 5);
}

namespace
{
mrpt::img::CImage solidImage(int w, int h, mrpt::img::TImageChannels ch, mrpt::img::TColor c)
{
  mrpt::img::CImage im(static_cast<unsigned>(w), static_cast<unsigned>(h), ch);
  im.filledRectangle({0, 0}, {w - 1, h - 1}, c);
  return im;
}

/** Renders a plane facing the camera, and returns the color at the center. */
RGB renderPlaneWithTexture(
    mrpt::opengl::CFBORender& renderer,
    mrpt::viz::Scene& scene,
    const mrpt::viz::CTexturedPlane& plane)
{
  (void)plane;
  const auto f = render(renderer, scene);
  return pixelRGB(f, W / 2, H / 2);
}
}  // namespace

TEST(OpenGLSceneUpdates, TexturesOfDifferentFormatsAndUpdates)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  auto scene = emptyScene(6.0f);
  auto plane = mrpt::viz::CTexturedPlane::Create(-2, 2, -2, 2);
  plane->enableLighting(false);
  scene->insert(plane);
  // Looking at the plane from above:
  scene->getViewport()->getCamera().setElevationDegrees(90);
  scene->getViewport()->getCamera().setAzimuthDegrees(0);

  // A non power-of-two color image:
  plane->assignImage(solidImage(33, 17, mrpt::img::CH_RGB, mrpt::img::TColor(0xff, 0x00, 0x00)));
  EXPECT_TRUE(isNear(renderPlaneWithTexture(*renderer, *scene, *plane), RED, 60));

  // Replacing the image updates the texture:
  plane->assignImage(solidImage(33, 17, mrpt::img::CH_RGB, mrpt::img::TColor(0x00, 0x00, 0xff)));
  EXPECT_TRUE(isNear(renderPlaneWithTexture(*renderer, *scene, *plane), BLUE, 60));

  // ...also with a different size:
  plane->assignImage(solidImage(64, 64, mrpt::img::CH_RGB, mrpt::img::TColor(0x00, 0xff, 0x00)));
  EXPECT_TRUE(isNear(renderPlaneWithTexture(*renderer, *scene, *plane), GREEN, 60));

  // A grayscale image gives a gray plane:
  plane->assignImage(solidImage(16, 16, mrpt::img::CH_GRAY, mrpt::img::TColor(200, 200, 200)));
  EXPECT_TRUE(isNear(renderPlaneWithTexture(*renderer, *scene, *plane), RGB{200, 200, 200}, 60));

  // A color image with a separate alpha channel: fully transparent shows the
  // background through it.
  plane->assignImage(
      solidImage(16, 16, mrpt::img::CH_RGB, mrpt::img::TColor(0xff, 0x00, 0x00)),
      solidImage(16, 16, mrpt::img::CH_GRAY, mrpt::img::TColor(0, 0, 0)));
  EXPECT_TRUE(isNear(renderPlaneWithTexture(*renderer, *scene, *plane), RGB{255, 255, 255}, 60));
}

TEST(OpenGLSceneUpdates, SkyBoxShowsADifferentFaceInEachDirection)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  using mrpt::viz::CUBE_TEXTURE_FACE;
  auto scene = emptyScene(3.0f);

  auto sky = mrpt::viz::CSkyBox::Create();
  const std::array<std::pair<CUBE_TEXTURE_FACE, mrpt::img::TColor>, 6> faces = {
      {
       {CUBE_TEXTURE_FACE::LEFT, mrpt::img::TColor(255, 0, 0)},
       {CUBE_TEXTURE_FACE::RIGHT, mrpt::img::TColor(0, 255, 0)},
       {CUBE_TEXTURE_FACE::TOP, mrpt::img::TColor(0, 0, 255)},
       {CUBE_TEXTURE_FACE::BOTTOM, mrpt::img::TColor(255, 255, 0)},
       {CUBE_TEXTURE_FACE::FRONT, mrpt::img::TColor(255, 0, 255)},
       {CUBE_TEXTURE_FACE::BACK, mrpt::img::TColor(0, 255, 255)},
       }
  };
  for (const auto& [face, color] : faces)
  {
    sky->assignImage(face, solidImage(8, 8, mrpt::img::CH_RGB, color));
  }
  scene->insert(sky);

  // Looking horizontally in four directions gives four different colors,
  // each one of the faces:
  std::vector<RGB> seen;
  for (const float az : {0.0f, 90.0f, 180.0f, 270.0f})
  {
    scene->getViewport()->getCamera().setAzimuthDegrees(az);
    const auto f = render(*renderer, *scene);
    seen.push_back(pixelRGB(f, W / 2, H / 2));
  }
  int matched = 0;
  for (const auto& s : seen)
  {
    for (const auto& [face, color] : faces)
    {
      if (isNear(s, RGB{color.R, color.G, color.B}, 60))
      {
        matched++;
        break;
      }
    }
  }
  EXPECT_EQ(matched, 4) << "the sky box faces are not rendered";
  for (size_t i = 0; i < seen.size(); i++)
  {
    for (size_t j = i + 1; j < seen.size(); j++)
    {
      EXPECT_FALSE(isNear(seen[i], seen[j], 60)) << "directions " << i << " and " << j;
    }
  }
}

TEST(OpenGLSceneUpdates, LightsAndFog)
{
  auto renderer = makeRenderer();
  if (!renderer)
  {
    GTEST_SKIP() << "No offscreen rendering available";
  }
  auto scene = emptyScene(9.0f);
  auto sphere = mrpt::viz::CSphere::Create(2.0f);
  sphere->setColor_u8(0xff, 0xff, 0xff, 0xff);
  scene->insert(sphere);
  auto vp = scene->getViewport();
  vp->setCustomBackgroundColor({0.0f, 0.0f, 0.0f});

  // Only a dim ambient light and no directional one: nearly dark
  auto& lp = vp->lightParameters();
  lp.ambient = 0.05f;
  lp.lights.clear();
  auto meanBrightness = [&]()
  {
    const auto f = render(*renderer, *scene);
    double sum = 0;
    for (int y = 0; y < H; y++)
    {
      for (int x = 0; x < W; x++)
      {
        const auto p = pixelRGB(f, x, y);
        sum += p.r + p.g + p.b;
      }
    }
    return sum / (3.0 * W * H);
  };
  const double dark = meanBrightness();

  // A point light near the camera side of the sphere brightens it:
  lp.lights.push_back(mrpt::viz::TLight::PointLight({6, 0, 0}));
  const double withPoint = meanBrightness();
  EXPECT_GT(withPoint, dark + 3.0);

  // A spot light aimed at it as well:
  lp.lights.clear();
  lp.lights.push_back(mrpt::viz::TLight::SpotLight({8, 0, 0}, {-1, 0, 0}));
  const double withSpot = meanBrightness();
  EXPECT_GT(withSpot, dark + 3.0);

  // Directional light again, then fog that hides everything: the sphere is
  // 7 m away from the camera and the fog is opaque from 1 m.
  lp.lights.clear();
  lp.lights.push_back(mrpt::viz::TLight::Directional({-1, 0, 0}));
  lp.fog_enabled = true;
  lp.fog_color = mrpt::img::TColorf(0.0f, 0.5f, 0.0f);
  lp.fog_near = 0.5f;
  lp.fog_far = 1.0f;
  const auto foggy = render(*renderer, *scene);
  // (the fog color goes through the gamma correction of the pipeline)
  auto isGreenish = [](const RGB& p) { return p.g > 100 && p.r < 40 && p.b < 40; };
  EXPECT_TRUE(isGreenish(pixelRGB(foggy, W / 2, H / 2))) << "the sphere was not covered by the fog";

  // Exponential fog too:
  lp.fog_mode = 1;
  lp.fog_density = 5.0f;
  const auto foggy2 = render(*renderer, *scene);
  EXPECT_TRUE(isGreenish(pixelRGB(foggy2, W / 2, H / 2)));

  lp.fog_enabled = false;
  const auto clear = render(*renderer, *scene);
  EXPECT_FALSE(isGreenish(pixelRGB(clear, W / 2, H / 2)));
}

#endif  // RUN_OFFSCREEN_RENDER_TESTS
