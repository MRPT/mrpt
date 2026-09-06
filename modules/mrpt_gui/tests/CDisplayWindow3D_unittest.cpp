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

/** Unit tests for the 3D scene window. They need a window server and a GL
 *  implementation; CI provides a virtual display (`xvfb-run`) plus Mesa's
 *  software rasterizer, and the tests self-skip when neither is available.
 *
 *  Rendered frames are only checked for plausibility (a frame was grabbed,
 *  with the right size), never against reference pixels: on a virtual display
 *  with no compositor the window is never mapped, so reading its pixels back
 *  yields a uniformly black image even though rendering did happen. Pixel
 *  comparisons against reference images belong in the offscreen (EGL/FBO)
 *  tests of mrpt_opengl instead.
 */

#include <gtest/gtest.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/img/CImage.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/stock_objects.h>

#include <thread>

#include "gui_test_common.h"

namespace
{
constexpr unsigned int WIN_W = 320;
constexpr unsigned int WIN_H = 240;

/** Fills the window's scene with a few objects and forces a redraw. */
void populateAndRender(mrpt::gui::CDisplayWindow3D& win)
{
  {
    mrpt::viz::Scene::Ptr& scene = win.get3DSceneAndLock();
    scene->insert(mrpt::viz::CGridPlaneXY::Create(-5, 5, -5, 5, 0, 1));
    scene->insert(mrpt::viz::stock_objects::CornerXYZ());
    auto box = mrpt::viz::CBox::Create();
    box->setBoxCorners({-1, -1, 0}, {1, 1, 2});
    scene->insert(box);
    win.unlockAccess3DScene();
  }
  win.repaint();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

}  // namespace

TEST(CDisplayWindow3D, create_and_populate_a_scene)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindow3D win("unit test 3D", WIN_W, WIN_H);
  ASSERT_TRUE(win.isOpen());

  populateAndRender(win);

  {
    mrpt::viz::Scene::Ptr& scene = win.get3DSceneAndLock();
    EXPECT_EQ(scene->getViewport()->size(), 3U);
    win.unlockAccess3DScene();
  }
  EXPECT_NE(win.getDefaultViewport(), nullptr);
  EXPECT_TRUE(win.isOpen());
}

TEST(CDisplayWindow3D, scene_locker_helper)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindow3D win("unit test locker", WIN_W, WIN_H);
  {
    mrpt::viz::Scene::Ptr scene;
    mrpt::gui::CDisplayWindow3DLocker lck(win, scene);
    ASSERT_NE(scene, nullptr);
    scene->insert(mrpt::viz::CSphere::Create(1.0f));
  }
  {
    // The overload that only locks:
    mrpt::gui::CDisplayWindow3DLocker lck(win);
  }
  win.repaint();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(win.isOpen());
}

TEST(CDisplayWindow3D, camera_parameters_round_trip)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindow3D win("unit test camera", WIN_W, WIN_H);

  win.setCameraAzimuthDeg(35.0f);
  win.setCameraElevationDeg(20.0f);
  win.setCameraZoom(7.5f);
  win.setCameraPointingToPoint(1.0f, 2.0f, 3.0f);
  win.setProjectiveModel(true);
  win.setFOV(50.0f);
  win.setMinRange(0.05f);
  win.setMaxRange(500.0f);

  EXPECT_FLOAT_EQ(win.getCameraAzimuthDeg(), 35.0f);
  EXPECT_FLOAT_EQ(win.getCameraElevationDeg(), 20.0f);
  EXPECT_FLOAT_EQ(win.getCameraZoom(), 7.5f);
  EXPECT_TRUE(win.isCameraProjective());
  EXPECT_FLOAT_EQ(win.getFOV(), 50.0f);

  float x = 0, y = 0, z = 0;
  win.getCameraPointingToPoint(x, y, z);
  EXPECT_FLOAT_EQ(x, 1.0f);
  EXPECT_FLOAT_EQ(y, 2.0f);
  EXPECT_FLOAT_EQ(z, 3.0f);

  win.setProjectiveModel(false);
  EXPECT_FALSE(win.isCameraProjective());

  win.useCameraFromScene(true);
  win.useCameraFromScene(false);
}

TEST(CDisplayWindow3D, text_messages)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindow3D win("unit test text", WIN_W, WIN_H);

  win.addTextMessage(0.01, 0.01, "hello", 0 /*unique_index*/);
  EXPECT_TRUE(win.updateTextMessage(0, "hello again"));
  EXPECT_FALSE(win.updateTextMessage(99, "no such index"));

  win.repaint();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  win.clearTextMessages();
  EXPECT_TRUE(win.isOpen());
}

TEST(CDisplayWindow3D, capture_the_rendered_frame)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindow3D win("unit test capture", WIN_W, WIN_H);
  populateAndRender(win);

  EXPECT_FALSE(win.isCapturingImgs());
  win.captureImagesStart();
  EXPECT_TRUE(win.isCapturingImgs());

  // Render a few frames so that at least one gets grabbed:
  mrpt::img::CImage frame;
  bool got = false;
  for (int i = 0; i < 20 && !got; i++)
  {
    win.repaint();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    got = win.getLastWindowImage(frame);
  }

  win.captureImagesStop();
  EXPECT_FALSE(win.isCapturingImgs());

  if (!got)
  {
    GTEST_SKIP() << "No GL frame could be grabbed (no usable GL "
                    "implementation in this environment).";
  }

  // Only the client area is grabbed, which under a window manager is smaller
  // than the requested outer size, so exact dimensions cannot be asserted:
  EXPECT_GT(frame.getWidth(), 0U);
  EXPECT_GT(frame.getHeight(), 0U);
  EXPECT_LE(frame.getWidth(), WIN_W);
  EXPECT_LE(frame.getHeight(), WIN_H);
  EXPECT_TRUE(frame.isColor());

  EXPECT_GT(win.getRenderingFPS(), 0.0);
}

TEST(CDisplayWindow3D, grab_frames_to_disk)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindow3D win("unit test grab", WIN_W, WIN_H);
  populateAndRender(win);

  const std::string prefix = mrpt::system::getTempFileName();  // unique, in the system temp dir
  win.grabImagesStart(prefix);
  for (int i = 0; i < 5; i++)
  {
    win.repaint();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  win.grabImagesStop();

  // Whether any file was actually written depends on the GL implementation,
  // so only the name generation is asserted here:
  const std::string next = win.grabImageGetNextFile();
  EXPECT_TRUE(next.empty() || next.find(prefix) != std::string::npos);
}

TEST(CDisplayWindow3D, image_view_mode)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindow3D win("unit test image view", WIN_W, WIN_H);

  mrpt::img::CImage img(WIN_W, WIN_H, mrpt::img::CH_RGB);
  img.filledRectangle(
      mrpt::img::TPixelCoord(0, 0),
      mrpt::img::TPixelCoord(static_cast<int>(WIN_W) - 1, static_cast<int>(WIN_H) - 1),
      mrpt::img::TColor(200, 30, 30));

  win.setImageView(img);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // ...and the move overload:
  mrpt::img::CImage img2 = img.makeDeepCopy();
  win.setImageView(std::move(img2));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_TRUE(win.isOpen());
}

TEST(CDisplayWindow3D, window_geometry_and_title)
{
  SKIP_IF_NO_GUI();

  auto win = mrpt::gui::CDisplayWindow3D::Create("unit test 3D geometry", WIN_W, WIN_H);
  win->setWindowTitle("renamed 3D");
  win->resize(400, 300);
  win->setPos(30, 40);
  win->setCursorCross(true);
  win->setCursorCross(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_NO_THROW((void)win->getLastMousePosition());
  EXPECT_NO_THROW((void)win->getLastMousePositionRay());
  EXPECT_TRUE(win->isOpen());
}
