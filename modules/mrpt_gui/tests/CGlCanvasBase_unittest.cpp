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

/** Unit tests for CGlCanvasBase through its headless implementation: the
 *  mouse/camera bookkeeping needs neither a window server nor a GL context.
 */

#include <gtest/gtest.h>
#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/Scene.h>

using mrpt::gui::CGlCanvasBaseHeadless;

TEST(CGlCanvasBase, default_state)
{
  CGlCanvasBaseHeadless c;

  EXPECT_FALSE(c.getUseCameraFromScene());
  EXPECT_NE(c.getOpenGLSceneRef(), nullptr);
  // No scene has been compiled yet:
  EXPECT_EQ(c.getShaderManager(), nullptr);

  int x = -1, y = -1;
  c.getLastMousePosition(x, y);
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 0);
}

TEST(CGlCanvasBase, last_mouse_position_is_tracked)
{
  CGlCanvasBaseHeadless c;

  c.updateLastPos(31, 47);
  int x = 0, y = 0;
  c.getLastMousePosition(x, y);
  EXPECT_EQ(x, 31);
  EXPECT_EQ(y, 47);

  c.updateLastPos(0, 0);
  c.getLastMousePosition(x, y);
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 0);
}

TEST(CGlCanvasBase, use_camera_from_scene_flag)
{
  CGlCanvasBaseHeadless c;
  c.setUseCameraFromScene(true);
  EXPECT_TRUE(c.getUseCameraFromScene());
  c.setUseCameraFromScene(false);
  EXPECT_FALSE(c.getUseCameraFromScene());
}

TEST(CGlCanvasBase, scene_reference_can_be_replaced)
{
  CGlCanvasBaseHeadless c;
  const auto original = c.getOpenGLSceneRef();
  ASSERT_NE(original, nullptr);

  auto scene = mrpt::viz::Scene::Create();
  scene->insert(mrpt::viz::CBox::Create());
  c.setOpenGLSceneRef(scene);

  EXPECT_EQ(c.getOpenGLSceneRef(), scene);
  EXPECT_NE(c.getOpenGLSceneRef(), original);
}

TEST(CGlCanvasBase, mouse_events_are_forwarded_to_the_camera_controller)
{
  CGlCanvasBaseHeadless c;
  auto& ctrl = c.orbitCameraController();
  ctrl.setAzimuthDegrees(0.0f);
  ctrl.setElevationDegrees(0.0f);

  using MB = mrpt::viz::COrbitCameraController::MouseButtons;
  using KM = mrpt::viz::COrbitCameraController::KeyModifiers;

  // Dragging with the left button must orbit the camera:
  ctrl.onMouseButton(100, 100, MB::ButtonLeft, true /*down*/);
  ctrl.onMouseMove(140, 120, MB::ButtonLeft, KM::ModNone);
  ctrl.onMouseButton(140, 120, MB::ButtonLeft, false /*down*/);

  EXPECT_NE(ctrl.getAzimuthDegrees(), 0.0f);

  // ...and the wheel must change the zoom distance:
  const float z0 = ctrl.getZoomDistance();
  ctrl.onScroll(+1.0f, KM::ModNone);
  EXPECT_LT(ctrl.getZoomDistance(), z0);
  ctrl.onScroll(-1.0f, KM::ModNone);
  EXPECT_GT(ctrl.getZoomDistance(), 0.0f);
}

TEST(CGlCanvasBase, orbit_camera_controller_is_exposed_and_mutable)
{
  CGlCanvasBaseHeadless c;

  auto& ctrl = c.orbitCameraController();
  ctrl.setAzimuthDegrees(45.0f);
  ctrl.setElevationDegrees(30.0f);
  ctrl.setZoomDistance(12.5f);
  ctrl.setCameraPointing(1.0f, 2.0f, 3.0f);

  const auto& constCtrl = static_cast<const CGlCanvasBaseHeadless&>(c).orbitCameraController();
  EXPECT_FLOAT_EQ(constCtrl.getAzimuthDegrees(), 45.0f);
  EXPECT_FLOAT_EQ(constCtrl.getElevationDegrees(), 30.0f);
  EXPECT_FLOAT_EQ(constCtrl.getZoomDistance(), 12.5f);
  EXPECT_FLOAT_EQ(constCtrl.getCameraPointingX(), 1.0f);
  EXPECT_FLOAT_EQ(constCtrl.getCameraPointingY(), 2.0f);
  EXPECT_FLOAT_EQ(constCtrl.getCameraPointingZ(), 3.0f);
}

TEST(CGlCanvasBase, resizeViewport_is_a_no_op_without_a_size)
{
  CGlCanvasBaseHeadless c;
  // -1 means "unknown size" and must return before touching the GL context,
  // which does not exist here:
  EXPECT_NO_THROW(c.resizeViewport(-1, -1));
  EXPECT_NO_THROW(c.resizeViewport(640, -1));
  EXPECT_NO_THROW(c.resizeViewport(-1, 480));
}
