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

#include <gtest/gtest.h>
#include <mrpt/viz/COrbitCameraController.h>

using namespace mrpt::viz;

TEST(COrbitCameraController, DefaultsAndSimpleSetters)
{
  COrbitCameraController ctrl;
  EXPECT_FLOAT_EQ(ctrl.getCameraPointingX(), 0.f);
  EXPECT_FLOAT_EQ(ctrl.getCameraPointingY(), 0.f);
  EXPECT_FLOAT_EQ(ctrl.getCameraPointingZ(), 0.f);

  ctrl.setCameraPointing(1.f, 2.f, 3.f);
  EXPECT_FLOAT_EQ(ctrl.getCameraPointingX(), 1.f);
  EXPECT_FLOAT_EQ(ctrl.getCameraPointingY(), 2.f);
  EXPECT_FLOAT_EQ(ctrl.getCameraPointingZ(), 3.f);

  ctrl.setCameraPointing(mrpt::math::TPoint3Df(4.f, 5.f, 6.f));
  EXPECT_FLOAT_EQ(ctrl.getCameraPointingX(), 4.f);

  ctrl.setCameraPointing(mrpt::math::TPoint3D(7.0, 8.0, 9.0));
  EXPECT_FLOAT_EQ(ctrl.getCameraPointingZ(), 9.f);

  ctrl.setAzimuthDegrees(45.f);
  EXPECT_FLOAT_EQ(ctrl.getAzimuthDegrees(), 45.f);

  ctrl.setElevationDegrees(30.f);
  EXPECT_FLOAT_EQ(ctrl.getElevationDegrees(), 30.f);

  ctrl.setRollDegrees(10.f);
  EXPECT_FLOAT_EQ(ctrl.getRollDegrees(), 10.f);

  EXPECT_TRUE(ctrl.isProjectiveModel());
  ctrl.setProjectiveModel(false);
  EXPECT_FALSE(ctrl.isProjectiveModel());

  ctrl.setFOVdeg(60.f);
  EXPECT_FLOAT_EQ(ctrl.getFOVdeg(), 60.f);
}

TEST(COrbitCameraController, ElevationClamping)
{
  COrbitCameraController ctrl;
  ctrl.setElevationDegrees(200.f);
  EXPECT_LE(ctrl.getElevationDegrees(), 89.9f);
  ctrl.setElevationDegrees(-200.f);
  EXPECT_GE(ctrl.getElevationDegrees(), -89.9f);
}

TEST(COrbitCameraController, ZoomLimitsAndClamping)
{
  COrbitCameraController ctrl;
  const auto [minZ, maxZ] = ctrl.getZoomLimits();
  EXPECT_FLOAT_EQ(minZ, 0.01f);
  EXPECT_FLOAT_EQ(maxZ, 3200.f);

  ctrl.setZoomLimits(1.f, 10.f);
  ctrl.setZoomDistance(100.f);
  EXPECT_FLOAT_EQ(ctrl.getZoomDistance(), 10.f);
  ctrl.setZoomDistance(0.001f);
  EXPECT_FLOAT_EQ(ctrl.getZoomDistance(), 1.f);
  ctrl.setZoomDistance(5.f);
  EXPECT_FLOAT_EQ(ctrl.getZoomDistance(), 5.f);
}

TEST(COrbitCameraController, MouseOrbitDrag)
{
  COrbitCameraController ctrl;
  const float az0 = ctrl.getAzimuthDegrees();
  const float el0 = ctrl.getElevationDegrees();

  ctrl.onMouseButton(100, 100, COrbitCameraController::ButtonLeft, true);
  ctrl.onMouseMove(110, 90, COrbitCameraController::ButtonLeft, COrbitCameraController::ModNone);

  // dx=10 -> azimuth decreases; dy=-10 -> elevation decreases
  EXPECT_LT(ctrl.getAzimuthDegrees(), az0);
  EXPECT_LT(ctrl.getElevationDegrees(), el0);

  ctrl.onMouseButton(110, 90, COrbitCameraController::ButtonLeft, false);
}

TEST(COrbitCameraController, MouseZoomDragWithShift)
{
  COrbitCameraController ctrl;
  ctrl.setZoomDistance(15.f);
  ctrl.onMouseButton(0, 0, COrbitCameraController::ButtonLeft, true);
  ctrl.onMouseMove(0, 20, COrbitCameraController::ButtonLeft, COrbitCameraController::ModShift);
  // dy=20 -> zoomDistance *= 1 + 0.2
  EXPECT_GT(ctrl.getZoomDistance(), 15.f);
}

TEST(COrbitCameraController, MouseRotateDragWithControl)
{
  COrbitCameraController ctrl;
  const float az0 = ctrl.getAzimuthDegrees();
  ctrl.onMouseButton(0, 0, COrbitCameraController::ButtonLeft, true);
  ctrl.onMouseMove(15, 0, COrbitCameraController::ButtonLeft, COrbitCameraController::ModControl);
  EXPECT_NE(ctrl.getAzimuthDegrees(), az0);
}

TEST(COrbitCameraController, MouseRollDragWithAlt)
{
  COrbitCameraController ctrl;
  const float roll0 = ctrl.getRollDegrees();
  ctrl.onMouseButton(0, 0, COrbitCameraController::ButtonLeft, true);
  ctrl.onMouseMove(10, 10, COrbitCameraController::ButtonLeft, COrbitCameraController::ModAlt);
  EXPECT_NE(ctrl.getRollDegrees(), roll0);
}

TEST(COrbitCameraController, MousePanWithRightOrMiddleButton)
{
  COrbitCameraController ctrl1;
  ctrl1.onMouseButton(0, 0, COrbitCameraController::ButtonRight, true);
  ctrl1.onMouseMove(10, 5, COrbitCameraController::ButtonRight, COrbitCameraController::ModNone);
  EXPECT_TRUE(ctrl1.getCameraPointingX() != 0.f || ctrl1.getCameraPointingY() != 0.f);

  COrbitCameraController ctrl2;
  ctrl2.onMouseButton(0, 0, COrbitCameraController::ButtonMiddle, true);
  ctrl2.onMouseMove(-10, 5, COrbitCameraController::ButtonMiddle, COrbitCameraController::ModNone);
  EXPECT_TRUE(ctrl2.getCameraPointingX() != 0.f || ctrl2.getCameraPointingY() != 0.f);
}

TEST(COrbitCameraController, MouseMoveNoButtonsIsNoOp)
{
  COrbitCameraController ctrl;
  const float az0 = ctrl.getAzimuthDegrees();
  ctrl.onMouseMove(50, 50, COrbitCameraController::ButtonNone, COrbitCameraController::ModNone);
  EXPECT_FLOAT_EQ(ctrl.getAzimuthDegrees(), az0);
}

TEST(COrbitCameraController, MouseGlitchFilterResetsLargeJump)
{
  COrbitCameraController ctrl;
  // No button-down event registered first: a big coordinate jump should be
  // absorbed by the glitch filter (first delta ~0) instead of producing a
  // huge orbit change.
  ctrl.onMouseMove(1000, 1000, COrbitCameraController::ButtonLeft, COrbitCameraController::ModNone);
  const float az1 = ctrl.getAzimuthDegrees();
  // A small follow-up move should now produce a small, bounded change:
  ctrl.onMouseMove(1005, 1000, COrbitCameraController::ButtonLeft, COrbitCameraController::ModNone);
  EXPECT_NEAR(ctrl.getAzimuthDegrees(), az1, 5.0f);
}

TEST(COrbitCameraController, ScrollZoom)
{
  COrbitCameraController ctrl;
  ctrl.setZoomDistance(20.f);
  ctrl.onScroll(1.0f, COrbitCameraController::ModNone);  // zoom in
  EXPECT_LT(ctrl.getZoomDistance(), 20.f);

  ctrl.setZoomDistance(20.f);
  ctrl.onScroll(-1.0f, COrbitCameraController::ModNone);  // zoom out
  EXPECT_GT(ctrl.getZoomDistance(), 20.f);
}

TEST(COrbitCameraController, ScrollWithShiftPansZ)
{
  COrbitCameraController ctrl;
  const float z0 = ctrl.getCameraPointingZ();
  ctrl.onScroll(1.0f, COrbitCameraController::ModShift);
  EXPECT_NE(ctrl.getCameraPointingZ(), z0);
}

TEST(COrbitCameraController, ApplyToAndSetFromRoundTrip)
{
  COrbitCameraController ctrl;
  ctrl.setCameraPointing(1.f, 2.f, 3.f);
  ctrl.setZoomDistance(12.f);
  ctrl.setAzimuthDegrees(20.f);
  ctrl.setElevationDegrees(15.f);
  ctrl.setRollDegrees(5.f);
  ctrl.setProjectiveModel(false);
  ctrl.setFOVdeg(70.f);

  mrpt::viz::CCamera cam;
  ctrl.applyTo(cam);

  EXPECT_FLOAT_EQ(cam.getPointingAtX(), 1.f);
  EXPECT_FLOAT_EQ(cam.getPointingAtY(), 2.f);
  EXPECT_FLOAT_EQ(cam.getPointingAtZ(), 3.f);
  EXPECT_FLOAT_EQ(cam.getZoomDistance(), 12.f);
  EXPECT_FLOAT_EQ(cam.getAzimuthDegrees(), 20.f);
  EXPECT_FLOAT_EQ(cam.getElevationDegrees(), 15.f);
  EXPECT_FLOAT_EQ(cam.getRollDegrees(), 5.f);
  EXPECT_FALSE(cam.isProjective());
  EXPECT_FLOAT_EQ(cam.getProjectiveFOVdeg(), 70.f);

  COrbitCameraController ctrl2;
  ctrl2.setFrom(cam);
  EXPECT_FLOAT_EQ(ctrl2.getCameraPointingX(), 1.f);
  EXPECT_FLOAT_EQ(ctrl2.getZoomDistance(), 12.f);
  EXPECT_FLOAT_EQ(ctrl2.getAzimuthDegrees(), 20.f);
  EXPECT_FLOAT_EQ(ctrl2.getElevationDegrees(), 15.f);
  EXPECT_FLOAT_EQ(ctrl2.getRollDegrees(), 5.f);
  EXPECT_FALSE(ctrl2.isProjectiveModel());
  EXPECT_FLOAT_EQ(ctrl2.getFOVdeg(), 70.f);
}

TEST(COrbitCameraController, CopyAndAssignSemantics)
{
  COrbitCameraController ctrl;
  ctrl.setAzimuthDegrees(33.f);

  COrbitCameraController copy(ctrl);
  EXPECT_FLOAT_EQ(copy.getAzimuthDegrees(), 33.f);

  COrbitCameraController assigned;
  assigned = ctrl;
  EXPECT_FLOAT_EQ(assigned.getAzimuthDegrees(), 33.f);
}
