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

#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/COrbitCameraController.h>

#include <algorithm>
#include <cmath>

using namespace mrpt::viz;

// ----------------------------------------------------------------
COrbitCameraController::COrbitCameraController() = default;

// ----------------------------------------------------------------
// Setters
// ----------------------------------------------------------------
void COrbitCameraController::setCameraPointing(float x, float y, float z)
{
  m_params.pointingX = x;
  m_params.pointingY = y;
  m_params.pointingZ = z;
}

void COrbitCameraController::setZoomDistance(float d)
{
  m_params.zoomDistance = d;
  clampZoom();
}

void COrbitCameraController::setAzimuthDegrees(float deg) { m_params.azimuthDeg = deg; }
void COrbitCameraController::setElevationDegrees(float deg)
{
  m_params.elevationDeg = clampElevation(deg);
}
void COrbitCameraController::setZoomLimits(float minZoom, float maxZoom)
{
  m_minZoom = minZoom;
  m_maxZoom = maxZoom;
}

// ----------------------------------------------------------------
// Mouse events
// ----------------------------------------------------------------
void COrbitCameraController::onMouseButton(int x, int y, uint8_t /*button*/, bool down)
{
  if (down)
  {
    m_clickX = x;
    m_clickY = y;
    m_lastX = x;
    m_lastY = y;
  }
  m_buttonDown = down;
}

void COrbitCameraController::onMouseMove(int x, int y, uint8_t buttons, uint8_t modifiers)
{
  // Guard against large coordinate jumps that occur when a button-down event
  // was missed (e.g. the click landed on an overlay widget and the cursor then
  // moved back onto the canvas while still held). Reset the last-position
  // anchor so the first delta after the glitch is zero rather than huge.
  mouseGlitchFilter(x, y, m_lastX, m_lastY);

  const int dx = x - m_lastX;
  const int dy = y - m_lastY;
  m_lastX = x;
  m_lastY = y;

  if (buttons == ButtonNone)
  {
    return;
  }

  const bool leftDown = (buttons & ButtonLeft) != 0;
  const bool rightDown = (buttons & ButtonRight) != 0;
  const bool middleDown = (buttons & ButtonMiddle) != 0;

  if (leftDown)
  {
    if ((modifiers & ModShift) != 0)
    {
      applyZoomDelta(dx, dy);
    }
    else if ((modifiers & ModControl) != 0)
    {
      applyRotate(dx, dy);
    }
    else if ((modifiers & ModAlt) != 0)
    {
      applyRoll(dx, dy);
    }
    else
    {
      applyOrbit(dx, dy);
    }
  }
  else if (rightDown || middleDown)
  {
    applyPan(dx, dy);
  }
}

void COrbitCameraController::onScroll(float delta, uint8_t modifiers)
{
  if ((modifiers & ModShift) != 0)
  {
    // Vertical pan along world Z instead of zoom
    m_params.pointingZ += delta * m_params.zoomDistance * 125.0f * 1e-4f;
  }
  else
  {
    applyZoomScroll(delta);
  }
}

// ----------------------------------------------------------------
// Glitch filter
// ----------------------------------------------------------------
void COrbitCameraController::mouseGlitchFilter(const int x, const int y, int& lastX, int& lastY)
{
  if (std::abs(x - lastX) > 60)
  {
    lastX = x;
  }
  if (std::abs(y - lastY) > 60)
  {
    lastY = y;
  }
}

// ----------------------------------------------------------------
// Pure math helpers
// ----------------------------------------------------------------
void COrbitCameraController::applyOrbit(int dx, int dy)
{
  m_params.azimuthDeg -= static_cast<float>(dx) * orbitSensitivity;
  m_params.elevationDeg =
      clampElevation(m_params.elevationDeg + static_cast<float>(dy) * orbitSensitivity);
}

void COrbitCameraController::applyPan(int dx, int dy)
{
  const float azRad = m_params.azimuthDeg * static_cast<float>(M_PI) / 180.0f;
  const float elRad = m_params.elevationDeg * static_cast<float>(M_PI) / 180.0f;

  const float scale = m_params.zoomDistance * panSensitivity;
  const float sinAz = std::sin(azRad);
  const float cosAz = std::cos(azRad);
  const float sinEl = std::sin(elRad);
  const float cosEl = std::cos(elRad);

  // Camera right vector (world XY)
  const float rx = cosAz;
  const float ry = sinAz;

  // Camera up vector (projected onto world)
  const float ux = -sinAz * sinEl;
  const float uy = cosAz * sinEl;
  const float uz = cosEl;

  const float panX = -static_cast<float>(dx) * scale;
  const float panY = static_cast<float>(dy) * scale;

  m_params.pointingX += rx * panX + ux * panY;
  m_params.pointingY += ry * panX + uy * panY;
  m_params.pointingZ += uz * panY;
}

void COrbitCameraController::applyRotate(int dx, int /*dy*/)
{
  // "Rotate" here means spin azimuth (matches legacy CGlCanvasBase behaviour)
  m_params.azimuthDeg -= static_cast<float>(dx) * orbitSensitivity;
}

void COrbitCameraController::applyRoll(int dx, int dy)
{
  const float d = std::sqrt(static_cast<float>(dx * dx + dy * dy));
  const float sign = (dx * dy >= 0) ? 1.0f : -1.0f;
  m_params.rollDeg += sign * d * rollSensitivity;
}

void COrbitCameraController::applyZoomDelta(int /*dx*/, int dy)
{
  m_params.zoomDistance *= 1.0f + static_cast<float>(dy) * 0.01f;
  clampZoom();
}

void COrbitCameraController::applyZoomScroll(float delta)
{
  m_params.zoomDistance *= 1.0f - delta * zoomSensitivity;
  clampZoom();
}

void COrbitCameraController::clampZoom()
{
  m_params.zoomDistance = std::clamp(m_params.zoomDistance, m_minZoom, m_maxZoom);
}

float COrbitCameraController::clampElevation(float deg) { return std::clamp(deg, -89.9f, 89.9f); }

// ----------------------------------------------------------------
// CCamera bridge
// ----------------------------------------------------------------
void COrbitCameraController::applyTo(mrpt::viz::CCamera& cam) const
{
  cam.setPointingAt(m_params.pointingX, m_params.pointingY, m_params.pointingZ);
  cam.setZoomDistance(m_params.zoomDistance);
  cam.setAzimuthDegrees(m_params.azimuthDeg);
  cam.setElevationDegrees(m_params.elevationDeg);
  cam.setRollDegrees(m_params.rollDeg);
  cam.setProjectiveModel(m_params.isProjective);
  cam.setProjectiveFOVdeg(m_params.fovDeg);
}

void COrbitCameraController::setFrom(const mrpt::viz::CCamera& cam)
{
  m_params.pointingX = cam.getPointingAtX();
  m_params.pointingY = cam.getPointingAtY();
  m_params.pointingZ = cam.getPointingAtZ();
  m_params.zoomDistance = cam.getZoomDistance();
  m_params.azimuthDeg = cam.getAzimuthDegrees();
  m_params.elevationDeg = cam.getElevationDegrees();
  m_params.rollDeg = cam.getRollDegrees();
  m_params.isProjective = cam.isProjective();
  m_params.fovDeg = cam.getProjectiveFOVdeg();
}