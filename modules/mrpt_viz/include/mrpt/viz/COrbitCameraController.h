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

#pragma once

#include <mrpt/viz/CCamera.h>

#include <cstdint>

namespace mrpt::viz
{
/**
 * @brief Framework-agnostic orbit/pan/zoom camera controller.
 *
 * Encapsulates the camera state (azimuth, elevation, zoom distance,
 * pointing-at, roll, FOV, projective flag) and the mouse-delta math
 * that converts raw pointer/scroll events into camera changes.
 *
 * It is intentionally free of any UI or OpenGL dependency so it can be
 * reused by mrpt_gui (nanogui), mrpt_imgui (Dear ImGui), or any future
 * frontend.
 *
 * Typical usage - forward framework mouse events into the three
 * `on*` methods; then call `applyTo(camera)` before rendering:
 *
 * @code
 *   mrpt::viz::COrbitCameraController ctrl;
 *   ctrl.setCameraPointing(0, 0, 0);
 *   ctrl.setZoomDistance(15.0f);
 *
 *   // Inside your mouse-move handler:
 *   ctrl.onMouseMove(x, y, MouseButtons::Left, KeyModifiers::None);
 *
 *   // Before rendering:
 *   ctrl.applyTo(myCamera);
 * @endcode
 *
 * \ingroup mrpt_viz_grp
 */
class COrbitCameraController
{
 public:
  // ----------------------------------------------------------------
  // Button / modifier flags (UI-framework-neutral bitmasks)
  // ----------------------------------------------------------------

  /** Bitmask for mouse buttons. */
  enum MouseButtons : uint8_t
  {
    ButtonNone = 0,
    ButtonLeft = 1 << 0,
    ButtonMiddle = 1 << 1,
    ButtonRight = 1 << 2,
  };

  /** Bitmask for keyboard modifiers. */
  enum KeyModifiers : uint8_t
  {
    ModNone = 0,
    ModShift = 1 << 0,
    ModControl = 1 << 1,
    ModAlt = 1 << 2,
  };

  // ----------------------------------------------------------------
  // Construction
  // ----------------------------------------------------------------

  COrbitCameraController();
  ~COrbitCameraController() = default;

  COrbitCameraController(const COrbitCameraController&) = default;
  COrbitCameraController& operator=(const COrbitCameraController&) = default;
  COrbitCameraController(COrbitCameraController&&) = default;
  COrbitCameraController& operator=(COrbitCameraController&&) = default;

  // ----------------------------------------------------------------
  // Camera parameter accessors
  // ----------------------------------------------------------------

  void setCameraPointing(float x, float y, float z);
  [[nodiscard]] float getCameraPointingX() const { return m_params.pointingX; }
  [[nodiscard]] float getCameraPointingY() const { return m_params.pointingY; }
  [[nodiscard]] float getCameraPointingZ() const { return m_params.pointingZ; }

  void setZoomDistance(float d);
  [[nodiscard]] float getZoomDistance() const { return m_params.zoomDistance; }

  void setAzimuthDegrees(float deg);
  [[nodiscard]] float getAzimuthDegrees() const { return m_params.azimuthDeg; }

  void setElevationDegrees(float deg);
  [[nodiscard]] float getElevationDegrees() const { return m_params.elevationDeg; }

  void setRollDegrees(float deg) { m_params.rollDeg = deg; }
  [[nodiscard]] float getRollDegrees() const { return m_params.rollDeg; }

  void setProjectiveModel(bool isProjective) { m_params.isProjective = isProjective; }
  [[nodiscard]] bool isProjectiveModel() const { return m_params.isProjective; }

  void setFOVdeg(float fov) { m_params.fovDeg = fov; }
  [[nodiscard]] float getFOVdeg() const { return m_params.fovDeg; }

  /** Clamp zoom to [minZoom, maxZoom] (default: [0.01, 3200]). */
  void setZoomLimits(float minZoom, float maxZoom);

  // ----------------------------------------------------------------
  // Camera interaction sensitivities
  // ----------------------------------------------------------------

  float orbitSensitivity = 0.25f;  ///< deg/pixel
  float panSensitivity = 0.01f;    ///< world-units * zoom factor per pixel
  float zoomSensitivity = 0.15f;   ///< fractional distance change per scroll unit
  float rollSensitivity = 0.15f;   ///< deg/pixel

  // ----------------------------------------------------------------
  // Generic mouse event API
  //   x, y         - pointer position in widget-local pixels
  //   buttons      - MouseButtons bitmask of currently held buttons
  //   modifiers    - KeyModifiers bitmask
  // ----------------------------------------------------------------

  /** Call on pointer-move events while any button is held. */
  void onMouseMove(int x, int y, uint8_t buttons, uint8_t modifiers);

  /** Call on button press/release. */
  void onMouseButton(int x, int y, uint8_t button, bool down);

  /** Call on scroll-wheel events.
   *  @param delta   +1 = wheel-up (zoom in), -1 = wheel-down (zoom out).
   *                 Fractional values are accepted (high-DPI trackpads).
   *  @param modifiers  KeyModifiers bitmask.
   */
  void onScroll(float delta, uint8_t modifiers);

  // ----------------------------------------------------------------
  // Apply to a CCamera object
  // ----------------------------------------------------------------

  /** Writes the current orbit parameters into @p cam. */
  void applyTo(mrpt::viz::CCamera& cam) const;

  /** Initialises the controller from an existing CCamera. */
  void setFrom(const mrpt::viz::CCamera& cam);

 private:
  // ----------------------------------------------------------------
  // Internal camera parameter bundle
  // ----------------------------------------------------------------
  struct Params
  {
    float pointingX = 0.f, pointingY = 0.f, pointingZ = 0.f;
    float zoomDistance = 15.f;
    float azimuthDeg = -135.f;
    float elevationDeg = 25.f;
    float rollDeg = 0.f;
    bool isProjective = true;
    float fovDeg = 45.f;
  };

  Params m_params;

  float m_minZoom = 0.01f;
  float m_maxZoom = 3200.f;

  // Mouse tracking
  int m_lastX = 0, m_lastY = 0;
  int m_clickX = 0, m_clickY = 0;
  bool m_buttonDown = false;

  // ----------------------------------------------------------------
  // Pure math helpers (no UI / GL dependency)
  // ----------------------------------------------------------------

  /** Left-drag, no modifier: orbit (azimuth + elevation). */
  void applyOrbit(int dx, int dy);

  /** Left-drag + Shift OR middle-drag: pan in camera's local XY plane. */
  void applyPan(int dx, int dy);

  /** Left-drag + Ctrl: rotate (spin around look direction). */
  void applyRotate(int dx, int dy);

  /** Left-drag + Alt: roll. */
  void applyRoll(int dx, int dy);

  /** Zoom by a raw pixel delta (zoom-drag mode). */
  void applyZoomDelta(int dx, int dy);

  /** Zoom by a scroll/wheel amount. */
  void applyZoomScroll(float delta);

  /** Clamp zoom to configured limits. */
  void clampZoom();

  /** Clamp elevation to (-90, 90). */
  static float clampElevation(float deg);

  static void mouseGlitchFilter(int x, int y, int& lastX, int& lastY);
};

}  // namespace mrpt::viz