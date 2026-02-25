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

#if __has_include(<imgui.h>)
#include <imgui.h>  // This must be available from the user code space, MRPT does not include it as a dependency.

#include <algorithm>
#define MRPT_IMGUI_AVAILABLE
#endif

#include <mrpt/opengl/opengl_api.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/Scene.h>

#include <cmath>
#include <functional>

namespace mrpt::imgui
{
/** Renders an mrpt::opengl::Scene into an OpenGL FBO texture and displays
 *  it as a Dear ImGui image widget, with built-in orbit camera controls.
 *
 *  Usage:
 *  \code
 *    // In your ImGui frame loop:
 *    if (ImGui::Begin("3D View"))
 *    {
 *        sceneView.render();
 *    }
 *    ImGui::End();
 *  \endcode
 *
 *  The class assumes an OpenGL 3.3+ context is already current (as provided
 *  by the Dear ImGui GLFW/SDL backend). No EGL context is created.
 *
 *  \ingroup mrpt_imgui_grp
 */
class CImGuiSceneView
{
 public:
  CImGuiSceneView();
  ~CImGuiSceneView();

  CImGuiSceneView(const CImGuiSceneView&) = delete;
  CImGuiSceneView& operator=(const CImGuiSceneView&) = delete;
  CImGuiSceneView(CImGuiSceneView&&) = delete;
  CImGuiSceneView& operator=(CImGuiSceneView&&) = delete;

  /** @name Scene access
   *  @{ */

  /** Set the scene to render. The caller retains ownership. */
  void setScene(const mrpt::opengl::Scene::Ptr& scene) { m_scene = scene; }

  /** Get the scene being rendered (may be nullptr). */
  [[nodiscard]] mrpt::opengl::Scene::Ptr scene() const { return m_scene; }

  /** @} */

  /** @name Camera control
   *  @{ */

  /** Direct access to the camera used for rendering.
   *  Modify orbit parameters (azimuth, elevation, distance, pointing-at)
   *  directly through this reference. */
  [[nodiscard]] mrpt::opengl::CCamera& camera() { return m_camera; }

  [[nodiscard]] const mrpt::opengl::CCamera& camera() const { return m_camera; }

  /** @} */

  /** @name Rendering
   *  @{ */

  /** Renders the scene into the FBO and displays it via ImGui::Image().
   *  Call this inside an ImGui window (between Begin/End).
   *
   *  The widget fills the available content region. If the region size
   *  changed since the last call, the FBO is recreated automatically.
   */
  void render();

  /** @} */

  /** @name Appearance
   *  @{ */

  /** Background color (default: dark gray 0.3, 0.3, 0.3) */
  void setBackgroundColor(float r, float g, float b, float a = 1.0f)
  {
    m_bgColor[0] = r;
    m_bgColor[1] = g;
    m_bgColor[2] = b;
    m_bgColor[3] = a;
  }

  /** @} */

  /** @name Mouse sensitivity
   *  @{ */

  void setOrbitSensitivity(float s) { m_orbitSensitivity = s; }
  void setPanSensitivity(float s) { m_panSensitivity = s; }
  void setZoomSensitivity(float s) { m_zoomSensitivity = s; }

  /** @} */

  /** @name Callbacks
   *  @{ */

  /** Called after the MRPT scene is rendered but before ImGui::Image().
   *  Useful for drawing overlay ImGui widgets on top. */
  std::function<void()> onOverlayGui;

  /** Called when the user left-clicks on the 3D view.
   *  Arguments: (pixel_x, pixel_y) in widget-local coordinates. */
  std::function<void(float, float)> onLeftClick;

  /** @} */

 private:
  // --- Scene & camera ---
  mrpt::opengl::Scene::Ptr m_scene;
  mrpt::opengl::CCamera m_camera;

  // --- FBO state ---
  unsigned int m_fbo = 0;
  unsigned int m_rboDepth = 0;
  unsigned int m_texColor = 0;
  int m_fboWidth = 0;
  int m_fboHeight = 0;

  void ensureFBO(int w, int h);
  void destroyFBO();

  // --- Appearance ---
  float m_bgColor[4] = {0.3f, 0.3f, 0.3f, 1.0f};

  // --- Camera interaction state ---
  float m_orbitSensitivity = 0.3f;
  float m_panSensitivity = 0.01f;
  float m_zoomSensitivity = 0.15f;
  bool m_isOrbiting = false;
  bool m_isPanning = false;

  void handleMouseInteraction(float widgetX, float widgetY, float w, float h);
};

// Implemented only from the user's translation unit to avoid forcing imgui.h inclusion in the
// header.

#if defined(MRPT_IMGUI_AVAILABLE)
// -----------------------------------------------------------------------
// Main render entry point — call inside ImGui Begin/End
// -----------------------------------------------------------------------
inline void CImGuiSceneView::render()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  // Determine available area
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const int w = std::max(1, static_cast<int>(avail.x));
  const int h = std::max(1, static_cast<int>(avail.y));

  // (Re)create FBO if needed
  ensureFBO(w, h);

  if (m_fbo == 0)
  {
    ImGui::TextUnformatted("(No OpenGL FBO available)");
    return;
  }

  // ---- Render the MRPT scene into our FBO ----
  if (m_scene)
  {
    // Save current GL state we are going to modify
    GLint prevViewport[4];
    glGetIntegerv(GL_VIEWPORT, prevViewport);
    GLint prevFBO = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFBO);

    // Bind our FBO
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glViewport(0, 0, static_cast<GLsizei>(w), static_cast<GLsizei>(h));

    // Clear with our background color
    glClearColor(m_bgColor[0], m_bgColor[1], m_bgColor[2], m_bgColor[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Apply our camera to the scene's main viewport
    auto vp = m_scene->getViewport("main");
    if (vp)
    {
      vp->setCustomBackgroundColor({m_bgColor[0], m_bgColor[1], m_bgColor[2], m_bgColor[3]});

      // Render each viewport, passing our camera
      vp->render(w, h, 0, 0, &m_camera);
    }

    // Restore previous GL state
    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(prevFBO));
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
  }

// ---- Display the rendered texture as an ImGui image ----
  const ImVec2 cursorScreenPos = ImGui::GetCursorScreenPos();
  const ImVec2 uv0(0.0f, 1.0f);
  const ImVec2 uv1(1.0f, 0.0f);
  const ImVec2 size(static_cast<float>(w), static_cast<float>(h));

  ImGui::Image(
      static_cast<ImTextureID>(static_cast<uintptr_t>(m_texColor)),
      size, uv0, uv1);

  // This prevents the window from being dragged when interacting with the scene.
  ImGui::SetCursorScreenPos(cursorScreenPos);
  ImGui::InvisibleButton("##scene_canvas", size, 
      ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);

  const bool isHovered = ImGui::IsItemHovered();
  const bool isActive = ImGui::IsItemActive(); // True if clicking/dragging the canvas

  // ---- Mouse-based camera interaction ----
  if (isHovered || isActive)
  {
    const ImVec2 mousePos = ImGui::GetMousePos();
    const float localX = mousePos.x - cursorScreenPos.x;
    const float localY = mousePos.y - cursorScreenPos.y;
    handleMouseInteraction(localX, localY, static_cast<float>(w), static_cast<float>(h));
  }
  else
  {
    m_isOrbiting = false;
    m_isPanning = false;
  }

  // ---- Overlay callback ----
  if (onOverlayGui)
  {
    onOverlayGui();
  }

#else
  ImGui::TextUnformatted("MRPT built without OpenGL support.");
#endif
}

// -----------------------------------------------------------------------
// Mouse interaction — orbit, pan, zoom
// -----------------------------------------------------------------------
inline void CImGuiSceneView::handleMouseInteraction(
    float /*widgetX*/, float widgetY, float /*w*/, float h)
{
  ImGuiIO& io = ImGui::GetIO();

  // --- Orbit (left-drag) ---
  if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
  {
    const ImVec2 delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
    ImGui::ResetMouseDragDelta(ImGuiMouseButton_Left);

    const float dAz = -delta.x * m_orbitSensitivity;
    const float dEl = delta.y * m_orbitSensitivity;

    m_camera.setAzimuthDegrees(m_camera.getAzimuthDegrees() + dAz);

    float newEl = m_camera.getElevationDegrees() + dEl;
    newEl = std::clamp(newEl, -89.9f, 89.9f);
    m_camera.setElevationDegrees(newEl);

    m_isOrbiting = true;
  }
  else
  {
    m_isOrbiting = false;
  }

  // --- Pan (middle-drag or Shift+left-drag) ---
  const bool shiftHeld = io.KeyShift;
  const bool panWithMiddle = ImGui::IsMouseDragging(ImGuiMouseButton_Middle);
  const bool panWithShiftLeft = shiftHeld && ImGui::IsMouseDragging(ImGuiMouseButton_Left);

  if (panWithMiddle || panWithShiftLeft)
  {
    const int btn = panWithMiddle ? ImGuiMouseButton_Middle : ImGuiMouseButton_Left;
    const ImVec2 delta = ImGui::GetMouseDragDelta(btn);
    ImGui::ResetMouseDragDelta(btn);

    const float dist = m_camera.getZoomDistance();
    const float azRad = m_camera.getAzimuthDegrees() * static_cast<float>(M_PI) / 180.0f;

    // Pan in the camera's local right/up directions (projected on XY)
    const float panScale = dist * m_panSensitivity;

    const float dx = -delta.x * panScale;
    const float dy = delta.y * panScale;

    const float sinAz = std::sin(azRad);
    const float cosAz = std::cos(azRad);

    // Right direction in world XY
    const float rx = cosAz;
    const float ry = sinAz;

    // Up direction approximation (world Z for small elevations,
    // or the perpendicular in the orbit plane)
    const float elRad = m_camera.getElevationDegrees() * static_cast<float>(M_PI) / 180.0f;
    const float cosEl = std::cos(elRad);
    const float sinEl = std::sin(elRad);

    // "up" in camera coords projects onto world as:
    const float ux = -sinAz * sinEl;
    const float uy = cosAz * sinEl;
    const float uz = cosEl;

    m_camera.setPointingAt(
        m_camera.getPointingAtX() + rx * dx + ux * dy,
        m_camera.getPointingAtY() + ry * dx + uy * dy, m_camera.getPointingAtZ() + uz * dy);

    m_isPanning = true;
  }
  else
  {
    m_isPanning = false;
  }

  // --- Zoom (scroll wheel) ---
  if (std::abs(io.MouseWheel) > 0.0f)
  {
    float dist = m_camera.getZoomDistance();
    dist *= 1.0f - io.MouseWheel * m_zoomSensitivity;
    dist = std::max(0.01f, dist);
    m_camera.setZoomDistance(dist);
  }

  // --- Left click callback ---
  if (onLeftClick && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !m_isOrbiting && !m_isPanning)
  {
    const ImVec2 mousePos = ImGui::GetMousePos();
    const ImVec2 origin = ImGui::GetItemRectMin();
    onLeftClick(mousePos.x - origin.x, mousePos.y - origin.y);
  }
}
#endif

}  // namespace mrpt::imgui
