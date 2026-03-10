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

#include <mrpt/opengl/CompiledScene.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/COrbitCameraController.h>
#include <mrpt/viz/Scene.h>

#include <cmath>
#include <functional>
#include <memory>

namespace mrpt::imgui
{
/** Renders an mrpt::viz::Scene into an OpenGL FBO texture and displays
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
  void setScene(const mrpt::viz::Scene::Ptr& scene) { m_scene = scene; }

  /** Get the scene being rendered (may be nullptr). */
  [[nodiscard]] mrpt::viz::Scene::Ptr scene() const { return m_scene; }

  /** @} */

  /** @name Camera control
   *  @{ */

  /** The orbit camera controller. Use this to read/write azimuth, elevation,
   *  zoom distance, pointing-at position, and interaction sensitivities.
   *
   *  Example:
   *  \code
   *    view.cameraController.setZoomDistance(20.0f);
   *    view.cameraController.setAzimuthDegrees(-135.0f);
   *    view.cameraController.orbitSensitivity = 0.5f;
   *  \endcode
   */
  mrpt::viz::COrbitCameraController cameraController;

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

  /** @name Callbacks
   *  @{ */

  /** Called after the MRPT scene is rendered but before ImGui::Image().
   *  Useful for drawing overlay ImGui widgets on top. */
  std::function<void()> onOverlayGui;

  /** Called when the user left-clicks on the 3D view without dragging.
   *  Arguments: (pixel_x, pixel_y) in widget-local coordinates. */
  std::function<void(float, float)> onLeftClick;

  /** @} */

 private:
  // --- Scene ---
  mrpt::viz::Scene::Ptr m_scene;

  // --- Render-time camera snapshot (written by cameraController.applyTo) ---
  mrpt::viz::CCamera m_camera;

  // --- Compiled scene for rendering pipeline ---
  std::unique_ptr<mrpt::opengl::CompiledScene> m_compiledScene;
  std::weak_ptr<mrpt::viz::Scene> m_lastCompiledScenePtr;

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

  // --- Camera interaction ---
  void handleMouseInteraction(float widgetX, float widgetY);
};

// Implemented only from the user's translation unit to avoid forcing imgui.h
// inclusion in the header.

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

  // ---- Sync controller → camera snapshot before rendering ----
  cameraController.applyTo(m_camera);

  // ---- Render the MRPT scene into our FBO ----
  if (m_scene)
  {
    // Push camera into the scene's main viewport
    if (auto vp = m_scene->getViewport("main"); vp)
    {
      vp->getCamera() = m_camera;
      vp->setCustomBackgroundColor({m_bgColor[0], m_bgColor[1], m_bgColor[2], m_bgColor[3]});
    }

    // Compile / incrementally update the scene
    auto lastPtr = m_lastCompiledScenePtr.lock();
    if (!m_compiledScene || lastPtr.get() != m_scene.get())
    {
      m_compiledScene = std::make_unique<mrpt::opengl::CompiledScene>();
      m_compiledScene->compile(*m_scene);
      m_lastCompiledScenePtr = m_scene;
    }
    else
    {
      m_compiledScene->updateIfNeeded();
    }

    // Save and bind our FBO
    GLint prevViewport[4];
    glGetIntegerv(GL_VIEWPORT, prevViewport);
    GLint prevFBO = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFBO);

    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glViewport(0, 0, static_cast<GLsizei>(w), static_cast<GLsizei>(h));

    glClearColor(m_bgColor[0], m_bgColor[1], m_bgColor[2], m_bgColor[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_compiledScene->render(w, h, 0, 0);

    // Restore previous GL state
    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(prevFBO));
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
  }

  // ---- Display the rendered texture as an ImGui image ----
  const ImVec2 cursorScreenPos = ImGui::GetCursorScreenPos();
  const ImVec2 uv0(0.0f, 1.0f);
  const ImVec2 uv1(1.0f, 0.0f);
  const ImVec2 size(static_cast<float>(w), static_cast<float>(h));

  ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(m_texColor)), size, uv0, uv1);

  // Invisible button on top prevents the window from being dragged while
  // interacting with the 3D scene.
  ImGui::SetCursorScreenPos(cursorScreenPos);
  ImGui::InvisibleButton(
      "##scene_canvas", size,
      ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
          ImGuiButtonFlags_MouseButtonMiddle);

  const bool isHovered = ImGui::IsItemHovered();
  const bool isActive = ImGui::IsItemActive();

  // ---- Mouse-based camera interaction ----
  if (isHovered || isActive)
  {
    const ImVec2 mousePos = ImGui::GetMousePos();
    const float localX = mousePos.x - cursorScreenPos.x;
    const float localY = mousePos.y - cursorScreenPos.y;
    handleMouseInteraction(localX, localY);
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
// Mouse interaction — orbit, pan, zoom via COrbitCameraController
// -----------------------------------------------------------------------
inline void CImGuiSceneView::handleMouseInteraction(float widgetX, float widgetY)
{
  using C = mrpt::viz::COrbitCameraController;

  ImGuiIO& io = ImGui::GetIO();

  const int ix = static_cast<int>(widgetX);
  const int iy = static_cast<int>(widgetY);

  // --- Build modifier bitmask ---
  uint8_t mods = 0;
  if (io.KeyShift) mods |= C::ModShift;
  if (io.KeyCtrl) mods |= C::ModControl;
  if (io.KeyAlt) mods |= C::ModAlt;

  // --- Fire press/release events so the controller can initialise its
  //     click-position for the glitch filter and track button state ---
  constexpr std::pair<ImGuiMouseButton, uint8_t> kButtonMap[3] = {
      {  ImGuiMouseButton_Left,   C::ButtonLeft},
      {ImGuiMouseButton_Middle, C::ButtonMiddle},
      { ImGuiMouseButton_Right,  C::ButtonRight},
  };
  for (auto [imguiBtn, mrptBtn] : kButtonMap)
  {
    if (ImGui::IsMouseClicked(imguiBtn))
      cameraController.onMouseButton(ix, iy, mrptBtn, /*down=*/true);
    else if (ImGui::IsMouseReleased(imguiBtn))
      cameraController.onMouseButton(ix, iy, mrptBtn, /*down=*/false);
  }

  // --- Build held-button bitmask and forward movement ---
  uint8_t buttons = 0;
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) buttons |= C::ButtonLeft;
  if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) buttons |= C::ButtonMiddle;
  if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) buttons |= C::ButtonRight;

  if (buttons != 0) cameraController.onMouseMove(ix, iy, buttons, mods);

  // --- Scroll wheel ---
  if (std::abs(io.MouseWheel) > 0.0f) cameraController.onScroll(io.MouseWheel, mods);

  // --- Left-click callback: fire on release with no significant drag ---
  if (onLeftClick && ImGui::IsMouseReleased(ImGuiMouseButton_Left))
  {
    const ImVec2 drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, /*lock_threshold=*/0.0f);
    const float dragDist = std::sqrt(drag.x * drag.x + drag.y * drag.y);
    if (dragDist < 4.0f)  // pixel threshold: treat as a click, not a drag
      onLeftClick(widgetX, widgetY);
  }
}

#endif  // MRPT_IMGUI_AVAILABLE

}  // namespace mrpt::imgui