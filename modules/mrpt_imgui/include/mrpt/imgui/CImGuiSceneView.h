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

#include <mrpt/math/TLine3D.h>
#include <mrpt/opengl/CompiledScene.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/COrbitCameraController.h>
#include <mrpt/viz/Scene.h>

#include <cmath>
#include <functional>
#include <memory>
#include <optional>

/** Defined if CImGuiSceneView has renderAsBackground() and mouseRay(), so
 *  user code can keep building against older MRPT versions. */
#define MRPT_IMGUI_HAS_BACKGROUND_SCENE_VIEW 1

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

  /** Like render(), but draws the scene straight into the framebuffer
   *  bound while ImGui renders (normally, the window's default one), behind
   *  all ImGui windows. This avoids the intermediary FBO and its extra copy,
   *  and keeps the window multisampling (MSAA), if any.
   *
   *  Call it inside a transparent ImGui window covering the region where the
   *  scene must appear (typically, the whole main viewport), which is only
   *  used to capture the mouse:
   *  \code
   *    const ImGuiViewport* vp = ImGui::GetMainViewport();
   *    ImGui::SetNextWindowPos(vp->WorkPos);
   *    ImGui::SetNextWindowSize(vp->WorkSize);
   *    ImGui::SetNextWindowBgAlpha(0.0f);
   *    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
   *    ImGui::Begin("##bg", nullptr,
   *        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
   *        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
   *        ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings);
   *    ImGui::PopStyleVar();
   *    view.renderAsBackground();
   *    ImGui::End();
   *  \endcode
   *
   *  The actual GL rendering happens later, from an ImGui draw callback
   *  inside ImGui_ImplOpenGL3_RenderDrawData(), so the scene and this object
   *  must stay alive until then. The framebuffer needs a depth buffer.
   *
   *  The scene gamma correction (sRGB encoding, see TLightParameters) is only
   *  applied if the framebuffer is sRGB-capable, e.g. with
   *  `glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE)` before creating a GLFW
   *  window. Otherwise, colors are written linearly, as with render().
   */
  void renderAsBackground();

  /** Returns the 3D ray (in scene coordinates) for the mouse position in the
   *  last render call, or nullopt if the mouse was not over the view or the
   *  scene has not been rendered yet. */
  [[nodiscard]] std::optional<mrpt::math::TLine3D> mouseRay() const;

  /** True if the mouse was over the view in the last render call. */
  [[nodiscard]] bool isHovered() const { return m_hovered; }

  /** @} */

  /** @name Appearance
   *  @{ */

  /** Background color override. If never called, the scene's own viewport
   *  background (typically a vertical gradient set up by mrpt::viz::Scene)
   *  is preserved. */
  void setBackgroundColor(float r, float g, float b, float a = 1.0f)
  {
    m_bgColor[0] = r;
    m_bgColor[1] = g;
    m_bgColor[2] = b;
    m_bgColor[3] = a;
    m_hasCustomBg = true;
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

  /** Syncs the camera and renders the scene into the FBO. Not inline, so that
   *  user code does not need GL 3.x prototypes visible at its include site. */
  void renderSceneToFBO(int w, int h);

  /** Syncs the camera into the scene and compiles (or updates) it.
   *  \return false if there is nothing to render. */
  bool prepareCompiledScene();

  /** Renders the scene into the current framebuffer, in m_directRect. */
  void renderSceneDirect();

  /** Framebuffer pixels for renderSceneDirect(): x, y (from the bottom-left
   *  corner), width, height */
  int m_directRect[4] = {0, 0, 0, 0};

  // --- Appearance ---
  float m_bgColor[4] = {0.3f, 0.3f, 0.3f, 1.0f};
  bool m_hasCustomBg = false;

  // --- Mouse state, for mouseRay() ---
  bool m_hovered = false;
  float m_mouseX = 0;  //!< widget-local, in ImGui units
  float m_mouseY = 0;
  float m_pixelScale = 1.0f;  //!< rendered pixels per ImGui unit

  // --- Camera interaction ---
  /** Places an input-capturing button over the widget area (in screen
   *  coordinates), and handles the camera and the overlay/click callbacks. */
  void handleWidgetInput(float screenX, float screenY, float w, float h);
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
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL

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

  // ---- Render the MRPT scene into our FBO (GL code lives in the .cpp) ----
  renderSceneToFBO(w, h);

  // ---- Display the rendered texture as an ImGui image ----
  const ImVec2 cursorScreenPos = ImGui::GetCursorScreenPos();
  const ImVec2 uv0(0.0f, 1.0f);
  const ImVec2 uv1(1.0f, 0.0f);
  const ImVec2 size(static_cast<float>(w), static_cast<float>(h));

  ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(m_texColor)), size, uv0, uv1);

  m_pixelScale = 1.0f;
  handleWidgetInput(cursorScreenPos.x, cursorScreenPos.y, size.x, size.y);

#else
  ImGui::TextUnformatted("MRPT built without OpenGL support.");
#endif
}

// -----------------------------------------------------------------------
// Background entry point: render straight into the current framebuffer
// -----------------------------------------------------------------------
inline void CImGuiSceneView::renderAsBackground()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const ImVec2 pos = ImGui::GetCursorScreenPos();
  const float w = std::max(1.0f, avail.x);
  const float h = std::max(1.0f, avail.y);

  // ImGui units to framebuffer pixels (HiDPI) of the viewport (platform
  // window) this window is in, with the origin at the bottom-left corner as
  // OpenGL expects:
  ImGuiViewport* viewport = ImGui::GetWindowViewport();
  ImVec2 scale = viewport->FramebufferScale;
  if (scale.x <= 0.0f || scale.y <= 0.0f)
  {
    scale = ImGui::GetIO().DisplayFramebufferScale;
  }
  m_directRect[0] = static_cast<int>((pos.x - viewport->Pos.x) * scale.x);
  m_directRect[1] = static_cast<int>((viewport->Size.y - (pos.y - viewport->Pos.y + h)) * scale.y);
  m_directRect[2] = static_cast<int>(w * scale.x);
  m_directRect[3] = static_cast<int>(h * scale.y);

  // The background draw list of the viewport is rendered before any window:
  ImDrawList* dl = ImGui::GetBackgroundDrawList(viewport);
  dl->AddCallback(
      [](const ImDrawList*, const ImDrawCmd* cmd)
      { static_cast<CImGuiSceneView*>(cmd->UserCallbackData)->renderSceneDirect(); },
      this);
  dl->AddCallback(ImDrawCallback_ResetRenderState, nullptr);

  m_pixelScale = scale.x;
  handleWidgetInput(pos.x, pos.y, w, h);
#else
  ImGui::TextUnformatted("MRPT built without OpenGL support.");
#endif
}

// -----------------------------------------------------------------------
// Input over the widget area: camera, overlay and click callbacks
// -----------------------------------------------------------------------
inline void CImGuiSceneView::handleWidgetInput(float screenX, float screenY, float w, float h)
{
  const ImVec2 cursorScreenPos(screenX, screenY);
  const ImVec2 size(w, h);

  // Invisible button on top prevents the window from being dragged while
  // interacting with the 3D scene.
  ImGui::SetCursorScreenPos(cursorScreenPos);
  ImGui::InvisibleButton(
      "##scene_canvas", size,
      ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
          ImGuiButtonFlags_MouseButtonMiddle);

  const bool isHovered = ImGui::IsItemHovered();
  const bool isActive = ImGui::IsItemActive();

  const ImVec2 mousePos = ImGui::GetMousePos();
  m_mouseX = mousePos.x - cursorScreenPos.x;
  m_mouseY = mousePos.y - cursorScreenPos.y;
  m_hovered = isHovered;

  // ---- Mouse-based camera interaction ----
  if (isHovered || isActive)
  {
    handleMouseInteraction(m_mouseX, m_mouseY);
  }

  // ---- Overlay callback ----
  if (onOverlayGui)
  {
    onOverlayGui();
  }
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