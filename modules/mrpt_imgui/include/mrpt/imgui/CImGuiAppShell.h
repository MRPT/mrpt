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

#include <mrpt/img/CImage.h>

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

struct GLFWwindow;

namespace mrpt::imgui
{
/** Window and main loop of a Dear ImGui application with docking panels.
 *
 * Owns the GLFW window, its OpenGL 3.3 core context, and the ImGui and ImPlot
 * contexts. Every frame it sets up a full-window dockspace whose central node
 * is transparent ("passthrough"), so the application can draw a 3D scene
 * behind all panels (see CImGuiBackgroundScene) and the user can dock panels
 * anywhere. The docking layout is saved to `imgui.ini` in the per-user config
 * directory of the application (see userConfigDirectory()), and restored on
 * the next run.
 *
 * It also loads the Material Symbols icon font (mrpt/imgui/icons.h), merged
 * into the default font, and scales the UI with the monitor content scale
 * (HiDPI).
 *
 * A minimal application:
 * \code
 *   mrpt::imgui::CImGuiAppShell app;
 *   if (!app.init("My app")) { return 1; }
 *   app.mainMenuBar = [&]() {
 *     if (ImGui::BeginMenu("File")) {
 *       if (ImGui::MenuItem("Quit")) { app.requestClose(); }
 *       ImGui::EndMenu();
 *     }
 *   };
 *   app.run([&]() {
 *     ImGui::Begin("Panel");
 *     ImGui::Text("Hello");
 *     ImGui::End();
 *   });
 * \endcode
 *
 * For automated (headless) tests, two environment variables are honored:
 * - `MRPT_IMGUI_MAX_FRAMES=N`: run() returns after N frames.
 * - `MRPT_IMGUI_SCREENSHOT=file.png`: the last frame is saved to that file.
 *
 * \ingroup mrpt_imgui_grp
 */
class CImGuiAppShell
{
 public:
  struct Options
  {
    int width = 1280;
    int height = 800;
    /** MSAA samples of the window framebuffer (0 = none). Antialiases the
     * 3D scenes drawn as the window background. */
    int msaaSamples = 4;
    /** If true, the window framebuffer is sRGB capable, so scenes with gamma
     * correction are rendered correctly as the window background. */
    bool srgbFramebuffer = true;
    bool vsync = true;
    /** Name of the per-user config subdirectory where the layout is saved.
     * If empty, the window title is used. */
    std::string configName;
    /** If false, no `imgui.ini` file is read or written. */
    bool persistLayout = true;
    /** Size of the default font, before DPI scaling, in pixels. */
    float fontSize = 15.0f;
  };

  CImGuiAppShell() = default;
  ~CImGuiAppShell();

  CImGuiAppShell(const CImGuiAppShell&) = delete;
  CImGuiAppShell& operator=(const CImGuiAppShell&) = delete;

  /** Creates the window, the GL context, and the ImGui/ImPlot contexts.
   * \return false on failure (e.g. no display); details are printed to
   * stderr. */
  bool init(const std::string& windowTitle, const Options& options);
  bool init(const std::string& windowTitle) { return init(windowTitle, Options()); }

  /** Runs the main loop until the window is closed or requestClose() is
   * called. `renderFrame` is called once per frame, after the dockspace has
   * been set up, to submit the application windows. */
  void run(const std::function<void()>& renderFrame);

  /** Destroys the contexts and the window. Safe to call more than once; also
   * called by the destructor. */
  void shutdown();

  /** run() returns after the current frame. */
  void requestClose();

  [[nodiscard]] bool isInitialized() const { return m_window != nullptr; }
  [[nodiscard]] GLFWwindow* windowHandle() const { return m_window; }

  void setWindowTitle(const std::string& title);

  /** Number of frames rendered so far by run(). */
  [[nodiscard]] uint64_t frameCount() const { return m_frameCount; }

  /** UI scale factor from the monitor content scale (1.0 on normal DPI). */
  [[nodiscard]] float dpiScale() const { return m_dpiScale; }

  /** Path of the `imgui.ini` file with the saved layout (empty if layout
   * persistence is disabled or no config directory is available). */
  [[nodiscard]] const std::string& iniFilePath() const { return m_iniPath; }

  /** ImGui ID of the main dockspace, valid after the first frame. */
  [[nodiscard]] unsigned int dockspaceId() const { return m_dockspaceId; }

  /** Rectangle (x, y, width, height), in ImGui coordinates, of the central
   * node of the dockspace in the last frame: the area not covered by docked
   * panels, where a background scene is visible. */
  [[nodiscard]] std::array<float, 4> centralNodeRect() const { return m_centralRect; }

  /** Saves the current window contents to an image file at the end of this
   * frame (or the next one, if called outside of a frame). */
  void requestScreenshot(const std::string& file) { m_pendingScreenshot = file; }

  /** Reads the window framebuffer contents now. Must be called from the GL
   * thread, between frames or from a renderFrame callback (in which case the
   * previous frame contents are read). */
  [[nodiscard]] std::optional<mrpt::img::CImage> grabFramebuffer() const;

  /** @name Hooks
   *  @{ */

  /** Called each frame inside ImGui::BeginMainMenuBar() / EndMainMenuBar(),
   * so the application menus go in the main menu bar. */
  std::function<void()> mainMenuBar;

  /** Called each frame before the dockspace is created. Windows that must
   * reduce the area of the dockspace go here, e.g. a toolbar or a timeline
   * pinned to an edge with ImGui::BeginViewportSideBar(). */
  std::function<void()> beforeDockspace;

  /** Called once, with the dockspace ID, the first time the application runs
   * with no saved layout, to dock its panels by default with
   * `ImGui::DockBuilderSplitNode()` and `ImGui::DockBuilderDockWindow()`. */
  std::function<void(unsigned int dockspaceId)> setupDefaultLayout;

  /** Called when the user tries to close the window. Return false to keep it
   * open (e.g. to ask about unsaved changes first). */
  std::function<bool()> onCloseRequest;

  /** Files dropped onto the window. */
  std::function<void(const std::vector<std::string>& files)> onFilesDropped;

  /** @} */

  /** Background (clear) color of the window, behind everything. */
  float clearColor[4] = {0.10f, 0.10f, 0.11f, 1.0f};

  /** Maximum frames to render before run() returns (0 = unlimited).
   * Initialized from the MRPT_IMGUI_MAX_FRAMES environment variable. */
  uint64_t maxFrames = 0;

 private:
  GLFWwindow* m_window = nullptr;
  std::string m_iniPath;
  bool m_imguiInitialized = false;
  bool m_closeRequested = false;
  uint64_t m_frameCount = 0;
  float m_dpiScale = 1.0f;
  unsigned int m_dockspaceId = 0;
  std::array<float, 4> m_centralRect{0, 0, 0, 0};
  std::string m_pendingScreenshot;
  std::string m_exitScreenshot;

  void buildDockspace();
  void handleScreenshotRequests(bool lastFrame);
  [[nodiscard]] bool shouldStop();
};

}  // namespace mrpt::imgui
