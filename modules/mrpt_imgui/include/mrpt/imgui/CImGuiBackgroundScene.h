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

#include <mrpt/imgui/CImGuiSceneView.h>

#include <array>
#include <string>

namespace mrpt::imgui
{
class CImGuiAppShell;

/** A 3D scene drawn as the background of the application window, behind all
 * the ImGui panels, with orbit/pan/zoom camera control where no panel covers
 * it. This is the usual main view of simulators and viewers built on
 * CImGuiAppShell: panels float or dock on top of the 3D world.
 *
 * \code
 *   mrpt::imgui::CImGuiBackgroundScene bg;
 *   bg.view.setScene(myScene);
 *   app.run([&]() {
 *     bg.render(app);   // first, so it stays behind the panels
 *     ... panels ...
 *   });
 * \endcode
 *
 * The scene is rendered straight into the window framebuffer (no FBO), with
 * its MSAA, through CImGuiSceneView::renderAsBackground().
 * \ingroup mrpt_imgui_grp
 */
class CImGuiBackgroundScene
{
 public:
  CImGuiBackgroundScene() = default;

  /** The scene, its camera controller, and the overlay/click callbacks. */
  CImGuiSceneView view;

  /** Renders the scene over the central node of the shell dockspace (the
   * area not covered by docked panels). Call it every frame, before any
   * other window. */
  void render(const CImGuiAppShell& shell);

  /** Renders the scene over the given rectangle (x, y, width, height), in
   * ImGui coordinates. */
  void render(const std::array<float, 4>& rect);

  /** True if the mouse was over the visible part of the scene in the last
   * frame (i.e. not over any panel). */
  [[nodiscard]] bool isHovered() const { return view.isHovered(); }

 private:
  std::string m_windowName = "##mrpt_background_scene";
};

}  // namespace mrpt::imgui
