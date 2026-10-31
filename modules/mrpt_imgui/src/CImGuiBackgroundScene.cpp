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

#include <imgui.h>
#include <imgui_internal.h>  // BringWindowToDisplayBack()
#include <mrpt/imgui/CImGuiAppShell.h>
#include <mrpt/imgui/CImGuiBackgroundScene.h>

namespace mrpt::imgui
{
void CImGuiBackgroundScene::render(const CImGuiAppShell& shell) { render(shell.centralNodeRect()); }

void CImGuiBackgroundScene::render(const std::array<float, 4>& rect)
{
  if (rect[2] < 1.0f || rect[3] < 1.0f)
  {
    return;
  }
  ImGui::SetNextWindowPos(ImVec2(rect[0], rect[1]));
  ImGui::SetNextWindowSize(ImVec2(rect[2], rect[3]));
  ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
  ImGui::SetNextWindowBgAlpha(0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  constexpr ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
      ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings |
      ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoFocusOnAppearing;
  const bool visible = ImGui::Begin(m_windowName.c_str(), nullptr, flags);
  ImGui::PopStyleVar(2);
  if (visible)
  {
    // Keep it behind any floating panel, even right after being created:
    ImGui::BringWindowToDisplayBack(ImGui::GetCurrentWindow());
    view.renderAsBackground();
  }
  ImGui::End();
}

}  // namespace mrpt::imgui
