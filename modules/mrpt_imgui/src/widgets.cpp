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

// clang-format off
#include <mrpt/opengl/opengl_api.h>
// clang-format on

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>  // BeginViewportSideBar()
#include <implot.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/imgui/widgets.h>
#include <mrpt/system/os.h>

namespace mrpt::imgui
{
void CImGuiMessageQueue::push(Kind kind, const std::string& title, const std::string& text)
{
  m_queue.push_back({kind, title, text});
}

void CImGuiMessageQueue::render()
{
  if (m_queue.empty())
  {
    return;
  }
  constexpr const char* popupId = "##mrpt_message_box";
  if (!m_popupOpened)
  {
    ImGui::OpenPopup(popupId);
    m_popupOpened = true;
  }
  const Msg& m = m_queue.front();
  const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  if (ImGui::BeginPopupModal(popupId, nullptr, ImGuiWindowFlags_AlwaysAutoResize))
  {
    const char* icon = m.kind == Kind::Error     ? ICON_MS_ERROR
                       : m.kind == Kind::Warning ? ICON_MS_WARNING
                                                 : ICON_MS_INFO;
    const ImVec4 col = m.kind == Kind::Error     ? ImVec4(1.0f, 0.45f, 0.45f, 1.0f)
                       : m.kind == Kind::Warning ? ImVec4(1.0f, 0.8f, 0.3f, 1.0f)
                                                 : ImVec4(0.5f, 0.75f, 1.0f, 1.0f);
    ImGui::TextColored(col, "%s %s", icon, m.title.c_str());
    ImGui::Separator();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 40.0f);
    ImGui::TextUnformatted(m.text.c_str());
    ImGui::PopTextWrapPos();
    ImGui::Spacing();
    // Keys are ignored in the frame the message appears, so the key press
    // that caused it does not close it at once (and without repeat, holding
    // a key does not close several messages):
    const bool keyClose =
        !ImGui::IsWindowAppearing() &&
        (ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_Escape, false));
    if (ImGui::Button("OK", ImVec2(ImGui::GetFontSize() * 6, 0)) || keyClose)
    {
      ImGui::CloseCurrentPopup();
      m_queue.pop_front();
      m_popupOpened = false;
    }
    ImGui::SetItemDefaultFocus();
    ImGui::EndPopup();
  }
  else
  {
    m_popupOpened = false;
  }
}

std::string CImGuiAboutDialog::informationText() const
{
  std::string s = appName + "\n";
  s += "Part of the MRPT project: https://www.mrpt.org/\n";
  s += "Bug reports and source code: https://github.com/MRPT/mrpt\n\n";
  if (!description.empty())
  {
    s += description + "\n\n";
  }
  s += "MRPT version:      " + mrpt::system::MRPT_getVersion() + "\n";
  s += "MRPT build date:   " + mrpt::system::MRPT_getCompilationDate() + "\n";
  s += std::string("Dear ImGui:        ") + IMGUI_VERSION + "\n";
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (glfwGetCurrentContext() == nullptr)
  {
    return s;
  }
  if (const auto* r = glGetString(GL_RENDERER); r != nullptr)
  {
    s += "OpenGL renderer:   " + std::string(reinterpret_cast<const char*>(r)) + "\n";
  }
  if (const auto* v = glGetString(GL_VERSION); v != nullptr)
  {
    s += "OpenGL version:    " + std::string(reinterpret_cast<const char*>(v)) + "\n";
  }
#endif
  return s;
}

void CImGuiAboutDialog::render()
{
  constexpr const char* popupId = "About##mrpt_about";
  if (m_requestOpen)
  {
    m_requestOpen = false;
    ImGui::OpenPopup(popupId);
  }
  const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(
      ImVec2(ImGui::GetFontSize() * 40, ImGui::GetFontSize() * 26), ImGuiCond_Appearing);
  if (!ImGui::BeginPopupModal(popupId, nullptr))
  {
    return;
  }
  ImGui::Text("%s %s", ICON_MS_INFO, appName.c_str());
  ImGui::Separator();

  const float bottom = ImGui::GetFrameHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y;
  if (ImGui::BeginTabBar("##about_tabs"))
  {
    const auto textTab = [&](const char* title, const std::string& text)
    {
      if (ImGui::BeginTabItem(title))
      {
        ImGui::BeginChild("##txt", ImVec2(0, -bottom), ImGuiChildFlags_Borders);
        ImGui::TextUnformatted(text.c_str());
        ImGui::EndChild();
        ImGui::EndTabItem();
      }
    };
    textTab("Information", informationText());
    for (const auto& [title, text] : extraTabs)
    {
      textTab(title.c_str(), text);
    }
    textTab("License", mrpt::system::getMRPTLicense());
    ImGui::EndTabBar();
  }
  if (ImGui::Button("Close") || ImGui::IsKeyPressed(ImGuiKey_Escape))
  {
    ImGui::CloseCurrentPopup();
  }
  ImGui::EndPopup();
}

bool beginStatusBar(const char* name)
{
  constexpr ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar;
  const ImGuiStyle& st = ImGui::GetStyle();
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(st.WindowPadding.x, 2.0f));
  const bool open = ImGui::BeginViewportSideBar(
      name, ImGui::GetMainViewport(), ImGuiDir_Down, ImGui::GetTextLineHeight() + 4.0f, flags);
  ImGui::PopStyleVar();
  return open;
}

void endStatusBar() { ImGui::End(); }

void keyValueTable(const char* id, const std::vector<std::pair<std::string, std::string>>& rows)
{
  if (ImGui::BeginTable(id, 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV))
  {
    for (const auto& [k, v] : rows)
    {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(k.c_str());
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(v.c_str());
    }
    ImGui::EndTable();
  }
}

}  // namespace mrpt::imgui
