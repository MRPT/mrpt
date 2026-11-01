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

#include <mrpt/imgui/style.h>

#include <algorithm>
#include <cmath>

namespace mrpt::imgui
{
void applyStyle(float dpiScale)
{
  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();

  style.WindowRounding = 4.0f;
  style.ChildRounding = 3.0f;
  style.FrameRounding = 3.0f;
  style.PopupRounding = 3.0f;
  style.GrabRounding = 3.0f;
  style.TabRounding = 3.0f;
  style.ScrollbarRounding = 6.0f;
  style.WindowPadding = ImVec2(8, 8);
  style.FramePadding = ImVec2(6, 4);
  style.ItemSpacing = ImVec2(8, 5);
  style.WindowBorderSize = 1.0f;

  ImVec4* c = style.Colors;
  const ImVec4 accent(0.26f, 0.55f, 0.86f, 1.0f);
  c[ImGuiCol_WindowBg] = ImVec4(0.11f, 0.12f, 0.14f, 0.96f);
  c[ImGuiCol_ChildBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
  c[ImGuiCol_PopupBg] = ImVec4(0.11f, 0.12f, 0.14f, 0.98f);
  c[ImGuiCol_MenuBarBg] = ImVec4(0.14f, 0.15f, 0.18f, 1.0f);
  c[ImGuiCol_TitleBg] = ImVec4(0.09f, 0.10f, 0.12f, 1.0f);
  c[ImGuiCol_TitleBgActive] = ImVec4(0.13f, 0.20f, 0.30f, 1.0f);
  c[ImGuiCol_FrameBg] = ImVec4(0.18f, 0.20f, 0.23f, 1.0f);
  c[ImGuiCol_FrameBgHovered] = ImVec4(0.24f, 0.27f, 0.31f, 1.0f);
  c[ImGuiCol_FrameBgActive] = ImVec4(0.28f, 0.32f, 0.37f, 1.0f);
  c[ImGuiCol_Button] = ImVec4(0.20f, 0.23f, 0.27f, 1.0f);
  c[ImGuiCol_ButtonHovered] = ImVec4(accent.x, accent.y, accent.z, 0.70f);
  c[ImGuiCol_ButtonActive] = accent;
  c[ImGuiCol_Header] = ImVec4(accent.x, accent.y, accent.z, 0.35f);
  c[ImGuiCol_HeaderHovered] = ImVec4(accent.x, accent.y, accent.z, 0.60f);
  c[ImGuiCol_HeaderActive] = accent;
  c[ImGuiCol_CheckMark] = accent;
  c[ImGuiCol_SliderGrab] = accent;
  c[ImGuiCol_SliderGrabActive] = ImVec4(0.40f, 0.68f, 0.96f, 1.0f);
  c[ImGuiCol_Tab] = ImVec4(0.15f, 0.17f, 0.20f, 1.0f);
  c[ImGuiCol_TabHovered] = ImVec4(accent.x, accent.y, accent.z, 0.70f);
  c[ImGuiCol_TabSelected] = ImVec4(0.18f, 0.30f, 0.45f, 1.0f);
  c[ImGuiCol_DockingPreview] = ImVec4(accent.x, accent.y, accent.z, 0.50f);

  if (dpiScale > 0.0f && std::abs(dpiScale - 1.0f) > 1e-3f)
  {
    style.ScaleAllSizes(dpiScale);
  }
}

mrpt::img::TColor toTColor(const ImVec4& c)
{
  const auto u8 = [](float v)
  { return static_cast<uint8_t>(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f)); };
  return {u8(c.x), u8(c.y), u8(c.z), u8(c.w)};
}

ImVec2 regionSizeInPixels(const ImVec2& size)
{
  const ImVec2 s = ImGui::GetIO().DisplayFramebufferScale;
  return {std::max(1.0f, std::round(size.x * s.x)), std::max(1.0f, std::round(size.y * s.y))};
}

void tooltip(const char* text)
{
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort) && ImGui::BeginTooltip())
  {
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void helpMarker(const char* text)
{
  ImGui::TextDisabled("(?)");
  tooltip(text);
}

}  // namespace mrpt::imgui
