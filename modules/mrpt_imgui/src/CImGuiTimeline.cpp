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
#include <imgui_internal.h>  // BeginViewportSideBar()
#include <mrpt/imgui/CImGuiTimeline.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/imgui/style.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace mrpt::imgui
{
void CImGuiTimeline::setRange(double tMin, double tMax)
{
  if (tMax < tMin)
  {
    std::swap(tMin, tMax);
  }
  m_tMin = tMin;
  m_tMax = tMax;
  setTime(m_t);
}

void CImGuiTimeline::setKeyTimes(std::vector<double> times)
{
  std::sort(times.begin(), times.end());
  m_keys = std::move(times);
  if (!m_keys.empty() && m_tMax <= m_tMin)
  {
    setRange(m_keys.front(), m_keys.back());
  }
}

std::optional<size_t> CImGuiTimeline::currentKeyIndex() const
{
  if (m_keys.empty())
  {
    return std::nullopt;
  }
  const auto it = std::upper_bound(m_keys.begin(), m_keys.end(), m_t);
  if (it == m_keys.begin())
  {
    return 0;
  }
  return static_cast<size_t>(std::distance(m_keys.begin(), it) - 1);
}

void CImGuiTimeline::seekToKey(size_t index)
{
  if (m_keys.empty())
  {
    return;
  }
  setTime(m_keys[std::min(index, m_keys.size() - 1)]);
}

void CImGuiTimeline::setTime(double t)
{
  const double clamped = std::clamp(t, m_tMin, std::max(m_tMin, m_tMax));
  if (clamped == m_t)
  {
    return;
  }
  m_t = clamped;
  if (onTimeChanged)
  {
    onTimeChanged(m_t);
  }
}

void CImGuiTimeline::play()
{
  if (m_t >= m_tMax)
  {
    jumpToStart();
  }
  m_playing = m_tMax > m_tMin;
}

void CImGuiTimeline::pause() { m_playing = false; }

void CImGuiTimeline::togglePlay()
{
  if (m_playing)
  {
    pause();
  }
  else
  {
    play();
  }
}

void CImGuiTimeline::step(int n)
{
  m_playing = false;
  if (n == 0)
  {
    return;
  }
  if (m_keys.empty())
  {
    setTime(m_t + n * stepSize);
    return;
  }
  // From the current key, move n keys. If the current time is between keys
  // (e.g. while playing), a step forward goes to the next one.
  const auto idx = static_cast<long long>(*currentKeyIndex());
  long long target = idx + n;
  if (n < 0 && m_t > m_keys[static_cast<size_t>(idx)])
  {
    target = idx + n + 1;
  }
  target = std::clamp<long long>(target, 0, static_cast<long long>(m_keys.size()) - 1);
  setTime(m_keys[static_cast<size_t>(target)]);
}

void CImGuiTimeline::jumpToStart()
{
  m_playing = false;
  setTime(m_tMin);
}

void CImGuiTimeline::jumpToEnd()
{
  m_playing = false;
  setTime(m_tMax);
}

bool CImGuiTimeline::update(double wallDt)
{
  if (!m_playing || wallDt <= 0)
  {
    return false;
  }
  const double old = m_t;
  double next = m_t + wallDt * speed;
  if (next >= m_tMax)
  {
    if (loop && m_tMax > m_tMin)
    {
      next = m_tMin + std::fmod(next - m_tMin, m_tMax - m_tMin);
    }
    else
    {
      next = m_tMax;
      m_playing = false;
    }
  }
  setTime(next);
  return m_t != old;
}

std::vector<int> CImGuiTimeline::tickColumns(
    const std::vector<double>& times, double tMin, double tMax, int widthPx)
{
  std::vector<int> cols;
  if (widthPx <= 0 || tMax <= tMin)
  {
    return cols;
  }
  std::vector<bool> used(static_cast<size_t>(widthPx), false);
  const double k = (widthPx - 1) / (tMax - tMin);
  for (const double t : times)
  {
    if (t < tMin || t > tMax)
    {
      continue;
    }
    const auto c = static_cast<size_t>(std::lround((t - tMin) * k));
    used[std::min(c, used.size() - 1)] = true;
  }
  for (size_t i = 0; i < used.size(); i++)
  {
    if (used[i])
    {
      cols.push_back(static_cast<int>(i));
    }
  }
  return cols;
}

std::string CImGuiTimeline::timeLabel() const
{
  if (timeFormatter)
  {
    return timeFormatter(m_t);
  }
  std::array<char, 64> buf{};
  std::snprintf(buf.data(), buf.size(), "t = %.3f s", m_t - m_tMin);
  return buf.data();
}

void CImGuiTimeline::handleKeyboard()
{
  if (!keyboardShortcuts)
  {
    return;
  }
  const ImGuiIO& io = ImGui::GetIO();
  if (io.WantTextInput || ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId))
  {
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Space, false))
  {
    togglePlay();
  }
  if (ImGui::IsKeyPressed(ImGuiKey_RightArrow))
  {
    step(1);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow))
  {
    step(-1);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_PageDown))
  {
    step(10);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_PageUp))
  {
    step(-10);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Home, false))
  {
    jumpToStart();
  }
  if (ImGui::IsKeyPressed(ImGuiKey_End, false))
  {
    jumpToEnd();
  }
}

float CImGuiTimeline::barHeight() const
{
  const ImGuiStyle& st = ImGui::GetStyle();
  const auto rows = static_cast<float>(std::max<size_t>(1, m_tracks.size()));
  const float trackH = std::min(6.0f, 24.0f / rows) * rows;
  return ImGui::GetFrameHeight() * 2.0f + trackH + st.WindowPadding.y * 2.0f + st.ItemSpacing.y;
}

bool CImGuiTimeline::render()
{
  const double tBefore = m_t;

  handleKeyboard();
  update(ImGui::GetIO().DeltaTime);

  ImGui::PushID(this);

  // --- Transport buttons ---
  if (ImGui::Button(ICON_MS_SKIP_PREVIOUS))
  {
    jumpToStart();
  }
  tooltip("Jump to start (Home)");
  ImGui::SameLine();
  if (ImGui::Button(ICON_MS_FAST_REWIND))
  {
    step(-1);
  }
  tooltip("Step back (Left arrow)");
  ImGui::SameLine();
  if (ImGui::Button(m_playing ? ICON_MS_PAUSE : ICON_MS_PLAY_ARROW))
  {
    togglePlay();
  }
  tooltip("Play / pause (Space)");
  ImGui::SameLine();
  if (ImGui::Button(ICON_MS_FAST_FORWARD))
  {
    step(1);
  }
  tooltip("Step forward (Right arrow)");
  ImGui::SameLine();
  if (ImGui::Button(ICON_MS_SKIP_NEXT))
  {
    jumpToEnd();
  }
  tooltip("Jump to end (End)");
  ImGui::SameLine();

  // --- Speed ---
  {
    static constexpr std::array<double, 9> speeds = {0.1, 0.25, 0.5, 1, 2, 5, 10, 20, 50};
    std::array<char, 32> preview{};
    std::snprintf(preview.data(), preview.size(), "x%g", speed);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4.5f);
    if (ImGui::BeginCombo("##speed", preview.data()))
    {
      for (const double s : speeds)
      {
        std::array<char, 32> lbl{};
        std::snprintf(lbl.data(), lbl.size(), "x%g", s);
        if (ImGui::Selectable(lbl.data(), s == speed))
        {
          speed = s;
        }
      }
      ImGui::EndCombo();
    }
    tooltip("Playback speed");
    ImGui::SameLine();
    ImGui::Checkbox(ICON_MS_REPEAT "##loop", &loop);
    tooltip("Loop");
  }

  // --- Readout ---
  ImGui::SameLine();
  std::string label = timeLabel();
  if (const auto idx = currentKeyIndex(); idx)
  {
    label += "   #" + std::to_string(*idx) + " / " + std::to_string(m_keys.size());
  }
  ImGui::TextUnformatted(label.c_str());

  // --- Axis with tick tracks and playhead ---
  const ImVec2 p0 = ImGui::GetCursorScreenPos();
  const float width = std::max(10.0f, ImGui::GetContentRegionAvail().x);
  size_t visibleTracks = 0;
  for (const auto& tr : m_tracks)
  {
    visibleTracks += tr.visible ? 1 : 0;
  }
  const float rowH =
      visibleTracks == 0 ? 0.0f : std::min(6.0f, 24.0f / static_cast<float>(visibleTracks));
  const float axisH = ImGui::GetFrameHeight() + rowH * static_cast<float>(visibleTracks);

  ImGui::InvisibleButton("##axis", ImVec2(width, axisH));
  const bool hovered = ImGui::IsItemHovered();
  const double span = std::max(1e-12, m_tMax - m_tMin);
  const auto xToTime = [&](float x)
  { return m_tMin + span * std::clamp<double>((x - p0.x) / width, 0.0, 1.0); };

  if (ImGui::IsItemActive() && ImGui::IsMouseDown(ImGuiMouseButton_Left))
  {
    m_playing = false;
    setTime(xToTime(ImGui::GetIO().MousePos.x));
  }

  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImU32 bgCol = ImGui::GetColorU32(ImGuiCol_FrameBg);
  dl->AddRectFilled(p0, ImVec2(p0.x + width, p0.y + axisH), bgCol, 3.0f);

  // Track ticks:
  float y = p0.y + 2.0f;
  for (const auto& tr : m_tracks)
  {
    if (!tr.visible)
    {
      continue;
    }
    const ImU32 col = toImU32(tr.color);
    for (const int c : tickColumns(tr.times, m_tMin, m_tMax, static_cast<int>(width)))
    {
      const float x = p0.x + static_cast<float>(c);
      dl->AddLine(ImVec2(x, y), ImVec2(x, y + std::max(1.0f, rowH - 1.0f)), col);
    }
    y += rowH;
  }

  // Playhead:
  if (m_tMax > m_tMin)
  {
    const float x = p0.x + static_cast<float>((m_t - m_tMin) / span) * width;
    const ImU32 col = ImGui::GetColorU32(ImGuiCol_SliderGrabActive);
    dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p0.y + axisH), col, 2.0f);
    dl->AddTriangleFilled(ImVec2(x - 5, p0.y), ImVec2(x + 5, p0.y), ImVec2(x, p0.y + 6), col);
  }

  // Hover: time under the mouse and track names
  if (hovered && ImGui::BeginTooltip())
  {
    const double th = xToTime(ImGui::GetIO().MousePos.x);
    if (timeFormatter)
    {
      ImGui::TextUnformatted(timeFormatter(th).c_str());
    }
    else
    {
      ImGui::Text("t = %.3f s", th - m_tMin);
    }
    for (const auto& tr : m_tracks)
    {
      if (tr.visible)
      {
        ImGui::TextColored(toImVec4(tr.color), "%s (%zu)", tr.name.c_str(), tr.times.size());
      }
    }
    ImGui::EndTooltip();
  }

  ImGui::PopID();
  return m_t != tBefore;
}

bool CImGuiTimeline::renderAsBottomBar(const char* name)
{
  constexpr ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar;
  bool changed = false;
  if (ImGui::BeginViewportSideBar(
          name, ImGui::GetMainViewport(), ImGuiDir_Down, barHeight(), flags))
  {
    changed = render();
  }
  ImGui::End();
  return changed;
}

}  // namespace mrpt::imgui
