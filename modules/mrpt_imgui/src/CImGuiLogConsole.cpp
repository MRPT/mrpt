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
#include <imgui_stdlib.h>
#include <mrpt/imgui/CImGuiLogConsole.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/imgui/style.h>
#include <mrpt/system/datetime.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace mrpt::imgui
{
namespace
{
std::string toLower(std::string s)
{
  std::transform(
      s.begin(), s.end(), s.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

const char* levelName(mrpt::system::VerbosityLevel l)
{
  switch (l)
  {
    case mrpt::system::LVL_DEBUG:
      return "DEBUG";
    case mrpt::system::LVL_INFO:
      return "INFO";
    case mrpt::system::LVL_WARN:
      return "WARN";
    case mrpt::system::LVL_ERROR:
      return "ERROR";
    default:
      return "?";
  }
}

ImVec4 levelColor(mrpt::system::VerbosityLevel l)
{
  switch (l)
  {
    case mrpt::system::LVL_DEBUG:
      return {0.6f, 0.6f, 0.6f, 1.0f};
    case mrpt::system::LVL_WARN:
      return {1.0f, 0.8f, 0.3f, 1.0f};
    case mrpt::system::LVL_ERROR:
      return {1.0f, 0.4f, 0.4f, 1.0f};
    case mrpt::system::LVL_INFO:
    default:
      return {0.9f, 0.9f, 0.9f, 1.0f};
  }
}
}  // namespace

bool CImGuiLogConsole::Filter::matches(const Entry& e) const
{
  if (e.level < minLevel)
  {
    return false;
  }
  if (hiddenSources.count(e.source) != 0)
  {
    return false;
  }
  if (!text.empty())
  {
    const auto needle = toLower(text);
    if (toLower(e.text).find(needle) == std::string::npos &&
        toLower(e.source).find(needle) == std::string::npos)
    {
      return false;
    }
  }
  return true;
}

CImGuiLogConsole::~CImGuiLogConsole()
{
  for (auto& [logger, id] : m_attached)
  {
    logger->logDeregisterCallback(id);
  }
}

void CImGuiLogConsole::push(
    mrpt::system::VerbosityLevel level,
    const std::string& source,
    const std::string& text,
    mrpt::Clock::time_point timestamp)
{
  std::lock_guard<std::mutex> lck(m_mtx);
  std::istringstream ss(text);
  std::string line;
  bool any = false;
  while (std::getline(ss, line))
  {
    if (!line.empty() && line.back() == '\r')
    {
      line.pop_back();
    }
    m_entries.push_back({timestamp, level, source, line});
    any = true;
  }
  if (!any)
  {
    m_entries.push_back({timestamp, level, source, std::string()});
  }
  m_sources.insert(source);
  const size_t cap = maxEntries.load();
  while (m_entries.size() > cap)
  {
    m_entries.pop_front();
  }
  m_version++;
}

void CImGuiLogConsole::attach(mrpt::system::COutputLogger& logger)
{
  const auto id = logger.logRegisterCallback(
      [this](
          std::string_view msg, const mrpt::system::VerbosityLevel level,
          std::string_view loggerName, const mrpt::Clock::time_point timestamp)
      {
        std::string text(msg);
        // COutputLogger messages carry a trailing newline:
        while (!text.empty() && (text.back() == '\n' || text.back() == '\r'))
        {
          text.pop_back();
        }
        push(level, std::string(loggerName), text, timestamp);
      });
  m_attached.emplace_back(&logger, id);
}

void CImGuiLogConsole::detach(mrpt::system::COutputLogger& logger)
{
  for (auto it = m_attached.begin(); it != m_attached.end();)
  {
    if (it->first == &logger)
    {
      logger.logDeregisterCallback(it->second);
      it = m_attached.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void CImGuiLogConsole::clear()
{
  std::lock_guard<std::mutex> lck(m_mtx);
  m_entries.clear();
  m_version++;
}

size_t CImGuiLogConsole::size() const
{
  std::lock_guard<std::mutex> lck(m_mtx);
  return m_entries.size();
}

std::vector<std::string> CImGuiLogConsole::sources() const
{
  std::lock_guard<std::mutex> lck(m_mtx);
  return {m_sources.begin(), m_sources.end()};
}

std::vector<CImGuiLogConsole::Entry> CImGuiLogConsole::filtered() const
{
  std::lock_guard<std::mutex> lck(m_mtx);
  std::vector<Entry> out;
  for (const auto& e : m_entries)
  {
    if (filter.matches(e))
    {
      out.push_back(e);
    }
  }
  return out;
}

std::string CImGuiLogConsole::format(const Entry& e) const
{
  std::string s;
  if (showTimestamps)
  {
    s += "[" + mrpt::system::timeLocalToString(e.timestamp, 3) + "] ";
  }
  s += "[";
  s += levelName(e.level);
  s += "] ";
  if (showSources && !e.source.empty())
  {
    s += "[" + e.source + "] ";
  }
  s += e.text;
  return s;
}

std::string CImGuiLogConsole::filteredAsText() const
{
  std::string s;
  for (const auto& e : filtered())
  {
    s += format(e);
    s += "\n";
  }
  return s;
}

void CImGuiLogConsole::render()
{
  ImGui::PushID(this);

  // --- Toolbar ---
  {
    static constexpr const char* levels[] = {"Debug", "Info", "Warning", "Error"};
    int lvl = static_cast<int>(filter.minLevel);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
    if (ImGui::Combo("##level", &lvl, levels, 4))
    {
      filter.minLevel = static_cast<mrpt::system::VerbosityLevel>(lvl);
    }
    tooltip("Minimum level shown");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MS_FILTER_LIST " Sources"))
    {
      ImGui::OpenPopup("##sources");
    }
    if (ImGui::BeginPopup("##sources"))
    {
      for (const auto& src : sources())
      {
        bool shown = filter.hiddenSources.count(src) == 0;
        ImGui::PushID(src.c_str());
        if (ImGui::Checkbox(src.empty() ? "(unnamed)" : src.c_str(), &shown))
        {
          if (shown)
          {
            filter.hiddenSources.erase(src);
          }
          else
          {
            filter.hiddenSources.insert(src);
          }
        }
        ImGui::PopID();
      }
      ImGui::EndPopup();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
    ImGui::InputTextWithHint("##text", ICON_MS_SEARCH " Filter", &filter.text);
    ImGui::SameLine();
    ImGui::Checkbox("Autoscroll", &autoScroll);
    ImGui::SameLine();
    if (ImGui::Button(ICON_MS_DELETE))
    {
      clear();
    }
    tooltip("Clear");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MS_SAVE))
    {
      m_saveDialog.open(
          CImGuiFileDialog::Mode::Save, "Save log",
          {
              {"Text files", {"txt", "log"}}
      });
    }
    tooltip("Save the shown messages to a file");
  }
  if (auto r = m_saveDialog.poll(); r && !r->empty())
  {
    std::ofstream f(r->front());
    f << filteredAsText();
  }

  ImGui::Separator();

  // --- Messages (cached filtered copy, rebuilt only on changes) ---
  std::string filterKey = std::to_string(static_cast<int>(filter.minLevel)) + "|" + filter.text;
  for (const auto& s : filter.hiddenSources)
  {
    filterKey += "|" + s;
  }
  if (m_cacheVersion != version() || m_cacheFilterKey != filterKey)
  {
    m_cacheVersion = version();
    m_cacheFilterKey = filterKey;
    m_cache = filtered();
  }

  if (ImGui::BeginChild(
          "##log", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar))
  {
    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(m_cache.size()));
    while (clipper.Step())
    {
      for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++)
      {
        const auto& e = m_cache[static_cast<size_t>(i)];
        ImGui::PushStyleColor(ImGuiCol_Text, levelColor(e.level));
        ImGui::TextUnformatted(format(e).c_str());
        ImGui::PopStyleColor();
      }
    }
    if (autoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f)
    {
      ImGui::SetScrollHereY(1.0f);
    }
  }
  ImGui::EndChild();

  ImGui::PopID();
}

}  // namespace mrpt::imgui
