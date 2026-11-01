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
#include <mrpt/imgui/SimpleFileDialog.h>
#include <mrpt/imgui/icons.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <system_error>

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
}  // namespace

bool fileMatchesFilter(const std::string& fileName, const FileDialogFilter& filter)
{
  if (filter.extensions.empty())
  {
    return true;
  }
  const std::string name = toLower(fileName);
  for (const auto& ext : filter.extensions)
  {
    if (ext.empty() || ext == "*")
    {
      return true;
    }
    const std::string suffix = "." + toLower(ext);
    if (name.size() > suffix.size() &&
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      return true;
    }
  }
  return false;
}

std::vector<std::string> toNativeFilterList(const std::vector<FileDialogFilter>& filters)
{
  std::vector<std::string> out;
  bool hasAll = false;
  for (const auto& f : filters)
  {
    std::string patterns;
    for (const auto& ext : f.extensions)
    {
      if (!patterns.empty())
      {
        patterns += " ";
      }
      patterns += (ext.empty() || ext == "*") ? std::string("*") : "*." + ext;
    }
    if (patterns.empty())
    {
      patterns = "*";
    }
    if (patterns == "*")
    {
      hasAll = true;
    }
    out.push_back(f.description + " (" + patterns + ")");
    out.push_back(patterns);
  }
  if (!hasAll)
  {
    out.emplace_back("All files (*)");
    out.emplace_back("*");
  }
  return out;
}

void SimpleFileDialog::open(
    Mode mode,
    const std::vector<FileDialogFilter>& filters,
    const std::string& title,
    const std::string& initialPath)
{
  namespace fs = std::filesystem;

  m_mode = mode;
  m_filters = filters;
  m_selectedFilterIdx = 0;
  if (!title.empty())
  {
    m_title = title;
  }
  else
  {
    m_title = mode == Mode::Open ? "Open file" : mode == Mode::Save ? "Save file" : "Select folder";
  }
  m_typedName.clear();
  m_errorMsg.clear();

  std::error_code ec;
  fs::path startDir;
  if (!initialPath.empty())
  {
    const fs::path p(initialPath);
    if (fs::is_directory(p, ec))
    {
      startDir = p;
    }
    else
    {
      startDir = p.parent_path();
      if (mode == Mode::Save)
      {
        m_typedName = p.filename().string();
      }
    }
  }
  if (startDir.empty() || !fs::is_directory(startDir, ec))
  {
    startDir = !m_currentDir.empty() ? fs::path(m_currentDir) : fs::current_path(ec);
  }
  if (!fs::is_directory(startDir, ec))
  {
    startDir = fs::current_path(ec);
  }
  m_currentDir = startDir.string();

  if (m_popupId.empty())
  {
    // Unique per instance, so two dialogs never share an ImGui popup ID:
    const auto instanceTag = std::to_string(reinterpret_cast<uintptr_t>(this));
    m_popupId = "Files##popup" + instanceTag;
    m_overwritePopupId = "Overwrite file?##overwrite" + instanceTag;
  }

  m_open = true;
  m_pendingOpen = true;
}

std::optional<std::string> SimpleFileDialog::render()
{
  namespace fs = std::filesystem;

  if (!m_open)
  {
    return std::nullopt;
  }

  // Popup IDs are scoped by the current window, so the popup is opened here,
  // next to BeginPopupModal(), and not in open():
  if (m_pendingOpen)
  {
    m_pendingOpen = false;
    ImGui::OpenPopup(m_popupId.c_str());
  }

  std::optional<std::string> result;

  ImGui::SetNextWindowSize(ImVec2(600, 440), ImGuiCond_FirstUseEver);
  if (ImGui::BeginPopupModal(m_popupId.c_str(), nullptr, ImGuiWindowFlags_NoSavedSettings))
  {
    ImGui::TextUnformatted(m_title.c_str());
    ImGui::Separator();

    if (ImGui::Button(ICON_MS_ARROW_UPWARD " Up"))
    {
      const fs::path p(m_currentDir);
      if (p.has_parent_path() && p.parent_path() != p)
      {
        m_currentDir = p.parent_path().string();
      }
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1);
    std::string dirEdit = m_currentDir;
    if (ImGui::InputText("##dir", &dirEdit, ImGuiInputTextFlags_EnterReturnsTrue))
    {
      std::error_code ec;
      if (fs::is_directory(dirEdit, ec))
      {
        m_currentDir = dirEdit;
      }
    }

    ImGui::Separator();

    const bool folderMode = m_mode == Mode::SelectFolder;

    if (ImGui::BeginChild("##fileList", ImVec2(0, -80), ImGuiChildFlags_Borders))
    {
      std::error_code ec;
      std::vector<fs::directory_entry> dirs;
      std::vector<fs::directory_entry> files;

      fs::directory_iterator dirIt(m_currentDir, ec);
      const fs::directory_iterator dirEnd;
      for (; !ec && dirIt != dirEnd; dirIt.increment(ec))
      {
        const auto& entry = *dirIt;
        std::error_code statusEc;
        const bool isDir = entry.is_directory(statusEc);
        if (statusEc)
        {
          continue;  // e.g. a broken symlink
        }
        if (isDir)
        {
          dirs.push_back(entry);
        }
        else if (
            !folderMode &&
            (m_filters.empty() || static_cast<size_t>(m_selectedFilterIdx) >= m_filters.size() ||
             fileMatchesFilter(
                 entry.path().filename().string(),
                 m_filters[static_cast<size_t>(m_selectedFilterIdx)])))
        {
          files.push_back(entry);
        }
      }
      m_errorMsg = ec ? "Unable to read directory: " + ec.message() : std::string();

      const auto byName = [](const auto& a, const auto& b)
      { return a.path().filename() < b.path().filename(); };
      std::sort(dirs.begin(), dirs.end(), byName);
      std::sort(files.begin(), files.end(), byName);

      for (const auto& d : dirs)
      {
        const std::string label = std::string(ICON_MS_FOLDER " ") + d.path().filename().string();
        if (ImGui::Selectable(label.c_str(), false, ImGuiSelectableFlags_AllowDoubleClick) &&
            ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
        {
          m_currentDir = d.path().string();
        }
      }
      for (const auto& f : files)
      {
        const std::string name = f.path().filename().string();
        const bool isSelected = name == m_typedName;
        if (ImGui::Selectable(name.c_str(), isSelected, ImGuiSelectableFlags_AllowDoubleClick))
        {
          m_typedName = name;
          // Only Open confirms on double-click; Save always goes through its
          // button and the overwrite check.
          if (m_mode == Mode::Open && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
          {
            result = (fs::path(m_currentDir) / name).string();
          }
        }
      }
    }
    ImGui::EndChild();

    if (!folderMode && !m_filters.empty())
    {
      const std::string& preview = m_filters[static_cast<size_t>(m_selectedFilterIdx)].description;
      if (ImGui::BeginCombo("Filter", preview.c_str()))
      {
        for (size_t i = 0; i < m_filters.size(); i++)
        {
          const bool isSelected = static_cast<int>(i) == m_selectedFilterIdx;
          if (ImGui::Selectable(m_filters[i].description.c_str(), isSelected))
          {
            m_selectedFilterIdx = static_cast<int>(i);
          }
        }
        ImGui::EndCombo();
      }
    }

    if (!folderMode)
    {
      ImGui::SetNextItemWidth(-1);
      ImGui::InputTextWithHint("##fileName", "File name", &m_typedName);
    }

    if (!m_errorMsg.empty())
    {
      ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", m_errorMsg.c_str());
    }

    const char* confirmLabel = m_mode == Mode::Open   ? "Open"
                               : m_mode == Mode::Save ? "Save"
                                                      : "Select this folder";
    if (ImGui::Button(confirmLabel))
    {
      if (folderMode)
      {
        result = m_currentDir;
      }
      else if (m_typedName.empty())
      {
        m_errorMsg = "Please select or type a file name.";
      }
      else
      {
        const auto candidate = fs::path(m_currentDir) / m_typedName;
        std::error_code candidateEc;
        if (m_mode == Mode::Open)
        {
          if (!fs::is_regular_file(candidate, candidateEc))
          {
            m_errorMsg = "File does not exist or is not a regular file.";
          }
          else
          {
            result = candidate.string();
          }
        }
        else if (fs::exists(candidate, candidateEc))
        {
          // Unlike native dialogs, this one has no built-in overwrite check:
          m_pendingOverwritePath = candidate.string();
          ImGui::OpenPopup(m_overwritePopupId.c_str());
        }
        else
        {
          result = candidate.string();
        }
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
      m_open = false;
      ImGui::CloseCurrentPopup();
    }

    if (ImGui::BeginPopupModal(
            m_overwritePopupId.c_str(), nullptr,
            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings))
    {
      ImGui::Text("File already exists:\n%s\n\nOverwrite it?", m_pendingOverwritePath.c_str());
      if (ImGui::Button("Overwrite"))
      {
        result = m_pendingOverwritePath;
        ImGui::CloseCurrentPopup();
      }
      ImGui::SameLine();
      if (ImGui::Button("Cancel"))
      {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }

    if (result.has_value())
    {
      m_open = false;
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
  else if (!m_pendingOpen)
  {
    // Dismissed by other means (e.g. the Esc key).
    m_open = false;
  }

  return result;
}

}  // namespace mrpt::imgui
