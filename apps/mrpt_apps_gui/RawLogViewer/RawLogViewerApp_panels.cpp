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

// The dockable panels of RawLogViewerApp.

#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_stdlib.h>
#include <implot.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/img/CImage.h>
#include <mrpt/imgui/style.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

#include <algorithm>
#include <cctype>
#include <limits>

#include "RawLogViewerApp.h"
#include "RawLogViewerApp_common.h"

using mrpt::imgui::CImGuiFileDialog;
using mrpt::imgui::helpMarker;
using mrpt::imgui::tooltip;
using namespace mrpt::obs;
using namespace rlv;

namespace
{
constexpr ImGuiTableFlags TABLE_FLAGS = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                        ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;

/** Class name without the "mrpt::obs::" namespace. */
std::string shortClassName(const char* name)
{
  std::string s(name);
  const std::string ns = "mrpt::obs::";
  if (s.rfind(ns, 0) == 0)
  {
    s = s.substr(ns.size());
  }
  return s;
}

std::string toLower(std::string s)
{
  std::transform(
      s.begin(), s.end(), s.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

/** Shows the "busy" placeholder; returns true if busy. */
bool busyPlaceholder(bool busy)
{
  if (busy)
  {
    ImGui::TextDisabled(ICON_MS_SCHEDULE " Working...");
  }
  return busy;
}

/** Combo to pick a sensor label. Returns true if it changed. */
bool sensorCombo(const char* id, const RawlogDocument& doc, std::string& label)
{
  bool changed = false;
  if (ImGui::BeginCombo(id, label.empty() ? "(select)" : label.c_str()))
  {
    for (const auto& [l, s] : doc.sensors())
    {
      if (ImGui::Selectable(l.empty() ? "(no label)" : l.c_str(), l == label))
      {
        label = l;
        changed = true;
      }
    }
    ImGui::EndCombo();
  }
  return changed;
}

/** Two ints [first, last] within [0, maxIndex]. */
void rangeEditor(const char* label, int* range, size_t count)
{
  const int maxIdx = count == 0 ? 0 : static_cast<int>(count - 1);
  ImGui::DragInt2(label, range, 1.0f, 0, maxIdx);
  range[0] = std::clamp(range[0], 0, maxIdx);
  range[1] = std::clamp(range[1], range[0], maxIdx);
}
}  // namespace

// ---------------------------------------------------------------------------
// Tree
// ---------------------------------------------------------------------------
void RawLogViewerApp::rebuildTreeRows()
{
  m_treeDirty = false;
  m_treeRows.clear();
  const auto& r = m_doc->rawlog;
  const auto filter = toLower(m_treeFilter);
  const auto matches = [&filter](const std::string& cls, const std::string& label)
  {
    return filter.empty() || toLower(cls).find(filter) != std::string::npos ||
           toLower(label).find(filter) != std::string::npos;
  };
  for (size_t i = 0; i < r.size(); i++)
  {
    const auto type = r.getType(i);
    if (type == CRawlog::TEntryType::etSensoryFrame)
    {
      const auto sf = r.getAsObservations(i);
      std::vector<int> children;
      for (size_t k = 0; k < sf->size(); k++)
      {
        const auto o = sf->getObservationByIndex(k);
        if (o && matches(o->GetRuntimeClass()->className, o->sensorLabel))
        {
          children.push_back(static_cast<int>(k));
        }
      }
      if (!filter.empty() && children.empty())
      {
        continue;
      }
      m_treeRows.push_back({i, -1});
      if (!filter.empty() || m_expanded.count(i) != 0)
      {
        for (const int k : children)
        {
          m_treeRows.push_back({i, k});
        }
      }
      continue;
    }
    std::string label;
    if (type == CRawlog::TEntryType::etObservation)
    {
      label = r.getAsObservation(i)->sensorLabel;
    }
    if (matches(r.getAsGeneric(i)->GetRuntimeClass()->className, label))
    {
      m_treeRows.push_back({i, -1});
    }
  }
}

void RawLogViewerApp::renderTreePanel()
{
  if (!m_showTree)
  {
    return;
  }
  if (ImGui::Begin(PANEL_TREE, &m_showTree) && !busyPlaceholder(busy()))
  {
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputTextWithHint(
            "##filter", ICON_MS_SEARCH " Filter by class or sensor label", &m_treeFilter))
    {
      m_treeDirty = true;
    }
    if (m_treeDirty)
    {
      rebuildTreeRows();
    }
    const auto& r = m_doc->rawlog;
    if (r.empty())
    {
      ImGui::TextDisabled("(Empty rawlog)");
    }
    else if (ImGui::BeginTable("entries", 4, TABLE_FLAGS))
    {
      ImGui::TableSetupScrollFreeze(0, 1);
      ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 90);
      ImGui::TableSetupColumn("Class");
      ImGui::TableSetupColumn("Label");
      ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 80);
      ImGui::TableHeadersRow();

      // Row of the selection, to scroll to it:
      std::optional<int> selRow;
      if (m_scrollTreeToSelection && m_curEntry)
      {
        for (size_t k = 0; k < m_treeRows.size(); k++)
        {
          if (m_treeRows[k].entry == *m_curEntry)
          {
            selRow = static_cast<int>(k);
            break;
          }
        }
        if (!selRow)
        {
          m_scrollTreeToSelection = false;
        }
      }

      ImGuiListClipper clipper;
      clipper.Begin(static_cast<int>(m_treeRows.size()));
      if (selRow)
      {
        clipper.IncludeItemByIndex(*selRow);
      }
      std::optional<std::pair<size_t, int>> clicked;
      while (clipper.Step())
      {
        for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; row++)
        {
          const auto [entry, k] = m_treeRows[static_cast<size_t>(row)];
          ImGui::PushID(row);
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);

          // Object of this row:
          mrpt::serialization::CSerializable::Ptr obj;
          std::string label;
          if (k >= 0)
          {
            obj = m_doc->rawlog.getAsObservations(entry)->getObservationByIndex(
                static_cast<size_t>(k));
          }
          else
          {
            obj = m_doc->rawlog.getAsGeneric(entry);
          }
          auto obs = std::dynamic_pointer_cast<CObservation>(obj);
          if (obs)
          {
            label = obs->sensorLabel;
          }

          const bool isSF = k < 0 && r.getType(entry) == CRawlog::TEntryType::etSensoryFrame;
          if (isSF)
          {
            const bool open = m_expanded.count(entry) != 0 || !m_treeFilter.empty();
            if (ImGui::SmallButton(open ? "-" : "+"))
            {
              if (open)
              {
                m_expanded.erase(entry);
              }
              else
              {
                m_expanded.insert(entry);
              }
              m_treeDirty = true;
            }
            ImGui::SameLine();
            label = mrpt::format("%zu observations", r.getAsObservations(entry)->size());
          }
          const bool selected = m_curEntry && *m_curEntry == entry && m_selectedIndexInSF == k;
          const std::string idText =
              k < 0 ? mrpt::format("%zu", entry) : mrpt::format("    .%d", k);
          if (ImGui::Selectable(
                  idText.c_str(), selected,
                  ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap |
                      ImGuiSelectableFlags_AllowDoubleClick))
          {
            clicked = std::make_pair(entry, k);
            if (obs && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            {
              openSensorPanel(obs->sensorLabel);
            }
          }
          if (ImGui::BeginPopupContextItem("rowMenu"))
          {
            if (obs && ImGui::MenuItem(ICON_MS_ADD_BOX " Open a panel for this sensor"))
            {
              openSensorPanel(obs->sensorLabel);
            }
            if (k < 0 && ImGui::MenuItem(ICON_MS_DELETE " Delete this entry"))
            {
              const size_t e = entry;
              startDocTask(
                  "Delete entry",
                  [e](RawlogDocument& doc, mrpt::imgui::AsyncTaskContext&)
                  {
                    doc.keepRange(e, e, true);
                    return mrpt::format("Entry %zu deleted.", e);
                  });
            }
            ImGui::EndPopup();
          }
          if (selRow && *selRow == row)
          {
            ImGui::SetScrollHereY(0.5f);
            m_scrollTreeToSelection = false;
          }
          ImGui::TableSetColumnIndex(1);
          ImGui::TextUnformatted(shortClassName(obj->GetRuntimeClass()->className).c_str());
          ImGui::TableSetColumnIndex(2);
          ImGui::TextUnformatted(label.c_str());
          ImGui::TableSetColumnIndex(3);
          if (obs && obs->timestamp != INVALID_TIMESTAMP && m_doc->usesTimestamps())
          {
            ImGui::Text("%.03f", m_doc->toTimelineTime(obs->timestamp));
          }
          else if (m_doc->usesTimestamps())
          {
            ImGui::TextDisabled("%.03f", m_doc->entryTime(entry));
          }
          ImGui::PopID();
        }
      }
      ImGui::EndTable();
      if (clicked)
      {
        selectObject(clicked->first, clicked->second);
        m_scrollTreeToSelection = false;  // it is already visible
      }
    }
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Selected object
// ---------------------------------------------------------------------------
void RawLogViewerApp::renderSelectedPanel()
{
  if (!m_showSelected)
  {
    return;
  }
  if (ImGui::Begin(PANEL_SELECTED, &m_showSelected) && !busyPlaceholder(busy()))
  {
    if (!m_selected || !m_curEntry)
    {
      ImGui::TextDisabled("Nothing selected. Pick an entry in the rawlog tree or the timeline.");
    }
    else
    {
      // A collection with one action, or a sensory frame with one
      // observation, is shown as its only object:
      auto viewObj = m_selected;
      if (auto acts = std::dynamic_pointer_cast<CActionCollection>(m_selected);
          acts && acts->size() == 1)
      {
        viewObj = acts->get(0);
      }
      else if (auto sf = std::dynamic_pointer_cast<CSensoryFrame>(m_selected);
               sf && sf->size() == 1)
      {
        viewObj = sf->getObservationByIndex(0);
      }
      const auto* cls = viewObj->GetRuntimeClass();
      if (!m_selectedPanel || m_selectedPanelClass != cls)
      {
        m_selectedPanel = mrpt::apps::ObservationPanelRegistry::Instance().create(cls);
        m_selectedPanelClass = cls;
      }
      if (!m_selectedTextPanel)
      {
        m_selectedTextPanel = mrpt::apps::ObservationPanelRegistry::createGeneric();
      }
      ImGui::Text("Entry %zu", *m_curEntry);
      if (m_selectedIndexInSF >= 0)
      {
        ImGui::SameLine();
        ImGui::TextDisabled("(observation %d of the sensory frame)", m_selectedIndexInSF);
      }
      if (auto obs = std::dynamic_pointer_cast<CObservation>(m_selected); obs)
      {
        ImGui::SameLine();
        if (ImGui::SmallButton(ICON_MS_ADD_BOX " Sensor panel"))
        {
          openSensorPanel(obs->sensorLabel);
        }
        tooltip("Open a panel that follows this sensor along the timeline");
      }
      if (std::string(m_selectedPanel->name()) == "Description")
      {
        m_selectedPanel->render(m_selected, m_panelCtx);
      }
      else if (ImGui::BeginTabBar("selectedTabs"))
      {
        if (ImGui::BeginTabItem(m_selectedPanel->name()))
        {
          m_selectedPanel->render(viewObj, m_panelCtx);
          ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Description"))
        {
          m_selectedTextPanel->render(m_selected, m_panelCtx);
          ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
      }
    }
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Rawlog information and statistics
// ---------------------------------------------------------------------------
void RawLogViewerApp::renderInfoPanel()
{
  if (!m_showInfo)
  {
    return;
  }
  if (ImGui::Begin(PANEL_INFO, &m_showInfo) && !busyPlaceholder(busy()))
  {
    const auto& doc = *m_doc;
    if (doc.fileName.empty() && doc.rawlog.empty())
    {
      ImGui::TextDisabled("No rawlog loaded: use File > Open..., or drop a file here.");
    }
    else
    {
      if (!doc.fileName.empty())
      {
        ImGui::TextWrapped("File: %s", doc.fileName.c_str());
        if (mrpt::system::fileExists(doc.fileName))
        {
          ImGui::Text(
              "File size: %s", mrpt::system::unitsFormat(
                                   static_cast<double>(mrpt::system::getFileSize(doc.fileName)))
                                   .c_str());
        }
      }
      ImGui::TextUnformatted(doc.summaryText().c_str());

      ImGui::SeparatorText("External images");
      ImGui::SetNextItemWidth(-150);
      ImGui::InputText("##imgdir", &m_imagesDirOverride);
      tooltip("Directory with the externally-stored images of this rawlog");
      ImGui::SameLine();
      if (ImGui::Button("Apply"))
      {
        setImagesDirectory(m_imagesDirOverride);
      }
      ImGui::SameLine();
      if (ImGui::Button(ICON_MS_FOLDER_OPEN))
      {
        m_fileDlg.tag = TAG_IMAGES_DIR;
        m_fileDlg.open(
            CImGuiFileDialog::Mode::SelectFolder, "Directory of the external images", {},
            m_imagesDirOverride);
      }
      tooltip("Choose the directory");

      ImGui::SeparatorText("Comments");
      const auto comments = doc.rawlog.getCommentText();
      if (comments.empty())
      {
        ImGui::TextDisabled("(No comments)");
      }
      else
      {
        ImGui::BeginChild(
            "comments", ImVec2(-1, ImGui::GetTextLineHeightWithSpacing() * 4),
            ImGuiChildFlags_Borders);
        ImGui::TextUnformatted(comments.c_str());
        ImGui::EndChild();
      }
      if (ImGui::Button(ICON_MS_EDIT " Edit comments..."))
      {
        m_commentsEdit = comments;
        m_openCommentsPopup = true;
      }

      ImGui::SeparatorText("Sensors");
      ImGui::TextDisabled("Double-click a sensor to open its panel.");
      const float h = ImGui::GetTextLineHeightWithSpacing() *
                      std::min<float>(12.0f, static_cast<float>(doc.sensors().size()) + 2.0f);
      if (ImGui::BeginTable("sensors", 4, TABLE_FLAGS, ImVec2(-1, h)))
      {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Label");
        ImGui::TableSetupColumn("Class");
        ImGui::TableSetupColumn("Count");
        ImGui::TableSetupColumn("Rate [Hz]");
        ImGui::TableHeadersRow();
        for (const auto& [label, s] : doc.sensors())
        {
          ImGui::PushID(label.c_str());
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          if (ImGui::Selectable(
                  label.empty() ? "(no label)" : label.c_str(), false,
                  ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick) &&
              ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
          {
            openSensorPanel(label);
          }
          ImGui::TableSetColumnIndex(1);
          std::string classes;
          for (const auto& c : s.classNames)
          {
            classes += (classes.empty() ? "" : ", ") + shortClassName(c.c_str());
          }
          ImGui::TextUnformatted(classes.c_str());
          ImGui::TableSetColumnIndex(2);
          ImGui::Text("%zu", s.count);
          ImGui::TableSetColumnIndex(3);
          if (const double rate = s.rate(); rate > 0)
          {
            ImGui::Text("%.02f", rate);
          }
          else
          {
            ImGui::TextDisabled("-");
          }
          ImGui::PopID();
        }
        ImGui::EndTable();
      }

      if (ImGui::CollapsingHeader("Objects by class"))
      {
        if (ImGui::BeginTable("classes", 2, TABLE_FLAGS & ~ImGuiTableFlags_ScrollY))
        {
          ImGui::TableSetupColumn("Class");
          ImGui::TableSetupColumn("Count");
          ImGui::TableHeadersRow();
          for (const auto& [cls, n] : doc.classCounts())
          {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(cls.c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%zu", n);
          }
          ImGui::EndTable();
        }
      }
    }
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// 3D view options
// ---------------------------------------------------------------------------
void RawLogViewerApp::renderViewPanel()
{
  if (!m_showView)
  {
    return;
  }
  if (ImGui::Begin(PANEL_VIEW, &m_showView))
  {
    bool changed = false;
    ImGui::SeparatorText("Show");
    changed |= ImGui::Checkbox("Selected entry", &m_showCurrentObs);
    ImGui::BeginDisabled(!m_showCurrentObs);
    changed |= ImGui::Checkbox("All sensors near the current time", &m_showNearbyObs);
    ImGui::SameLine();
    helpMarker(
        "For rawlogs with timestamps: show the latest observation of each sensor, if it is not "
        "older than the tolerance, instead of only the selected entry.");
    ImGui::BeginDisabled(!m_showNearbyObs);
    ImGui::SetNextItemWidth(120);
    changed |= ImGui::SliderFloat("Tolerance [s]", &m_nearbyToleranceSec, 0.01f, 5.0f, "%.2f");
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    changed |= ImGui::Checkbox("Odometry path", &m_showOdoPath);
    ImGui::SameLine();
    helpMarker(
        "From odometry actions or observations. The selected entry is shown at the robot pose "
        "of this path.");
    changed |= ImGui::Checkbox("GPS path (ENU, from the first fix)", &m_showGpsPath);
    changed |= ImGui::Checkbox("Raw map", &m_showRawMapInScene);
    changed |= ImGui::Checkbox("Camera follows the robot", &m_followRobot);
    ImGui::SeparatorText("Observations");
    changed |= mrpt::apps::editVisualizationParameters(m_panelCtx);
    if (changed && !busy())
    {
      updateCurrentInScene();
    }
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Edit
// ---------------------------------------------------------------------------
void RawLogViewerApp::renderEditPanel()
{
  if (!m_showEdit)
  {
    return;
  }
  if (ImGui::Begin(PANEL_EDIT, &m_showEdit) && !busyPlaceholder(busy()))
  {
    const auto& doc = *m_doc;
    ImGui::TextWrapped(
        "These operations modify the rawlog in memory. Use File > Save as... to keep the "
        "result, or File > Revert to undo them.");

    ImGui::SeparatorText("By entry index");
    rangeEditor("First, last", m_editRange, doc.rawlog.size());
    if (m_curEntry)
    {
      ImGui::SameLine();
      if (ImGui::SmallButton("From selection"))
      {
        m_editRange[0] = static_cast<int>(*m_curEntry);
      }
      ImGui::SameLine();
      if (ImGui::SmallButton("To selection"))
      {
        m_editRange[1] = static_cast<int>(*m_curEntry);
      }
    }
    for (const bool remove : {false, true})
    {
      if (ImGui::Button(remove ? "Remove this range" : "Keep this range only"))
      {
        const auto first = static_cast<size_t>(m_editRange[0]);
        const auto last = static_cast<size_t>(m_editRange[1]);
        startDocTask(
            "Edit", [first, last, remove](RawlogDocument& d, mrpt::imgui::AsyncTaskContext&)
            { return mrpt::format("Entries removed: %zu", d.keepRange(first, last, remove)); });
      }
      ImGui::SameLine();
    }
    ImGui::NewLine();

    // Observations by class:
    ImGui::SeparatorText("Observations by class");
    ImGui::BeginChild(
        "classes", ImVec2(-1, ImGui::GetTextLineHeightWithSpacing() * 5), ImGuiChildFlags_Borders);
    for (const auto& [cls, n] : doc.classCounts())
    {
      const auto* id = mrpt::rtti::findRegisteredClass(cls);
      if (id == nullptr || !id->derivedFrom(CLASS_ID(CObservation)))
      {
        continue;
      }
      bool sel = m_editClasses.count(cls) != 0;
      if (ImGui::Checkbox(mrpt::format("%s (%zu)", cls.c_str(), n).c_str(), &sel))
      {
        if (sel)
        {
          m_editClasses.insert(cls);
        }
        else
        {
          m_editClasses.erase(cls);
        }
      }
    }
    ImGui::EndChild();
    ImGui::BeginDisabled(m_editClasses.empty());
    for (const bool keep : {false, true})
    {
      if (ImGui::Button(keep ? "Keep only the selected classes" : "Remove the selected classes"))
      {
        const auto classes = m_editClasses;
        startDocTask(
            "Edit",
            [classes, keep](RawlogDocument& d, mrpt::imgui::AsyncTaskContext&)
            {
              return mrpt::format(
                  "Observations removed: %zu",
                  d.removeObservations(
                      [&](const CObservation& o)
                      { return (classes.count(o.GetRuntimeClass()->className) != 0) != keep; }));
            });
      }
      ImGui::SameLine();
    }
    ImGui::NewLine();
    ImGui::EndDisabled();

    // Observations by label:
    ImGui::SeparatorText("Observations by sensor label");
    ImGui::BeginChild(
        "labels", ImVec2(-1, ImGui::GetTextLineHeightWithSpacing() * 5), ImGuiChildFlags_Borders);
    for (const auto& [label, s] : doc.sensors())
    {
      bool sel = m_editLabels.count(label) != 0;
      if (ImGui::Checkbox(
              mrpt::format("%s (%zu)", label.empty() ? "(no label)" : label.c_str(), s.count)
                  .c_str(),
              &sel))
      {
        if (sel)
        {
          m_editLabels.insert(label);
        }
        else
        {
          m_editLabels.erase(label);
        }
      }
    }
    ImGui::EndChild();
    ImGui::BeginDisabled(m_editLabels.empty());
    for (const bool keep : {false, true})
    {
      if (ImGui::Button(keep ? "Keep only the selected sensors" : "Remove the selected sensors"))
      {
        const auto labels = m_editLabels;
        startDocTask(
            "Edit",
            [labels, keep](RawlogDocument& d, mrpt::imgui::AsyncTaskContext&)
            {
              return mrpt::format(
                  "Observations removed: %zu",
                  d.removeObservations([&](const CObservation& o)
                                       { return (labels.count(o.sensorLabel) != 0) != keep; }));
            });
      }
      ImGui::SameLine();
    }
    ImGui::NewLine();
    ImGui::EndDisabled();

    ImGui::SeparatorText("Miscellaneous");
    ImGui::SetNextItemWidth(120);
    ImGui::InputFloat("Max. pitch/roll [deg]", &m_maxPitchDeg);
    ImGui::SameLine();
    if (ImGui::Button("Keep horizontal 2D scans only"))
    {
      const double maxPitch = m_maxPitchDeg;
      startDocTask(
          "Edit", [maxPitch](RawlogDocument& d, mrpt::imgui::AsyncTaskContext&)
          { return mrpt::format("Scans removed: %zu", d.keepHorizontalScansOnly(maxPitch)); });
    }
    if (ImGui::Button("Swap the red and blue channels of all images"))
    {
      startDocTask(
          "Edit",
          [](RawlogDocument& d, mrpt::imgui::AsyncTaskContext&)
          {
            return mrpt::format(
                "Images modified: %zu (externally-stored images are not modified)",
                d.swapRedBlue());
          });
    }
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Sensor poses and camera parameters
// ---------------------------------------------------------------------------
void RawLogViewerApp::renderSensorParamsPanel()
{
  if (!m_showSensorParams)
  {
    return;
  }
  if (ImGui::Begin(PANEL_SENSOR_PARAMS, &m_showSensorParams) && !busyPlaceholder(busy()))
  {
    const auto& doc = *m_doc;
    ImGui::TextWrapped("Changes apply to the rawlog in memory.");
    if (sensorCombo("Sensor", doc, m_spLabel) || ImGui::SmallButton("Get current values"))
    {
      // From the first observation of the sensor:
      if (auto it = doc.sensors().find(m_spLabel);
          it != doc.sensors().end() && it->second.count != 0)
      {
        if (auto o = doc.latestObservation(m_spLabel, it->second.times.front()); o)
        {
          const auto p = o->getSensorPose();
          m_spPose[0] = static_cast<float>(p.x());
          m_spPose[1] = static_cast<float>(p.y());
          m_spPose[2] = static_cast<float>(p.z());
          m_spPose[3] = static_cast<float>(mrpt::RAD2DEG(p.yaw()));
          m_spPose[4] = static_cast<float>(mrpt::RAD2DEG(p.pitch()));
          m_spPose[5] = static_cast<float>(mrpt::RAD2DEG(p.roll()));
          if (auto im = std::dynamic_pointer_cast<CObservationImage>(o); im)
          {
            const auto& c = im->cameraParams;
            const double vals[9] = {c.fx(), c.fy(), c.cx(),
                                    c.cy(), c.k1(), c.k2(),
                                    c.p1(), c.p2(), c.focalLengthMeters};
            for (int i = 0; i < 9; i++)
            {
              m_spCam[i] = static_cast<float>(vals[i]);
            }
          }
        }
      }
    }
    ImGui::BeginDisabled(m_spLabel.empty() && doc.sensors().count("") == 0);
    ImGui::SeparatorText("Sensor pose on the robot");
    ImGui::InputFloat3("x, y, z [m]", &m_spPose[0], "%.4f");
    ImGui::BeginDisabled(m_spOnlyXYZ);
    ImGui::InputFloat3("yaw, pitch, roll [deg]", &m_spPose[3], "%.3f");
    ImGui::EndDisabled();
    ImGui::Checkbox("Change x, y, z only", &m_spOnlyXYZ);
    if (ImGui::Button("Apply the sensor pose"))
    {
      const auto label = m_spLabel;
      const bool onlyXYZ = m_spOnlyXYZ;
      const mrpt::poses::CPose3D pose(
          m_spPose[0], m_spPose[1], m_spPose[2], mrpt::DEG2RAD(m_spPose[3]),
          mrpt::DEG2RAD(m_spPose[4]), mrpt::DEG2RAD(m_spPose[5]));
      startDocTask(
          "Sensor pose",
          [label, pose, onlyXYZ](RawlogDocument& d, mrpt::imgui::AsyncTaskContext&) {
            return mrpt::format(
                "Observations modified: %zu", d.setSensorPose(label, pose, onlyXYZ));
          });
    }

    ImGui::SeparatorText("Camera parameters (monocular images)");
    ImGui::InputFloat4("fx, fy, cx, cy [px]", &m_spCam[0], "%.3f");
    ImGui::InputFloat4("k1, k2, p1, p2", &m_spCam[4], "%.6f");
    ImGui::InputFloat("Focal length [m]", &m_spCam[8], 0, 0, "%.5f");
    if (ImGui::Button("Apply the camera parameters"))
    {
      const auto label = m_spLabel;
      mrpt::img::TCamera cam;
      cam.fx(m_spCam[0]);
      cam.fy(m_spCam[1]);
      cam.cx(m_spCam[2]);
      cam.cy(m_spCam[3]);
      cam.k1(m_spCam[4]);
      cam.k2(m_spCam[5]);
      cam.p1(m_spCam[6]);
      cam.p2(m_spCam[7]);
      cam.focalLengthMeters = m_spCam[8];
      startDocTask(
          "Camera parameters", [label, cam](RawlogDocument& d, mrpt::imgui::AsyncTaskContext&)
          { return mrpt::format("Images modified: %zu", d.setCameraParams(label, cam)); });
    }
    ImGui::EndDisabled();

    ImGui::SeparatorText("Several sensors at once");
    if (ImGui::SmallButton(ICON_MS_FOLDER_OPEN " Load..."))
    {
      m_fileDlg.tag = TAG_BATCH_LOAD;
      m_fileDlg.open(CImGuiFileDialog::Mode::Open, "Load sensor poses", INI_FILTERS, lastDir());
    }
    ImGui::SameLine();
    if (ImGui::SmallButton(ICON_MS_SAVE " Save..."))
    {
      m_fileDlg.tag = TAG_BATCH_SAVE;
      m_fileDlg.open(CImGuiFileDialog::Mode::Save, "Save sensor poses", INI_FILTERS, lastDir());
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Apply"))
    {
      const auto cfg = m_batchPoseConfig;
      startDocTask(
          "Sensor poses",
          [cfg](RawlogDocument& d, mrpt::imgui::AsyncTaskContext&) {
            return mrpt::format("Observations modified: %zu", d.applySensorPosesFromConfig(cfg));
          });
    }
    ImGui::InputTextMultiline("##batch", &m_batchPoseConfig, ImVec2(-1, -1));
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Motion model
// ---------------------------------------------------------------------------
void RawLogViewerApp::renderMotionModelPanel()
{
  if (!m_showMotionModel)
  {
    return;
  }
  if (ImGui::Begin(PANEL_MOTION_MODEL, &m_showMotionModel) && !busyPlaceholder(busy()))
  {
    auto& mm = m_motionModel;
    bool changed = false;
    int model = mm.modelSelection == CActionRobotMovement2D::mmGaussian ? 0 : 1;
    changed |= ImGui::RadioButton("Gaussian model", &model, 0);
    ImGui::SameLine();
    changed |= ImGui::RadioButton("Thrun's book model", &model, 1);
    mm.modelSelection =
        model == 0 ? CActionRobotMovement2D::mmGaussian : CActionRobotMovement2D::mmThrun;

    const auto inputD = [&changed](const char* label, double& v, const char* fmt = "%.6f")
    { changed |= ImGui::InputDouble(label, &v, 0, 0, fmt); };
    const auto inputF = [&changed](const char* label, float& v, const char* fmt = "%.6f")
    { changed |= ImGui::InputFloat(label, &v, 0, 0, fmt); };
    if (model == 0)
    {
      auto& g = mm.gaussianModel;
      inputD("a1: motion to x/y std. [m/m]", g.a1);
      inputD("a2: rotation to x/y std. [m/rad]", g.a2);
      inputD("a3: motion to phi std. [rad/m]", g.a3);
      inputD("a4: rotation to phi std. [rad/rad]", g.a4);
      inputD("Min. std. of x/y [m]", g.minStdXY);
      inputD("Min. std. of phi [rad]", g.minStdPHI);
    }
    else
    {
      auto& t = mm.thrunModel;
      inputF("alpha1: rot. to rot. [rad/rad]", t.alfa1_rot_rot);
      inputF("alpha2: rot. to trans. [rad/m]", t.alfa2_rot_trans);
      inputF("alpha3: trans. to trans. [m/m]", t.alfa3_trans_trans);
      inputF("alpha4: trans. to rot. [m/rad]", t.alfa4_trans_rot);
      inputF("Additional std. of x/y [m]", t.additional_std_XY);
      inputF("Additional std. of phi [rad]", t.additional_std_phi);
      int n = static_cast<int>(t.nParticlesCount);
      if (ImGui::InputInt("Particles", &n))
      {
        t.nParticlesCount = static_cast<uint32_t>(std::max(10, n));
        changed = true;
      }
    }
    if (ImGui::Button("Reset to defaults"))
    {
      const auto sel = mm.modelSelection;
      mm = CActionRobotMovement2D::TMotionModelOptions();
      mm.modelSelection = sel;
      changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Get from the rawlog"))
    {
      if (const auto m = m_doc->firstMotionModel(); m)
      {
        mm = *m;
        changed = true;
      }
      else
      {
        m_msgs.info("Motion model", "The rawlog has no odometry actions.");
      }
    }

    ImGui::SeparatorText("Preview");
    changed |= ImGui::InputFloat3("Increment x, y [m], phi [deg]", m_motionTestIncr, "%.3f");
    if (changed || m_motionSamples[0].empty())
    {
      CActionRobotMovement2D act;
      act.computeFromOdometry(
          mrpt::poses::CPose2D(
              m_motionTestIncr[0], m_motionTestIncr[1], mrpt::DEG2RAD(m_motionTestIncr[2])),
          mm);
      std::vector<mrpt::math::CVectorDouble> samples;
      act.poseChange->drawManySamples(1000, samples);
      for (auto& v : m_motionSamples)
      {
        v.clear();
      }
      for (const auto& s : samples)
      {
        m_motionSamples[0].push_back(s[0]);
        m_motionSamples[1].push_back(s[1]);
        m_motionSamples[2].push_back(mrpt::RAD2DEG(s[2]));
      }
    }
    const float plotH = std::max(150.0f, ImGui::GetContentRegionAvail().y - 110.0f);
    const float plotW = ImGui::GetContentRegionAvail().x * 0.5f - 4;
    if (ImPlot::BeginPlot("x-y samples", ImVec2(plotW, plotH), ImPlotFlags_Equal))
    {
      ImPlot::SetupAxes("x [m]", "y [m]", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
      ImPlot::PlotScatter(
          "##xy", m_motionSamples[0].data(), m_motionSamples[1].data(),
          static_cast<int>(m_motionSamples[0].size()), ImPlotSpec(ImPlotProp_MarkerSize, 1.5f));
      ImPlot::EndPlot();
    }
    ImGui::SameLine();
    if (ImPlot::BeginPlot("phi samples", ImVec2(-1, plotH)))
    {
      ImPlot::SetupAxes("phi [deg]", "count", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
      ImPlot::PlotHistogram(
          "##phi", m_motionSamples[2].data(), static_cast<int>(m_motionSamples[2].size()));
      ImPlot::EndPlot();
    }

    ImGui::SeparatorText("Apply to the odometry of the rawlog");
    ImGui::Checkbox("All entries", &m_motionAll);
    if (!m_motionAll)
    {
      ImGui::SameLine();
      ImGui::SetNextItemWidth(200);
      rangeEditor("Range", m_motionRange, m_doc->rawlog.size());
    }
    if (ImGui::Button("Apply"))
    {
      const auto opts = mm;
      const size_t first = m_motionAll ? 0 : static_cast<size_t>(m_motionRange[0]);
      const size_t last =
          m_motionAll ? std::numeric_limits<size_t>::max() : static_cast<size_t>(m_motionRange[1]);
      startDocTask(
          "Motion model",
          [opts, first, last](RawlogDocument& d, mrpt::imgui::AsyncTaskContext&) {
            return mrpt::format(
                "Odometry actions modified: %zu", d.setMotionModel(opts, first, last));
          });
    }
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Raw map
// ---------------------------------------------------------------------------
void RawLogViewerApp::renderRawMapPanel()
{
  if (!m_showRawMapPanel)
  {
    return;
  }
  if (ImGui::Begin(PANEL_RAWMAP, &m_showRawMapPanel) && !busyPlaceholder(busy()))
  {
    ImGui::TextWrapped(
        "Builds a map by inserting the observations at the robot poses from odometry, and "
        "shows it in the 3D view.");
    rangeEditor("Entries", m_rawMapRange, m_doc->rawlog.size());
    ImGui::InputInt("Decimation", &m_rawMapDecimation);
    m_rawMapDecimation = std::max(1, m_rawMapDecimation);
    if (ImGui::CollapsingHeader("Map configuration"))
    {
      ImGui::InputTextMultiline(
          "##mapcfg", &m_rawMapConfig, ImVec2(-1, ImGui::GetTextLineHeightWithSpacing() * 10));
      if (ImGui::SmallButton("Reset to default"))
      {
        m_rawMapConfig = DEFAULT_RAWMAP_CONFIG;
      }
    }
    ImGui::BeginDisabled(m_doc->rawlog.empty());
    if (ImGui::Button(ICON_MS_PLAY_ARROW " Generate"))
    {
      const auto cfg = m_rawMapConfig;
      const auto first = static_cast<size_t>(m_rawMapRange[0]);
      const auto last = static_cast<size_t>(m_rawMapRange[1]);
      const auto decimation = static_cast<size_t>(m_rawMapDecimation);
      queueTask(
          "Building the raw map",
          [this, doc = m_doc, cfg, first, last,
           decimation](mrpt::imgui::AsyncTaskContext& ctx) -> std::function<void()>
          {
            try
            {
              std::shared_ptr<mrpt::maps::CMultiMetricMap> map;
              std::vector<mrpt::poses::CPose3D> path;
              const bool done = doc->buildRawMap(
                  cfg, first, last, decimation,
                  [&ctx](double f)
                  {
                    ctx.setProgress(f);
                    return !ctx.cancelRequested();
                  },
                  map, path);
              if (!done)
              {
                return [this]() { m_statusMsg = "Raw map: cancelled."; };
              }
              return [this, map, path]()
              {
                m_rawMap = map;
                m_rawMapPath = path;
                m_glRawMap->clear();
                if (map)
                {
                  m_glRawMap->insert(map->getVisualization());
                  auto glPath = mrpt::viz::CSetOfLines::Create();
                  glPath->setColor_u8(0xff, 0xff, 0x40);
                  for (size_t i = 1; i < path.size(); i++)
                  {
                    glPath->appendLine(
                        path[i - 1].x(), path[i - 1].y(), path[i - 1].z(), path[i].x(), path[i].y(),
                        path[i].z());
                  }
                  m_glRawMap->insert(glPath);
                }
                m_showRawMapInScene = true;
                m_console.push(
                    mrpt::system::LVL_INFO, "raw map",
                    mrpt::format("Map built along %zu poses.", path.size()));
                updateCurrentInScene();
              };
            }
            catch (const std::exception& e)
            {
              const std::string err = mrpt::exception_to_str(e);
              return [this, err]() { m_msgs.error("Raw map", err); };
            }
          });
    }
    ImGui::EndDisabled();
    if (m_rawMap)
    {
      ImGui::SeparatorText("Result");
      for (const auto& m : m_rawMap->mapsList())
      {
        if (auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(m); pts)
        {
          ImGui::Text("%s: %zu points", m->GetRuntimeClass()->className, pts->size());
        }
        else
        {
          ImGui::TextUnformatted(m->GetRuntimeClass()->className);
        }
      }
      if (ImGui::Button("Save the map as text..."))
      {
        m_fileDlg.tag = TAG_RAWMAP_TXT;
        m_fileDlg.open(CImGuiFileDialog::Mode::Save, "Save the map", TXT_FILTERS, lastDir());
      }
      ImGui::SameLine();
      if (ImGui::Button("Save as a 3D scene..."))
      {
        m_fileDlg.tag = TAG_RAWMAP_3D;
        m_fileDlg.open(CImGuiFileDialog::Mode::Save, "Save 3D scene", SCENE_FILTERS, lastDir());
      }
      ImGui::SameLine();
      if (ImGui::Button("Save the path..."))
      {
        m_fileDlg.tag = TAG_RAWMAP_PATH;
        m_fileDlg.open(CImGuiFileDialog::Mode::Save, "Save the path", TXT_FILTERS, lastDir());
      }
      if (ImGui::Button("Clear"))
      {
        m_rawMap.reset();
        m_rawMapPath.clear();
        m_glRawMap->clear();
      }
    }
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Export to text
// ---------------------------------------------------------------------------
void RawLogViewerApp::renderExportPanel()
{
  if (!m_showExport)
  {
    return;
  }
  if (ImGui::Begin(PANEL_EXPORT, &m_showExport) && !busyPlaceholder(busy()))
  {
    const auto& doc = *m_doc;
    ImGui::TextWrapped(
        "Writes one line per observation, with columns specific to each class. The first lines "
        "of the file describe them.");
    std::vector<std::string> classes;
    for (const auto& c : RawlogDocument::exportableClasses())
    {
      if (doc.classCounts().count(c) != 0)
      {
        classes.push_back(c);
      }
    }
    if (classes.empty())
    {
      ImGui::TextDisabled("The rawlog has no observations of an exportable class.");
    }
    else
    {
      if (std::find(classes.begin(), classes.end(), m_exportClass) == classes.end())
      {
        m_exportClass = classes.front();
        m_exportLabel.clear();
      }
      if (ImGui::BeginCombo("Class", m_exportClass.c_str()))
      {
        for (const auto& c : classes)
        {
          if (ImGui::Selectable(c.c_str(), c == m_exportClass))
          {
            m_exportClass = c;
            m_exportLabel.clear();
          }
        }
        ImGui::EndCombo();
      }
      if (ImGui::BeginCombo("Sensor", m_exportLabel.empty() ? "(all)" : m_exportLabel.c_str()))
      {
        if (ImGui::Selectable("(all)", m_exportLabel.empty()))
        {
          m_exportLabel.clear();
        }
        for (const auto& [label, s] : doc.sensors())
        {
          if (s.classNames.count(m_exportClass) != 0 &&
              ImGui::Selectable(label.c_str(), label == m_exportLabel))
          {
            m_exportLabel = label;
          }
        }
        ImGui::EndCombo();
      }
      if (ImGui::Button(ICON_MS_FILE_EXPORT " Export..."))
      {
        m_fileDlg.tag = TAG_EXPORT_TXT;
        m_fileDlg.open(
            CImGuiFileDialog::Mode::Save, "Export observations to text", TXT_FILTERS, lastDir());
      }
    }
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Sensor panels
// ---------------------------------------------------------------------------
void RawLogViewerApp::renderSensorPanels()
{
  for (auto& p : m_sensorPanels)
  {
    if (!p.open)
    {
      continue;
    }
    const std::string name = std::string(ICON_MS_SENSORS " ") +
                             (p.label.empty() ? "(no label)" : p.label) + "###sensor_" + p.label;
    if (m_bottomDockId != 0)
    {
      ImGui::SetNextWindowDockID(m_bottomDockId, ImGuiCond_FirstUseEver);
    }
    ImGui::SetNextWindowSize(ImVec2(520, 420), ImGuiCond_FirstUseEver);
    if (m_focusSensorPanel && *m_focusSensorPanel == p.label)
    {
      ImGui::SetNextWindowFocus();
      m_focusSensorPanel.reset();
    }
    if (ImGui::Begin(name.c_str(), &p.open) && !busyPlaceholder(busy()))
    {
      if (m_doc->sensors().count(p.label) == 0)
      {
        ImGui::TextDisabled("This rawlog has no sensor with this label.");
      }
      else if (auto obs = m_doc->latestObservation(p.label, m_timeline.time()); !obs)
      {
        ImGui::TextDisabled("No observation of this sensor before the current time.");
      }
      else
      {
        const auto* cls = obs->GetRuntimeClass();
        if (!p.panel || p.panelClass != cls)
        {
          p.panel = mrpt::apps::ObservationPanelRegistry::Instance().create(cls);
          p.panelClass = cls;
        }
        p.panel->render(obs, m_panelCtx);
      }
    }
    ImGui::End();
  }
}
