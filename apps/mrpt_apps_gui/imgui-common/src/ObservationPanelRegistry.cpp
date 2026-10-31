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
#include <mrpt/apps_imgui/ObservationPanels.h>
#include <mrpt/imgui/style.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/system/datetime.h>

#include <sstream>

namespace mrpt::apps
{
namespace
{
/** Any object: its text description, selectable for copying. */
class GenericPanel : public ObservationPanel
{
 public:
  void render(const mrpt::serialization::CSerializable::Ptr& obj, PanelContext& ctx) override
  {
    if (!obj)
    {
      ImGui::TextDisabled("(No data)");
      return;
    }
    if (obj != m_last)
    {
      m_last = obj;
      m_text = describeObject(*obj);
    }
    if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj); obs)
    {
      showCommonSensorInfo(*obs, ctx);
      ImGui::Separator();
    }
    // Read-only, but selectable so it can be copied:
    ImGui::InputTextMultiline(
        "##description", m_text.data(), m_text.size() + 1, ImVec2(-1, -1),
        ImGuiInputTextFlags_ReadOnly);
  }
  [[nodiscard]] const char* name() const override { return "Description"; }

 private:
  mrpt::serialization::CSerializable::Ptr m_last;
  std::string m_text;
};

}  // namespace

ObservationPanelRegistry& ObservationPanelRegistry::Instance()
{
  static ObservationPanelRegistry r = []()
  {
    ObservationPanelRegistry reg;
    registerTextPanels(reg);
    registerImagePanels(reg);
    registerScenePanels(reg);
    return reg;
  }();
  return r;
}

void ObservationPanelRegistry::add(const std::string& className, ObservationPanelFactory factory)
{
  m_factories[className] = std::move(factory);
}

bool ObservationPanelRegistry::has(const std::string& className) const
{
  return m_factories.count(className) != 0;
}

std::unique_ptr<ObservationPanel> ObservationPanelRegistry::create(
    const mrpt::rtti::TRuntimeClassId* cls) const
{
  for (const auto* c = cls; c != nullptr; c = c->getBaseClass ? c->getBaseClass() : nullptr)
  {
    if (c->className == nullptr)
    {
      break;
    }
    if (auto it = m_factories.find(c->className); it != m_factories.end())
    {
      return it->second();
    }
  }
  return createGeneric();
}

std::unique_ptr<ObservationPanel> ObservationPanelRegistry::createGeneric()
{
  return std::make_unique<GenericPanel>();
}

std::vector<std::string> ObservationPanelRegistry::registeredClasses() const
{
  std::vector<std::string> ret;
  for (const auto& [name, f] : m_factories)
  {
    ret.push_back(name);
  }
  return ret;
}

void showCommonSensorInfo(const mrpt::obs::CObservation& obs, const PanelContext& ctx)
{
  ImGui::Text(
      "Sensor: %s  (%s)", obs.sensorLabel.empty() ? "(no label)" : obs.sensorLabel.c_str(),
      obs.GetRuntimeClass()->className);
  if (obs.timestamp == INVALID_TIMESTAMP)
  {
    ImGui::TextDisabled("Timestamp: (invalid)");
  }
  else if (ctx.referenceTime != INVALID_TIMESTAMP)
  {
    ImGui::Text(
        "Timestamp: %s UTC  (t = %.03f s)", mrpt::system::dateTimeToString(obs.timestamp).c_str(),
        mrpt::system::timeDifference(ctx.referenceTime, obs.timestamp));
  }
  else
  {
    ImGui::Text("Timestamp: %s UTC", mrpt::system::dateTimeToString(obs.timestamp).c_str());
  }
  ImGui::Text("Sensor pose: %s", obs.getSensorPose().asString().c_str());
}

std::string describeObject(const mrpt::serialization::CSerializable& obj)
{
  std::stringstream ss;
  try
  {
    if (const auto* obs = dynamic_cast<const mrpt::obs::CObservation*>(&obj); obs)
    {
      // Externally-stored data must be loaded to be described:
      const_cast<mrpt::obs::CObservation*>(obs)->load();
      obs->getDescriptionAsText(ss);
    }
    else if (const auto* act = dynamic_cast<const mrpt::obs::CAction*>(&obj); act)
    {
      act->getDescriptionAsText(ss);
    }
    else if (const auto* acts = dynamic_cast<const mrpt::obs::CActionCollection*>(&obj); acts)
    {
      ss << "Collection of " << acts->size() << " actions:\n";
      for (size_t i = 0; i < acts->size(); i++)
      {
        ss << "\n--- Action #" << i << " ---\n";
        acts->get(i)->getDescriptionAsText(ss);
      }
    }
    else if (const auto* sf = dynamic_cast<const mrpt::obs::CSensoryFrame*>(&obj); sf)
    {
      ss << "Sensory frame with " << sf->size() << " observations:\n";
      for (const auto& o : *sf)
      {
        ss << " - " << o->GetRuntimeClass()->className << " '" << o->sensorLabel << "'\n";
      }
    }
    else
    {
      ss << "Object of class: " << obj.GetRuntimeClass()->className << "\n";
    }
  }
  catch (const std::exception& e)
  {
    ss << "\nError describing the object:\n" << e.what() << "\n";
  }
  return ss.str();
}

bool editVisualizationParameters(PanelContext& ctx)
{
  auto& p = ctx.vizParams;
  bool changed = false;

  float pointSize = static_cast<float>(p.pointSize);
  if (ImGui::SliderFloat("Point size", &pointSize, 1.0f, 20.0f, "%.1f"))
  {
    p.pointSize = pointSize;
    changed = true;
  }

  const char* fields[] = {"x", "y", "z", "intensity", "ring", "t", "rgb"};
  if (ImGui::BeginCombo("Color by", p.coloring.colorizeByField.c_str()))
  {
    for (const char* f : fields)
    {
      if (ImGui::Selectable(f, p.coloring.colorizeByField == f))
      {
        p.coloring.colorizeByField = f;
        changed = true;
      }
    }
    ImGui::EndCombo();
  }
  int cmap = static_cast<int>(p.coloring.colorMap);
  if (ImGui::Combo("Color map", &cmap, "Grayscale\0Jet\0Hot\0"))
  {
    p.coloring.colorMap = static_cast<mrpt::img::TColormap>(cmap);
    changed = true;
  }
  changed |= ImGui::Checkbox("Invert color map", &p.coloring.invertColorMapping);
  changed |= ImGui::Checkbox("Use RGB from camera image", &p.colorFromRGBimage);
  changed |= ImGui::Checkbox("Only points with color", &p.onlyPointsWithColor);

  ImGui::SeparatorText("Axes and sensor");
  changed |= ImGui::Checkbox("Show axes", &p.showAxis);
  ImGui::BeginDisabled(!p.showAxis);
  float axisLimits = static_cast<float>(p.axisLimits);
  if (ImGui::SliderFloat("Axes length [m]", &axisLimits, 1.0f, 200.0f, "%.0f"))
  {
    p.axisLimits = axisLimits;
    changed = true;
  }
  float tick = static_cast<float>(p.axisTickFrequency);
  if (ImGui::SliderFloat("Tick spacing [m]", &tick, 0.1f, 20.0f, "%.1f"))
  {
    p.axisTickFrequency = tick;
    changed = true;
  }
  ImGui::EndDisabled();
  changed |= ImGui::Checkbox("Draw sensor pose", &p.drawSensorPose);
  float sensorScale = static_cast<float>(p.sensorPoseScale);
  if (ImGui::SliderFloat("Sensor pose size", &sensorScale, 0.05f, 2.0f, "%.2f"))
  {
    p.sensorPoseScale = sensorScale;
    changed = true;
  }

  ImGui::SeparatorText("2D scans");
  changed |= ImGui::Checkbox("Show points", &p.showPointsIn2Dscans);
  changed |= ImGui::Checkbox("Show surface", &p.showSurfaceIn2Dscans);
  for (const auto& [label, col] : {
           std::pair{ "Points color",  &p.points2DscansColor},
           std::pair{"Surface color", &p.surface2DscansColor}
  })
  {
    ImVec4 c = mrpt::imgui::toImVec4(*col);
    if (ImGui::ColorEdit4(label, &c.x, ImGuiColorEditFlags_NoInputs))
    {
      *col = mrpt::imgui::toTColor(c);
      changed = true;
    }
  }

  if (changed)
  {
    ctx.vizParamsVersion++;
  }
  return changed;
}

}  // namespace mrpt::apps
