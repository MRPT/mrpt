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

#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CSerializable.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

/** Observation panels shared by the ImGui apps: one renderer per MRPT class,
 * looked up at runtime from the object class. Not installed. */
namespace mrpt::apps
{
/** Settings shared by all panels of an app (e.g. the 3D point styling). */
struct PanelContext
{
  /** Visualization of point clouds, scans, etc. in 3D panels. */
  mrpt::obs::VisualizationParameters vizParams;

  /** Increment after modifying vizParams, so 3D panels rebuild their scene. */
  int vizParamsVersion = 0;

  /** If valid, timestamps are also shown relative to this one. */
  mrpt::Clock::time_point referenceTime = INVALID_TIMESTAMP;
};

/** Renders one kind of object into the current ImGui window.
 * An instance keeps the GUI state of one panel (zoom, selected tab...), so
 * each window showing an object needs its own instance. */
class ObservationPanel
{
 public:
  ObservationPanel() = default;
  virtual ~ObservationPanel() = default;
  ObservationPanel(const ObservationPanel&) = delete;
  ObservationPanel& operator=(const ObservationPanel&) = delete;
  ObservationPanel(ObservationPanel&&) = delete;
  ObservationPanel& operator=(ObservationPanel&&) = delete;

  /** Draws the contents. `obj` may be null (nothing to show yet). Panels
   * rebuild cached data only when `obj` changes. */
  virtual void render(const mrpt::serialization::CSerializable::Ptr& obj, PanelContext& ctx) = 0;

  /** A short name for the kind of panel, e.g. "Image". */
  [[nodiscard]] virtual const char* name() const = 0;
};

using ObservationPanelFactory = std::function<std::unique_ptr<ObservationPanel>()>;

/** Maps MRPT class names to panel factories. */
class ObservationPanelRegistry
{
 public:
  /** The registry with all the built-in panels. */
  static ObservationPanelRegistry& Instance();

  /** Registers (or replaces) the panel of a class, e.g.
   * "mrpt::obs::CObservationImage". */
  void add(const std::string& className, ObservationPanelFactory factory);

  [[nodiscard]] bool has(const std::string& className) const;

  /** A new panel for the class, or for its closest registered base class,
   * or the generic (text) panel if there is none. */
  [[nodiscard]] std::unique_ptr<ObservationPanel> create(
      const mrpt::rtti::TRuntimeClassId* cls) const;

  /** The generic panel: a text description of any object. */
  [[nodiscard]] static std::unique_ptr<ObservationPanel> createGeneric();

  [[nodiscard]] std::vector<std::string> registeredClasses() const;

 private:
  std::map<std::string, ObservationPanelFactory> m_factories;
};

/** Shows the header common to all observation panels: sensor label, class,
 * timestamp and sensor pose on the robot. */
void showCommonSensorInfo(const mrpt::obs::CObservation& obs, const PanelContext& ctx);

/** The text description of an object (observations, actions, or any other
 * class with a known textual dump), as shown by the generic panel. */
[[nodiscard]] std::string describeObject(const mrpt::serialization::CSerializable& obj);

/** An ImGui editor of the 3D visualization parameters. Returns true if
 * something changed (and increments ctx.vizParamsVersion). */
bool editVisualizationParameters(PanelContext& ctx);

// Registration of the built-in panels, one function per source file:
void registerTextPanels(ObservationPanelRegistry& r);
void registerImagePanels(ObservationPanelRegistry& r);
void registerScenePanels(ObservationPanelRegistry& r);

}  // namespace mrpt::apps
