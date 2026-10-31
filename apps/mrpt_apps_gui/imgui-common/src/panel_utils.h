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

#include <imgui.h>
#include <mrpt/apps_imgui/ObservationPanels.h>
#include <mrpt/imgui/CImGuiImageView.h>

#include <memory>
#include <type_traits>

// Internal helpers of the panel implementations.
namespace mrpt::apps::detail
{
/** Base of panels of one observation class T: the common sensor header,
 * then renderObs(). */
template <class T>
class TypedPanel : public ObservationPanel
{
 public:
  void render(const mrpt::serialization::CSerializable::Ptr& obj, PanelContext& ctx) override
  {
    auto o = std::dynamic_pointer_cast<T>(obj);
    if (!o)
    {
      ImGui::TextDisabled("(No data)");
      return;
    }
    if constexpr (std::is_base_of_v<mrpt::obs::CObservation, T>)
    {
      showCommonSensorInfo(*o, ctx);
      ImGui::Separator();
    }
    const bool changed = (obj != m_lastObj);
    m_lastObj = obj;
    renderObs(*o, ctx, changed);
  }

 protected:
  /** `changed` is true the first time a given object is shown. */
  virtual void renderObs(T& o, PanelContext& ctx, bool changed) = 0;

 private:
  mrpt::serialization::CSerializable::Ptr m_lastObj;
};

template <class PANEL>
ObservationPanelFactory factory()
{
  return []() { return std::make_unique<PANEL>(); };
}

/** Shows `img` in `view`. An externally-stored image is loaded, copied, and
 * unloaded again (only that image, not the rest of its observation), to keep
 * memory usage low. Returns an error message, or an empty string on success. */
inline std::string setImageFrom(mrpt::imgui::CImGuiImageView& view, const mrpt::img::CImage& img)
{
  try
  {
    const bool external = img.isExternallyStored();
    if (external)
    {
      img.forceLoad();
    }
    view.setImage(img.makeDeepCopy());
    if (external)
    {
      img.unload();
    }
    return {};
  }
  catch (const std::exception& e)
  {
    view.clear();
    return std::string("Error loading the image:\n") + e.what();
  }
}

}  // namespace mrpt::apps::detail
