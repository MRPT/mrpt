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
#include <mrpt/imgui/CImGuiImageView.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>

#include "panel_utils.h"

namespace mrpt::apps
{
namespace
{
using detail::factory;
using detail::TypedPanel;

// --------------------------------------------------------------------------
class ImagePanel : public TypedPanel<mrpt::obs::CObservationImage>
{
 public:
  [[nodiscard]] const char* name() const override { return "Image"; }

 protected:
  void renderObs(mrpt::obs::CObservationImage& o, PanelContext& /*ctx*/, bool changed) override
  {
    if (changed)
    {
      m_error = detail::setImageFrom(m_view, o.image);
    }
    if (!m_error.empty())
    {
      ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", m_error.c_str());
      return;
    }
    ImGui::Text(
        "%ux%u, %u channel(s)", static_cast<unsigned>(m_view.image().getWidth()),
        static_cast<unsigned>(m_view.image().getHeight()),
        static_cast<unsigned>(m_view.image().channels()));
    m_view.render("##image");
  }

 private:
  mrpt::imgui::CImGuiImageView m_view;
  std::string m_error;
};

// --------------------------------------------------------------------------
class StereoImagesPanel : public TypedPanel<mrpt::obs::CObservationStereoImages>
{
 public:
  [[nodiscard]] const char* name() const override { return "Stereo images"; }

 protected:
  void renderObs(
      mrpt::obs::CObservationStereoImages& o, PanelContext& /*ctx*/, bool changed) override
  {
    if (changed)
    {
      m_error = detail::setImageFrom(m_left, o.imageLeft);
      m_hasRight = o.hasImageRight;
      m_hasDisparity = o.hasImageDisparity;
      if (m_error.empty() && m_hasRight)
      {
        m_error = detail::setImageFrom(m_right, o.imageRight);
      }
      if (m_error.empty() && m_hasDisparity)
      {
        m_error = detail::setImageFrom(m_disparity, o.imageDisparity);
      }
    }
    if (!m_error.empty())
    {
      ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", m_error.c_str());
      return;
    }
    ImGui::Checkbox("Side by side", &m_sideBySide);
    if (m_sideBySide && m_hasRight)
    {
      const float w = ImGui::GetContentRegionAvail().x * 0.5f - ImGui::GetStyle().ItemSpacing.x;
      m_left.render("##left", w, 0);
      ImGui::SameLine();
      m_right.render("##right", w, 0);
      return;
    }
    if (ImGui::BeginTabBar("stereo"))
    {
      if (ImGui::BeginTabItem("Left"))
      {
        m_left.render("##left");
        ImGui::EndTabItem();
      }
      if (m_hasRight && ImGui::BeginTabItem("Right"))
      {
        m_right.render("##right");
        ImGui::EndTabItem();
      }
      if (m_hasDisparity && ImGui::BeginTabItem("Disparity"))
      {
        m_disparity.render("##disparity");
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
  }

 private:
  mrpt::imgui::CImGuiImageView m_left;
  mrpt::imgui::CImGuiImageView m_right;
  mrpt::imgui::CImGuiImageView m_disparity;
  bool m_hasRight = false;
  bool m_hasDisparity = false;
  bool m_sideBySide = false;
  std::string m_error;
};

}  // namespace

void registerImagePanels(ObservationPanelRegistry& r)
{
  r.add("mrpt::obs::CObservationImage", factory<ImagePanel>());
  r.add("mrpt::obs::CObservationStereoImages", factory<StereoImagesPanel>());
}

}  // namespace mrpt::apps
