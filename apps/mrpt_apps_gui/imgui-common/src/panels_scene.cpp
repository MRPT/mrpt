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
#include <mrpt/imgui/CImGuiSceneView.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/imgui/style.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>

#include <algorithm>

#include "panel_utils.h"

namespace mrpt::apps
{
namespace
{
using detail::factory;

/** A [0,1] matrix as a gray image, with the first row at the top. */
template <class MATRIX>
mrpt::img::CImage normalizedImage(const MATRIX& m)
{
  mrpt::img::CImage img;
  const auto nRows = static_cast<int>(m.rows());
  const auto nCols = static_cast<int>(m.cols());
  if (nRows == 0 || nCols == 0)
  {
    return img;
  }
  float maxVal = 0;
  for (int r = 0; r < nRows; r++)
  {
    for (int c = 0; c < nCols; c++)
    {
      maxVal = std::max(maxVal, static_cast<float>(m(r, c)));
    }
  }
  if (maxVal == 0)
  {
    maxVal = 1.0f;
  }
  mrpt::math::CMatrixFloat normalized(nRows, nCols);
  for (int r = 0; r < nRows; r++)
  {
    for (int c = 0; c < nCols; c++)
    {
      normalized(r, c) = static_cast<float>(m(r, c)) / maxVal;
    }
  }
  img.setFromMatrix(normalized, true /*already in [0,1]*/, true /*flip_vertically*/);
  return img;
}

/** Observations shown as 3D points: 2D and 3D scans, point clouds, rotating
 * and Velodyne scans. Classes with images also get image tabs. */
class PointsPanel : public ObservationPanel
{
 public:
  PointsPanel()
  {
    auto scene = mrpt::viz::Scene::Create();
    m_glObs = mrpt::viz::CSetOfObjects::Create();
    scene->insert(m_glObs);
    m_view.setScene(scene);
    m_view.setBackgroundColor(0.15f, 0.15f, 0.18f);
    m_view.cameraController.setZoomDistance(15.0f);
    m_view.cameraController.setAzimuthDegrees(-140.0f);
    m_view.cameraController.setElevationDegrees(35.0f);
  }

  [[nodiscard]] const char* name() const override { return "3D points"; }

  void render(const mrpt::serialization::CSerializable::Ptr& obj, PanelContext& ctx) override
  {
    auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj);
    if (!obs)
    {
      ImGui::TextDisabled("(No data)");
      return;
    }
    showCommonSensorInfo(*obs, ctx);

    if (ImGui::Button(ICON_MS_TUNE " View options"))
    {
      ImGui::OpenPopup("viewOptions");
    }
    if (ImGui::BeginPopup("viewOptions"))
    {
      editVisualizationParameters(ctx);
      ImGui::EndPopup();
    }

    auto o3D = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(obs);
    if (o3D && o3D->hasConfidenceImage)
    {
      ImGui::SameLine();
      ImGui::SetNextItemWidth(150);
      if (ImGui::SliderInt("Min. confidence", &m_minConfidence, 0, 255))
      {
        m_lastObj.reset();  // rebuild the depth image
      }
      mrpt::imgui::tooltip("Depth pixels with a lower confidence are shown in black.");
    }

    if (obj != m_lastObj || ctx.vizParamsVersion != m_lastVizVersion)
    {
      m_lastObj = obj;
      m_lastVizVersion = ctx.vizParamsVersion;
      rebuild(obs, ctx);
    }
    if (!m_error.empty())
    {
      ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", m_error.c_str());
    }

    if (m_tabs.empty())
    {
      m_view.render();
      return;
    }
    if (ImGui::BeginTabBar("views"))
    {
      if (ImGui::BeginTabItem("3D"))
      {
        m_view.render();
        ImGui::EndTabItem();
      }
      for (auto& [label, view] : m_tabs)
      {
        if (ImGui::BeginTabItem(label.c_str()))
        {
          view->render(("##" + label).c_str());
          ImGui::EndTabItem();
        }
      }
      ImGui::EndTabBar();
    }
  }

 private:
  mrpt::imgui::CImGuiSceneView m_view;
  mrpt::viz::CSetOfObjects::Ptr m_glObs;
  /** Image tabs: label and view. Views are kept across observations, so
   * zoom and pan survive moving along the rawlog. */
  std::vector<std::pair<std::string, std::unique_ptr<mrpt::imgui::CImGuiImageView>>> m_tabs;
  mrpt::serialization::CSerializable::Ptr m_lastObj;
  int m_lastVizVersion = -1;
  int m_minConfidence = 0;
  std::string m_error;

  void setTab(size_t idx, const std::string& label, const mrpt::img::CImage& img)
  {
    if (m_tabs.size() <= idx)
    {
      m_tabs.resize(idx + 1);
    }
    m_tabs[idx].first = label;
    if (!m_tabs[idx].second)
    {
      m_tabs[idx].second = std::make_unique<mrpt::imgui::CImGuiImageView>();
    }
    m_tabs[idx].second->setImage(img);
  }

  void rebuild(const mrpt::obs::CObservation::Ptr& obs, const PanelContext& ctx)
  {
    using namespace mrpt::obs;
    m_error.clear();
    m_glObs->clear();
    try
    {
      obs->load();
      auto oVel = std::dynamic_pointer_cast<CObservationVelodyneScan>(obs);
      const bool generatedCloud = oVel && oVel->point_cloud.size() == 0;
      if (generatedCloud)
      {
        oVel->generatePointCloud();
      }

      if (!obs_to_viz(obs, ctx.vizParams, *m_glObs))
      {
        m_error = "This observation class has no 3D visualization.";
      }
      if (generatedCloud)
      {
        oVel->point_cloud.clear_deep();  // free memory
      }

      size_t nTabs = 0;
      if (auto o3D = std::dynamic_pointer_cast<CObservation3DRangeScan>(obs); o3D)
      {
        if (o3D->hasRangeImage)
        {
          setTab(nTabs++, "Depth", depthImage(*o3D));
        }
        if (o3D->hasIntensityImage)
        {
          setTab(nTabs++, "Intensity", o3D->intensityImage.makeDeepCopy());
        }
        if (o3D->hasConfidenceImage)
        {
          setTab(nTabs++, "Confidence", o3D->confidenceImage.makeDeepCopy());
        }
        o3D->unload();  // for externally-stored datasets
      }
      else if (auto oRS = std::dynamic_pointer_cast<CObservationRotatingScan>(obs); oRS)
      {
        setTab(nTabs++, "Range", normalizedImage(oRS->rangeImage));
        if (oRS->intensityImage.rows() != 0)
        {
          setTab(nTabs++, "Intensity", normalizedImage(oRS->intensityImage));
        }
      }
      m_tabs.resize(nTabs);
    }
    catch (const std::exception& e)
    {
      m_error = std::string("Error loading the observation:\n") + e.what();
    }
  }

  /** Depth with a color map, masking out pixels of low confidence. */
  [[nodiscard]] mrpt::img::CImage depthImage(const mrpt::obs::CObservation3DRangeScan& o) const
  {
    auto img = o.rangeImage_getAsImage(mrpt::img::TColormap::cmHOT);
    const auto& conf = o.confidenceImage;
    if (m_minConfidence <= 0 || !o.hasConfidenceImage || conf.isEmpty() ||
        conf.getWidth() != img.getWidth() || conf.getHeight() != img.getHeight() ||
        conf.channels() != 1)
    {
      return img;
    }
    for (int r = 0; r < static_cast<int>(img.getHeight()); r++)
    {
      for (int c = 0; c < static_cast<int>(img.getWidth()); c++)
      {
        if (conf.at<uint8_t>(c, r) < m_minConfidence)
        {
          img.setPixel({c, r}, mrpt::img::TColor::black());
        }
      }
    }
    return img;
  }
};

}  // namespace

void registerScenePanels(ObservationPanelRegistry& r)
{
  for (const char* c :
       {"mrpt::obs::CObservation2DRangeScan", "mrpt::obs::CObservation3DRangeScan",
        "mrpt::obs::CObservationPointCloud", "mrpt::obs::CObservationRotatingScan",
        "mrpt::obs::CObservationVelodyneScan"})
  {
    r.add(c, factory<PointsPanel>());
  }
}

}  // namespace mrpt::apps
