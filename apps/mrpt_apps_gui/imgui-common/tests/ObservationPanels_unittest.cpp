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

#include <gtest/gtest.h>
#include <imgui.h>
#include <implot.h>
#include <mrpt/apps_imgui/ObservationPanels.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/obs/CSensoryFrame.h>

using namespace mrpt::apps;
using namespace mrpt::obs;

TEST(ObservationPanelRegistry, BuiltinClasses)
{
  const auto& reg = ObservationPanelRegistry::Instance();
  for (const char* c :
       {"mrpt::obs::CObservationImage", "mrpt::obs::CObservationStereoImages",
        "mrpt::obs::CObservation3DRangeScan", "mrpt::obs::CObservation2DRangeScan",
        "mrpt::obs::CObservationPointCloud", "mrpt::obs::CObservationRotatingScan",
        "mrpt::obs::CObservationVelodyneScan", "mrpt::obs::CObservationGPS",
        "mrpt::obs::CObservationIMU", "mrpt::obs::CObservationOdometry",
        "mrpt::obs::CObservationBeaconRanges", "mrpt::obs::CObservationBearingRange",
        "mrpt::obs::CObservationGasSensors", "mrpt::obs::CObservationWirelessPower",
        "mrpt::obs::CObservationRange"})
  {
    EXPECT_TRUE(reg.has(c)) << c;
  }
}

TEST(ObservationPanelRegistry, LookupAndFallback)
{
  const auto& reg = ObservationPanelRegistry::Instance();
  EXPECT_STREQ(reg.create(CLASS_ID(CObservationImage))->name(), "Image");
  EXPECT_STREQ(reg.create(CLASS_ID(CObservationIMU))->name(), "IMU");
  EXPECT_STREQ(reg.create(CLASS_ID(CObservationVelodyneScan))->name(), "3D points");
  EXPECT_STREQ(reg.create(CLASS_ID(CActionRobotMovement2D))->name(), "2D motion");
  // Classes without a panel get the text description:
  EXPECT_STREQ(reg.create(CLASS_ID(CObservationComment))->name(), "Description");
  EXPECT_STREQ(reg.create(CLASS_ID(CSensoryFrame))->name(), "Description");
  EXPECT_STREQ(reg.create(nullptr)->name(), "Description");
}

TEST(ObservationPanelRegistry, CustomPanelOverridesBuiltin)
{
  struct MyPanel : public ObservationPanel
  {
    void render(const mrpt::serialization::CSerializable::Ptr&, PanelContext&) override {}
    [[nodiscard]] const char* name() const override { return "Mine"; }
  };
  ObservationPanelRegistry reg;  // a separate registry, not the global one
  registerTextPanels(reg);
  reg.add("mrpt::obs::CObservation", []() { return std::make_unique<MyPanel>(); });
  // Most derived registered class wins; otherwise, its base class:
  EXPECT_STREQ(reg.create(CLASS_ID(CObservationIMU))->name(), "IMU");
  EXPECT_STREQ(reg.create(CLASS_ID(CObservationComment))->name(), "Mine");
}

TEST(ObservationPanels, DescribeObject)
{
  CObservationOdometry odo;
  odo.sensorLabel = "my_odo";
  odo.odometry = mrpt::poses::CPose2D(1, 2, 0);
  const auto s = describeObject(odo);
  EXPECT_NE(s.find("my_odo"), std::string::npos);

  CSensoryFrame sf;
  sf.insert(CObservationOdometry::Create());
  EXPECT_NE(describeObject(sf).find("1 observations"), std::string::npos);
}

namespace
{
/** An ImGui + ImPlot context without any rendering backend. */
class HeadlessImGui
{
 public:
  HeadlessImGui()
  {
    ImGui::CreateContext();
    ImPlot::CreateContext();
    auto& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1280, 800);
    io.DeltaTime = 1.0f / 60.0f;
    io.IniFilename = nullptr;
    unsigned char* pixels = nullptr;
    int w = 0;
    int h = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
  }
  ~HeadlessImGui()
  {
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
  }
  HeadlessImGui(const HeadlessImGui&) = delete;
  HeadlessImGui& operator=(const HeadlessImGui&) = delete;

  /** Renders the panel in a window during a few frames. */
  void renderFrames(
      ObservationPanel& panel,
      const mrpt::serialization::CSerializable::Ptr& obj,
      PanelContext& ctx)
  {
    for (int i = 0; i < 3; i++)
    {
      ImGui::NewFrame();
      ImGui::SetNextWindowSize(ImVec2(600, 500));
      ImGui::Begin("panel");
      panel.render(obj, ctx);
      ImGui::End();
      ImGui::Render();
    }
  }
};

std::vector<mrpt::serialization::CSerializable::Ptr> sampleObjects()
{
  std::vector<mrpt::serialization::CSerializable::Ptr> v;
  {
    auto o = CObservationGPS::Create();
    gnss::Message_NMEA_GGA gga;
    gga.fields.latitude_degrees = 36.8;
    o->setMsg(gga);
    v.push_back(o);
  }
  {
    auto o = CObservationIMU::Create();
    o->set(IMU_WZ, 0.1);
    v.push_back(o);
  }
  v.push_back(CObservationOdometry::Create());
  {
    auto o = CObservationBeaconRanges::Create();
    CObservationBeaconRanges::TMeasurement m;
    m.beaconID = 3;
    m.sensedDistance = 2.5f;
    o->sensedData.push_back(m);
    v.push_back(o);
  }
  {
    auto o = CObservationBearingRange::Create();
    CObservationBearingRange::TMeasurement m;
    m.range = 3;
    m.yaw = 0.5f;
    m.pitch = 0;
    m.landmarkID = 1;
    o->sensedData.push_back(m);
    v.push_back(o);
  }
  {
    auto o = CObservationGasSensors::Create();
    CObservationGasSensors::TObservationENose e;
    e.readingsVoltage = {1.0f, 2.0f};
    e.sensorTypes = {0x2600, 0x2602};
    e.isActive = true;
    o->m_readings.push_back(e);
    v.push_back(o);
  }
  v.push_back(CObservationWirelessPower::Create());
  {
    auto o = CObservationRange::Create();
    o->sensedData.resize(2);
    v.push_back(o);
  }
  {
    auto a = CActionRobotMovement2D::Create();
    CActionRobotMovement2D::TMotionModelOptions opts;
    a->computeFromOdometry(mrpt::poses::CPose2D(1, 0, 0), opts);
    v.push_back(a);
  }
  v.push_back(CObservationComment::Create());
  return v;
}
}  // namespace

TEST(ObservationPanels, RenderTextPanels)
{
  HeadlessImGui gui;
  PanelContext ctx;
  ctx.referenceTime = mrpt::Clock::now();
  const auto& reg = ObservationPanelRegistry::Instance();
  for (const auto& obj : sampleObjects())
  {
    SCOPED_TRACE(obj->GetRuntimeClass()->className);
    auto panel = reg.create(obj->GetRuntimeClass());
    gui.renderFrames(*panel, obj, ctx);
    gui.renderFrames(*panel, nullptr, ctx);  // no data
  }
}

TEST(ObservationPanels, EditVisualizationParameters)
{
  HeadlessImGui gui;
  PanelContext ctx;
  ImGui::NewFrame();
  ImGui::Begin("params");
  EXPECT_FALSE(editVisualizationParameters(ctx));  // nothing touched
  ImGui::End();
  ImGui::Render();
  EXPECT_EQ(ctx.vizParamsVersion, 0);
}
