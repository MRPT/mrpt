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
#include <implot.h>
#include <mrpt/apps_imgui/ObservationPanels.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationWirelessPower.h>

#include <array>
#include <cmath>
#include <sstream>

#include "panel_utils.h"

namespace mrpt::apps
{
namespace
{
using detail::factory;
using detail::TypedPanel;

constexpr ImGuiTableFlags TABLE_FLAGS = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                        ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable;

void tableRow(const char* key, const std::string& value)
{
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextUnformatted(key);
  ImGui::TableSetColumnIndex(1);
  ImGui::TextUnformatted(value.c_str());
}

// --------------------------------------------------------------------------
class GPSPanel : public TypedPanel<mrpt::obs::CObservationGPS>
{
 public:
  [[nodiscard]] const char* name() const override { return "GPS"; }

 protected:
  void renderObs(mrpt::obs::CObservationGPS& o, PanelContext&, bool /*changed*/) override
  {
    using namespace mrpt::obs::gnss;
    ImGui::Text(
        "Satellite-based timestamp: %s", o.has_satellite_timestamp ? "yes" : "no (PC clock)");
    if (const auto* gga = o.getMsgByClassPtr<Message_NMEA_GGA>(); gga)
    {
      ImGui::Text("Latitude:  %.8f deg", gga->fields.latitude_degrees);
      ImGui::Text("Longitude: %.8f deg", gga->fields.longitude_degrees);
      ImGui::Text("Altitude:  %.3f m", gga->fields.altitude_meters);
      ImGui::Text(
          "Fix quality: %u  Satellites: %u  HDOP: %.2f",
          static_cast<unsigned>(gga->fields.fix_quality),
          static_cast<unsigned>(gga->fields.satellitesUsed), gga->fields.HDOP);
    }
    if (o.covariance_enu.has_value())
    {
      const auto& cov = o.covariance_enu.value();
      ImGui::Text(
          "Std. dev. ENU [m]: %.3f %.3f %.3f", std::sqrt(cov(0, 0)), std::sqrt(cov(1, 1)),
          std::sqrt(cov(2, 2)));
    }
    ImGui::SeparatorText("Messages");
    for (const auto& [type, msg] : o.messages)
    {
      if (!msg)
      {
        continue;
      }
      ImGui::PushID(static_cast<int>(type));
      if (ImGui::CollapsingHeader(msg->getMessageTypeAsString().c_str()))
      {
        std::stringstream ss;
        msg->dumpToConsole(ss);
        ImGui::TextUnformatted(ss.str().c_str());
      }
      ImGui::PopID();
    }
  }
};

// --------------------------------------------------------------------------
class IMUPanel : public TypedPanel<mrpt::obs::CObservationIMU>
{
 public:
  [[nodiscard]] const char* name() const override { return "IMU"; }

 protected:
  void renderObs(mrpt::obs::CObservationIMU& o, PanelContext&, bool /*changed*/) override
  {
    using namespace mrpt::obs;
    // Names of the TIMUDataIndex values, in order:
    static const std::array<const char*, COUNT_IMU_DATA_FIELDS> names = {
        "x acc. [m/s2]",
        "y acc. [m/s2]",
        "z acc. [m/s2]",
        "yaw vel. [rad/s]",
        "pitch vel. [rad/s]",
        "roll vel. [rad/s]",
        "x vel. [m/s]",
        "y vel. [m/s]",
        "z vel. [m/s]",
        "yaw [rad]",
        "pitch [rad]",
        "roll [rad]",
        "x [m]",
        "y [m]",
        "z [m]",
        "mag. x [gauss]",
        "mag. y [gauss]",
        "mag. z [gauss]",
        "pressure [Pa]",
        "altitude [m]",
        "temperature [C]",
        "quat. x",
        "quat. y",
        "quat. z",
        "quat. w",
        "yaw vel. global",
        "pitch vel. global",
        "roll vel. global",
        "x acc. global",
        "y acc. global",
        "z acc. global"};

    ImGui::Checkbox("Show missing fields", &m_showMissing);
    if (ImGui::BeginTable("imu", 2, TABLE_FLAGS))
    {
      ImGui::TableSetupColumn("Field");
      ImGui::TableSetupColumn("Value");
      ImGui::TableHeadersRow();
      for (size_t i = 0; i < COUNT_IMU_DATA_FIELDS; i++)
      {
        const bool present = o.dataIsPresent.at(i);
        if (!present && !m_showMissing)
        {
          continue;
        }
        tableRow(
            names.at(i), present ? mrpt::format("%.6f", o.rawMeasurements.at(i)) : "(missing)");
      }
      ImGui::EndTable();
    }
  }

 private:
  bool m_showMissing = false;
};

// --------------------------------------------------------------------------
class OdometryPanel : public TypedPanel<mrpt::obs::CObservationOdometry>
{
 public:
  [[nodiscard]] const char* name() const override { return "Odometry"; }

 protected:
  void renderObs(mrpt::obs::CObservationOdometry& o, PanelContext&, bool /*changed*/) override
  {
    ImGui::Text(
        "Odometry: x=%.4f m  y=%.4f m  phi=%.3f deg", o.odometry.x(), o.odometry.y(),
        mrpt::RAD2DEG(o.odometry.phi()));
    if (o.hasEncodersInfo)
    {
      ImGui::Text(
          "Encoder ticks: left=%i  right=%i", static_cast<int>(o.encoderLeftTicks),
          static_cast<int>(o.encoderRightTicks));
    }
    else
    {
      ImGui::TextDisabled("Encoder ticks: (none)");
    }
    if (o.hasVelocities)
    {
      ImGui::Text("Velocity (local): %s", o.velocityLocal.asString().c_str());
    }
    else
    {
      ImGui::TextDisabled("Velocity: (none)");
    }
  }
};

// --------------------------------------------------------------------------
class BeaconRangesPanel : public TypedPanel<mrpt::obs::CObservationBeaconRanges>
{
 public:
  [[nodiscard]] const char* name() const override { return "Beacon ranges"; }

 protected:
  void renderObs(mrpt::obs::CObservationBeaconRanges& o, PanelContext&, bool /*changed*/) override
  {
    ImGui::Text(
        "Range: [%.2f, %.2f] m  Std. error: %.3f m", o.minSensorDistance, o.maxSensorDistance,
        o.stdError);
    if (ImGui::BeginTable("beacons", 3, TABLE_FLAGS))
    {
      ImGui::TableSetupColumn("Beacon ID");
      ImGui::TableSetupColumn("Range [m]");
      ImGui::TableSetupColumn("Sensor on robot");
      ImGui::TableHeadersRow();
      for (const auto& m : o.sensedData)
      {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%i", static_cast<int>(m.beaconID));
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.4f", m.sensedDistance);
        ImGui::TableSetColumnIndex(2);
        ImGui::TextUnformatted(m.sensorLocationOnRobot.asString().c_str());
      }
      ImGui::EndTable();
    }
  }
};

// --------------------------------------------------------------------------
class BearingRangePanel : public TypedPanel<mrpt::obs::CObservationBearingRange>
{
 public:
  [[nodiscard]] const char* name() const override { return "Range-bearing"; }

 protected:
  void renderObs(mrpt::obs::CObservationBearingRange& o, PanelContext&, bool /*changed*/) override
  {
    // Landmarks in the robot frame:
    std::vector<double> xs;
    std::vector<double> ys;
    for (const auto& m : o.sensedData)
    {
      const mrpt::poses::CPoint3D local(
          m.range * std::cos(m.yaw) * std::cos(m.pitch),
          m.range * std::sin(m.yaw) * std::cos(m.pitch), m.range * std::sin(m.pitch));
      const auto p = o.sensorLocationOnRobot + local;
      xs.push_back(p.x());
      ys.push_back(p.y());
    }
    const float tableHeight = ImGui::GetContentRegionAvail().y * 0.4f;
    if (ImPlot::BeginPlot("##landmarks", ImVec2(-1, -tableHeight), ImPlotFlags_Equal))
    {
      ImPlot::SetupAxes("x [m]", "y [m]", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
      const double origin[1] = {0.0};
      ImPlot::PlotScatter(
          "Robot", origin, origin, 1, ImPlotSpec(ImPlotProp_Marker, ImPlotMarker_Square));
      ImPlot::PlotScatter(
          "Landmarks", xs.data(), ys.data(), static_cast<int>(xs.size()),
          ImPlotSpec(ImPlotProp_Marker, ImPlotMarker_Circle));
      ImPlot::EndPlot();
    }
    if (ImGui::BeginTable("landmarks", 4, TABLE_FLAGS))
    {
      ImGui::TableSetupColumn("ID");
      ImGui::TableSetupColumn("Range [m]");
      ImGui::TableSetupColumn("Yaw [deg]");
      ImGui::TableSetupColumn("Pitch [deg]");
      ImGui::TableHeadersRow();
      for (const auto& m : o.sensedData)
      {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%i", static_cast<int>(m.landmarkID));
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.4f", m.range);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.3f", mrpt::RAD2DEG(m.yaw));
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.3f", mrpt::RAD2DEG(m.pitch));
      }
      ImGui::EndTable();
    }
  }
};

// --------------------------------------------------------------------------
class GasSensorsPanel : public TypedPanel<mrpt::obs::CObservationGasSensors>
{
 public:
  [[nodiscard]] const char* name() const override { return "Gas sensors"; }

 protected:
  void renderObs(mrpt::obs::CObservationGasSensors& o, PanelContext&, bool /*changed*/) override
  {
    for (size_t i = 0; i < o.m_readings.size(); i++)
    {
      const auto& e = o.m_readings[i];
      ImGui::PushID(static_cast<int>(i));
      ImGui::SeparatorText(mrpt::format("e-nose #%zu", i).c_str());
      ImGui::Text("Pose on robot: %s", e.eNosePoseOnTheRobot.asString().c_str());
      if (e.hasTemperature)
      {
        ImGui::Text("Temperature: %.2f C", e.temperature);
      }
      if (ImGui::BeginTable("readings", 2, TABLE_FLAGS & ~ImGuiTableFlags_ScrollY))
      {
        ImGui::TableSetupColumn("Sensor type");
        ImGui::TableSetupColumn("Reading [V]");
        ImGui::TableHeadersRow();
        for (size_t k = 0; k < e.readingsVoltage.size(); k++)
        {
          tableRow(
              k < e.sensorTypes.size() ? mrpt::format("0x%04X", e.sensorTypes[k]).c_str() : "?",
              mrpt::format("%.4f", e.readingsVoltage[k]));
        }
        ImGui::EndTable();
      }
      ImGui::PopID();
    }
  }
};

// --------------------------------------------------------------------------
class WirelessPowerPanel : public TypedPanel<mrpt::obs::CObservationWirelessPower>
{
 public:
  [[nodiscard]] const char* name() const override { return "Wireless power"; }

 protected:
  void renderObs(mrpt::obs::CObservationWirelessPower& o, PanelContext&, bool /*changed*/) override
  {
    ImGui::Text("Power: %.3f", o.power);
  }
};

// --------------------------------------------------------------------------
class RangePanel : public TypedPanel<mrpt::obs::CObservationRange>
{
 public:
  [[nodiscard]] const char* name() const override { return "Range sensors"; }

 protected:
  void renderObs(mrpt::obs::CObservationRange& o, PanelContext&, bool /*changed*/) override
  {
    ImGui::Text("Range: [%.2f, %.2f] m", o.minSensorDistance, o.maxSensorDistance);
    if (ImGui::BeginTable("ranges", 4, TABLE_FLAGS))
    {
      ImGui::TableSetupColumn("ID");
      ImGui::TableSetupColumn("Range [m]");
      ImGui::TableSetupColumn("Std. dev. [m]");
      ImGui::TableSetupColumn("Sensor pose");
      ImGui::TableHeadersRow();
      for (const auto& m : o.sensedData)
      {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%u", static_cast<unsigned>(m.sensorID));
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.4f", m.sensedDistance);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.4f", m.sensorNoiseStdDeviation);
        ImGui::TableSetColumnIndex(3);
        ImGui::TextUnformatted(m.sensorPose.asString().c_str());
      }
      ImGui::EndTable();
    }
  }
};

// --------------------------------------------------------------------------
/** 2D motion actions: random samples of the pose increment. */
class Movement2DPanel : public TypedPanel<mrpt::obs::CActionRobotMovement2D>
{
 public:
  [[nodiscard]] const char* name() const override { return "2D motion"; }

 protected:
  void renderObs(mrpt::obs::CActionRobotMovement2D& a, PanelContext&, bool changed) override
  {
    if (changed)
    {
      constexpr size_t N = 1000;
      std::vector<mrpt::math::CVectorDouble> samples;
      a.poseChange->drawManySamples(N, samples);
      m_xs.clear();
      m_ys.clear();
      m_phis.clear();
      for (const auto& s : samples)
      {
        m_xs.push_back(s[0]);
        m_ys.push_back(s[1]);
        m_phis.push_back(mrpt::RAD2DEG(s[2]));
      }
      m_mean = a.poseChange->getMeanVal().asString();
    }
    ImGui::Text("Mean increment: %s", m_mean.c_str());
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    if (ImPlot::BeginPlot("x-y samples", ImVec2(avail.x * 0.6f, -1), ImPlotFlags_Equal))
    {
      ImPlot::SetupAxes("x [m]", "y [m]", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
      ImPlot::PlotScatter(
          "##xy", m_xs.data(), m_ys.data(), static_cast<int>(m_xs.size()),
          ImPlotSpec(ImPlotProp_MarkerSize, 1.5f));
      ImPlot::EndPlot();
    }
    ImGui::SameLine();
    if (ImPlot::BeginPlot("phi samples", ImVec2(-1, -1)))
    {
      ImPlot::SetupAxes("phi [deg]", "count", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
      ImPlot::PlotHistogram("##phi", m_phis.data(), static_cast<int>(m_phis.size()));
      ImPlot::EndPlot();
    }
  }

 private:
  std::vector<double> m_xs;
  std::vector<double> m_ys;
  std::vector<double> m_phis;
  std::string m_mean;
};

}  // namespace

void registerTextPanels(ObservationPanelRegistry& r)
{
  r.add("mrpt::obs::CObservationGPS", factory<GPSPanel>());
  r.add("mrpt::obs::CObservationIMU", factory<IMUPanel>());
  r.add("mrpt::obs::CObservationOdometry", factory<OdometryPanel>());
  r.add("mrpt::obs::CObservationBeaconRanges", factory<BeaconRangesPanel>());
  r.add("mrpt::obs::CObservationBearingRange", factory<BearingRangePanel>());
  r.add("mrpt::obs::CObservationGasSensors", factory<GasSensorsPanel>());
  r.add("mrpt::obs::CObservationWirelessPower", factory<WirelessPowerPanel>());
  r.add("mrpt::obs::CObservationRange", factory<RangePanel>());
  r.add("mrpt::obs::CActionRobotMovement2D", factory<Movement2DPanel>());
}

}  // namespace mrpt::apps
