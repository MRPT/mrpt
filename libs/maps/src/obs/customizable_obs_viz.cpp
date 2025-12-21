/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
#include "mrpt/math/ops_containers.h"
//
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>

#include <algorithm>  // std::clamp

using namespace mrpt::obs;

void VisualizationParameters::save_to_ini_file(
    mrpt::config::CConfigFileBase& cfg, const std::string& section) const
{
  auto& c = cfg;
  const std::string& s = section;

  MRPT_SAVE_CONFIG_VAR(axisTickFrequency, c, s);
  MRPT_SAVE_CONFIG_VAR(axisLimits, c, s);
  MRPT_SAVE_CONFIG_VAR(axisTickTextSize, c, s);
  MRPT_SAVE_CONFIG_VAR(colorFromRGBimage, c, s);
  MRPT_SAVE_CONFIG_VAR(pointSize, c, s);
  MRPT_SAVE_CONFIG_VAR(drawSensorPose, c, s);
  MRPT_SAVE_CONFIG_VAR(sensorPoseScale, c, s);
  MRPT_SAVE_CONFIG_VAR(showAxis, c, s);
  MRPT_SAVE_CONFIG_VAR(showSurfaceIn2Dscans, c, s);
  MRPT_SAVE_CONFIG_VAR(showPointsIn2Dscans, c, s);
  MRPT_SAVE_CONFIG_VAR(onlyPointsWithColor, c, s);

  MRPT_SAVE_CONFIG_VAR(surface2DscansColor.R, c, s);
  MRPT_SAVE_CONFIG_VAR(surface2DscansColor.G, c, s);
  MRPT_SAVE_CONFIG_VAR(surface2DscansColor.B, c, s);
  MRPT_SAVE_CONFIG_VAR(surface2DscansColor.A, c, s);

  MRPT_SAVE_CONFIG_VAR(points2DscansColor.R, c, s);
  MRPT_SAVE_CONFIG_VAR(points2DscansColor.G, c, s);
  MRPT_SAVE_CONFIG_VAR(points2DscansColor.B, c, s);
  MRPT_SAVE_CONFIG_VAR(points2DscansColor.A, c, s);

  coloring.save_to_ini_file(cfg, section);
}

void VisualizationParameters::load_from_ini_file(
    const mrpt::config::CConfigFileBase& cfg, const std::string& section)
{
  const auto& c = cfg;
  const std::string& s = section;

  MRPT_LOAD_CONFIG_VAR_CS(axisTickFrequency, double);
  MRPT_LOAD_CONFIG_VAR_CS(axisLimits, double);
  MRPT_LOAD_CONFIG_VAR_CS(axisTickTextSize, double);
  MRPT_LOAD_CONFIG_VAR_CS(colorFromRGBimage, bool);
  MRPT_LOAD_CONFIG_VAR_CS(pointSize, double);
  MRPT_LOAD_CONFIG_VAR_CS(drawSensorPose, bool);
  MRPT_LOAD_CONFIG_VAR_CS(sensorPoseScale, double);
  MRPT_LOAD_CONFIG_VAR_CS(showAxis, bool);
  MRPT_LOAD_CONFIG_VAR_CS(showSurfaceIn2Dscans, bool);
  MRPT_LOAD_CONFIG_VAR_CS(showPointsIn2Dscans, bool);
  MRPT_LOAD_CONFIG_VAR_CS(onlyPointsWithColor, bool);

  MRPT_LOAD_CONFIG_VAR_CS(surface2DscansColor.R, int);
  MRPT_LOAD_CONFIG_VAR_CS(surface2DscansColor.G, int);
  MRPT_LOAD_CONFIG_VAR_CS(surface2DscansColor.B, int);
  MRPT_LOAD_CONFIG_VAR_CS(surface2DscansColor.A, int);

  MRPT_LOAD_CONFIG_VAR_CS(points2DscansColor.R, int);
  MRPT_LOAD_CONFIG_VAR_CS(points2DscansColor.G, int);
  MRPT_LOAD_CONFIG_VAR_CS(points2DscansColor.B, int);
  MRPT_LOAD_CONFIG_VAR_CS(points2DscansColor.A, int);

  coloring.load_from_ini_file(cfg, section);
}

void PointCloudRecoloringParameters::save_to_ini_file(
    mrpt::config::CConfigFileBase& cfg, const std::string& section) const
{
  auto& c = cfg;
  const std::string& s = section;

  MRPT_SAVE_CONFIG_VAR(colorizeByField, c, s);
  MRPT_SAVE_CONFIG_VAR(invertColorMapping, c, s);
  MRPT_SAVE_CONFIG_VAR(colorMap, c, s);

  if (colorMapMinCoord)
  {
    c.write(s, "colorMapMinCoord", *colorMapMinCoord);
  }
  if (colorMapMaxCoord)
  {
    c.write(s, "colorMapMaxCoord", *colorMapMaxCoord);
  }
}

void PointCloudRecoloringParameters::load_from_ini_file(
    const mrpt::config::CConfigFileBase& cfg, const std::string& section)
{
  const auto& c = cfg;
  const std::string& s = section;

  MRPT_LOAD_CONFIG_VAR_CS(colorizeByField, string);
  MRPT_LOAD_CONFIG_VAR_CS(invertColorMapping, bool);
  colorMap = c.read_enum(s, "colorMap", colorMap);

  if (const auto sMin = c.read_string(s, "colorMapMinCoord", ""); !sMin.empty())
  {
    colorMapMinCoord = std::stof(sMin);
  }
  if (const auto sMax = c.read_string(s, "colorMapMaxCoord", ""); !sMax.empty())
  {
    colorMapMaxCoord = std::stof(sMax);
  }
}

// Bounding box memory so we have consistent coloring across different sensors:
namespace
{
thread_local std::map<std::string, std::pair<float, float>> bbMemory;
constexpr float bbMemoryFading = 0.99f;

void printRecolorLengthMismatchError()
{
  std::cerr << "[mrpt::obs::recolorize3Dpc] Warning: Skipping colorization due to mismatch in "
               "vector lengths.\n";
}

}  // namespace

void mrpt::obs::recolorize3Dpc(
    const mrpt::opengl::CPointCloudColoured::Ptr& pnts,
    const mrpt::maps::CPointsMap* originalPts,
    const PointCloudRecoloringParameters& p)
{
  if (!originalPts || p.colorMap == mrpt::img::cmNONE)
  {
    return;
  }

  std::pair<float, float> dataLimits{0, 0};
  std::size_t dataPoints = 0;

  // Special RGB cases:
  //    * - `rgb` for `{color_r,color_g,color_b}` (uint8_t, in range 0-255)
  // -------------------------------------------------------------------------------------------
  if (p.colorizeByField == "rgb")
  {
    const auto* data_r =
        originalPts->getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8);
    const auto* data_g =
        originalPts->getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8);
    const auto* data_b =
        originalPts->getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8);

    if (!data_r || !data_g || !data_b)
    {
      return;  // Missing fields!
    }

    if (data_r->size() != pnts->size() || data_g->size() != pnts->size() ||
        data_b->size() != pnts->size())
    {
      printRecolorLengthMismatchError();
      return;
    }

    for (size_t i = 0; i < pnts->size(); i++)
    {
      pnts->setPointColor_fast(
          i, mrpt::f2u8((*data_r)[i]), mrpt::f2u8((*data_g)[i]), mrpt::f2u8((*data_b)[i]));
    }

    return;
  }

  // Special RGB cases:
  //   * - `rgbf` for `{color_rf,color_gf,color_bf}` (float, in range 0-1)
  // -------------------------------------------------------------------------------------------
  if (p.colorizeByField == "rgbf")
  {
    const auto* data_r =
        originalPts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Rf);
    const auto* data_g =
        originalPts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gf);
    const auto* data_b =
        originalPts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bf);

    if (!data_r || !data_g || !data_b)
    {
      return;  // Missing fields!
    }

    if (data_r->size() != pnts->size() || data_g->size() != pnts->size() ||
        data_b->size() != pnts->size())
    {
      printRecolorLengthMismatchError();
      return;
    }

    for (size_t i = 0; i < pnts->size(); i++)
    {
      pnts->setPointColor_fast(i, (*data_r)[i], (*data_g)[i], (*data_b)[i]);
    }

    return;
  }

  // Generic field coloring:
  // -------------------------------------------------------------------------------------------
  const mrpt::aligned_std_vector<float>* fieldData_f = nullptr;
  const mrpt::aligned_std_vector<double>* fieldData_d = nullptr;
  const mrpt::aligned_std_vector<uint16_t>* fieldData_u16 = nullptr;
  const mrpt::aligned_std_vector<uint8_t>* fieldData_u8 = nullptr;

  if (fieldData_f = originalPts->getPointsBufferRef_float_field(p.colorizeByField); fieldData_f)
  {
    dataPoints = fieldData_f->size();
    if (dataPoints)
    {
      mrpt::math::minimum_maximum(*fieldData_f, dataLimits.first, dataLimits.second);
    }
  }
  else if (fieldData_d = originalPts->getPointsBufferRef_double_field(p.colorizeByField);
           fieldData_d)
  {
    dataPoints = fieldData_d->size();
    if (dataPoints)
    {
      double uMin = 0, uMax = 0;
      mrpt::math::minimum_maximum(*fieldData_d, uMin, uMax);
      dataLimits.first = static_cast<float>(uMin);
      dataLimits.second = static_cast<float>(uMax);
    }
  }
  else if (fieldData_u16 = originalPts->getPointsBufferRef_uint_field(p.colorizeByField);
           fieldData_u16)
  {
    dataPoints = fieldData_u16->size();

    if (dataPoints)
    {
      uint16_t uMin = 0, uMax = 0;
      mrpt::math::minimum_maximum(*fieldData_u16, uMin, uMax);
      dataLimits.first = static_cast<float>(uMin);
      dataLimits.second = static_cast<float>(uMax);
    }
  }
  else if (fieldData_u8 = originalPts->getPointsBufferRef_uint8_field(p.colorizeByField);
           fieldData_u8)
  {
    dataPoints = fieldData_u8->size();

    if (dataPoints)
    {
      uint8_t uMin = 0, uMax = 0;
      mrpt::math::minimum_maximum(*fieldData_u8, uMin, uMax);
      dataLimits.first = static_cast<float>(uMin);
      dataLimits.second = static_cast<float>(uMax);
    }
  }

  if (!fieldData_f && !fieldData_u16 && !fieldData_d && !fieldData_u8)
  {
    return;
  }

  if (dataPoints != pnts->size())
  {
    printRecolorLengthMismatchError();
    return;
  }

  // Slowly update bb memory:
  if (!bbMemory.count(p.colorizeByField))
  {
    // first time:
    bbMemory[p.colorizeByField] = dataLimits;
  }
  else
  {
    auto& bb = bbMemory[p.colorizeByField];
    bb.first = bb.first * bbMemoryFading + dataLimits.first * (1.0f - bbMemoryFading);
    bb.second = bb.second * bbMemoryFading + dataLimits.second * (1.0f - bbMemoryFading);
  }
  const auto& bb = bbMemory[p.colorizeByField];

  const float colMin = p.invertColorMapping ? bb.second : bb.first;
  const float colMax = p.invertColorMapping ? bb.first : bb.second;

  const float coord_range = colMax - colMin;
  const float coord_range_1 = coord_range != 0.0f ? 1.0f / coord_range : 1.0f;
  for (size_t i = 0; i < pnts->size(); i++)
  {
    const float col_idx = [&]()
    {
      if (fieldData_d)
      {
        const double coord = fieldData_d->at(i);
        return std::clamp(static_cast<float>(coord - colMin) * coord_range_1, 0.0f, 1.0f);
      }
      if (fieldData_f)
      {
        const float coord = fieldData_f->at(i);
        return std::clamp((coord - colMin) * coord_range_1, 0.0f, 1.0f);
      }
      if (fieldData_u8)
      {
        const float coord = static_cast<float>(fieldData_u8->at(i));
        return std::clamp((coord - colMin) * coord_range_1, 0.0f, 1.0f);
      }
      if (fieldData_u16)
      {
        const float coord = static_cast<float>(fieldData_u16->at(i));
        return std::clamp((coord - colMin) * coord_range_1, 0.0f, 1.0f);
      }
      return 0.0f;
    }();

    const auto rgb = mrpt::img::TColorf(mrpt::img::colormap(p.colorMap, col_idx));
    pnts->setPointColor_fast(i, rgb.R, rgb.G, rgb.B);
  }
}

namespace
{
void add_common_to_viz(
    const CObservation& obs, const VisualizationParameters& p, mrpt::opengl::CSetOfObjects& out)
{
  if (p.showAxis)
  {
    const float L = p.axisLimits;
    auto gl_axis = mrpt::opengl::CAxis::Create(-L, -L, -L, L, L, L, p.axisTickFrequency, 2, true);
    gl_axis->setTextScale(p.axisTickTextSize);
    gl_axis->setColor_u8(0xa0, 0xa0, 0xa0, 0x80);
    out.insert(gl_axis);

    // Show axis labels such that they can be read from the IIIrd-IVth
    // quadrant:
    const float yawIncrs[3] = {180.f, -90.f, 180.f};
    for (int axis = 0; axis < 3; axis++)
    {
      float yaw_deg, pitch_deg, roll_deg;
      gl_axis->getTextLabelOrientation(axis, yaw_deg, pitch_deg, roll_deg);
      yaw_deg += yawIncrs[axis];
      gl_axis->setTextLabelOrientation(axis, yaw_deg, pitch_deg, roll_deg);
    }
  }

  if (p.drawSensorPose)
  {
    const auto glCorner = mrpt::opengl::stock_objects::CornerXYZSimple(p.sensorPoseScale);
    glCorner->setPose(obs.sensorPose());
    out.insert(glCorner);
  }
}
}  // namespace

void mrpt::obs::obs3Dscan_to_viz(
    const CObservation3DRangeScan::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::opengl::CSetOfObjects& out)
{
  out.clear();

  // Generate/load 3D points
  // ----------------------
  mrpt::maps::CPointsMap::Ptr pointMap;
  mrpt::maps::CColouredPointsMap::Ptr pointMapCol;
  mrpt::obs::T3DPointsProjectionParams pp;
  pp.takeIntoAccountSensorPoseOnRobot = true;
  pp.onlyPointsWithIntensityColor = p.onlyPointsWithColor;

  // Color from intensity image?
  if (p.colorFromRGBimage && obs->hasRangeImage && obs->hasIntensityImage)
  {
    pointMapCol = mrpt::maps::CColouredPointsMap::Create();
    pointMapCol->colorScheme.scheme = mrpt::maps::CColouredPointsMap::cmFromIntensityImage;

    obs->unprojectInto(*pointMapCol, pp);
    pointMap = pointMapCol;
  }
  else
  {
    // Empty point set, or load from XYZ in observation:
    pointMap = mrpt::maps::CSimplePointsMap::Create();
    if (obs->hasPoints3D)
    {
      for (size_t i = 0; i < obs->points3D_x.size(); i++)
        pointMap->insertPoint(obs->points3D_x[i], obs->points3D_y[i], obs->points3D_z[i]);
    }
    else if (obs->hasRangeImage)
    {
      obs->unprojectInto(*pointMap, pp);
    }
  }

  add_common_to_viz(*obs, p, out);

  auto gl_pnts = mrpt::opengl::CPointCloudColoured::Create();
  // Load as RGB or grayscale points:
  if (pointMapCol)
    gl_pnts->loadFromPointsMap(pointMapCol.get());
  else
  {
    gl_pnts->loadFromPointsMap(pointMap.get());
    recolorize3Dpc(gl_pnts, pointMap.get(), p.coloring);
  }

  // No need to further transform 3D points
  gl_pnts->setPose(mrpt::poses::CPose3D());
  gl_pnts->setPointSize(p.pointSize);

  out.insert(gl_pnts);
}

void mrpt::obs::obsVelodyne_to_viz(
    const CObservationVelodyneScan::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::opengl::CSetOfObjects& out)
{
  out.clear();

  add_common_to_viz(*obs, p, out);

  auto pnts = mrpt::opengl::CPointCloudColoured::Create();
  out.insert(pnts);

  mrpt::maps::CColouredPointsMap pntsMap;
  pntsMap.loadFromVelodyneScan(*obs);
  pnts->loadFromPointsMap(&pntsMap);
  pnts->setPointSize(p.pointSize);

  if (!p.colorFromRGBimage)
  {
    recolorize3Dpc(pnts, &pntsMap, p.coloring);
  }
}

void mrpt::obs::obsPointCloud_to_viz(
    const mrpt::obs::CObservationPointCloud::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::opengl::CSetOfObjects& out)
{
  out.clear();

  add_common_to_viz(*obs, p, out);

  auto pnts = mrpt::opengl::CPointCloudColoured::Create();
  out.insert(pnts);

  if (obs->pointcloud) pnts->loadFromPointsMap(obs->pointcloud.get());
  pnts->setPose(obs->sensorPose);

  pnts->setPointSize(p.pointSize);

  if (!p.colorFromRGBimage)
  {
    recolorize3Dpc(pnts, obs->pointcloud.get(), p.coloring);
  }
}

void mrpt::obs::obsRotatingScan_to_viz(
    const mrpt::obs::CObservationRotatingScan::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::opengl::CSetOfObjects& out)
{
  out.clear();

  add_common_to_viz(*obs, p, out);

  auto pnts = mrpt::opengl::CPointCloudColoured::Create();
  out.insert(pnts);

  if (!obs->organizedPoints.empty())
  {
    for (const auto& pt : obs->organizedPoints) pnts->insertPoint({pt.x, pt.y, pt.z, 0, 0, 0, 0});
  }
  pnts->setPose(obs->sensorPose);

  pnts->setPointSize(p.pointSize);

  if (!p.colorFromRGBimage)
  {
    recolorize3Dpc(pnts, nullptr, p.coloring);
  }
}

void mrpt::obs::obs2Dscan_to_viz(
    const CObservation2DRangeScan::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::opengl::CSetOfObjects& out)
{
  out.clear();

  add_common_to_viz(*obs, p, out);

  auto pnts = mrpt::opengl::CPlanarLaserScan::Create();
  out.insert(pnts);

  pnts->setScan(*obs);
  pnts->setPointSize(p.pointSize);
  pnts->enableSurface(p.showSurfaceIn2Dscans);
  pnts->enablePoints(p.showPointsIn2Dscans);

  pnts->setSurfaceColor(
      mrpt::u8tof(p.surface2DscansColor.R), mrpt::u8tof(p.surface2DscansColor.G),
      mrpt::u8tof(p.surface2DscansColor.B), mrpt::u8tof(p.surface2DscansColor.A));
  pnts->setPointsColor(
      mrpt::u8tof(p.points2DscansColor.R), mrpt::u8tof(p.points2DscansColor.G),
      mrpt::u8tof(p.points2DscansColor.B), mrpt::u8tof(p.points2DscansColor.A));
}

bool mrpt::obs::obs_to_viz(
    const mrpt::obs::CObservation::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::opengl::CSetOfObjects& out)
{
  using namespace mrpt::obs;

  if (const auto o3D = std::dynamic_pointer_cast<CObservation3DRangeScan>(obs); o3D)
  {
    obs3Dscan_to_viz(o3D, p, out);
    return true;
  }
  else if (const auto oVel = std::dynamic_pointer_cast<CObservationVelodyneScan>(obs); oVel)
  {
    obsVelodyne_to_viz(oVel, p, out);
    return true;
  }
  else if (const auto oPC = std::dynamic_pointer_cast<CObservationPointCloud>(obs); oPC)
  {
    obsPointCloud_to_viz(oPC, p, out);
    return true;
  }
  else if (const auto oRS = std::dynamic_pointer_cast<CObservationRotatingScan>(obs); oRS)
  {
    obsRotatingScan_to_viz(oRS, p, out);
    return true;
  }
  else if (const auto o2D = std::dynamic_pointer_cast<CObservation2DRangeScan>(obs); o2D)
  {
    obs2Dscan_to_viz(o2D, p, out);
    return true;
  }

  // No conversion found:
  out.clear();
  return false;
}

bool mrpt::obs::obs_to_viz(
    const mrpt::obs::CSensoryFrame& sf,
    const VisualizationParameters& p,
    mrpt::opengl::CSetOfObjects& out)
{
  out.clear();

  // Make a copy, to avoid potentially duplicated axis:
  VisualizationParameters vp = p;

  for (const auto& obs : sf)
  {
    if (!obs) continue;
    auto glObj = mrpt::opengl::CSetOfObjects::Create();

    bool ok = obs_to_viz(obs, vp, *glObj);
    if (!ok) continue;

    out.insert(glObj);

    vp.showAxis = false;  // to avoid duplications
  }

  return !out.empty();
}
